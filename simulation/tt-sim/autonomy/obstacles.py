"""지도에 없는 장애물 찾기.

- 전방 ToF: 실제 TT의 ToF는 원뿔 안에서 가장 가까운 거리 하나만 준다. 같은 원뿔을 지도에 쏘아
  '지도대로라면 몇 m에서 막혀야 하는지'를 구해 두고, 측정값이 그보다 확실히 가까우면
  지도에 없는 물체로 본다. 방향은 모르므로 정면에 원뿔 폭만한 기둥을 세운다.
- YOLO: 상자 아래 변이 바닥에 닿는 점을 카메라 높이로 되짚어 물체 위치를 구한다
  (바닥에 서 있는 물체 가정). 원래 지도에 있는 물체거나 벽 너머로 나오면 버린다.
높이는 두 센서 모두 제대로 알 수 없으므로 찾은 장애물은 바닥부터 천장까지 막힌 기둥으로 둔다.
ROS와 무관한 순수 함수라 실기체 코드에서도 그대로 쓴다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .voxel_map import VoxelMap

# 바닥에 서 있어 피해야 하는 YOLO(COCO) 클래스. 상자·라바콘·카트는 COCO에 없어 ToF로만 찾는다.
AVOID_CLASSES = {"person", "chair", "couch", "potted plant", "bed", "dining table", "bench", "suitcase",
                 "backpack", "refrigerator", "bicycle", "dog", "cat", "toilet"}
# 클래스별 (깊이, 최대 폭). 없으면 깊이 = 상자 폭(최대 1 m), 최대 폭 1.5 m.
# 사람은 팔을 벌리면 상자가 넓어지지만 몸통은 그보다 작다.
CLASS_SIZE = {"person": (0.4, 0.7)}


# ---------------------------------------------------------------- ToF

@dataclass
class TofRays:
    """ToF 원뿔을 흉내 내는 광선 방향.

    줄마다 위아래로 v(위가 +)만큼 기울인 평면에 수평각 h(왼쪽이 +)로 광선을 펼친다.
    Gazebo에서 pitch를 준 한 줄짜리 라이다(models/tt_drone/model.sdf)와 같은 모양이다.
    """
    h_angles: list[float]
    v_angles: list[float]

    @classmethod
    def fan(cls, count: int, half: float, rows=(0.0,)) -> "TofRays":
        return cls(list(np.linspace(-half, half, count)) if count > 1 else [0.0], list(rows))

    def directions(self, yaw: float) -> list[np.ndarray]:
        c, s = math.cos(yaw), math.sin(yaw)
        out = []
        for v in self.v_angles:
            for h in self.h_angles:
                bx, by, bz = math.cos(h) * math.cos(v), math.sin(h), math.cos(h) * math.sin(v)
                out.append(np.array([c * bx - s * by, s * bx + c * by, bz]))
        return out


def expected_range(believed: VoxelMap, origin, directions, max_range: float, step: float = 0.02) -> float:
    """지도대로라면 원뿔 안에서 가장 가깝게 막히는 거리 (max_range 안에 없으면 inf)."""
    origin = np.asarray(origin, dtype=float)
    best = math.inf
    for d in directions:
        t = step
        while t <= min(best, max_range + 0.05):
            if believed.occupied_at(origin + d * t):
                best = t
                break
            t += step
    return best


def is_unexpected(measured: float, expected: float, max_range: float, margin: float = 0.2) -> bool:
    """측정값이 지도보다 margin 이상 가까우면 True (측정 범위 밖이면 False)."""
    if not math.isfinite(measured) or measured <= 0 or measured > max_range:
        return False
    return measured < min(expected, max_range + 1.0) - margin


def tof_box(origin, yaw: float, distance: float, half_angle: float, top: float, depth: float = 0.6) -> dict:
    """ToF가 닿은 거리에 기둥(바닥~top)을 세운다. 앞면이 닿은 점에 오고, 폭은 그 거리의 원뿔 폭이다.

    ToF는 앞면까지 거리만 주므로 깊이(뒤로 얼마나 큰지)는 모른다. 실내 물건(상자·사람·의자)을
    생각해 depth(기본 0.6 m)로 가정한다. 기울어진 직사각형을 감싸는 축 정렬 상자로 돌려준다.
    """
    width = max(0.3, 2 * distance * math.tan(half_angle))
    reach = distance + depth / 2
    c, s = math.cos(yaw), math.sin(yaw)
    return {"x": round(origin[0] + c * reach, 3), "y": round(origin[1] + s * reach, 3), "z": round(top / 2, 3),
            "sx": round(abs(c) * depth + abs(s) * width, 3), "sy": round(abs(s) * depth + abs(c) * width, 3),
            "sz": round(top, 3)}


# ---------------------------------------------------------------- YOLO

def ground_point(bbox, cam_pos, cam_yaw: float, width: int, height: int, hfov: float):
    """YOLO 상자 아래 변 가운데가 바닥에 닿는 점 (x, y)와 앞쪽 거리. 아래 변이 잘렸거나 수평선 위면 None.

    카메라는 수평(기울기 0)이라고 가정한다. 실기체는 움직일 때 앞으로 기울어 오차가 생긴다.
    """
    x1, y1, x2, y2 = bbox
    f = (width / 2) / math.tan(hfov / 2)
    cx, cy = width / 2, height / 2
    if y2 >= height - 3 or y2 <= cy + 2:
        return None
    dist = f * cam_pos[2] / (y2 - cy)
    left = dist * (cx - (x1 + x2) / 2) / f
    c, s = math.cos(cam_yaw), math.sin(cam_yaw)
    return cam_pos[0] + c * dist - s * left, cam_pos[1] + s * dist + c * left, dist


def ground_box(bbox, cam_pos, cam_yaw: float, width: int, height: int, hfov: float, top: float,
               max_distance: float = 4.0, name: str = "") -> dict | None:
    """바닥에 선 물체를 감싸는 기둥. 폭은 상자 폭, 깊이는 모르므로 폭과 같다고 본다(CLASS_SIZE에 있으면 그 값)."""
    hit = ground_point(bbox, cam_pos, cam_yaw, width, height, hfov)
    if hit is None or hit[2] > max_distance:
        return None
    gx, gy, dist = hit
    f = (width / 2) / math.tan(hfov / 2)
    fixed_depth, max_w = CLASS_SIZE.get(name, (None, 1.5))
    w = max(0.3, min(max_w, (bbox[2] - bbox[0]) * dist / f))
    depth = fixed_depth if fixed_depth is not None else min(w, 1.0)
    c, s = math.cos(cam_yaw), math.sin(cam_yaw)
    x, y = gx + c * depth / 2, gy + s * depth / 2
    sx = abs(c) * depth + abs(s) * w        # 돌아간 직사각형을 감싸는 축 정렬 상자
    sy = abs(s) * depth + abs(c) * w
    return {"x": round(x, 3), "y": round(y, 3), "z": round(top / 2, 3),
            "sx": round(sx, 3), "sy": round(sy, 3), "sz": round(top, 3)}


PERSON_HEAD = 1.7       # 가까운 사람 거리를 머리 높이로 어림할 때 쓰는 키 (m)
PERSON_SHOULDER = 0.45  # 머리까지 잘렸을 때 상자 폭으로 어림하는 어깨너비 (m)
PERSON_NEAR_MAX = 1.3   # 이보다 멀게 나오면 버린다. 서 있는 사람은 1.5 m 밖이면 발이 보이므로, 발이 잘렸는데
#                         멀게 나온 것은 가려졌거나 일부만 잡힌 상자다 (엉뚱한 자리에 장애물을 만들어 길을 막았다)


def near_person_box(bbox, cam_pos, cam_yaw: float, width: int, height: int, hfov: float, top: float):
    """발이 화면 아래로 잘린 가까운 사람의 기둥과 거리. ground_point를 못 쓰는 1.5 m 안쪽에서 쓴다.

    머리가 보이면 머리 높이(PERSON_HEAD)로, 머리도 잘렸으면 상자 폭을 어깨너비로 보고 거리를 어림하며,
    둘 다 되면 가까운 쪽을 쓴다(키 작은 사람은 머리로 재면 멀게 나온다). 발이 보이면 None (ground_box를 쓴다).
    """
    x1, y1, x2, y2 = bbox
    if y2 < height - 3:
        return None
    f = (width / 2) / math.tan(hfov / 2)
    cx, cy = width / 2, height / 2
    dists = []
    if y1 > 3 and cy - y1 > 1 and PERSON_HEAD > cam_pos[2]:
        dists.append(f * (PERSON_HEAD - cam_pos[2]) / (cy - y1))
    if x2 - x1 > 2 and x1 > 3 and x2 < width - 3:   # 옆으로도 잘리면 폭을 믿을 수 없다
        dists.append(f * PERSON_SHOULDER / (x2 - x1))
    if dists:
        dist = min(dists)
        if dist > PERSON_NEAR_MAX:
            return None
        dist = max(0.3, dist)
    elif y2 - y1 >= 0.9 * height:   # 머리도 옆도 잘리고 화면을 꽉 채움: 아주 가깝다
        dist = 0.5
    else:
        return None
    left = dist * (cx - (x1 + x2) / 2) / f
    c, s = math.cos(cam_yaw), math.sin(cam_yaw)
    depth, max_w = CLASS_SIZE["person"]
    w = max(0.3, min(max_w, (x2 - x1) * dist / f))
    gx, gy = cam_pos[0] + c * dist - s * left, cam_pos[1] + s * dist + c * left
    x, y = gx + c * depth / 2, gy + s * depth / 2
    box = {"x": round(x, 3), "y": round(y, 3), "z": round(top / 2, 3),
           "sx": round(abs(c) * depth + abs(s) * w, 3), "sy": round(abs(s) * depth + abs(c) * w, 3), "sz": round(top, 3)}
    return box, dist


def is_known(known: VoxelMap, box: dict, radius: float = 0.3, z: float = 0.3) -> bool:
    """상자 가운데 근처(radius)가 원래 지도에서 이미 막혀 있으면 아는 물체로 본다."""
    for dx in (-radius, 0.0, radius):
        for dy in (-radius, 0.0, radius):
            idx = known.index((box["x"] + dx, box["y"] + dy, z))
            if known.inside(idx) and known.occ[idx]:
                return True
    return False


def explained_by_map(known: VoxelMap, origin, yaw: float, distance: float, h_angles, v_half: float,
                     radius: float) -> bool:
    """ToF가 distance에서 무언가를 봤을 때, 원뿔이 닿는 자리 근처(radius)에 받은 지도의 벽·가구가 있으면 True.

    위치 추정이 조금 틀어진 채 아는 벽·문틀을 본 것으로 보고 새 장애물로 넣지 않는다.
    원뿔의 왼쪽·가운데·오른쪽, 위·가운데·아래 높이에서 radius 범위를 촘촘히 확인한다.
    """
    step = max(radius / 2, 0.05)
    offsets = np.arange(-radius, radius + 1e-9, step)
    heights = (origin[2], origin[2] + distance * math.sin(v_half), origin[2] - distance * math.sin(v_half))
    for a in h_angles:
        px = origin[0] + distance * math.cos(yaw + a)
        py = origin[1] + distance * math.sin(yaw + a)
        for z in heights:
            for dx in offsets:
                for dy in offsets:
                    idx = known.index((px + dx, py + dy, z))
                    if known.inside(idx) and known.occ[idx]:
                        return True
    return False


def near_known(known: VoxelMap, box: dict, margin: float, z: float) -> bool:
    """상자 바닥 면을 margin만큼 넓힌 범위 안(높이 z)에 받은 지도의 벽·가구가 있으면 True."""
    xs = np.arange(box["x"] - box["sx"] / 2 - margin, box["x"] + box["sx"] / 2 + margin + 1e-9, 0.1)
    ys = np.arange(box["y"] - box["sy"] / 2 - margin, box["y"] + box["sy"] / 2 + margin + 1e-9, 0.1)
    for x in xs:
        for y in ys:
            idx = known.index((x, y, z))
            if known.inside(idx) and known.occ[idx]:
                return True
    return False


def in_sight(known: VoxelMap, cam_pos, point) -> bool:
    """카메라에서 바닥 접점(조금 위)까지 원래 지도에서 가리는 것이 없으면 True. 벽 너머 추정을 거른다."""
    return known.segment_free(cam_pos, (point[0], point[1], 0.15))


# ---------------------------------------------------------------- 찾은 장애물 모음

@dataclass
class Found:
    box: dict
    sources: set[str]    # "tof", "yolo:chair" 처럼 이 장애물을 본 센서
    hits: int = 1


@dataclass
class ObstacleMemory:
    """찾은 장애물 목록. 상자들을 하나로 감싸 합치지 않고 따로 둔다.

    다른 방향에서 본 상자 둘을 감싸면 실제보다 훨씬 커져 좁은 길(문 앞, 창고)을 막는다.
    따로 두면 지도에는 실제로 본 부분의 합만 들어간다.
    """
    slack: float = 0.05
    items: list[Found] = field(default_factory=list)
    suspects: list[dict] = field(default_factory=list)

    def add(self, box: dict, source: str) -> bool:
        """새 상자면 넣고 True. 이미 있는 상자 안에 다 들어가면 그 상자의 기록만 늘리고 False."""
        for item in self.items:
            if _inside(box, item.box, self.slack):
                item.hits += 1
                item.sources.add(source)
                return False
        self.items.append(Found(box, {source}))
        return True

    def covering(self, x: float, y: float, margin: float = 0.1) -> Found | None:
        """(x, y)를 덮는 장애물. 흔들리는 YOLO 추정이 같은 물체를 여러 번 넣지 않게 할 때 쓴다."""
        for item in self.items:
            b = item.box
            if abs(x - b["x"]) <= b["sx"] / 2 + margin and abs(y - b["y"]) <= b["sy"] / 2 + margin:
                return item
        return None

    def drop_camera_only(self) -> int:
        """카메라로만 본 장애물을 지운다 (길이 완전히 막혔을 때 오탐을 의심해 다시 시도). 지운 개수.

        지운 자리는 suspects에 남겨, 같은 자리를 카메라로 다시 넣지 않게 한다(지웠다 넣었다 반복 방지).
        진짜 장애물이면 가까이 갔을 때 ToF가 찾는다.
        """
        keep = [i for i in self.items if any(not s.startswith("yolo") for s in i.sources)]
        self.suspects += [i.box for i in self.items if i not in keep]
        dropped = len(self.items) - len(keep)
        self.items = keep
        return dropped

    def drop_where(self, pred) -> int:
        """pred(장애물 기록 Found)가 참인 장애물을 지운다. 지운 개수."""
        keep = [i for i in self.items if not pred(i)]
        dropped = len(self.items) - len(keep)
        self.items = keep
        return dropped

    def suspected(self, x: float, y: float, margin: float = 0.3) -> bool:
        """카메라로만 봤다가 길을 막아 지운 자리 근처면 True."""
        return any(abs(x - b["x"]) <= b["sx"] / 2 + margin and abs(y - b["y"]) <= b["sy"] / 2 + margin
                   for b in self.suspects)

    def boxes(self) -> list[dict]:
        return [i.box for i in self.items]


def _inside(a: dict, b: dict, slack: float) -> bool:
    """상자 a의 바닥 면이 b(가장자리 slack 여유 포함) 안에 다 들어가면 True."""
    return all(abs(a[k] - b[k]) + a[s] / 2 <= b[s] / 2 + slack for k, s in (("x", "sx"), ("y", "sy")))


def inflate_box(box: dict, radius: float) -> dict:
    return {**box, "sx": box["sx"] + 2 * radius, "sy": box["sy"] + 2 * radius, "sz": box["sz"] + 2 * radius}


def with_boxes(base: VoxelMap, boxes: list[dict]) -> VoxelMap:
    """base 지도에 상자들을 더한 새 지도."""
    out = VoxelMap(base.origin.copy(), base.res, base.occ.copy())
    for b in boxes:
        out.add_box(b)
    return out


def path_clear(inflated: VoxelMap, points) -> bool:
    return all(inflated.segment_free(a, b) for a, b in zip(points, points[1:]))
