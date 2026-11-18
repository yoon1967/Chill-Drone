"""앞에 보이는 것이 기둥인지 벽인지: 깊이 장면의 가로 모양.

기둥(가만히 선 사람·기둥·의자 등받이)은 폭이 좁고 한쪽 옆이 비어 있어 옆으로 지나가면 되고, 벽은 넓어서 방향을 바꿔야 한다.
깊이 영상에서 비행 높이 띠의 열마다 가까운 거리를 내고(scene_ranges), 정면 장애물 거리와 비슷한 거리로 이어진 덩어리의
폭을 재서(front_shape) 둘을 나눈다. 2026-10-30까지는 전방 ToF 한 점만 보고 벽도 좁은 기둥으로 넣어, 경로가 그 옆으로
비집다 다시 막혀 재계획하기를 되풀이했다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

PILLAR_MAX_W = 1.0        # 이 폭 이하면 기둥 (사람 0.5 m, 의자 0.6 m, 이동식 디스플레이 0.9 m)
WALL_MIN_W = 1.5          # 이 폭 이상이면 벽
SAME_FRONT = 0.6          # 정면 거리 + 이만큼 안에 있는 열은 같은 앞면(한 덩어리)으로 본다 (m)
OPEN_BEYOND = 1.0         # 덩어리 옆 열이 정면 거리 + 이만큼보다 멀면 그쪽이 비어 있다 (m)
AHEAD = math.radians(8)   # '정면'으로 보는 각 (±)
PASS_MARGIN = math.radians(20)   # 기둥을 비켜 갈 때 덩어리 끝에서 이만큼 더 바깥으로 겨눈다
SEE_THROUGH_GAP = 0.5     # ToF 원뿔 안 깊이 열이 모두 ToF 거리 + 이만큼보다 멀면 카메라가 면을 꿰뚫어 본 것 (m)


@dataclass
class FrontShape:
    kind: str            # "pillar" 기둥 | "wall" 벽 | "unknown" 애매(폭 1.0–1.5 m 또는 양옆이 막힘) | "clear" 정면에 아무것도 안 보임
    width: float         # 덩어리 폭 (m). clear면 0
    a_left: float        # 덩어리 왼쪽 끝 각 (rad, 기수 기준 왼쪽 +)
    a_right: float       # 덩어리 오른쪽 끝 각 (rad, 음수)
    left_open: int       # 덩어리 왼쪽에서 비어 있는 열 수
    right_open: int
    edge_left: bool      # 덩어리가 화면 왼쪽 끝까지 닿음 (폭을 다 못 봄)
    edge_right: bool
    i0: int = 0          # 덩어리 열 범위 (scene_ranges 열 번호, i0 오른쪽 끝 … i1 왼쪽 끝). shape_points가 면을 따라 찍는 데 쓴다
    i1: int = -1

    @property
    def free_side(self) -> str | None:
        """비켜 갈 쪽: 더 넓게 비어 있는 쪽. 둘 다 막혔으면 None."""
        if self.left_open == 0 and self.right_open == 0:
            return None
        return "left" if self.left_open >= self.right_open else "right"

    def pass_angle(self) -> float | None:
        """기둥을 비켜 갈 방향 (rad, 기수 기준): 빈 쪽 덩어리 끝에서 PASS_MARGIN만큼 더 바깥."""
        side = self.free_side
        if side is None:
            return None
        return self.a_left + PASS_MARGIN if side == "left" else self.a_right - PASS_MARGIN

    def text(self) -> str:
        if self.kind == "clear":
            return "정면에 보이는 것 없음"
        if self.kind == "wall":
            return f"벽(폭 {self.width:.1f} m{'+' if self.edge_left or self.edge_right else ''})"
        side = {"left": "왼쪽", "right": "오른쪽", None: "양옆"}[self.free_side]
        if self.kind == "pillar":
            return f"기둥(폭 {self.width:.1f} m, {side} 빔)"
        return f"모양 애매(폭 {self.width:.1f} m, {side + ' 빔' if self.free_side else '양옆 막힘'})"


def scene_ranges(depth: np.ndarray, hfov: float, cam_z: float, height: float, band: float = 0.35,
                 bins: int = 27, stride: int = 3, max_range: float = 4.0) -> tuple[np.ndarray, np.ndarray]:
    """깊이 영상(광축 방향 거리 m, 배율을 맞춘 것) → (열 중심 각 [bins], 열마다 가까운 수평 거리 [bins]).

    비행 높이 ±band 안의 점만 쓰고(바닥·가구 윗면 제외), 열마다 10번째 백분위(가까운 쪽). 점이 셋 미만이면 max_range.
    각은 기수 기준 왼쪽 +. 열 수 27이면 약 3°씩이라 1.5 m 앞 사람(0.5 m)은 6열, 벽은 화면을 다 채운다.
    """
    h, w = depth.shape
    f = (w / 2) / math.tan(hfov / 2)
    vs, us = np.mgrid[stride // 2:h:stride, stride // 2:w:stride]
    d = depth[vs, us].astype(np.float64)
    left = -(us - w / 2) / f
    up = -(vs - h / 2) / f
    z = cam_z + d * up
    ok = np.isfinite(d) & (d > 0.05) & (np.abs(z - height) <= band) & (z > 0.3)
    ang = np.arctan(left[ok])
    rng = np.minimum(d[ok] * np.sqrt(1.0 + left[ok] ** 2), max_range)
    width = hfov / bins
    idx = np.clip(((ang + hfov / 2) / width).astype(int), 0, bins - 1)
    out = np.full(bins, max_range)
    for k in range(bins):
        v = rng[idx == k]
        if v.size >= 3:
            out[k] = float(np.percentile(v, 10))
    centers = -hfov / 2 + width * (np.arange(bins) + 0.5)
    return centers, out


def front_shape(centers, ranges, dist: float, ahead: float = AHEAD, same: float = SAME_FRONT,
                open_beyond: float = OPEN_BEYOND) -> FrontShape:
    """정면 dist m에 있는 것의 모양. centers·ranges는 scene_ranges 결과(기수 기준으로 돌려 놓은 것).

    정면(±ahead) 열 중 dist + same 안에 있는 열을 씨앗으로, 양옆으로 dist + same 안에 이어지는 열까지가 한 덩어리.
    폭은 그 각 범위를 dist에서 가로로 잰 길이. 덩어리 밖에서 dist + open_beyond보다 먼 열이 있으면 그쪽은 비어 있다.
    """
    centers, ranges = np.asarray(centers, dtype=float), np.asarray(ranges, dtype=float)
    n = len(centers)
    if n == 0:
        return FrontShape("clear", 0.0, 0.0, 0.0, 0, 0, False, False)
    blocked = ranges <= dist + same
    seed = np.nonzero(blocked & (np.abs(centers) <= ahead))[0]
    if seed.size == 0:
        return FrontShape("clear", 0.0, 0.0, 0.0, 0, 0, False, False)
    i0, i1 = int(seed.min()), int(seed.max())       # 열 0이 오른쪽 끝(각이 가장 작음)
    while i0 > 0 and blocked[i0 - 1]:
        i0 -= 1
    while i1 < n - 1 and blocked[i1 + 1]:
        i1 += 1
    half = (centers[1] - centers[0]) / 2 if n > 1 else abs(centers[0])
    a_right, a_left = float(centers[i0] - half), float(centers[i1] + half)
    width = dist * (math.tan(a_left) - math.tan(a_right))
    right_open = int((ranges[:i0] > dist + open_beyond).sum())
    left_open = int((ranges[i1 + 1:] > dist + open_beyond).sum())
    edge_right, edge_left = i0 == 0, i1 == n - 1
    if edge_left and edge_right:
        kind = "wall"
    elif width >= WALL_MIN_W:
        kind = "wall"
    elif width <= PILLAR_MAX_W and (left_open or right_open):
        kind = "pillar"
    else:
        kind = "unknown"
    return FrontShape(kind, round(width, 2), a_left, a_right, left_open, right_open, edge_left, edge_right, i0, i1)


FIT_INLIER = 0.2          # 맞춘 직선에서 이보다 벗어난 열은 다른 면(모퉁이 너머)으로 보고 찍지 않는다 (m)


def _fit_line(xy: np.ndarray, anchor: int, yaw: float) -> tuple[np.ndarray, np.ndarray]:
    """열 점 xy에 직선을 맞춘다(주성분). 직선에서 FIT_INLIER 넘게 벗어난 열은 정면 열(anchor)에서 이어진 범위 밖이면 버리고 한 번
    더 맞춘다 — 모퉁이 너머 다른 벽은 안 찍는다. (방향 u, 남긴 열 keep)."""
    keep = np.ones(len(xy), dtype=bool)
    u = np.array([-math.sin(yaw), math.cos(yaw)])           # 열 하나뿐이면 기수에 수직
    for _ in range(2):
        p = xy[keep]
        if len(p) < 2:
            break
        c = p.mean(axis=0)
        _, _, vt = np.linalg.svd(p - c, full_matrices=False)
        u = vt[0]
        ok = np.abs((xy - c) @ np.array([-u[1], u[0]])) <= FIT_INLIER
        ok[anchor] = True
        lo = hi = anchor
        while lo > 0 and ok[lo - 1]:
            lo -= 1
        while hi < len(xy) - 1 and ok[hi + 1]:
            hi += 1
        new = np.zeros(len(xy), dtype=bool)
        new[lo:hi + 1] = True
        if np.array_equal(new, keep):
            break
        keep = new
    return u, keep


def shape_points(origin, yaw: float, centers, ranges, shape: FrontShape, dist: float, z0: float, z1: float, res: float,
                 extra: float = 0.0, cone: float = 0.0, scale_limits: tuple[float, float] = (0.7, 1.4)) -> np.ndarray:
    """덩어리를 지도에 찍을 점들: 열마다 잰 점에 **직선을 맞춰** ToF가 닿은 자리를 지나게 놓는다 (보이는 면의 방향을 따르되 열
    하나하나의 깊이 오차는 평균낸다). centers·ranges는 지금 기수로 돌려 놓은 장면(front_shape에 준 것), dist는 정면 ToF 거리,
    cone은 ToF 원뿔 반각(rad).
    1) 정면(±AHEAD) 열의 거리가 dist가 되는 배율(scale_limits 안)을 모든 열에 곱해 직선을 맞춘다(_fit_line). 2) ToF는 원뿔 안에서
    가장 가까운 거리를 주므로 면을 비스듬히(θ) 볼 때 기수 방향의 실제 거리는 dist·cos(max(θ−cone, 0))/cos θ다 — 이 거리로 배율을
    다시 잡고 다시 맞춘다. 3) 직선을 그 자리를 지나게 옮기고 남은 열이 걸치는 구간을 찍는다. 화면 끝에 잘린 끝(edge)은 extra만큼 더.
    높이 z0–z1을 res 간격.

    2026-11-03 (Gazebo corridor_L, 기록 재생으로 확인): 기수에 수직인 직선(front_points)은 1.4 m 복도에서 벽을 50° 비스듬히 볼 때
    끝이 복도 안으로 0.5 m 넘게 들어와 복도가 0.5 m로 막혔고(빈 곳에 찍은 사건 674개 중 132개), 열마다 잰 거리대로 꺾은선을 찍으면
    단안 깊이가 열마다 16%(사분위 8–36%) 짧아 더 나빴다(405개). 직선 맞춤만으로는 202개 — 남은 것은 ToF 원뿔(±12.5°)이 비스듬한
    벽의 가까운 쪽을 재서 선이 벽에서 0.15–0.3 m 안쪽에 놓인 것이라 2)로 바로잡는다.
    """
    centers, ranges = np.asarray(centers, dtype=float), np.asarray(ranges, dtype=float)
    cols = np.arange(shape.i0, shape.i1 + 1)
    near = np.abs(centers[cols]) <= AHEAD
    ref = float(np.median(ranges[cols[near] if near.any() else cols]))
    a = yaw + centers[cols]
    anchor = int(np.argmin(np.abs(centers[cols])))        # 정면에 가장 가까운 열
    d = np.array([math.cos(yaw), math.sin(yaw)])

    def points(reach: float) -> np.ndarray:
        scale = min(max(reach / ref, scale_limits[0]), scale_limits[1]) if ref > 0.05 else 1.0
        r = ranges[cols] * scale
        return np.stack([origin[0] + r * np.cos(a), origin[1] + r * np.sin(a)], axis=1)

    xy = points(dist)
    u, keep = _fit_line(xy, anchor, yaw)
    reach = dist
    if cone > 0 and keep.sum() >= 2:
        cos_t = min(max(abs(float(d @ np.array([-u[1], u[0]]))), 0.3), 1.0)   # 면을 보는 각 θ (72°까지만)
        theta = math.acos(cos_t)
        reach = dist * math.cos(max(theta - cone, 0.0)) / cos_t
        xy = points(reach)
        u, keep = _fit_line(xy, anchor, yaw)
    hit = np.array([origin[0] + reach * d[0], origin[1] + reach * d[1]])
    s = (xy[keep] - hit) @ u                               # 직선은 ToF가 닿은 자리를 지난다
    s0, s1 = float(s.min()), float(s.max())
    first, last = int(np.argmax(keep)), int(len(keep) - 1 - np.argmax(keep[::-1]))
    sign = 1.0 if (xy[last] - hit) @ u >= (xy[first] - hit) @ u else -1.0   # u 방향이 열 번호 커지는 쪽(왼쪽)인가
    if shape.edge_right and first == 0:
        s0 -= extra if sign > 0 else 0.0
        s1 += extra if sign < 0 else 0.0
    if shape.edge_left and last == len(cols) - 1:
        s1 += extra if sign > 0 else 0.0
        s0 -= extra if sign < 0 else 0.0
    ss = np.arange(s0, s1 + 1e-9, res / 2)
    line = hit[None, :] + ss[:, None] * u[None, :]
    zs = np.arange(z0, z1 + 1e-9, res)
    ii, zz = np.meshgrid(np.arange(len(line)), zs, indexing="ij")
    return np.concatenate([line[ii.ravel()], zz.ravel()[:, None]], axis=1)


def rotate_scene(centers, scene_yaw: float, now_yaw: float) -> np.ndarray:
    """장면을 찍었을 때 기수(scene_yaw)로 잰 각을 지금 기수(now_yaw) 기준으로."""
    d = scene_yaw - now_yaw
    d = math.atan2(math.sin(d), math.cos(d))
    return np.asarray(centers, dtype=float) + d


def see_through(centers, ranges, front: float, cone: float, gap: float = SEE_THROUGH_GAP) -> float | None:
    """정면 ToF가 front m에서 면을 재는데 같은 방향(±cone) 깊이 열이 모두 front + gap보다 멀면, 카메라가 그 면(흰 벽·유리)을
    꿰뚫어 본 것이다. 그때 깊이 열의 가장 가까운 거리, 아니면 None. centers는 ToF를 잰 기수 기준으로 돌려 놓은 것.

    열마다 가까운 쪽(10번째 백분위) 거리라, 의자 등받이처럼 원뿔 일부에만 걸린 것도 한 열이라도 보이면 꿰뚫은 것이 아니다.
    11/04 실기체: 흰 벽 0.40–0.90 m를 ToF가 잴 때 깊이는 1.06–1.51 m였다 (창문 유리는 ToF도 통과해 잡지 못했다)."""
    if not math.isfinite(front):
        return None
    centers, ranges = np.asarray(centers, dtype=float), np.asarray(ranges, dtype=float)
    r = ranges[np.abs(centers) <= cone]
    if r.size < 2:
        return None
    near = float(np.min(r))
    return near if near > front + gap else None


def front_points(origin, yaw: float, dist: float, a_right: float, a_left: float, z0: float, z1: float, res: float,
                 extra: float = 0.0) -> np.ndarray:
    """덩어리를 지도에 찍을 점들: origin에서 기수 yaw 쪽 dist 앞, 각 범위(a_right–a_left)를 가로 폭으로 바꿔 res/2 간격으로,
    높이 z0–z1을 res 간격으로. extra는 양 끝을 더 늘이는 길이(m, 벽 끝이 화면 밖일 때)."""
    lat0, lat1 = dist * math.tan(a_right) - extra, dist * math.tan(a_left) + extra
    lats = np.arange(lat0, lat1 + 1e-9, res / 2)
    zs = np.arange(z0, z1 + 1e-9, res)
    c, s = math.cos(yaw), math.sin(yaw)
    ll, zz = np.meshgrid(lats, zs, indexing="ij")
    x = origin[0] + c * dist - s * ll.ravel()
    y = origin[1] + s * dist + c * ll.ravel()
    return np.stack([x, y, zz.ravel()], axis=1)


def segment_points(p0, p1, z0: float, z1: float, res: float, extra: float = 0.3) -> np.ndarray:
    """두 점을 잇는 선(양 끝 extra만큼 더)을 높이 z0–z1로 채운 점들. ToF 점이 옆으로 이어질 때 벽으로 잇는 데 쓴다."""
    p0, p1 = np.asarray(p0, dtype=float)[:2], np.asarray(p1, dtype=float)[:2]
    d = p1 - p0
    n = float(np.hypot(*d))
    u = d / n if n > 1e-6 else np.array([1.0, 0.0])
    ts = np.arange(-extra, n + extra + 1e-9, res / 2)
    zs = np.arange(z0, z1 + 1e-9, res)
    tt, zz = np.meshgrid(ts, zs, indexing="ij")
    xy = p0[None, :] + tt.ravel()[:, None] * u[None, :]
    return np.concatenate([xy, zz.ravel()[:, None]], axis=1)
