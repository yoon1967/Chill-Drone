"""카메라 깊이 영상으로 3D 복셀 지도를 만든다 (4단계: 받은 지도 없이 탐색). ROS와 무관.

칸마다 막힘 점수(log-odds)를 둔다. 깊이 영상에서 몇 화소마다 카메라에서 그 점까지 광선을 그어, 지나간 칸은
'비어 있음' 쪽으로, 끝 칸은 '막힘' 쪽으로 점수를 옮긴다. 한 번도 광선이 지나지 않은 칸은 '모름'이다.
- 바닥(floor m 아래)에 닿은 점은 막힘으로 치지 않는다 (바닥은 장애물이 아니다).
- 단안 깊이는 멀수록 틀리므로 max_range(4 m)까지만 쓴다. 그보다 먼 화소는 max_range까지만 비운다.
- 전방 ToF 한 점(integrate_ray)도 같은 방식으로 넣는다. 가까운 곳은 단안 깊이보다 ToF가 정확하다.
- 드론이 지나간 자리는 비어 있다 (mark_free_sphere).

계획용 지도(planning_map): 막힌 칸을 드론 크기만큼 부풀리고, 모르는 칸도 막힌 것으로 본다. 그래서 드론은
본 적 있는 빈 곳으로만 다닌다. 다음에 볼 자리(frontier_viewpoint)는 '아는 빈 칸'과 '모르는 칸'의 경계다.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

import numpy as np
from scipy import ndimage

from .voxel_map import VoxelMap

PASSAGE_MOUTH = 0.6   # 통로 입구: 본 빈 곳에서 (트인 칸을 따라) 이 안의 모르는 칸만 경계로 다시 넣는다 (m)
# 창턱(window_sills, 11/04): 창은 같은 벽면에 '아래 벽(창턱)'과 '위 벽'이 있고 그 사이(유리)는 깊이에 안 잡힌다
WALL_TALL = (1.9, 2.3)   # 창 위 벽 높이 (m). 가구는 이보다 낮고, 천장(2.4 m 위)은 들지 않게
WINDOW_SILL = (0.5, 1.2)  # 창턱 높이 (m). 0.3–0.5 m는 먼 바닥 오차가 많이 찍혀 뺐다 (10:17 기록: 창턱 칸 426 → 88)
WINDOW_LINE = 0.5     # 위 벽은 이 길이(m, 칸 수) 넘게 이어져야 한다 — 흩어진 높은 칸(매달린 물건·깊이 오차)은 창이 아니다
WINDOW_NEAR = 0.2     # 창턱과 위 벽이 이 안에 있어야 같은 벽면이다 (m, 깊이 오차 몫)


@dataclass
class CameraMapParams:
    res: float = 0.1
    max_range: float = 4.0        # 단안 깊이를 믿는 거리 (m). 2–4 m 평균 오차 약 8% (tools/eval_depth.py)
    min_range: float = 0.3        # 이보다 가까운 단안 깊이는 버린다
    hit_min_range: float = 1.0    # 이보다 가까운 단안 깊이는 그 거리까지 비우기만 하고 막힘은 찍지 않는다.
    #                               가까운 곳일수록 짧게 추정해(추정 0.3–1.2 m에서 실제는 1.2–1.3배) 벽 앞 빈 곳에
    #                               헛 장애물이 생겼다. 가까운 정면은 ToF가 맡는다
    stride: int = 6               # 깊이 영상 몇 화소마다 광선을 쏠지
    hit: float = 0.85             # 끝 칸에 더하는 log-odds (hit_near_range 안). 한 장면에 바로 막힘
    hit_near_range: float = 2.0   # 이보다 먼 끝점은 hit_far만 더한다: 단안 깊이는 멀수록 틀려 한 장면만으로 가짜 벽이
    #                               섰다 (10/30 실기체: 막힌 칸의 25%가 가구·벽에서 0.3 m 넘게 떨어짐). 두 장면이 맞아야 막힘
    hit_far: float = 0.4
    hit_max_range: float = 3.0    # 이보다 먼 끝점은 막힘을 찍지 않고 그 거리까지 비우기만 한다
    miss: float = -0.25           # 지나간 칸에 더하는 log-odds. 탁자 윗면처럼 비스듬히 보이는 면은 그 위를 스치는
    #                               광선이 계속 지워서, -0.4일 때는 탁자가 막힘으로 남지 않아 그 위로 날았다
    hard_miss: float = 0.25       # ToF로 확인한 '단단한' 칸(mark_hard)은 카메라 광선이 지나가도 miss의 이 배율만 깎는다:
    #                               단안 깊이가 흰 벽·유리를 꿰뚫어 보면 ToF가 찍은 벽이 몇 장면 만에 지워졌다
    clamp: float = 3.5
    occupied: float = 0.5         # 이보다 크면 막힘 (2 m 안 한 장면 0.85, 밖은 두 장면 0.8). 위치 맞추기·탐색 넓이·ToF 기대 거리에 씀
    suspect: float = 0.3          # 계획(planning_map)은 이 점수부터 피한다: 먼 한 장면(0.4)도 피하고, 광선이 한 번 지나가면(−0.25)
    #                               풀린다. 10/30 Gazebo에서 한 번 멀리서 본 의자를 두 장면 규칙 때문에 지도에 못 넣고 0.02 m로 지나쳤다
    floor: float = 0.3            # 이 높이 아래 끝점은 바닥으로 본다. 먼 바닥은 깊이 오차(3 m에서 약 0.25 m)로
    #                               0.15 m 위로 찍혀 빈 바닥에 점 장애물이 생겼다
    floor_frac: float = 0.45      # 깊이 영상(integrate_depth)은 카메라 높이의 이 비율 아래 끝점도 바닥으로 본다 (floor_cap까지). 단안 깊이는
    #                               무늬 없는 바닥을 30–40% 가깝게 읽어, 바닥이 카메라 높이의 0.3–0.4배 위에 찍힌다. 1.0 m로 날 때는 0.3 m
    #                               아래라 floor에 걸리지만, 가구를 넘느라 1.4 m로 오르면 0.45–0.55 m에 '가짜 바닥'이 깔리고 기둥 막기가
    #                               그 위 1.2 m까지 막아 1.0 m로는 못 내려온다 — 2026-11-11 09:32 실기체: 복도 폭 전체에 가짜 바닥(0.45·0.55 m
    #                               칸 480개), 1.35–1.45 m의 좁고 구불구불한 틈으로만 길이 나 지그재그로 날고 끝내 출발점 길도 막혔다
    floor_cap: float = 0.6        # 그 바닥 높이의 상한 (m): 높이 올라가도 책상(0.7 m~)은 그대로 본다
    column: tuple = (0.3, 1.3)    # 계획할 때 이 높이 사이에 막힌 칸이 있으면 그 자리를 기둥으로 막는다 (가구 위로 낮게 날지 않기)
    # 가구 위로 넘기 (11/03, 수직 공간 쓰기): 기둥을 위아래 전체 대신 '본 꼭대기 + over_clear'까지만(최소 over_min) 막는다.
    # 그 위는 카메라가 빈 것으로 본 칸이어야 지난다(모르는 칸은 막힘). 꼭대기는 over_scan 아래에서 가장 높은 막힌 칸(천장·등은 뺌).
    # 카메라가 얇은 모니터·등받이를 놓치므로 0.5 m를 더 두고, 1.2 m 아래로는 아무 가구도 넘지 않는다. 0이면 예전처럼 전체를 막는다
    over_clear: float = 0.6       # Gazebo 실습실(11/03): 1.4 m 상자를 카메라가 0.85 m쯤으로만 봐 1.55 m로 넘다가 얹혔다 → 하방 ToF 감시가 받친다
    over_min: float = 1.2
    over_scan: float = 2.0
    close_windows: bool = True    # 창턱(window_sills) 기둥은 위아래 전체를 막는다: 닫힌 창은 넘지 않는다 (11/04)
    unknown_margin: float = 0.1   # 계획할 때 모르는 칸을 이만큼 부풀린다
    edge_window: int = 3          # 깊이 경계 거르기: 이 크기 창 안의 최대/최소 거리 비가
    edge_ratio: float = 1.3       # 이보다 크면 경계 화소로 보고 가까운 쪽 거리를 쓴다.
    #                               단안 깊이는 문틀(가까움)과 그 너머(멂) 사이 화소에 중간 거리를 낸다. 저장한 150장으로
    #                               만든 지도에서 헛 장애물 칸 17–26% 감소, 진짜 장애물 칸 7–8% 손실 (창 5·1.15배는 손실 25%)
    edge_hit: bool = True         # 경계 화소 중 가까운 쪽 화소(제 값이 창의 최솟값, 곧 문틀 자체)는 막힘을 찍는다. 안 찍으면
    #                               벽 끝(문틀)이 늘 경계라 지도에서 빠지고, 다른 장면이 그 자리를 비워 문틀에 2.5 cm 닿았다
    #                               (seed 1, 위치 오차 0.01 m일 때). 가운데 화소는 창의 최솟값도 중간 거리라 찍지 않는다


class CameraMap:
    def __init__(self, bounds: dict, params: CameraMapParams | None = None):
        self.p = params or CameraMapParams()
        r = self.p.res
        self.origin = np.array([bounds["xmin"], bounds["ymin"], 0.0])
        top = np.array([bounds["xmax"], bounds["ymax"], bounds["zmax"]])
        self.shape = tuple(int(math.ceil(v)) for v in (top - self.origin) / r)
        self.logodds = np.zeros(self.shape, dtype=np.float32)
        self.seen = np.zeros(self.shape, dtype=bool)
        self.hard = np.zeros(self.shape, dtype=bool)   # ToF로 확인한 벽·기둥 칸 (mark_hard)
        self.flown = np.zeros(self.shape, dtype=bool)   # 드론이 실제로 지나간 칸 (언제든 다시 지날 수 있다)

    # ------------------------------------------------------------ 넣기

    def integrate_depth(self, depth: np.ndarray, cam, yaw: float, hfov: float, scale: float = 1.0) -> int:
        """depth: (H, W) 광축 방향 거리(m). cam: 카메라 위치 (x, y, z). 넣은 광선 수를 돌려준다.
        scale: depth에 이미 곱한 배율(정면 ToF로 맞춘 것) — 바닥 문턱을 그에 맞춘다(1보다 작을 때만, 아래).

        값의 뜻은 ROS 거리 규격(REP 117)을 따른다: +inf는 '범위 안에 아무것도 없음'(max_range까지 비움), NaN은 잴 수 없었던
        화소, −inf·0 이하는 너무 가까움 — 둘 다 넣지 않는다. 전에는 NaN도 '멂'으로 보아, NaN뿐인 영상 한 장이 앞 4 m를 통째로
        비웠다(11/04 외부 리뷰: 복셀 13,698개). 화소 절반 넘게 NaN이면 깨진 영상으로 보고 아무것도 넣지 않는다."""
        h, w = depth.shape
        s = self.p.stride
        vs, us = np.mgrid[s // 2:h:s, s // 2:w:s]
        d = depth[vs, us].astype(np.float64)
        invalid = ~(np.isfinite(d) | np.isposinf(d))      # NaN, −inf
        if invalid.mean() > 0.5:
            return 0
        edge = np.zeros(d.shape, dtype=bool)
        near_side = np.ones(d.shape, dtype=bool)
        if self.p.edge_ratio > 0:
            clean = np.where(np.isfinite(depth) & (depth > 0), depth, 1e3)
            near = ndimage.minimum_filter(clean, size=self.p.edge_window, mode="nearest")[vs, us]
            far_ = ndimage.maximum_filter(clean, size=self.p.edge_window, mode="nearest")[vs, us]
            edge = far_ > self.p.edge_ratio * near
            near_side = d <= 1.05 * near                    # 경계의 가까운 쪽 화소 (문틀 자체)
            d = np.where(edge, near, d)                     # 경계 화소는 가까운 쪽 거리까지만 비운다 (중간 거리가 헛 장애물)
        f = (w / 2) / math.tan(hfov / 2)
        left = -(us - w / 2) / f
        up = -(vs - h / 2) / f
        ok = np.isfinite(d) & (d >= self.p.min_range)
        far = (np.isposinf(d) | (np.isfinite(d) & (d > 50))) & ~invalid
        d = np.where(far, 1e3, d)
        ok = (ok | far) & ~invalid                          # 경계 거르기가 NaN 화소에 이웃 거리를 넣었어도 버린다
        c, sn = math.cos(yaw), math.sin(yaw)
        dirs = np.stack([c - sn * left, sn + c * left, up], axis=-1)[ok]          # 길이는 1 이상 (광축 1)
        lengths = d[ok] * np.linalg.norm(dirs, axis=-1)
        unit = dirs / np.linalg.norm(dirs, axis=-1, keepdims=True)
        can_hit = (d[ok] >= self.p.hit_min_range) & (~edge[ok] | (self.p.edge_hit & near_side[ok]))
        frac = self.p.floor_frac
        if frac > 0 and 0 < scale < 1.0:
            # 깊이에 1보다 작은 배율을 곱했으면 이미 가깝게 읽힌 바닥이 그만큼 더 가깝게(높게) 들어온다: 배율을 곱하기 전 깊이로
            # floor_frac 아래였을 점까지 바닥으로 본다. 배율 0.85면 카메라 높이의 0.53배. 실기체 11/06 14:35 가방(배율 0.85–0.9)을 다시
            # 쌓으면 0.45–0.65 m에 남던 가짜 칸이 971 → 637개 (배율 1.0이면 573개). 1 이상이면 그대로 둔다
            frac = 1.0 - (1.0 - frac) * scale
        floor = max(self.p.floor, min(self.p.floor_cap, frac * float(cam[2])))   # 높이 날수록 바닥을 높게 잘못 읽는다
        self._integrate_rays(np.asarray(cam, dtype=float), unit, lengths, can_hit=can_hit, floor=floor)
        return int(ok.sum())

    def integrate_ray(self, cam, yaw: float, distance: float, max_range: float) -> None:
        """정면 한 줄(전방 ToF). distance가 max_range보다 멀거나 inf면 max_range까지만 비운다."""
        unit = np.array([[math.cos(yaw), math.sin(yaw), 0.0]])
        length = np.array([distance if math.isfinite(distance) else 1e3])
        self._integrate_rays(np.asarray(cam, dtype=float), unit, length, max_range=max_range)

    def mark_free_sphere(self, center, radius: float = 0.15) -> None:
        """드론이 지나간 자리는 비어 있다."""
        idx = self._index(np.asarray(center, dtype=float)[None, :])[0]
        k = int(math.ceil(radius / self.p.res))
        sl = tuple(slice(max(i - k, 0), min(i + k + 1, n)) for i, n in zip(idx, self.shape))
        self.logodds[sl] = np.minimum(self.logodds[sl], -self.p.clamp / 2)
        self.seen[sl] = True
        self.flown[sl] = True

    def unfly(self, point, radius: float, z0: float, z1: float) -> int:
        """point(x, y) 수평 radius 안, 높이 z0–z1의 칸을 '지나온 칸'에서 뺀다. 뺀 칸 수를 돌려준다.

        정면 ToF가 면을 잰 자리에 쓴다. 지나온 칸은 계획 지도와 찾은 장애물에서 늘 열어 두는데(들어온 길 지키기), 위치가 속도
        적분이라 오래 날면 예전에 지나온 칸이 지금의 기둥·문짝 자리와 겹친다. 그러면 ToF가 막힘을 찍고 장애물을 넣어도 그 칸이 열려
        같은 곳으로 다시 계획했다 (11/05 13:04 실기체: 나무 문짝에 15초 동안 3번, 흰 칸막이에 30초 동안 6번 정면 0.09–0.37 m)."""
        r = self.p.res
        x, y = point[0], point[1]
        i0, i1 = (int(math.floor((v - self.origin[0]) / r)) for v in (x - radius, x + radius))
        j0, j1 = (int(math.floor((v - self.origin[1]) / r)) for v in (y - radius, y + radius))
        k0, k1 = self.slice_index(z0), self.slice_index(z1)
        i0, j0 = max(i0, 0), max(j0, 0)
        i1, j1 = min(i1, self.shape[0] - 1), min(j1, self.shape[1] - 1)
        if i0 > i1 or j0 > j1:
            return 0
        ii, jj = np.mgrid[i0:i1 + 1, j0:j1 + 1]
        near = (self.origin[0] + (ii + 0.5) * r - x) ** 2 + (self.origin[1] + (jj + 0.5) * r - y) ** 2 <= radius ** 2
        block = self.flown[i0:i1 + 1, j0:j1 + 1, k0:k1 + 1]
        n = int((block & near[:, :, None]).sum())
        block &= ~near[:, :, None]
        return n

    def mark_hard(self, points, weight: float = 1.5) -> int:
        """ToF로 확인한 점들(x, y, z)을 막힘으로 찍고 '단단한' 칸으로 둔다: 카메라가 벽 너머를 본다고 해도 천천히만 지워진다.

        2026-10-30: 카메라 지도는 1 m 안 단안 깊이로 막힘을 찍지 않아(hit_min_range) 벽에 다가가면 ToF 한 점만 남았다.
        기둥·벽을 나눠 그 폭대로 찍는 데 쓴다(nodes/mission_node.py note_tof_shape). 찍은 칸 수를 돌려준다.
        """
        pts = np.asarray(points, dtype=float).reshape(-1, 3)
        if not len(pts):
            return 0
        idx = self._index(pts)
        idx = idx[np.all((idx >= 0) & (idx < np.array(self.shape)), axis=1)]
        if not len(idx):
            return 0
        cells = np.unique(np.ravel_multi_index((idx[:, 0], idx[:, 1], idx[:, 2]), self.shape))
        flat = self.logodds.reshape(-1)
        flat[cells] = np.minimum(np.maximum(flat[cells] + weight, self.p.occupied + 0.3), self.p.clamp)
        self.seen.reshape(-1)[cells] = True
        self.hard.reshape(-1)[cells] = True
        return int(len(cells))

    def forget_behind(self, cam, yaw: float, d0: float, d1: float, half: float, z0: float, z1: float) -> int:
        """cam에서 기수 yaw ± half 부채꼴의 d0–d1 m, 높이 z0–z1에 있는 '본 빈 칸'을 모르는 칸으로 되돌린다. 지운 칸 수.

        정면 ToF가 d0 앞에서 면을 재는데 깊이 영상은 그보다 훨씬 멀리 본다면, 단안 깊이가 흰 벽·유리를 꿰뚫어 그 너머를 빈 곳으로
        칠한 것이다 (11/04 실기체: 벽 너머 약 8 m²가 '본 빈 곳'이 되어 거기 생긴 경계를 보러 갔다). 막힌 칸·ToF로 찍은 칸·
        드론이 지나간 칸은 그대로 둔다."""
        r = self.p.res
        cx, cy = float(cam[0]), float(cam[1])
        lo = self._index(np.array([[cx - d1, cy - d1, z0]]))[0]
        hi = self._index(np.array([[cx + d1, cy + d1, z1]]))[0] + 1
        lo = np.clip(lo, 0, np.array(self.shape))
        hi = np.clip(hi, 0, np.array(self.shape))
        if np.any(hi <= lo):
            return 0
        ii, jj = np.mgrid[lo[0]:hi[0], lo[1]:hi[1]]
        dx = self.origin[0] + (ii + 0.5) * r - cx
        dy = self.origin[1] + (jj + 0.5) * r - cy
        dist = np.hypot(dx, dy)
        ang = np.abs((np.arctan2(dy, dx) - yaw + math.pi) % (2 * math.pi) - math.pi)
        wedge = (dist >= d0) & (dist <= d1) & (ang <= half)
        sl = (slice(lo[0], hi[0]), slice(lo[1], hi[1]), slice(lo[2], hi[2]))
        lo3, seen3 = self.logodds[sl], self.seen[sl]
        gone = wedge[:, :, None] & seen3 & (lo3 <= self.p.suspect) & ~self.hard[sl] & ~self.flown[sl]
        lo3[gone] = 0.0
        seen3[gone] = False
        return int(gone.sum())

    def _integrate_rays(self, cam: np.ndarray, unit: np.ndarray, lengths: np.ndarray,
                        max_range: float | None = None, can_hit: np.ndarray | None = None, floor: float | None = None) -> None:
        r = self.p.res
        floor = self.p.floor if floor is None else floor
        max_range = self.p.max_range if max_range is None else max_range
        reach = np.minimum(lengths, max_range)
        hit = lengths <= min(max_range, self.p.hit_max_range)
        if can_hit is not None:
            hit &= can_hit
        weight = np.where(lengths <= self.p.hit_near_range, self.p.hit, self.p.hit_far)   # 먼 끝점은 약하게
        # 지나간 칸: 끝 칸 바로 앞까지 반 칸 간격으로
        steps = np.arange(0.05, max_range, r / 2)
        pts = cam + unit[:, None, :] * steps[None, :, None]                       # (N, S, 3)
        along = (steps[None, :] < (reach[:, None] - r / 2))
        free_idx = self._flat(pts[along])
        end = cam + unit * lengths[:, None]
        keep = hit & (end[:, 2] >= floor)                                          # 바닥에 닿은 점은 막힘이 아니다
        end, weight = end[keep], weight[keep]
        idx = self._index(end)
        inside = np.all((idx >= 0) & (idx < np.array(self.shape)), axis=1)        # 지도 밖 끝점은 _flat이 버리므로 무게도 같이
        end, weight = end[inside], weight[inside]
        hit_idx = self._flat(end)
        free_idx = np.setdiff1d(np.unique(free_idx), hit_idx)                      # 같은 장면에서 맞은 칸은 비우지 않는다
        flat = self.logodds.reshape(-1)
        hard = self.hard.reshape(-1)
        flat[free_idx] += np.where(hard[free_idx], self.p.miss * self.p.hard_miss, self.p.miss)
        cells, inv = np.unique(hit_idx, return_inverse=True)
        add = np.zeros(len(cells))
        np.maximum.at(add, inv, weight)                                            # 칸마다 한 장면에 한 번, 가까운 광선 기준
        flat[cells] += add
        np.clip(flat, -self.p.clamp, self.p.clamp, out=flat)
        hard[free_idx] &= flat[free_idx] > self.p.suspect      # 그래도 끝내 지워지면 단단함도 푼다
        seen = self.seen.reshape(-1)
        seen[free_idx] = True
        seen[hit_idx] = True

    def _index(self, pts: np.ndarray) -> np.ndarray:
        return np.floor((pts - self.origin) / self.p.res).astype(int)

    def _flat(self, pts: np.ndarray) -> np.ndarray:
        idx = self._index(pts.reshape(-1, 3))
        ok = np.all((idx >= 0) & (idx < np.array(self.shape)), axis=1)
        idx = idx[ok]
        return np.ravel_multi_index((idx[:, 0], idx[:, 1], idx[:, 2]), self.shape)

    # ------------------------------------------------------------ 읽기

    def occupied(self) -> np.ndarray:
        return self.logodds > self.p.occupied

    def seen_band(self) -> np.ndarray:
        """위아래 한 칸까지 본 것으로 친다. 광선은 위아래로 간격을 두고 지나가 칸 한 줄씩 비기 쉽다."""
        s = self.seen.copy()
        s[:, :, 1:] |= self.seen[:, :, :-1]
        s[:, :, :-1] |= self.seen[:, :, 1:]
        return s

    def known_free(self) -> np.ndarray:
        return self.seen_band() & ~self.occupied()

    def unknown(self) -> np.ndarray:
        return ~self.seen_band()

    def top_below(self, x: float, y: float, below_z: float, radius: float = 0.1, min_cells: int = 2,
                  floor_z: float = 0.3) -> float | None:
        """(x, y) 둘레 radius 안 기둥에서 높이 below_z 아래에 지도가 아는 막힌 칸(막힘 또는 ToF로 확인한 칸)의 가장 높은 꼭대기 (m).
        없으면 None. 비행 노드가 기체 밑에 지도의 가구가 있는지 위치 추정에 알린다: 하방 ToF는 바로 밑 물건까지 거리라 가구 위에서는
        높이가 아니고, 바닥 위 높이는 ToF + 꼭대기다 (11/09 17:07 실기체: 조형물 위에서 높이 추정 0.67 m → 천장에 눌림).
        floor_z 아래 칸은 바닥(계획 지도도 0.3 m 아래는 바닥으로 본다)이라 세지 않고, 그 위로 막힌 칸이 min_cells 넘게 쌓여야
        가구다: 단안 깊이가 카펫을 30%쯤 가깝게 읽어 빈 바닥 위 0.3–0.6 m에 유령 칸이 한두 칸 깔린다 (11/06 14:35 재연: 3칸 조건 없이는
        빈 바닥을 나는 동안 상태값의 68%에서 잡혔다). 17:07 조형물은 지도에 0.4–0.5 m 꼭대기로만 들어 있어 2칸으로 둔다 — 유령 칸이
        남아도 위치 추정은 ToF가 그 꼭대기만큼 줄었을 때만 쓰므로(state_estimator under_*) 빈 바닥에서는 영향이 없다.
        below_z는 추정 높이가 아니라 경로 높이 상한처럼 추정과 무관한 값을 준다 — 추정 높이를 주면 높이가 커질수록 더 높은 칸을
        세어 되먹임으로 치솟는다 (재연에서 3.06 m)."""
        r = self.p.res
        k0 = max(0, int(math.floor((floor_z - self.origin[2]) / r)))
        k1 = min(int(math.floor((below_z - self.origin[2]) / r)), self.shape[2])
        if k1 <= k0:
            return None
        n = int(math.ceil(radius / r))
        i0 = int(math.floor((x - self.origin[0]) / r))
        j0 = int(math.floor((y - self.origin[1]) / r))
        i_lo, i_hi = max(i0 - n, 0), min(i0 + n + 1, self.shape[0])
        j_lo, j_hi = max(j0 - n, 0), min(j0 + n + 1, self.shape[1])
        if i_lo >= i_hi or j_lo >= j_hi:
            return None
        col = (self.logodds[i_lo:i_hi, j_lo:j_hi, k0:k1] > self.p.occupied) | self.hard[i_lo:i_hi, j_lo:j_hi, k0:k1]
        ks = np.flatnonzero(col.any(axis=(0, 1)))
        if ks.size < min_cells:
            return None
        return float(self.origin[2] + (k0 + int(ks.max()) + 1) * r)

    def flown_at(self, p) -> bool:
        """p(x, y, z) 칸이 드론이 실제로 지나간 칸인가 (그 뒤 막힘으로 확정되지 않았으면). 지도 밖은 False.
        카메라가 '봤다'는 것보다 확실하다: 단안 깊이는 얇은 문틀·흰 벽을 못 찍거나 어긋나게 찍는다 (11/16 Gazebo 방 두 개: 지도에는
        빈 곳이던 문틀 쪽으로 물러나 0.04 m까지 붙었다)."""
        i, j, k = (int(math.floor((p[n] - self.origin[n]) / self.p.res)) for n in range(3))
        if not (0 <= i < self.shape[0] and 0 <= j < self.shape[1] and 0 <= k < self.shape[2]):
            return False
        return bool(self.flown[i, j, k] and self.logodds[i, j, k] <= self.p.occupied)

    def known_free_at(self, p) -> bool:
        """p(x, y, z) 칸이 지나온 칸(막힘 확정이 아니면)이거나, 위아래 한 칸까지 본 적 있고 그 세 칸이 모두 막힘 의심 아래인가.
        지도 밖·모르는 칸은 False. 뒤를 보는 센서가 없어 물러나기 전에 뒤가 아는 빈 곳인지 본다 (near.rear_unknown)."""
        i, j, k = (int(math.floor((p[n] - self.origin[n]) / self.p.res)) for n in range(3))
        if not (0 <= i < self.shape[0] and 0 <= j < self.shape[1] and 0 <= k < self.shape[2]):
            return False
        if self.flown[i, j, k] and self.logodds[i, j, k] <= self.p.occupied:
            return True
        k0, k1 = max(k - 1, 0), min(k + 2, self.shape[2])
        return bool(self.seen[i, j, k0:k1].any() and (self.logodds[i, j, k0:k1] <= self.p.suspect).all())

    def seen_slice(self, k: int) -> np.ndarray:
        """seen_band()의 k 단면만 (위아래 한 칸까지 본 것으로). 단면 하나 보려고 100만 칸을 다 복사하지 않는다 (11/03 속도)."""
        s = self.seen[:, :, k].copy()
        if k > 0:
            s |= self.seen[:, :, k - 1]
        if k + 1 < self.shape[2]:
            s |= self.seen[:, :, k + 1]
        return s

    def sight_line(self, cam, target_xy, d0: float, d1: float, height: float, band: float = 0.35) -> tuple[bool, float]:
        """cam에서 target 쪽 수평선의 d0–d1 m 구간(높이 height ± band): (막힌 칸이 있나(의심 이상), 본 칸 비율).

        찾은 사람이 아직 있는지 볼 때 쓴다. 사람 몸이나 가리는 물건이 있으면 깊이가 그 자리를 막힘으로 남긴다.
        """
        dx, dy = target_xy[0] - cam[0], target_xy[1] - cam[1]
        n = math.hypot(dx, dy)
        ds = np.arange(max(0.0, d0), d1, self.p.res / 2)
        zs = np.arange(height - band, height + band + 1e-9, self.p.res)
        if n < 1e-6 or not len(ds) or not len(zs):
            return False, 0.0
        dd, zz = np.meshgrid(ds, zs, indexing="ij")
        pts = np.stack([cam[0] + dx / n * dd.ravel(), cam[1] + dy / n * dd.ravel(), zz.ravel()], axis=1)
        idx = self._index(pts)
        idx = idx[np.all((idx >= 0) & (idx < np.array(self.shape)), axis=1)]
        if not len(idx):
            return False, 0.0
        i, j, k = idx.T
        # seen_band()를 통째로(100만 칸, 1.3 ms) 만들지 않고 지나는 칸의 위아래만 본다 (탐지 콜백에서 사람마다 세 번까지 부른다, 11/03)
        seen = self.seen[i, j, k] | self.seen[i, j, np.maximum(k - 1, 0)] | self.seen[i, j, np.minimum(k + 1, self.shape[2] - 1)]
        return bool((self.logodds[i, j, k] > self.p.suspect).any()), float(seen.mean())

    def voxel_map(self) -> VoxelMap:
        """막힌 칸만 담은 지도 (ToF 기대 거리·벽 근처 판단용)."""
        return VoxelMap(self.origin.copy(), self.p.res, self.occupied())

    def planning_map(self, radius: float) -> VoxelMap:
        """막힌 칸은 radius만큼, 모르는 칸은 unknown_margin만큼 부풀려 막는다. 본 적 있는 빈 곳으로만 다닌다.

        높이 column(0.3–1.3 m) 사이에 막힌 칸이 있는 자리는 본 꼭대기 + over_clear(최소 over_min)까지 막는다. 카메라에는
        책상 위가 얇은 모니터 말고는 비어 보여 그 위 1.1 m로 날다가, 모니터가 있는 책상(1.25 m)·TV장(1.2 m)에 닿았다.
        그 위로는 카메라가 빈 것으로 본 칸으로만 넘는다(11/03). 문 위 벽(2.1 m 위)은 이 범위 밖이라 문은 막지 않는다.
        부풀리기는 수평으로만 radius, 위아래로는 한 칸(0.1 m)이다 (_dilate, 11/03 속도).
        """
        r = self.p.res
        occupied = self.logodds > self.p.suspect   # 한 번이라도 맞은 칸은 피한다 (막힘 확정은 occupied)
        k0, k1 = (min(max(int(h / r), 0), self.shape[2]) for h in self.p.column)
        column = occupied[:, :, k0:k1].any(axis=2)
        if self.p.over_clear > 0:
            nz = self.shape[2]
            k_scan = min(max(int(self.p.over_scan / r), 1), nz)
            ks = np.arange(k_scan)[None, None, :]
            top = np.where(occupied[:, :, :k_scan], ks, -1).max(axis=2)   # 기둥마다 가장 높은 막힌 칸
            # ToF가 찍은 띠(비행 높이 ±0.35 m)는 '그 높이에 무엇이 있다'일 뿐 꼭대기를 본 것이 아니다. 카메라 광선이 띠의 위 칸부터
            # 지우면 '본 꼭대기'가 낮아져 그 위 1.85 m로 넘는 길이 열렸다 (11/11 Gazebo 복도: 복도 입구에 잘못 찍힌 띠가 0.8–1.3 m만
            # 남자 그 위로 넘었다 — 유리·흰 벽이면 카메라가 너머를 봐 띠를 지우므로 그대로 들이받는다). 찍힌 칸이 하나라도 남은
            # 기둥은 찍혔던 높이까지 있는 것으로 본다. 다 지워지면 비행 높이도 같이 열린다 (밑 감시가 찍은 칸은 잰 꼭대기까지라 그대로다)
            hard_left = (occupied & self.hard)[:, :, :k_scan].any(axis=2)
            if hard_left.any():
                hard_top = np.where(self.hard[:, :, :k_scan], ks, -1).max(axis=2)
                top = np.where(hard_left, np.maximum(top, hard_top), top)
            k_block = np.maximum(int(math.ceil(self.p.over_min / r)), top + int(math.ceil(self.p.over_clear / r)) + 1)
            below = np.arange(nz)[None, None, :] < k_block[:, :, None]
            occupied = occupied | (column[:, :, None] & below)
        else:
            occupied = occupied | column[:, :, None]
        if self.p.close_windows:   # 창턱 위 유리 너머로 넘는 길을 막는다 (11/04)
            occupied = occupied | self.window_sills()[:, :, None]
        occ = _dilate(occupied, int(math.ceil(radius / r - 1e-9)))
        unk = _dilate(self.unknown(), int(math.ceil(self.p.unknown_margin / r - 1e-9)))
        # 드론이 지나온 길은 열어 둔다. 지나온 뒤 아래에 탁자가 보이는 식으로 주변이 통째로 막히면
        # 돌아갈 길이 사라져 그 자리에서 멈췄다. 다만 지나온 칸에 ToF가 다시 막힘을 찍었으면(지나간 뒤 사람·의자가 들어옴) 그 칸과
        # 둘레 radius는 막는다 — 최신의 확실한 관측이 이긴다 (11/04 외부 리뷰). 단안 깊이의 막힘은 잡음이 많아 지나온 칸을 못 덮는다
        opened = self.flown
        again = self.flown & self.hard & (self.logodds > self.p.occupied)   # 지나온 뒤 ToF로 다시 막힘 (지날 때 −1.75로 비웠다)
        if again.any():
            opened = self.flown & ~_dilate(again, int(math.ceil(radius / r - 1e-9)))
        return VoxelMap(self.origin.copy(), r, (occ | unk) & ~opened)

    def slice_index(self, height: float) -> int:
        return min(max(int(math.floor((height - self.origin[2]) / self.p.res)), 0), self.shape[2] - 1)

    def explored_area(self, height: float) -> float:
        """비행 높이에서 본 적 있는 빈 칸 넓이 (m²). known_free()의 그 단면만 만든다."""
        k = self.slice_index(height)
        return float((self.seen_slice(k) & ~(self.logodds[:, :, k] > self.p.occupied)).sum() * self.p.res ** 2)

    # ------------------------------------------------------------ 다음 볼 자리

    def frontier_viewpoint(self, x: float, y: float, planning: VoxelMap, height: float,
                           min_cells: int = 6, cost_distance: float = 3.0, exclude=(),
                           heading: float | None = None, momentum: float = 0.5, visited=(), visited_radius: float = 1.5,
                           z_top: float | None = None, shadow: float = 0.0, clearance: float = 0.0, standoff: float = 0.0,
                           view_range: float = 0.0, min_gain: float = 0.0, max_dist: float = math.inf,
                           passage: float = 0.0, passage_area: float = 1.0, reach_frontier: bool = False,
                           room_prefer: float = 0.0, room_weight: float = 2.0):
        """비행 높이에서 '모르는 칸'에 닿은 빈 칸 무리(frontier) 중 (무리 크기) / (1 + 거리/cost_distance) × 방향 계수가
        가장 큰 무리의, 갈 수 있는 가장 가까운 칸. (x, y, 무리 칸 수, 거리, 새로 보일 넓이 m² 또는 None) 또는 None (더 볼 곳 없음).

        view_range: 주면 무리 크기 대신 '그 자리에서 view_range m 안에 새로 보일 넓이'로 센다(사방으로 쏜 광선이 막힌 칸에서
        멈춤, 가구·벽에 붙은 그늘은 빼고). 경계 칸 수는 막힌 구석·벽 너머 틈에서도 크게 나와, 11/04 실기체는 가구와 창문 벽으로
        둘러싸인 구석을 85초 동안 오가며 트인 라운지 쪽은 끝내 고르지 않았다. 넓이가 min_gain m²보다 작은 무리는 고르지 않는다.
        exclude: 가 보려다 못 간 자리 (x, y) 또는 (x, y, 반지름). 반지름이 없으면 0.6 m 안은 고르지 않는다.
        max_dist: 가는 거리(격자 경로)가 이보다 먼 자리는 고르지 않는다 — 남은 탐색 시간 안에 닿을 곳만 (11/04 12:57: 98초째에
        10.4 m 밖을 골랐다).
        visited: 이미 둘러본 자리. visited_radius 안은 고르지 않는다 — 같은 곳을 조금 옆에서 다시 찍지 않게 (10/30 저녁 목표).
        heading: 가던 방향. 뒤쪽 무리는 (1 − momentum × 돌아야 할 각/π)배로 덜 친다(바로 뒤는 절반): 복도에서 앞뒤 경계를
        번갈아 오가지 않고 가던 쪽을 마저 본다.
        z_top: 주면 비행 높이–z_top 사이 어느 높이로든 지날 수 있는 칸으로 거리를 잰다(가구 위로 넘어 닿는 경계도 고른다, 11/03).
        실제로 갈 수 있는지는 3차원 경로 계획이 다시 확인한다.
        shadow: 막힌 칸에서 이 안의 모르는 칸은 경계로 치지 않는다. 가구·벽에 붙은 그늘(선반 칸 안쪽, 가구 뒤)은 카메라가 그 앞
        공간을 이미 다 본 곳이다 (11/03 14:32 실기체: 노트북 보관함 선반 칸 안쪽을 보러 선반 앞까지 들어가 뒤집혔다).
        clearance: 고르는 자리는 막힌 칸(비행 높이 ±0.35 m)에서 이만큼 떨어져야 한다 — 그 자리에서 제자리로 돌며 둘러본다.
        standoff: 주면 경계 칸 자체가 아니라 경계에서 이 안의, clearance가 빈 칸에서 경계가 보이면 거기서 본다. 문 경계는
        문틀 옆이라 좁다: 문 앞에서 문 너머를 보고, 다음 경계가 문 너머로 넘어가면 경로 계획이 문을 지난다.
        passage: 주면 shadow로 뺀 모르는 칸 중 '비행 높이 passage m 틈으로 passage_area m² 넘는 모르는 곳에 이어지는 입구'는
        다시 경계로 친다(_passage_unknown). shadow 0.3만 쓰면 0.6 m보다 좁은 통로도 경계가 아니었다 (11/04 사용자: 30×30 cm 넘는
        통로가 있으면 열린 공간, 그보다 작은 구멍이나 막힌 곳뿐이면 닫힌 공간).
        reach_frontier: 경계 무리 자체에 닿을 수 있어야 고른다(z_top을 주면 가구 위로 넘어서라도). 창턱 위로는 넘지 않는다 — 계획
        지도가 창턱 기둥을 막는다(window_sills): 깊이가 유리 너머를 빈 곳으로 칠해도 닫힌 창이다 (11/04 사용자: 창문이 닫혀 있으면
        막힌 것으로). 탁자·의자처럼 홀로 선 가구 위로는 그대로 넘는다(10:17 실기체는 떠 있던 시간의 77%를 1.2 m 위에서 날았다).
        room_prefer: 주면(standoff와 함께) 후보 가운데 둘레가 이만큼(m) 빈 자리를 먼저 고른다 — 모자란 여유 1 m를 가는 거리
        room_weight m로 쳐서 순서를 매긴다. 전에는 가는 길에서 가장 가까운 후보를 골라, 둘레가 clearance만 겨우 넘는 벽 옆 자리가
        됐다 (11/14 실기체 12판의 둘러본 자리 25곳: 벽까지 중앙 0.51 m, 84%가 0.7 m 안, 80%는 1 m 안에 0.5 m 넘게 더 넓은 자리가
        있었다. 그 자리에서 제자리로 돌고, 떠나는 길도 벽 옆에서 시작한다).
        """
        k = self.slice_index(height)
        r = self.p.res
        free2d = ~planning.occ[:, :, k]
        need_occ = shadow > 0 or clearance > 0 or standoff > 0 or view_range > 0 or passage > 0
        unknown2d, occ2d, room = self._frontier_unknown(height, shadow, passage, passage_area, need_occ, z_top)
        frontier = free2d & self._near_unknown(unknown2d, occ2d, passage > 0)
        labels, n = ndimage.label(frontier, structure=np.ones((3, 3)))
        if n == 0:
            return None
        dist = _bfs(self._passable(planning, height, z_top), self.origin[:2], r, x, y)
        best = None
        for lab in range(1, n + 1):
            mask = labels == lab
            cells = np.argwhere(mask)
            if len(cells) < min_cells:
                continue
            if reach_frontier and not np.isfinite(dist[cells[:, 0], cells[:, 1]]).any():
                continue   # 창턱·가구를 넘어야만 닿는 경계
            target = None
            dd = dist[cells[:, 0], cells[:, 1]]
            order = np.argsort(dd)
            cand = cells
            if standoff > 0:   # 경계에서 standoff 안, 둘레가 빈 칸 (경계가 가려지지 않게 그 칸에서 가장 가까운 경계 칸까지 본다)
                # 무리를 ±standoff 상자로 잘라 EDT로 '경계에서 몇 칸'을 구한다. gap ≤ 반지름인 칸은 원반으로 부풀린 칸과 같은
                # 집합이고, 상자 밖은 더 멀다. 무리마다 25×25 부풀리기 10 ms가 0.2 ms로 (11/03, 무리 9개면 80 ms 절약)
                rs = int(math.ceil(standoff / r))
                box, gap, (ti, tj) = _near_cluster(mask, rs)
                sub = np.argwhere((gap <= rs) & free2d[box] & np.isfinite(dist[box]) & (room[box] >= clearance))
                if not len(sub):
                    continue
                cand = sub + np.array([box[0].start, box[1].start])
                target = (ti[sub[:, 0], sub[:, 1]] + box[0].start, tj[sub[:, 0], sub[:, 1]] + box[1].start)   # 후보마다 가장 가까운 경계 칸
                dd = dist[cand[:, 0], cand[:, 1]]
                key = dd + 1.5 * r * gap[sub[:, 0], sub[:, 1]]   # 가는 길에서 경계에 가장 가까운 칸부터
                if room_prefer > 0:   # 둘레가 넓은 자리 먼저
                    key = key + room_weight * np.clip(room_prefer - room[cand[:, 0], cand[:, 1]], 0.0, None)
                order = np.argsort(key)
            if not np.isfinite(dd).any():
                continue
            for j in order:
                if not math.isfinite(dd[j]):
                    break
                if dd[j] > max_dist:
                    continue
                ci, cj = cand[j]
                if room is not None and room[ci, cj] < clearance:
                    continue
                if target is not None and not _line_free(occ2d, (ci, cj), (target[0][j], target[1][j])):
                    continue
                cx, cy = self.origin[:2] + (cand[j] + 0.5) * r
                if any(math.hypot(cx - e[0], cy - e[1]) < (e[2] if len(e) > 2 else 0.6) for e in exclude):
                    continue
                if any(math.hypot(cx - vx, cy - vy) < visited_radius for vx, vy in visited):
                    continue
                area = None
                if view_range > 0:
                    area = _visible_unknown(unknown2d, occ2d, self.origin[:2], r, float(cx), float(cy), view_range) * r * r
                    if area < min_gain:
                        break   # 이 무리는 보러 가도 새로 보일 게 없다 (막힌 구석·벽 너머 틈)
                score = (area if area is not None else len(cells)) / (1.0 + dd[j] / cost_distance)
                if heading is not None:
                    turn = abs(((math.atan2(cy - y, cx - x) - heading) + math.pi) % (2 * math.pi) - math.pi)
                    score *= 1.0 - momentum * turn / math.pi
                if best is None or score > best[0]:
                    best = (score, float(cx), float(cy), len(cells), float(dd[j]), area)
                break
        return None if best is None else best[1:]

    def _near_unknown(self, unknown2d: np.ndarray, occ2d: np.ndarray | None, around: bool) -> np.ndarray:
        """모르는 칸에서 0.4 m 안의 칸. around면 막힌 칸(±0.35 m)을 넘지 않고 잰다 — 직선으로 재면 10 cm 벽 너머의 모르는 칸
        때문에 벽 앞 빈 칸이 경계가 되었다 (11/04, 창 장면 시험)."""
        r = self.p.res
        if not around or occ2d is None:
            return _dilate(unknown2d[:, :, None], int(math.ceil(0.4 / r)))[:, :, 0]
        return ndimage.binary_dilation(unknown2d, structure=np.ones((3, 3), dtype=bool),
                                       iterations=int(math.ceil(0.4 / r)), mask=~occ2d | unknown2d)

    def _passable(self, planning: VoxelMap, height: float, z_top: float | None, through_windows: bool = False) -> np.ndarray:
        """비행 높이(z_top을 주면 그 사이 어느 높이로든) 지날 수 있는 칸 (2D). 창턱은 계획 지도가 이미 막았다(window_sills).
        through_windows: 창턱과 그 둘레 0.5 m(계획 지도의 부풀림)도 지날 수 있다고 본다 — '닫힌 창 너머'인지 가릴 때만 쓴다."""
        k = self.slice_index(height)
        free2d = ~planning.occ[:, :, k]
        over = free2d if z_top is None or z_top <= height else ~planning.occ[:, :, k:self.slice_index(z_top) + 1].all(axis=2)
        if through_windows:
            sill = self.window_sills()
            if sill.any():
                over = over | _dilate(sill[:, :, None], int(round(0.5 / self.p.res)))[:, :, 0]
        return over

    def window_sills(self) -> np.ndarray:
        """창턱 칸 (2D): 창턱 높이(WINDOW_SILL 0.5–1.2 m)가 막혔고, 그 위(1.2–1.9 m)는 막히지 않았고, WINDOW_LINE 넘게 이어진
        위 벽(WALL_TALL 1.9–2.3 m에 막힌 칸)에서 WINDOW_NEAR 안. 깊이는 유리를 꿰뚫어 창 너머를 빈 곳으로 칠하고 창턱 위
        1.5–1.85 m는 비어 보여, 가구처럼 넘어 지나는 길이 생겼다 (11/04 사용자: 창문이 닫혀 있으면 막힌 것으로). 홀로 선
        가구(탁자·의자) 위에는 높은 칸이 없어 들지 않고, 벽(가운데도 막힘)은 들지 않는다. 드론이 지나온 자리(0.5 m 안)는 뺀다 —
        닫힌 창은 지날 수 없으니 덕트·등 아래 책상 위에 떠 있다 갇히지 않게."""
        r = self.p.res
        occ = self.logodds > self.p.suspect
        s0, s1, t0, t1 = (min(max(int(round((h - self.origin[2]) / r)), 0), self.shape[2]) for h in WINDOW_SILL + WALL_TALL)
        tall = occ[:, :, t0:t1].any(axis=2)
        if not tall.any():
            return np.zeros(tall.shape, dtype=bool)
        labels, n = ndimage.label(tall, structure=np.ones((3, 3)))
        sizes = ndimage.sum(tall, labels, index=np.arange(1, n + 1))
        tall = np.concatenate([[False], sizes * r >= WINDOW_LINE - 1e-9])[labels]
        sill = occ[:, :, s0:s1].any(axis=2) & ~occ[:, :, s1:t0].any(axis=2)
        sill &= _dilate(tall[:, :, None], int(math.ceil(WINDOW_NEAR / r - 1e-9)))[:, :, 0]
        if sill.any() and self.flown.any():
            sill &= ~_dilate(self.flown.any(axis=2)[:, :, None], int(round(0.5 / r)))[:, :, 0]
        return sill

    def _frontier_unknown(self, height: float, shadow: float, passage: float, passage_area: float, need_occ: bool,
                          z_top: float | None = None):
        """경계로 칠 모르는 칸(2D, 비행 높이), 막힌 칸(±0.35 m), 막힌 칸까지 거리(m). shadow·passage는 frontier_viewpoint와 같다."""
        k = self.slice_index(height)
        r = self.p.res
        raw = ~self.seen_slice(k)
        unknown2d = raw
        occ2d = room = None
        if need_occ or shadow > 0:
            k0, k1 = self.slice_index(height - 0.35), self.slice_index(height + 0.35) + 1
            occ2d = (self.logodds[:, :, k0:k1] > self.p.suspect).any(axis=2)
            room = ndimage.distance_transform_edt(~occ2d) * r   # 막힌 칸까지 거리 (m)
        if shadow > 0:
            unknown2d = raw & ~_dilate(occ2d[:, :, None], int(math.ceil(shadow / r - 1e-9)))[:, :, 0]
        if passage > 0:
            reach, mouth = self._passage_unknown(height, raw, passage, passage_area, z_top)
            unknown2d = (unknown2d & reach) | mouth
        return unknown2d, occ2d, room

    def _passage_unknown(self, height: float, raw: np.ndarray, passage: float, area: float, z_top: float | None = None):
        """비행 높이 ±passage/2 띠에서 막힌 칸과 passage/2 넘게 떨어진 '트인 칸'(= passage m 넘는 틈이 지나는 칸)으로 보아
        (reach, mouth)를 돌려준다.
        reach: 본 빈 곳과 트인 칸으로 이어진 모르는 칸. 30×30 cm보다 작은 구멍 너머·벽 너머는 들지 않는다.
        mouth: 그중 본 빈 곳에서 (트인 칸을 따라) PASSAGE_MOUTH m 안이고, 그 모르는 덩어리가 area m² 넘는 칸 — 문·좁은 통로의
        입구다. shadow가 막힌 칸 0.3 m 안을 다 빼서 0.6 m보다 좁은 통로는 경계가 아니었는데, 이 입구는 다시 넣는다.
        선반 칸은 모르는 덩어리가 작아 들지 않는다. z_top을 주면 비행 높이부터 z_top까지 passage 간격의 높이들 중 한 곳에서라도
        트였으면 트인 칸이다(가구 위로 넘어 지나는 틈도 통로)."""
        r = self.p.res
        top = height if z_top is None or z_top <= height else z_top - passage / 2
        open_ = np.zeros(raw.shape, dtype=bool)
        for h in np.arange(height, top + 1e-6, passage):
            s0, s1 = self.slice_index(h - passage / 2), self.slice_index(h + passage / 2) + 1
            occ_slot = (self.logodds[:, :, s0:s1] > self.p.suspect).any(axis=2)
            open_ |= ndimage.distance_transform_edt(~occ_slot) * r >= passage / 2 - 1e-9
        cross = ndimage.generate_binary_structure(2, 1)
        seen_open = open_ & ~raw
        comp, _ = ndimage.label(open_, structure=cross)
        linked = np.zeros(comp.max() + 1, dtype=bool)
        linked[np.unique(comp[seen_open])] = True
        linked[0] = False
        unk_open = raw & open_
        reach = unk_open & linked[comp]
        labels, n = ndimage.label(reach, structure=cross)
        if n == 0:
            return reach, np.zeros_like(raw)
        sizes = ndimage.sum(reach, labels, index=np.arange(1, n + 1)) * r * r
        big = np.concatenate([[False], sizes >= area])
        near = ndimage.binary_dilation(seen_open, structure=cross, iterations=max(1, int(round(PASSAGE_MOUTH / r))), mask=open_)
        return reach, reach & near & big[labels]

    def frontier_census(self, x: float, y: float, planning: VoxelMap, height: float, shadow: float = 0.3,
                        passage: float = 0.3, passage_area: float = 1.0, z_top: float | None = None, reach: float = 5.0,
                        min_cells: int = 6) -> dict:
        """지금 공간(드론에서 reach m 안, z_top을 주면 가구 위로 넘어서라도)의 경계를 센다: passages(닿는 통로 경계), beyond(창턱을
        넘어야만 닿는 경계 — 닫힌 창 너머), small(30×30보다 작거나 가구에 붙어 경계로 안 치는 모르는 구멍), local_m2(그 공간 넓이).
        11/04 사용자: 30×30 cm 넘는 통로가 없으면(작은 구멍·막힌 곳·닫힌 창뿐이면) 닫힌 공간 — 그 공간을 마치고 다른 곳으로."""
        k = self.slice_index(height)
        r = self.p.res
        free2d = ~planning.occ[:, :, k]
        unknown2d, occ2d, _ = self._frontier_unknown(height, shadow, passage, passage_area, True, z_top)
        labels, n = ndimage.label(free2d & self._near_unknown(unknown2d, occ2d, passage > 0), structure=np.ones((3, 3)))
        dist = _bfs(self._passable(planning, height, z_top), self.origin[:2], r, x, y)
        dist_over = _bfs(self._passable(planning, height, z_top, through_windows=True), self.origin[:2], r, x, y)
        passages = beyond = 0
        for lab in range(1, n + 1):
            cells = np.argwhere(labels == lab)
            if len(cells) < min_cells:
                continue
            d = float(dist[cells[:, 0], cells[:, 1]].min())
            if d <= reach:
                passages += 1
            elif not math.isfinite(d) and dist_over is not None and float(dist_over[cells[:, 0], cells[:, 1]].min()) <= reach + 1.0:
                beyond += 1
        local = np.isfinite(dist) & (dist <= reach)
        # 작은 구멍: 지금 공간의 본 빈 칸(막힌 칸 아님)과 바로 맞닿았는데 경계로 치지 않는 모르는 덩어리. 벽 너머 모르는 곳은
        # 막힌 칸하고만 맞닿아 세지 않는다
        cross = ndimage.generate_binary_structure(2, 1)
        seen = self.seen_slice(k)
        space = ndimage.binary_dilation(local, structure=cross, iterations=max(1, int(round(0.4 / r)))) & seen & ~occ2d
        holes = ~seen & ~unknown2d
        hl, _ = ndimage.label(holes, structure=cross)
        ids = np.unique(hl[ndimage.binary_dilation(space, structure=cross) & (hl > 0)])
        small = 0
        if len(ids):
            sizes = ndimage.sum(holes, hl, index=ids)
            small = int((sizes >= 4).sum())   # 0.04 m² 넘는 구멍만 센다
        return {"passages": passages, "beyond": beyond, "small": small, "local_m2": float(local.sum() * r * r)}

    def blocked_frontier_approach(self, x: float, y: float, planning: VoxelMap, height: float,
                                  min_cells: int = 6, max_gap: float = 2.5, exclude=(), shadow: float = 0.0,
                                  z_top: float | None = None):
        """갈 수 있는 경계가 없을 때 다가가 다시 볼 자리. 멀리서 본 헛 장애물(단안 깊이는 물체에 따라 20%쯤
        가깝게 찍는다)이 문·통로를 막았으면, 가까이서 보는 깊이 영상이 그 칸을 다시 비운다.

        '아는 빈 칸'과 '모르는 칸'의 경계 중 드론이 갈 수 없는 무리마다, 갈 수 있는 칸 가운데 그 무리에 가장
        가까운 칸(max_gap 안)을 찾는다. (접근 x, y, 볼 x, 볼 y, 무리 칸 수, 무리 중심 x, y) 또는 None.
        exclude: 이미 다가가 본 무리의 중심. 1 m 안 무리는 다시 고르지 않는다.
        shadow: frontier_viewpoint와 같다 (가구·벽에 붙은 그늘은 경계가 아니다).
        창턱(계획 지도가 막음)을 지나면 닿는 무리는 고르지 않는다 — 닫힌 창 너머다 (11/04). z_top을 주면 가구 위로 넘어
        닿는 무리도 고르지 않는다(볼 자리 고르기가 맡는다). 넘어서도 못 닿는
        무리(높이 전체가 막힘)만 헛 장애물이 막은 문·통로로 보고 다가간다."""
        k = self.slice_index(height)
        r = self.p.res
        seen2d = self.seen_slice(k)
        unknown2d = ~seen2d
        if shadow > 0:
            k0, k1 = self.slice_index(height - 0.35), self.slice_index(height + 0.35) + 1
            occ2d = (self.logodds[:, :, k0:k1] > self.p.suspect).any(axis=2)
            unknown2d = unknown2d & ~_dilate(occ2d[:, :, None], int(math.ceil(shadow / r - 1e-9)))[:, :, 0]
        free_plan = ~planning.occ[:, :, k]
        dist = _bfs(free_plan, self.origin[:2], r, x, y)
        reach = np.isfinite(dist)
        if not reach.any():
            return None
        over = np.isfinite(_bfs(self._passable(planning, height, z_top, through_windows=True), self.origin[:2], r, x, y))
        near_unknown = _dilate(unknown2d[:, :, None], int(math.ceil(0.4 / r)))[:, :, 0]
        frontier = seen2d & ~(self.logodds[:, :, k] > self.p.occupied) & near_unknown & ~reach   # known_free()의 이 단면
        labels, n = ndimage.label(frontier, structure=np.ones((3, 3)))
        best = None
        reach_cells = np.argwhere(reach)
        for lab in range(1, n + 1):
            cells = np.argwhere(labels == lab)
            if len(cells) < min_cells:
                continue
            cx, cy = self.origin[:2] + (cells.mean(axis=0) + 0.5) * r
            if any(math.hypot(cx - ex, cy - ey) < 1.0 for ex, ey in exclude):
                continue
            if over[cells[:, 0], cells[:, 1]].any():
                continue   # 창을 지나거나 가구를 넘으면 닿는 곳: 막힌 문·통로가 아니다
            edt, inds = ndimage.distance_transform_edt(labels != lab, return_indices=True)   # 무리까지 거리
            gaps = edt[reach_cells[:, 0], reach_cells[:, 1]] * r
            j = int(np.argmin(gaps))
            if gaps[j] > max_gap:
                continue
            ai, aj = reach_cells[j]
            ti, tj = inds[0][ai, aj], inds[1][ai, aj]
            score = len(cells) / (1.0 + dist[ai, aj] / 3.0)
            if best is None or score > best[0]:
                ax, ay = self.origin[:2] + (np.array([ai, aj]) + 0.5) * r
                tx, ty = self.origin[:2] + (np.array([ti, tj]) + 0.5) * r
                best = (score, float(ax), float(ay), float(tx), float(ty), len(cells), float(cx), float(cy))
        return None if best is None else best[1:]

    def best_look(self, x: float, y: float, height: float, hfov: float, view: float = 3.0,
                  dirs: int = 8, now_yaw: float | None = None) -> tuple[float, int]:
        """(x, y)에서 모르는 칸이 가장 많이 보이는 방향과 그 칸 수 (가림은 막힌 칸으로 판단).

        now_yaw를 주면 많이 돌아야 보는 방향은 덜 친다(반대쪽은 절반). 10/30 실기체는 볼 자리마다 한 바퀴씩 돌아
        비행의 1/4(43 s)을 제자리 회전에 썼다. 돌려준 칸 수는 깎지 않은 값이다.
        """
        k = self.slice_index(height)
        unknown2d = ~self.seen_slice(k)
        occ2d = self.logodds[:, :, k] > self.p.occupied
        r = self.p.res
        best, best_score = (0.0, 0), -1.0
        for d in range(dirs):
            yaw = -math.pi + d * 2 * math.pi / dirs
            count = 0
            for a in np.linspace(yaw - hfov / 2, yaw + hfov / 2, 15):
                for t in np.arange(0.1, view, r):
                    i = int((x + t * math.cos(a) - self.origin[0]) / r)
                    j = int((y + t * math.sin(a) - self.origin[1]) / r)
                    if not (0 <= i < self.shape[0] and 0 <= j < self.shape[1]) or occ2d[i, j]:
                        break
                    count += int(unknown2d[i, j])
            turn = abs((yaw - now_yaw + math.pi) % (2 * math.pi) - math.pi) if now_yaw is not None else 0.0
            score = count * (1.0 - 0.5 * turn / math.pi)
            if score > best_score:
                best, best_score = (yaw, count), score
        return best

    # ------------------------------------------------------------ 저장

    def save(self, path) -> None:
        np.savez_compressed(path, logodds=self.logodds, seen=self.seen, hard=self.hard, origin=self.origin, res=self.p.res)


def _dilate(mask: np.ndarray, r: int) -> np.ndarray:
    """참 칸을 수평으로 r칸 원반(dx² + dy² ≤ r²)만큼 부풀리고, 3D 배열이면 위아래로 한 칸(0.1 m)만 더 부풀린다.

    11/03 (속도): 9×9×9 공으로 100만 칸을 부풀리면(binary_dilation) 121–139 ms라 1초마다의 지도 갱신이 20 Hz 제어
    주기를 통째로 막았다. 원반은 줄(dy)마다 가로 선분이므로, 선분 폭마다 1D 최대 필터(가로)를 한 번 돌리고 세로로 dy만큼
    옮겨 OR하면 같은 집합이 9 ms에 나온다 (같은 입력에서 binary_dilation(원반)과 다른 칸 0). 위아래는 그 결과를 한 칸씩
    옮겨 OR한다. 수직 여유가 0.4 m(공)→0.1 m(기체 높이 10 cm)로 주는 것은 의도한 변경이다: 가구 위는 planning_map의
    기둥 막기와 '본 꼭대기 + over_clear 0.6 m' 규칙이 따로 막고, 높이는 하방 ToF로 정확히 잰다.
    2D로 쓰는 호출(mask[:, :, None])은 위아래 칸이 없으므로 예전처럼 원반으로만 부풀린다.
    """
    if r <= 0:
        return mask.copy()
    out = np.zeros_like(mask)
    src = mask.view(np.uint8)
    ny = mask.shape[1]
    lines = {}   # 가로 선분 폭 → 그 폭으로 가로 부풀린 배열 (같은 폭은 한 번만)
    for dy in range(-r, r + 1):
        w = int(math.floor(math.sqrt(r * r - dy * dy)))   # 이 줄에서 원반이 가로로 닿는 칸 수
        if w not in lines:
            lines[w] = ndimage.maximum_filter1d(src, 2 * w + 1, axis=0, mode="constant", cval=0).view(bool)
        if dy >= 0:
            out[:, dy:] |= lines[w][:, :ny - dy]
        else:
            out[:, :dy] |= lines[w][:, -dy:]
    if mask.ndim == 3 and mask.shape[2] > 1:
        mid = out.copy()
        out[:, :, 1:] |= mid[:, :, :-1]
        out[:, :, :-1] |= mid[:, :, 1:]
    return out


def _near_cluster(mask: np.ndarray, r: int):
    """2D 무리(mask)에서 r칸 안을 보려고 무리를 ±r 상자로 잘라 EDT를 구한다: (상자 slice, 무리까지 거리(칸), 가장 가까운 무리 칸).

    거리·인덱스는 상자 좌표다. gap ≤ r인 칸은 _dilate(mask, r)와 같은 집합(둘 다 유클리드 r)이고 상자 밖은 r보다 멀다.
    무리는 상자 안에 다 들어 있으므로 상자 안 거리는 전체 지도에서 구한 것과 같다 (11/03 속도: 무리마다 10 ms → 0.2 ms).
    """
    ii, jj = np.nonzero(mask)
    i0, i1 = max(int(ii.min()) - r, 0), min(int(ii.max()) + r + 1, mask.shape[0])
    j0, j1 = max(int(jj.min()) - r, 0), min(int(jj.max()) + r + 1, mask.shape[1])
    box = (slice(i0, i1), slice(j0, j1))
    gap, target = ndimage.distance_transform_edt(~mask[box], return_indices=True)
    return box, gap, target


def _visible_unknown(unknown2d: np.ndarray, block2d: np.ndarray, origin_xy, res: float, x: float, y: float,
                     view_range: float, rays: int = 120) -> int:
    """(x, y)에서 사방으로 쏜 광선이 막힌 칸(block2d)에 닿기 전까지 지나는 모르는 칸 수 (view_range m 안, 2D).

    볼 자리에 가면 제자리에서 돌며 둘러보므로 사방을 센다. 120줄이면 3 m에서 줄 사이 0.16 m (칸 0.1 m)."""
    a = np.linspace(-math.pi, math.pi, rays, endpoint=False)
    t = np.arange(res / 2, view_range, res / 2)
    i = np.floor((x + np.cos(a)[:, None] * t[None, :] - origin_xy[0]) / res).astype(int)
    j = np.floor((y + np.sin(a)[:, None] * t[None, :] - origin_xy[1]) / res).astype(int)
    nx, ny = block2d.shape
    inside = (i >= 0) & (i < nx) & (j >= 0) & (j < ny)
    i, j = np.clip(i, 0, nx - 1), np.clip(j, 0, ny - 1)
    alive = np.logical_and.accumulate(inside & ~block2d[i, j], axis=1)   # 처음 막힌 칸(또는 지도 끝)에서 멈춤
    hit = alive & unknown2d[i, j]
    return int(np.unique(i[hit] * ny + j[hit]).size)


def _line_free(occ2d: np.ndarray, a, b) -> bool:
    """칸 a에서 b까지 직선 위에 막힌 칸이 없는가 (2D, 반 칸 간격)."""
    (ai, aj), (bi, bj) = a, b
    steps = max(1, 2 * int(max(abs(bi - ai), abs(bj - aj))))
    for t in np.linspace(0.0, 1.0, steps + 1):
        if occ2d[int(round(ai + (bi - ai) * t)), int(round(aj + (bj - aj) * t))]:
            return False
    return True


def _bfs(free2d: np.ndarray, origin, res: float, x: float, y: float) -> np.ndarray:
    """(x, y)에서 갈 수 있는 칸까지의 대략 거리 (m, 8방향). 못 가면 inf. 시작이 막혀 있으면 가장 가까운 빈 칸에서.

    막힌 테두리를 한 칸 두른 평면 리스트로 돌아 칸마다의 numpy 조회·경계 검사를 뺀다 (11/03 속도: 200×200에서 9 ms → 3 ms).
    완화 순서(FIFO)·이동 순서·비용식이 예전과 같아 거리 값도 비트까지 같다. scipy(label + 거리 변환)는 격자 경로 거리가 아니고,
    csgraph dijkstra는 같은 길이의 다른 길을 고르면 마지막 자리가 달라 정렬 순서가 바뀔 수 있어 쓰지 않았다.
    """
    nx, ny = free2d.shape
    dist = np.full((nx, ny), np.inf)
    si, sj = int((x - origin[0]) / res), int((y - origin[1]) / res)
    if not (0 <= si < nx and 0 <= sj < ny):
        return dist
    if not free2d[si, sj]:
        cand = np.argwhere(free2d)
        if len(cand) == 0:
            return dist
        k = int(np.argmin((cand[:, 0] - si) ** 2 + (cand[:, 1] - sj) ** 2))
        if (cand[k, 0] - si) ** 2 + (cand[k, 1] - sj) ** 2 > 25:   # 0.5 m 안에 빈 칸이 없으면 못 간다
            return dist
        si, sj = int(cand[k, 0]), int(cand[k, 1])
    w = ny + 2                                   # 테두리를 두른 평면 배열의 한 줄 길이
    padded = np.zeros((nx + 2, ny + 2), dtype=bool)
    padded[1:-1, 1:-1] = free2d
    free = padded.ravel().tolist()
    d = [math.inf] * len(free)
    start = (si + 1) * w + (sj + 1)
    d[start] = 0.0
    queue = deque([start])
    pop, push = queue.popleft, queue.append
    moves = [(w, res), (-w, res), (1, res), (-1, res),
             (w + 1, 1.414 * res), (w - 1, 1.414 * res), (-w + 1, 1.414 * res), (-w - 1, 1.414 * res)]
    while queue:
        cur = pop()
        base = d[cur]
        for off, c in moves:
            n = cur + off
            if free[n] and d[n] > base + c + 1e-9:
                d[n] = base + c
                push(n)
    dist[:] = np.array(d).reshape(nx + 2, ny + 2)[1:-1, 1:-1]
    return dist
