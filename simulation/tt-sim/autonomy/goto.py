"""사람이 찍은 자리 (2026-11-11): 자율 탐색 중에 운용자가 웹 지도에서 찍은 곳을 다음 볼 자리로 삼는다.

비행 노드는 /tt/mission_cmd로 `goto <x> <y>` · `goto cancel`을 받는다. 가는 길·회피·문 지나기는 평소 볼 자리와 똑같이 자동이고,
도착하면 둘러본 뒤 자율 탐색을 잇는다. 여기에는 ROS 없이 시험할 수 있는 두 가지만 둔다: 명령 읽기(parse)와,
찍은 곳으로 바로 갈 수 없을 때(막힘·못 본 곳·아직 길이 안 이어짐) 갈 수 있는 가장 가까운 빈 자리로 옮기기(snap).
"""
from __future__ import annotations

import math

import numpy as np
from scipy import ndimage

# 찍은 곳으로 바로 못 가면(막힘·못 본 곳·아직 길이 안 이어짐) 드론과 이어진 빈 칸 가운데 찍은 곳에 가장 가까운 자리로 옮긴다.
# 그 자리가 찍은 곳 SNAP_MAX 안이거나, 지금 드론 자리보다 찍은 곳에 SNAP_GAIN 넘게 가까울 때만이다 — 이미 가장 가까이 와 있으면 받지 않는다.
# 11/11 Gazebo 방 두 개: 처음에는 SNAP_MAX 안만 받고 '찍은 칸이 비었으면 그대로'였는데, 웹 지도에 본 곳으로 보이지만 계획 지도에서는 아직
# 길이 이어지지 않은 3.9 m 밖 자리를 찍자 '가는 길이 없습니다'로 끝내고 반대쪽으로 탐색을 이어 갔다
SNAP_MAX = 1.5
SNAP_GAIN = 0.5
SNAP_CLEAR = 0.2    # 옮길 때는 둘레가 이만큼 빈 칸을 먼저 고른다: 도착해서 제자리에서 돌며 둘러본다 (m)
ROOMY_EXTRA = 0.5   # 그런 칸이 가장 가까운 빈 칸보다 이만큼 넘게 멀면 가장 가까운 빈 칸으로 간다 (m)
START_REACH = 0.4   # 드론이 있는 칸이 계획 지도에서 막혀 있으면(부풀린 벽 옆) 이 안의 빈 칸에서 이어진 곳을 찾는다 (m)
COORD_MAX = 1000.0  # 이보다 큰 좌표는 잘못 온 값으로 본다 (m)


def parse(text: str):
    """`goto <x> <y>` → ("goto", x, y), `goto cancel` → ("cancel",). goto 명령이 아니면 None, 꼴이 틀리면 ValueError."""
    parts = text.split()
    if not parts or parts[0] != "goto":
        return None
    if parts[1:] == ["cancel"]:
        return ("cancel",)
    if len(parts) != 3:
        raise ValueError("goto <x> <y> 또는 goto cancel 꼴이어야 합니다")
    try:
        x, y = float(parts[1]), float(parts[2])
    except ValueError:
        raise ValueError("좌표가 숫자가 아닙니다") from None
    if not (math.isfinite(x) and math.isfinite(y)) or max(abs(x), abs(y)) > COORD_MAX:
        raise ValueError("좌표가 범위를 벗어났습니다")
    return ("goto", x, y)


def snap(free2d, origin_xy, res: float, start, click, snap_max: float = SNAP_MAX, clear: float = SNAP_CLEAR,
         gain: float = SNAP_GAIN, exact: bool = True):
    """찍은 곳 click을 갈 자리로 바꾼다: (x, y, 찍은 곳에서 옮긴 거리 m). 받을 수 없으면 None.

    free2d[i, j]는 비행 높이에서 계획 지도가 빈 칸(본 적 있는 빈 곳, 벽은 부풀린 뒤)이다. start는 드론 자리.
    - exact이고 찍은 칸이 비었으면 그대로 간다 (옮긴 거리 0). 길이 있는지는 경로 계획이 본다 — 문·가구 위로 넘는 길은 이 단면에서 안 보인다.
    - 아니면(막힌 칸·못 본 칸, 또는 exact=False: 그 칸까지 길이 없었다) 드론 자리와 이어진 빈 칸 가운데 찍은 곳에 가장 가까운 칸으로 옮긴다.
      그 칸이 찍은 곳 snap_max 안이거나, 드론보다 찍은 곳에 gain 넘게 가까울 때만이다. 둘레 clear가 빈 칸이 있으면 그 가운데서 고른다.
    """
    free2d = np.asarray(free2d, dtype=bool)
    nx, ny = free2d.shape
    ox, oy = float(origin_xy[0]), float(origin_xy[1])

    def cell(p):
        return int(math.floor((p[0] - ox) / res)), int(math.floor((p[1] - oy) / res))

    ci, cj = cell(click)
    if exact and 0 <= ci < nx and 0 <= cj < ny and free2d[ci, cj]:
        return float(click[0]), float(click[1]), 0.0
    labels, n = ndimage.label(free2d, structure=np.ones((3, 3)))
    if n == 0:
        return None
    here = _label_near(labels, cell(start), int(math.ceil(START_REACH / res)))
    if here == 0:
        return None
    ii, jj = np.nonzero(labels == here)
    dist = np.hypot(ox + (ii + 0.5) * res - click[0], oy + (jj + 0.5) * res - click[1])
    ok = dist <= max(snap_max, math.hypot(start[0] - click[0], start[1] - click[1]) - gain)
    if not ok.any():
        return None
    if clear > 0:   # 둘레가 빈 칸을 먼저. 다만 가장 가까운 빈 칸보다 ROOMY_EXTRA 넘게 멀어지면서까지 고르지는 않는다
        roomy = ndimage.binary_erosion(free2d, structure=np.ones((3, 3)), iterations=int(math.ceil(clear / res - 1e-9)))
        pref = ok & roomy[ii, jj] & (dist <= float(dist[ok].min()) + ROOMY_EXTRA)
        if pref.any():
            ok = pref
    best = int(np.argmin(np.where(ok, dist, np.inf)))
    return float(ox + (ii[best] + 0.5) * res), float(oy + (jj[best] + 0.5) * res), float(dist[best])


def _label_near(labels: np.ndarray, idx, reach: int) -> int:
    """idx 칸의 덩어리 번호. 그 칸이 막혔으면 reach칸 안에서 가장 가까운 빈 칸의 번호 (없으면 0)."""
    nx, ny = labels.shape
    i0, j0 = idx
    if 0 <= i0 < nx and 0 <= j0 < ny and labels[i0, j0]:
        return int(labels[i0, j0])
    best, best_d = 0, math.inf
    for i in range(max(0, i0 - reach), min(nx, i0 + reach + 1)):
        for j in range(max(0, j0 - reach), min(ny, j0 + reach + 1)):
            d = math.hypot(i - i0, j - j0)
            if labels[i, j] and d <= reach and d < best_d:
                best, best_d = int(labels[i, j]), d
    return best
