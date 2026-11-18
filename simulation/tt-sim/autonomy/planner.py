"""부풀린 복셀 지도에서 3D A*로 경로를 찾고, 직선으로 이어도 되는 점은 건너뛰어 단순하게 만든다.

부풀린 경계에 딱 붙은 경로는 위치 추정이 조금만 틀려도 벽에 닿는다. 그래서 경계에서 prefer m 안의 칸은
지나는 비용을 높여 문·통로는 가운데로, 넓은 곳은 벽에서 떨어져 지나게 하고, 지름길도 원래 경로보다
벽에 더 붙지 않을 때만 쓴다.
"""
from __future__ import annotations

import heapq
import itertools
import math

import numpy as np
from scipy.ndimage import distance_transform_edt, label

from .voxel_map import VoxelMap

_MOVES = [(dx, dy, dz) for dx in (-1, 0, 1) for dy in (-1, 0, 1) for dz in (-1, 0, 1) if (dx, dy, dz) != (0, 0, 0)]
_COST = [math.sqrt(dx * dx + dy * dy + dz * dz) for dx, dy, dz in _MOVES]
# 대각 이동이 모서리를 자르지 않게: 지금 칸과 다음 칸이 이루는 상자 안의 다른 칸(옆 칸)도 비어야 지난다. 전에는 옆 두 칸이 막혀도
# 대각으로 건너, 같은 코드의 직선 검사(VoxelMap.segment_free)가 지나갈 수 없다고 보는 경로를 돌려주었다 (11/04 외부 리뷰)
_SIDES = [[(ax, ay, az) for ax in {0, dx} for ay in {0, dy} for az in {0, dz} if (ax, ay, az) not in ((0, 0, 0), (dx, dy, dz))]
          for dx, dy, dz in _MOVES]

PREFER = 0.5      # 부풀린 경계에서 이만큼까지는 멀수록 싸다 (m): 복도·통로 한가운데를 지난다. 0.2였을 때는 경계 0.2 m 밖은 다 같은
#                   값이라 1.2–1.6 m 복도에서 벽 쪽 0.3 m를 지났다 (11/03 실기체 요청 '복도 한가운데로'). 0.5면 좁은 통로에서 가운데가
#                   가장자리보다 25–40% 싸고, 넓은 곳에서는 경계 0.5 m 밖은 같은 값이라 길이 괜히 길어지지 않는다
#                   11/14 실기체 끝 지도 14판(실제 지나간 두 자리 40쌍)으로 다시 재 봄: 0.8·1.0·1.5로 늘리거나 1.2–1.5 m까지 완만한
#                   꼬리를 더해도 벽까지 거리가 달라지지 않았다(늘리면 기울기가 완만해져 가운데와 가장자리 값 차이가 준다). 그대로 둔다
NEAR_COST = 2.0   # 경계에 바로 붙은 칸을 지날 때 늘어나는 비용 배율 (멀어질수록 0으로 줄어든다)
CLIMB_COST = 2.0  # 오르내리는 칸의 비용 배율. 넓은 곳을 찾아 괜히 높이를 바꾸지 않고 비행 높이를 지키게
KEEP_HEIGHT = 0.2  # 먼저 출발·도착 높이 ± 이 안에서 길을 찾는다. 위치 추정의 지도 보정이 비행 높이 단면
#                    기준이라, 낮은 가구 위가 넓다고 1.35 m로 올라가면 ToF 보정이 끊긴다 (2단계 seed 5)


class PlanError(RuntimeError):
    pass


def _band(vmap: VoxelMap, z_min: float, z_max: float) -> tuple[int, int]:
    k_lo = max(0, int(math.floor((z_min - vmap.origin[2]) / vmap.res)))
    k_hi = min(vmap.occ.shape[2] - 1, int(math.floor((z_max - vmap.origin[2]) / vmap.res)))
    if k_hi < k_lo:
        raise PlanError("비행 높이 범위가 지도 밖입니다.")
    return k_lo, k_hi


def _nearest_free(vmap: VoxelMap, idx, k_lo: int, k_hi: int, reach: int = 4):
    """idx가 막혀 있으면 reach칸 안에서 가장 가까운 빈 칸을 찾는다 (벽 가까이 출발할 때)."""
    best, best_d = None, None
    for dx in range(-reach, reach + 1):
        for dy in range(-reach, reach + 1):
            for dz in range(-reach, reach + 1):
                c = (idx[0] + dx, idx[1] + dy, idx[2] + dz)
                if not (vmap.inside(c) and k_lo <= c[2] <= k_hi) or vmap.occ[c]:
                    continue
                d = dx * dx + dy * dy + dz * dz
                if best_d is None or d < best_d:
                    best, best_d = c, d
    return best


def clearance_field(inflated: VoxelMap, k_lo: int | None = None, k_hi: int | None = None) -> np.ndarray:
    """칸마다 같은 높이에서 가장 가까운 막힌 칸까지의 수평 거리 (m, 칸 중심 기준). 배열 모양은 지도와 같다.

    막힌 칸은 0, 경계에 붙은 빈 칸은 res. 높이는 하방 센서로 정확히 재므로 수평 여유만 본다 (높이마다 2D 거리).
    k_lo·k_hi를 주면 그 높이 칸 사이 단면만 구하고 나머지 단면은 inf로 둔다: 11/03 속도 — 200×200×25 전체 3D 거리 변환은
    계획마다 83 ms인데 A*는 keep_height 띠(5단면)만 보므로 단면마다 2D 거리 변환을 하면 9 ms. 막힌 칸이 없는 단면은
    전체 계산과 같게 1e3·res(지도 전체가 비었으면 inf).
    """
    if not inflated.occ.any():
        return np.full(inflated.occ.shape, np.inf)
    nz = inflated.occ.shape[2]
    k_lo = 0 if k_lo is None else max(0, int(k_lo))
    k_hi = nz - 1 if k_hi is None else min(nz - 1, int(k_hi))
    out = np.full(inflated.occ.shape, np.inf)
    for k in range(k_lo, k_hi + 1):
        plane = inflated.occ[:, :, k]
        out[:, :, k] = np.minimum(distance_transform_edt(~plane), 1e3) * inflated.res if plane.any() else 1e3 * inflated.res
    return out


def _clear_band(inflated: VoxelMap, start, goal, z_min: float, z_max: float) -> tuple[int, int]:
    """clearance_field를 구할 높이 칸 범위: A*가 보는 띠(z_min–z_max)에 출발·도착점의 칸을 더한 것.

    shortcut은 경로 첫·끝 점(띠 밖 높이일 수 있는 출발·도착점 그대로)의 여유도 보므로 그 칸까지 넣는다.
    """
    k_lo, k_hi = _band(inflated, z_min, z_max)
    nz = inflated.occ.shape[2]
    for pt in (start, goal):
        k = min(max(inflated.index(pt)[2], 0), nz - 1)
        k_lo, k_hi = min(k_lo, k), max(k_hi, k)
    return k_lo, k_hi


def _ends(inflated: VoxelMap, start, goal, k_lo: int, k_hi: int):
    """높이 띠 안으로 눌러 넣은 출발·도착 칸 (막혔으면 가장 가까운 빈 칸). 없으면 PlanError."""
    def cell(point):
        idx = inflated.index(point)
        idx = (idx[0], idx[1], min(max(idx[2], k_lo), k_hi))
        return idx if inflated.inside(idx) and not inflated.occ[idx] else _nearest_free(inflated, idx, k_lo, k_hi)

    s_free, g_free = cell(start), cell(goal)
    if s_free is None:
        raise PlanError("출발점 주변에 빈 칸이 없습니다.")
    if g_free is None:
        raise PlanError("목적지 주변에 빈 칸이 없습니다.")
    return s_free, g_free


def _reachable(inflated: VoxelMap, s_free, g_free, k_lo: int, k_hi: int) -> bool:
    """두 빈 칸이 높이 띠 안에서 이어져 있는가 (몇 ms).

    A*는 축 방향으로는 다음 칸만, 대각으로는 사이에 낀 옆 칸까지 다 비어야 지나므로, 갈 수 있는 곳은 빈 칸의 6방향 연결 덩어리와
    똑같다. 길이 없을 때 A*는 닿는 칸을 다 뒤진 뒤에야 실패한다 — 실기체 11/06 14:35 한 번에 0.8–1.4초, 11/11 09:32 0.6–2.2초
    (그동안 위치·ToF 처리가 밀려 제자리에 섰다). 11/09에 넣은 평면 검사(어느 높이든 빈 칸을 평면에 모음)는 층끼리 안 이어진
    지도(가짜 바닥 위 1.35 m 층만 열림)에서 통과시켜 A*가 그대로 다 뒤졌다. 3D 덩어리로 정확히 본다.
    """
    labels, _ = label(~inflated.occ[:, :, k_lo:k_hi + 1])
    a = labels[s_free[0], s_free[1], s_free[2] - k_lo]
    return bool(a != 0 and a == labels[g_free[0], g_free[1], g_free[2] - k_lo])


def _near_mult(clear, res: float, prefer: float):
    """경계까지 거리 clear(m, 배열)인 칸을 지날 때의 비용 배율: 경계에 붙은 칸(res) 1 + NEAR_COST, prefer 넘게 떨어지면 1."""
    return 1.0 + NEAR_COST * np.clip(1.0 - (np.asarray(clear, dtype=float) - res) / prefer, 0.0, 1.0)


def astar(inflated: VoxelMap, start, goal, z_min: float = 0.4, z_max: float = 1.8,
          weight: float = 1.3, max_expansions: int = 3_000_000, prefer: float = 0.0,
          clear: np.ndarray | None = None, climb: float = 1.0, checked: bool = False) -> list[np.ndarray]:
    """start→goal 경로(월드 좌표 점 목록). 높이는 z_min~z_max 안에서만 움직인다.

    weight > 1이면 최단 경로보다 조금 길 수 있지만 훨씬 빨리 찾는다.
    prefer > 0이면 부풀린 경계에서 prefer m 안의 칸을 지나는 비용을 높인다 (clear: clearance_field 결과. 높이 띠
    k_lo–k_hi 단면만 구한 것이어도 된다).
    climb > 1이면 오르내리는 칸의 비용을 그만큼 높인다.
    checked: 부른 쪽(plan)이 이미 이어져 있는지 봤으면 True (덩어리 검사를 다시 하지 않는다).
    """
    k_lo, k_hi = _band(inflated, z_min, z_max)
    nx, ny, _ = inflated.occ.shape
    nzb = k_hi - k_lo + 1
    s_free, g_free = _ends(inflated, start, goal, k_lo, k_hi)
    if not checked and not _reachable(inflated, s_free, g_free, k_lo, k_hi):
        raise PlanError("목적지까지 갈 수 있는 길이 없습니다.")
    blocked = np.ascontiguousarray(inflated.occ[:, :, k_lo:k_hi + 1]).ravel().tobytes()
    costs = [c * (climb if dz else 1.0) for (_, _, dz), c in zip(_MOVES, _COST)]
    near = None
    if prefer > 0:
        if clear is None:
            clear = clearance_field(inflated, k_lo, k_hi)
        near = _near_mult(clear[:, :, k_lo:k_hi + 1], inflated.res, prefer).ravel().tolist()

    def key(i, j, k):
        return (i * ny + j) * nzb + (k - k_lo)

    side_offs = [tuple((ax * ny + ay) * nzb + az for ax, ay, az in sides) for sides in _SIDES]   # 옆 칸 (평면 배열 차이)

    gi, gj, gk = g_free
    start_key, goal_key = key(*s_free), key(*g_free)
    best = {start_key: 0.0}
    parent = {start_key: None}
    tie = itertools.count()
    heap = [(0.0, next(tie), s_free)]
    closed = set()
    expansions = 0
    while heap:
        _, _, cur = heapq.heappop(heap)
        ck = key(*cur)
        if ck in closed:
            continue
        if ck == goal_key:
            break
        closed.add(ck)
        expansions += 1
        if expansions > max_expansions:
            raise PlanError("탐색 한도를 넘었습니다.")
        ci, cj, cz = cur
        base = best[ck]
        for (dx, dy, dz), step, sides in zip(_MOVES, costs, side_offs):
            ni, nj, nk = ci + dx, cj + dy, cz + dz
            if not (0 <= ni < nx and 0 <= nj < ny and k_lo <= nk <= k_hi):
                continue
            nkey = key(ni, nj, nk)
            if blocked[nkey] or nkey in closed:
                continue
            cut = False
            for o in sides:   # 옆 칸은 지금 칸과 다음 칸 사이라 늘 지도·높이 띠 안이다
                if blocked[ck + o]:
                    cut = True
                    break
            if cut:
                continue
            cost = base + (step if near is None else step * near[nkey])
            if cost < best.get(nkey, math.inf):
                best[nkey] = cost
                parent[nkey] = ck
                h = math.sqrt((ni - gi) ** 2 + (nj - gj) ** 2 + (nk - gk) ** 2)
                heapq.heappush(heap, (cost + weight * h, next(tie), (ni, nj, nk)))
    else:
        raise PlanError("목적지까지 갈 수 있는 길이 없습니다.")

    keys = []
    k = goal_key
    while k is not None:
        keys.append(k)
        k = parent[k]
    keys.reverse()
    cells = [(kk // nzb // ny, (kk // nzb) % ny, kk % nzb + k_lo) for kk in keys]
    points = [inflated.center(c) for c in cells]
    # 실제 출발·도착점은 칸 중심 앞뒤에 잇는다(이어도 될 때만). 전에는 첫·끝 칸 중심을 실제 점으로 바꿔, 칸 안 구석의 점에서 옆 칸
    # 중심으로 가는 선분이 막힌 모서리를 지날 수 있었다. 칸 중심끼리는 위 옆 칸 검사로 이미 지날 수 있다
    p0, pg = np.asarray(start, dtype=float), np.asarray(goal, dtype=float)
    if not inflated.occupied_at(p0) and np.linalg.norm(p0 - points[0]) > 1e-6 and inflated.segment_free(p0, points[0]):
        points.insert(0, p0)
    if not inflated.occupied_at(pg) and np.linalg.norm(pg - points[-1]) > 1e-6 and inflated.segment_free(points[-1], pg):
        points.append(pg)
    return points


def _clear_along(inflated: VoxelMap, clear: np.ndarray, a, b) -> float:
    """a→b 직선이 지나는 칸들의 clearance 최솟값 (지도 밖은 0)."""
    a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    steps = max(1, int(math.ceil(np.linalg.norm(b - a) / (inflated.res * 0.5))))
    pts = a + (b - a) * np.linspace(0.0, 1.0, steps + 1)[:, None]
    idx = np.floor((pts - inflated.origin) / inflated.res).astype(int)
    if (idx < 0).any() or (idx >= np.array(clear.shape)).any():
        return 0.0
    return float(clear[idx[:, 0], idx[:, 1], idx[:, 2]].min())


def _line_cost(inflated: VoxelMap, clear: np.ndarray, a, b, prefer: float) -> float:
    """a→b 직선을 A*와 같은 잣대로 잰 비용: 길이 × 지나는 칸의 경계 가까움 배율 (지도 밖이면 inf)."""
    a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    length = float(np.linalg.norm(b - a))
    if length < 1e-9:
        return 0.0
    steps = max(1, int(math.ceil(length / (inflated.res * 0.5))))
    pts = a + (b - a) * ((np.arange(steps) + 0.5) / steps)[:, None]
    idx = np.floor((pts - inflated.origin) / inflated.res).astype(int)
    if (idx < 0).any() or (idx >= np.array(clear.shape)).any():
        return math.inf
    return float(_near_mult(clear[idx[:, 0], idx[:, 1], idx[:, 2]], inflated.res, prefer).sum()) * length / steps


def shortcut(inflated: VoxelMap, path: list[np.ndarray], clear: np.ndarray | None = None,
             prefer: float = 0.0, by_cost: bool = True) -> list[np.ndarray]:
    """곧장 갈 수 있는 점끼리 이어 경유점 수를 줄인다.

    clear가 있으면 원래 경로보다 경계에 더 붙는 지름길은 쓰지 않는다 (문 가운데를 지나던 경로가
    문틀을 스치는 직선으로 바뀌지 않게). 경계에서 prefer m 넘게 떨어진 곳끼리는 자유롭게 잇는다.
    by_cost: 그리고 지름길의 비용(길이 × 경계 가까움 배율, A*와 같은 잣대)이 원래 구간보다 크면 쓰지 않는다. 위 규칙은 구간에서
    가장 좁은 곳만 비교해, 출발점이 벽 가까이면(둘러본 자리·떼어 낸 자리) A*가 가운데로 돌아간 길을 벽을 따라가는 직선으로
    되돌렸다 (11/14). 실기체 끝 지도 14판의 40쌍: 벽 0.9 m 안을 지나는 길이 49% → 42%, 0.7 m 안 20% → 16%, 경로 길이 +0.6%,
    경유점 4.5 → 5.2개.
    """
    if len(path) <= 2:
        return list(path)
    cum = None
    if clear is not None:
        cap = prefer + inflated.res
        own = np.array([_clear_along(inflated, clear, p, p) for p in path])
        if by_cost and prefer > 0:
            cum = np.concatenate([[0.0], np.cumsum([_line_cost(inflated, clear, p, q, prefer) for p, q in zip(path, path[1:])])])

    def ok(i: int, j: int) -> bool:
        if not inflated.segment_free(path[i], path[j]):
            return False
        if clear is None:
            return True
        need = min(cap, float(own[i:j + 1].min())) - inflated.res / 2
        if _clear_along(inflated, clear, path[i], path[j]) < need:
            return False
        return cum is None or _line_cost(inflated, clear, path[i], path[j], prefer) <= cum[j] - cum[i] + 1e-9

    out = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not ok(i, j):
            j -= 1
        out.append(path[j])
        i = j
    return out


def plan(inflated: VoxelMap, start, goal, prefer: float = PREFER, climb: float = CLIMB_COST,
         keep_height: float | None = KEEP_HEIGHT, by_cost: bool = True, **kwargs) -> list[np.ndarray]:
    """start→goal 경로. keep_height가 있으면 좁은 높이에서 먼저 찾고, 길이 없으면 넓힌다:
    ① 목적지 높이 한 층(2D) — 출발점에서 그 높이까지 곧게 오르내릴 수 있을 때 ② 출발·도착 높이 ± keep_height 띠 ③ 전체 높이.

    ①은 11/11에 넣었다: 가구를 넘느라 1.4 m에 떠 있으면 ②의 띠(0.8–1.6 m)에서는 오르내림 비용이 어디서나 같아 경로가 끝까지
    1.4 m에 머물렀다(09:32 실기체: 복도를 1.35–1.45 m로 날았다). 한 층만 보면 8방향·칸 수 1/5이라 A*도 열 배쯤 빠르다.
    단계마다 먼저 이어져 있는지 보고(_reachable, 몇 ms) 아니면 여유 계산·A* 없이 다음 단계로 간다.
    경계 여유(clearance_field)는 그때그때 A*가 보는 높이 띠만 구한다 (11/03: 전체 25단면 83 ms → 띠 5단면 9 ms).
    """
    z_min, z_max = kwargs.get("z_min", 0.4), kwargs.get("z_max", 1.8)
    bands = []
    if keep_height is not None:
        gz = float(goal[2])
        level = (float(start[0]), float(start[1]), gz)
        # 높이 차가 크면(가구 위에 떠 있음) 바로 밑이 그 높이까지 비어 있을 때만: 아니면 가구 속으로 비스듬히 내려가는 길이 된다
        if z_min <= gz <= z_max and (abs(float(start[2]) - gz) <= keep_height
                                     or (not inflated.occupied_at(level) and inflated.segment_free(start, level))):
            bands.append((gz, gz))
        bands.append((max(z_min, min(start[2], goal[2]) - keep_height), min(z_max, max(start[2], goal[2]) + keep_height)))
    bands.append((z_min, z_max))
    err = None
    for lo, hi in bands:
        try:
            k_lo, k_hi = _band(inflated, lo, hi)
            if not _reachable(inflated, *_ends(inflated, start, goal, k_lo, k_hi), k_lo, k_hi):
                raise PlanError("목적지까지 갈 수 있는 길이 없습니다.")
            clear = clearance_field(inflated, *_clear_band(inflated, start, goal, lo, hi)) if prefer > 0 else None
            path = astar(inflated, start, goal, prefer=prefer, clear=clear, climb=climb, checked=True, **dict(kwargs, z_min=lo, z_max=hi))
            path = _level(inflated, shortcut(inflated, path, clear, prefer, by_cost), float(goal[2]))
            return _checked(inflated, _hold_height(inflated, path, float(goal[2])))
        except PlanError as e:
            err = e
    raise err


def _checked(inflated: VoxelMap, path: list[np.ndarray]) -> list[np.ndarray]:
    """돌려주기 전에 모든 선분을 직선 검사로 다시 본다 (11/04 외부 리뷰). 출발점이 막힌 칸 안이면(장애물 옆, 부풀린 여유 안)
    첫 점은 가장 가까운 빈 칸 중심이라 그 앞 선분은 경로에 없다 — 빠져나오는 길은 따라가기가 맡는다."""
    for a, b in zip(path, path[1:]):
        if not inflated.segment_free(a, b):
            raise PlanError(f"경로 검사 실패: ({a[0]:.2f}, {a[1]:.2f}, {a[2]:.2f}) → ({b[0]:.2f}, {b[1]:.2f}, {b[2]:.2f})가 막힌 칸을 지납니다.")
    return path


def _level(inflated: VoxelMap, path: list[np.ndarray], height: float) -> list[np.ndarray]:
    """경유점을 갈 수 있으면 목적지 높이로 맞춘다 (앞에서부터 차례로: 맞춘 점과 앞뒤 선분이 다 비어 있을 때).

    0.1 m 칸 중심을 그대로 쓰면 비행 높이 1.0 m 대신 0.95 m로 날아, 지도에 없는 0.96 m 의자에 닿았다. 전에는 한 칸(0.1 m) 안의
    차이만 맞췄는데, 높이 떠서 출발한 경로가 끝까지 그 높이에 머물러(11/11 09:32) 차이가 커도 맞춘다 — 가구 위를 넘는 구간은
    맞춘 점이 막혀 있어 그대로 남는다.
    """
    out = [np.asarray(p, dtype=float).copy() for p in path]
    for i in range(1, len(out) - 1):
        q = out[i].copy()
        q[2] = height
        if (not inflated.occupied_at(q) and inflated.segment_free(out[i - 1], q)
                and inflated.segment_free(q, out[i + 1])):
            out[i] = q
    return out


RAMP = 0.5        # 넘어야 하는 곳 앞뒤로 오르내리는 기울기 (높이 ÷ 수평 거리): 0.35 m를 오르는 데 0.7 m
RAMP_STEP = 0.2   # 경로 높이를 다시 정할 때 경로를 나누는 간격 (m)


def _hold_height(inflated: VoxelMap, path: list[np.ndarray], height: float) -> list[np.ndarray]:
    """경로의 (x, y)는 그대로 두고 높이만 다시 정한다: 비행 높이로 갈 수 있는 곳은 비행 높이로 날고, 넘어야 하는 곳 앞뒤로만
    RAMP 기울기로 오르내린다 (가구 위로 넘을 때만 잠깐 오른다).

    A*는 오르내림 비용이 어디서나 같아 출발하자마자 오르고 끝에 가서야 내려오는 경로를 낸다. Gazebo 상자 넘기(11/11)에서 0.7 m 상자
    3.5 m 앞에서부터 1.35 m로 날았고, 09:32 실기체는 1.4 m로 뜬 뒤 계속 그 높이로 날았다(높이 날수록 단안 깊이가 바닥을 높게 읽는다).
    - 경로를 RAMP_STEP마다 나눠, 비행 높이 칸이 빈 곳이 (양옆 높이 차 ÷ RAMP + 0.4 m)보다 길게 이어지면 그 구간을 비행 높이로 내린다
      (가구 사이 짧은 틈에서는 오르내리지 않는다).
    - 내린 구간과 높은 구간 사이를 RAMP 기울기로 잇는다(높은 쪽을 넓히는 방향으로만).
    - 새 선분이 하나라도 막히면(오르내리는 비탈이 모르는 칸·막힌 칸을 지나면) 원래 경로를 그대로 돌려준다.
    """
    pts = [np.asarray(p, dtype=float) for p in path]
    if len(pts) < 2 or not any(p[2] > height + 1e-6 for p in pts):
        return list(path)
    samples = [pts[0]]
    for a, b in zip(pts, pts[1:]):
        n = max(1, int(math.ceil(float(np.linalg.norm(b[:2] - a[:2])) / RAMP_STEP)))
        samples.extend(a + (b - a) * i / n for i in range(1, n + 1))
    xy = [s[:2] for s in samples]
    z = [float(s[2]) for s in samples]
    last = len(samples) - 1
    gap = [float(np.linalg.norm(b - a)) for a, b in zip(xy, xy[1:])]
    # 비행 높이 칸이 빈 곳. 막힌 곳 바로 옆 한 칸(RAMP_STEP)은 내리지 않는다: 비탈이 막힌 기둥에 들어서기 전에 다 올라와 있어야 한다
    free = [not inflated.occupied_at((p[0], p[1], height)) for p in xy]
    low = [0 < i < last and z[i] > height + 1e-6 and free[i - 1] and free[i] and free[i + 1] for i in range(len(samples))]
    i = 1
    while i < last:
        if not low[i]:
            i += 1
            continue
        j = i
        while j + 1 < last and low[j + 1]:
            j += 1
        need = (max(z[i - 1] - height, 0.0) + max(z[j + 1] - height, 0.0)) / RAMP + 0.4
        if sum(gap[i - 1:j + 1]) >= need:
            for k in range(i, j + 1):
                z[k] = height
        i = j + 1
    for i in range(1, last):                      # 앞에서부터: 높은 곳에서 내려오는 비탈
        z[i] = max(z[i], z[i - 1] - RAMP * gap[i - 1])
    for i in range(last - 1, 0, -1):              # 뒤에서부터: 높은 곳으로 오르는 비탈
        z[i] = max(z[i], z[i + 1] - RAMP * gap[i])
    new = [np.array([xy[i][0], xy[i][1], z[i]]) for i in range(len(samples))]
    if any(not inflated.segment_free(a, b) for a, b in zip(new, new[1:])):
        return list(path)
    out = [new[0]]
    for i in range(1, last):                      # 곧게 이어지는 점은 뺀다
        u, v = new[i] - out[-1], new[i + 1] - new[i]
        nu, nv = float(np.linalg.norm(u)), float(np.linalg.norm(v))
        if nu < 1e-9:
            continue
        if nv < 1e-9 or float(np.linalg.norm(np.cross(u / nu, v / nv))) > 1e-3 or float(u @ v) < 0:   # 0.06° 넘게 꺾이는 점만 남긴다
            out.append(new[i])
    out.append(new[last])
    return out


def farthest_point(inflated: VoxelMap, start, height: float) -> np.ndarray:
    """height 높이 단면에서 start로부터 갈 수 있는 가장 먼 빈 칸 (목적지가 없는 지도에서 쓴다)."""
    k = inflated.index((0, 0, height))[2]
    k_lo, k_hi = _band(inflated, height, height)
    s = inflated.index((start[0], start[1], height))
    s = s if inflated.inside(s) and not inflated.occ[s] else _nearest_free(inflated, s, k_lo, k_hi)
    if s is None:
        raise PlanError("출발점 주변에 빈 칸이 없습니다.")
    plane = inflated.occ[:, :, k]
    nx, ny = plane.shape
    dist = {(s[0], s[1]): 0}
    queue = [(s[0], s[1])]
    for i, j in queue:   # 너비 우선 탐색 (리스트를 큐처럼 쓴다)
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            n = (i + di, j + dj)
            if 0 <= n[0] < nx and 0 <= n[1] < ny and n not in dist and not plane[n]:
                dist[n] = dist[(i, j)] + 1
                queue.append(n)
    far = max(dist, key=dist.get)
    return inflated.center((far[0], far[1], k))


def path_length(path: list[np.ndarray]) -> float:
    return float(sum(np.linalg.norm(b - a) for a, b in zip(path, path[1:])))
