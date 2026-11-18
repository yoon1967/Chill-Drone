"""사람 동선: 자율 비행 지도(벽·장애물) 위에서 대원이 걸어갈 길을 찾는다(표준 라이브러리만).

지도는 자율 비행 보기 게이트웨이의 `autonomy_grid()`(비행 노드 /tt/obstacle_grid를 줄인 것)다.
칸 코드 0 모름·1 본 곳·2 의심·3 막힘, 한 칸 res m, 좌표 map(켠 자리 원점, x 앞·y 왼쪽, m).

- 막힘·찾은 장애물 네모에서 사람 몸 반 폭(WIDTH_M/2) 안은 못 지나간다(칸 가운데에서 막힌 칸 가장자리까지).
  0.1 m 칸이면 몸 폭 0.6 m로 문은 0.7 m부터 지나가고 0.6 m 아래는 막힌다.
- 막힘에서 SOFT_M 안은 비용을 더 쳐서 복도 가운데로 걷게 한다.
- 의심 칸은 그 칸만 못 지나가고 둘레에 비용을 조금 더한다(비행 노드도 경로에서 피한다).
- 모르는 칸(못 본 곳)은 지나가지 않는다. 지나가게 하면 건물 밖 못 본 곳으로 도는 길이 나왔다(11/17 시험).
- 출발·도착이 못 지나가는 칸이면 SNAP_M 안의 가장 가까운 칸으로 옮긴다(서 있는 사람은 지도에서 막힘으로 찍힌다).
- 길이 없으면 갈 수 있는 칸 가운데 도착에 가장 가까운 곳까지 가고 남은 거리(gap_m)를 알린다.
- 8방향 A*(벽 모서리는 자르지 않음)로 찾고, 서로 보이는 점끼리 곧게 펴서 꺾이는 점만 남긴다.

드론 비행 높이(약 1 m) 단면 지도라 바닥의 낮은 물체·계단·구멍은 없을 수 있다. 결과는 추정 동선이다.
"""

from __future__ import annotations

import base64
import binascii
import heapq
import math
from collections import deque

WIDTH_M = 0.6          # 사람 몸 폭(보호 장비를 입은 대원 1명)
SOFT_M = 0.7           # 막힘에서 이 거리 안은 비용을 더 친다(복도 가운데로)
SOFT_WEIGHT = 3.0      # 벽에 붙을수록 더하는 비용(곧게 1배 → 벽 바로 옆 1+3배)
SUSPECT_WEIGHT = 1.5   # 의심 칸 둘레에 더하는 비용
WALK_MPS = 0.5         # 걸리는 시간 기준: 조심 걸음
SNAP_M = 1.0           # 출발·도착을 옮겨도 되는 거리
STRAIGHT_MAX = 1.6     # 곧게 펼 때 지나도 되는 비용 배수(벽에 붙어 펴지지 않게)
MAX_CELLS = 250_000    # 이보다 큰 지도는 계산하지 않는다(500×500칸, 게이트웨이 스레드를 오래 잡지 않게)

UNKNOWN, SEEN, SUSPECT, BLOCKED = 0, 1, 2, 3
SQRT2 = math.sqrt(2.0)
STEPS = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
         (1, 1, SQRT2), (1, -1, SQRT2), (-1, 1, SQRT2), (-1, -1, SQRT2))


def _number(value) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError("number")
    return float(value)


class CostMap:
    """지도 한 판의 걷는 비용. 같은 지도 판이면 동선 여러 개가 같이 쓴다."""

    def __init__(self, grid: dict, discovered: dict | None = None, *, width_m: float = WIDTH_M):
        try:
            self.w, self.h = int(grid["w"]), int(grid["h"])
            self.res, self.x0, self.y0 = _number(grid["res"]), _number(grid["x0"]), _number(grid["y0"])
            codes = base64.b64decode(grid["cells"], validate=True)
        except (KeyError, TypeError, ValueError, binascii.Error) as e:
            raise ValueError(f"invalid_grid: {e}") from None
        if self.w <= 0 or self.h <= 0 or self.res <= 0 or len(codes) != self.w * self.h:
            raise ValueError("invalid_grid: size")
        if self.w * self.h > MAX_CELLS:
            raise ValueError("grid_too_big")
        self.codes = codes
        self.ver = grid.get("ver")
        n = self.w * self.h
        blocked = {k for k in range(n) if codes[k] == BLOCKED}
        for box in (discovered or {}).get("obstacles") or []:   # 비행 노드가 찾은, 지도에 없던 장애물
            try:
                x, y, sx, sy = (_number(box[k]) for k in ("x", "y", "sx", "sy"))
            except (KeyError, TypeError, ValueError):
                continue
            blocked.update(self._cells_in(x - sx / 2, y - sy / 2, x + sx / 2, y + sy / 2))
        suspect = [k for k in range(n) if codes[k] == SUSPECT]
        res = self.res
        # 칸 가운데에서 막힌 칸 가장자리까지 = 칸 거리 × res − res/2. 몸 반 폭보다 가까우면 못 지나간다(0.3/0.1 소수 오차는 ε로)
        hard = (width_m / 2 + res / 2) / res - 1e-6
        soft = (SOFT_M + res / 2) / res
        wall = _distance(self.w, self.h, blocked, soft + 1)
        near_suspect = _distance(self.w, self.h, suspect, soft + 1) if suspect else None
        cost: list[float | None] = [None] * n
        for k in range(n):
            if codes[k] != SEEN or k in blocked or wall[k] < hard:
                continue
            m = 1.0
            if wall[k] < soft:
                m += SOFT_WEIGHT * (soft - wall[k]) / (soft - hard)
            if near_suspect is not None and near_suspect[k] < soft:
                m += SUSPECT_WEIGHT * (soft - near_suspect[k]) / soft
            cost[k] = m
        self.cost = cost

    # ---------- 좌표 ----------
    def cell(self, x: float, y: float) -> int | None:
        i, j = math.floor((x - self.x0) / self.res), math.floor((y - self.y0) / self.res)
        return j * self.w + i if 0 <= i < self.w and 0 <= j < self.h else None

    def xy(self, k: int) -> tuple[float, float]:
        return self.x0 + (k % self.w + 0.5) * self.res, self.y0 + (k // self.w + 0.5) * self.res

    def _cells_in(self, xa: float, ya: float, xb: float, yb: float) -> list[int]:
        i0, i1 = max(0, math.floor((xa - self.x0) / self.res)), min(self.w - 1, math.floor((xb - self.x0) / self.res))
        j0, j1 = max(0, math.floor((ya - self.y0) / self.res)), min(self.h - 1, math.floor((yb - self.y0) / self.res))
        return [j * self.w + i for j in range(j0, j1 + 1) for i in range(i0, i1 + 1)]

    # ---------- 찾기 ----------
    def snap(self, x: float, y: float) -> tuple[int | None, float]:
        """(x, y)에서 SNAP_M 안의 가장 가까운 지나갈 수 있는 칸과 옮긴 거리. 없으면 (None, 0)."""
        k = self.cell(x, y)
        if k is not None and self.cost[k] is not None:
            return k, 0.0
        best, best_d = None, SNAP_M
        r = math.ceil(SNAP_M / self.res) + 1
        ci, cj = math.floor((x - self.x0) / self.res), math.floor((y - self.y0) / self.res)
        for j in range(max(0, cj - r), min(self.h, cj + r + 1)):
            for i in range(max(0, ci - r), min(self.w, ci + r + 1)):
                c = j * self.w + i
                if self.cost[c] is None:
                    continue
                d = math.dist((x, y), self.xy(c))
                if d <= best_d:
                    best, best_d = c, d
        return best, (best_d if best is not None else 0.0)

    def astar(self, s: int, t: int) -> list[int] | None:
        w, cost = self.w, self.cost
        ti, tj = t % w, t // w

        def guess(k):   # 옥타일 거리(비용 배수는 1 이상)
            dx, dy = abs(k % w - ti), abs(k // w - tj)
            return dx + dy + (SQRT2 - 2) * min(dx, dy)

        best = {s: 0.0}
        came: dict[int, int] = {}
        todo = [(guess(s), 0.0, s)]
        done = set()
        while todo:
            _, g, k = heapq.heappop(todo)
            if k in done:
                continue
            if k == t:
                path = [k]
                while k in came:
                    k = came[k]
                    path.append(k)
                return path[::-1]
            done.add(k)
            i, j = k % w, k // w
            for di, dj, step in STEPS:
                ni, nj = i + di, j + dj
                if not (0 <= ni < w and 0 <= nj < self.h):
                    continue
                nk = nj * w + ni
                if cost[nk] is None or nk in done:
                    continue
                if di and dj and (cost[j * w + ni] is None or cost[nj * w + i] is None):
                    continue   # 벽 모서리를 대각선으로 자르지 않는다
                ng = g + step * (cost[k] + cost[nk]) / 2
                if ng < best.get(nk, math.inf):
                    best[nk] = ng
                    came[nk] = k
                    heapq.heappush(todo, (ng + guess(nk), ng, nk))
        return None

    def nearest_reachable(self, s: int, x: float, y: float) -> int:
        """s에서 갈 수 있는 칸 가운데 (x, y)에 가장 가까운 칸."""
        w, cost = self.w, self.cost
        seen, todo = {s}, deque([s])
        best, best_d = s, math.dist(self.xy(s), (x, y))
        while todo:
            k = todo.popleft()
            d = math.dist(self.xy(k), (x, y))
            if d < best_d:
                best, best_d = k, d
            i, j = k % w, k // w
            for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                ni, nj = i + di, j + dj
                if 0 <= ni < w and 0 <= nj < self.h:
                    nk = nj * w + ni
                    if nk not in seen and cost[nk] is not None:
                        seen.add(nk)
                        todo.append(nk)
        return best

    def straighten(self, path: list[int]) -> list[int]:
        """줄 당기기: 지금 점에서 곧게 보이는 가장 먼 점으로 건너뛴다(벽 옆 비싼 칸을 지나면 펴지 않는다)."""
        if len(path) < 3:
            return path
        out, i = [path[0]], 0
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i + 1 and not self._clear(path[i], path[j]):
                j -= 1
            out.append(path[j])
            i = j
        return out

    def _clear(self, a: int, b: int) -> bool:
        w, cost = self.w, self.cost
        limit = max(STRAIGHT_MAX, cost[a] + 0.01, cost[b] + 0.01)
        ai, aj, bi, bj = a % w, a // w, b % w, b // w
        n = max(abs(bi - ai), abs(bj - aj)) * 2 + 1
        for s in range(n + 1):
            i = round(ai + (bi - ai) * s / n)
            j = round(aj + (bj - aj) * s / n)
            c = cost[j * w + i]
            if c is None or c > limit:
                return False
        return True


def _distance(w: int, h: int, sources, limit: float) -> list[float]:
    """sources 칸에서의 거리(칸 수, 곧게 1·대각선 √2), limit 넘으면 그만 센다(그 너머는 inf)."""
    dist = [math.inf] * (w * h)
    todo = []
    for k in sources:
        dist[k] = 0.0
        todo.append((0.0, k))
    heapq.heapify(todo)
    while todo:
        d, k = heapq.heappop(todo)
        if d > dist[k] or d > limit:
            continue
        i, j = k % w, k // w
        for di, dj, step in STEPS:
            ni, nj = i + di, j + dj
            if 0 <= ni < w and 0 <= nj < h:
                nk = nj * w + ni
                nd = d + step
                if nd < dist[nk]:
                    dist[nk] = nd
                    heapq.heappush(todo, (nd, nk))
    return dist


def plan(cmap: CostMap, start: tuple[float, float], goal: tuple[float, float], *, walk_mps: float = WALK_MPS) -> dict:
    """start에서 goal까지 사람 동선. 돌려주는 모양(board.py가 그대로 result로 싣는다):

    state ok(끝까지) · partial(갈 수 있는 데까지, gap_m = 남은 곧은 거리) · no_path(출발 둘레가 막힘·지도 밖)
    pts [[x, y], …](m, 꺾이는 점만), length_m, walk_s(walk_mps 기준), snap_start_m·snap_goal_m(옮긴 거리), reason
    """
    s, snap_s = cmap.snap(*start)
    if s is None:
        reason = "start_off_map" if cmap.cell(*start) is None else "start_blocked"
        return {"state": "no_path", "reason": reason, "pts": [], "length_m": None, "walk_s": None, "gap_m": None,
                "snap_start_m": None, "snap_goal_m": None}
    t, snap_t = cmap.snap(*goal)
    path = cmap.astar(s, t) if t is not None else None
    gap = None
    if path is None:   # 길이 없다: 도착에 가장 가까운 곳까지
        near = cmap.nearest_reachable(s, *goal)
        path = cmap.astar(s, near) if near != s else [s]
        gap = math.dist(cmap.xy(near), goal)
        snap_t = None
    if gap is not None and len(path) < 2:
        return {"state": "no_path", "reason": "boxed_in", "pts": [], "length_m": None, "walk_s": None,
                "gap_m": round(gap, 2), "snap_start_m": round(snap_s, 2), "snap_goal_m": None}
    pts = [cmap.xy(k) for k in cmap.straighten(path)]
    length = sum(math.dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1))
    return {"state": "ok" if gap is None else "partial", "reason": None if gap is None else "goal_unreachable",
            "pts": [[round(x, 2), round(y, 2)] for x, y in pts], "length_m": round(length, 2),
            "walk_s": round(length / walk_mps, 1), "gap_m": None if gap is None else round(gap, 2),
            "snap_start_m": round(snap_s, 2), "snap_goal_m": None if snap_t is None else round(snap_t, 2)}
