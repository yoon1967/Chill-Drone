"""사람 찾기 탐색(3단계): 받은 지도에서 카메라로 아직 못 본 곳을 골라 돌며 본다. ROS와 무관.

- 본 곳 지도: 바닥을 cell(기본 25 cm) 칸으로 나누고, 카메라 시야(수평 화각, 최대 거리) 안에서
  벽·가구에 가리지 않은 칸을 '봤음'으로 표시한다. 가림은 비행 높이 단면 지도로 판단한다.
- 다음 볼 자리: 후보 자리마다 (그 자리에서 둘러보면 새로 보이는 칸 수) / (1 + 가는 거리 / 비용 거리)
  가 가장 큰 곳을 고른다. 도착하면 안 본 칸이 많은 방향으로 기수를 돌린다.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

import numpy as np


@dataclass
class ExploreParams:
    cell: float = 0.25             # 본 곳 지도 칸 크기 (m)
    view_range: float = 4.0        # 사람을 찾을 수 있다고 보는 거리 (m)
    hfov: float = 1.4416           # 카메라 수평 화각 (rad)
    candidate_step: float = 0.75   # 다음 볼 자리 후보 간격 (m)
    min_gain_cells: int = 6        # 이보다 적게 새로 보이면 갈 가치가 없다 (약 0.4 m²)
    cost_distance: float = 3.0     # 이 거리만큼 더 가면 이득을 절반으로 친다
    look_dirs: int = 8             # 둘러볼 방향 후보 수


class CoverageMap:
    """카메라로 본 바닥 칸 기록."""

    def __init__(self, occ2d: np.ndarray, origin, res: float, params: ExploreParams | None = None):
        self.p = params or ExploreParams()
        self.occ = occ2d                     # 비행 높이 단면 (True = 막힘, 가림 판단용)
        self.origin = np.asarray(origin, dtype=float)
        self.res = res
        width, depth = occ2d.shape[0] * res, occ2d.shape[1] * res
        self.nx = int(math.floor(width / self.p.cell))
        self.ny = int(math.floor(depth / self.p.cell))
        ii, jj = np.meshgrid(np.arange(self.nx), np.arange(self.ny), indexing="ij")
        self.cx = self.origin[0] + (ii + 0.5) * self.p.cell
        self.cy = self.origin[1] + (jj + 0.5) * self.p.cell
        self.target = ~self._occupied(self.cx, self.cy)      # 벽·가구 속이 아닌 칸만 볼 대상
        self.seen = np.zeros_like(self.target)

    # ------------------------------------------------------------ 보기

    def visible(self, x: float, y: float, yaw: float | None = None) -> np.ndarray:
        """(x, y)에서 yaw 방향 카메라로 보이는 칸 (yaw가 None이면 360° 둘러볼 때)."""
        dx, dy = self.cx - x, self.cy - y
        dist = np.hypot(dx, dy)
        mask = self.target & (dist <= self.p.view_range)
        if yaw is not None:
            ang = np.arctan2(dy, dx) - yaw
            ang = (ang + np.pi) % (2 * np.pi) - np.pi
            mask &= (np.abs(ang) <= self.p.hfov / 2) | (dist < self.p.cell)
        idx = np.argwhere(mask)
        if len(idx) == 0:
            return mask
        tx, ty = self.cx[mask], self.cy[mask]
        steps = np.arange(0.1, self.p.view_range, 0.1)
        frac = steps[None, :] / np.maximum(dist[mask][:, None], 1e-6)
        frac = np.where(frac < 1.0, frac, np.nan)
        px = x + frac * (tx - x)[:, None]
        py = y + frac * (ty - y)[:, None]
        blocked = self._occupied(np.nan_to_num(px, nan=x), np.nan_to_num(py, nan=y)) & ~np.isnan(frac)
        clear = ~blocked.any(axis=1)
        out = np.zeros_like(mask)
        out[idx[clear, 0], idx[clear, 1]] = True
        return out

    def observe(self, x: float, y: float, yaw: float) -> int:
        """카메라로 본 칸을 기록하고, 새로 본 칸 수를 돌려준다."""
        new = self.visible(x, y, yaw) & ~self.seen
        self.seen |= new
        return int(new.sum())

    def coverage(self) -> float:
        return float(self.seen.sum() / max(self.target.sum(), 1))

    def best_look(self, x: float, y: float) -> tuple[float, int]:
        """(x, y)에서 안 본 칸이 가장 많이 보이는 방향과 그 칸 수."""
        best = (0.0, 0)
        for k in range(self.p.look_dirs):
            yaw = -math.pi + k * 2 * math.pi / self.p.look_dirs
            gain = int((self.visible(x, y, yaw) & ~self.seen).sum())
            if gain > best[1]:
                best = (yaw, gain)
        return best

    # ------------------------------------------------------------ 다음 볼 자리

    def next_viewpoint(self, x: float, y: float, free2d: np.ndarray, free_origin, free_res: float,
                       exclude=()):
        """다음에 갈 자리 (x, y, 이득 칸 수, 가는 거리) 또는 None (더 볼 곳 없음).

        free2d: 드론 크기만큼 부풀린 비행 높이 단면에서 비어 있는 칸 (갈 수 있는 곳).
        exclude: 가 보려다 길이 없었던 자리들. 0.6 m 안의 후보는 고르지 않는다.
        """
        dist = _bfs_distance(free2d, free_origin, free_res, x, y)
        step = self.p.candidate_step
        best = None
        for gx in np.arange(self.origin[0] + step / 2, self.origin[0] + self.nx * self.p.cell, step):
            for gy in np.arange(self.origin[1] + step / 2, self.origin[1] + self.ny * self.p.cell, step):
                i = int((gx - free_origin[0]) / free_res)
                j = int((gy - free_origin[1]) / free_res)
                if not (0 <= i < free2d.shape[0] and 0 <= j < free2d.shape[1]) or not math.isfinite(dist[i, j]):
                    continue
                if any(math.hypot(gx - ex, gy - ey) < 0.6 for ex, ey in exclude):
                    continue
                gain = int((self.visible(gx, gy) & ~self.seen).sum())
                if gain < self.p.min_gain_cells:
                    continue
                score = gain / (1.0 + dist[i, j] / self.p.cost_distance)
                if best is None or score > best[0]:
                    best = (score, float(gx), float(gy), gain, float(dist[i, j]))
        return None if best is None else best[1:]

    # ------------------------------------------------------------ 내부

    def _occupied(self, x, y) -> np.ndarray:
        i = np.floor((np.asarray(x) - self.origin[0]) / self.res).astype(int)
        j = np.floor((np.asarray(y) - self.origin[1]) / self.res).astype(int)
        ok = (i >= 0) & (j >= 0) & (i < self.occ.shape[0]) & (j < self.occ.shape[1])
        out = np.ones(np.shape(x), dtype=bool)
        out[ok] = self.occ[i[ok], j[ok]]
        return out


def _bfs_distance(free2d: np.ndarray, origin, res: float, x: float, y: float) -> np.ndarray:
    """(x, y)에서 갈 수 있는 칸까지의 대략 거리 (m, 8방향 BFS). 못 가면 inf."""
    nx, ny = free2d.shape
    dist = np.full((nx, ny), np.inf)
    si, sj = int((x - origin[0]) / res), int((y - origin[1]) / res)
    if not (0 <= si < nx and 0 <= sj < ny):
        return dist
    if not free2d[si, sj]:   # 벽 가까이에 붙어 있으면 가장 가까운 빈 칸에서 시작
        cand = np.argwhere(free2d)
        if len(cand) == 0:
            return dist
        k = int(np.argmin((cand[:, 0] - si) ** 2 + (cand[:, 1] - sj) ** 2))
        si, sj = int(cand[k, 0]), int(cand[k, 1])
    dist[si, sj] = 0.0
    queue = deque([(si, sj)])
    moves = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
             (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)]
    while queue:
        i, j = queue.popleft()
        d = dist[i, j]
        for di, dj, c in moves:
            ni, nj = i + di, j + dj
            if 0 <= ni < nx and 0 <= nj < ny and free2d[ni, nj] and dist[ni, nj] > d + c * res + 1e-9:
                dist[ni, nj] = d + c * res
                queue.append((ni, nj))
    return dist


VP_REPLANS = 6           # 볼 자리 하나에서 이만큼 넘게 다시 계획하면 포기한다
VP_STALL = 12.0          # 움직이는 동안(제자리 회전·물러남은 빼고) 이 시간(s) 넘게 남은 경로가 VP_PROGRESS만큼도 줄지 않으면 포기한다
VP_PROGRESS = 0.3        # (m)
VP_TICK_MAX = 0.5        # 한 번에 세는 시간 상한 (s): 다른 단계(물러남·되짚기)에 있던 시간은 세지 않는다
VP_REPLAN_GAP = 1.0      # 재계획은 이 간격(s)에 한 번까지만 센다. 계획용 지도를 0.5초마다 다시 만들자(11/04) 같은 막힘에 재계획이
#                          30–60% 늘어 Gazebo 복도 끝에서 세 자리를 연달아 포기했다. 한도 6번은 1초마다 지도를 만들던 비행 기록에서 정했다
VP_REPLANS_TIGHT = 3     # 줄인 지도 여유(좁은 길)로만 계획이 나온 자리는 더 빨리 포기한다
VP_STALL_TIGHT = 8.0


class ViewpointBudget:
    """볼 자리 하나에 쓰는 재계획 횟수·시간의 한도. 넘으면 그 자리를 포기하고 다른 곳을 고른다.

    2026-11-04 실기체: 가는 길이 가구·사람에 계속 막힌 두 자리에 28초(재계획 10번)와 56초(16번)를 써, 탐색 146초의
    57%가 닿지 못한 자리였다. 다시 계획할 때마다 경로가 길어졌다(3.7 → 6.9 m). 이 한도면 그 기록에서 약 8초와 45초 일찍 포기한다.
    """

    def __init__(self, now: float):
        self.started = now
        self.replans = 0
        self.best = math.inf      # 지금까지 가장 짧았던 남은 경로 길이 (m)
        self.stalled = 0.0        # 그 뒤로 움직이라고 한 시간 (s)
        self.last = None          # 마지막으로 tick한 시각
        self.last_replan = None   # 마지막으로 센 재계획 시각
        self.tight = False        # 줄인 여유로만 계획이 나왔다
        self.door = False         # 길이 문(좁은 틈)을 지난다: 문은 원래 좁으므로 tight 한도를 쓰지 않는다 (11/05 10:51 실기체:
        #                           열린 문을 첫 볼 자리로 골랐다가 재계획 3번 한도에 11초 만에 포기했다)

    def note_replan(self, now: float) -> None:
        """다시 계획할 때 부른다. VP_REPLAN_GAP 안에 또 오면 세지 않는다."""
        if self.last_replan is None or now - self.last_replan >= VP_REPLAN_GAP - 1e-6:
            self.replans += 1
            self.last_replan = now

    def remaining(self, length: float, now: float) -> None:
        """남은 경로 길이를 알린다. VP_PROGRESS 넘게 줄었을 때만 '나아갔다'로 친다."""
        if length < self.best - VP_PROGRESS or not math.isfinite(self.best):
            self.best, self.stalled = length, 0.0

    def tick(self, now: float, moving: bool) -> None:
        """경로를 따라가는 주기마다 부른다. moving: 앞·옆으로 움직이라고 했나. 제자리 회전은 세지 않는다 —
        장애물 가까이서는 17°/s로 천천히 돌아, 출발 전에 돌기만 해도 10초가 넘었다(Gazebo 복도·문 앞, 11/04)."""
        if self.last is not None and moving:
            self.stalled += min(now - self.last, VP_TICK_MAX)
        self.last = now

    def give_up(self, now: float) -> str | None:
        """포기할 이유(로그 문구) 또는 None."""
        tight = self.tight and not self.door
        limit = VP_REPLANS_TIGHT if tight else VP_REPLANS
        if self.replans > limit:
            return f"재계획 {self.replans}번" + (" (좁은 길로만 계획이 나옴)" if tight else " (문 너머)" if self.door else "")
        stall = VP_STALL_TIGHT if tight else VP_STALL
        if self.stalled > stall:
            return f"{self.stalled:.0f}초 움직이는 동안 남은 길이 {VP_PROGRESS} m도 줄지 않음"
        return None


@dataclass
class FoundPerson:
    x: float
    y: float
    conf: float
    t_first: float
    hits: int = 1
    t_last: float = 0.0
    misses: int = 0        # 그 자리를 카메라로 똑바로 봤는데 아무도 없었던 횟수 (다시 잡히면 0)
    miss_since: float | None = None   # 그 '없음'이 처음 시작된 시각
    gone: bool = False     # 다시 봤는데 없어 길을 막지 않는다 (찾은 사람 목록에는 남는다, 다시 잡히면 다시 막는다)
    top: float | None = None   # 잰 머리 높이(m)의 최댓값. 머리가 화면 위로 잘려 한 번도 못 쟀으면 None (키 큰 사람으로 본다)
    top_n: int = 0             # 머리 높이를 잰 횟수 (한두 번 잰 값은 기체가 뒤로 기울어 작게 잰 것일 수 있다)


PERSON_TALL = 1.8        # 키를 못 쟀을 때 보는 사람 키 (m)
PERSON_RISK_NEAR = 0.6   # 사람 중심이 드론에서 이 안이면 부딪힐 위험이 높다 (사람 반지름 0.25 + 드론 0.25 + 0.1)
PERSON_RISK_RING = 1.0   # 이 안의 사람이 세 명 넘게 드론을 둘러싸 빠져나갈 반쪽이 없으면 위험이 높다


def person_top(bbox, cam_z: float, dist: float, width: int, height: int, hfov: float) -> float | None:
    """YOLO 사람 상자 위 변(머리 끝)의 높이 (m). 머리가 화면 위로 잘렸으면 None.

    카메라가 수평이라고 본다. 앞으로 날며 기수가 숙으면 머리가 화면 위쪽으로 가 키를 크게 잰다(안전한 쪽).
    """
    x1, y1, x2, y2 = bbox
    if y1 <= 3 or dist <= 0:
        return None
    f = (width / 2) / math.tan(hfov / 2)
    return cam_z + dist * (height / 2 - y1) / f


def people_risk(people, x: float, y: float, near: float = PERSON_RISK_NEAR, ring: float = PERSON_RISK_RING) -> str | None:
    """(x, y)의 드론이 사람과 부딪힐 위험이 높으면 그 이유(문장), 낮으면 None.

    높음: 가장 가까운 사람이 near m 안이거나, ring m 안의 사람 세 명 넘게가 사방을 둘러싸 사람 없는 반쪽(180°)이 없을 때.
    2026-11-02 사용자 결정: 사람이 길을 막아도 위험이 낮으면 탐색을 이어 가고, 높을 때만 그 자리에 내린다.
    """
    around = sorted((math.hypot(p.x - x, p.y - y), math.atan2(p.y - y, p.x - x)) for p in people)
    if not around:
        return None
    if around[0][0] < near:
        return f"가장 가까운 사람이 {around[0][0]:.1f} m 옆에 있습니다"
    angles = sorted(a for d, a in around if d < ring)
    if len(angles) >= 3:
        gaps = [b - a for a, b in zip(angles, angles[1:])] + [angles[0] + 2 * math.pi - angles[-1]]
        if max(gaps) < math.pi:
            return f"{ring:g} m 안의 사람 {len(angles)}명이 사방을 둘러쌌습니다 (빠져나갈 쪽 가장 넓은 틈 {math.degrees(max(gaps)):.0f}°)"
    return None


def person_in_view(px: float, py: float, cam, yaw: float, hfov: float,
                   near: float = 0.8, far: float = 3.5, edge: float = 0.2):
    """카메라(cam, 기수 yaw)가 (px, py)를 화면 가운데 쪽으로 똑바로 보는가. 보면 (거리, 화면 안 방향 rad), 아니면 None.

    멀면 YOLO가 사람을 놓칠 수 있고 화면 가장자리는 몸이 잘려 못 잡을 수 있어 뺀다."""
    dx, dy = px - cam[0], py - cam[1]
    dist = math.hypot(dx, dy)
    b = math.atan2(math.sin(math.atan2(dy, dx) - yaw), math.cos(math.atan2(dy, dx) - yaw))
    if not (near <= dist <= far) or abs(b) > hfov / 2 - edge:
        return None
    return dist, b


def _taller(a: float | None, b: float | None) -> float | None:
    """두 키 중 큰 쪽 (잰 적 없는 None은 빼고)."""
    return b if a is None else a if b is None else max(a, b)


def person_seen_toward(bearing: float, dist: float, bearings) -> bool:
    """그 방향(bearing)에 YOLO 사람 상자가 있는가. 사람 위치 오차(0.5 m 안팎)만큼 방향을 넉넉히 본다."""
    tol = max(0.25, math.atan2(0.7, dist))
    return any(abs(math.atan2(math.sin(b - bearing), math.cos(b - bearing))) < tol for b in bearings)


class PeopleLog:
    """찾은 사람 목록. merge m 안의 탐지는 같은 사람으로 보고 위치를 평균한다."""

    def __init__(self, merge: float = 1.0):
        self.merge = merge
        self.people: list[FoundPerson] = []

    def add(self, x: float, y: float, conf: float, t: float, top: float | None = None) -> FoundPerson | None:
        """새 사람이면 그 사람을, 이미 찾은 사람이면 None을 돌려준다. top은 이번에 잰 머리 높이(못 쟀으면 None)."""
        for p in self.people:
            if math.hypot(p.x - x, p.y - y) < self.merge:
                p.x = (p.x * p.hits + x) / (p.hits + 1)
                p.y = (p.y * p.hits + y) / (p.hits + 1)
                p.hits += 1
                p.conf = max(p.conf, conf)
                p.t_last, p.misses, p.gone = max(p.t_last, t), 0, False
                p.top = _taller(p.top, top)
                p.top_n += top is not None
                self._consolidate(p)
                return None
        person = FoundPerson(x, y, conf, t, t_last=t, top=top, top_n=int(top is not None))
        self.people.append(person)
        return person

    def _consolidate(self, p: FoundPerson) -> None:
        """평균이 옮겨 가 다른 항목과 merge 안으로 들어오면 합친다. 첫 추정이 크게 빗나가면 같은 사람이
        따로 생겼다가 나중에 0.2 m까지 가까워지고도 둘로 남았다 (3단계 seed 3)."""
        for q in [q for q in self.people if q is not p and math.hypot(q.x - p.x, q.y - p.y) < self.merge]:
            n = p.hits + q.hits
            p.x, p.y = (p.x * p.hits + q.x * q.hits) / n, (p.y * p.hits + q.y * q.hits) / n
            p.hits, p.conf, p.t_first = n, max(p.conf, q.conf), min(p.t_first, q.t_first)
            p.t_last = max(p.t_last, q.t_last)
            p.top, p.top_n = _taller(p.top, q.top), p.top_n + q.top_n
            self.people.remove(q)

    def confirmed(self, min_hits: int = 2, min_conf: float = 0.7) -> list[FoundPerson]:
        """두 번 넘게 잡혔거나 한 번이라도 확실히(min_conf) 잡힌 사람. 한 번 흐릿하게 잡힌 것은 오탐이 많다."""
        return [p for p in self.people if p.hits >= min_hits or p.conf >= min_conf]

    def blocking(self) -> list[FoundPerson]:
        """길을 막는 사람: 확정된 사람 중 그 자리를 다시 봤을 때 비어 있지 않았던 사람."""
        return [p for p in self.confirmed() if not p.gone]

    def note_empty(self, p: FoundPerson, need: int, t: float | None = None, min_time: float = 0.0) -> bool:
        """p가 있던 자리를 똑바로 봤는데 아무도 없었다. need번 이어지고 첫 번째부터 min_time초가 지나면 p는 길을 막지 않는다.
        막기를 푼 순간 True. 횟수만 보면 탐지가 빨라질수록(초당 5 → 10번) 더 빨리 지워진다."""
        if p.misses == 0:
            p.miss_since = t
        p.misses += 1
        long_enough = min_time <= 0 or (t is not None and p.miss_since is not None and t - p.miss_since >= min_time)
        if not p.gone and p.misses >= need and long_enough:
            p.gone = True
            return True
        return False
