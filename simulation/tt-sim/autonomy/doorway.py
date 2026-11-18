"""경로 위의 문(좁은 틈) 찾기. ROS와 무관.

2026-11-05 10:51 실기체: 열린 문(공기청정기와 문 열린 사물함 사이)을 첫 볼 자리로 골랐는데, 그리로 가는 길이 줄인 지도
여유(0.2 m)로만 나왔다. 문틀 둘레 지도가 0.5초마다 조금씩 바뀌어 '새 장애물'로 다시 계획하기를 되풀이하다가, 좁은 길 포기
규칙(재계획 3번)에 걸려 11초 만에 포기하고 문 둘레 1.5 m를 금지했다. 그 뒤 '닫힌 공간'으로 판정하고 45초를 남긴 채 돌아왔다.

그래서 경로가 좁은 틈을 지나면 그 틈을 문으로 본다. 경로를 따라 막힌 칸과의 거리(칸 중심 기준)를 재어, 평소 여유로는
막히는 구간(좁은 구간) 가운데 가장 좁은 점에서 양옆 문틀을 찾는다. 가로지르는 선을 ±50° 돌려 가며 양옆 문틀 사이가
가장 짧은 선을 문의 가로선으로 본다(비스듬히 재면 폭이 늘어난다). 그 수직이 지나갈 방향이다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from scipy import ndimage

DOOR_SCAN = 1.0          # 가장 좁은 점에서 양옆으로 문틀을 찾는 거리 (m). 한쪽이라도 없으면 문이 아니다(옆으로 비킬 수 있다)
DOOR_ANGLES = range(-50, 51, 5)   # 가로선을 돌려 보는 각 (°). 10:51 경로는 문에 약 40° 비스듬히 다가갔다
DOOR_WIDTH = (0.5, 1.6)   # 문으로 볼 폭 (m, 지도에서 잰 값). 1.6 m 넘으면 좁은 틈이 아니다. 지도는 문을 좁게 잰다: 11/05 13:04 실기체
#                          교실 문 0.55 m(지나감), 열린 문짝과 벽 사이 0.48 m(가짜 문) — 0.45는 가짜 문까지 들였다
DOOR_RUN = 2.5           # 좁은 구간이 경로를 따라 이보다 길면 문이 아니라 좁은 복도다 (곧게 지나기를 쓰지 않는다).
#                          문에 40° 비스듬히 다가가면 벽 0.4 m 안을 지나는 길이가 1.35 m, 문 앞에 사물함·기둥이 있으면 더 길다
#                          (Gazebo 문 월드 11/05: 1.5–1.9 m)
DOOR_SPECK = 4           # 이보다 작은 막힌 덩어리(칸 수)는 문틀이 아니라 깊이 잡음으로 보고 뺀다
# 복도 가리기 (11/11): 폭 1.4 m 복도가 지도에 1.1–1.3 m로 잡히면 좁은 구간이 띄엄띄엄(2.5 m보다 짧게) 생겨 복도 한가운데를 문으로 봤다
# (Gazebo ㄱ자 복도: 한 판에 3–6번, 탐색 150초 중 60–75초를 멈추고 재고 0.15 m/s로 지나는 데 씀)
DOOR_WIDE = 1.05         # 이보다 좁은 틈은 앞뒤가 어떻든 문으로 본다 (m). 실제 문은 0.8–1.0 m, 지도는 그보다 좁게 잰다
DOOR_PINCH = 0.3         # 틈 앞·뒤 통로의 폭이 틈보다 이만큼 넘게 넓으면 '조인 곳'(문)이다 (m)
DOOR_REACH = (0.5, 1.0)  # 틈 가운데에서 앞·뒤로 이 거리마다 통로 폭을 잰다 (m)
# 문 앞 사람 (11/12 17:19 실기체): 문 오른쪽 문틀 뒤(가운데 선에서 옆 0.8–0.9 m, 문 면 너머 0.5 m)에 찾은 사람이 남아 '문 둘레 1.2 m 안에
# 사람'으로 세 번 그만두고 문 너머를 못 봤다. 곧게 지나면 그 사람 몸 가장자리와 0.57 m 넘게 떨어지는 자리였다.
# 그래서 곧게 지나는 선분에서 몸 가장자리까지 이만큼 안에 든 사람만 문을 막는다고 본다. 두 사람 사이 틈을 문으로 본 경우
# (1.2 m 간격 두 사람: 몸 가장자리 0.35 m, Gazebo people_pair 11/10)는 그대로 막는다
PASS_PEOPLE_CLEAR = 0.45


@dataclass
class Doorway:
    center: np.ndarray    # 문 가운데 (x, y)
    normal: np.ndarray    # 지나갈 방향 (단위 벡터, 경로 방향 쪽)
    width: float          # 양옆 문틀 사이 빈 폭 (m)
    z: float | None = None   # 경로가 문을 지나는 높이 (m, 경로가 3D일 때)

    def along(self, x: float, y: float) -> float:
        """문 가운데를 0으로, 지나갈 방향으로 잰 거리 (m). 문 앞은 음수."""
        return float((np.array([x, y]) - self.center) @ self.normal)

    def across(self, x: float, y: float) -> float:
        """문 가운데 선에서 왼쪽(+)으로 벗어난 거리 (m)."""
        d = np.array([x, y]) - self.center
        return float(-d[0] * self.normal[1] + d[1] * self.normal[0])

    def point(self, along: float, across: float = 0.0) -> np.ndarray:
        n = self.normal
        return self.center + n * along + np.array([-n[1], n[0]]) * across

    def pass_gap(self, x: float, y: float, pre: float, post: float) -> float:
        """곧게 지나는 선분(가운데 선 위, 문 앞 pre부터 너머 post까지)에서 (x, y)까지 거리 (m)."""
        q = self.point(min(max(self.along(x, y), -pre), post))
        return float(math.hypot(x - float(q[0]), y - float(q[1])))

    def on_pass(self, x: float, y: float, pre: float, post: float, body: float, clear: float = PASS_PEOPLE_CLEAR) -> bool:
        """(x, y)에 선 몸(지름 body)의 가장자리가 곧게 지나는 선분에서 clear 안인가 (그 사람이 문을 막는가)."""
        return self.pass_gap(x, y, pre, post) - body / 2 < clear

    def covers(self, pos, goal, pre: float, side: float, near: float = 0.5) -> bool:
        """문을 곧게 지나 pos에 섰을 때 목적지 goal에 이미 닿은 셈인가: pos에서 near 안이거나, 방금 곧게 지나온 구간(문 앞 pre부터
        지금 자리까지)의 옆 side 안에 있다.

        11/11 Gazebo ㄱ자 복도: 볼 자리가 틈 가로선 위(가운데에서 옆 0.4 m)였는데, 틈을 0.55 m 지나쳐 나온 뒤 그 자리로 돌아가는
        0.6 m 경로가 같은 틈을 반대로 지난다고 보고 되돌아 1.2 m를 곧게 지났다가 다시 돌아왔다 (탐색 150초 중 27초).
        """
        gx, gy = float(goal[0]), float(goal[1])
        if math.hypot(gx - float(pos[0]), gy - float(pos[1])) < near:
            return True
        return -pre <= self.along(gx, gy) <= self.along(float(pos[0]), float(pos[1])) + 0.2 and abs(self.across(gx, gy)) <= side


def _sample(path, step: float) -> np.ndarray:
    """경로 점(2D 또는 3D)을 수평 step마다 나눈다. 3D면 높이도 같이 나눠 둔다."""
    dim = 3 if len(np.asarray(path[0])) >= 3 else 2
    pts = [np.asarray(path[0], dtype=float)[:dim]]
    for a, b in zip(path, path[1:]):
        a, b = np.asarray(a, dtype=float)[:dim], np.asarray(b, dtype=float)[:dim]
        n = max(1, int(math.ceil(np.linalg.norm(b[:2] - a[:2]) / step)))
        pts += [a + (b - a) * t for t in np.linspace(0.0, 1.0, n + 1)[1:]]
    return np.array(pts)


def clean(blocked: np.ndarray, speck: int = DOOR_SPECK) -> np.ndarray:
    """speck칸보다 작은 막힌 덩어리(깊이 잡음 점)를 뺀다. 벽·문틀·가구는 큰 덩어리다.
    Gazebo 문 월드(11/05): 문 둘레 점 하나에 비스듬한 가로선이 걸려 폭을 0.03 m로 재 문을 못 봤다."""
    if speck <= 1 or not blocked.any():
        return blocked
    lab, n = ndimage.label(blocked, structure=np.ones((3, 3), dtype=bool))
    sizes = np.bincount(lab.ravel())
    keep = sizes >= speck
    keep[0] = False
    return keep[lab]


class _Grid:
    def __init__(self, blocked: np.ndarray, origin, res: float):
        self.b, self.o, self.r = blocked, np.asarray(origin, dtype=float)[:2], float(res)

    def idx(self, p):
        return int(math.floor((p[0] - self.o[0]) / self.r)), int(math.floor((p[1] - self.o[1]) / self.r))

    def blocked(self, p) -> bool:
        i, j = self.idx(p)
        return not (0 <= i < self.b.shape[0] and 0 <= j < self.b.shape[1]) or bool(self.b[i, j])

    def first_hit(self, p, u, reach: float) -> float | None:
        """p에서 u 방향으로 처음 막힌 칸까지 거리 (m). reach 안에 없으면 None."""
        step = self.r / 4
        for s in np.arange(step, reach + 1e-9, step):
            if self.blocked(p + u * s):
                return float(s)
        return None


def measure(blocked: np.ndarray, origin, res: float, point, direction, scan: float = DOOR_SCAN,
            angles=DOOR_ANGLES, speck: int = DOOR_SPECK) -> Doorway | None:
    """point(틈 안의 한 점)에서 가로선을 돌려 가며 양옆 문틀을 찾는다. direction은 대략 지나갈 방향."""
    g = _Grid(clean(blocked, speck), origin, res)
    p = np.asarray(point, dtype=float)[:2]
    t = np.asarray(direction, dtype=float)[:2]
    t = t / max(np.linalg.norm(t), 1e-9)
    best = None
    for deg in angles:
        a = math.atan2(t[1], t[0]) + math.pi / 2 + math.radians(deg)
        u = np.array([math.cos(a), math.sin(a)])      # 가로선 방향 (지나갈 방향의 왼쪽 쪽)
        left, right = g.first_hit(p, u, scan), g.first_hit(p, -u, scan)
        if left is None or right is None:
            continue
        if best is None or left + right < best[0] + best[1] - 1e-9:
            best = (left, right, u)
    if best is None:
        return None
    left, right, u = best
    width = left + right - res / 2          # 처음 막힌 칸에 든 점은 칸 가장자리에서 평균 res/4 안쪽
    n = np.array([u[1], -u[0]])              # u의 오른쪽 수직 = 지나갈 방향
    if n @ t < 0:
        n = -n
    center = p + u * (left - right) / 2
    return Doorway(center=center, normal=n, width=float(width))


def is_corridor(blocked: np.ndarray, origin, res: float, d: Doorway, scan: float = DOOR_SCAN, wide: float = DOOR_WIDE,
                pinch: float = DOOR_PINCH, reach=DOOR_REACH, speck: int = DOOR_SPECK) -> bool:
    """d가 문이 아니라 좁은 복도의 한 자리인가. 폭이 wide 이상이고, 틈 가운데에서 앞·뒤 reach 자리마다 양옆 벽이 scan 안에 있으며
    그 폭이 d.width + pinch를 넘지 않을 때다 — 양옆 벽이 같은 간격으로 이어진다. 앞이나 뒤 한 곳이라도 트여 있거나(방), 막혔거나,
    본 적 없으면(blocked에 모르는 칸이 들어 있다) 문으로 둔다: 문은 얇은 벽에 난 조인 곳이고, 문틀을 스치는 것보다 한 번 멈추는 편이 낫다."""
    if d.width < wide:
        return False
    b = clean(blocked, speck)
    g = _Grid(b, origin, res)
    for side in (-1.0, 1.0):
        for r in reach:
            p = d.point(side * r)
            if g.blocked(p):
                return False
            m = measure(b, origin, res, p, d.normal, scan=scan, speck=0)
            if m is None or m.width > d.width + pinch:
                return False
    return True


def open_axis(vmap, d: Doorway, before: float, after: float, z: float):
    """계획 지도 vmap(VoxelMap)의 사본에서 문 d의 가운데 선을 연다: 문 앞 before부터 문 너머 after까지, 옆 ±1칸, 높이 z ±1칸.

    문 지나기로 지나온 좁은 틈은 그 뒤 양쪽 문틀에 ToF가 찍히면 계획 지도에서 닫힌다(찍는 자리가 문틀보다 0.1–0.2 m 안쪽이고, 지나온
    칸에 다시 찍힌 막힘은 둘레 여유까지 막는다). 이 사본은 '문 너머에서 목적지까지 길이 있는가'만 보는 데 쓴다 — 지나는 것 자체는
    지도가 아니라 문 지나기(문 앞에서 다시 재고 정면 ToF 확인)가 판단한다."""
    out = type(vmap)(vmap.origin.copy(), vmap.res, vmap.occ.copy())
    r = float(vmap.res)
    for s in np.arange(-before, after + 1e-9, r / 2):
        for w in (-r, 0.0, r):
            p = d.point(float(s), w)
            for dz in (-r, 0.0, r):
                idx = out.index((p[0], p[1], z + dz))
                if out.inside(idx):
                    out.occ[idx] = False
    return out


def trail_gap(points, blocked_at, max_span: float) -> Doorway | None:
    """지나온 자취 points(오래된 것 → 최근, (x, y, …))를 끝에서부터 거꾸로 따라가다 처음 만나는 '지금은 막힌' 구간을 문으로 돌려준다:
    가운데는 구간 양 끝(막히기 직전의 빈 점들)의 가운데, 지나갈 방향은 최근 쪽 끝 → 옛 쪽 끝(되나가는 방향), 폭은 모름(0).

    좁은 틈을 문 지나기가 아니라 보통 경로(줄인 여유)로 지난 경우다 — Gazebo 실습실(11/11 두 번째): 0.75 m 틈을 0.25 m/s로 지난 뒤
    그 틈이 지도에서 닫혀 출발점 2.6 m 앞에 내렸다. 막힌 구간이 없거나, 드론이 선 자리부터 막혔거나, 자취 처음까지 막혀 있거나,
    두 끝 사이가 max_span보다 멀면(틈이 아니라 넓게 막힌 것) None."""
    i = len(points) - 1
    while i >= 0 and not blocked_at(points[i]):
        i -= 1
    if i < 0 or i == len(points) - 1:
        return None
    near = np.asarray(points[i + 1], dtype=float)[:2]
    while i >= 0 and blocked_at(points[i]):
        i -= 1
    if i < 0:
        return None
    far = np.asarray(points[i], dtype=float)[:2]
    span = float(np.linalg.norm(far - near))
    if span < 1e-6 or span > max_span:
        return None
    return Doorway(center=(near + far) / 2, normal=(far - near) / span, width=0.0)


def find_doorway(blocked: np.ndarray, origin, res: float, path, narrow: float, skip: float = 0.3,
                 scan: float = DOOR_SCAN, width=DOOR_WIDTH, run: float = DOOR_RUN, speck: int = DOOR_SPECK,
                 ahead: float = 0.4) -> Doorway | None:
    """경로가 지나는 첫 좁은 틈. blocked: 비행 높이에 실제로 있는 막힌 칸(부풀리기 전, 모르는 칸 포함) 2D 배열 [x, y].
    탁자처럼 그 높이 아래인 가구는 넣지 않는다 — 경로는 그 위로 넘을 수 있고, 넣으면 가구 사이가 문처럼 좁아 보인다.
    narrow: 평소 지도 여유 (m). 경로 점에서 가장 가까운 막힌 칸 중심까지가 이 안이면 평소 여유로는 막히는 '좁은 구간'이다.
    skip: 출발점에서 경로를 따라 이 거리 안은 보지 않는다 (드론이 이미 좁은 곳에 있으면 그 자리는 문이 아니다).
    ahead: 출발점에서 이어지는 좁은 구간이라도 가장 좁은 곳이 출발점에서 이만큼 넘게 앞이면 문으로 본다 (드론은 아직 문 앞).
    10:51 bag 재연: 출발 자리가 막힌 칸 둘레라 '첫 좁은 구간'이 늘 출발점에서 시작해 문을 못 봤다.
    path가 3D면 문에서의 경로 높이를 Doorway.z에 둔다 (Gazebo 문 월드 11/05: 경로가 탁자 위 1.35 m 그대로 문을 지났다)."""
    if path is None or len(path) < 2:
        return None
    blocked = clean(blocked, speck)
    pts3 = _sample(path, res / 2)
    pts = pts3[:, :2]
    walked = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))])
    dist = ndimage.distance_transform_edt(~blocked) * res      # 빈 칸 중심에서 가장 가까운 막힌 칸 중심까지 (m)
    g = _Grid(blocked, origin, res)
    clear = np.array([0.0 if g.blocked(p) else dist[g.idx(p)] for p in pts])
    tight = (clear <= narrow + 1e-9) & (walked >= skip)
    start = first = int(np.argmax(walked >= skip))
    while start < len(pts) and tight[start]:      # 건너뛴 자리에서 이어지는 좁은 구간은 드론이 이미 들어가 있는 곳
        tight[start] = False
        start += 1
    # 다만 그 구간의 가장 좁은 곳이 출발점에서 ahead 넘게 앞이면 드론은 아직 문 앞이다: 그 문을 돌려준다 (11/11). narrow 0.6 m로 보면
    # 0.8 m 문은 문틀 앞 0.45 m부터 좁은 구간이라, 문 0.75 m 안에서 다시 계획하면(문 앞에서 둘러본 뒤, 문 지나기를 그만두고 물러난 뒤)
    # 문이 통째로 건너뛰어져 보통 경로로 비스듬히 지났다 — Gazebo 방 두 개: 위치 오차 0.15 m로 문틀 모서리에 −0.00 m
    if start > first and float(walked[start - 1] - walked[first]) <= run:
        d = _narrowest(blocked, origin, res, pts3, first, start - 1, scan)
        if d is not None and width[0] <= d.width <= width[1] and d.along(float(pts[0][0]), float(pts[0][1])) <= -ahead:
            return d
    # 0.1 m 안의 끊김은 이어 붙인다 (비스듬히 다가가면 문틀 모서리 앞에서 잠깐 넓어졌다 다시 좁아진다)
    tight = ndimage.binary_closing(tight, structure=np.ones(5, dtype=bool)) & (walked >= skip) | tight
    labels, n = ndimage.label(tight)
    for lab in range(1, n + 1):
        idx = np.flatnonzero(labels == lab)
        i0, i1 = int(idx[0]), int(idx[-1])
        if float(walked[i1] - walked[i0]) > run:
            continue   # 좁은 복도는 문이 아니다 (곧게 지나지 않고 보통 경로로). 그 너머의 문은 본다
        d = _narrowest(blocked, origin, res, pts3, i0, i1, scan)
        if d is not None and width[0] <= d.width <= width[1]:
            return d
    return None


def _narrowest(blocked, origin, res, pts3, i0: int, i1: int, scan: float) -> Doorway | None:
    """좁은 구간을 0.1 m마다 재어 가장 좁은 곳(문틀 사이)을 고른다. 막힌 칸과 가장 가까운 점은 비스듬히 다가갈 때 문틀 모서리
    앞(문 밖)이라 가로선이 한쪽 벽면에만 닿는다. 벽 두께만큼 같은 폭이 이어지면 그 가운데 (앞쪽 끝이면 문 가운데가 앞으로 밀린다)."""
    pts = pts3[:, :2]
    found = []
    for k in range(i0, i1 + 1, 2):
        back, ahead = pts[max(0, k - 6)], pts[min(len(pts) - 1, k + 6)]   # ±0.3 m로 경로 방향
        if np.linalg.norm(ahead - back) < 1e-6:
            continue
        d = measure(blocked, origin, res, pts[k], ahead - back, scan=scan, speck=0)   # 이미 잡음을 뺀 지도
        if d is not None:
            if pts3.shape[1] >= 3:
                d.z = float(pts3[k, 2])
            found.append(d)
    if not found:
        return None
    wmin = min(d.width for d in found)
    ties = [d for d in found if d.width <= wmin + 0.03]
    return ties[len(ties) // 2]
