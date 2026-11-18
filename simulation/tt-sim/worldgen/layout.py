"""방·문·가구·잡동사니·사람·출발점·목적지를 무작위로 배치한다.

좌표계: x 동쪽, y 북쪽, z 위, 단위 m. 맵 영역은 (0, 0) ~ (width, depth).
같은 seed는 단계(stage)를 바꿔도 같은 건물을 만든다. 단계는 드론에게
무엇을 알려 주는지(지도·목적지)와 사람 배치 여부만 바꾼다.
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass, field

from . import furnish
from .grid import OccupancyGrid

# 단계별로 드론에게 주는 정보
STAGES: dict[int, dict] = {
    1: {"desc": "3D 지도 있음, 목적지 명확",
        "hide_obstacles": False, "humans": False, "goal": True, "known_map": True},
    2: {"desc": "3D 지도 있음, 목적지 명확, 지도에 없는 물건 있음",
        "hide_obstacles": True, "humans": False, "goal": True, "known_map": True},
    3: {"desc": "3D 지도 있음, 지도에 없는 물건 있음, 목적지 없음(사람 탐색)",
        "hide_obstacles": True, "humans": True, "goal": False, "known_map": True},
    4: {"desc": "지도 없음, 사람 탐색",
        "hide_obstacles": True, "humans": True, "goal": False, "known_map": False},
}

ART_COUNT = 6   # 그림 텍스처 종류 수


@dataclass
class LayoutParams:
    width: float = 12.0              # 맵 가로 (m)
    depth: float = 10.0              # 맵 세로 (m)
    wall_height: float = 2.5
    wall_thickness: float = 0.1
    min_room: float = 3.0            # 방 한 변의 최소 길이
    door_width: float = 1.0
    door_height: float = 2.1
    max_panel: float = 2.0           # 벽을 이 길이 이하 조각으로 나눈다
    clutter: int | None = None       # 지도에 없는 물건 수 (None이면 방 수에 맞춰 자동)
    blockers: int = 0                # 출발점→목적지 최단 경로 위에 일부러 놓는 지도에 없는 물건 수
    humans: int = 3
    drone_radius: float = 0.2        # 가드 포함 반지름 약 0.09 m + 여유 (가정)
    grid_res: float = 0.1
    flight_height: float = 1.0

    def validate(self) -> None:
        if self.min_room < self.door_width + 1.0 + 0.2:
            raise ValueError("min_room은 door_width + 1.2 m 이상이어야 문을 낼 수 있습니다.")
        if self.width < self.min_room or self.depth < self.min_room:
            raise ValueError("맵 크기가 min_room보다 작습니다.")
        if not 0 < self.door_height < self.wall_height:
            raise ValueError("door_height는 0보다 크고 wall_height보다 작아야 합니다.")
        if self.clutter is not None and self.clutter < 0:
            raise ValueError("clutter는 0 이상이어야 합니다.")
        if self.blockers < 0:
            raise ValueError("blockers는 0 이상이어야 합니다.")


@dataclass(frozen=True)
class Box:
    """회전 없는 상자. (x, y, z)는 중심, (sx, sy, sz)는 크기."""
    name: str
    x: float
    y: float
    z: float
    sx: float
    sy: float
    sz: float
    texture: str = "paint"

    def bounds2d(self) -> tuple[float, float, float, float]:
        return (self.x - self.sx / 2, self.y - self.sy / 2, self.x + self.sx / 2, self.y + self.sy / 2)


@dataclass(frozen=True)
class Rect:
    x0: float
    y0: float
    x1: float
    y1: float

    @property
    def w(self) -> float:
        return self.x1 - self.x0

    @property
    def h(self) -> float:
        return self.y1 - self.y0


@dataclass(frozen=True)
class Room(Rect):
    kind: str = "office"     # furnish.ROOM_KINDS의 키
    floor: str = "tile"      # wood | tile | carpet | concrete


@dataclass(frozen=True)
class Door:
    x: float
    y: float
    wall: str      # "x": x가 일정한 세로 벽에 난 문, "y": y가 일정한 가로 벽에 난 문
    width: float

    def approach_points(self, offset: float = 0.5) -> list[tuple[float, float]]:
        """문 양쪽으로 offset만큼 떨어진 점. 문을 지나갈 수 있는지 검사할 때 쓴다."""
        if self.wall == "x":
            return [(self.x - offset, self.y), (self.x + offset, self.y)]
        return [(self.x, self.y - offset), (self.x, self.y + offset)]


@dataclass
class Obstacle:
    """가구 또는 잡동사니. box는 충돌·지도용 바닥 면적 상자(z 0 ~ 높이)."""
    box: Box
    kind: str = "box"
    yaw: float = 0.0        # 정면(방 쪽)이 향하는 방향
    w: float = 0.0          # 지역 좌표 폭
    d: float = 0.0          # 지역 좌표 깊이
    hidden: bool = False    # True면 2단계 이상에서 known_map에서 빠진다 (새로 놓인 물건)
    seed: int = 0           # 책 색 같은 세부 모양 난수
    blocker: bool = False   # 출발점→목적지 최단 경로를 막으려고 놓은 물건

    def parts(self) -> list[furnish.Part]:
        return furnish.build(self.kind, self.box.x, self.box.y, self.yaw, self.w, self.d, self.box.sz, self.seed)


@dataclass(frozen=True)
class Human:
    name: str
    x: float
    y: float
    yaw: float
    radius: float = 0.25


@dataclass
class World:
    seed: int
    stage: int
    params: LayoutParams
    rooms: list[Room]
    doors: list[Door]
    walls: list[Box]
    lintels: list[Box]             # 문 위 벽(높이 door_height 이상). 비행 높이에서는 막지 않는다
    floor: list[Box]               # 방마다 하나
    obstacles: list[Obstacle]
    humans: list[Human]            # 이 단계에서 실제로 배치하는 사람
    spawn: dict
    goal: dict | None              # 이 단계에서 드론에게 주는 목적지
    decor: list[furnish.Part]      # 걸레받이·문틀·창문·그림 (충돌체 없음)
    paint: tuple[int, int, int]    # 벽 색 (RGB)

    def is_known(self, obstacle: Obstacle) -> bool:
        """이 단계에서 드론이 받는 지도에 이 물건이 들어 있는지."""
        cfg = STAGES[self.stage]
        if not cfg["known_map"]:
            return False
        return not (cfg["hide_obstacles"] and obstacle.hidden)


@dataclass
class _Segment:
    """축에 평행한 벽 중심선. gaps는 벽 방향 좌표로 표시한 문 구간."""
    x0: float
    y0: float
    x1: float
    y1: float
    gaps: list[tuple[float, float]] = field(default_factory=list)


PAINTS = [(236, 234, 226), (228, 224, 212), (222, 226, 228), (232, 226, 214)]


def generate(seed: int, stage: int, params: LayoutParams | None = None) -> World:
    if stage not in STAGES:
        raise ValueError(f"stage는 {sorted(STAGES)} 중 하나여야 합니다: {stage}")
    p = params or LayoutParams()
    p.validate()

    # 용도별로 난수열을 나눠, 한쪽 설정을 바꿔도 다른 쪽 배치가 흔들리지 않게 한다.
    layout_rng = random.Random(f"{seed}:layout")
    rects, doors, segments = _split_rooms(layout_rng, p)
    rooms = _assign_rooms(layout_rng, rects)
    walls = _wall_boxes(segments, p)
    lintels = _lintels(doors, p)
    floor = [Box(f"floor_{i:02d}", round((r.x0 + r.x1) / 2, 4), round((r.y0 + r.y1) / 2, 4), -0.05,
                 round(r.w, 4), round(r.h, 4), 0.1, texture=f"floor_{i}") for i, r in enumerate(rooms)]

    grid = OccupancyGrid(p.width, p.depth, p.grid_res)
    for wall in walls:   # 문 위 벽(lintel)은 비행 높이 위라 격자에 넣지 않는다
        grid.mark_rect(*wall.bounds2d(), inflate=p.drone_radius)
    keypoints = [pt for door in doors for pt in door.approach_points()]
    if not grid.connected(keypoints):
        raise RuntimeError(f"seed {seed}: 방 배치가 서로 이어지지 않습니다.")

    furniture, grid = _furnish(random.Random(f"{seed}:furniture"), p, rooms, doors, grid, keypoints)
    clutter, grid = _place_clutter(random.Random(f"{seed}:clutter"), p, rooms, doors, grid, keypoints, furniture)
    obstacles = furniture + clutter

    spawn_rng = random.Random(f"{seed}:spawn")
    spawn, dist = _choose_spawn(spawn_rng, rooms, grid, keypoints)
    goal = _choose_goal(spawn_rng, p, grid, dist)
    if p.blockers and goal:
        blockers, grid = _place_blockers(random.Random(f"{seed}:blockers"), p, doors, walls, lintels, grid,
                                         keypoints, spawn, goal, obstacles)
        obstacles = obstacles + blockers

    solids = [w.bounds2d() for w in walls] + [o.box.bounds2d() for o in obstacles]
    humans = _place_humans(random.Random(f"{seed}:humans"), p, grid, keypoints, spawn, solids)

    decor_rng = random.Random(f"{seed}:decor")
    decor = furnish.decorations(decor_rng, p, rooms, doors, walls, obstacles, ART_COUNT)

    cfg = STAGES[stage]
    return World(
        seed=seed,
        stage=stage,
        params=p,
        rooms=rooms,
        doors=doors,
        walls=walls,
        lintels=lintels,
        floor=floor,
        obstacles=obstacles,
        humans=humans if cfg["humans"] else [],
        spawn=spawn,
        goal=goal if cfg["goal"] else None,
        decor=decor,
        paint=random.Random(f"{seed}:paint").choice(PAINTS),
    )


# ---------------------------------------------------------------- 방 나누기

def _split_rooms(rng: random.Random, p: LayoutParams):
    W, D, t = p.width, p.depth, p.wall_thickness
    segments = [
        _Segment(-t / 2, 0.0, W + t / 2, 0.0),   # 남쪽 외벽
        _Segment(-t / 2, D, W + t / 2, D),       # 북쪽 외벽
        _Segment(0.0, 0.0, 0.0, D),              # 서쪽 외벽
        _Segment(W, 0.0, W, D),                  # 동쪽 외벽
    ]
    rooms: list[Rect] = []
    doors: list[Door] = []
    stack = [(Rect(0.0, 0.0, W, D), 0)]
    while stack:
        rect, depth = stack.pop()
        split = _try_split(rng, rect, depth, p, doors)
        if split is None:
            rooms.append(rect)
            continue
        segment, door, a, b = split
        segments.append(segment)
        doors.append(door)
        stack.append((a, depth + 1))
        stack.append((b, depth + 1))
    rooms.sort(key=lambda r: (r.y0, r.x0))
    return rooms, doors, segments


def _try_split(rng, rect: Rect, depth: int, p: LayoutParams, doors: list[Door]):
    can_vertical = rect.w >= 2 * p.min_room     # x = s 인 세로 벽으로 나누기
    can_horizontal = rect.h >= 2 * p.min_room   # y = s 인 가로 벽으로 나누기
    if not (can_vertical or can_horizontal):
        return None
    if depth >= 2 and rng.random() < 0.35:      # 큰 방도 가끔 남겨 둔다
        return None
    if can_vertical and can_horizontal:
        vertical = rng.random() < rect.w / (rect.w + rect.h)   # 긴 쪽을 자를 확률이 높다
    else:
        vertical = can_vertical

    for _ in range(20):
        if vertical:
            s = round(rng.uniform(rect.x0 + p.min_room, rect.x1 - p.min_room), 2)
            if _ends_in_door(s, rect.y0, rect.y1, "y", doors):
                continue
            c = _door_center(rng, rect.y0, rect.y1, p)
            segment = _Segment(s, rect.y0, s, rect.y1, gaps=[(c - p.door_width / 2, c + p.door_width / 2)])
            return segment, Door(s, c, "x", p.door_width), Rect(rect.x0, rect.y0, s, rect.y1), Rect(s, rect.y0, rect.x1, rect.y1)
        s = round(rng.uniform(rect.y0 + p.min_room, rect.y1 - p.min_room), 2)
        if _ends_in_door(s, rect.x0, rect.x1, "x", doors):
            continue
        c = _door_center(rng, rect.x0, rect.x1, p)
        segment = _Segment(rect.x0, s, rect.x1, s, gaps=[(c - p.door_width / 2, c + p.door_width / 2)])
        return segment, Door(c, s, "y", p.door_width), Rect(rect.x0, rect.y0, rect.x1, s), Rect(rect.x0, s, rect.x1, rect.y1)
    return None


def _ends_in_door(s: float, end_a: float, end_b: float, end_wall: str, doors: list[Door]) -> bool:
    """새 벽의 양 끝이 기존 문을 막으면 True.

    세로 벽(x = s)은 y = end_a, end_b 위치의 가로 벽(문 종류 "y")에서 끝난다.
    가로 벽(y = s)은 x = end_a, end_b 위치의 세로 벽(문 종류 "x")에서 끝난다.
    """
    for door in doors:
        if door.wall != end_wall:
            continue
        if end_wall == "y":
            on_end = abs(door.y - end_a) < 1e-6 or abs(door.y - end_b) < 1e-6
            along = door.x
        else:
            on_end = abs(door.x - end_a) < 1e-6 or abs(door.x - end_b) < 1e-6
            along = door.y
        if on_end and abs(along - s) < door.width / 2 + 0.3:
            return True
    return False


def _door_center(rng, lo: float, hi: float, p: LayoutParams) -> float:
    margin = 0.5 + p.door_width / 2   # 벽 끝(모서리)에서 0.5 m 이상 떨어뜨린다
    return round(rng.uniform(lo + margin, hi - margin), 2)


def _assign_rooms(rng, rects: list[Rect]) -> list[Room]:
    """넓이에 맞춰 방 종류를 정한다. 아직 안 나온 종류를 먼저 고른다."""
    rooms, used = [], set()
    for r in sorted(rects, key=lambda r: -(r.w * r.h)):   # 큰 방부터 정한다
        area = r.w * r.h
        if area < 13:
            options = ["storage", "bedroom", "office"]
        elif area < 20:
            options = ["office", "bedroom", "meeting", "living"]
        else:
            options = ["living", "meeting", "office"]
        fresh = [k for k in options if k not in used]
        kind = rng.choice(fresh or options)
        used.add(kind)
        rooms.append(Room(r.x0, r.y0, r.x1, r.y1, kind, furnish.ROOM_KINDS[kind]["floor"]))
    rooms.sort(key=lambda r: (r.y0, r.x0))
    return rooms


def _subtract(lo: float, hi: float, gaps: list[tuple[float, float]]) -> list[tuple[float, float]]:
    parts, cur = [], lo
    for g0, g1 in sorted(gaps):
        if g0 > cur:
            parts.append((cur, g0))
        cur = max(cur, g1)
    if hi > cur:
        parts.append((cur, hi))
    return [(a, b) for a, b in parts if b - a > 1e-6]


def _wall_boxes(segments: list[_Segment], p: LayoutParams) -> list[Box]:
    boxes: list[Box] = []
    t, h = p.wall_thickness, p.wall_height
    for seg in segments:
        horizontal = abs(seg.y1 - seg.y0) < 1e-9
        lo, hi = (seg.x0, seg.x1) if horizontal else (seg.y0, seg.y1)
        for a, b in _subtract(lo, hi, seg.gaps):
            n = max(1, math.ceil((b - a) / p.max_panel - 1e-9))
            step = (b - a) / n
            for k in range(n):
                c = round(a + step * (k + 0.5), 4)
                name = f"wall_{len(boxes):03d}"
                if horizontal:
                    boxes.append(Box(name, c, seg.y0, h / 2, round(step, 4), t, h))
                else:
                    boxes.append(Box(name, seg.x0, c, h / 2, t, round(step, 4), h))
    return boxes


def _lintels(doors: list[Door], p: LayoutParams) -> list[Box]:
    t, h0, H = p.wall_thickness, p.door_height, p.wall_height
    out = []
    for i, d in enumerate(doors):
        sx, sy = (t, d.width) if d.wall == "x" else (d.width, t)
        out.append(Box(f"lintel_{i:02d}", d.x, d.y, round((h0 + H) / 2, 4), sx, sy, round(H - h0, 4)))
    return out


# ---------------------------------------------------------------- 가구·잡동사니·사람

def _rect_gap(a, b) -> float:
    dx = max(b[0] - a[2], a[0] - b[2], 0.0)
    dy = max(b[1] - a[3], a[1] - b[3], 0.0)
    return math.hypot(dx, dy)


def _point_rect_dist(pt, r) -> float:
    dx = max(r[0] - pt[0], 0.0, pt[0] - r[2])
    dy = max(r[1] - pt[1], 0.0, pt[1] - r[3])
    return math.hypot(dx, dy)


def _wall_slot(rng, room: Room, w: float, d: float, p: LayoutParams):
    """벽에 등을 붙인 자리. 정면(지역 +y)이 방 안쪽을 본다."""
    t2, gap = p.wall_thickness / 2, 0.03
    side = rng.choice("SNWE")
    if side in "SN":
        lo, hi = room.x0 + t2 + gap + w / 2, room.x1 - t2 - gap - w / 2
        if lo > hi or room.h - 2 * (t2 + gap) < d:
            return None
        cx = rng.uniform(lo, hi)
        cy = room.y0 + t2 + gap + d / 2 if side == "S" else room.y1 - t2 - gap - d / 2
        return cx, cy, w, d, 0.0 if side == "S" else math.pi
    lo, hi = room.y0 + t2 + gap + w / 2, room.y1 - t2 - gap - w / 2
    if lo > hi or room.w - 2 * (t2 + gap) < d:
        return None
    cy = rng.uniform(lo, hi)
    cx = room.x0 + t2 + gap + d / 2 if side == "W" else room.x1 - t2 - gap - d / 2
    return cx, cy, d, w, -math.pi / 2 if side == "W" else math.pi / 2


def _center_slot(rng, room: Room, w: float, d: float, p: LayoutParams, margin: float):
    """벽에서 margin 이상 떨어진 방 가운데 자리. 90°로 돌려 놓을 수도 있다."""
    t2 = p.wall_thickness / 2
    turned = rng.random() < 0.5
    sx, sy, yaw = (d, w, math.pi / 2) if turned else (w, d, 0.0)
    lo_x, hi_x = room.x0 + t2 + margin + sx / 2, room.x1 - t2 - margin - sx / 2
    lo_y, hi_y = room.y0 + t2 + margin + sy / 2, room.y1 - t2 - margin - sy / 2
    if lo_x > hi_x or lo_y > hi_y:
        return None
    return rng.uniform(lo_x, hi_x), rng.uniform(lo_y, hi_y), sx, sy, yaw


def _try_place(rng, p, room, doors, grid, keypoints, placed, kind, where, name, hidden=False):
    for _ in range(25):
        w, d, h = furnish.sample_size(rng, kind)
        slot = _wall_slot(rng, room, w, d, p) if where == "wall" else \
            _center_slot(rng, room, w, d, p, margin=0.6 if where == "center" else 0.35)
        if slot is None:
            continue
        cx, cy, sx, sy, yaw = slot
        box = Box(name, round(cx, 3), round(cy, 3), round(h / 2, 3), round(sx, 3), round(sy, 3), round(h, 3))
        bounds = box.bounds2d()
        if any(_rect_gap(bounds, o.box.bounds2d()) < 0.05 for o in placed):
            continue
        if any(_point_rect_dist((dr.x, dr.y), bounds) < p.door_width for dr in doors):
            continue   # 문 앞은 비워 둔다
        trial = grid.copy()
        trial.mark_rect(*bounds, inflate=p.drone_radius)
        if not trial.connected(keypoints):
            continue   # 방과 방 사이 길을 끊으면 버린다
        return Obstacle(box, kind, round(yaw, 4), w, d, hidden, rng.randrange(1 << 30)), trial
    return None, grid


def _furnish(rng, p, rooms, doors, grid, keypoints):
    items: list[Obstacle] = []
    for room in rooms:
        plan = furnish.ROOM_KINDS[room.kind]
        wishlist = [("wall", kind) for kind, lo, hi in plan["wall"] for _ in range(rng.randint(lo, hi))]
        wishlist += [("center", kind) for kind, lo, hi in plan["center"] for _ in range(rng.randint(lo, hi))]
        for where, kind in wishlist:
            item, grid = _try_place(rng, p, room, doors, grid, keypoints, items, kind, where,
                                    f"obstacle_{len(items):02d}_{kind}")
            if item:
                items.append(item)
    return items, grid


def _place_clutter(rng, p, rooms, doors, grid, keypoints, furniture):
    """지도에 없는 물건(상자 더미·라바콘·카트·의자). 2단계 이상에서 숨긴다."""
    target = p.clutter if p.clutter is not None else max(1, round(len(rooms) * 0.8))
    clutter: list[Obstacle] = []
    for _ in range(target * 10):
        if len(clutter) >= target:
            break
        kind = rng.choice(furnish.CLUTTER)
        item, grid = _try_place(rng, p, rng.choice(rooms), doors, grid, keypoints, furniture + clutter,
                                kind, "loose", f"obstacle_{len(furniture) + len(clutter):02d}_{kind}", hidden=True)
        if item:
            clutter.append(item)
    return clutter, grid


PLAN_RADIUS = 0.25   # 비행 코드(nodes/mission_node.py)가 지도를 부풀리는 반지름


def _drone_route(p, walls, lintels, obstacles, spawn, goal) -> list[tuple[float, float, float]]:
    """드론이 2단계에서 받는 지도(숨긴 물건 제외)로 비행 코드와 같은 플래너가 짤 경로 (0.1 m 간격 x, y, z).

    플래너는 낮은 가구 위가 넓으면 비행 높이보다 조금 높이 지나기도 하므로 높이도 돌려준다.

    맵 생성기에서 비행 코드를 빌려 쓰는 유일한 곳이다. 같은 경로 위에 물건을 놓아야
    '받은 지도대로 가면 부딪히는' 시험이 된다.
    """
    from autonomy import PlanError, VoxelMap, plan

    def box(b: Box) -> dict:
        return {"x": b.x, "y": b.y, "z": b.z, "sx": b.sx, "sy": b.sy, "sz": b.sz}

    data = {"bounds": {"xmin": 0, "ymin": 0, "xmax": p.width, "ymax": p.depth, "zmax": p.wall_height},
            "walls": [box(b) for b in walls], "lintels": [box(b) for b in lintels],
            "obstacles": [box(o.box) for o in obstacles if not o.hidden]}
    h = p.flight_height
    try:
        inflated = VoxelMap.from_known_map(data, 0.1).inflate(PLAN_RADIUS)
        path = plan(inflated, (spawn["x"], spawn["y"], h), (goal["x"], goal["y"], h))
    except PlanError:
        return []
    route = []
    for a, b in zip(path, path[1:]):
        n = max(1, int(math.dist(a[:2], b[:2]) / 0.1))
        route += [tuple(float(a[c] + (b[c] - a[c]) * k / n) for c in range(3)) for k in range(n)]
    route.append(tuple(float(v) for v in path[-1]))
    return route


def _place_blockers(rng, p, doors, walls, lintels, grid, keypoints, spawn, goal, placed):
    """드론이 받은 지도로 짤 경로 위에 지도에 없는 물건(상자 더미·사람)을 놓는다 (2단계 회피 시험용).

    n번째 물건은 경로의 (n+1)/(blockers+1) 지점 근처에, 경로가 물건 가운데쪽(가장자리에서 0.15 m 이상
    안쪽)을 지나도록 둔다. 출발점·목적지·문 근처는 비우고, 놓은 뒤에도 실제로 목적지까지 갈 수 있어야 한다.
    """
    start, end = (spawn["x"], spawn["y"]), (goal["x"], goal["y"])
    route = _drone_route(p, walls, lintels, placed, spawn, goal)
    blockers: list[Obstacle] = []
    kinds = list(furnish.BLOCKERS)
    rng.shuffle(kinds)   # 상자 더미와 사람을 번갈아 놓는다
    for n in range(p.blockers):
        if len(route) < 3:
            break
        frac = (n + 1) / (p.blockers + 1)
        order = sorted(range(len(route)), key=lambda k: abs(k / (len(route) - 1) - frac))
        solids = [w.bounds2d() for w in walls] + [o.box.bounds2d() for o in placed + blockers]
        item = None
        for k in order:
            px, py, pz = route[k]
            if min(math.dist((px, py), start), math.dist((px, py), end)) < 1.2:
                continue
            if any(math.dist((px, py), (dr.x, dr.y)) < p.door_width + 0.2 for dr in doors):
                continue
            if any(math.dist((px, py), (o.box.x, o.box.y)) < 1.2 for o in blockers):
                continue
            kind = kinds[n % len(kinds)]
            w, d, h = furnish.sample_size(rng, kind)
            if kind == "box_stack":
                h = max(rng.uniform(1.2, 1.6), pz + 0.35)
            if h < pz + 0.35:   # 경로가 그 위로 넘어가면 막는 물건이 아니다
                continue
            turned = rng.random() < 0.5
            sx, sy, yaw = (d, w, math.pi / 2) if turned else (w, d, 0.0)
            # 벽·가구에 붙은 자리면 가운데를 조금 옮겨 본다 (경로는 계속 물건 가운데쪽을 지나야 한다)
            for ox, oy in _SHIFTS:
                if abs(ox) > sx / 2 - 0.15 or abs(oy) > sy / 2 - 0.15:
                    continue
                box = Box(f"obstacle_{len(placed) + len(blockers):02d}_{kind}", round(px + ox, 3), round(py + oy, 3),
                          round(h / 2, 3), round(sx, 3), round(sy, 3), round(h, 3))
                if any(_rect_gap(box.bounds2d(), s) < 0.05 for s in solids):
                    continue
                trial = grid.copy()
                trial.mark_rect(*box.bounds2d(), inflate=p.drone_radius)
                if trial.connected(keypoints + [start, end]):
                    item = Obstacle(box, kind, round(yaw, 4), w, d, True, rng.randrange(1 << 30), blocker=True)
                    break
            if item:
                blockers.append(item)
                grid = trial
                break
    return blockers, grid


_SHIFTS = sorted({(a, b) for a in (-0.3, -0.15, 0.0, 0.15, 0.3) for b in (-0.3, -0.15, 0.0, 0.15, 0.3)},
                 key=lambda o: math.hypot(*o))


def _choose_spawn(rng, rooms, grid: OccupancyGrid, keypoints):
    reach = grid.distances_from(*grid.cell_of(*keypoints[0])) if keypoints else None
    order = list(rooms)
    rng.shuffle(order)
    for room in order:
        cells = [(i, j) for i, j in grid.cells_in(room.x0, room.y0, room.x1, room.y1)
                 if reach is None or reach[j][i] >= 0]
        if cells:
            i, j = rng.choice(cells)
            x, y = grid.center_of(i, j)
            spawn = {"x": round(x, 2), "y": round(y, 2), "z": 0.1, "yaw": round(rng.uniform(-math.pi, math.pi), 3)}
            return spawn, grid.distances_from(i, j)
    raise RuntimeError("출발점을 놓을 빈 칸이 없습니다.")


def _choose_goal(rng, p: LayoutParams, grid: OccupancyGrid, dist):
    reachable = [(d, i, j) for j, row in enumerate(dist) for i, d in enumerate(row) if d > 0]
    if not reachable:
        return None
    far_limit = 0.6 * max(d for d, _, _ in reachable)
    i, j = rng.choice([(i, j) for d, i, j in reachable if d >= far_limit])
    x, y = grid.center_of(i, j)
    return {"x": round(x, 2), "y": round(y, 2), "z": p.flight_height}


def _place_humans(rng, p: LayoutParams, grid: OccupancyGrid, keypoints, spawn, solids) -> list[Human]:
    humans: list[Human] = []
    start = (spawn["x"], spawn["y"])
    dist = grid.distances_from(*grid.cell_of(*start))
    candidates = [(i, j) for j, row in enumerate(dist) for i, d in enumerate(row) if d >= 0]
    for _ in range(p.humans * 50):
        if len(humans) >= p.humans or not candidates:
            break
        x, y = grid.center_of(*rng.choice(candidates))
        if math.dist((x, y), start) < 1.5:
            continue
        if any(math.dist((x, y), (h.x, h.y)) < 1.0 for h in humans):
            continue
        if not grid.connected([start, (x, y)]):
            continue   # 앞서 놓은 사람 때문에 막힌 곳
        r = 0.25
        footprint = (x - r, y - r, x + r, y + r)
        if any(_rect_gap(footprint, s) < 0.1 for s in solids):
            continue
        trial = grid.copy()
        trial.mark_rect(*footprint, inflate=p.drone_radius)
        if not trial.connected(keypoints + [start]):
            continue   # 사람이 길을 막으면 버린다
        humans.append(Human(f"human_{len(humans)}", round(x, 2), round(y, 2),
                            round(rng.uniform(-math.pi, math.pi), 3), r))
        grid = trial
    return humans
