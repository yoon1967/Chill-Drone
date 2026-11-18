"""가구·잡동사니·벽 장식을 기본 도형(상자·원기둥·구)으로 조립한다.

가구의 충돌체는 바닥 면적 상자 하나이고, 보이는 모양만 여러 도형으로 만든다.
가구 지역 좌표: x = 폭(벽을 따라), y = 깊이(-d/2 벽 쪽 → +d/2 방 쪽), z = 바닥에서 위.
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass

Color = tuple[float, float, float]

# 방 종류별 바닥과 가구 목록. (가구, 최소 개수, 최대 개수)
ROOM_KINDS: dict[str, dict] = {
    "office": {"floor": "tile",
               "wall": [("desk", 1, 3), ("bookshelf", 0, 1), ("cabinet", 0, 1), ("plant", 0, 1)],
               "center": [], "decor": ["whiteboard", "picture"]},
    "living": {"floor": "wood",
               "wall": [("sofa", 1, 1), ("tv_stand", 1, 1), ("bookshelf", 0, 1), ("plant", 1, 2)],
               "center": [("coffee_table", 0, 1)], "decor": ["picture", "picture"]},
    "bedroom": {"floor": "wood",
                "wall": [("bed", 1, 1), ("wardrobe", 1, 1), ("desk", 0, 1), ("plant", 0, 1)],
                "center": [], "decor": ["picture"]},
    "meeting": {"floor": "carpet",
                "wall": [("cabinet", 0, 1), ("plant", 1, 2)],
                "center": [("meeting_table", 1, 1)], "decor": ["whiteboard", "picture"]},
    "storage": {"floor": "concrete",
                "wall": [("rack", 2, 3)],
                "center": [("box_stack", 0, 2)], "decor": []},
}

# 지도에 없는 장애물로 쓰는 '새로 놓인 물건'
CLUTTER = ["box_stack", "cone", "cart", "chair"]
BLOCKERS = ["box_stack", "person"]   # --blockers로 경로 위에 놓는 물건: 비행 높이(1 m)보다 확실히 높은 것

# (폭, 깊이, 높이). 튜플은 범위, 숫자는 고정값. 책상·회의 탁자 깊이에는 의자 자리가 들어 있다.
SPECS: dict[str, tuple] = {
    "desk": ((1.2, 1.6), (1.2, 1.35), 1.25),
    "bookshelf": ((0.8, 1.2), 0.32, (1.8, 2.0)),
    "cabinet": ((0.8, 1.2), 0.45, (0.9, 1.1)),
    "wardrobe": ((1.0, 1.4), 0.6, (1.9, 2.1)),
    "sofa": ((1.6, 2.2), 0.88, 0.85),
    "tv_stand": ((1.2, 1.6), 0.42, 1.2),
    "bed": ((1.4, 1.6), 2.05, 1.0),
    "plant": (0.6, 0.6, 1.1),
    "rack": ((1.2, 1.8), 0.5, (1.8, 2.0)),
    "coffee_table": ((0.9, 1.2), (0.5, 0.6), 0.5),
    "meeting_table": ((1.6, 2.4), (2.0, 2.1), 1.0),
    "box_stack": ((0.5, 0.65), (0.5, 0.65), (0.45, 1.3)),
    "cone": (0.4, 0.4, 0.7),
    "cart": (0.9, 0.55, 0.85),
    "chair": (0.6, 0.6, 0.96),        # 등받이 꼭대기 높이
    "person": (0.6, 0.6, 1.75),       # 모양은 sdf.py가 사람 모델(actor)로 넣는다
}

WOODS: list[Color] = [(0.55, 0.38, 0.22), (0.66, 0.50, 0.33), (0.40, 0.27, 0.16), (0.80, 0.68, 0.50)]
FABRICS: list[Color] = [(0.35, 0.40, 0.48), (0.55, 0.30, 0.28), (0.40, 0.45, 0.35), (0.62, 0.60, 0.56), (0.25, 0.25, 0.28)]
BOOKS: list[Color] = [(0.55, 0.12, 0.12), (0.12, 0.25, 0.50), (0.15, 0.40, 0.20), (0.75, 0.60, 0.20),
                      (0.30, 0.30, 0.32), (0.85, 0.82, 0.75), (0.45, 0.20, 0.40), (0.20, 0.45, 0.50)]
GREENS: list[Color] = [(0.18, 0.42, 0.18), (0.25, 0.50, 0.20), (0.15, 0.35, 0.22)]
WHITE: Color = (0.92, 0.92, 0.90)
TRIM: Color = (0.84, 0.84, 0.82)
DARK: Color = (0.14, 0.14, 0.15)
BLACK: Color = (0.05, 0.05, 0.06)
METAL: Color = (0.62, 0.64, 0.67)
CARDBOARD: Color = (0.67, 0.50, 0.32)
ORANGE: Color = (0.95, 0.40, 0.08)


@dataclass(frozen=True)
class Part:
    shape: str                      # "box" | "cylinder" | "sphere"
    x: float                        # 월드 좌표 중심
    y: float
    z: float
    yaw: float
    size: tuple[float, ...]         # box (sx, sy, sz) / cylinder (반지름, 높이) / sphere (반지름,)
    color: Color = (0.8, 0.8, 0.8)
    texture: str | None = None      # export가 파일 경로로 바꾸는 텍스처 이름
    emissive: Color | None = None
    transparency: float = 0.0


class _Builder:
    """지역 좌표로 도형을 놓으면 월드 좌표 Part로 바꿔 모은다."""

    def __init__(self, cx: float, cy: float, yaw: float):
        self.cx, self.cy, self.yaw = cx, cy, yaw
        self.c, self.s = math.cos(yaw), math.sin(yaw)
        self.parts: list[Part] = []

    def _world(self, lx: float, ly: float) -> tuple[float, float]:
        return (round(self.cx + self.c * lx - self.s * ly, 4), round(self.cy + self.s * lx + self.c * ly, 4))

    def box(self, lx, ly, z, sx, sy, sz, color, texture=None, emissive=None, transparency=0.0):
        x, y = self._world(lx, ly)
        self.parts.append(Part("box", x, y, round(z, 4), round(self.yaw, 4),
                               (round(sx, 4), round(sy, 4), round(sz, 4)), color, texture, emissive, transparency))

    def cylinder(self, lx, ly, z, radius, length, color):
        x, y = self._world(lx, ly)
        self.parts.append(Part("cylinder", x, y, round(z, 4), 0.0, (round(radius, 4), round(length, 4)), color))

    def sphere(self, lx, ly, z, radius, color):
        x, y = self._world(lx, ly)
        self.parts.append(Part("sphere", x, y, round(z, 4), 0.0, (round(radius, 4),), color))


def _pick(rng, value) -> float:
    return round(rng.uniform(*value), 2) if isinstance(value, tuple) else value


def sample_size(rng, kind: str) -> tuple[float, float, float]:
    w, d, h = SPECS[kind]
    return _pick(rng, w), _pick(rng, d), _pick(rng, h)


def _lighter(color: Color, amount: float = 0.08) -> Color:
    return tuple(min(1.0, c + amount) for c in color)


# ---------------------------------------------------------------- 가구

def _chair(b: _Builder, lx: float, ly: float, back: int, rng) -> None:
    """사무용 의자. back=+1이면 등받이가 +y 쪽."""
    fabric = rng.choice(FABRICS)
    b.cylinder(lx, ly, 0.02, 0.26, 0.04, DARK)
    b.cylinder(lx, ly, 0.25, 0.03, 0.42, METAL)
    b.box(lx, ly, 0.48, 0.46, 0.46, 0.06, fabric)
    b.box(lx, ly + back * 0.22, 0.74, 0.44, 0.05, 0.44, fabric)


def _desk(b, w, depth, h, rng):
    d = depth - 0.55
    top = 0.75
    wood = rng.choice(WOODS)
    yd = -depth / 2 + d / 2
    b.box(0, yd, top - 0.015, w, d, 0.03, wood)
    for sx in (-1, 1):
        for sy in (-1, 1):
            b.box(sx * (w / 2 - 0.04), yd + sy * (d / 2 - 0.04), (top - 0.03) / 2, 0.05, 0.05, top - 0.03, DARK)
    my = -depth / 2 + 0.12
    b.box(0, my, top + 0.09, 0.06, 0.06, 0.15, DARK)
    b.box(0, my, top + 0.33, 0.56, 0.035, 0.33, BLACK)
    b.box(0, my + 0.02, top + 0.33, 0.52, 0.006, 0.29, (0.10, 0.16, 0.25), emissive=(0.05, 0.08, 0.12))
    b.box(0, yd + 0.12, top + 0.01, 0.45, 0.15, 0.02, (0.22, 0.22, 0.24))
    if rng.random() < 0.6:
        b.box(w / 2 - 0.25, yd, top + 0.05, 0.22, 0.3, 0.08, rng.choice(BOOKS))     # 서류 더미
    _chair(b, rng.uniform(-0.15, 0.15), -depth / 2 + d + 0.28, +1, rng)


def _bookshelf(b, w, d, h, rng):
    wood = rng.choice(WOODS)
    for sx in (-1, 1):
        b.box(sx * (w / 2 - 0.01), 0, h / 2, 0.02, d, h, wood)
    b.box(0, -d / 2 + 0.005, h / 2, w, 0.01, h, _lighter(wood, -0.08))
    levels = 5
    gap = (h - 0.04) / levels
    for k in range(levels + 1):
        b.box(0, 0, 0.01 + k * gap, w - 0.04, d, 0.02, wood)
    for k in range(levels):
        if rng.random() < 0.15:
            continue
        x = -w / 2 + 0.03
        floor = 0.02 + k * gap
        while x < w / 2 - 0.06:
            bw = rng.uniform(0.022, 0.05)
            if rng.random() < 0.08:          # 빈 칸
                x += rng.uniform(0.08, 0.2)
                continue
            bh = gap * rng.uniform(0.6, 0.88)
            bd = d * rng.uniform(0.7, 0.85)
            b.box(x + bw / 2, -d / 2 + 0.015 + bd / 2, floor + bh / 2, bw - 0.003, bd, bh, rng.choice(BOOKS))
            x += bw


def _cabinet(b, w, d, h, rng, doors: int = 2):
    body = rng.choice([WHITE, *WOODS])
    front = d / 2 - 0.025                       # 손잡이 자리를 남기고 몸통을 뒤로 붙인다
    b.box(0, -0.0125, (h - 0.02) / 2, w, d - 0.025, h - 0.02, body)
    b.box(0, -0.0125, h - 0.01, w, d - 0.025, 0.02, _lighter(body, -0.1))
    for k in range(1, doors):
        b.box(-w / 2 + k * w / doors, front + 0.002, h / 2, 0.006, 0.004, h - 0.08, DARK)
    for k in range(doors):
        cx = -w / 2 + (k + 0.5) * w / doors
        hx = cx + (w / doors * 0.35 if k % 2 == 0 else -w / doors * 0.35)
        b.box(hx, front + 0.012, h * 0.55, 0.02, 0.02, 0.14, METAL)


def _wardrobe(b, w, d, h, rng):
    _cabinet(b, w, d, h, rng, doors=2 if w < 1.2 else 3)


def _sofa(b, w, d, h, rng):
    fabric = rng.choice(FABRICS)
    b.box(0, 0, 0.21, w, d, 0.42, fabric)
    b.box(0, -d / 2 + 0.1, 0.63, w, 0.2, 0.42, fabric)
    for sx in (-1, 1):
        b.box(sx * (w / 2 - 0.09), 0, 0.31, 0.18, d, 0.62, fabric)
    seats = 3 if w > 1.9 else 2
    inner = w - 0.36
    for k in range(seats):
        cx = -inner / 2 + (k + 0.5) * inner / seats
        b.box(cx, 0.06, 0.48, inner / seats - 0.02, d - 0.28, 0.12, _lighter(fabric))
    for sx in (-1, 1):
        b.box(sx * (w / 2 - 0.35), -d / 2 + 0.28, 0.66, 0.4, 0.12, 0.34, rng.choice(FABRICS))


def _tv_stand(b, w, d, h, rng):
    wood = rng.choice(WOODS)
    b.box(0, 0, 0.225, w, d, 0.45, wood)
    b.box(0, d / 2 + 0.002, 0.225, 0.006, 0.004, 0.38, DARK)
    tv_w = min(w - 0.1, 1.25)
    b.box(0, -d / 2 + 0.12, 0.48, 0.3, 0.15, 0.06, BLACK)
    b.box(0, -d / 2 + 0.12, h - 0.36, tv_w, 0.05, 0.66, BLACK)
    b.box(0, -d / 2 + 0.147, h - 0.36, tv_w - 0.04, 0.006, 0.62, (0.06, 0.08, 0.12), emissive=(0.02, 0.03, 0.05))


def _bed(b, w, d, h, rng):
    wood = rng.choice(WOODS)
    b.box(0, 0, 0.15, w, d, 0.3, wood)
    b.box(0, 0.02, 0.4, w - 0.06, d - 0.08, 0.2, WHITE)
    blanket = rng.choice(FABRICS)
    b.box(0, d / 2 - d * 0.3 - 0.02, 0.52, w - 0.02, d * 0.6, 0.06, blanket)
    pillows = 2 if w >= 1.5 else 1
    for k in range(pillows):
        cx = -w / 2 + (k + 0.5) * w / pillows
        b.box(cx, -d / 2 + 0.3, 0.57, w / pillows - 0.15, 0.35, 0.12, _lighter(WHITE, -0.04))
    b.box(0, -d / 2 + 0.03, h / 2, w, 0.06, h, wood)


def _plant(b, w, d, h, rng):
    b.cylinder(0, 0, 0.175, 0.18, 0.35, (0.62, 0.33, 0.20))
    b.cylinder(0, 0, 0.355, 0.16, 0.02, (0.20, 0.13, 0.08))
    b.cylinder(0, 0, 0.5, 0.02, 0.35, (0.30, 0.22, 0.12))
    for _ in range(3):
        b.sphere(rng.uniform(-0.06, 0.06), rng.uniform(-0.06, 0.06), rng.uniform(0.65, 0.86),
                 rng.uniform(0.16, 0.22), rng.choice(GREENS))


def _rack(b, w, d, h, rng):
    for sx in (-1, 1):
        for sy in (-1, 1):
            b.box(sx * (w / 2 - 0.02), sy * (d / 2 - 0.02), h / 2, 0.03, 0.03, h, METAL)
    levels = 4
    gap = (h - 0.1) / (levels - 1)
    for k in range(levels):
        z = 0.08 + k * gap
        b.box(0, 0, z, w, d, 0.02, METAL)
        if k == levels - 1:
            continue
        x = -w / 2 + 0.05
        while x < w / 2 - 0.25:
            bw = rng.uniform(0.25, 0.45)
            if x + bw > w / 2 - 0.05:
                break
            bh = min(gap - 0.06, rng.uniform(0.18, 0.35))
            b.box(x + bw / 2, 0, z + 0.01 + bh / 2, bw - 0.02, d - 0.08, bh, CARDBOARD, texture="cardboard")
            x += bw + rng.uniform(0.02, 0.1)


def _coffee_table(b, w, d, h, rng):
    wood = rng.choice(WOODS)
    top = h - 0.05                              # 위에 올린 책까지 h 안에 들어간다
    b.box(0, 0, top - 0.02, w, d, 0.04, wood)
    for sx in (-1, 1):
        for sy in (-1, 1):
            b.box(sx * (w / 2 - 0.05), sy * (d / 2 - 0.05), (top - 0.04) / 2, 0.04, 0.04, top - 0.04, wood)
    if rng.random() < 0.6:
        b.box(rng.uniform(-0.2, 0.2), 0, top + 0.015, 0.3, 0.22, 0.03, rng.choice(BOOKS))


def _meeting_table(b, w, d, h, rng):
    table_d = d - 1.1
    wood = rng.choice(WOODS)
    b.box(0, 0, 0.73, w, table_d, 0.04, wood)
    for sx in (-1, 1):
        b.box(sx * (w / 2 - 0.3), 0, 0.355, 0.1, table_d * 0.6, 0.71, DARK)
    per_side = max(1, int(w / 0.75))
    for side in (-1, 1):
        for k in range(per_side):
            lx = -w / 2 + (k + 0.5) * w / per_side
            _chair(b, lx, side * (table_d / 2 + 0.3), side, rng)


def _box_stack(b, w, d, h, rng):
    z = 0.0
    while z < h - 0.05:
        bh = min(h - z, rng.uniform(0.35, 0.55))
        bw = min(w, rng.uniform(0.45, 0.62))
        bd = min(d, rng.uniform(0.45, 0.62))
        b.box(rng.uniform(-(w - bw) / 2, (w - bw) / 2), rng.uniform(-(d - bd) / 2, (d - bd) / 2),
              z + bh / 2, bw, bd, bh, CARDBOARD, texture="cardboard")
        z += bh


def _cone(b, w, d, h, rng):
    b.box(0, 0, 0.015, 0.38, 0.38, 0.03, (0.15, 0.15, 0.15))
    b.cylinder(0, 0, 0.14, 0.15, 0.22, ORANGE)
    b.cylinder(0, 0, 0.33, 0.115, 0.16, WHITE)
    b.cylinder(0, 0, 0.53, 0.08, 0.24, ORANGE)


def _cart(b, w, d, h, rng):
    body = rng.choice([(0.15, 0.30, 0.60), (0.55, 0.12, 0.12), METAL])
    for sx in (-1, 1):
        for sy in (-1, 1):
            b.cylinder(sx * (w / 2 - 0.08), sy * (d / 2 - 0.08), 0.05, 0.05, 0.1, BLACK)
    b.box(0, 0, 0.2, w, d, 0.05, body)
    for sy in (-1, 1):
        b.box(-w / 2 + 0.03, sy * (d / 2 - 0.04), (0.2 + h) / 2, 0.03, 0.03, h - 0.2, METAL)
    b.box(-w / 2 + 0.03, 0, h - 0.015, 0.03, d - 0.05, 0.03, METAL)
    b.box(0.08, 0, 0.42, 0.5, 0.4, 0.4, CARDBOARD, texture="cardboard")


def _loose_chair(b, w, d, h, rng):
    _chair(b, 0, 0, rng.choice((-1, 1)), rng)


def _person(b, w, d, h, rng):
    """보이는 모양은 sdf.py가 사람 모델로 따로 넣는다 (여기서는 충돌 상자만)."""


BUILDERS = {
    "desk": _desk, "bookshelf": _bookshelf, "cabinet": _cabinet, "wardrobe": _wardrobe,
    "sofa": _sofa, "tv_stand": _tv_stand, "bed": _bed, "plant": _plant, "rack": _rack,
    "coffee_table": _coffee_table, "meeting_table": _meeting_table,
    "box_stack": _box_stack, "cone": _cone, "cart": _cart, "chair": _loose_chair, "person": _person,
}


def build(kind: str, cx: float, cy: float, yaw: float, w: float, d: float, h: float, seed: int) -> list[Part]:
    b = _Builder(cx, cy, yaw)
    BUILDERS[kind](b, w, d, h, random.Random(seed))
    return b.parts


# ---------------------------------------------------------------- 벽 장식 (충돌체 없음)

def _side_frame(room, side: str, t: float):
    """방의 한 변(S/N/W/E)에 붙일 장식의 좌표계: (벽면 위치, 벽 방향 범위, 방 안쪽을 향하는 yaw)."""
    if side == "S":
        return ("y", room.y0 + t / 2, room.x0, room.x1, 0.0)
    if side == "N":
        return ("y", room.y1 - t / 2, room.x0, room.x1, math.pi)
    if side == "W":
        return ("x", room.x0 + t / 2, room.y0, room.y1, -math.pi / 2)
    return ("x", room.x1 - t / 2, room.y0, room.y1, math.pi / 2)


def _subtract(intervals, a, b):
    out = []
    for lo, hi in intervals:
        if b <= lo or a >= hi:
            out.append((lo, hi))
            continue
        if a > lo:
            out.append((lo, a))
        if b < hi:
            out.append((b, hi))
    return out


def _free_intervals(room, side, p, doors, obstacles):
    axis, face, lo, hi, _ = _side_frame(room, side, p.wall_thickness)
    line = face - (p.wall_thickness / 2 if side in "SW" else -p.wall_thickness / 2)
    free = [(lo + p.wall_thickness / 2 + 0.15, hi - p.wall_thickness / 2 - 0.15)]
    for door in doors:
        on_line = (door.wall == "y" and axis == "y" and abs(door.y - line) < 1e-6) or \
                  (door.wall == "x" and axis == "x" and abs(door.x - line) < 1e-6)
        if on_line:
            c = door.x if axis == "y" else door.y
            free = _subtract(free, c - door.width / 2 - 0.25, c + door.width / 2 + 0.25)
    for o in obstacles:   # 이 방 안에서 벽 앞을 가리는 키 큰 가구가 있으면 그 구간은 비운다
        if o.box.sz <= 1.0 or not (room.x0 <= o.box.x <= room.x1 and room.y0 <= o.box.y <= room.y1):
            continue
        x0, y0, x1, y1 = o.box.bounds2d()
        gap = max(y0 - face, face - y1, 0.0) if axis == "y" else max(x0 - face, face - x1, 0.0)
        if gap < 0.5:
            a, c = (x0, x1) if axis == "y" else (y0, y1)
            free = _subtract(free, a - 0.05, c + 0.05)
    return [(a, b) for a, b in free if b - a > 0.3]


def _wall_item(rng, room, side, p, along, kind, art_count) -> list[Part]:
    axis, face, _, _, yaw = _side_frame(room, side, p.wall_thickness)
    cx, cy = (along, face) if axis == "y" else (face, along)
    b = _Builder(cx, cy, yaw)
    if kind == "window":
        w, h, zc = 1.2, 1.1, 1.45
        b.box(0, 0.01, zc, w - 0.08, 0.01, h - 0.08, (0.62, 0.78, 0.92), emissive=(0.35, 0.45, 0.55), transparency=0.15)
        for sz in (-1, 1):
            b.box(0, 0.02, zc + sz * (h / 2 - 0.03), w, 0.04, 0.06, WHITE)
        for sx in (-1, 1):
            b.box(sx * (w / 2 - 0.03), 0.02, zc, 0.06, 0.04, h, WHITE)
        b.box(0, 0.025, zc, 0.03, 0.03, h - 0.08, WHITE)
        b.box(0, 0.06, zc - h / 2 - 0.015, w + 0.1, 0.12, 0.03, WHITE)
    elif kind == "picture":
        w, h = rng.uniform(0.5, 0.9), rng.uniform(0.4, 0.65)
        zc = rng.uniform(1.45, 1.65)
        b.box(0, 0.015, zc, w + 0.06, 0.03, h + 0.06, rng.choice([DARK, WOODS[2], WHITE]))
        b.box(0, 0.032, zc, w, 0.005, h, WHITE, texture=f"art_{rng.randrange(art_count)}")
    elif kind == "whiteboard":
        zc = 1.4
        b.box(0, 0.0125, zc, 1.66, 0.025, 1.06, METAL)
        b.box(0, 0.027, zc, 1.6, 0.005, 1.0, WHITE, texture="whiteboard")
        b.box(0, 0.04, zc - 0.53, 0.6, 0.06, 0.02, METAL)
    return b.parts


DECOR_WIDTH = {"window": 1.4, "picture": 1.0, "whiteboard": 1.9}


def decorations(rng, p, rooms, doors, walls, obstacles, art_count: int) -> list[Part]:
    """걸레받이, 문틀, 창문, 그림, 화이트보드. 모두 충돌체가 없는 장식이다."""
    t = p.wall_thickness
    parts: list[Part] = []
    for w in walls:
        if w.sx >= w.sy:
            parts.append(Part("box", w.x, w.y, 0.04, 0.0, (w.sx, round(w.sy + 0.024, 4), 0.08), TRIM))
        else:
            parts.append(Part("box", w.x, w.y, 0.04, 0.0, (round(w.sx + 0.024, 4), w.sy, 0.08), TRIM))
    for door in doors:
        hz = p.door_height
        for s in (-1, 1):
            if door.wall == "x":
                parts.append(Part("box", door.x, round(door.y + s * (door.width / 2 + 0.03), 4), hz / 2, 0.0,
                                  (t + 0.03, 0.06, hz), TRIM))
            else:
                parts.append(Part("box", round(door.x + s * (door.width / 2 + 0.03), 4), door.y, hz / 2, 0.0,
                                  (0.06, t + 0.03, hz), TRIM))
        size = (t + 0.03, door.width + 0.12, 0.06) if door.wall == "x" else (door.width + 0.12, t + 0.03, 0.06)
        parts.append(Part("box", door.x, door.y, round(hz + 0.03, 4), 0.0, size, TRIM))

    width, depth = p.width, p.depth
    for room in rooms:
        free = {side: _free_intervals(room, side, p, doors, obstacles) for side in "SNWE"}
        exterior = {"S": room.y0 == 0, "N": abs(room.y1 - depth) < 1e-6,
                    "W": room.x0 == 0, "E": abs(room.x1 - width) < 1e-6}
        wishes = [("window", s) for s in "SNWE" if exterior[s] and rng.random() < 0.8]
        wishes += [(kind, None) for kind in ROOM_KINDS[room.kind]["decor"]]
        for kind, want_side in wishes:
            need = DECOR_WIDTH[kind]
            options = [(s, a, b) for s in ([want_side] if want_side else "SNWE")
                       for a, b in free[s] if b - a >= need]
            if not options:
                continue
            side, a, b = rng.choice(options)
            along = round(rng.uniform(a + need / 2, b - need / 2), 3)
            parts += _wall_item(rng, room, side, p, along, kind, art_count)
            free[side] = _subtract(free[side], along - need / 2, along + need / 2)
    return parts
