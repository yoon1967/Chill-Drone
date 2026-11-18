"""표준 라이브러리만으로 PNG를 쓰고, 실내 재질 텍스처와 위에서 본 미리보기를 그린다."""
from __future__ import annotations

import math
import struct
import zlib
from pathlib import Path

Rows = list[bytearray]   # 한 줄에 width*3 바이트(RGB)
RGB = tuple[int, int, int]


def write_png(path: Path, width: int, height: int, rows: Rows) -> None:
    raw = b"".join(b"\x00" + bytes(row) for row in rows)

    def chunk(tag: bytes, data: bytes) -> bytes:
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

    header = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    Path(path).write_bytes(
        b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", header) + chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b"")
    )


def _clamp(v: float) -> int:
    return 0 if v < 0 else 255 if v > 255 else int(v)


def _shade(color: RGB, delta: float) -> bytes:
    return bytes(_clamp(c + delta) for c in color)


def _jitter(rng, color: RGB, amount: int) -> RGB:
    d = rng.randint(-amount, amount)
    return tuple(_clamp(c + d + rng.randint(-amount // 3, amount // 3)) for c in color)


def _noise_rows(rng, width: int, base: RGB, amp: int, count: int = 24) -> list[bytes]:
    """미리 만든 잡음 줄. 큰 텍스처는 이 줄을 무작위로 밀어 가며 재사용해 빨리 만든다."""
    out = []
    for _ in range(count):
        row = bytearray()
        for _ in range(width):
            d = rng.randint(-amp, amp)
            row += bytes(_clamp(c + d) for c in base)
        out.append(bytes(row))
    return out


def _noisy(rng, width: int, height: int, base: RGB, amp: int) -> Rows:
    pool = _noise_rows(rng, width, base, amp)
    rows = []
    for _ in range(height):
        src = rng.choice(pool)
        off = rng.randrange(width) * 3
        rows.append(bytearray(src[off:] + src[:off]))
    return rows


def _fill(rows: Rows, x: int, y: int, w: int, h: int, color: bytes) -> None:
    height, width = len(rows), len(rows[0]) // 3
    x0, x1 = max(0, x), min(width, x + w)
    if x1 <= x0:
        return
    fill = color * (x1 - x0)
    for r in range(max(0, y), min(height, y + h)):
        rows[r][x0 * 3:x1 * 3] = fill


def _disc(rows: Rows, cx: float, cy: float, radius: float, color: bytes) -> None:
    height, width = len(rows), len(rows[0]) // 3
    for r in range(max(0, int(cy - radius)), min(height, int(cy + radius) + 1)):
        half = radius * radius - (r - cy) ** 2
        if half <= 0:
            continue
        half = math.sqrt(half)
        x0, x1 = max(0, int(cx - half)), min(width, int(cx + half) + 1)
        if x1 > x0:
            rows[r][x0 * 3:x1 * 3] = color * (x1 - x0)


def _line(rows: Rows, x0: float, y0: float, x1: float, y1: float, color: bytes, thick: int = 2) -> None:
    steps = int(max(abs(x1 - x0), abs(y1 - y0))) + 1
    for k in range(steps + 1):
        t = k / steps
        _fill(rows, int(x0 + (x1 - x0) * t), int(y0 + (y1 - y0) * t), thick, thick, color)


# ---------------------------------------------------------------- 바닥

def wood_floor(rng, width: int, height: int, ppm: int) -> Rows:
    """폭 약 18 cm 마루판. 판마다 색이 조금 다르고 이음매가 있다."""
    base = rng.choice([(150, 105, 62), (168, 122, 76), (128, 88, 52), (182, 142, 96)])
    plank = max(4, int(0.18 * ppm))
    seam = _shade(base, -60)
    rows: Rows = []
    while len(rows) < height:
        segments, x = [], -rng.randint(0, int(1.5 * ppm))
        while x < width:
            length = rng.randint(int(1.0 * ppm), int(2.0 * ppm))
            segments.append((max(0, x), min(width, x + length), _jitter(rng, base, 14)))
            x += length
        for r in range(plank):
            if len(rows) >= height:
                break
            row = bytearray(width * 3)
            grain = rng.randint(-5, 5)
            for a, b, color in segments:
                row[a * 3:b * 3] = _shade(color, grain) * (b - a)
                if a > 0:
                    row[a * 3:a * 3 + 3] = seam
            rows.append(bytearray(seam * width) if r == plank - 1 else row)
    return rows


def tile_floor(rng, width: int, height: int, ppm: int, size_m: float = 0.6) -> Rows:
    """60 cm 바닥 타일과 줄눈."""
    base = rng.choice([(214, 208, 196), (196, 196, 192), (226, 220, 206)])
    grout = _shade(base, -55)
    s = max(8, int(size_m * ppm))
    nx, ny = width // s + 2, height // s + 2
    colors = [[_shade(_jitter(rng, base, 6), 0) for _ in range(nx)] for _ in range(ny)]
    rows: Rows = []
    for y in range(height):
        if y % s < 2:
            rows.append(bytearray(grout * width))
            continue
        row = bytearray(width * 3)
        for tx in range(nx):
            a, b = tx * s, min(width, (tx + 1) * s)
            if a >= width:
                break
            row[a * 3:b * 3] = colors[y // s][tx] * (b - a)
            row[a * 3:min(width, a + 2) * 3] = grout * (min(width, a + 2) - a)
        rows.append(row)
    for _ in range(width * height // 400):
        _fill(rows, rng.randrange(width), rng.randrange(height), 1, 1, _shade(base, rng.choice((-30, 25))))
    return rows


def carpet_floor(rng, width: int, height: int) -> Rows:
    base = rng.choice([(96, 104, 118), (120, 112, 104), (88, 100, 92), (110, 110, 116)])
    return _noisy(rng, width, height, base, 14)


def concrete_floor(rng, width: int, height: int) -> Rows:
    base = (150, 150, 146)
    rows = _noisy(rng, width, height, base, 9)
    for _ in range(max(6, width * height // 6000)):
        r = rng.uniform(6, 30)
        _disc(rows, rng.uniform(0, width), rng.uniform(0, height), r, _shade(base, rng.choice((-14, -9, 8, 12))))
    return rows


# ---------------------------------------------------------------- 벽·물건

def paint(rng, color: RGB, size: int = 128) -> Rows:
    """칠한 벽. 거의 단색이라 시각 SLAM에는 실제처럼 특징점이 적다."""
    return _noisy(rng, size, size, color, 3)


def cardboard(rng, size: int = 128) -> Rows:
    base = (172, 128, 82)
    rows = _noisy(rng, size, size, base, 7)
    _fill(rows, 0, size // 2 - 10, size, 20, _shade(base, 30))                 # 테이프
    _fill(rows, size // 8, size // 8, size // 3, size // 5, b"\xee\xee\xe8")   # 라벨
    for k in range(3):
        _fill(rows, size // 8 + 4, size // 8 + 5 + k * 6, size // 4, 2, b"\x40\x40\x40")
    return rows


def whiteboard(rng, width: int = 320, height: int = 200) -> Rows:
    rows = [bytearray(b"\xf4\xf4\xf2" * width) for _ in range(height)]
    for _ in range(rng.randint(4, 8)):   # 글씨처럼 보이는 선
        color = rng.choice([b"\x20\x30\x90", b"\x20\x20\x20", b"\xa0\x20\x20"])
        x, y = rng.uniform(10, width * 0.6), rng.uniform(15, height - 20)
        for _ in range(rng.randint(5, 12)):
            nx, ny = x + rng.uniform(6, 22), y + rng.uniform(-6, 6)
            _line(rows, x, y, nx, ny, color)
            x, y = nx, ny
    for _ in range(rng.randint(1, 3)):   # 포스트잇
        _fill(rows, rng.randrange(width - 30), rng.randrange(height - 30), 26, 26, b"\xf2\xe0\x50")
    if rng.random() < 0.6:               # 표
        x0, y0 = int(width * 0.62), int(height * 0.2)
        for k in range(4):
            _line(rows, x0, y0 + k * 22, x0 + 90, y0 + k * 22, b"\x30\x30\x30", 1)
        _line(rows, x0, y0, x0, y0 + 66, b"\x30\x30\x30", 1)
        _line(rows, x0 + 45, y0, x0 + 45, y0 + 66, b"\x30\x30\x30", 1)
    return rows


def artwork(rng, width: int = 256, height: int = 192) -> Rows:
    """액자 속 그림: 풍경화 또는 추상화."""
    if rng.random() < 0.6:
        top, bottom = _jitter(rng, (110, 160, 215), 20), _jitter(rng, (220, 200, 170), 20)
        rows = []
        for y in range(height):
            t = y / height
            rows.append(bytearray(bytes(_clamp(a + (b - a) * t) for a, b in zip(top, bottom)) * width))
        _disc(rows, rng.uniform(0.2, 0.8) * width, rng.uniform(0.15, 0.4) * height, rng.uniform(10, 22), b"\xf6\xe0\xa0")
        for layer, base in enumerate([(90, 110, 130), (70, 110, 70), (50, 85, 50)]):
            color = _shade(_jitter(rng, base, 12), 0)
            phase, freq = rng.uniform(0, 6.28), rng.uniform(1.5, 4.0)
            level = height * (0.45 + 0.15 * layer)
            for x in range(width):
                top_y = int(level + 18 * math.sin(phase + freq * x / width * 6.28) * (1 - 0.3 * layer))
                for y in range(max(0, top_y), height):
                    rows[y][x * 3:x * 3 + 3] = color
        return rows
    base = _jitter(rng, (230, 225, 210), 10)
    rows = [bytearray(_shade(base, 0) * width) for _ in range(height)]
    palette = [(200, 40, 40), (30, 60, 150), (240, 190, 40), (20, 20, 20), (60, 140, 90)]
    for _ in range(rng.randint(5, 9)):
        color = _shade(rng.choice(palette), rng.randint(-15, 15))
        if rng.random() < 0.6:
            _fill(rows, rng.randrange(width), rng.randrange(height), rng.randint(20, 90), rng.randint(20, 90), color)
        else:
            _disc(rows, rng.uniform(0, width), rng.uniform(0, height), rng.uniform(12, 40), color)
    return rows


# ---------------------------------------------------------------- 미리보기

class TopView:
    """위에서 본 맵 그림. y가 위로 가도록 뒤집어 그린다."""

    def __init__(self, width_m: float, depth_m: float, px_per_m: int = 40):
        self.s = px_per_m
        self.w = math.ceil(width_m * px_per_m)
        self.h = math.ceil(depth_m * px_per_m)
        self.rows: Rows = [bytearray(b"\xf5\xf5\xf5" * self.w) for _ in range(self.h)]

    def rect(self, xmin: float, ymin: float, xmax: float, ymax: float, color: tuple[int, int, int]) -> None:
        c0 = max(0, int(xmin * self.s))
        c1 = min(self.w, max(c0 + 1, int(math.ceil(xmax * self.s))))
        r0 = max(0, self.h - int(math.ceil(ymax * self.s)))
        r1 = min(self.h, max(r0 + 1, self.h - int(ymin * self.s)))
        if c0 >= self.w or r0 >= self.h:
            return
        fill = bytes(color) * (c1 - c0)
        for r in range(r0, r1):
            self.rows[r][c0 * 3:c1 * 3] = fill

    def save(self, path: Path) -> None:
        write_png(path, self.w, self.h, self.rows)
