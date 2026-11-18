"""가장 단순한 3D 지도: 공간을 정육면체(복셀)로 나눠 칸마다 '막힘/비어 있음'만 적는다.

known_map.json의 상자를 복셀로 바꾸고, 드론 크기만큼 부풀린 지도로 경로를 계획한다.
나중에 카메라로 만든 지도도 같은 형식으로 채우면 비행 코드는 그대로 쓸 수 있다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass
class VoxelMap:
    origin: np.ndarray   # (3,) 첫 복셀 (0, 0, 0)의 최소 모서리 월드 좌표
    res: float           # 복셀 한 변 (m)
    occ: np.ndarray      # (nx, ny, nz) bool, True = 막힘

    # ------------------------------------------------------------ 만들기

    @classmethod
    def empty(cls, lo, hi, res: float) -> "VoxelMap":
        lo = np.asarray(lo, dtype=float)
        shape = np.ceil((np.asarray(hi, dtype=float) - lo) / res - 1e-9).astype(int)
        return cls(lo, float(res), np.zeros(tuple(shape), dtype=bool))

    @classmethod
    def from_known_map(cls, data: dict, res: float = 0.1) -> "VoxelMap":
        """known_map.json(벽·문 위 벽·가구 상자)을 복셀 지도로 바꾼다. 바닥면이 z = 0이다."""
        b = data["bounds"]
        vmap = cls.empty((b["xmin"], b["ymin"], 0.0), (b["xmax"], b["ymax"], b["zmax"]), res)
        for box in data.get("walls", []) + data.get("lintels", []) + data.get("obstacles", []):
            vmap.add_box(box)
        return vmap

    def add_box(self, box: dict) -> None:
        """중심(x, y, z)·크기(sx, sy, sz) 상자와 겹치는 복셀을 막는다."""
        lo = np.array([box["x"] - box["sx"] / 2, box["y"] - box["sy"] / 2, box["z"] - box["sz"] / 2])
        hi = np.array([box["x"] + box["sx"] / 2, box["y"] + box["sy"] / 2, box["z"] + box["sz"] / 2])
        i0 = np.floor((lo - self.origin) / self.res + 1e-9).astype(int)
        i1 = np.ceil((hi - self.origin) / self.res - 1e-9).astype(int)
        i0 = np.clip(i0, 0, self.shape)
        i1 = np.clip(i1, 0, self.shape)
        if np.all(i1 > i0):
            self.occ[i0[0]:i1[0], i0[1]:i1[1], i0[2]:i1[2]] = True

    def inflate(self, radius: float) -> "VoxelMap":
        """막힌 칸을 radius만큼 둥글게 부풀린 새 지도. 드론 크기와 여유를 반영할 때 쓴다."""
        r = int(math.ceil(radius / self.res - 1e-9))
        out = self.occ.copy()
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                for dz in range(-r, r + 1):
                    if (dx, dy, dz) != (0, 0, 0) and dx * dx + dy * dy + dz * dz <= r * r:
                        _shift_or(out, self.occ, dx, dy, dz)
        return VoxelMap(self.origin.copy(), self.res, out)

    # ------------------------------------------------------------ 조회

    @property
    def shape(self) -> np.ndarray:
        return np.array(self.occ.shape)

    @property
    def upper(self) -> np.ndarray:
        return self.origin + self.shape * self.res

    def index(self, point) -> tuple[int, int, int]:
        i = np.floor((np.asarray(point, dtype=float) - self.origin) / self.res).astype(int)
        return int(i[0]), int(i[1]), int(i[2])

    def center(self, idx) -> np.ndarray:
        return self.origin + (np.asarray(idx, dtype=float) + 0.5) * self.res

    def inside(self, idx) -> bool:
        return all(0 <= v < n for v, n in zip(idx, self.occ.shape))

    def occupied_at(self, point) -> bool:
        """지도 밖(바닥 아래·벽 높이 위 포함)은 막힌 것으로 본다."""
        idx = self.index(point)
        return not self.inside(idx) or bool(self.occ[idx])

    def segment_free(self, a, b) -> bool:
        """a에서 b까지 직선이 막힌 칸을 지나지 않으면 True."""
        a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
        steps = max(1, int(math.ceil(np.linalg.norm(b - a) / (self.res * 0.5))))
        for t in np.linspace(0.0, 1.0, steps + 1):
            if self.occupied_at(a + (b - a) * t):
                return False
        return True

    # ------------------------------------------------------------ 저장

    def save(self, path: str | Path) -> None:
        np.savez_compressed(path, occ=self.occ, origin=self.origin, res=self.res)

    @classmethod
    def load(cls, path: str | Path) -> "VoxelMap":
        data = np.load(path)
        return cls(data["origin"], float(data["res"]), data["occ"].astype(bool))

    def surface(self) -> np.ndarray:
        """비어 있는 칸과 맞닿은 막힌 칸(겉면)의 인덱스. 시각화용."""
        free = ~self.occ
        touch = np.zeros_like(self.occ)
        for axis in range(3):
            for d in (-1, 1):
                shifted = np.zeros_like(free)
                _shift_or(shifted, free, *[d if k == axis else 0 for k in range(3)])
                touch |= shifted
        return np.argwhere(self.occ & touch)

    def to_ply(self, path: str | Path) -> int:
        """겉면 복셀 중심을 높이에 따라 색을 입힌 점 구름(PLY)으로 쓴다. 점 개수를 돌려준다."""
        idx = self.surface()
        pts = self.center(idx) if len(idx) else np.zeros((0, 3))
        zmax = max(float(self.upper[2]), 1e-6)
        lines = [
            "ply", "format ascii 1.0", f"element vertex {len(pts)}",
            "property float x", "property float y", "property float z",
            "property uchar red", "property uchar green", "property uchar blue", "end_header",
        ]
        for x, y, z in pts:
            t = min(1.0, max(0.0, z / zmax))
            r, g, b = int(40 + 200 * t), int(120 + 60 * (1 - abs(2 * t - 1))), int(230 - 190 * t)
            lines.append(f"{x:.3f} {y:.3f} {z:.3f} {r} {g} {b}")
        Path(path).write_text("\n".join(lines) + "\n", encoding="ascii")
        return len(pts)


def _shift_or(dst: np.ndarray, src: np.ndarray, dx: int, dy: int, dz: int) -> None:
    """dst[i+dx, j+dy, k+dz] |= src[i, j, k] (배열 밖으로 나가는 부분은 버린다)."""
    def pair(d: int, n: int):
        return (slice(d, n), slice(0, n - d)) if d >= 0 else (slice(0, n + d), slice(-d, n))

    (dxs, sxs), (dys, sys_), (dzs, szs) = (pair(d, n) for d, n in zip((dx, dy, dz), src.shape))
    dst[dxs, dys, dzs] |= src[sxs, sys_, szs]
