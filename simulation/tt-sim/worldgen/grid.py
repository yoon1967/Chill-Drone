"""바닥 평면 점유 격자. 맵 생성 중 연결성 검사에 쓴다.

셀 (i, j)는 x ∈ [i·res, (i+1)·res), y ∈ [j·res, (j+1)·res) 영역이다.
값 0은 비어 있음, 1은 막힘.
"""
from __future__ import annotations

import math
from collections import deque


class OccupancyGrid:
    def __init__(self, width: float, depth: float, res: float):
        self.width = width
        self.depth = depth
        self.res = res
        self.nx = math.ceil(width / res - 1e-9)
        self.ny = math.ceil(depth / res - 1e-9)
        self.cells = [bytearray(self.nx) for _ in range(self.ny)]

    def copy(self) -> "OccupancyGrid":
        other = OccupancyGrid.__new__(OccupancyGrid)
        other.width, other.depth, other.res = self.width, self.depth, self.res
        other.nx, other.ny = self.nx, self.ny
        other.cells = [bytearray(row) for row in self.cells]
        return other

    def mark_rect(self, xmin: float, ymin: float, xmax: float, ymax: float, inflate: float = 0.0) -> None:
        """사각형(+inflate 여유)과 겹치는 셀을 모두 막는다."""
        i0 = max(0, math.floor((xmin - inflate) / self.res))
        i1 = min(self.nx - 1, math.ceil((xmax + inflate) / self.res) - 1)
        j0 = max(0, math.floor((ymin - inflate) / self.res))
        j1 = min(self.ny - 1, math.ceil((ymax + inflate) / self.res) - 1)
        if i1 < i0 or j1 < j0:
            return
        fill = b"\x01" * (i1 - i0 + 1)
        for j in range(j0, j1 + 1):
            self.cells[j][i0:i1 + 1] = fill

    def cell_of(self, x: float, y: float) -> tuple[int, int]:
        i = min(self.nx - 1, max(0, int(x / self.res)))
        j = min(self.ny - 1, max(0, int(y / self.res)))
        return i, j

    def center_of(self, i: int, j: int) -> tuple[float, float]:
        return (i + 0.5) * self.res, (j + 0.5) * self.res

    def is_free(self, i: int, j: int) -> bool:
        return 0 <= i < self.nx and 0 <= j < self.ny and self.cells[j][i] == 0

    def cells_in(self, x0: float, y0: float, x1: float, y1: float) -> list[tuple[int, int]]:
        """중심이 사각형 안에 있는 빈 셀 목록."""
        i_min = max(0, math.ceil(x0 / self.res - 0.5 + 1e-9))
        i_max = min(self.nx - 1, math.floor(x1 / self.res - 0.5 - 1e-9))
        j_min = max(0, math.ceil(y0 / self.res - 0.5 + 1e-9))
        j_max = min(self.ny - 1, math.floor(y1 / self.res - 0.5 - 1e-9))
        return [
            (i, j)
            for j in range(j_min, j_max + 1)
            for i in range(i_min, i_max + 1)
            if self.cells[j][i] == 0
        ]

    def distances_from(self, i: int, j: int) -> list[list[int]]:
        """4방향 BFS 거리(셀 단위). 도달할 수 없으면 -1."""
        dist = [[-1] * self.nx for _ in range(self.ny)]
        if not self.is_free(i, j):
            return dist
        dist[j][i] = 0
        queue = deque([(i, j)])
        while queue:
            ci, cj = queue.popleft()
            d = dist[cj][ci] + 1
            for ni, nj in ((ci + 1, cj), (ci - 1, cj), (ci, cj + 1), (ci, cj - 1)):
                if 0 <= ni < self.nx and 0 <= nj < self.ny and dist[nj][ni] < 0 and self.cells[nj][ni] == 0:
                    dist[nj][ni] = d
                    queue.append((ni, nj))
        return dist

    def connected(self, points: list[tuple[float, float]]) -> bool:
        """모든 점이 빈 셀 위에 있고 서로 오갈 수 있으면 True."""
        if not points:
            return True
        cells = [self.cell_of(x, y) for x, y in points]
        if not all(self.is_free(i, j) for i, j in cells):
            return False
        dist = self.distances_from(*cells[0])
        return all(dist[j][i] >= 0 for i, j in cells)
