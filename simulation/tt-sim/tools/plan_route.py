#!/usr/bin/env python3
"""known_map.json으로 복셀 지도를 만들고 출발점→목적지 경로를 계획해 파일로 남긴다 (Gazebo 없이 확인용).

world_dir 안에 쓰는 파일
  voxel_map.npz   복셀 지도 (막힘 = True, 기본 해상도 0.1 m)
  voxels.ply      지도 겉면 점 구름 (MeshLab·CloudCompare 등에서 3D로 볼 수 있다)
  plan.png        위에서 본 지도와 경로
  map3d.png       남서쪽 위에서 내려다본 3D 지도와 경로

예) python3 tools/plan_route.py worlds/s7_stage1
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from autonomy import PlanError, VoxelMap, path_length, plan  # noqa: E402
from worldgen.images import TopView, write_png  # noqa: E402

DRONE_RADIUS = 0.25


def draw(vmap: VoxelMap, inflated: VoxelMap, path, height: float, out: Path) -> None:
    """비행 높이에서 막힌 칸은 진회색, 그 높이에선 비지만 아래·위에 뭔가 있는 칸은 연회색."""
    lo, hi = vmap.origin, vmap.upper
    view = TopView(hi[0] - lo[0], hi[1] - lo[1])
    k = vmap.index((0, 0, height))[2]
    any_z = vmap.occ.any(axis=2)
    r = vmap.res
    for i, j in np.argwhere(inflated.occ[:, :, k] & ~vmap.occ[:, :, k]):
        view.rect(i * r, j * r, (i + 1) * r, (j + 1) * r, (232, 222, 206))
    for i, j in np.argwhere(any_z & ~vmap.occ[:, :, k]):
        view.rect(i * r, j * r, (i + 1) * r, (j + 1) * r, (190, 190, 190))
    for i, j in np.argwhere(vmap.occ[:, :, k]):
        view.rect(i * r, j * r, (i + 1) * r, (j + 1) * r, (60, 60, 60))
    for a, b in zip(path, path[1:]):
        for t in np.linspace(0, 1, max(2, int(np.linalg.norm(b - a) / 0.03))):
            p = a + (b - a) * t
            view.rect(p[0] - 0.03, p[1] - 0.03, p[0] + 0.03, p[1] + 0.03, (240, 120, 0))
    for p in path:
        view.rect(p[0] - 0.07, p[1] - 0.07, p[0] + 0.07, p[1] + 0.07, (200, 60, 0))
    s, g = path[0], path[-1]
    view.rect(s[0] - 0.15, s[1] - 0.15, s[0] + 0.15, s[1] + 0.15, (40, 170, 70))
    view.rect(g[0] - 0.15, g[1] - 0.15, g[0] + 0.15, g[1] + 0.15, (220, 40, 40))
    view.save(out)


def render_3d(vmap: VoxelMap, path, out: Path, azimuth_deg: float = 45, elevation_deg: float = 55,
              px_per_m: float = 45) -> None:
    """복셀 지도 겉면을 남서쪽 위에서 내려다본 3D 그림. 높이에 따라 색이 바뀌고 경로는 주황색."""
    a, e = np.radians(azimuth_deg), np.radians(elevation_deg)
    pts = vmap.center(vmap.surface())

    def project(p):
        p = np.atleast_2d(p)
        fwd = p[:, 0] * np.cos(a) + p[:, 1] * np.sin(a)
        sx = p[:, 0] * np.sin(a) - p[:, 1] * np.cos(a)
        sy = -fwd * np.sin(e) + p[:, 2] * np.cos(e)
        depth = fwd * np.cos(e) - p[:, 2] * np.sin(e)
        return sx, sy, depth

    sx, sy, depth = project(pts)
    margin = 20
    x0, y1 = sx.min(), sy.max()
    width = int((sx.max() - x0) * px_per_m) + 2 * margin
    height = int((y1 - sy.min()) * px_per_m) + 2 * margin
    canvas = np.full((height, width, 3), 245, dtype=np.uint8)
    zmax = max(float(vmap.upper[2]), 1e-6)
    size = max(2, int(vmap.res * px_per_m * 0.9))
    shade = 1.0 - 0.3 * (depth - depth.min()) / max(np.ptp(depth), 1e-6)
    for k in np.argsort(-depth):                       # 먼 점부터 그린다
        t = min(1.0, pts[k, 2] / zmax)
        color = np.array([60 + 170 * t, 110 + 90 * (1 - abs(2 * t - 1)), 220 - 170 * t]) * (0.75 + 0.25 * shade[k])
        cx = int((sx[k] - x0) * px_per_m) + margin
        cy = int((y1 - sy[k]) * px_per_m) + margin
        canvas[max(0, cy - size // 2):cy + size // 2 + 1, max(0, cx - size // 2):cx + size // 2 + 1] = color
    for p, q in zip(path, path[1:]):                   # 경로는 맨 위에 그린다
        for t in np.linspace(0, 1, max(2, int(np.linalg.norm(q - p) / 0.02))):
            px, py, _ = project(p + (q - p) * t)
            cx, cy = int((px[0] - x0) * px_per_m) + margin, int((y1 - py[0]) * px_per_m) + margin
            canvas[max(0, cy - 2):cy + 3, max(0, cx - 2):cx + 3] = (240, 110, 0)
    write_png(out, width, height, [bytearray(row.tobytes()) for row in canvas])


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="복셀 지도 만들기 + 3D 경로 계획")
    ap.add_argument("world_dir", type=Path)
    ap.add_argument("--res", type=float, default=0.1, help="복셀 한 변 (m)")
    ap.add_argument("--height", type=float, default=None, help="비행 높이 (기본: known_map의 flight_height)")
    args = ap.parse_args(argv)

    data = json.loads((args.world_dir / "known_map.json").read_text(encoding="utf-8"))
    height = args.height or data["flight_height"]
    vmap = VoxelMap.from_known_map(data, args.res)
    inflated = vmap.inflate(DRONE_RADIUS)
    vmap.save(args.world_dir / "voxel_map.npz")
    points = vmap.to_ply(args.world_dir / "voxels.ply")
    print(f"복셀 지도 {vmap.occ.shape} (해상도 {args.res} m), 막힌 칸 {int(vmap.occ.sum())}개, 겉면 점 {points}개")

    if not data.get("goal"):
        render_3d(vmap, [], args.world_dir / "map3d.png")
        print("이 단계 지도에는 목적지가 없어 경로는 계획하지 않습니다.")
        return 0
    start = np.array([data["spawn"]["x"], data["spawn"]["y"], height])
    goal = np.array([data["goal"]["x"], data["goal"]["y"], data["goal"]["z"]])
    try:
        path = plan(inflated, start, goal)
    except PlanError as err:
        print(f"경로 계획 실패: {err}")
        return 1
    draw(vmap, inflated, path, height, args.world_dir / "plan.png")
    render_3d(vmap, path, args.world_dir / "map3d.png")
    print(f"경로 {path_length(path):.2f} m, 경유점 {len(path) - 1}개")
    for p in path:
        print(f"  ({p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
