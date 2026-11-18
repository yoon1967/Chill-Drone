#!/usr/bin/env python3
"""4단계: 드론이 카메라로 만든 지도(camera_map.npz)를 정답과 비교하고, 장애물과 사람을 표시한 그림을 만든다.

정답 막힘은 벽·가구·지도에 없던 물건·사람을 높이 0.3–1.8 m로 위에서 본 것이다. 지도는 추정 위치로 만들었으므로
위치가 틀어진 만큼 정답과 어긋난다 (그것까지 포함한 점수).
  막힘 정확도: 지도가 막힘이라고 한 칸 중 0.2 m 안에 정답 막힘이 있는 비율 (헛것을 얼마나 그렸나)
  막힘 찾은 비율: 드론이 본 곳 가까이에 있는 정답 표면 중 0.2 m 안에 지도 막힘이 있는 비율 (본 곳에서 얼마나 놓쳤나)
  비어 있다고 잘못 본 곳: 비행 높이에서 지도가 '빈 곳'이라고 한 칸 중 정답이 막힘인 비율 (가장 위험한 오류)
  담은 넓이: 비행 높이에서 지도가 '빈 곳'으로 아는 넓이 / 정답의 빈 넓이

사용: python3 tools/eval_map.py worlds/s7_stage4          (map.png를 같은 폴더에 쓴다)
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np
from scipy import ndimage

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from autonomy import VoxelMap  # noqa: E402
from worldgen.images import write_png  # noqa: E402

BAND = (0.3, 1.8)
TOL = 2   # 0.2 m (칸)


def ground_truth_occ(truth: dict, shape, origin, res) -> np.ndarray:
    humans = [{"x": h["x"], "y": h["y"], "z": 0.875, "sx": 2 * h["radius"], "sy": 2 * h["radius"], "sz": 1.75}
              for h in truth.get("humans", [])]
    p = truth["params"]
    vm = VoxelMap.from_known_map({"bounds": {"xmin": origin[0], "ymin": origin[1], "xmax": p["width"],
                                             "ymax": p["depth"], "zmax": p["wall_height"]},
                                  "walls": truth["walls"], "lintels": truth["lintels"],
                                  "obstacles": truth["obstacles"] + humans}, res)
    occ = np.zeros(shape, dtype=bool)
    n = tuple(min(a, b) for a, b in zip(shape, vm.occ.shape))
    occ[:n[0], :n[1], :n[2]] = vm.occ[:n[0], :n[1], :n[2]]
    return occ


def map_scores(logodds, seen, origin, res: float, truth: dict, height: float = 1.0) -> dict:
    """막힘 정확도·찾은 비율·비어 있다고 잘못 본 곳·담은 넓이 (tools/replay_map.py도 쓴다). 그림용 칸 배열도 돌려준다."""
    k0, k1 = int(BAND[0] / res), int(BAND[1] / res)
    kf = int((height - origin[2]) / res)
    occ3 = logodds > 0.5
    seen_v = seen.copy()
    seen_v[:, :, 1:] |= seen[:, :, :-1]
    seen_v[:, :, :-1] |= seen[:, :, 1:]
    mapped = occ3[:, :, k0:k1].any(axis=2)
    gt3 = ground_truth_occ(truth, logodds.shape, origin, res)
    gt = gt3[:, :, k0:k1].any(axis=2)
    free_fly = seen_v[:, :, kf] & ~occ3[:, :, kf]

    ring = np.ones((2 * TOL + 1, 2 * TOL + 1), dtype=bool)
    gt_near = ndimage.binary_dilation(gt, structure=ring)
    surface = gt & ndimage.binary_dilation(~gt, structure=np.ones((3, 3), dtype=bool))
    visible = surface & ndimage.binary_dilation(free_fly, structure=ring)
    mapped_near = ndimage.binary_dilation(mapped, structure=ring)
    return {"precision": float((mapped & gt_near).sum() / max(mapped.sum(), 1)),
            "recall": float((visible & mapped_near).sum() / max(visible.sum(), 1)),
            "wrong_free": float((free_fly & gt3[:, :, kf]).sum() / max(free_fly.sum(), 1)),
            "explored": float(free_fly.sum()) * res * res,
            "gt_free_area": float((~gt3[:, :, kf]).sum()) * res * res,
            "mapped": mapped, "gt": gt, "free_fly": free_fly, "n_visible": int(visible.sum())}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("world_dir", type=Path)
    ap.add_argument("--height", type=float, default=1.0)
    ap.add_argument("--png", type=Path, default=None)
    args = ap.parse_args(argv)
    wd = args.world_dir
    m = np.load(wd / "camera_map.npz")
    logodds, seen, origin, res = m["logodds"], m["seen"], m["origin"], float(m["res"])
    truth = json.loads((wd / "ground_truth.json").read_text(encoding="utf-8"))

    sc = map_scores(logodds, seen, origin, res, truth, args.height)
    precision, recall, wrong_free = sc["precision"], sc["recall"], sc["wrong_free"]
    explored, gt_free_area = sc["explored"], sc["gt_free_area"]
    mapped, gt, free_fly = sc["mapped"], sc["gt"], sc["free_fly"]
    visible_n = sc["n_visible"]
    print(f"막힘 정확도        {precision:.0%} (지도의 막힘 {int(mapped.sum())}칸 중 0.2 m 안에 정답 막힘)")
    print(f"막힘 찾은 비율      {recall:.0%} (본 곳 가까운 정답 표면 {visible_n}칸 중)")
    print(f"비어 있다고 잘못 본 곳 {wrong_free:.1%} (비행 높이)")
    print(f"담은 넓이          {explored:.0f} m² / 정답 빈 넓이 {gt_free_area:.0f} m² ({explored / max(gt_free_area, 1e-6):.0%})")

    # 사람: 찾은 사람(people.json)과 정답
    found = []
    if (wd / "people.json").exists():
        found = json.loads((wd / "people.json").read_text(encoding="utf-8"))["people"]
    humans = truth.get("humans", [])
    matched = [h for h in humans if any(math.hypot(h["x"] - q["x"], h["y"] - q["y"]) < 1.0 for q in found)]
    wrong = [q for q in found if all(math.hypot(h["x"] - q["x"], h["y"] - q["y"]) >= 1.0 for h in humans)]
    errs = [min(math.hypot(h["x"] - q["x"], h["y"] - q["y"]) for q in found) for h in matched]
    print(f"사람 표시          {len(humans)}명 중 {len(matched)}명 (엉뚱한 자리 {len(wrong)}개"
          + (f", 위치 오차 {min(errs):.2f}–{max(errs):.2f} m)" if errs else ")"))

    # 그림: 4 px/칸 (40 px/m). 모름 회색, 빈 곳 흰색, 지도 막힘 검정, 정답 막힘 테두리 빨강
    s = 4
    nx, ny = mapped.shape
    img = np.full((ny * s, nx * s, 3), 200, dtype=np.uint8)
    cell = np.zeros((nx, ny, 3), dtype=np.uint8)
    cell[:] = (200, 200, 200)
    cell[free_fly] = (250, 250, 250)
    cell[mapped] = (30, 30, 30)
    edge = gt & ~ndimage.binary_erosion(gt)
    cell[edge & ~mapped] = (230, 80, 80)
    img = np.repeat(np.repeat(cell.transpose(1, 0, 2)[::-1], s, axis=0), s, axis=1)

    def px(x, y):
        return int((x - origin[0]) / res * s), int(ny * s - (y - origin[1]) / res * s)

    def dot(x, y, r, color):
        cx, cy = px(x, y)
        img[max(cy - r, 0):cy + r + 1, max(cx - r, 0):cx + r + 1] = color

    for name, color, step in (("truth_log.csv", (40, 90, 220), 4), ("flight_log.csv", (120, 190, 240), 4)):
        if (wd / name).exists():
            for i, r in enumerate(csv.DictReader(open(wd / name, encoding="utf-8"))):
                if i % step == 0 and float(r["z"]) > 0.3:
                    dot(float(r["x"]), float(r["y"]), 1, color)
    for h in humans:
        dot(h["x"], h["y"], 8, (240, 120, 190))
    for q in found:
        dot(q["x"], q["y"], 5, (120, 30, 160))
    sp = truth["spawn"]
    dot(sp["x"], sp["y"], 6, (40, 170, 70))
    out = args.png or wd / "map.png"
    write_png(out, img.shape[1], img.shape[0], [bytearray(row.tobytes()) for row in img])
    (wd / "map_eval.json").write_text(json.dumps({
        "precision": round(precision, 3), "recall": round(recall, 3), "wrong_free": round(wrong_free, 4),
        "explored_m2": round(explored, 1), "gt_free_m2": round(gt_free_area, 1),
        "people_total": len(humans), "people_found": len(matched), "people_wrong": len(wrong)},
        ensure_ascii=False, indent=1), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
