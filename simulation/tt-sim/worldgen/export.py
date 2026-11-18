"""World를 파일로 쓴다: Gazebo 월드, 드론에게 줄 지도, 평가용 정답, 미리보기."""
from __future__ import annotations

import json
import random
from dataclasses import asdict
from pathlib import Path

from . import images
from .layout import ART_COUNT, STAGES, Box, World
from .sdf import build_world_sdf

FLOOR_PX_PER_M = 64


def _box(b: Box) -> dict:
    return {"name": b.name, "x": b.x, "y": b.y, "z": b.z, "sx": b.sx, "sy": b.sy, "sz": b.sz}


def _obstacle(o) -> dict:
    return {**_box(o.box), "kind": o.kind, "yaw": o.yaw}


def known_map(world: World) -> dict:
    """드론(알고리즘)이 받아도 되는 정보만 담는다."""
    p = world.params
    cfg = STAGES[world.stage]
    return {
        "format": "tt-sim/known-map@2",
        "stage": world.stage,
        "description": cfg["desc"],
        "frame": "world: x 동쪽, y 북쪽, z 위, 단위 m. 상자는 중심(x, y, z)과 크기(sx, sy, sz), 회전 없음",
        "bounds": {"xmin": 0.0, "ymin": 0.0, "xmax": p.width, "ymax": p.depth, "zmax": p.wall_height},
        "flight_height": p.flight_height,
        "spawn": world.spawn,
        "goal": world.goal,
        "walls": [_box(b) for b in world.walls] if cfg["known_map"] else [],
        "lintels": [_box(b) for b in world.lintels] if cfg["known_map"] else [],
        "obstacles": [_obstacle(o) for o in world.obstacles if world.is_known(o)],
    }


def ground_truth(world: World) -> dict:
    """평가 전용. 드론 코드가 읽으면 안 된다."""
    return {
        "format": "tt-sim/ground-truth@2",
        "seed": world.seed,
        "stage": world.stage,
        "params": asdict(world.params),
        "rooms": [asdict(r) for r in world.rooms],
        "doors": [asdict(d) for d in world.doors],
        "walls": [_box(b) for b in world.walls],
        "lintels": [_box(b) for b in world.lintels],
        "obstacles": [
            {**_obstacle(o), "hidden": o.hidden, "blocker": o.blocker, "in_known_map": world.is_known(o)}
            for o in world.obstacles
        ],
        "humans": [asdict(h) for h in world.humans],
        "spawn": world.spawn,
        "goal": world.goal,
    }


FLOOR_PREVIEW = {"wood": (226, 208, 186), "tile": (228, 228, 224), "carpet": (206, 214, 228), "concrete": (214, 214, 210)}


def _preview(world: World, path: Path) -> None:
    p = world.params
    view = images.TopView(p.width, p.depth)
    for r in world.rooms:
        view.rect(r.x0, r.y0, r.x1, r.y1, FLOOR_PREVIEW[r.floor])
    for o in world.obstacles:
        color = (140, 140, 140) if world.is_known(o) else (220, 60, 60)
        view.rect(*o.box.bounds2d(), color)
    for b in world.walls:
        view.rect(*b.bounds2d(), (30, 30, 30))
    for h in world.humans:
        view.rect(h.x - h.radius, h.y - h.radius, h.x + h.radius, h.y + h.radius, (40, 90, 220))
    s = world.spawn
    view.rect(s["x"] - 0.15, s["y"] - 0.15, s["x"] + 0.15, s["y"] + 0.15, (40, 170, 70))
    if world.goal:
        g = world.goal
        view.rect(g["x"] - 0.15, g["y"] - 0.15, g["x"] + 0.15, g["y"] + 0.15, (240, 160, 0))
    view.save(path)


def _write_textures(world: World, tex_dir: Path) -> dict[str, Path]:
    rng = random.Random(f"{world.seed}:texture-pixels")
    out: dict[str, Path] = {}

    def save(key: str, rows: images.Rows) -> None:
        path = tex_dir / f"{key}.png"
        images.write_png(path, len(rows[0]) // 3, len(rows), rows)
        out[key] = path

    save("paint", images.paint(rng, world.paint))
    for i, room in enumerate(world.rooms):
        w, h = max(8, round(room.w * FLOOR_PX_PER_M)), max(8, round(room.h * FLOOR_PX_PER_M))
        if room.floor == "wood":
            rows = images.wood_floor(rng, w, h, FLOOR_PX_PER_M)
        elif room.floor == "tile":
            rows = images.tile_floor(rng, w, h, FLOOR_PX_PER_M)
        elif room.floor == "carpet":
            rows = images.carpet_floor(rng, w, h)
        else:
            rows = images.concrete_floor(rng, w, h)
        save(f"floor_{i}", rows)
    for k in range(ART_COUNT):
        save(f"art_{k}", images.artwork(rng))
    save("whiteboard", images.whiteboard(rng))
    save("cardboard", images.cardboard(rng))
    return out


def write_world(world: World, out_dir: Path, human_model: str = "actor") -> dict[str, Path]:
    out_dir = Path(out_dir).resolve()
    tex_dir = out_dir / "textures"
    tex_dir.mkdir(parents=True, exist_ok=True)
    textures = _write_textures(world, tex_dir)

    paths = {
        "world": out_dir / "world.sdf",
        "known_map": out_dir / "known_map.json",
        "ground_truth": out_dir / "ground_truth.json",
        "preview": out_dir / "preview.png",
    }
    paths["world"].write_text(build_world_sdf(world, textures, human_model), encoding="utf-8")
    paths["known_map"].write_text(json.dumps(known_map(world), ensure_ascii=False, indent=2), encoding="utf-8")
    paths["ground_truth"].write_text(json.dumps(ground_truth(world), ensure_ascii=False, indent=2), encoding="utf-8")
    _preview(world, paths["preview"])
    return paths
