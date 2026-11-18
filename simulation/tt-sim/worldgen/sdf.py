"""World 객체를 Gazebo Harmonic(gz-sim 8)용 SDF 월드로 바꾼다."""
from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

from .furnish import Part
from .layout import Box, Human, World

# gz-sim 예제(actor.sdf)가 쓰는 Fuel 사람 메시. 처음 실행할 때 인터넷에서 내려받는다.
ACTOR_MESH = "https://fuel.gazebosim.org/1.0/Mingfei/models/actor/tip/files/meshes/talk_b.dae"


def _sub(parent: ET.Element, tag: str, text: str | None = None, **attrs) -> ET.Element:
    element = ET.SubElement(parent, tag, attrs)
    if text is not None:
        element.text = text
    return element


def _rgb(c) -> str:
    return f"{c[0]:.3f} {c[1]:.3f} {c[2]:.3f} 1"


def _material(parent: ET.Element, color, texture: Path | None = None, emissive=None) -> None:
    material = _sub(parent, "material")
    if texture is not None:
        _sub(material, "diffuse", "1 1 1 1")
        _sub(material, "specular", "0.05 0.05 0.05 1")
        metal = _sub(_sub(material, "pbr"), "metal")
        _sub(metal, "albedo_map", texture.as_posix())
        _sub(metal, "roughness", "0.9")
        _sub(metal, "metalness", "0.0")
    else:
        _sub(material, "ambient", _rgb(color))
        _sub(material, "diffuse", _rgb(color))
        _sub(material, "specular", "0.08 0.08 0.08 1")
    if emissive is not None:
        _sub(material, "emissive", _rgb(emissive))


def _geometry(parent: ET.Element, shape: str, size) -> None:
    geometry = _sub(parent, "geometry")
    if shape == "box":
        _sub(_sub(geometry, "box"), "size", f"{size[0]:.4f} {size[1]:.4f} {size[2]:.4f}")
    elif shape == "cylinder":
        cylinder = _sub(geometry, "cylinder")
        _sub(cylinder, "radius", f"{size[0]:.4f}")
        _sub(cylinder, "length", f"{size[1]:.4f}")
    else:
        _sub(_sub(geometry, "sphere"), "radius", f"{size[0]:.4f}")


def _part_visual(link: ET.Element, name: str, part: Part, textures: dict[str, Path]) -> None:
    visual = _sub(link, "visual", name=name)
    _sub(visual, "pose", f"{part.x:.4f} {part.y:.4f} {part.z:.4f} 0 0 {part.yaw:.4f}")
    _geometry(visual, part.shape, part.size)
    _material(visual, part.color, textures.get(part.texture) if part.texture else None, part.emissive)
    if part.transparency:
        _sub(visual, "transparency", f"{part.transparency:.2f}")


def _box_collision(link: ET.Element, b: Box) -> None:
    collision = _sub(link, "collision", name=f"{b.name}_collision")
    _sub(collision, "pose", f"{b.x:.4f} {b.y:.4f} {b.z:.4f} 0 0 0")
    _geometry(collision, "box", (b.sx, b.sy, b.sz))


def _textured_boxes(world_el: ET.Element, name: str, boxes: list[Box], textures: dict[str, Path]) -> ET.Element:
    """벽·바닥처럼 충돌체와 모양이 같은 상자 묶음."""
    model = _sub(world_el, "model", name=name)
    _sub(model, "static", "true")
    link = _sub(model, "link", name="link")
    for b in boxes:
        _box_collision(link, b)
        visual = _sub(link, "visual", name=f"{b.name}_visual")
        _sub(visual, "pose", f"{b.x:.4f} {b.y:.4f} {b.z:.4f} 0 0 0")
        _geometry(visual, "box", (b.sx, b.sy, b.sz))
        _material(visual, (0.9, 0.9, 0.9), textures[b.texture])
    return link


def _actor(world_el: ET.Element, h: Human) -> None:
    """제자리에 서 있는 사람. gz-sim의 actor에는 충돌체가 없어 드론이 통과한다(감지 대상으로만 쓴다).

    위치는 trajectory 웨이포인트에만 적는다. actor 자체에 <pose>를 함께 주면
    웨이포인트가 그 위치 기준으로 한 번 더 더해져 사람이 엉뚱한 곳(좌표 2배)에 나타난다.
    """
    pose = f"{h.x:.3f} {h.y:.3f} 1.0 0 0 {h.yaw:.3f}"
    actor = _sub(world_el, "actor", name=h.name)
    skin = _sub(actor, "skin")
    _sub(skin, "filename", ACTOR_MESH)
    _sub(skin, "scale", "1.0")
    animation = _sub(actor, "animation", name="talk_b")
    _sub(animation, "filename", ACTOR_MESH)
    _sub(animation, "scale", "1.0")
    _sub(animation, "interpolate_x", "true")
    script = _sub(actor, "script")
    _sub(script, "loop", "true")
    _sub(script, "auto_start", "true")
    trajectory = _sub(script, "trajectory", id="0", type="talk_b")
    for t in ("0", "30"):
        waypoint = _sub(trajectory, "waypoint")
        _sub(waypoint, "time", t)
        _sub(waypoint, "pose", pose)


def _simple_human(world_el: ET.Element, h: Human) -> None:
    """인터넷 없이 쓰는 단순 사람 모양(원기둥 몸 + 구 머리). YOLO로는 사람으로 잡히지 않는다."""
    model = _sub(world_el, "model", name=h.name)
    _sub(model, "static", "true")
    _sub(model, "pose", f"{h.x:.3f} {h.y:.3f} 0 0 0 {h.yaw:.3f}")
    link = _sub(model, "link", name="link")
    parts = [
        ("body", "0 0 0.725 0 0 0", "cylinder", (0.2, 1.45), (0.2, 0.3, 0.6)),
        ("head", "0 0 1.6 0 0 0", "sphere", (0.12,), (0.9, 0.75, 0.6)),
    ]
    for name, pose, shape, size, color in parts:
        for kind in ("collision", "visual"):
            element = _sub(link, kind, name=f"{name}_{kind}")
            _sub(element, "pose", pose)
            _geometry(element, shape, size)
            if kind == "visual":
                _material(element, color)


def build_world_sdf(world: World, textures: dict[str, Path], human_model: str = "actor") -> str:
    if human_model not in ("actor", "simple"):
        raise ValueError("human_model은 'actor' 또는 'simple'이어야 합니다.")
    p = world.params

    sdf = ET.Element("sdf", version="1.9")
    w = _sub(sdf, "world", name="tt_world")
    physics = _sub(w, "physics", name="4ms", type="ignored")
    _sub(physics, "max_step_size", "0.004")
    _sub(physics, "real_time_factor", "1.0")
    w.append(ET.Comment(
        " 중력 0: tt_drone은 속도 명령으로 움직이는 운동학 모델이라 중력이 있으면 아래로 처진다."
        " 나머지 물체는 모두 static이라 영향이 없다. "
    ))
    _sub(w, "gravity", "0 0 0")

    for filename, name in (
        ("gz-sim-physics-system", "gz::sim::systems::Physics"),
        ("gz-sim-user-commands-system", "gz::sim::systems::UserCommands"),
        ("gz-sim-scene-broadcaster-system", "gz::sim::systems::SceneBroadcaster"),
    ):
        _sub(w, "plugin", filename=filename, name=name)
    sensors = _sub(w, "plugin", filename="gz-sim-sensors-system", name="gz::sim::systems::Sensors")
    _sub(sensors, "render_engine", "ogre2")
    _sub(w, "plugin", filename="gz-sim-imu-system", name="gz::sim::systems::Imu")

    scene = _sub(w, "scene")
    _sub(scene, "ambient", "0.55 0.55 0.55 1")
    _sub(scene, "background", "0.8 0.85 0.9 1")
    _sub(scene, "shadows", "false")
    sun = _sub(w, "light", type="directional", name="sun")
    _sub(sun, "cast_shadows", "false")
    _sub(sun, "pose", "0 0 10 0 0 0")
    _sub(sun, "diffuse", "0.55 0.55 0.55 1")
    _sub(sun, "specular", "0.1 0.1 0.1 1")
    _sub(sun, "direction", "-0.3 0.2 -1")
    for i, room in enumerate(world.rooms):   # 방마다 천장등
        lamp = _sub(w, "light", type="point", name=f"ceiling_light_{i:02d}")
        _sub(lamp, "cast_shadows", "false")
        _sub(lamp, "pose", f"{(room.x0 + room.x1) / 2:.3f} {(room.y0 + room.y1) / 2:.3f} {p.wall_height - 0.2:.2f} 0 0 0")
        _sub(lamp, "diffuse", "0.55 0.52 0.48 1")
        _sub(lamp, "specular", "0.1 0.1 0.1 1")
        attenuation = _sub(lamp, "attenuation")
        _sub(attenuation, "range", "9")
        _sub(attenuation, "constant", "0.5")
        _sub(attenuation, "linear", "0.08")
        _sub(attenuation, "quadratic", "0.01")

    _textured_boxes(w, "floor", world.floor, textures)
    _textured_boxes(w, "walls", world.walls + world.lintels, textures)

    for obstacle in world.obstacles:
        model = _sub(w, "model", name=obstacle.box.name)
        _sub(model, "static", "true")
        link = _sub(model, "link", name="link")
        _box_collision(link, obstacle.box)     # 충돌체는 바닥 면적 상자 하나
        for k, part in enumerate(obstacle.parts()):
            _part_visual(link, f"v{k:03d}", part, textures)
        if obstacle.kind == "person":          # 길에 서 있는 사람: 보이는 모양은 사람 모델
            body = Human(f"{obstacle.box.name}_body", obstacle.box.x, obstacle.box.y, obstacle.yaw)
            (_actor if human_model == "actor" else _simple_human)(w, body)

    decor = _sub(w, "model", name="decor")
    _sub(decor, "static", "true")
    decor_link = _sub(decor, "link", name="link")
    for k, part in enumerate(world.decor):
        _part_visual(decor_link, f"decor_{k:03d}", part, textures)

    for human in world.humans:
        (_actor if human_model == "actor" else _simple_human)(w, human)

    s = world.spawn
    include = _sub(w, "include")
    _sub(include, "uri", "model://tt_drone")
    _sub(include, "name", "tt_drone")
    _sub(include, "pose", f"{s['x']:.3f} {s['y']:.3f} {s['z']:.3f} 0 0 {s['yaw']:.3f}")

    ET.indent(sdf, space="  ")
    return '<?xml version="1.0"?>\n' + ET.tostring(sdf, encoding="unicode") + "\n"
