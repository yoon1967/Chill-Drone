#!/usr/bin/env python3
"""비행 도중에 드론 앞에 장애물을 갑자기 생성한다 (시험용 노드).

드론이 계획한 경로(/tt/path)에서 앞으로 ahead_min~ahead_max m 지점을 골라 상자 더미나 사람을
Gazebo에 새로 만든다. 비행 코드는 이걸 모른 채 ToF·YOLO로만 알아채야 한다.
  - 목적지 근처, 문 바로 앞, 벽·가구와 겹치는 자리는 고르지 않는다.
  - 만들어도 드론이 목적지까지 갈 수 있는 자리만 고른다. 드론은 새 장애물을 실제보다 크게 어림하므로,
    비행 중에 만든 장애물은 각 변을 ESTIMATE_SLACK만큼 키워도 길이 남아야 한다 (정답 지도 기준).
  - 만든 장애물과 시각은 world_dir/spawned.json에 적는다 (tools/eval_flight.py가 평가에 쓴다).

이 노드는 시험 장치라 정답 위치(/tt/odometry_gt)와 정답 지도(ground_truth.json)를 써도 된다.
비행 코드(nodes/mission_node.py)는 이 파일을 모른다.

실행 예)
  python3 tools/spawn_obstacles.py --ros-args -p world_dir:=$PWD/worlds/s7_stage1 -p count:=2 -p use_sim_time:=true
"""
from __future__ import annotations

import json
import math
import random
import subprocess
import sys
from pathlib import Path

import numpy as np

try:
    import rclpy
    from nav_msgs.msg import Odometry
    from nav_msgs.msg import Path as PathMsg
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from std_msgs.msg import String
except ImportError:   # ROS가 없는 PC의 단위 테스트에서는 아래 도우미 함수만 쓴다
    rclpy = None
    Node = object

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from autonomy import PlanError, VoxelMap, plan  # noqa: E402

ACTOR_MESH = "https://fuel.gazebosim.org/1.0/Mingfei/models/actor/tip/files/meshes/talk_b.dae"
CARDBOARD = "0.62 0.45 0.27 1"
SIZES = {"box_stack": (0.6, 0.6, 1.4), "person": (0.6, 0.6, 1.75)}
DRONE_RADIUS = 0.25   # 정답 지도로 '갈 수 있는지' 볼 때 드론을 이만큼 부풀린다 (비행 코드와 같게)
ESTIMATE_SLACK = 0.35  # 드론은 새 장애물 크기를 어림해 실제보다 크게 잡는다. 이만큼 키워도 길이 남는 자리에만 놓는다


# ---------------------------------------------------------------- Gazebo에 만들기

def _box_xml(name: str, pose: str, size, color: str, collision: bool) -> str:
    geo = f"<geometry><box><size>{size[0]:.3f} {size[1]:.3f} {size[2]:.3f}</size></box></geometry>"
    out = f"<visual name='{name}_v'><pose>{pose}</pose>{geo}<material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual>"
    if collision:
        out += f"<collision name='{name}_c'><pose>{pose}</pose>{geo}</collision>"
    return out


def box_stack_sdf(name: str, x: float, y: float, yaw: float, rng: random.Random) -> str:
    """상자 2–3개를 쌓은 더미. 충돌체는 전체를 감싸는 상자 하나."""
    sx, sy, sz = SIZES["box_stack"]
    parts, z = [], 0.0
    while z < sz - 0.05:
        h = min(sz - z, rng.uniform(0.4, 0.55))
        w, d = rng.uniform(0.48, sx), rng.uniform(0.48, sy)
        ox, oy = rng.uniform(-(sx - w) / 2, (sx - w) / 2), rng.uniform(-(sy - d) / 2, (sy - d) / 2)
        parts.append(_box_xml(f"b{len(parts)}", f"{ox:.3f} {oy:.3f} {z + h / 2:.3f} 0 0 0", (w, d, h), CARDBOARD, False))
        z += h
    collision = (f"<collision name='footprint'><pose>0 0 {sz / 2:.3f} 0 0 0</pose>"
                 f"<geometry><box><size>{sx} {sy} {sz}</size></box></geometry></collision>")
    return (f"<sdf version='1.9'><model name='{name}'><static>true</static><pose>{x:.3f} {y:.3f} 0 0 0 {yaw:.3f}</pose>"
            f"<link name='link'>{collision}{''.join(parts)}</link></model></sdf>")


def person_sdf(name: str, x: float, y: float, yaw: float) -> str:
    """제자리에서 말하는 사람(actor). actor에는 충돌체가 없어 collision_sdf를 따로 만든다.

    위치는 trajectory에만 적는다 (actor에 <pose>를 함께 주면 좌표가 두 번 더해진다. worldgen/sdf.py 참고).
    """
    pose = f"{x:.3f} {y:.3f} 1.0 0 0 {yaw:.3f}"
    waypoints = "".join(f"<waypoint><time>{t}</time><pose>{pose}</pose></waypoint>" for t in (0, 30))
    return (f"<sdf version='1.9'><actor name='{name}'>"
            f"<skin><filename>{ACTOR_MESH}</filename><scale>1.0</scale></skin>"
            f"<animation name='talk_b'><filename>{ACTOR_MESH}</filename><scale>1.0</scale>"
            f"<interpolate_x>true</interpolate_x></animation>"
            f"<script><loop>true</loop><auto_start>true</auto_start>"
            f"<trajectory id='0' type='talk_b'>{waypoints}</trajectory></script></actor></sdf>")


def collision_sdf(name: str, x: float, y: float, size) -> str:
    """보이지 않는 충돌 상자 (사람 자리)."""
    sx, sy, sz = size
    return (f"<sdf version='1.9'><model name='{name}'><static>true</static><pose>{x:.3f} {y:.3f} {sz / 2:.3f} 0 0 0</pose>"
            f"<link name='link'><collision name='c'><geometry><box><size>{sx} {sy} {sz}</size></box></geometry>"
            f"</collision></link></model></sdf>")


def gz_create(sdf: str, world: str = "tt_world") -> bool:
    req = 'sdf: "' + sdf.replace('"', "'") + '"'
    r = subprocess.run(["gz", "service", "-s", f"/world/{world}/create", "--reqtype", "gz.msgs.EntityFactory",
                        "--reptype", "gz.msgs.Boolean", "--timeout", "5000", "--req", req],
                       capture_output=True, text=True)
    return r.returncode == 0 and "true" in r.stdout


# ---------------------------------------------------------------- 자리 고르기

def along_path(path: list[np.ndarray], start_xy, distances) -> list[tuple[float, float, float]]:
    """경로에서 start_xy와 가장 가까운 곳부터 경로를 따라 distances만큼 간 점들 (x, y, 진행 방향)."""
    best, best_i, best_t = math.inf, 0, 0.0
    for i, (a, b) in enumerate(zip(path, path[1:])):
        ab = b[:2] - a[:2]
        t = float(np.clip(np.dot(np.asarray(start_xy) - a[:2], ab) / max(float(np.dot(ab, ab)), 1e-9), 0, 1))
        d = float(np.linalg.norm(a[:2] + ab * t - np.asarray(start_xy)))
        if d < best:
            best, best_i, best_t = d, i, t
    out = []
    for want in distances:
        i, t, left = best_i, best_t, want
        while i < len(path) - 1:
            a, b = path[i][:2], path[i + 1][:2]
            seg = float(np.linalg.norm(b - a))
            rest = seg * (1 - t)
            if left <= rest and seg > 1e-9:
                p = a + (b - a) * (t + left / seg)
                out.append((float(p[0]), float(p[1]), math.atan2(b[1] - a[1], b[0] - a[0])))
                break
            left -= rest
            i, t = i + 1, 0.0
    return out


def footprint(x: float, y: float, size) -> dict:
    return {"x": x, "y": y, "z": size[2] / 2, "sx": size[0], "sy": size[1], "sz": size[2]}


def gap2d(a: dict, b: dict) -> float:
    gx = max(abs(a["x"] - b["x"]) - (a["sx"] + b["sx"]) / 2, 0.0)
    gy = max(abs(a["y"] - b["y"]) - (a["sy"] + b["sy"]) / 2, 0.0)
    return math.hypot(gx, gy)


def reachable(solids: list[dict], bounds: dict, start, goal) -> bool:
    """정답 지도(solids)에서 드론이 start에서 goal까지 갈 수 있으면 True."""
    inflated = VoxelMap.from_known_map({"bounds": bounds, "obstacles": solids}, 0.1).inflate(DRONE_RADIUS)
    try:
        plan(inflated, (start[0], start[1], max(start[2], 0.5)), goal, max_expansions=400_000)
        return True
    except PlanError:
        return False


def grown(box: dict, margin: float, top: float) -> dict:
    """드론이 보는 모양: 각 변을 margin만큼 키우고, 높이는 모르므로 바닥~top 기둥 (nodes/mission_node.py와 같게)."""
    return {**box, "sx": box["sx"] + 2 * margin, "sy": box["sy"] + 2 * margin, "z": top / 2, "sz": top}


def choose_spot(path, pose, size, solids, doors, bounds, rng, ahead=(1.8, 3.0), spawned=()):
    """경로를 따라 ahead 거리 안에서 장애물 놓을 자리 (x, y, 진행 방향, 직선거리)와 못 고른 이유 집계.

    solids는 원래 있던 벽·가구, spawned는 앞서 비행 중에 만든 장애물이다.
    """
    why: dict[str, int] = {}

    def no(reason: str) -> None:
        why[reason] = why.get(reason, 0) + 1

    goal = path[-1]
    candidates = along_path(path, pose[:2], np.arange(ahead[0], ahead[1] + 1e-6, 0.2))
    if not candidates:
        no("경로가 끝남")
    rng.shuffle(candidates)
    for x, y, heading in candidates:
        box = footprint(x, y, size)
        if math.hypot(x - goal[0], y - goal[1]) < 1.2:
            no("목적지 앞")
        elif any(math.hypot(x - dx, y - dy) < 0.9 for dx, dy in doors):
            no("문 앞")
        elif any(gap2d(box, s) < 0.05 for s in list(solids) + list(spawned)):
            no("벽·가구와 겹침")
        elif gap2d(box, footprint(pose[0], pose[1], (0.18, 0.18, 0.05))) < 1.0:
            no("드론 코앞")
        elif not reachable(list(solids) + [grown(b, ESTIMATE_SLACK, bounds["zmax"]) for b in list(spawned) + [box]],
                           bounds, pose, goal):
            no("길이 막힘")
        else:
            return (x, y, heading, math.hypot(x - pose[0], y - pose[1])), why
    return None, why


class Spawner(Node):
    def __init__(self) -> None:
        super().__init__("tt_obstacle_spawner")
        self.declare_parameter("world_dir", "")
        self.declare_parameter("count", 2)
        self.declare_parameter("kinds", "box_stack,person")
        self.declare_parameter("ahead_min", 1.8)
        self.declare_parameter("ahead_max", 3.0)
        self.declare_parameter("first_delay", 1.0)   # FOLLOW가 시작되고 이만큼 뒤에 첫 장애물
        self.declare_parameter("gap", 4.0)           # 장애물 사이 최소 간격 (초)
        self.declare_parameter("seed", 0)
        self.declare_parameter("human_model", "actor")

        world_dir = self.get_parameter("world_dir").value
        if not world_dir:
            raise SystemExit("world_dir 파라미터를 주세요.")
        self.world_dir = Path(world_dir)
        self.truth = json.loads((self.world_dir / "ground_truth.json").read_text(encoding="utf-8"))
        self.count = int(self.get_parameter("count").value)
        self.kinds = [k.strip() for k in self.get_parameter("kinds").value.split(",") if k.strip()]
        self.ahead = (float(self.get_parameter("ahead_min").value), float(self.get_parameter("ahead_max").value))
        self.first_delay = float(self.get_parameter("first_delay").value)
        self.gap = float(self.get_parameter("gap").value)
        self.rng = random.Random(int(self.get_parameter("seed").value))
        self.human_model = self.get_parameter("human_model").value

        p = self.truth["params"]
        self.bounds = {"xmin": 0, "ymin": 0, "xmax": p["width"], "ymax": p["depth"], "zmax": p["wall_height"]}
        self.solids = self.truth["walls"] + self.truth["obstacles"] + [
            footprint(h["x"], h["y"], (2 * h["radius"], 2 * h["radius"], 1.75)) for h in self.truth["humans"]]
        self.doors = [(d["x"], d["y"]) for d in self.truth["doors"]]
        self.spawned: list[dict] = []
        self.misses = 0
        self.pose = None
        self.path: list[np.ndarray] = []
        self.state = ""
        self.follow_since = None
        self.last_spawn = -math.inf
        self.write()

        self.create_subscription(Odometry, "/tt/odometry_gt", self.on_pose, 10)
        self.create_subscription(PathMsg, "/tt/path", self.on_path, 10)
        self.create_subscription(String, "/tt/mission_state", self.on_state, 10)
        self.create_timer(0.5, self.step)
        self.get_logger().info(f"비행 중 장애물 {self.count}개를 경로 {self.ahead[0]}–{self.ahead[1]} m 앞에 만듭니다 "
                               f"({', '.join(self.kinds)})")

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_pose(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self.pose = (p.x, p.y, p.z)

    def on_path(self, msg: PathMsg) -> None:
        self.path = [np.array([q.pose.position.x, q.pose.position.y, q.pose.position.z]) for q in msg.poses]

    def on_state(self, msg: String) -> None:
        self.state = msg.data
        if self.state == "FOLLOW" and self.follow_since is None:
            self.follow_since = self.now()

    def step(self) -> None:
        if len(self.spawned) >= self.count or self.state != "FOLLOW" or self.pose is None or len(self.path) < 2:
            return
        now = self.now()
        if now - self.follow_since < self.first_delay or now - self.last_spawn < self.gap:
            return
        kind = self.kinds[len(self.spawned) % len(self.kinds)]
        spot = self.choose(kind)
        if spot is None:
            return   # 이번엔 알맞은 자리가 없다. 다음 주기에 다시
        x, y, heading, ahead = spot
        name = f"spawned_{len(self.spawned)}_{kind}"
        size = SIZES[kind]
        yaw = heading + math.pi + self.rng.uniform(-0.5, 0.5)   # 대략 드론 쪽을 본다
        if kind == "person":
            body = person_sdf(f"{name}_body", x, y, yaw) if self.human_model == "actor" else None
            ok = gz_create(collision_sdf(name, x, y, size)) and (body is None or gz_create(body))
        else:
            ok = gz_create(box_stack_sdf(name, x, y, yaw, self.rng))
        if not ok:
            self.get_logger().error(f"{name} 생성 실패")
            self.count -= 1
            return
        box = footprint(x, y, size)
        self.spawned.append({**box, "name": name, "kind": kind, "t": round(now, 3), "ahead": round(ahead, 2)})
        self.last_spawn = now
        self.write()
        self.get_logger().info(f"장애물 생성: {kind} ({x:.2f}, {y:.2f}), 경로를 따라 {ahead:.1f} m 앞")

    def choose(self, kind: str):
        spot, why = choose_spot(self.path, self.pose, SIZES[kind], self.solids, self.doors, self.bounds,
                                self.rng, self.ahead, spawned=self.spawned)
        if spot is None:
            self.misses += 1
            if self.misses % 10 == 1:
                self.get_logger().info(f"알맞은 자리가 없어 기다립니다 ({', '.join(f'{k} {v}' for k, v in why.items())})")
        return spot

    def write(self) -> None:
        (self.world_dir / "spawned.json").write_text(json.dumps({"obstacles": self.spawned}, ensure_ascii=False, indent=1),
                                                     encoding="utf-8")


def main() -> None:
    rclpy.init()
    node = Spawner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
