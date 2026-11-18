#!/usr/bin/env python3
"""정답 위치 없이 드론 위치를 추정해 /tt/pose_est로 낸다. 실기체·시뮬레이션 공통.

입력  /tt/state       Tello 상태 문자열 JSON {"stamp", "raw"} (실기체 드라이버 또는 sim_tello_state.py)
      /tt/tof_front   전방 ToF 가운데 줄 (지도 보정에 쓴다)
      map             known_map.json: 출발 위치·방향(spawn)과 받은 지도
출력  /tt/pose_est    nav_msgs/Odometry. 공분산 대각에 위치 불확실성(σ²)을 넣는다
      input_log       (선택) 받은 상태 문자열·ToF를 그대로 적은 CSV. tools/replay_pose.py --inputs로 똑같이 다시 돌린다
      /tt/relocalize  위치 다시 잡기 요청 JSON {"xy": m, "yaw": rad}. 위치 후보를 그만큼 넓게 흩는다
                      (비행 노드가 보내고, 제자리에서 천천히 한 바퀴 돈다)

드론은 known_map.json의 spawn 위치에 spawn 방향을 보게 놓고 시작한다고 가정한다 (실기체도 같은 규칙).
지도 보정은 비행 높이 근처(±0.25 m)에서만 한다 (지도 단면이 그 높이 기준이라).
"""
from __future__ import annotations

import csv
import json
import math
import sys
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from autonomy import tello_state  # noqa: E402
from autonomy.state_estimator import EstimatorParams, PoseEstimator, occupancy_slice  # noqa: E402

TOF_HALF = math.radians(12.5)   # nodes/mission_node.py·model.sdf와 같게


class PoseNode(Node):
    def __init__(self) -> None:
        super().__init__("tt_pose")
        self.declare_parameter("map", "")
        self.declare_parameter("use_map", True)
        self.declare_parameter("flight_height", 1.0)
        self.declare_parameter("n_particles", 300)
        self.declare_parameter("seed", 0)
        self.declare_parameter("input_log", "")
        # 높이: "h"(Tello h, 시뮬레이션) 또는 "tof"(하방 ToF, 실기체. real.launch.py가 준다)
        self.declare_parameter("height_source", "h")
        # 상태 속도값 규칙(autonomy/tello_state.py CONVENTION). 실기체 시험으로 바뀌면 실행 인자로 고친다
        conv = tello_state.CONVENTION
        self.declare_parameter("vel_frame", conv["vel_frame"])
        self.declare_parameter("yaw_sign", conv["yaw_sign"])
        self.declare_parameter("vx_sign", conv["vx_sign"])
        self.declare_parameter("vy_sign", conv["vy_sign"])
        self.declare_parameter("vel_scale", 1.0)   # 수평 속도값에 곱한다 (줄자로 잰 거리 ÷ 추정 거리)
        self.conv = {"vel_frame": str(self.get_parameter("vel_frame").value),
                     "yaw_sign": float(self.get_parameter("yaw_sign").value),
                     "vx_sign": float(self.get_parameter("vx_sign").value),
                     "vy_sign": float(self.get_parameter("vy_sign").value)}
        self.vel_scale = float(self.get_parameter("vel_scale").value)
        data = json.loads(Path(self.get_parameter("map").value).read_text(encoding="utf-8"))
        self.height = float(self.get_parameter("flight_height").value)
        spawn = data["spawn"]
        occ, origin, res = occupancy_slice(data, self.height, 0.05) if data.get("walls") else (None, (0, 0), 0.05)
        # 벽·문 위 벽만: ToF가 뚫고 볼 수 없는 것 (가구는 빈틈으로 상자보다 멀게 보일 수 있다)
        solid = occupancy_slice({**data, "obstacles": []}, self.height, 0.05)[0] if occ is not None else None
        params = EstimatorParams(n_particles=int(self.get_parameter("n_particles").value),
                                 use_map=bool(self.get_parameter("use_map").value),
                                 height_source=str(self.get_parameter("height_source").value))
        self.est = PoseEstimator((spawn["x"], spawn["y"]), spawn.get("yaw", 0.0), spawn.get("z", 0.0),
                                 occ2d=occ, origin=origin, res=res, params=params,
                                 seed=int(self.get_parameter("seed").value), solid2d=solid)
        self.fan = np.linspace(-TOF_HALF, TOF_HALF, 7)
        path = self.get_parameter("input_log").value
        self.inputs = csv.writer(open(path, "w", newline="", buffering=1)) if path else None
        if self.inputs:
            self.inputs.writerow(["t", "kind", "value"])
        self.pub = self.create_publisher(Odometry, "/tt/pose_est", 10)
        self.create_subscription(String, "/tt/state", self.on_state, 10)
        self.create_subscription(LaserScan, "/tt/tof_front", self.on_tof, qos_profile_sensor_data)
        self.create_subscription(String, "/tt/relocalize", self.on_relocalize, 10)
        self.create_subscription(String, "/tt/under_map", self.on_under, 10)   # 카메라 지도가 아는 기체 밑 가구 꼭대기 (비행 노드, 11/09)
        mode = "속도 적분 + 지도 보정" if self.est.use_map else "속도 적분만"
        self.get_logger().info(f"위치 추정 시작: {mode}, 출발 ({spawn['x']:.2f}, {spawn['y']:.2f}), "
                               f"방향 {math.degrees(spawn.get('yaw', 0.0)):.0f}°, 속도 규칙 {self.conv}, "
                               f"속도 배율 {self.vel_scale:g}")

    def on_state(self, msg: String) -> None:
        data = json.loads(msg.data)
        if self.inputs:
            self.inputs.writerow([f"{float(data['stamp']):.4f}", "state", data["raw"]])
        si = tello_state.to_si(tello_state.parse(data["raw"]), self.conv)
        si["vx"] *= self.vel_scale
        si["vy"] *= self.vel_scale
        self.est.on_state(si, float(data["stamp"]))
        x, y, z, yaw, sigma = self.est.estimate()
        out = Odometry()
        out.header.frame_id = "world"
        out.child_frame_id = "tt_est"
        sec = int(data["stamp"])
        out.header.stamp.sec, out.header.stamp.nanosec = sec, int((float(data["stamp"]) - sec) * 1e9)
        out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = x, y, z
        out.pose.pose.orientation.z, out.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        var = sigma * sigma if math.isfinite(sigma) else -1.0   # -1: 모름 (속도 적분만)
        cov = [0.0] * 36
        cov[0] = cov[7] = var
        out.pose.covariance = cov
        self.pub.publish(out)

    def on_relocalize(self, msg: String) -> None:
        data = json.loads(msg.data)
        if self.inputs:
            self.inputs.writerow([f"{self.get_clock().now().nanoseconds * 1e-9:.4f}", "spread", msg.data])
        self.est.spread(float(data["xy"]), float(data["yaw"]))
        self.get_logger().info(f"위치 다시 잡기: 후보를 {data['xy']:.2f} m 넓게 흩었습니다")

    def on_under(self, msg: String) -> None:
        data = json.loads(msg.data)
        if self.inputs:
            self.inputs.writerow([f"{self.get_clock().now().nanoseconds * 1e-9:.4f}", "under", msg.data])
        top = data.get("top")
        self.est.on_under(None if top is None else float(top), float(data["t"]))

    def on_tof(self, msg: LaserScan) -> None:
        valid = [r for r in msg.ranges if math.isfinite(r) and r > 0]
        measured = min(valid) if valid else math.inf
        if self.inputs:
            self.inputs.writerow([f"{self.get_clock().now().nanoseconds * 1e-9:.4f}", "tof", f"{measured:.3f}"])
        if abs(self.est.z - self.height) > 0.25:
            return
        self.est.on_tof(measured, self.fan, self.get_clock().now().nanoseconds * 1e-9)


def main() -> None:
    rclpy.init()
    node = PoseNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
