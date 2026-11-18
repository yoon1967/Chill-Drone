#!/usr/bin/env python3
"""(시뮬레이션 전용) Gazebo 정답으로 Tello EDU 상태 문자열을 흉내 내 /tt/state로 낸다.

실기체에서는 Tello 드라이버(modules/autonomy/tello_driver)가 UDP 8890으로 받은 상태 문자열을 같은 형식으로 낸다.
위치 추정 노드(pose_node.py)는 둘을 구분하지 않는다. link:=direct(속도 명령을 Gazebo에 바로 넣는 비교용)일 때만 쓴다.
기본(link:=tello)은 gz_tello.py가 같은 흉내(simdrone/tello_emu.py)를 UDP로 드라이버에 보낸다.

흉내 내는 것 (모두 가정값, 실기체를 재서 바꾼다):
  초당 10번, 도착 지연 30 ms
  속도: 0.1 m/s 단위 정수, 잡음 0.03 m/s, 비행마다 배율 오차(1σ 3%)
  yaw: 1° 단위 정수, 잡음 0.3°, 누적 1°/분, 0도 방향은 전원 켤 때 방향(임의)
  높이 h: 1 cm 단위, 잡음 2 cm. 하방 tof: Gazebo 하방 거리 센서 값
출력: /tt/state (std_msgs/String, JSON {"stamp": 받은 시각, "raw": 상태 문자열})
"""
from __future__ import annotations

import json
import math
import sys
from collections import deque
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from simdrone.tello_emu import StateSynth  # noqa: E402


class TelloStateSim(Node):
    def __init__(self) -> None:
        super().__init__("tt_tello_state_sim")
        for name, value in (("rate", 10.0), ("latency", 0.03), ("vel_noise", 0.03), ("scale_sigma", 0.03),
                            ("yaw_noise_deg", 0.3), ("yaw_drift_deg_per_min", 1.0), ("h_noise", 0.02), ("seed", 0)):
            self.declare_parameter(name, value)
        g = lambda k: self.get_parameter(k).value  # noqa: E731
        self.latency = float(g("latency"))
        self.synth = StateSynth(int(g("seed")), float(g("vel_noise")), float(g("scale_sigma")),
                                float(g("yaw_noise_deg")), float(g("yaw_drift_deg_per_min")), float(g("h_noise")))

        self.truth = None        # (t, x, y, z, yaw)
        self.z0 = None
        self.tof_down = None
        self.queue: deque = deque()
        self.pub = self.create_publisher(String, "/tt/state", 10)
        self.create_subscription(Odometry, "/tt/odometry_gt", self.on_odom, 10)
        self.create_subscription(LaserScan, "/tt/tof_down", self.on_tof, qos_profile_sensor_data)
        self.create_timer(1.0 / float(g("rate")), self.sample)
        self.create_timer(0.005, self.flush)
        self.get_logger().info(f"Tello 상태값 흉내: 속도 배율 {self.synth.scale:.3f}, yaw 누적 "
                               f"{float(g('yaw_drift_deg_per_min')):.1f}°/분, 지연 {self.latency * 1000:.0f} ms")

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, msg: Odometry) -> None:
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.truth = (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, p.x, p.y, p.z, yaw)
        if self.z0 is None:
            self.z0 = p.z

    def on_tof(self, msg: LaserScan) -> None:
        valid = [r for r in msg.ranges if math.isfinite(r) and r > 0]
        self.tof_down = min(valid) if valid else None

    def sample(self) -> None:
        if self.truth is None:
            return
        t, x, y, z, yaw = self.truth
        raw = self.synth.sample(t, x, y, z, yaw, self.z0, self.tof_down)
        self.queue.append((self.now() + self.latency, raw))

    def flush(self) -> None:
        now = self.now()
        while self.queue and self.queue[0][0] <= now:
            _, raw = self.queue.popleft()
            self.pub.publish(String(data=json.dumps({"stamp": round(now, 4), "raw": raw})))


def main() -> None:
    rclpy.init()
    node = TelloStateSim()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
