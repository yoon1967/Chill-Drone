#!/usr/bin/env python3
"""(평가 전용) 시뮬레이션 정답 위치를 CSV로 남긴다. 비행 코드는 정답을 쓰지 않으므로,
충돌·목적지 오차·위치 추정 오차는 이 기록으로 잰다 (tools/eval_flight.py).

예) python3 tools/log_truth.py --ros-args -p out:=worlds/s7_stage1/truth_log.csv -p use_sim_time:=true
"""
from __future__ import annotations

import csv
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class TruthLogger(Node):
    def __init__(self) -> None:
        super().__init__("tt_truth_logger")
        self.declare_parameter("out", "truth_log.csv")
        self.declare_parameter("period", 0.05)
        self.period = float(self.get_parameter("period").value)
        self.writer = csv.writer(open(self.get_parameter("out").value, "w", newline="", buffering=1))
        self.writer.writerow(["t", "x", "y", "z", "yaw"])
        self.last = -math.inf
        self.create_subscription(Odometry, "/tt/odometry_gt", self.on_odom, 10)

    def on_odom(self, msg: Odometry) -> None:
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t - self.last < self.period:
            return
        self.last = t
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.writer.writerow([f"{t:.3f}", f"{p.x:.3f}", f"{p.y:.3f}", f"{p.z:.3f}", f"{yaw:.3f}"])


def main() -> None:
    rclpy.init()
    node = TruthLogger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
