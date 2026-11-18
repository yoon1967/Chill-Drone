#!/usr/bin/env python3
"""드론 카메라 영상 한 장으로 거리(깊이) 영상을 추정해 낸다 (perception/depth.py). 실기체·시뮬레이션 공통.

입력  image_topic (기본 /tt/camera/image_delayed: 실제 TT처럼 늦게 도착하는 영상)
출력  /tt/depth   sensor_msgs/Image 32FC1, 광축 방향 거리(m), out_width 폭으로 줄인 것.
                  header는 원본 영상 그대로라 stamp가 촬영 시각이다 (지도에 넣을 때 그 시각의 위치를 쓴다)

오래된 프레임 처리: 처리 중에 들어온 프레임은 쌓지 않고 가장 최근 것 하나만 남긴다.
YOLO 가상환경의 파이썬으로 실행한다 (torch, transformers).
  ~/venvs/tt/bin/python nodes/depth_node.py --ros-args -p use_sim_time:=true
"""
from __future__ import annotations

import math
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from perception.depth import DepthEstimator  # noqa: E402


def image_to_rgb(msg: Image) -> np.ndarray:
    buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    return buf[:, :msg.width * 3].reshape(msg.height, msg.width, 3)


class DepthNode(Node):
    def __init__(self) -> None:
        super().__init__("tt_depth")
        self.declare_parameter("image_topic", "/tt/camera/image_delayed")
        self.declare_parameter("hfov", 1.4416)        # 카메라 수평 화각 (rad). Gazebo 82.6°; 실기체는 실측 0.955 (real.launch)
        self.declare_parameter("depth_scale", 0.0)    # 0보다 크면 화각 공식 대신 이 배율 (실기체 0.85, ToF로 맞춤 11/03)
        self.declare_parameter("rate", 5.0)
        self.declare_parameter("out_width", 240)
        hfov = float(self.get_parameter("hfov").value)
        self.est = DepthEstimator(hfov=hfov, depth_scale=float(self.get_parameter("depth_scale").value) or None)
        self.out_width = int(self.get_parameter("out_width").value)
        self.get_logger().info(f"깊이 추정 준비 ({self.est.device}, 화각 {math.degrees(hfov):.1f}°, 깊이 배율 ×{self.est.focal_scale:.2f})")
        self.latest: Image | None = None
        self.dropped = 0
        self.times: list[float] = []
        self.pub = self.create_publisher(Image, "/tt/depth", 2)
        self.create_subscription(Image, self.get_parameter("image_topic").value, self.on_image, qos_profile_sensor_data)
        self.create_timer(1.0 / float(self.get_parameter("rate").value), self.step)
        self.create_timer(10.0, self.report)

    def on_image(self, msg: Image) -> None:
        if self.latest is not None:
            self.dropped += 1
        self.latest = msg

    def step(self) -> None:
        if self.latest is None:
            return
        import cv2

        msg, self.latest = self.latest, None
        t0 = time.perf_counter()
        depth = self.est.estimate(image_to_rgb(msg))
        h = int(round(depth.shape[0] * self.out_width / depth.shape[1]))
        small = cv2.resize(depth, (self.out_width, h), interpolation=cv2.INTER_AREA).astype(np.float32)
        self.times.append(time.perf_counter() - t0)
        out = Image()
        out.header = msg.header
        out.height, out.width = small.shape
        out.encoding = "32FC1"
        out.step = out.width * 4
        out.data = small.tobytes()
        self.pub.publish(out)

    def report(self) -> None:
        if self.times:
            ms = 1000 * np.array(self.times)
            self.get_logger().info(f"깊이 추정 {len(ms)}회, 평균 {ms.mean():.0f} ms, 버린 프레임 {self.dropped}개")
            self.times.clear()


def main() -> None:
    rclpy.init()
    node = DepthNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
