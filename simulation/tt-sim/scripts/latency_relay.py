#!/usr/bin/env python3
"""카메라 영상을 정해진 시간만큼 늦춰 다시 내보낸다.

실제 TT는 Wi-Fi로 영상을 보내 PC에 도착하기까지 평균 약 175 ms가 걸린다
(AirSwarm, arXiv 2503.06890 측정치). 시뮬레이션 영상은 지연이 거의 없으므로
인식·제어 코드는 이 노드가 내는 /tt/camera/image_delayed를 쓰게 한다.
헤더의 촬영 시각(stamp)은 바꾸지 않으므로, 받는 쪽은 실제처럼 '오래된 프레임'을 보게 된다.
"""
from collections import deque

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class LatencyRelay(Node):
    def __init__(self) -> None:
        super().__init__("tt_video_latency")
        self.declare_parameter("delay_ms", 175)
        self.declare_parameter("input_topic", "/tt/camera/image_raw")
        self.declare_parameter("output_topic", "/tt/camera/image_delayed")
        self.delay = Duration(nanoseconds=int(self.get_parameter("delay_ms").value) * 1_000_000)
        self.queue: deque[tuple] = deque()
        self.pub = self.create_publisher(
            Image, self.get_parameter("output_topic").value, qos_profile_sensor_data)
        self.create_subscription(
            Image, self.get_parameter("input_topic").value, self.on_image, qos_profile_sensor_data)
        self.create_timer(0.005, self.flush)

    def on_image(self, msg: Image) -> None:
        self.queue.append((self.get_clock().now(), msg))

    def flush(self) -> None:
        now = self.get_clock().now()
        while self.queue and now - self.queue[0][0] >= self.delay:
            self.pub.publish(self.queue.popleft()[1])


def main() -> None:
    rclpy.init()
    node = LatencyRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
