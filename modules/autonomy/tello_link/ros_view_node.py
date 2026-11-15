#!/usr/bin/env python3
"""자율 비행(ROS) 보기 노드 (ROS 2, WSL): 자율 비행 프로그램의 토픽을 구독해 게이트웨이가 가져가게 내준다.

자율 비행 프로그램(드라이버·중계·비행 노드·웹)은 그대로 둔다. 이 노드는 구독만 하고 아무것도 내지 않는다(1단계).
Windows 게이트웨이가 `gateway_server.py --ros-view http://127.0.0.1:8094`로 붙어 관제 화면에 보인다(ros_view.py).
ROS와 무관한 부분(값 보관·JPEG·HTTP)은 ros_view_core.py에 있다.

구독 (자율 드라이버 tello_driver_node.py가 내는 것)
  /tt/state            std_msgs/String  JSON {stamp, raw}: Tello 상태 원문
  /tt/flight_status    std_msgs/String  JSON: 떠 있음·진행 중·이륙 전 점검·자동 착륙 까닭
  front_tof_topic      sensor_msgs/LaserScan  전방 ToF 한 줄(기본 /tt/tof_front, 범위 밖 inf)
  camera_topic         sensor_msgs/Image  rgb8(기본 /tt/camera/image_raw)

실행 (WSL, 자율 비행 launch와 따로):
  source /opt/ros/jazzy/setup.bash
  python3 modules/autonomy/tello_link/ros_view_node.py                 # 127.0.0.1:8094
  python3 modules/autonomy/tello_link/ros_view_node.py --ros-args -p port:=8095
"""

import os
import sys
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from ros_view_core import DEFAULT_PORT, ViewState, make_server, scan_range  # noqa: E402


class RosView(Node):
    def __init__(self):
        super().__init__("gateway_ros_view")
        for name, value in (("bind", "127.0.0.1"), ("port", DEFAULT_PORT), ("camera_topic", "/tt/camera/image_raw"),
                            ("front_tof_topic", "/tt/tof_front")):
            self.declare_parameter(name, value)
        g = lambda name: self.get_parameter(name).value   # noqa: E731
        self.view = ViewState()
        self.create_subscription(String, "/tt/state", lambda m: self.view.on_state(m.data), 10)
        self.create_subscription(String, "/tt/flight_status", lambda m: self.view.on_flight_status(m.data), 10)
        tof_topic, cam_topic = g("front_tof_topic"), g("camera_topic")
        # 드라이버는 ToF를 sensor_data(best effort)로, 카메라를 기본(reliable)으로 낸다. best effort로 받으면 둘 다 받는다
        self.create_subscription(LaserScan, tof_topic, lambda m: self.view.on_front_tof(scan_range(m.ranges), tof_topic),
                                 qos_profile_sensor_data)
        self.create_subscription(Image, cam_topic, lambda m: self.view.on_image(m.width, m.height, m.encoding, bytes(m.data),
                                                                                 m.step, cam_topic), qos_profile_sensor_data)
        self.server = make_server(self.view, g("bind"), int(g("port")))
        threading.Thread(target=self.server.serve_forever, name="ros-view-http", daemon=True).start()
        self.get_logger().info(f"자율 비행 보기: http://{g('bind')}:{g('port')} (게이트웨이 --ros-view로 붙음), "
                               f"카메라 {cam_topic}, 전방 ToF {tof_topic}")

    def close(self) -> None:
        self.server.stopping.set()
        self.server.shutdown()
        self.server.server_close()


def main():
    rclpy.init()
    node = RosView()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
