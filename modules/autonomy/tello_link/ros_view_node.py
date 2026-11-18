#!/usr/bin/env python3
"""자율 비행(ROS) 보기 노드 (ROS 2, WSL): 자율 비행 프로그램의 토픽을 구독해 게이트웨이가 가져가게 내준다.

자율 비행 프로그램(드라이버·중계·비행 노드·웹)은 그대로 둔다.
Windows 게이트웨이가 `gateway_server.py --ros-view http://127.0.0.1:8094`로 붙어 관제 화면에 보인다(ros_view.py).
ROS와 무관한 부분(값 보관·JPEG·HTTP·찾은 사람 파일 지켜보기·명령 문자열)은 ros_view_core.py에 있다.

구독 (자율 드라이버 tello_driver_node.py·비행 노드 mission_node.py·위치 추정 pose_node.py·YOLO 탐지 노드가 내는 것)
  /tt/state            std_msgs/String  JSON {stamp, raw}: Tello 상태 원문
  /tt/flight_status    std_msgs/String  JSON: 떠 있음·진행 중·이륙 전 점검·자동 착륙 까닭
  front_tof_topic      sensor_msgs/LaserScan  전방 ToF 한 줄(기본 /tt/tof_front, 범위 밖 inf)
  camera_topic         sensor_msgs/Image  rgb8(기본 /tt/camera/image_raw)
  /tt/pose_est         nav_msgs/Odometry  자율 위치(world = 켠 자리 원점, x 앞·y 왼쪽)
  /tt/path             nav_msgs/Path  계획 경로
  /tt/mission_state    std_msgs/String  비행 노드 단계 이름(WAIT_POSE·READY·FOLLOW·LOOK 등)
  /tt/operator_goal    std_msgs/String  JSON: 찍은 자리 상태
  /tt/mission_note     std_msgs/String  JSON: 지금 하는 일과 이유(남은 탐색 시간·돌아가는 이유·내린 이유, 1초마다)
  /tt/detections       std_msgs/String  YOLO 결과(받은 때만 본다: 3초 안이면 YOLO 준비됨)
  /tt/obstacle_grid    nav_msgs/OccupancyGrid  비행 노드가 카메라·ToF로 만드는 지도의 비행 높이 단면(자율 비행 웹이 그리는 지도)
  /rosout              rcl_interfaces/Log  자율 비행 노드들의 경고·오류(WARN 이상, 이 노드 것은 뺀다). 게이트웨이가 비행 기록에 남긴다
찾은 사람: flights_root(기본 ~/ttrepo/data/outputs/flights)의 가장 새 회차 people.json(자율 비행 웹과 같은 파일), people로 직접 줄 수도 있다.
찾은 장애물: 같은 회차의 discovered.json(자율 비행 웹이 지도에 빨갛게 그리는 것), discovered로 직접 줄 수도 있다.
지도 높이 층: 같은 회차의 camera_map.npz(비행 노드 3D 카메라 지도, 30초마다 저장)로 막힌 칸을 벽·높은 것과 낮은 물체로 나눈다.
  camera_map으로 직접 줄 수도 있다. numpy가 없으면 높이 층만 빠진다.
지도 파일(자율 비행 웹이 그리는 것): 같은 회차의 map.json(실기체 빈 지도), map으로 직접(시뮬레이터 world_dir/known_map.json: 벽·가구·목적지).
정답 위치: truth:=true면 /tt/odometry_gt(nav_msgs/Odometry, 시뮬레이션만, 자율 비행 웹 truth:=true와 같은 토픽).

명령(3단계): 관제 화면 버튼을 자율 비행 웹(autonomy_web.py)과 **같은 토픽·같은 문자열**로 낸다.
  /tt/mission_cmd  start | land | goto <x> <y> | goto cancel      /tt/flight_cmd  emergency | confirm | unconfirm | land
  받을지 말지는 비행 노드·드라이버가 정한다(기준·흐름은 그대로). commands:=false면 아무것도 내지 않는다(보기만).
need_yolo(기본 켬): 자율 비행 웹의 need_yolo처럼 YOLO가 준비돼야 관제 화면의 시작이 열린다. YOLO 없이 띄운 시뮬레이션이면 끈다.

실행 (WSL, 자율 비행 launch와 따로):
  source /opt/ros/jazzy/setup.bash
  python3 modules/autonomy/tello_link/ros_view_node.py                 # 127.0.0.1:8094
  python3 modules/autonomy/tello_link/ros_view_node.py --ros-args -p port:=8095 -p flights_root:=/path/to/flights
  python3 modules/autonomy/tello_link/ros_view_node.py --ros-args -p commands:=false   # 보기만(명령을 내지 않음)
"""

import os
import sys
import threading

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.executors import ExternalShutdownException
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from ros_view_core import (DEFAULT_PORT, FLIGHT_CMD, MISSION_CMD, ViewState, make_server, pose_sigma,  # noqa: E402
                           quat_yaw, scan_range, watch_discovered, watch_people)
from ros_view_core import watch_camera_map, watch_world_map  # noqa: E402


class RosView(Node):
    def __init__(self):
        super().__init__("gateway_ros_view")
        for name, value in (("bind", "127.0.0.1"), ("port", DEFAULT_PORT), ("camera_topic", "/tt/camera/image_raw"),
                            ("front_tof_topic", "/tt/tof_front"),
                            ("flights_root", os.path.expanduser("~/ttrepo/data/outputs/flights")), ("people", ""),
                            ("discovered", ""), ("commands", True), ("need_yolo", True), ("map", ""), ("truth", False)):
            self.declare_parameter(name, value)
        g = lambda name: self.get_parameter(name).value   # noqa: E731
        self._pubs = {}
        if g("commands"):   # 자율 비행 웹과 같은 토픽·QoS(깊이 10)
            self._pubs = {MISSION_CMD: self.create_publisher(String, MISSION_CMD, 10),
                          FLIGHT_CMD: self.create_publisher(String, FLIGHT_CMD, 10)}
        self.view = ViewState(publish=self.publish if self._pubs else None, need_yolo=bool(g("need_yolo")))
        self._stop = threading.Event()
        self.create_subscription(String, "/tt/state", lambda m: self.view.on_state(m.data), 10)
        self.create_subscription(String, "/tt/flight_status", lambda m: self.view.on_flight_status(m.data), 10)
        tof_topic, cam_topic = g("front_tof_topic"), g("camera_topic")
        # 드라이버는 ToF를 sensor_data(best effort)로, 카메라를 기본(reliable)으로 낸다. best effort로 받으면 둘 다 받는다
        self.create_subscription(LaserScan, tof_topic, lambda m: self.view.on_front_tof(scan_range(m.ranges), tof_topic),
                                 qos_profile_sensor_data)
        self.create_subscription(Image, cam_topic, lambda m: self.view.on_image(m.width, m.height, m.encoding, bytes(m.data),
                                                                                 m.step, cam_topic), qos_profile_sensor_data)
        self.create_subscription(Odometry, "/tt/pose_est", self.on_pose, 10)
        self.create_subscription(Path, "/tt/path", lambda m: self.view.on_path(
            [(p.pose.position.x, p.pose.position.y, p.pose.position.z) for p in m.poses]), 10)
        self.create_subscription(String, "/tt/mission_state", lambda m: self.view.on_mission_state(m.data), 10)
        self.create_subscription(String, "/tt/operator_goal", lambda m: self.view.on_goal(m.data), 10)
        self.create_subscription(String, "/tt/mission_note", lambda m: self.view.on_note(m.data), 10)   # 자율 비행 웹과 같은 QoS
        self.create_subscription(String, "/tt/detections", lambda m: self.view.on_detections(m.data), 10)
        self.create_subscription(OccupancyGrid, "/tt/obstacle_grid", self.on_grid, 1)   # 자율 비행 웹과 같은 QoS(깊이 1)
        self.create_subscription(Log, "/rosout", self.on_rosout, 100)   # 자율 비행 노드 경고·오류(11/17 충돌 뒤)
        root, people = g("flights_root"), g("people") or None
        threading.Thread(target=watch_people, args=(self.view, root, people, self._stop), name="ros-view-people",
                         daemon=True).start()
        threading.Thread(target=watch_discovered, args=(self.view, root, g("discovered") or None, self._stop),
                         name="ros-view-discovered", daemon=True).start()
        self.declare_parameter("camera_map", "")   # 지도 높이 층: 비행 노드 camera_map.npz(기본 가장 새 회차)
        threading.Thread(target=watch_camera_map, args=(self.view, root, g("camera_map") or None, self._stop),
                         kwargs={"log": self.get_logger().warn}, name="ros-view-heights", daemon=True).start()
        threading.Thread(target=watch_world_map, args=(self.view, root, g("map") or None, self._stop),
                         name="ros-view-map", daemon=True).start()
        if g("truth"):   # 시뮬레이션 정답 위치(자율 비행 웹과 같은 토픽·QoS)
            self.create_subscription(Odometry, "/tt/odometry_gt", self.on_truth, 10)
        self.server = make_server(self.view, g("bind"), int(g("port")))
        threading.Thread(target=self.server.serve_forever, name="ros-view-http", daemon=True).start()
        self.get_logger().info(f"자율 비행 보기: http://{g('bind')}:{g('port')} (게이트웨이 --ros-view로 붙음), "
                               f"카메라 {cam_topic}, 전방 ToF {tof_topic}, 찾은 사람 {people or root + '/<가장 새 회차>/people.json'}, "
                               f"명령 {'냄(' + ', '.join(self._pubs) + ')' if self._pubs else '안 냄(보기만)'}, "
                               f"YOLO 준비 기다림 {'켬' if g('need_yolo') else '끔'}")

    def publish(self, topic: str, text: str) -> None:
        """게이트웨이가 넘긴 명령을 자율 비행 웹과 같은 문자열로 낸다(HTTP 스레드에서 부른다)."""
        self._pubs[topic].publish(String(data=text))
        self.get_logger().info(f"관제 화면 명령: {topic} '{text}'")

    def on_pose(self, m: Odometry) -> None:
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.view.on_pose(p.x, p.y, p.z, quat_yaw(q.x, q.y, q.z, q.w), pose_sigma(m.pose.covariance))

    def on_truth(self, m: Odometry) -> None:
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.view.on_truth(p.x, p.y, p.z, quat_yaw(q.x, q.y, q.z, q.w))

    def on_rosout(self, m: Log) -> None:
        if m.name != self.get_name():   # 이 노드가 낸 것(관제 화면 명령 알림 등)은 빼고
            self.view.on_log(int(m.level), m.name, m.msg)

    def on_grid(self, m: OccupancyGrid) -> None:
        i = m.info
        data = m.data.tobytes() if hasattr(m.data, "tobytes") else bytes(v & 0xFF for v in m.data)   # int8 → 바이트(−1 = 255)
        self.view.on_grid(i.resolution, i.width, i.height, i.origin.position.x, i.origin.position.y, data)

    def close(self) -> None:
        self._stop.set()
        self.server.stopping.set()
        self.server.shutdown()
        self.server.server_close()


def main():
    rclpy.init()
    node = RosView()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):   # 실행기 끄기(SIGINT)·SIGTERM: rclpy가 먼저 받아 이것으로 끝난다
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
