#!/usr/bin/env python3
"""Tello EDU ROS 2 드라이버 노드. 실기체와 Gazebo 시뮬레이션(simulation/tt-sim/nodes/gz_tello.py)이 같은 노드를 쓴다.

입력  /tt/cmd_vel        (geometry_msgs/Twist) 기체 기준 속도 명령. 비행 노드(mission_node.py)가 20Hz로 낸다
      /tt/flight_cmd     (std_msgs/String) takeoff | land | emergency | confirm | unconfirm (운용자 확인, 웹 체크박스)
출력  /tt/state          (std_msgs/String) JSON {"stamp", "raw"}. 위치 추정 노드(pose_node.py)의 입력
      /tt/flight_status  (std_msgs/String) JSON {flying, busy, connected, can_takeoff, preflight, last_rc, last_result, ...}
      camera_topic       (sensor_msgs/Image, rgb8) video:=true일 때 Tello 영상
      front_tof_topic    (sensor_msgs/LaserScan, 한 줄) front_tof:=true일 때 확장 키트 전방 ToF. 떠 있는 동안 새 값만,
                         범위(1.2 m) 밖은 inf. 비행 노드가 지도에 없는 장애물을 피하는 데 쓴다

안전 장치 (tello_link/flight_control.py): 이륙 전 점검(SDK·상태 수신·배터리 기준 없음(10% 이하만 거절)·온도 temp_max_takeoff·운용자 확인),
rc ±rc_limit, 높이 상한, 입력이 0.3초 끊기면 제자리, 상태 1초 끊김·배터리 10% 이하·온도 temp_land 이상이면 자동 착륙.
온도 기준 기본은 flight_control.py와 같은 80·85°C다.
이 노드는 여기에 더해 속도 명령이 cmd_timeout 넘게 끊기면 착륙하고, 노드를 끄면(Ctrl+C) 착륙한다.

실행 예)
  실기체(주변 3 m 비움·가드·안전 담당 확인 뒤):
    python3 tello_driver_node.py --ros-args -p operator_confirmed:=true
  시뮬레이션: simulation/tt-sim/launch/mission.launch.py가 gz_tello.py와 함께 띄운다(link:=tello).
"""
from __future__ import annotations

import json
import math
import sys
import threading
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "tello_link"))
from driver_core import RcScale, TelloDriver  # noqa: E402
from tello_link import TelloLink  # noqa: E402

REPO = HERE.parents[2]


class TelloDriverNode(Node):
    def __init__(self) -> None:
        super().__init__("tt_tello_driver")
        for name, value in (("host", "192.168.10.1"), ("cmd_port", 8889), ("local_cmd_port", 8889),
                            ("state_port", 8890), ("bind_ip", "0.0.0.0"), ("rc_limit", 30), ("height_limit", 1.5),
                            ("v_full", 1.0), ("v_dead", 0.0), ("vz_full", 1.0), ("vz_dead", 0.0), ("yaw_full_deg", 100.0),
                            ("cmd_timeout", 2.0), ("camera_width", 0), ("rc_slew", 1.5),
                            ("operator_confirmed", False), ("log_root", str(REPO / "data" / "outputs" / "tello")),
                            ("source", "tello"), ("video", False), ("video_port", 11111), ("video_record", ""),
                            ("camera_topic", "/tt/camera/image_raw"), ("front_tof", False),
                            ("front_tof_topic", "/tt/tof_front"), ("temp_max_takeoff", 80.0), ("temp_land", 85.0),
                            ("battery_min_takeoff", 0.0), ("battery_land", 10.0)):
            self.declare_parameter(name, value)
        g = lambda k: self.get_parameter(k).value  # noqa: E731
        self.link = TelloLink(host=g("host"), cmd_port=int(g("cmd_port")), local_cmd_port=int(g("local_cmd_port")),
                              state_port=int(g("state_port")), bind_ip=g("bind_ip"), allow_flight=True,
                              log_root=g("log_root") or None, source=g("source")).start()
        self.link.update_manifest(tool="modules/autonomy/tello_driver", autonomy=True)
        scale = RcScale(float(g("v_full")), float(g("vz_full")), math.radians(float(g("yaw_full_deg"))),
                        v_dead=float(g("v_dead")), vz_dead=float(g("vz_dead")))
        # camera_width > 0이면 이 폭으로 줄여 낸다. 960×720 rgb(2 MB)를 초당 15장 내면 ROS가 큰 메시지를 버려
        # 깊이 노드가 10초에 2–10장밖에 못 받았다 (2026-10-29 실기체). 깊이·YOLO는 어차피 줄여 쓴다
        self.camera_width = int(g("camera_width"))
        if g("front_tof"):
            self.tof_pub = self.create_publisher(LaserScan, g("front_tof_topic"), qos_profile_sensor_data)
        self.driver = TelloDriver(self.link, scale, rc_limit=int(g("rc_limit")), height_limit_m=float(g("height_limit")),
                                  cmd_timeout=float(g("cmd_timeout")), log=self.get_logger().info,
                                  front_tof=bool(g("front_tof")), on_front_tof=self.publish_front_tof,
                                  temp_limits=(float(g("temp_max_takeoff")), float(g("temp_land"))),
                                  battery_limits=(float(g("battery_min_takeoff")), float(g("battery_land"))),
                                  rc_slew=float(g("rc_slew"))).start()
        if g("operator_confirmed"):
            self.driver.ctrl.confirm_operator(True)
        self.get_logger().info(
            f"Tello 드라이버: {g('host')}:{g('cmd_port')} (상태 포트 {g('state_port')}, {g('source')}), rc 상한 ±{g('rc_limit')}, "
            f"rc 100 = {scale.v_full:.2f} m/s (무시 구간 rc {scale.v_dead * 100:.0f}) · 상하 {scale.vz_full:.2f} m/s (무시 구간 rc {scale.vz_dead * 100:.0f}) · "
            f"{g('yaw_full_deg'):.0f}°/s, rc 상승 한도 초당 {float(g('rc_slew')) * 100:.0f}, "
            f"영상 {'폭 ' + str(self.camera_width) if self.camera_width else '원본 크기'}로 냄, "
            f"운용자 확인 {'됨' if g('operator_confirmed') else '안 됨 (이륙 요청을 거절합니다)'}, "
            f"전방 ToF {'→ ' + g('front_tof_topic') if g('front_tof') else '안 씀'}, "
            f"온도 기준 이륙 {g('temp_max_takeoff'):g}°C 미만·착륙 {g('temp_land'):g}°C 이상, "
            f"배터리 기준 이륙 {g('battery_min_takeoff'):g}% 이상(0이면 없음)·착륙 {g('battery_land'):g}% 이하, "
            f"높이 상한 {g('height_limit'):g} m, 기록 {self.link.run_id}")

        self.state_pub = self.create_publisher(String, "/tt/state", 10)
        self.status_pub = self.create_publisher(String, "/tt/flight_status", 10)
        self.create_subscription(Twist, "/tt/cmd_vel", self.on_cmd_vel, 10)
        self.create_subscription(String, "/tt/flight_cmd", self.on_flight_cmd, 10)
        # 상태 전달·안전 검사는 시뮬레이션 시계가 멈춰도 돌도록 실제 시계로 돈다
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.01, self.poll_state, clock=steady)
        self.create_timer(0.1, self.driver.tick, clock=steady)
        self._warned: dict | None = None   # 마지막으로 로그에 낸 드라이버 경고 (같은 것을 되풀이하지 않는다)
        self.create_timer(0.2, self.publish_status, clock=steady)

        self.video = None
        if g("video"):
            self.cam_pub = self.create_publisher(Image, g("camera_topic"), 2)
            self.last_frame = None
            threading.Thread(target=self.start_video, args=(int(g("video_port")), g("video_record") or None),
                             daemon=True).start()
            self.create_timer(1.0 / 15, self.publish_frame, clock=steady)
            self.video_seen = {"packets": 0, "frames": 0}
            self.video_ok = False
            self.create_timer(5.0, self.check_video, clock=steady)

    def ros_now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_cmd_vel(self, msg: Twist) -> None:
        self.driver.on_cmd_vel(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z)

    def on_flight_cmd(self, msg: String) -> None:
        cmd = msg.data.strip()
        ok = self.driver.request(cmd)
        st = self.driver.ctrl
        self.get_logger().info(f"요청 {cmd}: {'받음' if ok else '거절'} (떠 있음 {st.flying}, 진행 중 {st.busy}, "
                               f"연결 {self.driver.connected})")
        self.publish_status()

    def poll_state(self) -> None:
        raw = self.driver.poll_state()
        if raw:
            self.state_pub.publish(String(data=json.dumps({"stamp": round(self.ros_now(), 4), "raw": raw})))

    def publish_status(self) -> None:
        s = self.driver.status()
        w = s.get("last_warning")
        if w and w != self._warned:   # 2026-10-30 복도: 상승 입력 중에 기체가 내려앉았는데 아무 말이 없었다
            self._warned = w
            d = w.get("detail") or {}
            if w.get("type") == "climb_stalled":
                self.get_logger().warning(f"상승 입력(rc {d.get('ud')})을 {d.get('stall_s')}초 보냈는데 Tello 높이가 {d.get('h_m')} m에서 "
                                          f"오르지 않습니다(천장·매달린 것에 눌림). 이번 비행은 {d.get('cap_m')} m 위로 오르지 않고 잠깐 내려옵니다")
            else:
                self.get_logger().warning(f"기체가 상승 입력(rc {d.get('ud')})과 반대로 {d.get('window_s')}초에 {d.get('drop_m')} m 내려갔습니다 "
                                          f"(하방 ToF, Tello 높이도 {d.get('h_drop_m')} m 같이 내려감)")
        self.status_pub.publish(String(data=json.dumps(s, ensure_ascii=False, default=str)))

    def publish_front_tof(self, range_m: float, mono: float) -> None:
        """드라이버의 ToF 스레드에서 불린다. 시각은 잰 순간(왕복의 가운데)으로 적는다."""
        stamp = self.ros_now() - (time.monotonic() - mono)
        msg = LaserScan()
        msg.header.stamp.sec = int(stamp)
        msg.header.stamp.nanosec = int((stamp - int(stamp)) * 1e9)
        msg.header.frame_id = "tt_tof_front"
        msg.range_min, msg.range_max = 0.02, 1.2
        msg.ranges = [float(range_m)]
        self.tof_pub.publish(msg)

    # ---------- 영상 (실기체) ----------
    def start_video(self, port: int, record_dir: str | None = None) -> None:
        while not self.driver.connected:
            time.sleep(0.2)
        r = self.link.send("streamon", source="driver")
        if not r.ok:
            self.get_logger().error(f"streamon 실패: {r.reason or r.response}")
            return
        from tello_video import TelloVideo
        video = TelloVideo(port=port, record_dir=record_dir)   # record_dir: 받은 H.264 그대로 + 프레임 시각 저장
        if record_dir:
            self.get_logger().info(f"영상 저장: {record_dir}/video.h264, video_frames.csv")
        try:
            video.start()   # PyAV(av)로 H.264를 푼다
        except ImportError:
            self.get_logger().error("영상을 풀 PyAV가 없습니다. WSL에서 `~/venvs/tt/bin/pip install av`로 설치하고 "
                                    "드라이버를 그 파이썬(~/venvs/tt/bin/python)으로 실행하세요. 비행은 영상 없이 계속됩니다.")
            return
        self.video = video
        self.get_logger().info(f"영상 받기 시작 (UDP {port})")

    def check_video(self) -> None:
        """영상이 안 나오면 어디서 막혔는지 5초마다 남긴다. 나오기 시작하면 한 번만 알린다.

        2026-10-27 실기체: Windows 중계는 영상 패킷을 받았는데(5초에 약 570개) 웹 화면에 영상이 나오지 않았다.
        가짜 Tello로는 WSL 직접·중계 경유 모두 나왔으므로, 패킷이 WSL에 오는지와 풀리는지를 나눠 본다.
        """
        if self.video is None:
            return
        s = self.video.stats()
        dp = s["packets"] - self.video_seen["packets"]
        df = s["frames"] - self.video_seen["frames"]
        self.video_seen = {"packets": s["packets"], "frames": s["frames"]}
        if df > 0:
            if not self.video_ok:
                self.get_logger().info(f"영상 정상: {s['width']}×{s['height']}, {s['fps']} fps")
            self.video_ok = True
            return
        self.video_ok = False
        if dp == 0:
            self.get_logger().warning(f"영상 패킷이 5초 동안 오지 않습니다 (WSL UDP {s['port']}). "
                                      "중계 창의 '영상 패킷' 수와 streamon 응답을 보세요.")
        else:
            self.get_logger().warning(f"영상 패킷은 5초에 {dp}개 오는데 프레임이 풀리지 않습니다 "
                                      f"(디코딩 실패 누적 {s['decode_errors']}, 프레임 누적 {s['frames']}).")

    def publish_frame(self) -> None:
        if self.video is None:
            return
        got = self.video.rgb(self.camera_width)
        if got is None or got[1] == self.last_frame:
            return
        img, mono = got
        self.last_frame = mono
        stamp = self.ros_now() - (time.monotonic() - mono)   # 받은 순간의 시각 (디코딩까지의 지연은 모른다)
        msg = Image()
        msg.header.stamp.sec = int(stamp)
        msg.header.stamp.nanosec = int((stamp - int(stamp)) * 1e9)
        msg.header.frame_id = "tt_camera"
        msg.height, msg.width = img.shape[:2]
        msg.encoding, msg.step = "rgb8", img.shape[1] * 3
        msg.data = img.tobytes()
        self.cam_pub.publish(msg)

    def close(self) -> None:
        if self.video:
            self.video.stop()
        self.driver.close()


def main() -> None:
    rclpy.init()
    node = TelloDriverNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.get_logger().info("드라이버를 끕니다 (떠 있으면 착륙).")
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
