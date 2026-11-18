#!/usr/bin/env python3
"""(시뮬레이션 전용) Gazebo 드론을 UDP로 말하는 Tello EDU처럼 보이게 한다.

실기체와 같은 드라이버(modules/autonomy/tello_driver/tello_driver_node.py)가 이 노드에 SDK 명령을 보낸다.
  명령  UDP cmd_port ← command, takeoff, land, rc a b c d, emergency, 질의(battery? 등)
  상태  UDP → (command를 보낸 주소):state_port. 시뮬레이션 시각으로 초당 10번, 지연 30 ms
  기체  rc → 속도(1차 지연) → sim_cmd_topic (Twist, 기체 기준) → ros_gz_bridge → Gazebo VelocityControl
  전방 ToF  front_tof_topic(Gazebo 전방 ToF 가운데 줄)을 받으면 `EXT tof?`에 확장 키트처럼 답한다 (mission.launch tof:=ext)
흉내 값은 simdrone/tello_emu.py에 있다. 영상은 UDP H.264로 보내지 않고 Gazebo 카메라 토픽을 그대로 쓴다.
"""
from __future__ import annotations

import math
import queue
import socket
import sys
import threading
from collections import deque
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from simdrone.tello_emu import EmuParams, StateSynth, TelloEmu  # noqa: E402


class GzTello(Node):
    def __init__(self) -> None:
        super().__init__("tt_gz_tello")
        for name, value in (("bind", "127.0.0.1"), ("cmd_port", 28889), ("state_port", 28890), ("rate", 10.0),
                            ("latency", 0.03), ("vel_noise", 0.03), ("scale_sigma", 0.03), ("yaw_noise_deg", 0.3),
                            ("yaw_drift_deg_per_min", 1.0), ("h_noise", 0.02), ("seed", 0), ("v_full", 1.0),
                            ("vz_full", 1.0), ("yaw_full_deg", 100.0), ("tau", 0.3), ("tau_yaw", 0.15),
                            ("sim_cmd_topic", "/tt/sim/cmd_vel"), ("front_tof_topic", "")):
            self.declare_parameter(name, value)
        g = lambda k: self.get_parameter(k).value  # noqa: E731
        self.emu = TelloEmu(EmuParams(v_full=float(g("v_full")), vz_full=float(g("vz_full")),
                                      yaw_full=math.radians(float(g("yaw_full_deg"))), tau=float(g("tau")),
                                      tau_yaw=float(g("tau_yaw"))))
        self.synth = StateSynth(int(g("seed")), float(g("vel_noise")), float(g("scale_sigma")),
                                float(g("yaw_noise_deg")), float(g("yaw_drift_deg_per_min")), float(g("h_noise")))
        self.latency = float(g("latency"))
        self.state_port = int(g("state_port"))

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((g("bind"), int(g("cmd_port"))))
        self.sock.settimeout(0.2)
        self.out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.inbox: queue.Queue = queue.Queue()
        self.client = None          # command를 보낸 주소 (상태를 그 IP로 보낸다)
        self.reply_to = None        # takeoff·land 'ok'를 늦게 보낼 주소
        self.closing = threading.Event()
        threading.Thread(target=self.recv_loop, daemon=True).start()

        self.truth = None           # (x, y, z, yaw)
        self.z0 = None
        self.tof_down = None
        self.outbox: deque = deque()
        self.phase_seen = self.emu.phase
        self.pub = self.create_publisher(Twist, g("sim_cmd_topic"), 10)
        self.create_subscription(Odometry, "/tt/odometry_gt", self.on_odom, 10)
        self.create_subscription(LaserScan, "/tt/tof_down", self.on_tof, qos_profile_sensor_data)
        if g("front_tof_topic"):
            self.create_subscription(LaserScan, g("front_tof_topic"), self.on_front_tof, qos_profile_sensor_data)
        self.create_timer(0.02, self.step)
        self.create_timer(1.0 / float(g("rate")), self.sample)
        self.create_timer(0.005, self.flush)
        p = self.emu.p
        self.get_logger().info(
            f"Gazebo Tello: 명령 UDP {g('bind')}:{g('cmd_port')}, 상태 → :{self.state_port}, 속도 배율 {self.synth.scale:.3f}, "
            f"rc 100 = {p.v_full:.2f} m/s · {math.degrees(p.yaw_full):.0f}°/s, 시상수 {p.tau:.2f} s (가정)")

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def recv_loop(self) -> None:
        while not self.closing.is_set():
            try:
                data, addr = self.sock.recvfrom(1024)
            except socket.timeout:
                continue
            except OSError:
                break
            self.inbox.put((data.decode("utf-8", "replace").strip(), addr))

    def on_odom(self, msg: Odometry) -> None:
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.truth = (p.x, p.y, p.z, yaw)
        if self.z0 is None:
            self.z0 = p.z

    def on_tof(self, msg: LaserScan) -> None:
        valid = [r for r in msg.ranges if math.isfinite(r) and r > 0]
        self.tof_down = min(valid) if valid else None

    def on_front_tof(self, msg: LaserScan) -> None:
        valid = [r for r in msg.ranges if math.isfinite(r) and r > 0]
        self.emu.front_tof_m = min(valid) if valid else math.inf

    def send(self, text: str, addr) -> None:
        try:
            self.sock.sendto(text.encode(), addr)
        except OSError:
            pass

    def step(self) -> None:
        t = self.now()
        while not self.inbox.empty():
            cmd, addr = self.inbox.get_nowait()
            reply = self.emu.command(t, cmd)
            if cmd == "command":
                self.client = addr
            if cmd in ("takeoff", "land") and reply is None:
                self.reply_to = addr
            if reply is not None:
                self.send(reply, addr)
        if self.truth is None:
            return
        vel, replies = self.emu.update(t, self.truth[2] - self.z0)
        for r in replies:
            if self.reply_to:
                self.send(r, self.reply_to)
        if self.emu.phase != self.phase_seen:
            self.get_logger().info(f"Tello 흉내: {self.phase_seen} → {self.emu.phase} "
                                   f"(높이 {self.truth[2] - self.z0:.2f} m)")
            self.phase_seen = self.emu.phase
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z = (float(v) for v in vel)
        self.pub.publish(msg)

    def sample(self) -> None:
        if self.truth is None or self.client is None:
            return
        x, y, z, yaw = self.truth
        # 지금 건 수평 속도 명령을 같이 주면 걸린 만큼 닿음 반력 가속(agx·agy)이 나온다 (tello_emu.contact_accel)
        raw = self.synth.sample(self.now(), x, y, z, yaw, self.z0, self.tof_down, cmd=(self.emu.vel[0], self.emu.vel[1]))
        self.outbox.append((self.now() + self.latency, raw))

    def flush(self) -> None:
        now = self.now()
        while self.outbox and self.outbox[0][0] <= now:
            _, raw = self.outbox.popleft()
            try:
                self.out.sendto((raw + "\r\n").encode(), (self.client[0], self.state_port))
            except OSError:
                pass

    def close(self) -> None:
        self.closing.set()
        self.sock.close()
        self.out.close()


def main() -> None:
    rclpy.init()
    node = GzTello()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
