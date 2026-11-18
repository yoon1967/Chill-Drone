#!/usr/bin/env python3
"""실기체 자율 명령 시험: 드라이버를 거쳐 이륙 → 정해 둔 동작들 → 착륙. 끝에 드론이 '스스로 잰' 이동을 요약한다.

드라이버 노드(tello_driver_node.py)가 떠 있어야 한다. 사람이 키보드로 조종하지 않고 이 노드가 /tt/cmd_vel·/tt/flight_cmd를 낸다.
이륙 지점에 테이프로 십자와 기수 방향 화살표를 붙이고, 착륙 지점까지 앞뒤·좌우 거리를 줄자로 재서 끝에 나오는
'상태값으로 적분한 이동'과 비교한다. 이 차이로 속도 단위·배율, 바닥 인식(VPS) 상태, 속도·방향 부호를 확인한다.

  python3 hover_check.py --ros-args -p hold:=5.0                          # 이륙 → 5초 제자리 → 착륙
  python3 hover_check.py --ros-args -p step_speed:=0.2 -p step_time:=2.0    # 제자리 → 0.2 m/s로 2초 앞 → 제자리
  python3 hover_check.py --ros-args -p steps:="hold 10"                     # 실험: 동작을 쉼표로 이어 쓴다
  python3 hover_check.py --ros-args -p steps:="hold 3, fwd 0.2 3, hold 3, back 0.2 3, hold 3"
  python3 hover_check.py --ros-args -p steps:="hold 3, turn -30 3, hold 3"  # 오른쪽(시계 방향)으로 30°/s × 3초

동작: hold 초 | fwd·back·left·right 속도(m/s) 초 | turn 각속도(°/s, + 왼쪽=반시계) 초
안전 한도: 속도 0.3 m/s, 회전 45°/s, 한 동작 10초, 전체 60초 (넘으면 시작하지 않는다)

멈추기: 이 노드를 Ctrl+C로 끄면 속도 명령이 끊겨 드라이버가 2초 뒤 착륙시킨다(cmd_timeout).
바로 내려야 하면 드라이버 노드를 Ctrl+C로 끈다(끄면서 착륙). 위급하면 드라이버의 비상 정지(/tt/flight_cmd emergency).
"""
from __future__ import annotations

import json
import math
import statistics

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

MAX_SPEED, MAX_TURN_DEG, MAX_SEG_S, MAX_TOTAL_S = 0.3, 45.0, 10.0, 60.0
MOVES = {"fwd": (1, 0), "back": (-1, 0), "left": (0, 1), "right": (0, -1)}


def parse_steps(text: str) -> list[dict]:
    """'hold 3, fwd 0.2 3, turn -30 3' → [{name, dur, vx, vy, wz}]. 잘못됐거나 한도를 넘으면 ValueError."""
    steps = []
    for part in [p.strip() for p in text.split(",") if p.strip()]:
        w = part.split()
        kind = w[0].lower()
        if kind == "hold" and len(w) == 2:
            steps.append({"name": part, "dur": float(w[1]), "vx": 0.0, "vy": 0.0, "wz": 0.0})
        elif kind in MOVES and len(w) == 3:
            v = float(w[1])
            if not 0 < v <= MAX_SPEED:
                raise ValueError(f"{part}: 속도는 0보다 크고 {MAX_SPEED} m/s 이하")
            sx, sy = MOVES[kind]
            steps.append({"name": part, "dur": float(w[2]), "vx": sx * v, "vy": sy * v, "wz": 0.0})
        elif kind == "turn" and len(w) == 3:
            deg = float(w[1])
            if not 0 < abs(deg) <= MAX_TURN_DEG:
                raise ValueError(f"{part}: 회전은 0이 아니고 ±{MAX_TURN_DEG:g}°/s 이하")
            steps.append({"name": part, "dur": float(w[2]), "vx": 0.0, "vy": 0.0, "wz": math.radians(deg)})
        else:
            raise ValueError(f"모르는 동작: {part}")
        if not 0 < steps[-1]["dur"] <= MAX_SEG_S:
            raise ValueError(f"{part}: 한 동작은 0초보다 길고 {MAX_SEG_S:g}초 이하")
    if not steps:
        raise ValueError("동작이 없습니다")
    if sum(s["dur"] for s in steps) > MAX_TOTAL_S:
        raise ValueError(f"전체 시간이 {MAX_TOTAL_S:g}초를 넘습니다")
    return steps


class HoverCheck(Node):
    def __init__(self) -> None:
        super().__init__("tt_hover_check")
        for name, value in (("hold", 5.0), ("step_speed", 0.0), ("step_time", 2.0), ("takeoff_wait", 30.0), ("steps", "")):
            self.declare_parameter(name, value)
        g = lambda k: self.get_parameter(k).value  # noqa: E731
        text = str(g("steps")).strip()
        if not text:   # 예전 방식: 제자리 → (앞으로) → 제자리
            hold, speed = float(g("hold")), float(g("step_speed"))
            text = f"hold {hold:g}" + (f", fwd {speed:g} {float(g('step_time')):g}, hold {hold:g}" if speed else "")
        self.plan = parse_steps(text)
        self.takeoff_wait = float(g("takeoff_wait"))
        self.status: dict = {}
        self.phase = "wait"       # wait → takeoff → seg → land → done
        self.phase_t = None
        self.index = -1
        self.samples: list[dict] = []
        self.cmd_pub = self.create_publisher(Twist, "/tt/cmd_vel", 10)
        self.req_pub = self.create_publisher(String, "/tt/flight_cmd", 10)
        self.create_subscription(String, "/tt/flight_status", self.on_status, 10)
        self.create_subscription(String, "/tt/state", self.on_state, 10)
        self.create_timer(0.05, self.step)
        self.get_logger().info("시험 순서: 이륙 → " + " → ".join(s["name"] for s in self.plan) + " → 착륙")

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_status(self, msg: String) -> None:
        self.status = json.loads(msg.data)

    def on_state(self, msg: String) -> None:
        data = json.loads(msg.data)
        raw = dict(p.split(":", 1) for p in data["raw"].strip(";").split(";") if ":" in p)
        try:
            self.samples.append({"t": float(data["stamp"]), "phase": self.phase, "seg": self.index,
                                 **{k: float(raw[k]) for k in ("vgx", "vgy", "yaw", "h", "tof")}})
        except (KeyError, ValueError):
            pass

    def send(self, vx: float = 0.0, vy: float = 0.0, wz: float = 0.0) -> None:
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = vx, vy, wz
        self.cmd_pub.publish(msg)

    def request(self, cmd: str) -> None:
        self.req_pub.publish(String(data=cmd))
        self.get_logger().info(f"요청: {cmd}")

    def set_phase(self, name: str) -> None:
        self.phase, self.phase_t = name, self.now()

    @property
    def finished(self) -> bool:
        return self.phase == "done"

    def step(self) -> None:
        flying = bool(self.status.get("flying"))
        if self.phase == "wait":
            self.send()
            if not self.status.get("connected"):
                return
            if not self.status.get("can_takeoff"):
                failed = [c["key"] for c in self.status.get("preflight", []) if not c["ok"]]
                self.get_logger().info(f"이륙 전 점검 대기: {failed}", throttle_duration_sec=2.0)
                return
            self.request("takeoff")
            self.set_phase("takeoff")
        elif self.phase == "takeoff":
            self.send()
            if flying:
                self.index = 0
                self.set_phase("seg")
            elif self.now() - self.phase_t > self.takeoff_wait:
                self.get_logger().error(f"{self.takeoff_wait:.0f}초 안에 뜨지 않았습니다: {self.status.get('last_result')}")
                self.set_phase("done")
        elif self.phase == "seg":
            if not flying:
                self.get_logger().error(f"비행 중 내려앉았습니다: {self.status.get('last_auto_land') or self.status.get('landing_reason')}")
                self.set_phase("done")
                return
            s = self.plan[self.index]
            self.send(s["vx"], s["vy"], s["wz"])
            if self.now() - self.phase_t >= s["dur"]:
                self.index += 1
                if self.index < len(self.plan):
                    self.set_phase("seg")
                    self.get_logger().info(f"동작 {self.index + 1}: {self.plan[self.index]['name']}")
                else:
                    self.send()
                    self.request("land")
                    self.set_phase("land")
        elif self.phase == "land":
            self.send()
            if not flying:
                self.report()
                self.set_phase("done")
        else:
            self.send()

    def report(self) -> None:
        air = [s for s in self.samples if s["phase"] in ("seg", "land")]
        for i, seg in enumerate(self.plan):
            rows = [s for s in air if s["phase"] == "seg" and s["seg"] == i]
            if not rows:
                continue
            speed = [math.hypot(r["vgx"], r["vgy"]) / 10 for r in rows]
            dyaw = rows[-1]["yaw"] - rows[0]["yaw"]
            self.get_logger().info(
                f"{i + 1}) {seg['name']}: 상태 {len(rows)}개, 수평 속도 중앙 {statistics.median(speed):.2f} 최대 {max(speed):.2f} m/s, "
                f"vgx 중앙 {statistics.median(r['vgx'] for r in rows):.0f} · vgy 중앙 {statistics.median(r['vgy'] for r in rows):.0f}, "
                f"yaw {rows[0]['yaw']:.0f}° → {rows[-1]['yaw']:.0f}° ({dyaw:+.0f}°), 하방 ToF 중앙 {statistics.median(r['tof'] for r in rows):.0f} cm")
        dx = dy = 0.0
        for a, b in zip(air, air[1:]):
            dt = min(max(b["t"] - a["t"], 0.0), 0.3)
            dx += a["vgx"] / 10 * dt
            dy += a["vgy"] / 10 * dt
        nominal = [(s["vx"] * s["dur"], s["vy"] * s["dur"]) for s in self.plan]
        nx, ny = sum(n[0] for n in nominal), sum(n[1] for n in nominal)
        yaw0 = air[0]["yaw"] if air else float("nan")
        self.get_logger().info(
            f"상태값으로 적분한 이동 (이륙~착륙, dm/s로 봄): vgx 방향 {dx:+.2f} m, vgy 방향 {dy:+.2f} m, 크기 {math.hypot(dx, dy):.2f} m "
            f"(이륙 때 yaw {yaw0:.0f}°)")
        self.get_logger().info(
            f"명령한 이동: 기수 기준 앞 {nx:+.2f} m, 왼쪽 {ny:+.2f} m. 줄자로 잰 착륙 지점(앞뒤·좌우)을 함께 적어 주세요.")
        self.get_logger().info("시험 끝.")


def main() -> None:
    rclpy.init()
    try:
        node = HoverCheck()
    except ValueError as e:
        print(f"시험을 시작하지 않습니다: {e}")
        rclpy.try_shutdown()
        return
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
        for _ in range(10):   # 마지막 정지 명령이 나가도록 잠깐 더 돈다
            node.send()
            rclpy.spin_once(node, timeout_sec=0.05)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
