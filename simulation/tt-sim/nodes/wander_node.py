#!/usr/bin/env python3
"""지도 없이 둘러보기: 전방 ToF 하나로 공간을 돌아다닌다. 실기체(확장 키트) 첫 자율 탐색용.

받는 지도는 **공간 테두리와 출발점만** 쓴다(가구는 무시). 테두리는 추정 위치가 바깥으로 나가지 않게 하는 울타리다.
  앞이 비었으면 앞으로 간다. ToF에 무엇이 보이기 시작하면(1.2 m 안) 속도를 절반으로 줄이고 가면서 미리 방향을 튼다.
  그래도 앞 clear 안까지 가까워지거나, 울타리·이미 부딪힌 자리가 ahead 앞이면 제자리에서 돌아 방향을 바꾼다:
  거친 격자(0.25 m)에 ToF로 본 자리(막힘)와 지나온 자리를 적어 두고, 지금 방향은 빼고 아직 안 가 본 쪽으로 멀리
  비어 있는 방향을 고른다. ToF가 stop_distance보다 가까우면 멈춰 조금 물러난 뒤 방향을 바꾼다.
  explore_time이 지나면 그 자리에 착륙한다(출발점으로 돌아가지 않는다: 속도 적분 위치를 믿지 않는다).

옆 여유 (2026-10-29): ToF 빛줄기는 앞만 보므로, 앞이 비었다고 직진하면 예전에 본 가구 모서리를 옆으로 스친다.
0.3 m/s Gazebo 실습실에서 가장 가까웠던 순간은 모두 장애물이 기수 기준 오른쪽 60–135°에 있었다(-0.02·0.02 m).
그래서 앞을 볼 때 한 점이 아니라 폭 2×SIDE의 띠(드론이 지나갈 자리)를 격자에서 확인한다.
  - 직진: 앞 LOOK(속도에 비례) 안의 띠에 본 자리가 있으면 방향을 바꾼다
  - 방향 고르기: 띠가 막히기 전까지의 거리로 점수를 낸다
  - 속도: 본 자리가 가까울수록 줄이고, 가면서 틀 때는 speed의 SOFT_SPEED배로 간다

카메라 근거 (depth_topic, 선택): 카메라 한 장 깊이 추정(nodes/depth_node.py, /tt/depth)으로 화각 82° 안의 물체를 본다.
ToF 빛줄기(25°) 밖 옆쪽 가구를 미리 띠 검사에 넣는 것이 목적이다. 단안 깊이는 가까울수록 짧게 틀리고(0.3–1.2 m에서
실제가 1.2–1.3배) 멀면 흔들리므로 CAM_RANGE(1.0–3.0 m) 안, 비행 높이 ±CAM_BAND 안의 점만 쓴다. 바닥 화소로 장면 배율을
맞추고(perception.depth.floor_scale), 같은 칸이 CAM_HITS 장면에서 보여야 막힌 자리로 친다. 광선이 지나간 칸은 점수를 깎아
한두 장 튄 헛 장애물이 사라지게 한다. 가까운 정면은 여전히 ToF가 맡는다.

카메라로 바로 판단 (2026-10-29): 격자는 위치가 틀리면 같이 틀리므로, 장면마다 위치와 상관없이 판단하는 층을 둔다.
  - 배율: 깊이 영상 가운데(ToF 원뿔 ±12.5°)를 ToF 실측에 맞춘다(perception.depth.scale_to_anchor). ToF가 없으면 바닥으로
  - 부채꼴: 화각 82°를 SECTORS개로 나눠, 비행 높이 띠 안 화소의 가까운 쪽 거리(10번째 백분위)를 칸마다 낸다
  - 진행 방향(±15°)이 cam_clear(0.6 + speed m)보다 가까우면 보이는 쪽 중 가장 넓게 빈 부채꼴로 가면서 튼다.
    보이는 쪽이 다 막혔으면 제자리에서 돈다(격자로 화각 밖 방향을 고른다). CAM_SLOW m 안에 무엇이 있으면 감속한다
  - 0.4 m 안 비상 정지는 그대로 ToF가 맡는다 (단안 깊이는 가까울수록 짧게 틀린다)

mission_node.py와 같은 토픽을 쓴다: /tt/cmd_vel·/tt/flight_cmd(드라이버), /tt/pose_est(위치 추정), /tt/tof_front,
/tt/mission_cmd start|land, /tt/mission_state. 자율비행 웹 화면도 그대로 쓴다(찾은 자리는 discovered_out으로).

한계 (실기체): ToF는 비행 높이에서 앞으로 빛줄기 하나(원뿔 약 25°, 최대 1.2 m)다. 비행 높이(1.0 m)보다 낮은 의자·책상 모서리,
얇은 기둥, 비스듬한 면은 못 본다. 책상 위로 지나가면 하방 ToF가 책상 면을 재서 그만큼 더 올라간다.

  ros2 launch launch/real.launch.py mode:=wander speed:=0.2 search_time:=60      (실기체는 늘 빈 지도, 11/02)
"""
from __future__ import annotations

import csv
import json
import math
import sys
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from autonomy.shape import front_shape, rotate_scene, scene_ranges  # noqa: E402
from autonomy.speed import FAST_FOV, FAST_FULL, boosted, fast_factor  # noqa: E402
from autonomy.tello_state import describe_landing  # noqa: E402
from perception.depth import floor_scale, scale_to_anchor  # noqa: E402

SENSOR_X = 0.05              # 기체 중심에서 ToF까지 앞쪽 거리
TOF_MAX = 1.2                # 확장 키트 전방 ToF 최대 거리
TOF_TOPICS = ("/tt/tof_front", "/tt/tof_front_up", "/tt/tof_front_down")
CELL = 0.25                  # 격자 한 칸 (m)
AHEAD = 0.6                  # 이만큼 앞의 칸이 울타리 밖이거나 막힌 자리면 방향을 바꾼다 (m)
RAY = 2.5                    # 방향을 고를 때 내다보는 거리 (m)
YAW_RATE = 0.8               # 제자리 회전 속도 (rad/s)
VZ_MAX = 0.3
K_POS = 0.8
TAKEOFF_TOL = 0.08           # 비행 높이에 이만큼 안이면 둘러보기 시작
CLIMB_MIN = 0.15             # 실기체 Tello는 작은 rc(상하 3–7)를 무시한다
TAKEOFF_RETRY, TAKEOFF_TRIES, LAND_RETRY = 20.0, 3, 10.0
STUCK_TIME = 2.5             # 앞으로 가라는데 이 시간 넘게 제자리면 (ToF가 못 본 것에 걸림) 막힌 자리로 본다
BACKOFF_DIST, BACKOFF_TIME = 0.2, 2.0
# 비상 정지 제동: 처음 BRAKE_TIME 동안은 세게 뒤로 밀어 앞으로 가던 관성을 끊는다. 2026-10-29 실기체는 0.4 m/s로
# 가다 0.32 m에서 멈췄지만 뒤로 0.15 m/s(rc −25)로는 못 멈춰 0.11 m까지 다가가 모니터에 닿았다
BRAKE_SPEED, BRAKE_TIME = 0.4, 0.5
MAX_TURN_FAILS = 3           # 방향을 바꿔도 계속 막히면 그 자리에 내린다
SIDE = 0.35                  # 띠 반폭 (m): 드론 반폭 0.09 m + 여유. 진행선에서 이만큼 옆의 본 자리도 막힌 것으로 본다
LOOK_BASE, LOOK_GAIN = 0.4, 1.5   # 직진할 때 띠를 내다보는 거리 = LOOK_BASE + LOOK_GAIN × speed (m)
SOFT_SPEED = 0.35            # 가면서 방향을 틀 때 speed에 곱한다 (도는 동안 기체가 옆으로 쓸고 지나간다)
NEAR_SLOW = (0.3, 0.8)       # 가장 가까운 본 자리가 이 범위면 속도를 0.4–1배로 줄인다 (m)
HFOV = 1.4416                # Tello 카메라 수평 화각 (rad, 82.6°)
CAM_RANGE = (1.0, 3.0)       # 카메라 깊이를 막힘으로 믿는 거리 (m)
CAM_BAND = 0.35              # 비행 높이 ±이만큼 안의 점만 (위아래 가구 윗면·바닥 제외)
CAM_HITS = 3                 # 이만큼의 장면에서 보여야 막힌 자리
CAM_STRIDE = 6               # 깊이 영상 몇 화소마다 쓸지
SECTORS = 9                  # 화각을 이만큼의 부채꼴로 나눈다 (82° / 9 ≈ 9°)
CAM_AHEAD = math.radians(15)  # 진행 방향으로 보는 부채꼴 폭 (±)
CAM_SLOW = 1.5               # 진행 방향 이 안에 무엇이 있으면 감속 (m)
CAM_FRESH = 0.7              # 이보다 오래된 장면 판단은 쓰지 않는다 (s)
PILLAR_PASS_MIN = 0.15       # 기둥을 비켜 갈 때 비상 정지 거리에서 이만큼은 더 떨어져 있어야 한다 (m). 더 가까우면 멈춰 돈다
TOF_HALF = math.radians(12.5)  # ToF 원뿔 반각: 깊이 배율을 맞출 영상 가운데 영역
AIRBORNE = ("WANDER", "TURN", "BACKOFF", "GAP")
GAP_JUMP = 0.5               # 문틀 ↔ 그 너머 거리 차이가 이보다 커야 틈으로 본다 (m)
GAP_WIDTH = (0.65, 1.6)      # 지나갈 틈의 폭 (m). 드론 폭 0.18 m + 양옆 0.23 m 이상
GAP_EDGE_DIFF = 0.5          # 양쪽 문틀 거리 차이가 이보다 크면 틈으로 보지 않는다 (m)
GAP_SIDE = 0.2               # 틈을 지나는 동안 쓰는 좁은 띠 반폭 (m): 드론 반폭 0.09 m + 0.11 m
GAP_CLEAR_TOF = 1.0          # 방향을 고정할 때 정면 ToF가 이보다 멀어야 한다 (틈 사이로 저 너머가 보여야)
GAP_RANGE = (0.7, 3.0)       # 틈 가운데까지의 거리 (m)
GAP_SPEED = 0.15             # 틈을 지날 때 속도 (m/s)
GAP_COMMIT = 0.6             # 틈 가운데가 이보다 가까우면 방향을 고정한다 (m)
GAP_THROUGH = 1.6            # 방향을 고정한 뒤 곧게 가는 거리 (m): 추정이 짧아도 문 너머 0.6 m 넘게
GAP_TIMEOUT = 25.0           # 장면으로 틈을 다시 못 본 채 이 시간이 지나면 그만둔다 (s)
GAP_STOP = 0.3               # 틈을 지날 때 정면 ToF 비상 정지 거리 (m). 틈 안에서는 문틀이 옆으로 가깝다
PERSON_SLOW, PERSON_SPEED = 1.5, 0.1   # YOLO 사람이 깊이로 이 거리 안·앞쪽 ±35°면 이 속도 이하로 (비키는 것은 카메라 격자·장면 판단이 한다)


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


class WanderNode(Node):
    def __init__(self) -> None:
        super().__init__("tt_mission")   # 웹 화면·기록이 비행 노드와 같은 이름으로 보게
        for name, value in (("map", ""), ("speed", 0.2), ("flight_height", 1.0), ("clear", 0.9), ("stop_distance", 0.4),
                            ("explore_time", 60.0), ("margin", 0.6), ("log_csv", ""), ("discovered_out", ""),
                            ("wait_start", True), ("pose_topic", "/tt/pose_est"), ("depth_topic", ""),
                            # 틈(문) 지나기. 기본은 끔: 실습실 Gazebo에서 가구 사이를 문으로 보고 들어가 0.07–0.17 m까지
                            # 붙었다(10/29). 문 시험 때만 켠다 (real.launch doors:=true, run_mission.sh --doors)
                            ("doors", False)):
            self.declare_parameter(name, value)
        g = lambda k: self.get_parameter(k).value  # noqa: E731
        data = json.loads(Path(g("map")).read_text(encoding="utf-8"))
        b = data["bounds"]
        self.fence = (b["xmin"] + float(g("margin")), b["ymin"] + float(g("margin")),
                      b["xmax"] - float(g("margin")), b["ymax"] - float(g("margin")))
        self.origin = (b["xmin"], b["ymin"])
        self.shape = (int(math.ceil((b["xmax"] - b["xmin"]) / CELL)), int(math.ceil((b["ymax"] - b["ymin"]) / CELL)))
        self.hits = np.zeros(self.shape, dtype=np.int16)      # ToF로 본 막힌 자리
        self.cam_hits = np.zeros(self.shape, dtype=np.int16)  # 카메라 깊이로 본 막힌 자리 (장면 수, 지나간 광선이 깎는다)
        self.cam_frames = 0
        self.depth_scale = 1.0
        self.scale_src = {"tof": 0, "floor": 0, "keep": 0}
        self.sectors = None          # (받은 시각, 그때 yaw, 부채꼴 가운데 각(기수 기준, 반시계 +), 거리)
        self.scene = None            # 같은 장면을 27열로 잘게: 앞이 기둥인지 벽인지 (autonomy/shape.py)
        self.cam_steers = 0
        self.gap_seen = None        # (본 시각, 가운데 (x, y), 지나갈 방향 단위벡터, 폭)
        self.gap = None             # 지나가는 중인 틈
        self.gaps_done: list[tuple[float, float]] = []
        self.gap_cooldown = 0.0
        self.visited = np.zeros(self.shape, dtype=np.int16)   # 지나온 자리
        self.speed, self.height = float(g("speed")), float(g("flight_height"))
        self.clear, self.stop_distance = float(g("clear")), float(g("stop_distance"))
        self.declare_parameter("fast", 0.0)   # 앞이 확실히 비면 이 속도까지 (m/s, autonomy/speed.py). 0이면 끔
        self.fast, self.fast_logged = float(g("fast")), 0.0
        self.explore_time = float(g("explore_time"))
        self.discovered_out = g("discovered_out")
        self.wait_start = bool(g("wait_start"))
        log_path = g("log_csv")
        self.log = csv.writer(open(log_path, "w", newline="", buffering=1)) if log_path else None
        if self.log:
            # 뒤 열은 판단 근거(비행 뒤 '왜 그렇게 움직였나'를 시각대로 본다): 정면 ToF, 카메라 진행 방향 거리,
            # 띠가 빈 거리, 가장 가까운 막힌 자리, 보낸 속도·회전, 가면서 트는 목표와 그 이유
            self.log.writerow(["t", "x", "y", "z", "yaw", "state", "sigma", "corr_x", "corr_y", "corr_yaw",
                               "front_tof", "cam_ahead", "lane_free", "near_hit", "cmd_vx", "cmd_wz", "soft_target", "soft_src"])
        self.decision: dict = {}
        self.last_cmd = (0.0, 0.0, 0.0, 0.0)

        self.pose = None
        self.pose_sigma = 0.0
        self.flying, self.flight_info, self.flight_req, self.forced_landing = False, {}, None, None
        self.tof_rows = {t: (math.inf, -math.inf) for t in TOF_TOPICS}
        self.front = math.inf
        self.state = "WAIT_POSE"
        self.start_requested = self.land_requested = False
        self.started_at = None
        self.turn_target = None
        self.soft_target = None     # 가면서 미리 트는 방향 (ToF에 무엇이 보일 때, 카메라가 앞이 가깝다고 볼 때)
        self.soft_src = None        # "tof" | "camera"
        self.turn_fails = 0
        self.backoff = None
        self.progress = None
        self.found: list[dict] = []
        self.distance = 0.0
        self.last_xy = None

        self.cmd_pub = self.create_publisher(Twist, "/tt/cmd_vel", 10)
        self.flight_pub = self.create_publisher(String, "/tt/flight_cmd", 10)
        self.state_pub = self.create_publisher(String, "/tt/mission_state", 10)
        self.create_subscription(Odometry, g("pose_topic"), self.on_pose, 10)
        self.create_subscription(String, "/tt/flight_status", self.on_flight_status, 10)
        self.create_subscription(String, "/tt/mission_cmd", self.on_mission_cmd, 10)
        self.create_subscription(String, "/tt/detections", self.on_detections, 10)   # YOLO (detect:=true일 때만 온다)
        self.person = None   # 가장 가까운 사람 (시각, 거리 m, 방위 rad)
        for topic in TOF_TOPICS:
            self.create_subscription(LaserScan, topic, lambda m, t=topic: self.on_tof(t, m), qos_profile_sensor_data)
        self.use_camera = bool(g("depth_topic"))
        self.doors = bool(g("doors"))
        if self.use_camera:
            self.create_subscription(Image, g("depth_topic"), self.on_depth, 2)
        self.create_timer(0.05, self.step)
        self.create_timer(1.0, lambda: self.state_pub.publish(String(data=self.state)))
        self.get_logger().info(f"지도 없이 둘러보기: 울타리 x {self.fence[0]:.1f}–{self.fence[2]:.1f}, "
                               f"y {self.fence[1]:.1f}–{self.fence[3]:.1f} m, 속도 {self.speed} m/s, 높이 {self.height} m, "
                               f"앞 {self.clear} m 안이면 방향 바꿈, {self.stop_distance} m 안이면 물러남, "
                               f"지나갈 띠 폭 {2 * SIDE:.1f} m·{self.look():.2f} m 앞까지, "
                               f"{self.explore_time:.0f}초 뒤 착륙, 카메라 깊이 {'씀 (' + g('depth_topic') + ')' if self.use_camera else '안 씀'}")

    # ------------------------------------------------------------ 입력
    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_pose(self, msg: Odometry) -> None:
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, p.z, yaw)
        var = max(msg.pose.covariance[0], msg.pose.covariance[7])
        self.pose_sigma = math.sqrt(var) if var > 0 else 0.0

    def on_flight_status(self, msg: String) -> None:
        info = json.loads(msg.data)
        flying = bool(info.get("flying"))
        if self.flying and not flying and self.state not in ("LAND", "DONE", "ABORT"):
            auto = info.get("last_auto_land") or {}
            self.forced_landing = auto.get("reason") or info.get("landing_reason") or "알 수 없음"
        self.flying, self.flight_info = flying, info

    def on_mission_cmd(self, msg: String) -> None:
        cmd = msg.data.strip()
        if cmd == "start" and self.state == "READY":
            self.start_requested = True
            self.get_logger().info("시작 요청을 받았습니다.")
        elif cmd == "land":
            self.land_requested = True
            self.get_logger().info(f"착륙 요청을 받았습니다 (지금 단계 {self.state}).")
        else:
            self.get_logger().warn(f"무시한 명령: {cmd} (지금 단계 {self.state})")

    def on_tof(self, topic: str, msg: LaserScan) -> None:
        valid = [r for r in msg.ranges if math.isfinite(r) and r > 0]
        now = self.now()
        self.tof_rows[topic] = (min(valid) if valid else math.inf, now)
        self.front = min((r for r, t in self.tof_rows.values() if now - t < 0.3), default=math.inf)
        if self.pose is not None and self.state in AIRBORNE and math.isfinite(self.front):
            x, y, _, yaw = self.pose
            self.mark_hit(x + (SENSOR_X + self.front) * math.cos(yaw), y + (SENSOR_X + self.front) * math.sin(yaw))

    def on_detections(self, msg: String) -> None:
        """YOLO 사람: 마스크 안 깊이로 잰 거리(depth_m × 바닥 배율)가 가까우면 느리게 간다.

        격자에 막힘으로 적지는 않는다: 격자의 ToF 자리는 지워지지 않아 사람이 움직이면 헛 장애물로 남는다. 사람 몸은
        깊이 영상에도 보이므로 카메라 격자(장면이 바뀌면 깎임)와 장면 판단이 비켜 간다.
        """
        data = json.loads(msg.data)
        w = float(data.get("width", 960))
        f = (w / 2) / math.tan(HFOV / 2)
        best = None
        for d in data["detections"]:
            if d.get("name") != "person" or d.get("depth_m") is None:
                continue
            dist = float(d["depth_m"]) * self.depth_scale
            bearing = math.atan((w / 2 - (d["x1"] + d["x2"]) / 2) / f)
            if best is None or dist < best[1]:
                best = (self.now(), dist, bearing)
        self.person = best

    def on_depth(self, msg: Image) -> None:
        """카메라 깊이 영상 → 비행 높이 띠 안의 점을 격자에 막힘으로, 그 앞까지 광선이 지나간 칸은 깎는다.

        이륙 끝 무렵(비행 높이 ±0.3 m 안)의 장면도 쓴다: 둘러보기를 시작하자마자 정면이 가까우면 장면 없이 돌아서 버렸다
        (기둥 0.95 m 앞에서 출발한 Gazebo). 높이가 아직 안 맞으면 어차피 아래에서 거른다.
        """
        if self.pose is None or self.state not in AIRBORNE + ("TAKEOFF",):
            return
        x, y, z, yaw = self.pose
        if abs(z - self.height) > 0.3:
            return
        depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        cam_z = z
        h, w = depth.shape
        f = (w / 2) / math.tan(HFOV / 2)
        # 배율: ToF가 앞 1.2 m 안을 재고 있으면 영상 가운데를 그 실측에 맞춘다. 아니면 바닥, 둘 다 없으면 직전 값
        tof, t_tof = self.tof_rows.get("/tt/tof_front", (math.inf, -math.inf))
        scale, src = None, "keep"
        if math.isfinite(tof) and tof < TOF_MAX - 0.05 and self.now() - t_tof < 0.3:
            r = int(f * math.tan(TOF_HALF))
            s = scale_to_anchor(depth, tof + SENSOR_X, (slice(h // 2 - r, h // 2 + r), slice(w // 2 - r, w // 2 + r)))
            if 0.6 <= s <= 1.6:
                scale, src = s, "tof"
        if scale is None:
            s = floor_scale(depth, HFOV, cam_z)
            if s is not None:
                scale, src = s, "floor"
        if scale is not None:
            self.depth_scale = 0.5 * self.depth_scale + 0.5 * scale if self.cam_frames else scale
        self.scale_src[src] += 1
        vs, us = np.mgrid[CAM_STRIDE // 2:h:CAM_STRIDE, CAM_STRIDE // 2:w:CAM_STRIDE]
        d = depth[vs, us].astype(np.float64) * self.depth_scale
        left, up = -(us - w / 2) / f, -(vs - h / 2) / f
        height = cam_z + d * up
        band = np.isfinite(d) & (np.abs(height - self.height) <= CAM_BAND) & (height > 0.3)
        # 부채꼴: 비행 높이 띠 안 화소의 수평 거리, 칸마다 가까운 쪽(10번째 백분위). 위치와 상관없는 장면 판단에 쓴다
        ang = np.arctan(left[band])
        rng = np.minimum(d[band] * np.sqrt(1 + left[band] ** 2), 4.0)
        width = HFOV / SECTORS
        idx = np.clip(((ang + HFOV / 2) / width).astype(int), 0, SECTORS - 1)
        dists = np.full(SECTORS, 4.0)
        for k in range(SECTORS):
            v = rng[idx == k]
            if v.size >= 3:
                dists[k] = float(np.percentile(v, 10))
        centers = -HFOV / 2 + width * (np.arange(SECTORS) + 0.5)
        self.sectors = (self.now(), yaw, centers, dists)
        self.scene = (self.now(), yaw, *scene_ranges(depth * self.depth_scale, HFOV, cam_z, self.height))
        self.find_gap(x, y, yaw, us, left, d, band)
        c, s = math.cos(yaw), math.sin(yaw)
        cx = x + SENSOR_X * c
        cy = y + SENSOR_X * s
        hit_cells, pass_cells = set(), set()
        for dd, lf in zip(d[band], left[band]):
            dx, dy = c - s * lf, s + c * lf          # 수평 방향 (광축 1 기준)
            norm = math.hypot(dx, dy)
            reach = min(dd, CAM_RANGE[1]) * norm
            ux, uy = dx / norm, dy / norm
            t = CELL / 2
            while t < reach - CELL:
                cc = self.cell(cx + t * ux, cy + t * uy)
                if cc is not None:
                    pass_cells.add(cc)
                t += CELL / 2
            if CAM_RANGE[0] <= dd <= CAM_RANGE[1]:
                cc = self.cell(cx + dd * dx, cy + dd * dy)
                if cc is not None:
                    hit_cells.add(cc)
        for cc in pass_cells - hit_cells:
            if self.cam_hits[cc] > 0:
                self.cam_hits[cc] -= 1
        for cc in hit_cells:
            self.cam_hits[cc] = min(self.cam_hits[cc] + 1, 10)
            if self.cam_hits[cc] == CAM_HITS and self.hits[cc] < 2:
                ox, oy = self.origin[0] + (cc[0] + 0.5) * CELL, self.origin[1] + (cc[1] + 0.5) * CELL
                self.found.append({"x": round(ox, 2), "y": round(oy, 2), "z": self.height, "sx": CELL, "sy": CELL,
                                   "sz": 2.0, "sources": ["camera"], "hits": CAM_HITS})
                self.save_found()
        self.cam_frames += 1

    def find_gap(self, x: float, y: float, yaw: float, us, left, d, band) -> None:
        """장면에서 '가까움-멂-가까움' 모양(문틀 사이로 저 너머가 보임)을 찾아 self.gap_seen에 둔다.

        열(약 2°)마다 비행 높이 띠의 가까운 쪽 거리를 내고, 거리가 GAP_JUMP 넘게 뛰는 곳을 양쪽 문틀로 본다.
        두 문틀 점 사이 폭이 GAP_WIDTH 안이고 가운데가 GAP_RANGE 안이면 후보다. 화면 끝에 걸린 문틀은 쓰지 않는다
        (한쪽이 화면 밖이면 폭을 모른다). 단안 깊이는 문틀과 그 너머 사이 화소에 중간 거리를 내므로 두 열 창으로 본다.
        """
        cols = np.unique(us[0])
        r = np.full(len(cols), np.nan)
        a = np.arctan(left[0])
        for i, u in enumerate(cols):
            v = d[:, i][band[:, i]] * math.sqrt(1 + left[0, i] ** 2)
            if v.size >= 2:
                r[i] = float(np.percentile(np.minimum(v, 6.0), 10))
        best = None
        n = len(r)
        for p in range(1, n - 3):
            if not np.isfinite(r[p]) or not (np.nanmax(r[p + 1:p + 3]) - r[p] > GAP_JUMP):
                continue
            for q in range(p + 1, n - 2):
                if not np.isfinite(r[q + 1]) or not (np.nanmax(r[max(p + 1, q - 1):q + 1]) - r[q + 1] > GAP_JUMP):
                    continue
                inner = r[p + 1:q + 1]
                if not np.isfinite(inner).any() or np.nanmin(inner) < max(r[p], r[q + 1]) + 0.3:
                    break
                # 문틀 두 쪽은 거의 같은 거리에 있다. 많이 다르면 가구 모서리 둘이 겹쳐 보인 것이 많고 폭도 크게 틀렸다
                # (Gazebo 실습실: 책상과 상자 사이를 문으로 보고 들어가 스쳤다)
                if abs(r[p] - r[q + 1]) > GAP_EDGE_DIFF:
                    break
                p1 = np.array([r[p] * math.cos(a[p]), r[p] * math.sin(a[p])])
                p2 = np.array([r[q + 1] * math.cos(a[q + 1]), r[q + 1] * math.sin(a[q + 1])])
                wdt = float(np.linalg.norm(p1 - p2))
                mid = (p1 + p2) / 2
                dist = float(np.linalg.norm(mid))
                if GAP_WIDTH[0] <= wdt <= GAP_WIDTH[1] and GAP_RANGE[0] <= dist <= GAP_RANGE[1]:
                    bearing = math.atan2(mid[1], mid[0])
                    if best is None or abs(bearing) < abs(best[0]):
                        best = (bearing, mid, p1, p2, wdt)
                break
        if best is None:
            return
        _, mid, p1, p2, wdt = best
        c, s = math.cos(yaw), math.sin(yaw)
        to_w = lambda v: np.array([x + SENSOR_X * c + c * v[0] - s * v[1], y + SENSOR_X * s + s * v[0] + c * v[1]])  # noqa: E731
        # 지나갈 방향은 드론에서 틈 가운데로 가는 방향이다. 두 문틀 점을 이은 선의 수직은 쓰지 않는다: 단안 깊이는
        # 두 문틀 거리를 따로 틀려(0.2–0.4 m) 선이 비스듬해지고, 그 수직을 따라가다 문틀에 붙었다 (2026-10-29 Gazebo)
        cw = to_w(mid)
        to_c = cw - np.array([x, y])
        self.gap_seen = (self.now(), cw, to_c / max(np.linalg.norm(to_c), 1e-6), wdt)

    def gap_wanted(self, x: float, y: float) -> bool:
        """방금 본 틈으로 들어갈까: 새것이고, 예전에 지나간 틈이 아니고, 너머가 아직 안 가 본 곳일 때."""
        if self.gap_seen is None or self.now() - self.gap_seen[0] > CAM_FRESH or self.now() < self.gap_cooldown:
            return False
        _, cw, normal, _ = self.gap_seen
        if any(math.hypot(cw[0] - px, cw[1] - py) < 1.0 for px, py in self.gaps_done):
            return False
        far = self.cell(*(cw + normal * 1.0))
        return far is None or self.visited[far] == 0

    def abort_gap(self, x: float, y: float, why: str) -> None:
        cw = self.gap["center"]
        self.get_logger().warn(f"틈 지나기를 그만둡니다 ({why}). 이 틈은 다시 고르지 않습니다.")
        self.gaps_done.append((float(cw[0]), float(cw[1])))
        self.gap, self.gap_cooldown = None, self.now() + 5.0
        self.backoff = (x, y, self.now())
        self.set_state("BACKOFF")

    def start_gap(self, x: float, y: float, yaw: float) -> None:
        _, cw, normal, wdt = self.gap_seen
        self.gap = {"center": cw, "normal": normal, "width": wdt, "t0": self.now()}
        self.soft_target = None
        self.get_logger().info(f"카메라: 폭 {wdt:.2f} m 틈(문)을 봤습니다. 가운데 ({cw[0]:.2f}, {cw[1]:.2f})로 맞춰 지나갑니다.")
        self.set_state("GAP")

    # ------------------------------------------------------------ 격자
    def cell(self, x: float, y: float):
        i, j = int((x - self.origin[0]) / CELL), int((y - self.origin[1]) / CELL)
        return (i, j) if 0 <= i < self.shape[0] and 0 <= j < self.shape[1] else None

    def mark_hit(self, x: float, y: float) -> None:
        c = self.cell(x, y)
        if c is None:
            return
        self.hits[c] += 1
        if self.hits[c] == 2:   # 두 번 본 자리만 남긴다 (한 번 튄 값은 무시)
            cx, cy = self.origin[0] + (c[0] + 0.5) * CELL, self.origin[1] + (c[1] + 0.5) * CELL
            self.found.append({"x": round(cx, 2), "y": round(cy, 2), "z": self.height, "sx": CELL, "sy": CELL,
                               "sz": 2.0, "sources": ["tof"], "hits": 2})
            self.save_found()

    def blocked(self, x: float, y: float) -> bool:
        """울타리 밖이거나 ToF로 두 번 넘게 본 자리."""
        if not (self.fence[0] <= x <= self.fence[2] and self.fence[1] <= y <= self.fence[3]):
            return True
        return self.hit_at(x, y)

    def hit_at(self, x: float, y: float, camera: bool = True) -> bool:
        """ToF로 두 번 넘게 봤거나 (camera면) 카메라 깊이로 CAM_HITS 장면 넘게 본 자리."""
        c = self.cell(x, y)
        return c is not None and (self.hits[c] >= 2 or (camera and self.cam_hits[c] >= CAM_HITS))

    def lane_free(self, x: float, y: float, h: float, length: float, start: float = 0.0, side: float = SIDE) -> float:
        """h 방향으로 폭 2×side의 띠를 내다봐, 본 자리에 걸리기 전까지 갈 수 있는 거리. 울타리는 가운데 줄만 본다."""
        c, s = math.cos(h), math.sin(h)
        offsets = (-side, -side / 2, 0.0, side / 2, side)
        free, d = start, start
        while d <= length:
            px, py = x + d * c, y + d * s
            if not (self.fence[0] <= px <= self.fence[2] and self.fence[1] <= py <= self.fence[3]):
                return free
            # 기체 바로 옆(0.3 m 안)은 가운데 줄만 본다: 벽 옆에 떠 있을 때 모든 방향이 막혀 내려 버리지 않게.
            # 카메라 칸도 띠 가장자리까지 센다. 가장자리에서 빼 봤더니 ToF가 못 본 책상 모서리를 옆으로 스쳤다
            # (2026-10-29 Gazebo 0.3 m/s 0.03 m, 0.4 m/s 닿음). 옆을 보는 것은 카메라뿐이다
            if any(self.hit_at(px - o * s, py + o * c) for o in (offsets if d >= 0.3 else (0.0,))):
                return free
            free = d
            d += CELL / 2
        return free

    def cam_clear(self) -> float:
        """카메라가 진행 방향에서 이보다 가까운 것을 보면 튼다: 빠를수록 일찍 (0.2 m/s 0.8 m, 0.3 m/s 0.9 m)."""
        return 0.6 + self.speed

    def fence_ahead(self, x: float, y: float, yaw: float, length: float) -> float:
        """기수 방향으로 울타리(방 테두리)까지 거리 (length까지만)."""
        c, s = math.cos(yaw), math.sin(yaw)
        d = 0.0
        while d < length:
            px, py = x + d * c, y + d * s
            if not (self.fence[0] <= px <= self.fence[2] and self.fence[1] <= py <= self.fence[3]):
                return d
            d += 0.25
        return length

    def front_shape(self, yaw: float):
        """최근 깊이 장면으로 정면 ToF 거리에 있는 것의 모양(기둥/벽). 카메라를 안 쓰거나 장면이 오래됐으면 None."""
        if self.scene is None or self.now() - self.scene[0] > CAM_FRESH or not math.isfinite(self.front):
            return None
        _, scene_yaw, centers, ranges = self.scene
        return front_shape(rotate_scene(centers, scene_yaw, yaw), ranges, self.front)

    def camera_view(self, yaw: float):
        """최근 장면의 부채꼴 (지금 기수 기준 각, 거리). 오래됐거나 카메라를 안 쓰면 None."""
        if self.sectors is None or self.now() - self.sectors[0] > CAM_FRESH:
            return None
        t, yaw_cap, centers, dists = self.sectors
        rel = np.array([wrap(c + yaw_cap - yaw) for c in centers])
        return rel, dists

    def camera_heading(self, x: float, y: float, yaw: float, rel, dists, look: float) -> float | None:
        """보이는 부채꼴 중 갈 방향: 충분히 비어 있고(cam_clear + 0.3) 격자 띠도 비어 있는 것 중 넓고 덜 도는 쪽."""
        best, best_score = None, -math.inf
        for a, dist in zip(rel, dists):
            if abs(a) <= CAM_AHEAD / 2 or dist < self.cam_clear() + 0.3:
                continue
            h = wrap(yaw + a)
            if self.lane_free(x, y, h, look) < look - CELL / 2:
                continue
            score = dist - 0.6 * abs(a)
            if score > best_score:
                best, best_score = h, score
        return best

    def look(self) -> float:
        """직진할 때 띠를 내다보는 거리: 빠를수록 멀리 (0.2 m/s 0.7 m, 0.3 m/s 0.85 m, 0.4 m/s 1.0 m)."""
        return max(AHEAD, LOOK_BASE + LOOK_GAIN * self.speed)

    def nearest_hit(self, x: float, y: float, radius: float) -> float:
        """반경 안에서 가장 가까운 본 자리(칸 중심)까지 거리. 없으면 inf."""
        i0, j0 = int((x - self.origin[0]) / CELL), int((y - self.origin[1]) / CELL)
        r = int(math.ceil(radius / CELL)) + 1
        best = math.inf
        for i in range(max(0, i0 - r), min(self.shape[0], i0 + r + 1)):
            for j in range(max(0, j0 - r), min(self.shape[1], j0 + r + 1)):
                if self.hits[i, j] >= 2:
                    cx, cy = self.origin[0] + (i + 0.5) * CELL, self.origin[1] + (j + 0.5) * CELL
                    best = min(best, math.hypot(cx - x, cy - y) - CELL / 2)
        return best if best <= radius else math.inf

    def choose_heading(self, x: float, y: float, yaw: float, max_turn: float | None = None) -> float | None:
        """30°마다 후보를 내다보고 고른다: 띠가 막히기 전까지의 거리 + 안 가 본 칸 수, 크게 도는 쪽은 조금 덜 좋게.

        지금 방향(k=0)은 뺀다: 이 함수는 지금 방향이 막혔을 때 불리므로, 같은 방향을 다시 고르면 제자리에서 맴돈다
        (Gazebo 첫 시험에서 "+0° 돌아"가 일곱 번 이어졌다). 격자는 반 칸씩 내다봐 대각선에서 막힌 칸을 건너뛰지 않는다.
        거리는 가운데 줄이 아니라 폭 2×SIDE의 띠로 잰다(옆으로 가구 모서리를 스치지 않게).
        max_turn을 주면 가면서 틀 방향을 찾는다: ±15°마다 그 안에서만 고른다.
        """
        best, best_score = None, -math.inf
        if max_turn is None:
            turns = [k * math.pi / 6 for k in range(1, 12)]
        else:
            n = int(max_turn / math.radians(15))
            turns = [s * k * math.radians(15) for k in range(1, n + 1) for s in (1, -1)]
        for turn in turns:
            h = wrap(yaw + turn)
            free, fresh = self.lane_free(x, y, h, RAY, start=CELL / 2), 0
            seen = set()
            d = CELL / 2
            while d <= free:
                px, py = x + d * math.cos(h), y + d * math.sin(h)
                c = self.cell(px, py)
                if c is not None and c not in seen:
                    seen.add(c)
                    if self.visited[c] == 0:
                        fresh += 1
                d += CELL / 2
            if free < max(AHEAD + CELL, self.look() + CELL / 2):   # 돌자마자 직진 검사에 다시 걸리지 않게
                continue
            score = free + 0.5 * fresh * CELL - 0.4 * abs(wrap(h - yaw)) / math.pi
            if score > best_score:
                best, best_score = h, score
        return best

    def save_found(self) -> None:
        if self.discovered_out:
            Path(self.discovered_out).write_text(json.dumps({"obstacles": self.found, "replans": 0}, ensure_ascii=False),
                                                 encoding="utf-8")

    # ------------------------------------------------------------ 동작
    def send(self, vx=0.0, vy=0.0, vz=0.0, yaw_rate=0.0) -> None:
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z = float(vx), float(vy), float(vz), float(yaw_rate)
        self.cmd_pub.publish(msg)
        self.last_cmd = (float(vx), float(vy), float(vz), float(yaw_rate))

    def set_state(self, state: str) -> None:
        if state != self.state:
            self.get_logger().info(f"상태: {state}")
            self.state = state
            self.state_pub.publish(String(data=state))

    def request_flight(self, cmd: str, retry: float) -> int:
        req = self.flight_req
        if req is None or req[0] != cmd or self.now() - req[1] > retry:
            n = 1 if req is None or req[0] != cmd else req[2] + 1
            self.flight_req = (cmd, self.now(), n)
            self.flight_pub.publish(String(data=cmd))
            self.get_logger().info(f"드라이버에 {cmd} 요청 ({n}번째)")
        return self.flight_req[2]

    def hold_height(self, z: float) -> float:
        return max(-VZ_MAX, min(VZ_MAX, K_POS * (self.height - z)))

    def start_turn(self, x: float, y: float, yaw: float, why: str) -> None:
        h = self.choose_heading(x, y, yaw)
        if h is None:
            self.turn_fails += 1
            self.get_logger().warn(f"{why}: 갈 만한 방향이 없습니다 ({self.turn_fails}번째). 뒤로 돌아 봅니다.")
            h = wrap(yaw + math.pi)
            if self.turn_fails >= MAX_TURN_FAILS:
                self.get_logger().error("계속 막혀 그 자리에 착륙합니다.")
                self.start_landing()
                return
        else:
            self.turn_fails = 0
            self.get_logger().info(f"{why}: {math.degrees(wrap(h - yaw)):+.0f}° 돌아 {math.degrees(h):.0f}° 쪽으로 갑니다.")
        self.turn_target, self.soft_target = h, None
        self.set_state("TURN")

    def start_landing(self) -> None:
        self.send()
        self.set_state("LAND")

    def stuck(self, pushing: bool) -> bool:
        x, y, _, yaw = self.pose
        now = self.now()
        if (not pushing or self.progress is None or math.hypot(x - self.progress[0], y - self.progress[1]) > 0.05
                or abs(wrap(yaw - self.progress[3])) > 0.1):
            self.progress = (x, y, now, yaw)
            return False
        return now - self.progress[2] > STUCK_TIME

    def step(self) -> None:
        if self.pose is None:
            return
        x, y, z, yaw = self.pose
        if self.log:
            dc = self.decision
            f = lambda v: "" if v is None or (isinstance(v, float) and not math.isfinite(v)) else f"{v:.3f}"  # noqa: E731
            self.log.writerow([f"{self.now():.3f}", f"{x:.3f}", f"{y:.3f}", f"{z:.3f}", f"{yaw:.3f}", self.state,
                               f"{self.pose_sigma:.3f}", "0.000", "0.000", "0.0000",
                               f(self.front), f(dc.get("cam_ahead")), f(dc.get("lane")), f(dc.get("near")),
                               f(self.last_cmd[0]), f(self.last_cmd[3]), f(dc.get("soft")), dc.get("src") or ""])
        if self.state in AIRBORNE:
            c = self.cell(x, y)
            if c is not None:
                self.visited[c] += 1
            if self.last_xy:
                self.distance += math.hypot(x - self.last_xy[0], y - self.last_xy[1])
            self.last_xy = (x, y)

        if self.forced_landing and self.state not in ("DONE", "ABORT"):
            self.get_logger().error(f"드라이버가 기체를 내렸습니다 ({describe_landing(self.forced_landing)}). 중단합니다.")
            self.forced_landing = None
            self.set_state("ABORT")
            return
        if self.land_requested:
            self.land_requested = False
            if self.state in ("WAIT_POSE", "READY"):
                self.set_state("ABORT")
                return
            if self.state == "TAKEOFF" and not self.flying:
                self.set_state("ABORT")
                return
            if self.state not in ("LAND", "DONE", "ABORT"):
                self.get_logger().info("요청에 따라 그 자리에 착륙합니다.")
                self.start_landing()
                return

        if self.state == "WAIT_POSE":
            self.set_state("READY" if self.wait_start else "TAKEOFF")

        elif self.state == "READY":
            self.send()
            if self.start_requested:
                self.start_requested = False
                self.set_state("TAKEOFF")

        elif self.state == "TAKEOFF":
            if not self.flying:
                self.send()
                tries = self.flight_req[2] if self.flight_req and self.flight_req[0] == "takeoff" else 0
                if tries >= TAKEOFF_TRIES and self.now() - self.flight_req[1] > TAKEOFF_RETRY:
                    failed = (self.flight_info.get("last_result") or {}).get("failed")
                    self.get_logger().error(f"이륙 요청 {tries}번에도 뜨지 않았습니다 (점검 실패: {failed}). 중단합니다.")
                    self.set_state("ABORT")
                else:
                    self.request_flight("takeoff", TAKEOFF_RETRY)
                return
            dz = self.height - z
            vz = K_POS * dz
            if abs(dz) >= TAKEOFF_TOL:
                vz = math.copysign(max(abs(vz), CLIMB_MIN), dz)
            self.send(0.0, 0.0, max(-VZ_MAX, min(VZ_MAX, vz)), 0.0)
            if abs(dz) < TAKEOFF_TOL:
                self.started_at = self.now()
                self.progress = None
                self.set_state("WANDER")

        elif self.state == "WANDER":
            if self.now() - self.started_at >= self.explore_time:
                self.get_logger().info(f"둘러보기 시간이 다 돼 그 자리에 내립니다. 움직인 거리 {self.distance:.1f} m, "
                                       f"지나온 칸 {int((self.visited > 0).sum())}개, 본 막힌 자리 {len(self.found)}개"
                                       + (f", 카메라 장면 {self.cam_frames}장(배율 ToF {self.scale_src['tof']}·바닥 "
                                          f"{self.scale_src['floor']}·유지 {self.scale_src['keep']}), 카메라로 튼 횟수 "
                                          f"{self.cam_steers}" if self.use_camera else ""))
                self.start_landing()
                return
            vz = self.hold_height(z)
            if self.front < self.stop_distance:
                self.send(vz=vz)
                self.get_logger().warn(f"전방 {self.front:.2f} m. 멈추고 조금 물러납니다.")
                self.backoff = (x, y, self.now())
                self.set_state("BACKOFF")
                return
            if self.stuck(True):
                self.get_logger().warn(f"앞으로 가라는데 {STUCK_TIME:.1f}초째 제자리입니다. 바로 앞을 막힌 자리로 봅니다.")
                for _ in range(2):   # 두 번 본 것으로 적어 바로 막힌 자리가 되게
                    self.mark_hit(x + (SENSOR_X + 0.1) * math.cos(yaw), y + (SENSOR_X + 0.1) * math.sin(yaw))
                self.backoff = (x, y, self.now())
                self.set_state("BACKOFF")
                return
            # 카메라가 아직 안 가 본 쪽으로 난 틈(문)을 보면 그쪽으로 맞춰 지나간다
            if self.use_camera and self.doors and self.gap_wanted(x, y):
                self.start_gap(x, y, yaw)
                return
            ax, ay = x + AHEAD * math.cos(yaw), y + AHEAD * math.sin(yaw)
            if self.front < self.clear:
                # 기둥(가만히 선 사람 등, 좁고 한쪽이 빔)은 멈춰 돌지 않고 빈 쪽으로 가면서 비켜 간다. 벽은 돌아선다 (10/30 오후)
                shape = self.front_shape(yaw)
                if (shape is not None and shape.kind == "pillar" and shape.free_side
                        and self.front >= self.stop_distance + PILLAR_PASS_MIN):
                    if self.soft_src != "pillar" or self.soft_target is None:
                        side = "왼쪽" if shape.free_side == "left" else "오른쪽"
                        self.soft_target, self.soft_src = wrap(yaw + shape.pass_angle()), "pillar"
                        self.get_logger().info(f"앞 {self.front:.2f} m는 {shape.text()}: {side}으로 비켜 지나갑니다.")
                else:
                    why = f"전방 {self.front:.2f} m" + (f" {shape.text()}" if shape is not None and shape.kind != "clear" else "")
                    self.start_turn(x, y, yaw, why)
                    return
            if not (self.fence[0] <= ax <= self.fence[2] and self.fence[1] <= ay <= self.fence[3]):
                self.start_turn(x, y, yaw, f"{AHEAD} m 앞이 울타리")
                return
            look = self.look()
            # 카메라 장면 판단: 진행 방향이 가까우면 보이는 쪽 중 가장 넓게 빈 부채꼴로 가면서 튼다
            cam_k, cam = 1.0, self.camera_view(yaw)
            if cam is not None:
                rel, dists = cam
                ahead = float(dists[np.abs(rel) <= CAM_AHEAD].min(initial=4.0))
                cam_k = max(0.4, min(1.0, (ahead - 0.5) / (CAM_SLOW - 0.5)))
                if ahead < self.cam_clear() and self.soft_target is None:
                    h = self.camera_heading(x, y, yaw, rel, dists, look)
                    if h is None:
                        self.start_turn(x, y, yaw, f"카메라: 앞 {ahead:.2f} m, 보이는 쪽이 다 막힘")
                        return
                    self.soft_target, self.soft_src = h, "camera"
                    self.cam_steers += 1
                    self.get_logger().info(f"카메라: 앞 {ahead:.2f} m → 빈 쪽 {math.degrees(wrap(h - yaw)):+.0f}°로 가면서 틉니다.")
            # 지나갈 띠(폭 2×SIDE)를 속도에 맞춰 내다본다. 앞 ToF가 비어도 예전에 본 가구 모서리가 옆에 걸리면 돌린다.
            # 카메라로 트는 중이면 갈 방향의 띠를 본다
            lane_h = self.soft_target if self.soft_target is not None else yaw
            lane = self.lane_free(x, y, lane_h, look)
            if lane < look - CELL / 2:
                # 아직 0.5 m 넘게 남았으면 멈춰 돌지 않고 가면서 튼다: 보이는 빈 부채꼴, 없으면 격자로 ±60° 안에서
                h = None
                if lane >= 0.5 and self.soft_target is None:
                    if cam is not None:
                        h = self.camera_heading(x, y, yaw, *cam, look)
                    if h is None:
                        h = self.choose_heading(x, y, yaw, max_turn=math.pi / 3)
                if h is None:
                    self.start_turn(x, y, yaw, f"{look:.1f} m 앞길 옆에 본 자리")
                    return
                self.soft_target, self.soft_src = h, "lane"
            # ToF에 무엇이 보이기 시작하면(1.2 m 안) 멈추지 않고 속도를 줄여 가면서 미리 방향을 튼다
            if self.front < TOF_MAX - 0.05:
                if self.soft_target is None:
                    self.soft_target, self.soft_src = self.choose_heading(x, y, yaw), "tof"
            elif self.soft_src == "tof":
                self.soft_target = None
            # 본 자리가 가까우면 속도를 줄인다 (0.3 m에서 0.4배, 0.8 m 넘으면 그대로). 카메라 앞 거리로도 줄인다
            near = self.nearest_hit(x, y, NEAR_SLOW[1])
            k = 1.0 if not math.isfinite(near) else max(0.4, min(1.0, (near - NEAR_SLOW[0]) / (NEAR_SLOW[1] - NEAR_SLOW[0])))
            k = min(k, cam_k)
            v, rate = k * self.speed, 0.0
            if self.soft_target is not None:
                err = wrap(self.soft_target - yaw)
                if abs(err) < 0.15:
                    self.soft_target = None
                else:
                    v, rate = SOFT_SPEED * k * self.speed, max(-YAW_RATE, min(YAW_RATE, 1.5 * err))
            # 앞이 확실히 비면(카메라 정면 ±15°·띠·울타리가 다 멀고 ToF에 아무것도 없음) speed 대신 fast까지 (10/30 저녁 요청)
            if (self.fast > self.speed and rate == 0.0 and self.soft_target is None and cam is not None
                    and not math.isfinite(self.front)):
                rel, dists = cam
                ahead_cam = float(dists[np.abs(rel) <= FAST_FOV].min(initial=4.0))
                clear = min(ahead_cam, self.lane_free(x, y, yaw, FAST_FULL), self.fence_ahead(x, y, yaw, FAST_FULL))
                kf = fast_factor(clear)
                if kf > 0:
                    v = boosted(v, k * self.speed, k * self.fast, kf)
                    if self.now() - self.fast_logged > 5.0:
                        self.fast_logged = self.now()
                        self.get_logger().info(f"앞 {clear:.1f} m가 비어 {v:.2f} m/s로 갑니다")
            # 정면 ToF 거리 감속 (mission_node와 같은 규칙): 비상 정지 거리에 닿기 전에 충분히 느려지게
            if math.isfinite(self.front) and self.front < TOF_MAX:
                v = min(v, max(0.05, 0.5 * (self.front - (self.stop_distance + 0.1))))   # 비상 정지 0.1 m 앞을 겨눈다
            if self.person and self.now() - self.person[0] < 1.0 and self.person[1] < PERSON_SLOW                     and abs(self.person[2]) < math.radians(35):
                v = min(v, PERSON_SPEED)   # 앞에 사람: 천천히
            self.decision = {"front": self.front, "cam_ahead": ahead if cam is not None else math.nan, "lane": lane,
                             "near": near, "v": v, "rate": rate, "soft": self.soft_target, "src": self.soft_src}
            self.send(v, 0.0, vz, rate)

        elif self.state == "GAP":
            # 두 단계: 다가가기(장면마다 고친 틈 가운데를 겨눈다) → 지나가기(그 방향 그대로 GAP_THROUGH m 곧게).
            # 단안 깊이는 문까지 거리를 0.2–0.4 m 짧게 잰다. 그래서 '가운데를 지났다'를 깊이로 판단하지 않고,
            # 가운데 GAP_COMMIT m 앞에서 방향을 고정한 뒤 문 너머까지 충분히 곧게 간다. 정면 ToF가 GAP_STOP 안이면 멈춘다
            vz = self.hold_height(z)
            g = self.gap
            cw = g["center"]
            dist = math.hypot(cw[0] - x, cw[1] - y)
            if g.get("through") is None:
                if self.gap_seen and self.gap_seen[0] > g["t0"] and np.linalg.norm(self.gap_seen[1] - cw) < 0.5:
                    g["center"] = cw = 0.5 * cw + 0.5 * self.gap_seen[1]
                    g["t0"] = self.gap_seen[0]
                    dist = math.hypot(cw[0] - x, cw[1] - y)
                heading = math.atan2(cw[1] - y, cw[0] - x)
                if dist < GAP_COMMIT and abs(wrap(heading - yaw)) < math.radians(15):
                    if self.front < GAP_CLEAR_TOF:   # 틈 사이로 저 너머가 안 보인다: 틈이 아니거나 겨냥이 틀렸다
                        self.abort_gap(x, y, f"방향을 고정하려는데 정면 ToF {self.front:.2f} m")
                        return
                    g["through"] = (x, y, heading)
                    self.get_logger().info(f"틈 {dist:.2f} m 앞: 방향 {math.degrees(heading):.0f}°로 고정하고 지나갑니다.")
                gone = 0.0
            else:
                tx, ty, heading = g["through"]
                gone = math.hypot(x - tx, y - ty)
            # 좁은 띠(반폭 GAP_SIDE)로 바로 앞 0.5 m에 본 자리가 있으면 그만둔다 (틈이 생각보다 좁거나 옆 가구에 붙음)
            ahead = min(0.5, max(0.0, GAP_THROUGH - gone))
            if ahead > 0.2 and self.lane_free(x, y, heading, ahead, side=GAP_SIDE) < ahead - CELL / 2:
                self.abort_gap(x, y, "지나갈 좁은 띠에 본 자리")
                return
                if gone > GAP_THROUGH:
                    self.gaps_done.append((float(cw[0]), float(cw[1])))
                    self.gap, self.gap_cooldown = None, self.now() + 5.0
                    self.get_logger().info(f"틈을 지났습니다 ({gone:.2f} m 곧게). 둘러보기를 이어 갑니다.")
                    self.progress = None
                    self.set_state("WANDER")
                    return
            if self.now() - g["t0"] > GAP_TIMEOUT or self.front < GAP_STOP:
                self.abort_gap(x, y, f"정면 {self.front:.2f} m" if self.front < GAP_STOP else f"{GAP_TIMEOUT:.0f}초 안에 못 지남")
                return
            err = wrap(heading - yaw)
            v = GAP_SPEED if abs(err) < math.radians(15) else 0.0
            if self.now() - g.get("logged", 0.0) > 1.0:
                g["logged"] = self.now()
                self.get_logger().info(f"틈: 가운데 ({cw[0]:.2f}, {cw[1]:.2f}) 폭 {g['width']:.2f}, 남은 거리 {dist:.2f} m, "
                                       f"{'지나는 중' if g.get('through') else '다가가는 중'}, 방향 오차 {math.degrees(err):+.0f}°, "
                                       f"정면 ToF {self.front:.2f} m")
            self.send(v, 0.0, vz, max(-YAW_RATE, min(YAW_RATE, 1.5 * err)))

        elif self.state == "TURN":
            vz = self.hold_height(z)
            err = wrap(self.turn_target - yaw)
            if abs(err) < 0.15:
                if self.front < self.clear:   # 돌고 나니 이쪽도 막혀 있다
                    self.start_turn(x, y, yaw, f"돌고 보니 전방 {self.front:.2f} m")
                    return
                self.progress, self.soft_target = None, None
                self.set_state("WANDER")
                self.send(vz=vz)
                return
            self.send(0.0, 0.0, vz, max(-YAW_RATE, min(YAW_RATE, 1.5 * err)))

        elif self.state == "BACKOFF":
            bx, by, bt = self.backoff
            if math.hypot(x - bx, y - by) >= BACKOFF_DIST or self.now() - bt > BACKOFF_TIME:
                self.start_turn(x, y, yaw, "물러난 뒤")
                return
            self.send(-(BRAKE_SPEED if self.now() - bt < BRAKE_TIME else 0.15), 0.0, self.hold_height(z), 0.0)

        elif self.state == "LAND":
            self.send()
            if self.flying:
                self.request_flight("land", LAND_RETRY)
                return
            self.get_logger().info(f"착륙 완료. 움직인 거리 {self.distance:.1f} m, 지나온 칸 {int((self.visited > 0).sum())}개, "
                                   f"본 막힌 자리 {len(self.found)}개")
            self.set_state("DONE")

        elif self.state == "ABORT":
            self.send()
            if self.flying:
                self.request_flight("land", LAND_RETRY)
                return
            self.set_state("DONE")

    def close(self) -> None:
        try:
            self.send()
        except Exception:   # Ctrl+C로 ROS가 먼저 내려가면 보낼 수 없다 (드라이버는 입력이 끊기면 제자리에 선다)
            pass
        self.save_found()


def main() -> None:
    rclpy.init()
    node = WanderNode()
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
