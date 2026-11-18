"""가짜 자율 비행 보기 노드(시험용): ROS 없이 보기 노드(ros_view_node.py)와 같은 HTTP를 낸다.

진짜 노드와 같은 ros_view_core(값 보관·JPEG·HTTP·명령)를 쓰고, 자율 드라이버 토픽 대신 지어낸 값을 넣는다.
가짜 자율 비행 프로그램(FakeFlight)이 관제 화면 명령을 보기 노드가 내는 토픽·문자열 그대로 받아 따라간다:
  주변 확인(confirm) → 시작(start) → 이륙 2초 → 탐색 24초(원을 돈다) → 출발점으로 돌아와 → 착륙 2초.
  land면 바로 착륙, emergency면 바로 땅(ABORT), goto x y면 그 자리로 가서 3초 둘러본 뒤 다시 돈다(돌아가던 중이면 다시 돌아간다),
  goto cancel이면 그만둔다. 끝난 뒤 5초면 다시 READY(시험 편하게).
  찍은 자리 답(/tt/operator_goal text·warn)도 비행 노드처럼 낸다: 받음·도착·둘러봄·취소, 방 밖을 찍으면 받지 않음(warn).
--flip-after S: 날기 시작하고 S초 뒤 뒤집힌다(11/17 실기체처럼): 드라이버가 모터를 끄고(last_auto_land flipped) 임무는 중단(ABORT).
--sink-after S: 날기 시작하고 S초 뒤부터 실제 높이(하방 ToF·기압)가 0.35 m까지 내려간다. 자율 비행 추정 높이(/tt/pose_est z)는
  0.9 m 그대로다(11/17 충돌처럼 추정이 높게 굳음). 관제 화면 높이 알림 확인용.
자율 비행 노드 경고·오류(/rosout)도 흉내 낸다: 땅에서 드라이버 "영상 패킷이 안 옴"(5초마다, 비행 기록에는 안 남아야 한다),
  가라앉는 동안 비행 노드 "바로 밑 … 무엇이 있습니다"(2초마다), 뒤집히면 비행 노드 오류 한 번.
  지금 하는 일과 이유(/tt/mission_note)도 비행 노드 모양으로 낸다: 남은 탐색 시간, 돌아가는 이유, 내린 이유.
--auto: 예전처럼 READY에서 스스로 시작한다(땅 → 비행 → 착륙 되풀이). 그림은 480×360(자율 launch 기본)·15장/초.
--refuse-takeoff N: 드론이 처음 N번 이륙을 error로 거절한다(11/16 실기체 첫 takeoff처럼). 비행 노드(11/17)처럼 다시 보내지 않고
  시작 대기(READY)로 돌아간다. 다시 시작하면 다시 요청한다.
--yaw-drift S: 땅에 있을 때(켠 뒤·내린 뒤) 처음 S초 동안 Tello 기수 값이 1°/초로 흐른다(11/16 실기체처럼). 비행 노드처럼
  흐르는 동안(최근 10초에 2° 이상)은 시작을 받지 않고, 30초 안에 한 번 더 시작하면 그대로 요청한다.
자율 정보도 지어낸다: 위치(켠 자리 원점, x 앞·y 왼쪽), 계획 경로(앞쪽 원호, 찍은 자리로 갈 땐 그쪽 직선), 임무 단계(비행 노드 이름),
찾은 사람 둘(미확정 → 확정, 비행마다 새로), 찍은 자리 상태, YOLO 결과(늘 준비됨).

    python modules/autonomy/tello_link/fake_ros_view.py                 # 127.0.0.1:8094, 명령을 따라간다
    python modules/autonomy/tello_link/fake_ros_view.py --auto          # 스스로 되풀이
    python modules/autonomy/tello_link/gateway_server.py --ros-view http://127.0.0.1:8094 --port 8079

필요: 그림을 만들고 JPEG로 바꾸려면 numpy·OpenCV(게이트웨이 venv에 있다). --no-video면 없어도 된다.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import threading
import time
from collections import deque

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from ros_view_core import (DEFAULT_PORT, FLIGHT_CMD, MISSION_CMD, TAKEOFF_AGAIN_S, YAW_SETTLE_DEG,  # noqa: E402
                           YAW_SETTLE_WINDOW_S, ViewState, make_server, yaw_span)

GROUND_S, TAKEOFF_S, FLY_S, LAND_S = 8.0, 2.0, 30.0, 2.0
SEARCH_S = 24.0         # 탐색 시간(비행 노드 search_time): 다 되면 출발점으로 돌아와 내린다. 원 한 바퀴(FLY_S)보다 짧다
WIDTH, HEIGHT = 480, 360
RADIUS = 1.5            # 비행 중 그리는 원의 반지름(m)
FLY_STATES = ("PLAN", "FOLLOW", "LOOK", "FOLLOW", "REPLAN", "FOLLOW", "DOOR", "FOLLOW", "HOMESPIN")
PEOPLE = ((2.2, 1.0, 6.0, 9.0), (0.4, 2.6, 15.0, 18.0))   # (x, y, 처음 본 비행 초, 확정한 비행 초)
# 가짜 지도(비행 노드 /tt/obstacle_grid 흉내): 방 x −2.5~5·y −2~4.5 m의 벽, 책상(막힘), 의심 칸. 드론이 지나간 자리 둘레 2.5 m를 본 곳으로
GRID_RES, GRID_X0, GRID_Y0, GRID_W, GRID_H = 0.1, -4.0, -3.5, 110, 95
ROOM = (-2.5, 5.0, -2.0, 4.5)
BOXES = ((2.0, 3.0, 1.2, 2.0),)
SUSPECT = ((3.5, 4.1, -1.2, -0.6),)
SEEN_R = 2.5
FOUND = {"obstacles": [{"x": 3.0, "y": 3.6, "sx": 0.4, "sy": 0.4, "sources": ["tof"]}]}   # 찾은 장애물(비행 10초부터)
# --world: 시뮬레이터처럼 세계 지도(world_dir/known_map.json 모양: 범위·출발·목적지·벽·가구)와 정답 위치(/tt/odometry_gt)를 낸다
_X0, _X1, _Y0, _Y1 = ROOM
FAKE_WORLD = {"format": "tt-sim/known-map@2", "stage": 3, "description": "가짜 세계 지도(시험용, 시뮬레이터 known_map.json 모양)",
              "bounds": {"xmin": _X0, "ymin": _Y0, "xmax": _X1, "ymax": _Y1, "zmax": 2.5}, "flight_height": 0.9,
              "spawn": {"x": 0.0, "y": 0.0, "z": 0.1, "yaw": 0.0}, "goal": {"x": 4.2, "y": 3.8, "z": 0.9},
              "walls": [{"name": "wall_s", "x": (_X0 + _X1) / 2, "y": _Y0, "z": 1.25, "sx": _X1 - _X0, "sy": 0.1, "sz": 2.5},
                        {"name": "wall_n", "x": (_X0 + _X1) / 2, "y": _Y1, "z": 1.25, "sx": _X1 - _X0, "sy": 0.1, "sz": 2.5},
                        {"name": "wall_w", "x": _X0, "y": (_Y0 + _Y1) / 2, "z": 1.25, "sx": 0.1, "sy": _Y1 - _Y0, "sz": 2.5},
                        {"name": "wall_e", "x": _X1, "y": (_Y0 + _Y1) / 2, "z": 1.25, "sx": 0.1, "sy": _Y1 - _Y0, "sz": 2.5}],
              "obstacles": [{"name": "obstacle_00_desk", "x": 2.5, "y": 1.6, "z": 0.6, "sx": 1.0, "sy": 0.8, "sz": 1.2,
                             "kind": "desk", "yaw": 0.3}]}
TRUTH_OFFSET = (0.08, -0.05)   # 정답 위치 = 가짜 추정 위치 + 이만큼(시험에서 둘이 보이게)


def fake_cell(x: float, y: float) -> int:
    """가짜 지도 한 칸의 값(int8): 100 막힘·50 의심·0 본 빈 곳·−1 모름(방 밖)."""
    x0, x1, y0, y1 = ROOM
    h = GRID_RES
    wall = ((abs(x - x0) < h or abs(x - x1) < h) and y0 - h <= y <= y1 + h) or \
           ((abs(y - y0) < h or abs(y - y1) < h) and x0 - h <= x <= x1 + h)
    if wall or any(a <= x <= b and c <= y <= d for a, b, c, d in BOXES):
        return 100
    if any(a <= x <= b and c <= y <= d for a, b, c, d in SUSPECT):
        return 50
    return 0 if x0 < x < x1 and y0 < y < y1 else -1


class FakeGrid:
    """본 곳이 넓어지는 가짜 지도. data()는 OccupancyGrid data(int8을 바이트로, y 줄마다 x)."""

    def __init__(self):
        self.base = bytes(fake_cell(GRID_X0 + (i + 0.5) * GRID_RES, GRID_Y0 + (j + 0.5) * GRID_RES) & 0xFF
                          for j in range(GRID_H) for i in range(GRID_W))
        self.seen = bytearray(GRID_W * GRID_H)

    def reset(self) -> None:
        self.seen = bytearray(GRID_W * GRID_H)

    def look(self, px: float, py: float) -> None:
        r = int(SEEN_R / GRID_RES) + 1
        ci, cj = int((px - GRID_X0) / GRID_RES), int((py - GRID_Y0) / GRID_RES)
        for j in range(max(0, cj - r), min(GRID_H, cj + r + 1)):
            for i in range(max(0, ci - r), min(GRID_W, ci + r + 1)):
                if (i - ci) ** 2 + (j - cj) ** 2 <= r * r:
                    self.seen[j * GRID_W + i] = 1

    def data(self) -> bytes:
        return bytes(b if s else 255 for b, s in zip(self.base, self.seen))


def mission_at(phase: str, into: float) -> str:
    """비행 노드 단계 이름(mission_node.py와 같은 이름)."""
    if phase == "ground":
        return "WAIT_POSE" if into < 3.0 else "READY"
    if phase == "takeoff":
        return "TAKEOFF"
    if phase == "land":
        return "LAND"
    return FLY_STATES[min(len(FLY_STATES) - 1, int(into / FLY_S * len(FLY_STATES)))]


def circle(th: float) -> tuple[float, float]:
    """원 위 자리(시작점에서): 앞으로 나가 왼쪽으로 돈다."""
    return RADIUS * math.sin(th), RADIUS * (1 - math.cos(th))


def pose_at(phase: str, into: float) -> tuple[float, float, float, float]:
    """(x, y, z, yaw): 켠 자리 원점, x 앞·y 왼쪽, m·rad. 비행 중에는 앞으로 나가 왼쪽으로 원을 한 바퀴 돈다."""
    if phase == "ground":
        return 0.0, 0.0, 0.0, 0.0
    if phase == "takeoff":
        return 0.0, 0.0, 0.9 * into / TAKEOFF_S, 0.0
    if phase == "land":
        return 0.0, 0.0, 0.9 * (1 - into / LAND_S), 0.0
    th = 2 * math.pi * into / FLY_S
    return *circle(th), 0.9, math.atan2(math.sin(th), math.cos(th))


def path_at(into: float, center: tuple[float, float] = (0.0, 0.0)) -> list:
    """앞쪽 60° 원호(계획 경로). 2초마다 새로 계획한 것처럼 바뀐다."""
    th0 = 2 * math.pi * (int(into / 2.0) * 2.0) / FLY_S
    return [(center[0] + circle(th0 + k * math.pi / 30)[0], center[1] + circle(th0 + k * math.pi / 30)[1], 0.9) for k in range(11)]


def people_at(into: float) -> dict:
    """people.json 모양(비행 노드 save_people와 같은 키). 비행한 지 into초."""
    found = [{"x": x, "y": y, "conf": 0.86, "hits": 4 if into >= sure else 1, "gone": False, "t_first": round(seen, 1), "top": 1.6}
             for x, y, seen, sure in PEOPLE if into >= seen]
    return {"coverage": round(min(0.85, into / FLY_S), 3), "explored_m2": round(14 * min(1.0, into / FLY_S), 1),
            "t_start": 0.0, "search_time": round(into, 1),
            "people": [q for q, (_, _, _, sure) in zip(found, PEOPLE) if into >= sure],
            "unconfirmed": [q for q, (_, _, _, sure) in zip(found, PEOPLE) if into < sure]}


def phase_at(t: float) -> tuple[str, float]:
    """되풀이 안에서 지금 단계(ground·takeoff·flying·land)와 그 단계에 들어선 지 몇 초."""
    t %= GROUND_S + TAKEOFF_S + FLY_S + LAND_S
    for name, length in (("ground", GROUND_S), ("takeoff", TAKEOFF_S), ("flying", FLY_S), ("land", LAND_S)):
        if t < length:
            return name, t
        t -= length
    return "ground", 0.0


def tello_raw(t: float, phase: str, into: float, ground_yaw: int = 0, real_h: float | None = None) -> str:
    """Tello 상태 원문 한 줄(자율 드라이버가 /tt/state의 raw로 넘기는 모양). ground_yaw: 날지 않을 때의 기수 값(°).
    real_h: 실제 높이(m)를 따로 줄 때(--sink-after). 하방 ToF와 기압만 그 높이로 낸다."""
    up = {"ground": 0.0, "takeoff": min(1.0, into / TAKEOFF_S), "flying": 1.0, "land": max(0.0, 1.0 - into / LAND_S)}[phase]
    tof = max(10, round(10 + 80 * up + (3 * math.sin(t) if phase == "flying" else 0)))   # cm, 땅이면 10
    h = round(90 * up)
    baro = 12.5 + h / 100
    if real_h is not None:
        tof, baro = max(10, round(real_h * 100 + 3 * math.sin(t))), 12.5 + real_h
    yaw = round((into * 6) % 360 - 180) if phase == "flying" else ground_yaw
    bat = max(20, 95 - int(t / 10))
    temp = 55 + min(20, int(t / 30))
    return (f"mid:-1;x:-100;y:-100;z:-100;mpry:0,0,0;pitch:{round(2 * math.sin(t))};roll:1;yaw:{yaw};"
            f"vgx:0;vgy:0;vgz:0;templ:{temp};temph:{temp + 3};tof:{tof};h:{h};bat:{bat};baro:{baro:.2f};"
            f"time:{int(t) if phase != 'ground' else 0};agx:0.00;agy:0.00;agz:-1000.00;")


def flight_status(phase: str, bat: int, temp: int, operator: bool | None = None, last_result: dict | None = None,
                  last_auto_land: dict | None = None) -> dict:
    """자율 드라이버 /tt/flight_status 모양(FlightController.status + 드라이버가 더한 것). 값은 지어낸 것.
    operator: 드라이버 주변 확인(confirm/unconfirm). None이면 예전처럼 비행 중에만 확인됨으로 둔다.
    last_result: 드라이버의 마지막 이륙·착륙 결과({cmd, source, ok, …}). last_auto_land: 마지막 자동 착륙 {at, reason}."""
    op = phase != "ground" if operator is None else operator
    checks = [
        {"key": "sdk", "title": "SDK 모드", "ok": True, "detail": "command → ok", "blocking": True},
        {"key": "state", "title": "상태 수신", "ok": True, "detail": "0.1초 전", "blocking": True},
        {"key": "battery", "title": "배터리 10% 초과 (이하면 바로 착륙)", "ok": bat > 10, "detail": f"{bat}%", "blocking": True},
        {"key": "temp", "title": "온도 80°C 미만", "ok": temp < 80, "detail": f"{temp}°C", "blocking": True},
        {"key": "operator", "title": "주변 비움·가드 장착·안전 담당 확인", "ok": op, "detail": "확인함" if op else "확인 전",
         "blocking": True},
    ]
    # FlightController처럼 이륙 명령이 끝나기 전에는 떠 있지 않다(busy만 takeoff), 착륙 중에는 아직 떠 있다
    return {"flying": phase in ("flying", "land"), "busy": phase if phase in ("takeoff", "land") else None,
            "can_takeoff": phase == "ground" and all(c["ok"] for c in checks), "preflight": checks, "rc_limit": 80,
            "height_limit_m": 1.5, "ceiling_cap_m": None, "height_source": "down_tof", "last_rc": [0, 0, 0, 0],
            "input_source": "autonomy", "last_auto_land": last_auto_land, "last_warning": None, "connected": True,
            "last_result": last_result,
            "landing_reason": None, "state_rate_hz": 10.0, "source": "fake", "battery_land_pct": 10.0, "temp_land_c": 85.0}


class FakeFlight:
    """가짜 자율 비행 프로그램(비행 노드·드라이버 흉내). 명령은 보기 노드가 내는 토픽·문자열 그대로 받는다(on_command)."""

    GOTO_SPEED = 0.5    # m/s, 찍은 자리로 가는 빠르기
    ARRIVED_S = 3.0     # 찍은 자리에 도착해 둘러보는 시간
    DONE_S = 5.0        # 끝난 뒤 다시 READY가 되기까지(진짜 비행 노드는 DONE에 머문다. 시험을 되풀이하려고)
    AUTO_WAIT_S = 2.0   # --auto: READY가 되고 이만큼 뒤 스스로 시작
    YAW_DRIFT_DEG_S = 1.0   # --yaw-drift: 땅에서 기수 값이 흐르는 빠르기(°/초)

    def __init__(self, auto: bool = False, clock=time.monotonic, refuse: int = 0, yaw_drift: float = 0.0,
                 flip_after: float | None = None, sink_after: float | None = None):
        self.clock, self.auto = clock, auto
        self.sink_after = sink_after       # 날기 시작하고 이만큼(초) 뒤부터 실제 높이가 내려간다(추정은 그대로)
        self.flip_after = flip_after       # 날기 시작하고 이만큼(초) 뒤 뒤집힌다(None이면 안 뒤집힘)
        self.last_auto_land: dict | None = None   # 드라이버 last_auto_land {at, reason}
        self.refuse = refuse               # 드론이 error로 거절할 남은 이륙 수
        self.yaw_drift = yaw_drift         # 땅에 내려(켠 뒤·내린 뒤) 이만큼(초) 기수 값이 흐른다
        self.ground_yaw: deque = deque()   # 비행 노드가 모으는 땅 기수 값 (때, °)
        self.start_refused_at: float | None = None   # 기수 값이 흘러 시작을 받지 않은 때(비행 노드 start_refused_t)
        self.start_holds = 0               # 그렇게 받지 않은 시작 수
        self.requests = 0                  # 드라이버가 받은 이륙 요청 수(결과마다 다른 모양이 되게)
        self.last_result: dict | None = None   # 드라이버 last_result
        self.started = clock()
        self.phase, self.since = "ground", self.started
        self.ended: str | None = None      # 지난 비행이 어떻게 끝났나(DONE·ABORT). 끝난 뒤 잠깐 그 단계를 보인다
        self.operator = False              # 드라이버 주변 확인(confirm/unconfirm). 이륙하면 쓴 것으로 푼다
        self.fly_t = 0.0                   # 원을 돈 시간(찍은 자리로 가는 동안은 멈춤)
        self.center = (0.0, 0.0)           # 원의 시작점(찍은 자리에 도착하면 거기서 다시 돈다)
        self.pos = (0.0, 0.0)
        self.yaw = 0.0
        self.goal: dict | None = None
        self.goal_seq = 0
        self.goal_last = {"seq": 0, "t": 0.0, "state": "none", "text": "", "warn": False, "click": None, "goal": None}
        self.flights = 0
        self.returning = False             # 탐색 시간이 다 돼 출발점으로 돌아가는 중
        self.why_return: str | None = None
        self.why_end: str | None = None    # 내리거나 중단한 이유(비행 노드 why_end)
        self.received: list[tuple[str, str]] = []
        self._last = self.started
        self._lock = threading.Lock()

    # ---------- 명령 (보기 노드 publish 자리) ----------
    def on_command(self, topic: str, text: str) -> None:
        with self._lock:
            self.received.append((topic, text))
            now = self.clock()
            if topic == MISSION_CMD and text == "start":
                # 드라이버: 점검(주변 확인 포함)을 통과해야 이륙. 비행 노드: 땅에서 기수 값이 흐르면 시작을 받지 않는다
                if self.phase == "ground" and self._mission(now) == "READY" and self.operator and not self._start_blocked(now):
                    self._request_takeoff(now)
            elif text == "land" and topic in (MISSION_CMD, FLIGHT_CMD):
                if self.phase in ("takeoff", "flying"):
                    self.why_end = f"착륙 요청을 받았습니다 (지금 단계 {self._mission(now)})."   # 비행 노드: 바로 앞 로그
                    self._go("land", now)
                    if self.goal is not None:
                        self.goal = None
                        self._goal_note("내리게 되어 찍은 자리로는 가지 않습니다.")
            elif topic == FLIGHT_CMD and text == "emergency":
                if self.phase != "ground":   # 모터를 바로 끈다(공중이면 떨어진다)
                    self._go("ground", now)
                    self.ended, self.goal = "ABORT", None
                    self.why_end = "비상 정지로 모터를 껐습니다."
            elif topic == FLIGHT_CMD and text in ("confirm", "unconfirm"):
                if self.phase == "ground":
                    self.operator = text == "confirm"
            elif topic == MISSION_CMD and text == "goto cancel":
                if self.goal is not None:
                    self.goal = None
                    self._goal_note(f"찍은 자리를 취소했습니다. {self._goto_after()}.")
            elif topic == MISSION_CMD and text.startswith("goto "):
                try:
                    x, y = (float(v) for v in text.split()[1:3])
                except ValueError:
                    return
                if self.phase != "flying":
                    return
                x0, x1, y0, y1 = ROOM
                if not (x0 < x < x1 and y0 < y < y1):   # 비행 노드 goto_request: 갈 수 없으면 받지 않고 까닭을 알린다
                    self._goal_note(f"찍은 자리 ({x:.1f}, {y:.1f})를 받지 않습니다: 지도의 방 밖이라 갈 수 있는 빈 곳이 없습니다.",
                                    warn=True)
                    return
                d = math.hypot(x - self.pos[0], y - self.pos[1])
                self.goal = {"state": "going", "click": [x, y], "goal": [x, y], "since": now}
                self._goal_note(f"찍은 자리 ({x:.1f}, {y:.1f})로 갑니다 ({d:.1f} m 거리). 도착하면 둘러본 뒤 {self._goto_after()}.")

    def _request_takeoff(self, now: float) -> None:
        """비행 노드의 이륙 요청 한 번(드라이버가 받는다). 드론이 바로 거절하면(error) 비행 노드(11/17)처럼
        다시 보내지 않고 시작 대기(READY)에 남는다."""
        self.requests += 1
        base = {"cmd": "takeoff", "source": "autonomy"}
        if self.refuse > 0:   # 드론이 바로 error로 답했다(실기체 11/16)
            self.refuse -= 1
            self.last_result = {**base, "ok": False, "flying": False,
                                "result": {"cmd": "takeoff", "response": "error", "ok": False, "reason": "error_response",
                                           "request": self.requests}}
            return
        self.last_result = {**base, "ok": True, "flying": True,
                            "result": {"cmd": "takeoff", "response": "ok", "ok": True, "request": self.requests}}
        self._go("takeoff", now)

    def _goal_note(self, text: str, warn: bool = False) -> None:
        """비행 노드 goto_note: 찍은 자리의 지금 상태와 알릴 말을 새 번호·그때 시각으로 낸다(가는 중에는 같은 것을 다시 낸다)."""
        self.goal_seq += 1
        g = self.goal
        self.goal_last = {"seq": self.goal_seq, "t": round(self.clock() - self.started, 2),
                          "state": "none" if g is None else g["state"], "text": text, "warn": warn,
                          "click": None if g is None else list(g["click"]), "goal": None if g is None else list(g["goal"])}

    def _goto_after(self) -> str:
        """찍은 자리를 마친 뒤 할 일(비행 노드 goto_after와 같은 뜻)."""
        return "출발점으로 돌아갑니다" if self.returning else "자율 탐색을 이어 갑니다"

    def _start_blocked(self, now: float) -> bool:
        """비행 노드 start_blocked: 땅에서 기수 값이 흐르면 시작을 받지 않는다(30초 안에 한 번 더 누르면 받는다)."""
        span = yaw_span(self.ground_yaw, now)
        if span is None or span < YAW_SETTLE_DEG:
            self.start_refused_at = None
            return False
        if self.start_refused_at is not None and now - self.start_refused_at <= TAKEOFF_AGAIN_S:
            self.start_refused_at = None
            return False
        self.start_refused_at = now
        self.start_holds += 1
        return True

    def ground_yaw_now(self, now: float) -> int:
        """땅에 있을 때 Tello 기수 값(°, 1° 단위): 켠 뒤·내린 뒤 yaw_drift초 동안 1°/초로 흐르다 멈춘다."""
        return round(self.YAW_DRIFT_DEG_S * min(max(0.0, now - self.since), self.yaw_drift))

    def _go(self, phase: str, now: float) -> None:
        self.phase, self.since = phase, now
        if phase == "takeoff":
            self.flights += 1
            self.operator, self.ended = False, None   # 드라이버가 이륙에 주변 확인을 썼다
            self.returning, self.why_return, self.why_end = False, None, None
            self.fly_t, self.center, self.pos, self.yaw, self.goal = 0.0, (0.0, 0.0), (0.0, 0.0), 0.0, None

    # ---------- 시간 ----------
    def step(self) -> None:
        with self._lock:
            now = self.clock()
            dt, self._last = now - self._last, now
            into = now - self.since
            if self.phase == "ground":   # 비행 노드가 땅 기수 값을 모은다(뜨면 비운다: 웹과 같게)
                self.ground_yaw.append((now, float(self.ground_yaw_now(now))))
                while self.ground_yaw and self.ground_yaw[0][0] < now - YAW_SETTLE_WINDOW_S - 2.0:
                    self.ground_yaw.popleft()
            else:
                self.ground_yaw.clear()
            if self.phase == "ground":
                if self.ended and into >= self.DONE_S:
                    self.ended = None
                if self.auto and self._mission(now) == "READY" and into >= self.AUTO_WAIT_S + (0 if self.flights else 3.0):
                    self.operator = True
                    self._go("takeoff", now)
            elif self.phase == "takeoff" and into >= TAKEOFF_S:
                self._go("flying", now)
            elif self.phase == "flying":
                if self.flip_after is not None and into >= self.flip_after:
                    self._flip(now)
                else:
                    self._fly(now, dt)
            elif self.phase == "land" and into >= LAND_S:
                self._go("ground", now)
                self.ended, self.goal = "DONE", None

    def real_height(self, now: float) -> float | None:
        """--sink-after: 실제 높이(m). 날기 시작하고 S초 뒤부터 0.2 m/s로 0.35 m까지 내려간다. 보통이면 None."""
        if self.sink_after is None or self.phase != "flying" or now - self.since < self.sink_after:
            return None
        return max(0.35, 0.9 - 0.2 * (now - self.since - self.sink_after))

    def _flip(self, now: float) -> None:
        """드라이버 뒤집힘 처리: 모터를 바로 꺼 땅(떨어짐), 자동 착륙 까닭 flipped, 임무 중단."""
        self._go("ground", now)
        self.ended, self.goal = "ABORT", None
        self.last_auto_land = {"at": time.strftime("%Y-%m-%dT%H:%M:%S") + f".{self.flights}", "reason": "flipped"}
        self.why_end = "드라이버가 뒤집힘으로 보고 모터를 껐습니다."

    def _fly(self, now: float, dt: float) -> None:
        g = self.goal
        if g and g["state"] == "going":
            (gx, gy), (x, y) = g["goal"], self.pos
            d = math.hypot(gx - x, gy - y)
            if d <= 0.15:
                g.update(state="arrived", since=now)
                self._goal_note("찍은 자리에 도착해 둘러봅니다.")
            else:
                s = min(d, self.GOTO_SPEED * dt)
                self.pos, self.yaw = (x + (gx - x) / d * s, y + (gy - y) / d * s), math.atan2(gy - y, gx - x)
            return
        if g and g["state"] == "arrived":
            self.yaw += 1.0 * dt   # 둘러본다
            if now - g["since"] >= self.ARRIVED_S:
                self.goal = None
                self._goal_note(f"찍은 자리를 둘러봤습니다. {self._goto_after()}.")
                th = 2 * math.pi * self.fly_t / FLY_S   # 이 자리에서 원을 이어 돈다(튀지 않게)
                ox, oy = circle(th)
                self.center = (self.pos[0] - ox, self.pos[1] - oy)
            return
        if self.returning:   # 출발점(켠 자리)으로 곧장 와서 내린다
            (x, y) = self.pos
            d = math.hypot(x, y)
            if d <= 0.15:
                self.why_end = f"{self.why_return} 출발점으로 돌아와 내립니다."
                self._go("land", now)
            else:
                s = min(d, self.GOTO_SPEED * dt)
                self.pos, self.yaw = (x - x / d * s, y - y / d * s), math.atan2(-y, -x)
            return
        self.fly_t += dt
        th = 2 * math.pi * self.fly_t / FLY_S
        ox, oy = circle(th)
        self.pos, self.yaw = (self.center[0] + ox, self.center[1] + oy), math.atan2(math.sin(th), math.cos(th))
        if self.fly_t >= SEARCH_S:
            self.returning, self.why_return = True, f"탐색 시간({SEARCH_S:.0f}초)이 다 돼"

    # ---------- 내보내는 값 ----------
    def _mission(self, now: float) -> str:
        if self.phase == "ground":
            return self.ended or ("WAIT_POSE" if now - self.started < 3.0 else "READY")
        if self.phase == "flying":
            if self.goal:
                return "FOLLOW" if self.goal["state"] == "going" else "LOOK"
            return mission_at("flying", self.fly_t)
        return mission_at(self.phase, 0.0)

    def mission(self) -> str:
        with self._lock:
            return self._mission(self.clock())

    def pose(self) -> tuple[float, float, float, float]:
        with self._lock:
            into = self.clock() - self.since
            z = {"ground": 0.0, "takeoff": 0.9 * min(1.0, into / TAKEOFF_S), "flying": 0.9,
                 "land": 0.9 * max(0.0, 1 - into / LAND_S)}[self.phase]
            return self.pos[0], self.pos[1], z, math.atan2(math.sin(self.yaw), math.cos(self.yaw))

    def path(self) -> list:
        with self._lock:
            g = self.goal
            if g and g["state"] == "going":   # 찍은 자리로 곧장
                (x, y), (gx, gy) = self.pos, g["goal"]
                return [(x + (gx - x) * k / 10, y + (gy - y) * k / 10, 0.9) for k in range(11)]
            if self.returning:   # 출발점으로 곧장
                (x, y) = self.pos
                return [(x * (1 - k / 10), y * (1 - k / 10), 0.9) for k in range(11)]
            return path_at(self.fly_t, self.center)

    def note_msg(self) -> dict:
        """비행 노드 /tt/mission_note 모양 {t, state, search{elapsed, limit}, returning, why_return, return_home, why_end}."""
        with self._lock:
            now = self.clock()
            return {"t": round(now - self.started, 2), "state": self._mission(now),
                    "search": {"elapsed": round(self.fly_t, 1), "limit": SEARCH_S} if self.flights else None,
                    "returning": self.returning, "why_return": self.why_return if self.returning else None,
                    "return_home": True, "why_end": self.why_end}

    def goal_msg(self) -> dict:
        """비행 노드 /tt/operator_goal 마지막 것 {seq, t, state(none·pending·going·arrived), text, warn, click, goal}."""
        with self._lock:
            return dict(self.goal_last)


def frames(view: ViewState, stop: threading.Event, fps: float = 15.0) -> None:
    import numpy as np

    bars = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
    colors = [(200, 200, 200), (200, 200, 60), (60, 200, 200), (60, 200, 60), (200, 60, 200), (200, 60, 60), (60, 60, 200)]
    for i, c in enumerate(colors):   # RGB
        bars[:, i * WIDTH // len(colors):(i + 1) * WIDTH // len(colors)] = c
    t0, n = time.monotonic(), 0
    while not stop.wait(max(0.0, t0 + n / fps - time.monotonic())):
        img = bars.copy()
        x = int((WIDTH - 60) * (0.5 + 0.5 * math.sin(n / fps)))
        img[HEIGHT // 2 - 30:HEIGHT // 2 + 30, x:x + 60] = (255, 255, 255)
        view.on_image(WIDTH, HEIGHT, "rgb8", img.tobytes(), WIDTH * 3)
        n += 1


def feed(view: ViewState, fake: FakeFlight, stop: threading.Event, world: bool = False) -> None:
    t0 = time.monotonic()
    if world:   # 시뮬레이터처럼 세계 지도 파일(켤 때 한 번)
        view.on_world_map(FAKE_WORLD, "fake/known_map.json")
    i = 0
    people_key = None
    flight_seen = 0
    goal_sent = None
    flip_said = None
    grid, found = FakeGrid(), False
    while not stop.wait(0.1):
        t = time.monotonic() - t0
        fake.step()
        phase, into = fake.phase, fake.clock() - fake.since
        yaw = fake.ground_yaw_now(fake.clock()) if phase == "ground" else 0
        real = fake.real_height(fake.clock())
        view.on_state(json.dumps({"stamp": round(t, 3), "raw": tello_raw(t, phase, into, yaw, real)}))
        if phase == "ground" and i % 50 == 0:   # 드라이버: 땅에서 영상이 안 오면 5초마다(비행 기록에는 안 남아야 한다)
            view.on_log(30, "tt_tello_driver", "영상 패킷이 5초 동안 오지 않습니다 (WSL UDP 11111).")
        if real is not None and real < 0.5 and i % 20 == 0:
            view.on_log(30, "tt_mission", f"바로 밑 {real:.2f} m에 무엇이 있습니다(꼭대기 약 {0.9 - real:.2f} m). "
                                          "멈추고 0.6 m 떨어질 때까지 오릅니다.")
        if fake.last_auto_land and fake.last_auto_land["at"] != flip_said:
            flip_said = fake.last_auto_land["at"]
            view.on_log(40, "tt_mission", "드라이버가 기체를 내렸습니다 (flipped: 뒤집힘 (90° 넘게 기운 채 0.5초)). 임무를 중단합니다.")
        if i % 2 == 0:   # 드라이버는 0.2초마다
            bat, temp = max(20, 95 - int(t / 10)), 55 + min(20, int(t / 30))
            view.on_flight_status(json.dumps(flight_status(phase, bat, temp, fake.operator, fake.last_result,
                                                           fake.last_auto_land)))
            view.on_front_tof(float("inf") if phase != "flying" or (int(t) % 10) < 3 else 0.6 + 0.4 * math.sin(t / 2))
        x, y, z, yaw = fake.pose()
        view.on_pose(x, y, z, yaw, 0.12 if phase == "flying" else None)   # 위치 추정은 10Hz, 땅에선 오차 모름
        if world:
            view.on_truth(x + TRUTH_OFFSET[0], y + TRUTH_OFFSET[1], z, yaw)
        if i % 10 == 0:   # 비행 노드는 단계와 지금 하는 일과 이유를 1초마다 다시 낸다
            view.on_mission_state(fake.mission())
            view.on_note(json.dumps(fake.note_msg(), ensure_ascii=False))
        goal = fake.goal_msg()   # 찍은 자리: 바뀌면 바로, 가는 중·둘러보는 중에는 1초마다 같은 것을 다시(비행 노드와 같게)
        if goal["seq"] != goal_sent or (i % 10 == 0 and goal["state"] in ("pending", "going", "arrived")):
            view.on_goal(json.dumps(goal, ensure_ascii=False))
            goal_sent = goal["seq"]
        if i % 5 == 0:   # YOLO는 늘 준비됨(가짜 결과)
            view.on_detections("{}")
        if phase == "flying":
            view.on_path(fake.path())   # 같은 경로면 version이 오르지 않는다
            people = people_at(fake.fly_t)
            key = (len(people["people"]), len(people["unconfirmed"]), people["coverage"])
            if key != people_key:
                view.on_people(people, "fake/people.json")
                people_key = key
            grid.look(x, y)   # 비행 노드처럼 날면서 지도를 만든다(1초마다 낸다)
            if i % 10 == 0:
                view.on_grid(GRID_RES, GRID_W, GRID_H, GRID_X0, GRID_Y0, grid.data())
            if not found and fake.fly_t >= 10.0:
                view.on_discovered(FOUND, "fake/discovered.json")
                found = True
        elif phase == "takeoff" and fake.flights != flight_seen:   # 새 비행: 사람·찾은 장애물 파일은 아직 없다, 지도는 새로
            view.on_people(None, None)
            view.on_discovered(None, None)
            grid.reset()
            people_key, flight_seen, found = None, fake.flights, False
        i += 1


def main() -> None:
    ap = argparse.ArgumentParser(description="가짜 자율 비행 보기 노드(게이트웨이 --ros-view 시험용)")
    ap.add_argument("--bind", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--no-video", action="store_true", help="그림을 만들지 않는다(numpy·OpenCV 없이)")
    ap.add_argument("--auto", action="store_true", help="READY에서 스스로 시작한다(땅 → 비행 → 착륙 되풀이)")
    ap.add_argument("--no-commands", action="store_true", help="명령을 받지 않는다(보기만, 진짜 노드 commands:=false)")
    ap.add_argument("--refuse-takeoff", type=int, default=0, help="드론이 처음 N번 이륙을 error로 거절한다(실기체 11/16처럼)")
    ap.add_argument("--yaw-drift", type=float, default=0.0, metavar="S",
                    help="땅에 있을 때 처음 S초 동안 기수 값이 1°/초로 흐른다(비행 노드가 그동안 시작을 받지 않는다)")
    ap.add_argument("--flip-after", type=float, default=None, metavar="S",
                    help="날기 시작하고 S초 뒤 뒤집힌다(드라이버가 모터를 끔, 임무 중단)")
    ap.add_argument("--world", action="store_true",
                    help="시뮬레이터처럼 세계 지도(벽·가구·목적지·출발)와 정답 위치를 낸다(자율 비행 웹이 그리는 것)")
    ap.add_argument("--sink-after", type=float, default=None, metavar="S",
                    help="날기 시작하고 S초 뒤부터 실제 높이가 0.35 m까지 내려간다(자율 비행 추정은 0.9 m 그대로)")
    args = ap.parse_args()
    fake, stop = FakeFlight(auto=args.auto, refuse=args.refuse_takeoff, yaw_drift=args.yaw_drift,
                            flip_after=args.flip_after, sink_after=args.sink_after), threading.Event()
    view = ViewState(publish=None if args.no_commands else fake.on_command, need_yolo=True)
    threading.Thread(target=feed, args=(view, fake, stop, args.world), daemon=True).start()
    if not args.no_video:
        threading.Thread(target=frames, args=(view, stop), daemon=True).start()
    server = make_server(view, args.bind, args.port)
    how = "스스로 되풀이" if args.auto else "관제 화면 명령을 따라감(주변 확인 → 시작 두 번)"
    print(f"가짜 자율 비행 보기 노드: http://{args.bind}:{args.port}  ({how}, 탐색 {SEARCH_S:.0f}초)", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        server.stopping.set()
        server.server_close()


if __name__ == "__main__":
    main()
