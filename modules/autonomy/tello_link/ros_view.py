"""자율 비행(ROS) 보기: 게이트웨이가 드론 대신 보기 노드(ros_view_node.py)에서 상태·영상을 받아 화면에 보인다 (표준 라이브러리만).

자율 비행 프로그램이 드론에 붙어 난다(드라이버·안전 장치·이륙 흐름 그대로). 게이트웨이는 드론에 붙지 않고 보기만 한다.
서버 `--ros-view http://127.0.0.1:8094`. 기록 재생(replay.py)처럼 서버가 쓰는 모양만 맞춘다.
- 상태: Tello 원문을 parse_state로 풀어 배터리·높이·자세. 모드는 자율 드라이버 상태로 나눈다:
  보기 노드·드론 끊김 disconnected, 안 떠 있음 grounded, 착륙 중 landing, 떠 있음 auto(자율이 조종). 이륙 전 점검은 드라이버 것 그대로.
- 영상: 보기 노드 MJPEG → 프레임 저장소(/snapshot.jpg·/video.mjpg). 사람 탐지 연결기가 지금처럼 가져간다.
- 사람 탐지·확인·오탐, AI 합성, 3D 지도 받기는 실시간 게이트웨이와 같다.
- 자율 정보(2단계): 보기 노드의 위치·계획 경로·찾은 사람·임무 단계를 자율 창구(AutonomyLink)에 그대로 넣는다.
  2D 지도·발견 지점·기록(autonomy/)이 가짜 자율 때와 같이 돈다. 좌표는 자율 비행 world = 게이트웨이 map(바꾸지 않음).
  임무 단계(WAIT_POSE·FOLLOW 등)는 drone_state.mission.state로 그대로, phase는 대기·진행 중·도착·실패로 나눈다.
  지금 하는 일과 이유(11/17 /tt/mission_note: 남은 탐색 시간·돌아가는 이유·내린 이유)는 drone_state.mission.note로
  자율 비행 웹과 같은 규칙(3초 넘게 안 오면 내린 이유만)으로 넘기고, 돌아가기 시작한 이유·내리는 이유는 비행 기록에 적는다.
  찍은 자리에 대한 비행 노드의 마지막 답(받음·거절·그만둠·도착, /tt/operator_goal text·warn)은 drone_state.mission.goal_note로
  처음 들은 지 몇 초와 함께 넘기고(화면이 [여기로 보내기] 옆에 20초 보인다) 비행 기록에도 적는다.
- 명령(3단계): 화면 버튼을 보기 노드(POST /api/<명령>)에 넘기면, 보기 노드가 자율 비행 웹과 같은 토픽·문자열로 낸다.
  이륙(takeoff) → 자율 시작(start), 착륙 → land, 비상 정지 → emergency, 주변 확인 → confirm/unconfirm, 지도 찍은 자리 → goto.
  버튼을 여는 조건(allowed)도 자율 비행 웹과 같다: 시작은 임무 READY + 드라이버 이륙 가능 + YOLO 준비(need_yolo일 때),
  착륙은 떠 있거나 이륙·착륙 중이거나 임무가 공중 단계, 찍은 자리는 탐색 비행 단계에서만. 수동 조종·일시정지·넘겨받기는 없다.
  드라이버가 스스로 내리면(속도 명령 끊김·배터리·충돌·뒤집힘 등) 그 까닭을 모드 바뀜 까닭으로 남긴다.
  시작 칸(11/17): 땅에서 Tello 기수 값이 흐르면(최근 10초에 2° 이상, 비행 노드와 같은 잣대) ros_view.start.yaw_drifting으로 알린다.
  그동안 비행 노드는 시작을 받지 않지만(30초 안에 한 번 더 누르면 받는다) 게이트웨이는 막지 않는다.
- 높이 지켜보기(11/17 충돌 뒤, height_watch.py): 자율 비행 중 아래가 가깝거나(하방 0.5 m 안), 자율 비행 높이 추정이
  잰 높이(기압·하방)와 0.4 m 넘게 어긋나면 drone_state.warnings(low_altitude·height_mismatch)·height_watch와
  비행 기록으로 알린다. 비행에는 끼어들지 않는다(병우님 기준 그대로).
- 회차 폴더(manifest source "ros"): sensor(상태)·events(모드 바뀜·보기 노드 연결)·detections·map·metrics.
  게이트웨이 회차와 같은 모양이라 기록 재생(--replay)으로 다시 볼 수 있다(영상 원본은 자율 드라이버가 자기 폴더에 남긴다).
"""

from __future__ import annotations

import json
import math
import os
import re
import secrets
import threading
import time
from collections import deque
from urllib.error import HTTPError
from urllib.request import Request, urlopen

from autonomy_link import MAX_PATH_POINTS, MAX_PEOPLE, AutonomyLink
from detection_inbox import DetectionInbox
from frame_store import FrameStore
from height_watch import HeightWatch
from map_inbox import MapInbox
from mode_arbiter import SCHEMA_VERSION
from overlay_store import OverlayStore
from ros_view_core import CLIENT_HEADER, YAW_SETTLE_DEG, YAW_SETTLE_WINDOW_S, goto_command, yaw_span
from sensor_pose import TelloOdometry
from tello_link import RunLog, now_iso, parse_state

POLL_S = 0.05             # 보기 노드 상태를 묻는 간격(Tello 상태 약 10Hz의 두 배): 화면·지도용 센서 자세가 늦지 않게.
                          # 상태는 번호로 놓친 것까지 받는다(?states_after=). 한 바퀴가 늦어도 빠지지 않는다
POLL_TIMEOUT_S = 1.0
TAP_STALE_S = 1.5         # 보기 노드가 이보다 오래 답하지 않으면 끊김
STATE_STALE_S = 1.0       # Tello 상태가 이보다 오래되면 끊김(게이트웨이·자율 드라이버와 같은 1초)
FLIGHT_STALE_S = 2.0      # 자율 드라이버 상태(0.2초마다)가 이보다 오래되면 끊김
VIDEO_STALE_S = 1.0
FRONT_TOF_STALE_S = 1.0
STREAM_TIMEOUT_S = 5.0    # 영상이 이만큼 안 오면 다시 붙는다
RECONNECT_S = 1.0
BATTERY_WARN_PCT = 30
TEMP_WARN_C = 75
AUTONOMY_SESSION = "ros-view"
AUTONOMY_STALE_S = 5.0    # 위치·임무 단계가 이만큼 안 오면 자율 연결을 닫는다(비행 노드가 꺼짐)
MISSION_IDLE = ("WAIT_POSE", "READY")   # 비행 노드가 출발 전(위치 잡는 중·시작 기다림)
# 자율 비행 웹(static/index.html)과 같은 단계 묶음: 공중 단계(착륙 버튼), 찍은 자리를 보낼 수 있는 탐색 비행 단계
AIRBORNE = ("TAKEOFF", "PLAN", "FOLLOW", "REPLAN", "BACKOFF", "HOLD", "UNSTICK", "EXIT", "LOOK", "RELOC", "HOMESPIN", "DOOR",
            "WANDER", "TURN", "GAP", "LAND")
GOTO_STATES = ("PLAN", "FOLLOW", "REPLAN", "BACKOFF", "HOLD", "EXIT", "LOOK", "RELOC", "DOOR")
GOAL_ACTIVE = ("pending", "going", "arrived")
GOAL_FRESH_S = 3.0        # 웹: 찍은 자리 소식이 3초 넘게 없으면(비행 노드 꺼짐) 없는 것으로 본다
NOTE_FRESH_S = 3.0        # 웹: 지금 하는 일과 이유(1초마다)가 3초 넘게 없으면(비행 노드 꺼짐) 내린 이유만 남긴다
# 화면 명령 → 보기 노드 명령(자율 비행 웹 API와 같은 이름). 여기 없는 것(수동·일시정지·넘겨받기·영상)은 보기 모드에 없다
ROS_CMDS = {"takeoff": "mission/start", "land": "mission/land", "emergency": "flight/emergency",
            "confirm_area": "flight/confirm", "goto": "mission/goto", "goto_cancel": "mission/goto_cancel"}
CMD_TIMEOUT_S = 1.0
EMERGENCY_REASON_S = 5.0  # 비상 정지를 보내고 이 안에 땅으로 바뀌면 까닭은 비상 정지
AIR_MODES = ("auto", "landing")   # 비행 시간을 세는 모드(드라이버가 날고 있거나 내려오는 중)
MAP_SIZE = (960, 720)             # 지도 파트 실시간 지도가 받는 영상 크기(Tello 원본, 내장 보정값)
ERROR_LOG_FIRST = 3               # 같은 오류는 처음 3번과 그 뒤 100번마다만 회차 기록에 남긴다(10 Hz로 쌓이지 않게)
GRID_SAVE_S = 5.0                 # 자율 비행 지도를 회차 폴더(autonomy/grid.json)에 남기는 간격
# 자율 비행 노드 경고·오류(/rosout WARN 이상)를 비행 기록에: 떠 있을 때(내린 뒤 10초까지)의 경고와 늘 오류만.
# 11/17 실기체: 비행 노드 경고는 한 비행에 약 60개였고, 드라이버의 "영상 패킷이 안 옴"은 땅에서도 853번 났다.
# 그래서 같은 말(숫자는 빼고 견줌)은 30초에 한 번까지만 적는다
LOG_ERROR = 40
LOG_AFTER_LANDING_S = 10.0
LOG_REPEAT_S = 30.0


def _finite_xy(p) -> bool:
    return isinstance(p, (list, tuple)) and len(p) in (2, 3) and all(
        isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in p)


def _good_person(q) -> bool:
    return isinstance(q, dict) and _finite_xy([q.get("x"), q.get("y")])


def clean_path(points: list) -> list:
    """자율 창구가 받는 경로: 좌표가 숫자인 점만, 너무 길면 끝쪽(앞으로 갈 길) MAX_PATH_POINTS개."""
    return [p for p in points if _finite_xy(p)][-MAX_PATH_POINTS:]


def clean_people(people: list, unconfirmed: list) -> tuple[list, list]:
    """자율 창구가 받는 찾은 사람: 좌표가 숫자인 사람만, 합쳐 MAX_PEOPLE명까지(확정 먼저). 비행 노드 기록은 줄이지 않는다."""
    people = [q for q in people if _good_person(q)][:MAX_PEOPLE]
    return people, [q for q in unconfirmed if _good_person(q)][:MAX_PEOPLE - len(people)]


def mission_phase(state: str) -> str:
    """비행 노드 단계 이름 → 화면 phase(idle·running·arrived·failed)."""
    if state in MISSION_IDLE:
        return "idle"
    if state == "DONE":
        return "arrived"
    if state == "ABORT":
        return "failed"
    return "running"


def found_item(o: dict) -> dict:
    """찾은 장애물 하나: 자리·크기와 출처 이름(sources, 문자열만 6개까지)."""
    item = {k: o[k] for k in ("x", "y", "sx", "sy")}
    src = o.get("sources")
    if isinstance(src, list):
        item["sources"] = [s[:40] for s in src if isinstance(s, str)][:6]
    return item


def _num(v) -> float | None:
    return float(v) if isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) else None


def world_map_item(d: dict) -> dict:
    """자율 비행 웹이 그리는 지도 파일(보기 노드 /api/map)에서 화면이 그리는 것만(웹 drawMap과 같음): 범위·출발·목적지·벽·가구
    (자리·크기·방향), 빈 지도인지(open: 실기체), 설명. 이상한 값은 뺀다."""
    def box(o):
        if not isinstance(o, dict) or any(_num(o.get(k)) is None for k in ("x", "y", "sx", "sy")):
            return None
        item = {k: _num(o[k]) for k in ("x", "y", "sx", "sy")}
        if _num(o.get("yaw")) is not None:
            item["yaw"] = _num(o.get("yaw"))
        return item

    def point(o):
        ok = isinstance(o, dict) and _num(o.get("x")) is not None and _num(o.get("y")) is not None
        return {"x": _num(o["x"]), "y": _num(o["y"])} if ok else None

    def boxes(v):
        return [x for x in map(box, v) if x][:2000] if isinstance(v, list) else []
    b = d.get("bounds")
    ok = isinstance(b, dict) and all(_num(b.get(k)) is not None for k in ("xmin", "ymin", "xmax", "ymax"))
    bounds = {k: _num(b[k]) for k in ("xmin", "ymin", "xmax", "ymax")} if ok and b["xmax"] > b["xmin"] and b["ymax"] > b["ymin"] else None
    return {"bounds": bounds, "spawn": point(d.get("spawn")), "goal": point(d.get("goal")), "walls": boxes(d.get("walls")),
            "obstacles": boxes(d.get("obstacles")), "open": bool(d.get("open")), "description": _text(d.get("description"), 200),
            "frame": "map"}


def _text(v, limit: int = 300) -> str | None:
    return v[:limit] if isinstance(v, str) and v.strip() else None


def clean_note(note: dict) -> dict:
    """화면에 넘길 지금 하는 일과 이유(비행 노드 /tt/mission_note). 아는 칸만, 모양이 틀린 값은 None(지어내지 않는다)."""
    s = note.get("search")
    ok = isinstance(s, dict) and all(isinstance(s.get(k), (int, float)) and not isinstance(s.get(k), bool)
                                     and math.isfinite(s[k]) for k in ("elapsed", "limit"))
    home = note.get("return_home")
    return {"state": _text(note.get("state"), 40),
            "search": {"elapsed": round(float(s["elapsed"]), 1), "limit": round(float(s["limit"]), 1)} if ok else None,
            "returning": note.get("returning") is True, "why_return": _text(note.get("why_return")),
            "return_home": home if isinstance(home, bool) else None, "why_end": _text(note.get("why_end"))}


def note_events(old: dict | None, new: dict) -> list[dict]:
    """비행 기록에 적을 것(정리한 note끼리): 자율 비행 웹 note_events와 같은 때만. 돌아가기 시작한 이유와
    내리는(중단한) 이유가 새로 생기거나 바뀔 때. 1초마다 오는 나머지(남은 탐색 시간)는 적지 않는다."""
    old = old or {}
    out = []
    why = new.get("why_return")
    if new.get("returning") and why and not (old.get("returning") and old.get("why_return") == why):
        out.append({"what": "returning", "why_return": why})
    if new.get("why_end") and new.get("why_end") != old.get("why_end"):
        out.append({"what": "end", "why_end": new["why_end"], "state": new.get("state")})
    return out


class ViewLink:
    """서버가 쓰는 gw.link 자리: 회차 번호·기록(RunLog)·끝낼 때 metrics. 드론에는 붙지 않는다."""

    def __init__(self, log_root: str, url: str):
        self.run_id = time.strftime("ros_%Y%m%d_%H%M%S_") + secrets.token_hex(3)
        self.log = RunLog(log_root, self.run_id, {
            "run_id": self.run_id, "source": "ros", "ros_view_url": url, "started_at": now_iso(),
            "operator_controlled": False, "note": "자율 비행 보기: 자율 비행 프로그램이 드론에 붙고 게이트웨이는 보기만 한다"})
        self.metrics = lambda: {}
        self._started = time.monotonic()
        self._closed = False

    def note_state(self, sample) -> None:
        """새 Tello 상태 하나(센서 자세를 만들 때만 쓴다: SensorViewLink)."""

    def event(self, kind: str, detail: dict) -> None:
        self.log.append("events", {"t": now_iso(), "type": kind, "detail": detail})

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self.log.close({"run_id": self.run_id, "source": "ros", "duration_s": round(time.monotonic() - self._started, 1),
                        **self.metrics()})


class SensorViewLink(ViewLink):
    """지도 파트에 실기체 게이트웨이(TelloLink)와 같은 센서 자세(sensor_pose_at)와 센서 기록(sensor_samples)을 준다.

    자율 비행 드라이버가 받은 Tello 상태 원문을 같은 TelloOdometry로 적분한다(받은 때 = 게이트웨이 시계, 영상 장 시각과 같은 시계).
    지도 파트 실시간 지도는 Tello 입력이면 장마다 이 자세로 위치를 잡는다. 없으면 영상 특징점과 깊이로 앞서 본 장면에
    맞추다가(OpenCV) 놓치면 짐작한 위치로 쌓아서, 날면서 만든 지도가 번졌다.

    실시간으로 도는 드론(실기체·가짜 Tello)에서만 켠다(--ros-view-sensor-pose). 이 PC 시뮬레이터는 실제보다 느리게 돌아
    (상태 약 4 Hz) 벽시계로 적분하면 거리가 맞지 않고 상태 사이가 길어 자세 없는 장이 많아서, 영상만 쓰는 편이 낫다."""

    def __init__(self, log_root: str, url: str):
        super().__init__(log_root, url)
        self._state_lock = threading.Lock()
        self._odometry = TelloOdometry()
        self._sensor_history: deque = deque(maxlen=512)   # GET /api/drone/sensors(지도 파트 녹화), TelloLink와 같은 수
        self.state_count = 0

    def note_state(self, sample) -> None:
        with self._state_lock:
            self._odometry.update(sample.raw, sample.received_mono)
            self.state_count += 1
            self._sensor_history.append({**sample.to_dict(), "time_source": "ros_view", "sequence": self.state_count})

    def sensor_pose_at(self, frame_stamp):
        with self._state_lock:
            return self._odometry.at(frame_stamp)

    def sensor_samples(self, after=0):
        """TelloLink.sensor_samples와 같은 모양(tello-sensor-batch/1.0). 빠진 것은 lost로 알린다."""
        with self._state_lock:
            first = self._sensor_history[0]['sequence'] if self._sensor_history else self.state_count + 1
            return {"schema_version": "tello-sensor-batch/1.0", "run_id": self.run_id,
                    "next_sequence": self.state_count, "lost": max(0, first - after - 1),
                    "samples": [dict(row) for row in self._sensor_history if row['sequence'] > after]}


def scale_jpeg(jpeg: bytes, size: tuple[int, int]) -> bytes:
    """JPEG을 size로 키운다(지도용). 이미 그 크기이거나 가로세로 비율이 다르면(화각이 달라 보정이 안 맞음) 그대로 둔다."""
    try:
        import cv2
        import numpy as np
    except ImportError:
        return jpeg
    img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        return jpeg
    h, w = img.shape[:2]
    if (w, h) == size or w * size[1] != h * size[0]:
        return jpeg
    ok, out = cv2.imencode(".jpg", cv2.resize(img, size, interpolation=cv2.INTER_LINEAR), [cv2.IMWRITE_JPEG_QUALITY, 90])
    return out.tobytes() if ok else jpeg


class ViewVideo:
    """보기 노드의 MJPEG를 읽어 최신 장 하나를 들고 있다. 서버가 쓰는 gw.video 자리(jpeg·jpeg_snapshot·running·stop)."""

    def __init__(self, url: str, clock=time.monotonic):
        self.url = url
        self.clock = clock
        self.size: tuple[int, int] | None = None   # 보기 노드 상태에서 채운다
        self.session = 0                           # 영상 연결을 새로 할 때마다 1 오른다(프레임 번호 세션)
        self.frames = 0
        self._latest: tuple[bytes, float, float] | None = None   # (JPEG, 찍힌 때 = 받은 때 − 보기 노드에서 기다린 시간, wall)
        self._times: list[float] = []
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._map_lock = threading.Lock()
        self._map: tuple | None = None   # (받은 장 묶음, 지도용 장 묶음): 같은 장은 한 번만 키운다

    def map_snapshot(self) -> tuple[bytes, float, float] | None:
        """지도용 장과 그 장의 시각(GET /video.mjpg?mapping=1, jpeg_snapshot과 같은 모양: 서버가 그 시각의 센서 자세를 붙인다).

        지도 파트 실시간 지도는 Tello 원본 크기 960×720과 내장 보정값만 받는데, 자율 비행 드라이버는 실기체 영상을
        480×360으로 줄여 낸다(camera_width, 깊이·YOLO 부하). 화면 전체를 비율 그대로 줄인 것이라(자르지 않음) 다시
        960×720으로 키우면 보정값이 맞는다. 시뮬레이터처럼 이미 그 크기면 그대로."""
        snap = self.jpeg_snapshot()
        if snap is None or self.size == MAP_SIZE:
            return snap
        with self._map_lock:
            if self._map is None or self._map[0] is not snap:   # 같은 장은 한 번만 키운다
                self._map = (snap, (scale_jpeg(snap[0], MAP_SIZE), *snap[1:]))
            return self._map[1]

    def map_jpeg(self) -> bytes | None:
        """지도용 장만(센서 자세를 붙이지 않을 때)."""
        snap = self.map_snapshot()
        return None if snap is None else snap[0]

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive() and not self._stop.is_set()

    def start(self) -> None:
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, name="ros-view-video", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()

    def put(self, jpeg: bytes, age_s: float = 0.0) -> None:
        now = self.clock()
        with self._lock:
            self._latest = (jpeg, now - age_s, time.time() - age_s)
            self.frames += 1
            self._times = [t for t in self._times if now - t <= 2.0] + [now]

    def jpeg(self) -> bytes | None:
        with self._lock:
            return None if self._latest is None else self._latest[0]

    def jpeg_snapshot(self) -> tuple[bytes, float, float] | None:
        with self._lock:
            return self._latest

    def stats(self) -> dict:
        now = self.clock()
        with self._lock:
            latest, times = self._latest, list(self._times)
        fps = round((len(times) - 1) / (times[-1] - times[0]), 1) if len(times) >= 2 and times[-1] > times[0] else None
        return {"running": self.running, "fps": fps, "frame_age_s": None if latest is None else round(now - latest[1], 3),
                "frames": self.frames, "session": self.session}

    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                with urlopen(self.url + "/video.mjpg", timeout=STREAM_TIMEOUT_S) as r:
                    self.session += 1
                    self._read(r)
            except (OSError, ValueError):
                pass   # 보기 노드가 꺼졌거나 영상이 끊김: 잠시 뒤 다시 붙는다
            self._stop.wait(RECONNECT_S)

    def _read(self, r) -> None:
        while not self._stop.is_set():
            line = r.readline()
            if not line:
                return
            if not line.startswith(b"--"):
                continue
            headers = {}
            while True:
                h = r.readline()
                if h in (b"\r\n", b"\n", b""):
                    break
                k, _, v = h.decode("latin-1").partition(":")
                headers[k.strip().lower()] = v.strip()
            n = int(headers.get("content-length", "0"))
            data = r.read(n) if n > 0 else b""
            if len(data) < n:
                return
            if data:
                self.put(data, int(headers.get("x-frame-age-ms", "0")) / 1000)


class RosViewGateway:
    def __init__(self, url: str, *, log_root: str, clock=time.monotonic, sensor_pose: bool = False):
        self.url = url.rstrip("/")
        self.source = "ros"
        self.clock = clock
        self.listeners: list = []
        # sensor_pose: 지도용 센서 자세(실시간으로 도는 실기체·가짜 Tello만, SensorViewLink)
        self.link = (SensorViewLink if sensor_pose else ViewLink)(log_root, self.url)
        run_dir = self.link.log.dir
        self.video = ViewVideo(self.url, clock=clock)
        self.frames = FrameStore(self.link.run_id, clock=clock)
        self.detections = DetectionInbox(self.frames, run_dir=run_dir, emit=self._emit, clock=clock)
        self.overlay = OverlayStore(clock=clock)
        self.map = MapInbox(run_dir=run_dir, emit=self._emit, clock=clock)
        self.autonomy = AutonomyLink(run_dir=run_dir, clock=clock)   # 2단계에서 위치·경로·찾은 사람을 넣는다
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._tap: dict | None = None              # 보기 노드가 준 마지막 /api/state
        self._tap_mono: float | None = None
        self._tap_errors = 0
        self._sample = None                        # 마지막 Tello 상태(StateSample)
        self._state_key = None
        self._state_seq: int | None = None         # 보기 노드 상태 번호: 이 다음부터 받는다(B16). None이면 마지막 하나만
        self.states_lost = 0                       # 보기 노드가 모아 둔 것(약 13초)보다 오래 못 물어 빠진 상태 수
        self._state_mono = -math.inf               # 마지막 상태를 받은 때(게이트웨이 시계)
        self.states = 0
        self.mode, self.mode_reason = "disconnected", "ros_view_not_started"
        self.last_transition: dict | None = None
        self._mode_changes = 0
        self._video_session = None
        self._auto_open = False                    # 자율 창구를 보기 노드 자율 정보로 열었는가
        self._ground_origin: bool | None = None    # 자율 정보가 땅에서 들어왔는가(위치 원점 = 출발점)
        self._auto_counts: dict = {}               # 토픽별 받은 수(새 메시지만 넣으려고)
        self._path_version = None
        self._people_version = None
        self._people_coverage = None
        self._mission_state: str | None = None
        self._note: dict | None = None             # 비행 노드가 마지막으로 알린 지금 하는 일과 이유(정리한 것, 기록할 때를 고르려고)
        self._goal: dict | None = None
        self._goal_key = None                      # 찍은 자리 소식의 (seq, t): 바뀌면 비행 노드의 새 답
        self._goal_said: float | None = None       # 그 답을 처음 들은 때(보기 노드가 받은 때, 게이트웨이 시계)
        self._auto_land_before = None              # 이번 비행 전 드라이버의 마지막 자동 착륙 시각(새 자동 착륙을 가리려고)
        self._auto_land_init = False
        self._emergency_at: float | None = None    # 이 화면에서 비상 정지를 보낸 때(바로 뒤 땅으로 바뀐 까닭)
        self._air_since: float | None = None       # 이번에 뜬 때. 비행 시간은 게이트웨이가 본 만큼 센다
        self._air_total = 0.0                      # (시뮬레이터 Tello는 모터 시간을 늘 0으로 준다)
        self._start_pose: dict | None = None       # 출발 자리: 땅에 있는 동안의 마지막 자율 위치, 처음 뜰 때 굳힌다
        self._start_fixed = False                  # (시뮬레이터는 지도 좌표라 원점이 출발이 아니다)
        self._result_key: str | None = None        # 드라이버 last_result(마지막 이륙·착륙 결과)를 마지막으로 본 모양
        self._result_seen = False                  # 처음 본 결과는 게이트웨이를 켜기 전 것이라 알리지 않는다
        self._errors: dict[str, int] = {}          # 보기 한 바퀴·자율 창구가 거절한 수(종류별). 스레드는 멈추지 않는다
        self._grid: dict | None = None             # 자율 비행 지도 단면(보기 노드 /api/grid), 번호가 바뀔 때만 다시 받는다
        self._grid_ver = None
        self._grid_saved = -math.inf               # 회차 폴더에 마지막 지도를 남긴 때(몇 초에 한 번)
        self._grid_dirty = False                   # 받았는데 아직 회차 폴더에 안 남긴 지도가 있다
        self._found: dict | None = None            # 찾은 장애물(보기 노드 /api/discovered)
        self._found_ver = None
        self._world_map: dict | None = None        # 자율 비행 웹이 그리는 지도 파일(보기 노드 /api/map): 시뮬레이터 벽·가구·목적지, 실기체 빈 지도
        self._world_map_ver = None
        self._yaw_hist: deque = deque()            # 땅에 있는 동안 받은 Tello 기수 값 (받은 때, °). 뜨면 비운다
        self._height = HeightWatch()               # 아래가 가까운가·자율 비행 높이 추정이 잰 높이와 어긋나는가(알리기만)
        self._log_seq: int | None = None           # 보기 노드 경고·오류 번호: 이 다음부터 받는다(None이면 처음, 지난 것은 건너뜀)
        self._log_last: dict = {}                  # 같은 말(숫자 뺌)을 마지막으로 적은 때
        self._ground_at: float | None = None       # 마지막으로 내려앉은 때(그 뒤 10초까지의 경고도 적는다)
        self.commands = 0
        self.link.metrics = lambda: {"states": self.states, "frames": self.video.frames, "tap_errors": self._tap_errors,
                                     "mode_changes": self._mode_changes, "commands": self.commands,
                                     "flight_time_s": self._flight_time(), "errors": dict(self._errors),
                                     "states_lost": self.states_lost}

    # ---------- 수명 ----------
    def start(self) -> "RosViewGateway":
        self.video.start()
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, name="ros-view-gateway", daemon=True)
        self._thread.start()
        return self

    def close(self) -> None:
        self._stop.set()
        self.video.stop()
        if self._thread:
            self._thread.join(timeout=2)
        self._save_grid(force=True)   # 5초 묶음에 걸려 아직 안 남긴 마지막 지도

    def _emit(self, msg: dict) -> None:
        for fn in list(self.listeners):
            try:
                fn(msg)
            except Exception:
                pass

    def _loop(self) -> None:
        while not self._stop.is_set():
            t0 = time.monotonic()
            try:
                self.tick()
            except Exception as e:   # 이상한 값 하나에 스레드가 죽으면 모드·연결·버튼이 그 자리에 굳는다(영상만 계속 나온다)
                self._error("tick", e)
            self._stop.wait(max(0.0, POLL_S - (time.monotonic() - t0)))

    def _error(self, kind: str, e: Exception) -> None:
        """멈추지 않고 센다. 회차 기록에는 처음 몇 번과 그 뒤 100번마다만 남긴다."""
        n = self._errors[kind] = self._errors.get(kind, 0) + 1
        if n <= ERROR_LOG_FIRST or n % 100 == 0:
            self.link.event("ros_view_error", {"kind": kind, "count": n, "error": f"{type(e).__name__}: {e}"[:200]})

    def _receive(self, msg: dict) -> None:
        """자율 창구에 넣는다. 형식이 틀린 것 하나는 버리고 다음 것을 계속 넣는다."""
        try:
            self.autonomy.receive(msg)
        except ValueError as e:
            self._error("autonomy_" + str(msg.get("type")), e)

    # ---------- 보기 노드 ----------
    def tick(self) -> None:
        """보기 노드 상태를 한 번 묻고 모드를 다시 정한다. 시험에서는 직접 부른다."""
        params = ([] if self._state_seq is None else [f"states_after={self._state_seq}"]) + (
            [] if self._log_seq is None else [f"log_after={self._log_seq}"])
        query = "?" + "&".join(params) if params else ""
        try:
            with urlopen(self.url + "/api/state" + query, timeout=POLL_TIMEOUT_S) as r:
                tap = json.loads(r.read())
            if not isinstance(tap, dict):
                raise ValueError("not a dict")
        except (OSError, ValueError) as e:
            self._tap_errors += 1
            if self._tap_mono is not None and self.clock() - self._tap_mono > TAP_STALE_S and self._tap is not None:
                self._tap = None
                self.link.event("ros_view_lost", {"error": type(e).__name__})
            if self._tap is None:
                self._autonomy_tick(None)
            self._update_mode()
            return
        now = self.clock()
        if self._tap is None:
            self.link.event("ros_view_connected", {"url": self.url, "topics": sorted((tap.get("topics") or {}))})
        self._tap, self._tap_mono = tap, now
        video = tap.get("video") or {}
        if isinstance(video.get("width"), int) and isinstance(video.get("height"), int):
            self.video.size = (video["width"], video["height"])
        st = tap.get("state") or {}
        seq = st.get("seq") if type(st.get("seq")) is int else None
        states = tap.get("states")
        if isinstance(states, list) and seq is not None and self._state_seq is not None and seq >= self._state_seq:
            # 놓친 것까지 번호 순서대로(보기 노드가 받은 때 그대로). 한 바퀴가 늦어도 지도용 센서 자세가 끊기지 않는다
            if type(tap.get("states_lost")) is int:
                self.states_lost += tap["states_lost"]
            for s in sorted((s for s in states if isinstance(s, dict) and type(s.get("seq")) is int and s["seq"] > self._state_seq),
                            key=lambda s: s["seq"]):
                self._take_state(s.get("raw"), s.get("ros_stamp"), s.get("age_s"), now)
            self._state_seq = seq
        else:   # 처음이거나 번호 없는 옛 보기 노드·보기 노드를 다시 켬(번호가 처음부터): 마지막 상태 하나만
            self._take_state(st.get("raw"), st.get("ros_stamp"), st.get("age_s"), now)
            self._state_seq = seq
        self._update_mode()
        self._driver_result(tap)
        self._take_logs(tap, now)
        self._autonomy_tick(tap)

    def _take_state(self, raw, ros_stamp, age_s, now: float) -> None:
        """Tello 상태 하나를 받는다(같은 것은 한 번만). 받은 때 = 보기 노드가 받은 때를 게이트웨이 시계로."""
        key = (ros_stamp, raw)
        if not isinstance(raw, str) or key == self._state_key:
            return
        self._state_key = key
        age = float(age_s) if isinstance(age_s, (int, float)) and math.isfinite(age_s) and age_s >= 0 else 0.0
        # 받은 순서대로 둔다: Windows 시계 단위(15.6 ms)와 HTTP 지연으로 붙어 온 두 상태의 앞뒤가 바뀌지 않게
        mono = self._state_mono = max(now - age, self._state_mono + 0.001)
        sample = parse_state(raw, received=time.time() - age, received_mono=mono)
        self._sample = sample
        self.link.note_state(sample)   # 지도용 센서 자세(받은 때 기준, 영상 장 시각과 같은 시계)
        self._note_yaw(sample, mono)
        self._watch_height(sample, mono)
        self.states += 1
        self.link.log.append("sensor", {**sample.to_dict(), "time_source": "ros_view", "sequence": self.states})

    def _note_yaw(self, sample, mono: float) -> None:
        """땅에 있는 동안의 Tello 기수 값을 모은다. 자율 비행 웹처럼 드라이버 상태가 땅(떠 있지도, 이륙·착륙 중도 아님)일 때만,
        뜨거나 드라이버 상태가 끊기면 비운다. 받은 때는 보기 노드가 받은 때(게이트웨이 시계)."""
        fs = self._flight()
        if fs is None or fs.get("flying") or fs.get("busy"):
            self._yaw_hist.clear()
            return
        yaw = sample.raw.get("yaw")
        if isinstance(yaw, (int, float)) and not isinstance(yaw, bool) and math.isfinite(yaw):
            self._yaw_hist.append((mono, float(yaw)))
        while self._yaw_hist and self._yaw_hist[0][0] < mono - YAW_SETTLE_WINDOW_S - 2.0:
            self._yaw_hist.popleft()

    def _watch_height(self, sample, mono: float) -> None:
        """자율 비행 중 아래가 가까운지·추정 높이가 잰 높이와 어긋나는지(height_watch.py). 비행에 끼어들지 않고 알리기만 한다.
        11/17 실기체 충돌: 비행 노드 높이 추정이 0.4~0.9 m 높게 굳어 바닥 0.4 m까지 내려갔는데 화면에 아무 알림이 없었다."""
        fs = self._flight()
        flying = (self.mode == "auto" and fs is not None and bool(fs.get("flying")) and not fs.get("busy")
                  and self._mission_state not in ("TAKEOFF", "LAND"))
        si = sample.si or {}
        pose = (self._tap or {}).get("pose")
        est = pose.get("z_m") if isinstance(pose, dict) and (pose.get("age_s") or 0) <= 1.0 else None
        for detail in self._height.update(mono, flying, si.get("tof_m") if si.get("tof_valid") else None,
                                          si.get("tof_reason") == "below_min_or_on_ground", sample.raw.get("baro"), est):
            self.link.event("height_watch", detail)
            self._emit({"schema_version": SCHEMA_VERSION, "type": "event", "at": now_iso(), "kind": "height_watch",
                        "detail": detail})

    def _yaw_settle(self) -> dict:
        """땅에서 기수 값이 흐르는가(drone_state.ros_view.start에 싣는다). 모르면(땅이 아니거나 3초도 안 쌓임) 폭 None."""
        span = yaw_span(self._yaw_hist, max(self.clock(), self._yaw_hist[-1][0])) if self._yaw_hist else None
        return {"yaw_span_deg": None if span is None else round(span, 1),
                "yaw_drifting": span is not None and span >= YAW_SETTLE_DEG}

    def _driver_result(self, tap: dict) -> None:
        """드라이버의 마지막 이륙·착륙 결과(flight_status.last_result)가 새로 실패면 화면에 알린다.
        11/16 실기체: 첫 takeoff에 Tello가 바로 error로 답했는데 화면에 안 보였다. 자율 비행은 20초 뒤 다시 요청해 떴지만
        그동안 운용자는 까닭을 모른 채 주변 확인을 껐다 켰다. 게이트웨이 조종권 관리자가 이륙 거절을 알리는 것과 같은
        command_rejected로 낸다(by: autonomy_driver, 드론 응답 그대로)."""
        fs = tap.get("flight_status")
        if not isinstance(fs, dict):
            return
        res = fs.get("last_result")
        key = json.dumps(res, sort_keys=True, ensure_ascii=False, default=str) if isinstance(res, dict) else None
        if not self._result_seen:   # 처음 본 드라이버 상태: 그 결과는 지난 것(결과가 없으면 다음 것부터 새것)
            self._result_key, self._result_seen = key, True
            return
        if key is None or key == self._result_key:
            return
        self._result_key = key
        if res.get("ok") or res.get("cmd") not in ("takeoff", "land"):
            return
        inner = res.get("result") if isinstance(res.get("result"), dict) else {}
        failed = res.get("failed") if isinstance(res.get("failed"), list) else []
        detail = {"name": res["cmd"], "reason": str(res.get("reason") or inner.get("reason") or "failed"),
                  "by": "autonomy_driver", "response": inner.get("response"), "failed": [str(k) for k in failed] or None}
        self.link.event("driver_result", detail)
        self._emit({"schema_version": SCHEMA_VERSION, "type": "event", "at": now_iso(), "kind": "command_rejected",
                    "detail": detail})

    def _take_logs(self, tap: dict, now: float) -> None:
        """자율 비행 노드 경고·오류(보기 노드가 /rosout에서 모은 것)를 비행 기록에 남긴다(kind autonomy_log).
        처음 붙을 때 이미 있던 것은 지난 일이라 건너뛴다. 떠 있을 때(내린 뒤 10초까지)의 경고와 늘 오류만,
        같은 노드의 같은 말(숫자는 빼고 견줌)은 30초에 한 번까지."""
        seq = (tap.get("log") or {}).get("seq")
        if type(seq) is not int:
            return
        if self._log_seq is None or seq < self._log_seq:   # 처음이거나 보기 노드를 다시 켬
            self._log_seq = seq
            return
        air = self.mode in AIR_MODES or (self._ground_at is not None and now - self._ground_at <= LOG_AFTER_LANDING_S)
        rows = [x for x in tap.get("logs") or [] if isinstance(x, dict) and type(x.get("seq")) is int and x["seq"] > self._log_seq]
        for x in sorted(rows, key=lambda x: x["seq"]):
            text, node, level = _text(x.get("text")), _text(x.get("node"), 60) or "?", x.get("level")
            if text is None or type(level) is not int or (level < LOG_ERROR and not air):
                continue
            key = (node, re.sub(r"\d+(?:\.\d+)?", "#", text))
            if now - self._log_last.get(key, -math.inf) < LOG_REPEAT_S:
                continue
            self._log_last[key] = now
            detail = {"level": "error" if level >= LOG_ERROR else "warn", "node": node, "text": text}
            self.link.event("autonomy_log", detail)
            self._emit({"schema_version": SCHEMA_VERSION, "type": "event", "at": now_iso(), "kind": "autonomy_log",
                        "detail": detail})
        self._log_seq = seq

    # ---------- 자율 정보 (2단계) ----------
    def _new(self, tap: dict, topic: str) -> bool:
        """보기 노드가 그 토픽을 새로 받았는가(받은 수가 늘었나)."""
        count = ((tap.get("topics") or {}).get(topic) or {}).get("count")
        if count is None or count == self._auto_counts.get(topic):
            return False
        self._auto_counts[topic] = count
        return True

    def _get(self, path: str):
        try:
            with urlopen(self.url + path, timeout=POLL_TIMEOUT_S) as r:
                return json.loads(r.read())
        except (OSError, ValueError):
            return None

    def _autonomy_tick(self, tap: dict | None) -> None:
        """보기 노드의 자율 정보를 자율 창구(AutonomyLink)에 넣는다. 위치·임무 단계가 오면 연결로 열고, 오래 안 오면 닫는다."""
        pose, mission = (tap or {}).get("pose"), (tap or {}).get("mission")
        fresh = [x for x in (pose, mission) if isinstance(x, dict) and (x.get("age_s") or 0) <= AUTONOMY_STALE_S]
        if not fresh:
            if self._auto_open:
                self._save_grid(force=True)   # 비행 노드가 꺼졌다: 그 지도의 마지막 판을 남긴다
                self.autonomy.close(AUTONOMY_SESSION)
                self._auto_open, self._mission_state, self._goal = False, None, None
                self.link.event("ros_view_autonomy", {"connected": False})
            return
        if not self._auto_open:
            self.autonomy.open(AUTONOMY_SESSION)   # 새 연결은 새 지도: 지난 위치·경로·찾은 사람은 화면에서 비운다
            self._receive({"schema_version": SCHEMA_VERSION, "type": "hello", "node": "자율 비행 보기 노드",
                           "version": "ros-view"})
            self._auto_open, self._auto_counts = True, {}
            self._path_version = self._people_version = self._note = None
            self._grid = self._grid_ver = self._found = self._found_ver = None   # 새 지도
            self._world_map = self._world_map_ver = None
            self._grid_dirty = False
            self._start_pose, self._start_fixed = None, False   # 새 지도: 출발도 새로
            self._ground_origin = self.mode in ("grounded", "disconnected")
            self.link.event("ros_view_autonomy", {"connected": True, "ground_origin": self._ground_origin})
        if isinstance(pose, dict) and self._new(tap, "/tt/pose_est"):
            self._receive({"schema_version": SCHEMA_VERSION, "type": "pose", "frame": "map",
                           **{k: pose.get(k) for k in ("x_m", "y_m", "z_m", "yaw_rad", "sigma_m")}})
            xy = [pose.get(k) for k in ("x_m", "y_m")]
            if self.mode == "grounded" and not self._start_fixed and all(
                    isinstance(v, (int, float)) and math.isfinite(v) for v in xy):
                self._start_pose = {"x_m": round(float(xy[0]), 3), "y_m": round(float(xy[1]), 3)}
        path = tap.get("path")
        if isinstance(path, dict) and path.get("version") != self._path_version:
            full = self._get("/api/path")
            if isinstance(full, dict) and isinstance(full.get("points"), list):
                self._receive({"schema_version": SCHEMA_VERSION, "type": "path", "frame": "map",
                               "points": clean_path(full["points"])})
                self._path_version = path.get("version")
        people = tap.get("people")
        if (people or {}).get("version") != self._people_version:
            full = self._get("/api/people") if people else {"people": [], "unconfirmed": []}   # 새 회차에 아직 없으면 비운다
            if isinstance(full, dict):
                found, unsure = clean_people(full.get("people") or [], full.get("unconfirmed") or [])
                self._receive({"schema_version": SCHEMA_VERSION, "type": "people", "frame": "map",
                               **{k: full.get(k) for k in ("coverage", "explored_m2")},
                               "people": found, "unconfirmed": unsure})
                self._people_coverage = full.get("coverage")
                self._people_version = (people or {}).get("version")
        if isinstance(mission, dict) and mission.get("state"):
            state = str(mission["state"])
            if state != self._mission_state:
                self._mission_state = state
                self._receive({"schema_version": SCHEMA_VERSION, "type": "mission_state", "state": state,
                               "phase": mission_phase(state)})   # 바뀔 때만 기록(autonomy/events.jsonl)
        goal = tap.get("goal")
        if isinstance(goal, dict):
            self._take_goal(goal)
        # 지도에 그릴 찍은 자리: 웹처럼 받아 기다리거나 가는 중·도착해 둘러보는 중이고, 3초 안에 다시 알린 것만(비행 노드 꺼짐)
        self._goal = goal if (isinstance(goal, dict) and goal.get("state") in GOAL_ACTIVE
                              and (goal.get("age_s") or 0) < GOAL_FRESH_S) else None
        note = tap.get("note")
        if isinstance(note, dict) and self._new(tap, "/tt/mission_note"):
            self._take_note(note)
        self._grid_tick(tap)

    def _take_goal(self, goal: dict) -> None:
        """찍은 자리 소식(/tt/operator_goal). 가는 중에는 1초마다 같은 것이 다시 오므로 (seq, t)가 바뀐 것만 비행 노드의 새 답으로 본다
        (자율 비행 웹과 같음). 처음 들은 때는 보기 노드가 받은 때. 그 말(받음·거절·그만둠·도착)은 비행 기록에 적는다.
        게이트웨이를 켜기 전에 온 답(3초 넘음)은 적지 않는다."""
        key = (goal.get("seq"), goal.get("t"))
        if key == self._goal_key:
            return
        age = goal.get("age_s")
        age = float(age) if isinstance(age, (int, float)) and math.isfinite(age) and age >= 0 else 0.0
        self._goal_key, self._goal_said = key, self.clock() - age
        text = _text(goal.get("text"))
        if text and age < GOAL_FRESH_S:
            detail = {"text": text, "warn": goal.get("warn") is True, "state": _text(goal.get("state"), 20)}
            self.link.event("goal_note", detail)
            self._emit({"schema_version": SCHEMA_VERSION, "type": "event", "at": now_iso(), "kind": "goal_note", "detail": detail})

    def _goal_note(self) -> dict | None:
        """drone_state.mission.goal_note: 찍은 자리에 대한 비행 노드의 마지막 답과 처음 들은 지 몇 초.
        화면은 가는 자리가 없을 때 [여기로 보내기] 옆에 20초 보인다(자율 비행 웹 operator_note와 같음)."""
        goal = (self._tap or {}).get("goal")
        text = _text(goal.get("text")) if isinstance(goal, dict) else None
        if not text or self._goal_said is None:
            return None
        return {"text": text, "warn": goal.get("warn") is True, "age_s": round(self.clock() - self._goal_said, 1)}

    def _take_note(self, raw: dict) -> None:
        """비행 노드가 새로 알린 지금 하는 일과 이유. 돌아가기 시작한 이유·내리는 이유가 바뀌면 비행 기록에 적는다(화면·회차 기록)."""
        note = clean_note(raw)
        events, self._note = note_events(self._note, note), note
        for detail in events:
            self.link.event("mission_note", detail)
            self._emit({"schema_version": SCHEMA_VERSION, "type": "event", "at": now_iso(), "kind": "mission_note",
                        "detail": detail})

    def _note_view(self) -> dict | None:
        """drone_state.mission.note: 자율 비행 웹 status note와 같은 규칙. 3초 넘게 안 오면(비행 노드 꺼짐) 내린 이유만(stale)."""
        raw = (self._tap or {}).get("note")
        if not isinstance(raw, dict):
            return None
        note, age = clean_note(raw), raw.get("age_s")
        if isinstance(age, (int, float)) and age < NOTE_FRESH_S:
            return note
        return {"state": note["state"], "why_end": note["why_end"], "stale": True}

    def _grid_tick(self, tap: dict) -> None:
        """자율 비행이 만드는 지도(비행 높이 단면)와 찾은 장애물: 번호가 바뀔 때만 보기 노드에서 받아 둔다(칸 전체는 상태에 안 싣는다)."""
        grid = tap.get("grid")
        if isinstance(grid, dict) and grid.get("ver") != self._grid_ver:
            full = self._get("/api/grid")
            if (isinstance(full, dict) and isinstance(full.get("cells"), str)
                    and all(type(full.get(k)) is int and full[k] > 0 for k in ("w", "h"))
                    and all(isinstance(full.get(k), (int, float)) and math.isfinite(full[k]) for k in ("res", "x0", "y0"))):
                self._grid, self._grid_ver = {**full, "frame": "map"}, grid.get("ver")
                self._grid_dirty = True
        self._save_grid()   # 5초에 한 번까지. 그 사이에 온 마지막 판은 다음 바퀴에 남는다(새 판이 안 와도)
        found = tap.get("discovered")
        if (found or {}).get("version") != self._found_ver:
            full = self._get("/api/discovered") if found else None   # 새 회차에 아직 없으면 비운다
            if found is None or isinstance(full, dict):
                obstacles = [o for o in (full or {}).get("obstacles") or []
                             if isinstance(o, dict) and all(isinstance(o.get(k), (int, float)) and math.isfinite(o[k])
                                                            for k in ("x", "y", "sx", "sy"))]
                self._found = None if found is None else {"obstacles": [found_item(o) for o in obstacles], "frame": "map"}
                self._found_ver = (found or {}).get("version")
                if self._found is not None:
                    self._write_json("discovered.json", {**self._found, "saved_at": now_iso()}, "found_save")
        world = tap.get("world_map")   # 자율 비행 웹이 그리는 지도 파일: 번호가 바뀔 때만(보통 켤 때 한 번)
        if (world or {}).get("version") != self._world_map_ver:
            full = self._get("/api/map") if world else None
            if world is None or isinstance(full, dict):
                self._world_map = None if world is None else world_map_item(full)
                self._world_map_ver = (world or {}).get("version")
                if self._world_map is not None:
                    self._write_json("map.json", {**self._world_map, "saved_at": now_iso()}, "map_save")

    def _save_grid(self, force: bool = False) -> None:
        """회차 폴더 autonomy/grid.json에 마지막 지도를 남긴다(탐색 결과·기록 재생용). 5초에 한 번까지 쓰고, 그 사이에 온 마지막 판은
        다음 바퀴(5초 뒤)에, 자율 연결이 닫히거나 게이트웨이를 끌 때는 바로(force) 쓴다.
        11/17 실기체: 5초 안에 온 마지막 판을 안 남겨 회차 지도에서 끝 7초(닿은 자리 둘레)가 빠졌다."""
        now = self.clock()
        if not self._grid_dirty or self._grid is None or (not force and now - self._grid_saved < GRID_SAVE_S):
            return
        self._grid_saved, self._grid_dirty = now, False
        self._write_json("grid.json", {**self._grid, "saved_at": now_iso()}, "grid_save")

    def _write_json(self, name: str, data: dict, kind: str) -> None:
        """회차 autonomy 폴더에 통째로 쓴다(반쯤 쓴 파일이 남지 않게 임시 파일 뒤 바꿔 넣기)."""
        if not self.autonomy.run_dir:
            return
        path = os.path.join(self.autonomy.run_dir, name)
        try:
            with open(path + ".tmp", "w", encoding="utf-8") as f:
                json.dump(data, f, ensure_ascii=False)
            os.replace(path + ".tmp", path)
        except OSError as e:
            self._error(kind, e)

    def autonomy_grid(self) -> dict | None:
        """GET /api/autonomy/grid: 자율 비행 지도 단면(자율 비행 웹 /api/grid와 같은 모양 + frame)."""
        return self._grid

    def autonomy_world_map(self) -> dict | None:
        """GET /api/autonomy/map: 자율 비행 웹이 그리는 지도 파일(시뮬레이터 세계 지도의 벽·가구·목적지·출발, 실기체 빈 지도)."""
        return self._world_map

    def _truth_view(self) -> dict | None:
        """시뮬레이션 정답 위치(보기 노드 truth:=true, 자율 비행 웹과 같은 /tt/odometry_gt). 2초 넘게 안 오면 없음."""
        t = (self._tap or {}).get("truth")
        if not isinstance(t, dict) or _num(t.get("age_s")) is None or t["age_s"] > 2:
            return None
        if _num(t.get("x_m")) is None or _num(t.get("y_m")) is None:
            return None
        return {"x_m": _num(t["x_m"]), "y_m": _num(t["y_m"]), "yaw_rad": _num(t.get("yaw_rad"))}

    def autonomy_discovered(self) -> dict | None:
        """GET /api/autonomy/discovered: 비행 노드가 찾은 장애물 {obstacles: [{x, y, sx, sy, sources?}], frame}.
        sources는 비행 노드가 붙인 출처 이름 그대로(yolo:chair·contact·tof 등, 화면이 이름표로 바꾼다)."""
        return self._found

    def _mission(self) -> dict | None:
        """drone_state.mission: 비행 노드 단계 그대로(state)와 화면 phase. 진행은 탐색 넓이(coverage)가 있으면 그것."""
        if not self._auto_open or not self._mission_state:
            return None
        goal = self._goal
        cov = self._people_coverage
        return {"state": self._mission_state, "phase": mission_phase(self._mission_state),
                "goal": None if goal is None else {"state": goal.get("state"), "text": goal.get("text"), "goal": goal.get("goal"),
                                                   "click": goal.get("click")},
                "progress": round(float(cov), 3) if isinstance(cov, (int, float)) and math.isfinite(cov) else None,
                "reason": None, "note": self._note_view(), "goal_note": self._goal_note()}

    def _flight(self) -> dict | None:
        """자율 드라이버 상태. 오래됐으면 None."""
        fs = (self._tap or {}).get("flight_status")
        if not isinstance(fs, dict) or (fs.get("age_s") or 0) > FLIGHT_STALE_S:
            return None
        return fs

    def _derive_mode(self) -> tuple[str, str]:
        now = self.clock()
        if self._tap is None or self._tap_mono is None or now - self._tap_mono > TAP_STALE_S:
            return "disconnected", "ros_view_unreachable"
        fs = self._flight()
        if fs is None:
            return "disconnected", "autonomy_driver_silent"
        if fs.get("connected") is False:
            return "disconnected", "drone_not_connected"
        sample = self._sample
        if sample is None or now - sample.received_mono > STATE_STALE_S:
            return "disconnected", "state_lost"
        if not self._auto_land_init:   # 처음 본 드라이버 상태의 자동 착륙은 지난 비행 것이다
            self._auto_land_before, self._auto_land_init = self._auto_land_at(fs), True
        if fs.get("busy") == "land":
            return "landing", self._landing_reason(fs)
        if fs.get("flying"):
            return "auto", "autonomy_flying"
        if self.mode in ("auto", "landing"):   # 막 내려옴: 드라이버가 스스로 내렸으면 그 까닭(뒤집힘은 착륙 없이 모터를 끈다)
            if self._emergency_at is not None and now - self._emergency_at < EMERGENCY_REASON_S:
                return "grounded", "emergency"   # 이 화면에서 보낸 비상 정지로 모터가 꺼졌다
            reason = self._landing_reason(fs)
            return "grounded", "autonomy_ground" if reason == "autonomy_landing" else reason
        self._auto_land_before = self._auto_land_at(fs)   # 땅: 다음 비행에서 새 자동 착륙을 가린다
        return "grounded", "autonomy_ground"

    @staticmethod
    def _auto_land_at(fs: dict):
        auto = fs.get("last_auto_land")
        return auto.get("at") if isinstance(auto, dict) else None

    def _landing_reason(self, fs: dict) -> str:
        """착륙 까닭: 드라이버가 스스로 내리면 그 까닭(cmd_timeout·battery_low·state_lost·crash·flipped·grounded 등).
        드라이버 landing_reason(속도 명령 끊김)이 먼저, 다음은 이번 비행에 새로 생긴 FlightController 자동 착륙. 아니면 자율 비행이 내림."""
        if fs.get("landing_reason"):
            return str(fs["landing_reason"])
        auto = fs.get("last_auto_land")
        if isinstance(auto, dict) and auto.get("reason") and auto.get("at") != self._auto_land_before:
            return str(auto["reason"])
        return "autonomy_landing"

    def _update_mode(self) -> None:
        mode, reason = self._derive_mode()
        with self._lock:
            if mode == self.mode:
                self.mode_reason = reason
                return
            self.last_transition = {"from": self.mode, "to": mode, "by": "ros_view", "reason": reason}
            now = self.clock()
            if mode in AIR_MODES and self.mode not in AIR_MODES:
                self._air_since = now
                self._start_fixed = self._start_fixed or self._start_pose is not None   # 처음 뜬 자리를 출발로 굳힌다
            elif mode not in AIR_MODES and self.mode in AIR_MODES and self._air_since is not None:
                self._air_total += now - self._air_since
                self._air_since = None
                self._ground_at = now
            self.mode, self.mode_reason = mode, reason
            self._mode_changes += 1
            transition = dict(self.last_transition)
        self.link.event("gateway_mode_changed", transition)   # 게이트웨이 회차와 같은 이름(기록 재생이 읽는다)
        self._emit({"schema_version": SCHEMA_VERSION, "type": "event", "at": now_iso(), "kind": "mode_changed",
                    "detail": transition})

    def _flight_time(self) -> float:
        """이번 회차에 게이트웨이가 본 비행 시간(뜬 뒤 땅으로 바뀔 때까지 모은 초). 보기가 끊긴 사이는 세지 않는다."""
        air = self._air_total + (self.clock() - self._air_since if self._air_since is not None else 0.0)
        return round(air, 1)

    # ---------- 화면 명령 (3단계) ----------
    def _goal_active(self) -> bool:
        goal = (self._tap or {}).get("goal")
        return isinstance(goal, dict) and goal.get("state") in GOAL_ACTIVE and (goal.get("age_s") or 0) < GOAL_FRESH_S

    def _start_info(self) -> dict:
        """시작(이륙)을 열 수 있는가와 못 여는 까닭. 자율 비행 웹과 같다:
        임무 READY + 드라이버 can_takeoff(드라이버 상태가 없으면 통과) + YOLO 준비(need_yolo일 때). can_takeoff에는 주변 확인도 들었다."""
        tap = self._tap or {}
        fs = tap.get("flight_status") if isinstance(tap.get("flight_status"), dict) else None   # 웹 s.flight: 마지막 드라이버 상태
        mission = (tap.get("mission") or {}).get("state")
        yolo_ok = not tap.get("need_yolo") or bool((tap.get("yolo") or {}).get("ready"))
        can = fs is None or bool(fs.get("can_takeoff"))
        failed = [c.get("key") for c in (fs or {}).get("preflight") or []
                  if isinstance(c, dict) and not c.get("ok") and c.get("blocking", True)]
        return {"ok": mission == "READY" and can and yolo_ok, "mission": mission, "can_takeoff": can, "failed": failed,
                "yolo": yolo_ok}

    def _allowed(self) -> list[str]:
        """지금 열 화면 버튼. 자율 비행 웹(renderControl·renderGoto)이 버튼을 켜는 조건과 같다."""
        tap = self._tap
        if tap is None or self.mode_reason == "ros_view_unreachable" or not (tap.get("commands") or {}).get("enabled"):
            return []   # 보기 노드가 끊겼거나 명령을 받지 않게 켜졌다(commands:=false)
        fs_any = tap.get("flight_status") if isinstance(tap.get("flight_status"), dict) else None
        fs = self._flight()   # 웹 driver: 2초 안의 드라이버 상태
        mission = (tap.get("mission") or {}).get("state")
        names = ["emergency"]   # 웹: Esc는 늘
        if fs is not None and not (fs.get("flying") or fs.get("busy")):
            names.append("confirm_area")
        if self._start_info()["ok"]:
            names.append("takeoff")
        if (fs_any and (fs_any.get("flying") or fs_any.get("busy"))) or mission in AIRBORNE:
            names.append("land")
        if mission in GOTO_STATES:
            names.append("goto")
        if self._goal_active():
            names.append("goto_cancel")
        return names

    def _command(self, name: str, args: dict) -> tuple[bool, str | None, list | None]:
        path = ROS_CMDS.get(name)
        if path is None:
            return False, "ros_view_not_available", None
        if self._tap is None or self.mode_reason == "ros_view_unreachable":
            return False, "ros_view_unreachable", None
        if not (self._tap.get("commands") or {}).get("enabled"):
            return False, "commands_disabled", None
        if name not in self._allowed():
            return False, "not_allowed_in_mode", None
        if name == "confirm_area":
            if type(args.get("confirmed")) is not bool:
                return False, "invalid_command_args", None
            body = {"confirmed": args["confirmed"]}
        elif name == "goto":
            if goto_command(args) is None:
                return False, "bad_xy", None
            body = {"x": args["x"], "y": args["y"]}
        else:
            body = {}
        req = Request(self.url + "/api/" + path, data=json.dumps(body).encode("utf-8"), method="POST",
                      headers={"Content-Type": "application/json", CLIENT_HEADER: "1"})
        try:
            with urlopen(req, timeout=CMD_TIMEOUT_S) as r:
                res = json.loads(r.read())
        except HTTPError as e:
            try:
                res = json.loads(e.read())
            except ValueError:
                res = {}
            return False, str(res.get("reason") or f"http_{e.code}"), None
        except (OSError, ValueError):
            return False, "ros_view_unreachable", None
        return bool(res.get("ok")), res.get("reason"), res.get("sent")

    def handle_message(self, msg: dict, client_key: str = "") -> dict | None:
        if msg.get("type") != "cmd":
            return None   # rc: 자율 비행 보기에는 사람 rc 길이 없다(자율 비행 웹과 같음)
        name = str(msg.get("name", ""))
        args = msg.get("args") if isinstance(msg.get("args"), dict) else {}
        client = msg.get("client") if msg.get("client") in ("monitor", "control") else "monitor"
        ok, reason, sent = self._command(name, args)
        if ok:
            self.commands += 1
            if name == "emergency":
                self._emergency_at = self.clock()
        # 시작을 보낼 때 기수 값이 흐르고 있었는지도 남긴다(비행 노드가 받지 않았거나 드론이 거절한 까닭을 나중에 보려고)
        yaw = self._yaw_settle() if name == "takeoff" else {}
        self.link.event("gateway_cmd", {"name": name, "client": client, "ok": ok, "reason": reason, "sent": sent, **yaw})
        return {"schema_version": SCHEMA_VERSION, "type": "result", "id": msg.get("id"), "ok": ok, "reason": reason,
                "mode": self.mode}

    def client_closed(self, client_key: str) -> None:
        pass

    def autonomy_open(self, session: str, send, peer: str | None = None) -> bool:
        return False   # 자율은 보기 노드로 본다(서버가 busy로 닫는다)

    def autonomy_message(self, session: str, msg) -> None:
        return None

    def autonomy_closed(self, session: str) -> None:
        pass

    def _brief(self) -> dict:
        """탐지 사진에 붙이는 그때 드론 상태. 위치는 자율 위치가 새 값일 때만(실시간 게이트웨이 _drone_brief와 같은 규칙)."""
        si = (self._sample.si if self._sample is not None else None) or {}
        pose = self.autonomy.pose_state()
        if pose is not None and pose.get("valid"):
            pose = {"x_m": round(pose["x_m"], 3), "y_m": round(pose["y_m"], 3), "yaw_rad": round(pose["yaw_rad"], 3),
                    "sigma_m": pose.get("sigma_m"), "frame": pose.get("frame", "map")}
        else:
            pose = None
        return {"mode": self.mode, "height_m": si.get("tof_m") if si.get("tof_valid") else None,
                "yaw_rad": si.get("yaw_rad"), "battery_pct": si.get("battery_pct"), "pose": pose}

    def video_frame(self):
        snap = self.video.jpeg_snapshot()
        if snap is None:
            return None, "no_frame"
        jpeg, stamp, wall = snap
        if self.clock() - stamp > VIDEO_STALE_S:
            return None, "video_stale"
        if self.video.session != self._video_session:   # 영상 연결을 새로 했으면 새 영상 세션(탐지 결과 연결용)
            self._video_session = self.video.session
            self.frames.new_session()
        frame = self.frames.add(jpeg, stamp, wall, self.video.size, self._brief())
        return (frame, None) if frame is not None else (None, "no_frame")

    def _front_tof(self) -> dict:
        tof = (self._tap or {}).get("front_tof")
        if not isinstance(tof, dict):
            return {"m": None, "valid": False, "reason": "not_received", "age_s": None}
        age = tof.get("age_s")
        if isinstance(age, (int, float)) and age > FRONT_TOF_STALE_S:
            return {"m": None, "valid": False, "reason": "stale", "age_s": age}
        m = tof.get("m")
        if isinstance(m, (int, float)) and math.isfinite(m):
            return {"m": round(float(m), 3), "valid": True, "reason": None, "age_s": age}
        return {"m": None, "valid": False, "reason": "out_of_range" if tof.get("inf") else "no_value", "age_s": age}

    def drone_state(self) -> dict:
        now = self.clock()
        sample = self._sample
        si = (sample.si if sample is not None else None) or {}
        state_age = None if sample is None else round(now - sample.received_mono, 3)
        mode = self.mode
        fs = self._flight() or {}
        vstats = self.video.stats()
        video_on = vstats["frame_age_s"] is not None and vstats["frame_age_s"] < VIDEO_STALE_S
        warnings = []
        bat, temp = si.get("battery_pct"), si.get("temp_high_c")
        if isinstance(bat, (int, float)) and bat < BATTERY_WARN_PCT:
            warnings.append("battery_low")
        if isinstance(temp, (int, float)) and temp >= TEMP_WARN_C:
            warnings.append("temp_high")
        if mode in ("auto", "landing") and not si.get("tof_valid"):
            warnings.append("height_unknown")
        height = self._height.state()
        if height["low"]:
            warnings.append("low_altitude")      # 아래가 가깝다(하방 0.5 m 안, 2초)
        if height["gap"]:
            warnings.append("height_mismatch")   # 자율 비행 높이 추정이 잰 높이와 0.4 m 넘게 어긋남(3초)
        down_valid = bool(si.get("tof_valid"))
        tap = self._tap or {}
        return {
            "schema_version": SCHEMA_VERSION, "type": "drone_state", "run_id": self.link.run_id, "at": now_iso(),
            "time_source": "gateway_receive", "source": "ros", "mapping_console_url": None,
            "connected": mode != "disconnected", "state_age_s": state_age, "mode": mode,
            "flight_busy": fs.get("busy") if fs.get("busy") in ("takeoff", "land") else None,
            "authority": "autonomy" if mode in ("auto", "landing") else "none", "allowed": self._allowed(),
            "warnings": warnings, "battery_pct": bat, "height_m": si.get("tof_m") if down_valid else None,
            "height_source": "down_tof", "temp_high_c": temp,
            "attitude": {k: si.get(k) for k in ("roll_rad", "pitch_rad", "yaw_rad")},
            "front_tof": self._front_tof(),
            "height_watch": height,   # {low: {tof_m}, gap: {gap_m, est_m, baro_m, tof_m}, baro_ref_m}(켜진 것만, height_watch.py)
            "down_tof": {"m": si.get("tof_m"), "valid": down_valid, "reason": si.get("tof_reason")},
            "pose": self.autonomy.pose_state(), "mission": self._mission(),
            "autonomy": {**self.autonomy.status(), "ground_origin": self._ground_origin if self._auto_open else None,
                         "mission_active": (self._mission() or {}).get("phase") == "running",
                         "start_pose": self._start_pose if self._auto_open else None,   # 지도의 "출발"(이륙한 자리)
                         # 자율 비행 지도(벽·장애물): 번호와 칸 수만. 화면은 번호가 바뀌면 /api/autonomy/grid를 가져간다
                         "grid": None if self._grid is None else {
                             "ver": self._grid_ver, "w": self._grid["w"], "h": self._grid["h"], "res": self._grid["res"],
                             **(self._grid.get("counts") or {})},
                         "discovered": None if self._found is None else {
                             "version": self._found_ver, "obstacles": len(self._found["obstacles"])},
                         # 자율 비행 웹이 그리는 지도 파일(번호·크기만, 내용은 /api/autonomy/map)과 시뮬레이션 정답 위치
                         "world_map": None if self._world_map is None else {
                             "version": self._world_map_ver, "open": self._world_map["open"],
                             "walls": len(self._world_map["walls"]), "obstacles": len(self._world_map["obstacles"])},
                         "truth": self._truth_view()},
            "video": {"enabled": True, "on": video_on, "reason": "ros_view", "ground_s": None, "ground_off_s": None,
                      "available": video_on, "fps": vstats["fps"] if video_on else None,
                      "frame_age_s": vstats["frame_age_s"]},
            "motor_time_s": si.get("motor_time_s"), "flight_time_s": self._flight_time(),
            "last_transition": self.last_transition,
            "operator_interventions": None, "autonomy_used": True,
            "detection": self.detections.status(), "overlay": self.overlay.status(), "map": self.map.status(),
            "preflight": fs.get("preflight") if mode == "grounded" else None,   # 자율 드라이버의 점검(그 기준) 그대로
            "ros_view": {
                "url": self.url, "connected": self._tap is not None, "reason": self.mode_reason,
                # 시작 칸: 여는 조건(자율 비행 웹과 같음)과 땅에서 기수 값이 흐르는가(막지 않고 알린다)
                "commands": bool((tap.get("commands") or {}).get("enabled")), "start": {**self._start_info(), **self._yaw_settle()},
                "need_yolo": tap.get("need_yolo"), "yolo": tap.get("yolo"),
                "topics": tap.get("topics") or {},
                "driver": {k: fs.get(k) for k in ("connected", "flying", "busy", "last_auto_land", "last_warning",
                                                   "landing_reason", "last_result", "rc_limit")} if fs else None,
            },
        }
