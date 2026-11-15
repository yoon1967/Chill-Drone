"""자율 비행(ROS) 보기: 게이트웨이가 드론 대신 보기 노드(ros_view_node.py)에서 상태·영상을 받아 화면에 보인다 (표준 라이브러리만).

자율 비행 프로그램이 드론에 붙어 난다(드라이버·안전 장치·이륙 흐름 그대로). 게이트웨이는 드론에 붙지 않고 보기만 한다.
서버 `--ros-view http://127.0.0.1:8094`. 기록 재생(replay.py)처럼 서버가 쓰는 모양만 맞춘다.
- 상태: Tello 원문을 parse_state로 풀어 배터리·높이·자세. 모드는 자율 드라이버 상태로 나눈다:
  보기 노드·드론 끊김 disconnected, 안 떠 있음 grounded, 착륙 중 landing, 떠 있음 auto(자율이 조종). 이륙 전 점검은 드라이버 것 그대로.
- 영상: 보기 노드 MJPEG → 프레임 저장소(/snapshot.jpg·/video.mjpg). 사람 탐지 연결기가 지금처럼 가져간다.
- 사람 탐지·확인·오탐, AI 합성, 3D 지도 받기는 실시간 게이트웨이와 같다.
- 명령: 1단계는 받지 않는다(ros_view_read_only). 조종은 자율 비행 프로그램(웹)이 한다.
- 회차 폴더(manifest source "ros"): sensor(상태)·events(모드 바뀜·보기 노드 연결)·detections·map·metrics.
  게이트웨이 회차와 같은 모양이라 기록 재생(--replay)으로 다시 볼 수 있다(영상 원본은 자율 드라이버가 자기 폴더에 남긴다).
"""

from __future__ import annotations

import json
import math
import secrets
import threading
import time
from urllib.request import urlopen

from autonomy_link import AutonomyLink
from detection_inbox import DetectionInbox
from frame_store import FrameStore
from map_inbox import MapInbox
from mode_arbiter import SCHEMA_VERSION
from overlay_store import OverlayStore
from tello_link import RunLog, now_iso, parse_state

POLL_S = 0.1              # 보기 노드 상태를 묻는 간격(Tello 상태 10Hz)
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

    def event(self, kind: str, detail: dict) -> None:
        self.log.append("events", {"t": now_iso(), "type": kind, "detail": detail})

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self.log.close({"run_id": self.run_id, "source": "ros", "duration_s": round(time.monotonic() - self._started, 1),
                        **self.metrics()})


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
    def __init__(self, url: str, *, log_root: str, clock=time.monotonic):
        self.url = url.rstrip("/")
        self.source = "ros"
        self.clock = clock
        self.listeners: list = []
        self.link = ViewLink(log_root, self.url)
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
        self.states = 0
        self.mode, self.mode_reason = "disconnected", "ros_view_not_started"
        self.last_transition: dict | None = None
        self._mode_changes = 0
        self._video_session = None
        self.link.metrics = lambda: {"states": self.states, "frames": self.video.frames, "tap_errors": self._tap_errors,
                                     "mode_changes": self._mode_changes}

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

    def _emit(self, msg: dict) -> None:
        for fn in list(self.listeners):
            try:
                fn(msg)
            except Exception:
                pass

    def _loop(self) -> None:
        while not self._stop.is_set():
            t0 = time.monotonic()
            self.tick()
            self._stop.wait(max(0.0, POLL_S - (time.monotonic() - t0)))

    # ---------- 보기 노드 ----------
    def tick(self) -> None:
        """보기 노드 상태를 한 번 묻고 모드를 다시 정한다. 시험에서는 직접 부른다."""
        try:
            with urlopen(self.url + "/api/state", timeout=POLL_TIMEOUT_S) as r:
                tap = json.loads(r.read())
            if not isinstance(tap, dict):
                raise ValueError("not a dict")
        except (OSError, ValueError) as e:
            self._tap_errors += 1
            if self._tap_mono is not None and self.clock() - self._tap_mono > TAP_STALE_S and self._tap is not None:
                self._tap = None
                self.link.event("ros_view_lost", {"error": type(e).__name__})
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
        key = (st.get("ros_stamp"), st.get("raw"))
        if isinstance(st.get("raw"), str) and key != self._state_key:
            self._state_key = key
            age = float(st.get("age_s") or 0.0)
            sample = parse_state(st["raw"], received=time.time() - age, received_mono=now - age)
            self._sample = sample
            self.states += 1
            self.link.log.append("sensor", {**sample.to_dict(), "time_source": "ros_view", "sequence": self.states})
        self._update_mode()

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
        if fs.get("busy") == "land":
            return "landing", "autonomy_landing"
        if fs.get("flying"):
            return "auto", "autonomy_flying"
        return "grounded", "autonomy_ground"

    def _update_mode(self) -> None:
        mode, reason = self._derive_mode()
        with self._lock:
            if mode == self.mode:
                self.mode_reason = reason
                return
            self.last_transition = {"from": self.mode, "to": mode, "by": "ros_view", "reason": reason}
            self.mode, self.mode_reason = mode, reason
            self._mode_changes += 1
            transition = dict(self.last_transition)
        self.link.event("gateway_mode_changed", transition)   # 게이트웨이 회차와 같은 이름(기록 재생이 읽는다)
        self._emit({"schema_version": SCHEMA_VERSION, "type": "event", "at": now_iso(), "kind": "mode_changed",
                    "detail": transition})

    # ---------- 화면 ----------
    def handle_message(self, msg: dict, client_key: str = "") -> dict | None:
        if msg.get("type") == "cmd":
            self.link.event("ros_view_cmd_refused", {"name": msg.get("name"), "client": msg.get("client")})
            return {"schema_version": SCHEMA_VERSION, "type": "result", "id": msg.get("id"), "ok": False,
                    "reason": "ros_view_read_only", "mode": self.mode}
        return None   # rc: 자율 비행 중 사람 rc 길은 없다

    def client_closed(self, client_key: str) -> None:
        pass

    def autonomy_open(self, session: str, send, peer: str | None = None) -> bool:
        return False   # 자율은 보기 노드로 본다(서버가 busy로 닫는다)

    def autonomy_message(self, session: str, msg) -> None:
        return None

    def autonomy_closed(self, session: str) -> None:
        pass

    def _brief(self) -> dict:
        si = (self._sample.si if self._sample is not None else None) or {}
        return {"mode": self.mode, "height_m": si.get("tof_m") if si.get("tof_valid") else None,
                "yaw_rad": si.get("yaw_rad"), "battery_pct": si.get("battery_pct"), "pose": None}

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
        down_valid = bool(si.get("tof_valid"))
        tap = self._tap or {}
        return {
            "schema_version": SCHEMA_VERSION, "type": "drone_state", "run_id": self.link.run_id, "at": now_iso(),
            "time_source": "gateway_receive", "source": "ros", "mapping_console_url": None,
            "connected": mode != "disconnected", "state_age_s": state_age, "mode": mode,
            "flight_busy": fs.get("busy") if fs.get("busy") in ("takeoff", "land") else None,
            "authority": "autonomy" if mode in ("auto", "landing") else "none", "allowed": [],
            "warnings": warnings, "battery_pct": bat, "height_m": si.get("tof_m") if down_valid else None,
            "height_source": "down_tof", "temp_high_c": temp,
            "attitude": {k: si.get(k) for k in ("roll_rad", "pitch_rad", "yaw_rad")},
            "front_tof": self._front_tof(),
            "down_tof": {"m": si.get("tof_m"), "valid": down_valid, "reason": si.get("tof_reason")},
            "pose": self.autonomy.pose_state(), "mission": None,
            "autonomy": {**self.autonomy.status(), "ground_origin": None, "mission_active": None},
            "video": {"enabled": True, "on": video_on, "reason": "ros_view", "ground_s": None, "ground_off_s": None,
                      "available": video_on, "fps": vstats["fps"] if video_on else None,
                      "frame_age_s": vstats["frame_age_s"]},
            "motor_time_s": si.get("motor_time_s"), "last_transition": self.last_transition,
            "operator_interventions": None, "autonomy_used": True,
            "detection": self.detections.status(), "overlay": self.overlay.status(), "map": self.map.status(),
            "preflight": fs.get("preflight") if mode == "grounded" else None,   # 자율 드라이버의 점검(그 기준) 그대로
            "ros_view": {
                "url": self.url, "connected": self._tap is not None, "reason": self.mode_reason,
                "topics": tap.get("topics") or {},
                "driver": {k: fs.get(k) for k in ("connected", "flying", "busy", "last_auto_land", "last_warning",
                                                   "landing_reason", "last_result", "rc_limit")} if fs else None,
            },
        }
