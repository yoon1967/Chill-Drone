"""드론 게이트웨이: TelloLink·FlightController·ModeArbiter를 묶어 화면과 주고받는다 (표준 라이브러리만 사용).

계약: contracts/api/DRONE_CONTROL_V0_1.md (화면 ↔ 게이트웨이).
- 화면 메시지(`cmd`, `rc`)를 판정기에 넘기고, 판정기가 내준 할 일을 실행한다.
- 20Hz 루프: 연결 상태·안전 착륙·착륙 완료를 판정기에 알리고, 판정기가 고른 rc를 FlightController에 넣는다.
- `drone_state()`로 화면에 보낼 상태를 만든다.
- 영상: 지상에서 켜 두면 1분에 약 16°C 오른다(Tello는 팬이 없고 날면 식는다). 그래서 자동으로 켜지 않는다.
  화면의 `video_on`·`video_off`(판정기와 무관한 보조 명령), 이륙 성공 시 자동으로 켠다. 지상에서 오래 켜져 있으면 경고 후 끈다.
  `video_frame()`은 내준 프레임에 영상 세션·프레임 번호를 붙여 최근 것만 보관한다(frame_store.py, 탐지 결과 연결용).
- 사람 탐지: `detections`(detection_inbox.py)가 탐지 결과를 받아 회차 폴더에 저장하고 화면에 `detection` 메시지로 보낸다.
  드론을 자동으로 멈추지 않는다(운용자가 정함).
- AI 합성 영상: `overlay`(overlay_store.py)가 탐지 프로그램이 그려 보낸 최신 장을 들고 있다. 화면이 원본 대신 볼 수 있다.
- 3D 지도: `map`(map_inbox.py)이 지도 연결기(gateway_map_bridge.py)가 보낸 최신 점군을 들고 있다가 화면에 `map` 알림을 보낸다.
실행(이륙·착륙·비상 정지, rc 전송, 안전 착륙, 높이 상한, 데드맨)은 실기체로 검증한 FlightController가 그대로 맡는다.
웹 서버는 gateway_server.py에 있다. 자율 비행 연결(`/ws/autonomy`)은 다음 작업이다.
"""

from __future__ import annotations

import threading
import time

from flight_control import STATE_STALE_S, FlightController
from detection_inbox import DetectionInbox
from frame_store import FrameStore
from map_inbox import MapInbox
from overlay_store import OverlayStore
from mode_arbiter import SCHEMA_VERSION, ModeArbiter
from tello_link import TelloLink, now_iso

LOOP_HZ = 20
SDK_RETRY_S = 2.0            # command 응답이 없으면 이 간격으로 다시 보낸다
SDK_RESYNC_S = 3.0           # 땅에서 상태가 이만큼 끊기면 command를 다시 보낸다(드론을 껐다 켜면 SDK 모드가 풀린다)
EMERGENCY_SETTLE_S = 3.0     # 비상 정지 뒤 지상 판정을 기다리는 최대 시간
BATTERY_WARN_PCT = 30        # 경고만. 자동 착륙(15%)은 FlightController
TEMP_WARN_C = 75             # 경고만. 고온 자동 착륙은 하지 않는다(기체 자체 보호에 맡김)
VIDEO_STALE_S = 1.0
VIDEO_START_GRACE_S = 3.0    # streamon 직후 첫 프레임까지는 video_stale로 보지 않는다
VIDEO_GROUND_WARN_S = 30.0   # 지상에서 영상이 이만큼 켜져 있으면 video_on_ground 경고
VIDEO_GROUND_OFF_S = 60.0    # 지상에서 영상이 이만큼 켜져 있으면 자동으로 끈다
VIDEO_CMDS = ("video_on", "video_off")
CLIENTS = ("monitor", "control")


class Gateway:
    def __init__(self, link: TelloLink, *, source: str = "tello", video=None, clock=time.monotonic):
        if not link.allow_flight:
            raise ValueError("게이트웨이는 allow_flight=True인 TelloLink가 필요하다")
        self.link = link
        self.source = source
        self.video = video                      # TelloVideo 또는 None
        self.clock = clock
        self.flight = FlightController(link)
        self.arbiter = ModeArbiter(source=source, clock=clock)
        self.listeners: list = []               # 화면으로 보낼 메시지를 받는 함수들(서버가 등록)
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._last_auto_land = None
        self._emergency_mono: float | None = None
        self._sdk_try_mono = -1e9
        self._link_lost = False                 # 상태 끊김으로 command를 다시 보내는 중(끊김 한 번에 영상 정리 한 번)
        self._rc_clients: set[str] = set()
        self._video_lock = threading.Lock()     # streamon·streamoff를 한 번에 하나씩
        self._video_on = False                  # 기체에 streamon이 받아들여진 상태
        self._video_reason: str | None = None   # 마지막으로 바뀐 이유: operator·takeoff·ground_timeout
        self._video_on_mono: float | None = None
        self._video_ground_mono: float | None = None  # 지상에서 영상이 켜진 채 있기 시작한 시각
        self._video_retry_mono = -1e9
        self._video_busy = False
        self._video_synced = False              # 시작 시 streamoff로 기체 쪽 영상 상태를 맞췄는지
        self.frames = FrameStore(link.run_id, clock=clock)  # 내준 프레임 번호·최근 프레임
        self.detections = DetectionInbox(self.frames, run_dir=link.log.dir if link.log else None,
                                         emit=self._emit, clock=clock)
        self.overlay = OverlayStore(clock=clock)
        self.map = MapInbox(run_dir=link.log.dir if link.log else None, emit=self._emit, clock=clock)  # 3D 지도(지도 연결기)

    # ---------- 수명 ----------
    def start(self) -> "Gateway":
        self.flight.start()
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, name="drone-gateway", daemon=True)
        self._thread.start()
        return self

    def close(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2)
        self.flight.close()  # 비행 중이면 착륙부터

    # ---------- 화면 메시지 ----------
    def handle_message(self, msg: dict, client_key: str = "") -> dict | None:
        """화면에서 온 JSON 하나. `cmd`면 result를 돌려준다. `rc`는 돌려주지 않는다."""
        kind = msg.get("type")
        client = msg.get("client") if msg.get("client") in CLIENTS else "monitor"
        if kind == "cmd" and msg.get("name") in VIDEO_CMDS:  # 판정기 밖: 조종권·모드와 무관하다
            ok, reason = self.set_video(msg["name"] == "video_on", "operator")
            self.link.event("gateway_cmd", {"name": msg["name"], "client": client, "ok": ok, "reason": reason})
            return {"schema_version": SCHEMA_VERSION, "type": "result", "id": msg.get("id"), "ok": ok,
                    "reason": reason, "mode": self.arbiter.mode}
        with self._lock:
            if kind == "cmd":
                name = str(msg.get("name", ""))
                if name == "takeoff" and self.arbiter.mode == "grounded" and not self.flight.can_takeoff()[0]:
                    failed = [c.key for c in self.flight.preflight() if not c.ok]
                    return {"schema_version": SCHEMA_VERSION, "type": "result", "id": msg.get("id"), "ok": False,
                            "reason": "preflight_failed", "failed": failed, "mode": self.arbiter.mode}
                result = self.arbiter.handle_cmd(name, msg.get("args") or {}, client=client, cmd_id=msg.get("id"))
                self.link.event("gateway_cmd", {"name": msg.get("name"), "client": client, "ok": result["ok"],
                                                "reason": result["reason"]})
                self._run_actions()
                return result
            if kind == "rc":
                axes = msg.get("axes")
                if isinstance(axes, list) and len(axes) == 4:
                    self._rc_clients.add(client_key)
                    self.arbiter.on_operator_rc(axes, client=client)
                    self._run_actions()
                return None
        return {"schema_version": SCHEMA_VERSION, "type": "result", "id": msg.get("id"), "ok": False,
                "reason": "unknown_message", "mode": self.arbiter.mode}

    def client_closed(self, client_key: str) -> None:
        """화면 연결이 끊기면 그 화면의 rc를 바로 0으로 본다."""
        with self._lock:
            if client_key in self._rc_clients:
                self._rc_clients.discard(client_key)
                self.arbiter.on_operator_disconnected()

    # ---------- 할 일 실행 ----------
    def _run_actions(self) -> None:
        for action in self.arbiter.drain():
            p = action.payload
            if action.kind == "event":
                self.link.event("gateway_" + p["kind"], p["detail"])
                self._emit({"schema_version": SCHEMA_VERSION, "type": "event", "at": now_iso(), **p})
            elif action.kind == "flight":
                self._run_flight(p)
            # autonomy: 자율 비행 연결(/ws/autonomy)이 생기면 보낸다. 지금은 연결된 자율이 없어 판정기가 내지 않는다.

    def _run_flight(self, p: dict) -> None:
        name = p["name"]
        if name == "emergency":
            self.flight.emergency(source="gateway")   # 잠금 없이 바로 나간다
            self._emergency_mono = self.clock()
        elif name == "confirm_area":
            self.flight.confirm_operator(bool(p.get("confirmed", True)))
        elif name == "takeoff":
            threading.Thread(target=self._takeoff, name="gateway-takeoff", daemon=True).start()
        elif name == "land":
            threading.Thread(target=self._land, name="gateway-land", daemon=True).start()

    def _takeoff(self) -> None:
        r = self.flight.takeoff(source="gateway")  # 응답까지 약 9초
        with self._lock:
            self.arbiter.on_takeoff_done(bool(r.get("flying")), None if r.get("flying") else r.get("reason") or "error_response")
            self._run_actions()
        if r.get("flying") and self.video is not None and not self._video_on:
            self.set_video(True, "takeoff")  # 날면 식으므로 이륙 뒤에 켠다

    def _land(self) -> None:
        r = self.flight.land(source="gateway")
        with self._lock:
            if r["ok"] or not self.flight.flying:
                self.arbiter.on_landed("landed")
            self._run_actions()

    # ---------- 영상 ----------
    def set_video(self, on: bool, reason: str) -> tuple[bool, str | None]:
        """streamon·streamoff를 보내고 응답(ok)을 받은 경우에만 상태를 바꾼다. 응답까지 막힌다(최대 약 3초)."""
        if self.video is None:
            return False, "video_disabled"   # --no-video: 영상 수신 자체를 끈 실행
        with self._video_lock:
            if on and not self.video.running:
                try:
                    self.video.start()
                except (ImportError, OSError) as e:  # PyAV 없음, 포트 사용 중
                    self.link.event("gateway_video", {"on": True, "ok": False, "reason": "receiver_failed", "error": str(e)})
                    return False, "video_receiver_failed"
            r = self.link.send("streamon" if on else "streamoff", timeout=3, source="gateway")
            if not r.ok:
                self.link.event("gateway_video", {"on": on, "ok": False, "reason": r.reason})
                return False, "no_response" if r.response is None else r.reason
            self._video_synced = True
            with self._lock:
                changed = self._video_on != on
                if on and changed:
                    self.frames.new_session()
                self._video_on, self._video_reason = on, reason
                self._video_on_mono = self.clock() if on else None
                self._video_ground_mono = None
            self.link.event("gateway_video", {"on": on, "ok": True, "reason": reason})
            if changed:
                self._emit({"schema_version": SCHEMA_VERSION, "type": "event", "at": now_iso(), "kind": "video_changed",
                            "detail": {"on": on, "reason": reason}})
            return True, None

    def video_frame(self) -> tuple[dict | None, str | None]:
        """최신 프레임(JPEG·영상 세션·프레임 번호·수신 시각). 없으면 (None, 이유).

        영상을 켠 뒤 디코딩한 프레임만 내준다. 끄기 전 마지막 프레임이 수신기에 남아 있어도 내주지 않는다.
        """
        if self.video is None:
            return None, "video_disabled"
        with self._lock:
            video_on, on_mono = self._video_on, self._video_on_mono
        if not video_on:
            return None, "video_off"
        snap = self.video.jpeg_snapshot()
        if snap is None or on_mono is None or snap[1] < on_mono:
            return None, "no_frame"
        jpeg, stamp, wall = snap
        if self.clock() - stamp > VIDEO_STALE_S:
            return None, "video_stale"
        frame = self.frames.add(jpeg, stamp, wall, getattr(self.video, "size", None), self._drone_brief())
        return (frame, None) if frame is not None else (None, "no_frame")

    def _drone_brief(self) -> dict:
        """프레임에 붙이는 그때 드론 상태 요약. 위치(pose)는 자율 비행 연결 전이라 없다."""
        si = (self.link.latest_state() or {}).get("si") or {}
        with self._lock:
            mode = self.arbiter.mode
        return {"mode": mode, "height_m": si.get("tof_m") if si.get("tof_valid") else None,
                "yaw_rad": si.get("yaw_rad"), "battery_pct": si.get("battery_pct"), "pose": None}

    def _video_auto(self, on: bool, reason: str) -> None:
        try:
            ok, _ = self.set_video(on, reason)
            if not ok:
                self._video_retry_mono = self.clock() + SDK_RETRY_S
        finally:
            self._video_busy = False

    def _video_tick(self) -> None:
        """루프에서 부른다(잠금 안). 시작 시 기체 영상 상태 맞추기, 지상 장시간 자동 끄기."""
        if self._video_busy or not self.link.sdk_mode or self.clock() < self._video_retry_mono:
            return
        if not self._video_synced:
            # 게이트웨이를 다시 켜도 기체는 전에 켠 영상을 계속 보낼 수 있다. 꺼진 상태로 맞추고 시작한다.
            self._video_synced = True
            self._video_busy = True
            threading.Thread(target=self._video_quiet_off, name="gateway-video", daemon=True).start()
            return
        if not self._video_on or self.arbiter.mode != "grounded":
            self._video_ground_mono = None
            return
        if self._video_ground_mono is None:
            self._video_ground_mono = self.clock()
        elif self.clock() - self._video_ground_mono >= VIDEO_GROUND_OFF_S:
            self._video_busy = True
            threading.Thread(target=self._video_auto, args=(False, "ground_timeout"), name="gateway-video",
                             daemon=True).start()

    def _video_quiet_off(self) -> None:
        try:
            with self._video_lock:
                if self._video_on:  # 그사이 화면에서 켰다
                    return
                r = self.link.send("streamoff", timeout=3, source="gateway")
                self.link.event("gateway_video", {"on": False, "ok": r.ok, "reason": "startup_sync"})
                if not r.ok:
                    self._video_synced = False
                    self._video_retry_mono = self.clock() + SDK_RETRY_S
        finally:
            self._video_busy = False

    def _video_ground_s(self) -> float | None:
        return None if self._video_ground_mono is None else self.clock() - self._video_ground_mono

    def _emit(self, msg: dict) -> None:
        for fn in list(self.listeners):
            try:
                fn(msg)
            except Exception:  # 화면 하나가 잘못돼도 게이트웨이는 계속 돈다
                pass

    # ---------- 루프 ----------
    def _ensure_sdk(self) -> None:
        """처음 연결할 때, 그리고 땅에서 상태가 SDK_RESYNC_S 넘게 끊기면 command를 (다시) 보낸다.

        드론을 껐다 켜면 SDK 모드가 풀려 상태를 보내지 않는데 link.sdk_mode는 참으로 남아,
        다시 보내지 않고 계속 끊김으로 있었다(11/05 실기체, 41분).
        비행 중에는 다시 보내지 않는다: 응답을 기다리는 동안 rc·land가 같은 잠금에서 밀리고,
        비행 중 끊김은 FlightController가 착륙시킨다. 착륙해 flying이 풀리면 다시 보낸다.
        """
        age = self._state_age()
        if age < SDK_RESYNC_S:
            self._link_lost = False
        if self.clock() - self._sdk_try_mono < SDK_RETRY_S:
            return
        if self.link.sdk_mode and (age < SDK_RESYNC_S or self.flight.flying):
            return
        if self.link.sdk_mode and not self._link_lost:
            self._link_lost = True
            self.link.event("gateway_link", {"lost": True, "state_age_s": None if age == float("inf") else round(age, 1)})
            self._forget_video()
        self._sdk_try_mono = self.clock()
        threading.Thread(target=lambda: self.link.send("command", timeout=1.5, source="gateway"), daemon=True).start()

    def _state_age(self) -> float:
        state = self.link.latest_state()
        return float("inf") if state is None else state.get("age_s", float("inf"))

    def _forget_video(self) -> None:
        """끊긴 동안 기체를 껐다 켰으면 기체 영상은 꺼져 있다. 꺼진 것으로 두고, 다시 붙으면 시작할 때처럼 streamoff로 맞춘다."""
        with self._lock:
            was_on = self._video_on
            self._video_on, self._video_on_mono, self._video_ground_mono = False, None, None
            if was_on:
                self._video_reason = "link_lost"
            self._video_synced = False
        if was_on:
            self.link.event("gateway_video", {"on": False, "ok": True, "reason": "link_lost"})
            self._emit({"schema_version": SCHEMA_VERSION, "type": "event", "at": now_iso(), "kind": "video_changed",
                        "detail": {"on": False, "reason": "link_lost"}})

    def _state_ok(self) -> bool:
        state = self.link.latest_state()
        return state is not None and state.get("age_s", 99) <= STATE_STALE_S

    def _on_ground(self) -> bool:
        si = (self.link.latest_state() or {}).get("si") or {}
        return si.get("tof_reason") == "below_min_or_on_ground"

    def tick(self) -> None:
        """루프 한 번. 시험에서는 직접 부른다."""
        self._ensure_sdk()
        with self._lock:
            arb = self.arbiter
            arb.on_link(self._state_ok() and self.link.sdk_mode)
            auto_land = self.flight.last_auto_land
            if auto_land is not None and auto_land is not self._last_auto_land:
                self._last_auto_land = auto_land
                arb.on_safety_landing(auto_land["reason"])
            if arb.mode == "landing" and not self.flight.flying and self.flight.busy is None:
                arb.on_landed("landed")
            if arb.mode == "emergency" and self._emergency_mono is not None and (
                    self._on_ground() or self.clock() - self._emergency_mono > EMERGENCY_SETTLE_S):
                self._emergency_mono = None
                arb.on_landed("emergency_stopped")
            out = arb.tick()
            if out.rc is not None and self.flight.flying and self.flight.busy is None:
                self.flight.set_input(*(v / 100 for v in out.rc), source="gateway:" + out.source)
            self._run_actions()
            self._video_tick()

    def _loop(self) -> None:
        period = 1.0 / LOOP_HZ
        while not self._stop.is_set():
            t0 = time.monotonic()
            try:
                self.tick()
            except OSError as e:  # 종료 중 소켓이 닫힌 경우
                if self._stop.is_set():
                    break
                self.link.event("gateway_loop_error", {"error": str(e)})
            self._stop.wait(max(0.0, period - (time.monotonic() - t0)))

    # ---------- 화면으로 보낼 상태 ----------
    def drone_state(self) -> dict:
        state = self.link.latest_state() or {}
        si = state.get("si") or {}
        with self._lock:
            snap = self.arbiter.snapshot()
            video_on, video_reason, ground_s = self._video_on, self._video_reason, self._video_ground_s()
            on_s = None if self._video_on_mono is None else self.clock() - self._video_on_mono
        warnings = list(snap["warnings"])
        bat, temp = si.get("battery_pct"), si.get("temp_high_c")
        if isinstance(bat, (int, float)) and bat < BATTERY_WARN_PCT:
            warnings.append("battery_low")
        if isinstance(temp, (int, float)) and temp >= TEMP_WARN_C:
            warnings.append("temp_high")
        if snap["flying"] and not si.get("tof_valid"):
            warnings.append("height_unknown")
        video = self.video.stats() if self.video is not None else None
        video_ok = bool(video_on and video and video["running"] and video["frame_age_s"] is not None
                        and video["frame_age_s"] < VIDEO_STALE_S)
        if video_on and not video_ok and on_s is not None and on_s >= VIDEO_START_GRACE_S:
            warnings.append("video_stale")
        if ground_s is not None and ground_s >= VIDEO_GROUND_WARN_S:
            warnings.append("video_on_ground")  # 지상 발열. VIDEO_GROUND_OFF_S가 되면 자동으로 끈다
        down_valid = bool(si.get("tof_valid"))
        return {
            "schema_version": SCHEMA_VERSION,
            "type": "drone_state",
            "at": now_iso(),
            "time_source": "gateway_receive",
            "source": self.source,
            "connected": self.arbiter.link_ok,
            "state_age_s": None if state.get("age_s") is None else round(state["age_s"], 3),
            "mode": snap["mode"],
            "authority": snap["authority"],
            "allowed": snap["allowed"],
            "warnings": warnings,
            "battery_pct": bat,
            "height_m": si.get("tof_m") if down_valid else None,   # h는 믿기 어려워 하방 tof를 쓴다
            "height_source": "down_tof",
            "temp_high_c": temp,
            "attitude": {k: si.get(k) for k in ("roll_rad", "pitch_rad", "yaw_rad")},
            "front_tof": {"m": None, "valid": False, "reason": "not_polled", "age_s": None},
            "down_tof": {"m": si.get("tof_m"), "valid": down_valid, "reason": si.get("tof_reason")},
            "pose": None,
            "mission": snap["mission"],
            "video": {"enabled": self.video is not None, "on": video_on, "reason": video_reason,
                      "ground_s": None if ground_s is None else round(ground_s, 1), "ground_off_s": VIDEO_GROUND_OFF_S,
                      "available": video_ok, "fps": video["fps"] if video_ok else None,
                      "frame_age_s": video["frame_age_s"] if video_on and video else None},
            "motor_time_s": si.get("motor_time_s"),
            "last_transition": snap["last_transition"],
            "operator_interventions": snap["operator_interventions"],
            "autonomy_used": snap["autonomy_used"],
            "detection": self.detections.status(),
            "overlay": self.overlay.status(),
            "map": self.map.status(),
            "preflight": [c.__dict__ for c in self.flight.preflight()] if snap["mode"] == "grounded" else None,
        }
