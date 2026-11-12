"""기록 재생: 게이트웨이 회차 폴더를 시간에 맞춰 다시 내보낸다 (`gateway_server.py --replay <회차 폴더>`).

화면은 실시간과 똑같이 붙는다(같은 /ws/drone·/video.mjpg·/snapshot.jpg·/api/...). 기체가 없고 명령은 모두
`replay_source`로 거절한다(판정기 계약과 같다). 원래 회차 폴더에는 아무것도 쓰지 않는다.

다시 내보내는 것(회차 폴더 → 화면):
- 드론 상태 sensor.jsonl(받은 시각), 모드 변화·영상 켜고 끔 events.jsonl
- 영상 video/part_NNNN.h264 + 색인(패킷을 받은 시각대로 같은 디코더에 넣는다. 영상 원본이 없는 회차는 영상 없음)
- 사람 탐지 detections.jsonl·detections/(발견은 기록된 finding_id로 다시 묶는다, 영상 위 윤곽선용 detection 메시지)
  확인·오탐(review 줄)은 그 시각에 detection_review로 다시 보내고 발견 목록에도 싣는다. 재생 중에는 바꿀 수 없다(409)
- 3D 지도 map/snapshots(없으면 map/latest.json 하나)
- 자율 autonomy/: 위치·경로·찾은 사람·임무 상태·연결
시간은 원래 회차의 UTC 시각 그대로 흐른다(`--replay-speed`, `--replay-loop`). drone_state.replay에 재생 위치가 있다.
"""

from __future__ import annotations

import bisect
import gzip
import json
import os
import re
import threading
import time
from collections import OrderedDict
from datetime import datetime

from frame_store import FrameStore
from mode_arbiter import SCHEMA_VERSION
from overlay_store import OverlayStore
from tello_link import now_iso
from tello_video import TelloVideo

DETECT_ALIVE_S = 2.0
MAP_ALIVE_S = 30.0
POSE_STALE_S = 1.0
STATE_STALE_S = 1.0
VIDEO_STALE_S = 1.0
BATTERY_WARN_PCT = 30
TEMP_WARN_C = 75
UUID = re.compile(r"^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$")
FLYING = ("manual", "auto", "paused", "landing")
SCREEN_EVENTS = {"gateway_mode_changed": "mode_changed", "gateway_safety_landing": "safety_landing",
                 "gateway_autonomy_connected": "autonomy_connected", "gateway_autonomy_disconnected": "autonomy_disconnected",
                 "gateway_mission_finished": "mission_finished"}


def _t(text) -> float | None:
    if not text:
        return None
    try:
        return datetime.fromisoformat(str(text).replace("Z", "+00:00")).timestamp()
    except ValueError:
        return None


def _jsonl(path: str) -> list[dict]:
    if not os.path.isfile(path):
        return []
    out = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            try:
                out.append(json.loads(line))
            except ValueError:
                pass   # 강제 종료로 마지막 줄이 잘린 회차도 읽는다
    return out


def _timed(rows: list[dict], key: str) -> tuple[list[float], list[dict]]:
    pairs = sorted(((_t(r.get(key)), r) for r in rows if _t(r.get(key)) is not None), key=lambda p: p[0])
    return [p[0] for p in pairs], [p[1] for p in pairs]


def _at(times: list[float], rows: list, t: float):
    i = bisect.bisect_right(times, t)
    return (rows[i - 1], times[i - 1]) if i else (None, None)


def _has_sps(data: bytes) -> bool:
    """H.264 SPS(NAL 7)가 들어 있는 패킷: 여기서부터 디코딩을 새로 시작할 수 있다."""
    i = data.find(b"\x00\x00\x01")
    while i != -1 and i + 3 < len(data):
        if data[i + 3] & 0x1F == 7:
            return True
        i = data.find(b"\x00\x00\x01", i + 3)
    return False


class Recording:
    """회차 폴더 하나를 읽어 시각순으로 들고 있다."""

    def __init__(self, run_dir: str):
        if not os.path.isfile(os.path.join(run_dir, "manifest.json")) and not os.path.isfile(os.path.join(run_dir, "sensor.jsonl")):
            raise FileNotFoundError(f"게이트웨이 회차 폴더가 아니다: {run_dir}")
        self.dir = run_dir
        self.run_id = os.path.basename(os.path.normpath(run_dir))
        self.sensor_t, self.sensor = _timed(_jsonl(os.path.join(run_dir, "sensor.jsonl")), "received_at")
        self.event_t, self.events = _timed(_jsonl(os.path.join(run_dir, "events.jsonl")), "t")
        modes = [(t, e["detail"]) for t, e in zip(self.event_t, self.events) if e.get("type") == "gateway_mode_changed"]
        self.mode_t, self.modes = [m[0] for m in modes], [m[1] for m in modes]
        vids = [(t, e["detail"]) for t, e in zip(self.event_t, self.events)
                if e.get("type") == "gateway_video" and (e.get("detail") or {}).get("ok")]
        self.video_t, self.video_on = [v[0] for v in vids], [v[1] for v in vids]
        det_rows = _jsonl(os.path.join(run_dir, "detections.jsonl"))
        self.det_t, self.dets = _timed([r for r in det_rows if r.get("type") == "event"], "received_at")
        self.review_t, self.reviews = _timed([r for r in det_rows if r.get("type") == "review" and r.get("finding_id")], "at")
        self.map_t, self.map_files, self.map_lines = self._maps()
        a = os.path.join(run_dir, "autonomy")
        self.pose_t, self.poses = _timed(_jsonl(os.path.join(a, "pose.jsonl")), "at")
        self.path_t, self.paths = _timed(_jsonl(os.path.join(a, "path.jsonl")), "at")
        self.people_t, self.peoples = _timed(_jsonl(os.path.join(a, "people.jsonl")), "at")
        self.auto_t, self.auto_events = _timed(_jsonl(os.path.join(a, "events.jsonl")), "at")
        self.parts = self._video_parts()
        if not self.sensor_t and not self.event_t:
            raise ValueError(f"재생할 상태·이벤트가 없다: {run_dir}")
        lists = [self.sensor_t, self.event_t, self.det_t, self.review_t, self.map_t, self.pose_t, self.path_t, self.people_t,
                 self.auto_t, [p["start"] for p in self.parts], [p["end"] for p in self.parts]]
        self.t0 = min(ts[0] for ts in lists if ts)
        self.t1 = max(ts[-1] for ts in lists if ts)

    def _maps(self):
        lines_t, lines = _timed(_jsonl(os.path.join(self.dir, "map_updates.jsonl")), "received_at")
        files = [(t, os.path.join(self.dir, r["saved"])) for t, r in zip(lines_t, lines)
                 if r.get("saved") and os.path.isfile(os.path.join(self.dir, r["saved"]))]
        latest = os.path.join(self.dir, "map", "latest.json")
        if os.path.isfile(latest) and lines_t:   # 버전별 저장 전 회차: 마지막 지도 하나만 마지막 수신 시각에
            last_saved = files[-1][0] if files else None
            if last_saved is None or last_saved < lines_t[-1]:
                files.append((lines_t[-1], latest))
        return [f[0] for f in files], [f[1] for f in files], (lines_t, lines)

    def _video_parts(self) -> list[dict]:
        vdir = os.path.join(self.dir, "video")
        starts = {r["id"]: r for r in _jsonl(os.path.join(vdir, "parts.jsonl")) if r.get("event") == "start"}
        parts = []
        for pid in sorted(starts):
            raw, index = (os.path.join(vdir, f"part_{pid:04d}.{ext}") for ext in ("h264", "jsonl"))
            if not (os.path.isfile(raw) and os.path.isfile(index)):
                continue
            rows = [r for r in _jsonl(index) if isinstance(r.get("wall"), (int, float))]
            if not rows:
                continue
            with open(raw, "rb") as f:
                data = f.read()
            packets = [(r["wall"], r["offset"], r["size"]) for r in rows if r["offset"] + r["size"] <= len(data)]
            keys = [i for i, (_, off, size) in enumerate(packets) if _has_sps(data[off:off + size])] or [0]
            parts.append({"id": pid, "input_id": starts[pid].get("input_id"), "path": raw, "packets": packets,
                          "times": [p[0] for p in packets], "keys": keys, "start": packets[0][0], "end": packets[-1][0]})
        return parts


class ReplayClock:
    """재생 위치(초). 원래 회차 시각 = t0 + 위치."""

    def __init__(self, t0: float, t1: float, *, speed: float = 1.0, loop: bool = False, start_s: float = 0.0,
                 mono=time.monotonic):
        self.t0, self.duration = t0, max(0.0, t1 - t0)
        self.speed, self.loop, self.mono = speed, loop, mono
        self._lock = threading.Lock()
        self._base_mono, self._base_pos = mono(), max(0.0, start_s)
        self.paused = False

    def pos(self) -> float:
        with self._lock:
            p = self._base_pos if self.paused else self._base_pos + (self.mono() - self._base_mono) * self.speed
        if self.duration <= 0:
            return 0.0
        return p % self.duration if self.loop else min(p, self.duration)

    def wall(self) -> float:
        return self.t0 + self.pos()

    def seek(self, pos_s: float) -> None:
        with self._lock:
            self._base_mono, self._base_pos = self.mono(), min(max(0.0, pos_s), self.duration)

    def ended(self) -> bool:
        return not self.loop and self.pos() >= self.duration


class ReplayVideo(TelloVideo):
    """녹화된 패킷을 받은 시각대로 같은 디코더에 넣는다. 되감기·다른 조각으로 넘어가면 마지막 SPS 패킷부터 다시 푼다."""

    def __init__(self, parts: list[dict], clock: ReplayClock):
        super().__init__(port=0)
        self.parts, self.clock = parts, clock
        self.part_id: int | None = None

    def start(self) -> None:
        if self.running:
            return
        import av
        self._av = av
        self._codec = None
        self._stop.clear()
        self.started_mono = time.monotonic()
        self._thread = threading.Thread(target=self._play, name="replay-video", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2)
        self._thread = None

    def part_at(self, t: float) -> dict | None:
        for p in self.parts:
            if p["start"] <= t <= p["end"] + 0.5:
                return p
        return None

    def _handle(self, data: bytes) -> None:
        """녹화 패킷 하나를 디코딩한다. TelloVideo 수신 루프의 디코딩과 같은 일(녹화로 넘기는 일만 없다).
        tello_video.py는 팀원 수정(JPEG 변환·원본 녹화 연결)이 있어 고치지 않고 여기에 같은 처리를 둔다."""
        self.packets += 1
        self.bytes += len(data)
        self.last_packet_mono = time.monotonic()
        try:
            for packet in self._codec.parse(data):
                for frame in self._codec.decode(packet):
                    now = time.monotonic()
                    with self._lock:
                        self._frame = frame
                        self._frame_mono = now
                        self._frame_wall = time.time()
                        self._frame_times.append(now)
                        self.frames += 1
                        self.size = (frame.width, frame.height)
        except Exception:  # 키프레임 전 패킷 등은 디코딩 실패가 정상이다
            self.decode_errors += 1

    def _play(self) -> None:
        cur, f, i, last = None, None, 0, None
        try:
            while not self._stop.is_set():
                t = self.clock.wall()
                part = self.part_at(t)
                if part is None:
                    cur, self.part_id = None, None
                    self._stop.wait(0.02)
                    continue
                if part is not cur or (last is not None and t < last - 0.05):
                    if f:
                        f.close()
                    f, cur, self.part_id = open(part["path"], "rb"), part, part["id"]
                    self._codec = self._av.CodecContext.create("h264", "r")
                    self._codec.thread_count = 1
                    target = bisect.bisect_right(part["times"], t)
                    i = part["keys"][max(0, bisect.bisect_right(part["keys"], max(0, target - 1)) - 1)]
                last = t
                fed = 0
                while i < len(cur["packets"]) and cur["packets"][i][0] <= t and fed < 400 and not self._stop.is_set():
                    _, off, size = cur["packets"][i]
                    f.seek(off)
                    self._handle(f.read(size))
                    i += 1
                    fed += 1
                self._stop.wait(0.005)
        finally:
            if f:
                f.close()


class _Link:
    """서버가 끝낼 때 부르는 자리(gw.link.close)."""
    def close(self) -> None:
        pass


class ReplayGateway:
    def __init__(self, run_dir: str, *, speed: float = 1.0, loop: bool = False, start_s: float = 0.0,
                 video: bool = True, mono=time.monotonic):
        self.rec = Recording(run_dir)
        self.source = "replay"
        self.clock = ReplayClock(self.rec.t0, self.rec.t1, speed=speed, loop=loop, start_s=start_s, mono=mono)
        self.listeners: list = []
        self.link = _Link()
        self.video = ReplayVideo(self.rec.parts, self.clock) if video and self.rec.parts else None
        self.frames = FrameStore(self.rec.run_id + "-replay")
        self.overlay = OverlayStore()
        self.detections = _Detections(self)
        self.map = _Map(self)
        self.autonomy = _Autonomy(self)
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._last_wall: float | None = None
        self._event_i = self._det_i = self._review_i = 0
        self._map_file: str | None = None
        self._video_input: str | None = None

    # ---------- 수명 ----------
    def start(self) -> "ReplayGateway":
        if self.video is not None:
            self.video.start()
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, name="replay-gateway", daemon=True)
        self._thread.start()
        return self

    def close(self) -> None:
        self._stop.set()
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
            self.tick()
            self._stop.wait(0.1)

    def tick(self) -> None:
        """재생 시각이 지난 사건을 화면에 보낸다(모드·영상 변화, 사람 탐지, 지도 새 버전). 되감기면 처음부터 센다."""
        t = self.clock.wall()
        if self._last_wall is not None and t < self._last_wall:
            self._event_i = self._det_i = self._review_i = 0
            self._map_file = None
            self._emit({"schema_version": SCHEMA_VERSION, "type": "event", "at": now_iso(t), "kind": "replay_restarted",
                        "detail": {"run_id": self.rec.run_id}})
        self._last_wall = t
        rec = self.rec
        while self._event_i < len(rec.events) and rec.event_t[self._event_i] <= t:
            e = rec.events[self._event_i]
            kind, detail = e.get("type"), e.get("detail") or {}
            screen = SCREEN_EVENTS.get(kind)
            if kind == "gateway_video" and detail.get("ok") and detail.get("reason") != "startup_sync":
                screen, detail = "video_changed", {"on": detail.get("on"), "reason": detail.get("reason")}
            if screen:
                self._emit({"schema_version": SCHEMA_VERSION, "type": "event", "at": e.get("t"), "kind": screen, "detail": detail})
            self._event_i += 1
        while self._det_i < len(rec.dets) and rec.det_t[self._det_i] <= t:
            r = rec.dets[self._det_i]
            self._det_i += 1
            if r.get("finding_id") and r["event"].get("detections"):
                finding = self.detections.finding_at(r["finding_id"], self._det_i)
                self._emit({"schema_version": SCHEMA_VERSION, "type": "detection", "at": r.get("received_at"), "event": r["event"],
                            "finding": finding, "latency_ms": r.get("latency_ms"),
                            "photo_url": f"/api/detections/{r['event']['event_id']}.jpg" if r.get("photo") else None})
        while self._review_i < len(rec.reviews) and rec.review_t[self._review_i] <= t:
            r = rec.reviews[self._review_i]
            self._review_i += 1
            self._emit({"schema_version": SCHEMA_VERSION, "type": "detection_review", "at": r.get("at"),
                        "finding_id": r["finding_id"], "review": r.get("review"), "previous": r.get("previous")})
        snap_file = _at(rec.map_t, rec.map_files, t)[0]
        if snap_file and snap_file != self._map_file:
            self._map_file = snap_file
            status = self.map.status()
            self._emit({"schema_version": SCHEMA_VERSION, "type": "map", "at": now_iso(t), "revision": status["revision"],
                        "session_id": None, "segment_id": status["segment_id"], "points": status["points"]})

    # ---------- 화면 ----------
    def handle_message(self, msg: dict, client_key: str = "") -> dict | None:
        if msg.get("type") == "cmd":
            return {"schema_version": SCHEMA_VERSION, "type": "result", "id": msg.get("id"), "ok": False,
                    "reason": "replay_source", "mode": self._mode(self.clock.wall())[0]}
        return None

    def client_closed(self, client_key: str) -> None:
        pass

    def autonomy_open(self, session: str, send) -> bool:
        return False   # 재생 중에는 자율을 받지 않는다(서버가 busy로 닫는다)

    def autonomy_message(self, session: str, msg) -> None:
        return None

    def autonomy_closed(self, session: str) -> None:
        pass

    def _mode(self, t: float):
        d, at = _at(self.rec.mode_t, self.rec.modes, t)
        if d is None:
            first = self.rec.modes[0]["from"] if self.rec.modes else "disconnected"
            return first, None
        return d.get("to"), d

    def _video(self, t: float):
        d, _ = _at(self.rec.video_t, self.rec.video_on, t)
        return (bool(d.get("on")), d.get("reason")) if d else (False, None)

    def video_frame(self):
        t = self.clock.wall()
        on, _ = self._video(t)
        if self.video is None:
            return None, "video_disabled"
        if not on:
            return None, "video_off"
        part = self.video.part_at(t)
        if part and part.get("input_id") != self._video_input:
            self._video_input = part.get("input_id")
            self.frames.new_session()
        snap = self.video.jpeg_snapshot()
        if snap is None:
            return None, "no_frame"
        jpeg, stamp, wall = snap
        if time.monotonic() - stamp > VIDEO_STALE_S:
            return None, "video_stale"
        frame = self.frames.add(jpeg, stamp, t, getattr(self.video, "size", None), self._brief(t))
        return (frame, None) if frame is not None else (None, "no_frame")

    def _brief(self, t: float) -> dict:
        s, _ = _at(self.rec.sensor_t, self.rec.sensor, t)
        si = (s or {}).get("si") or {}
        return {"mode": self._mode(t)[0], "height_m": si.get("tof_m") if si.get("tof_valid") else None,
                "yaw_rad": si.get("yaw_rad"), "battery_pct": si.get("battery_pct"), "pose": None}

    def drone_state(self) -> dict:
        t = self.clock.wall()
        sample, st = _at(self.rec.sensor_t, self.rec.sensor, t)
        si = (sample or {}).get("si") or {}
        age = None if st is None else t - st
        mode, transition = self._mode(t)
        connected = age is not None and age <= STATE_STALE_S
        video_on, video_reason = self._video(t)
        vstats = self.video.stats() if self.video is not None else None
        video_ok = bool(video_on and vstats and vstats["frame_age_s"] is not None and vstats["frame_age_s"] < VIDEO_STALE_S)
        warnings = []
        bat, temp = si.get("battery_pct"), si.get("temp_high_c")
        if isinstance(bat, (int, float)) and bat < BATTERY_WARN_PCT:
            warnings.append("battery_low")
        if isinstance(temp, (int, float)) and temp >= TEMP_WARN_C:
            warnings.append("temp_high")
        if mode in FLYING and not si.get("tof_valid"):
            warnings.append("height_unknown")
        if mode == "paused" and transition and transition.get("reason") in ("autonomy_timeout", "autonomy_disconnected"):
            warnings.append(transition["reason"])
        auto = self.autonomy.status()
        joined = [e.get("detail") or {} for x, e in zip(self.rec.event_t, self.rec.events)
                  if x <= t and e.get("type") == "gateway_autonomy_connected"]
        down_valid = bool(si.get("tof_valid"))
        return {
            "schema_version": SCHEMA_VERSION, "type": "drone_state", "at": now_iso(t), "time_source": "replay",
            "source": "replay", "connected": connected, "state_age_s": None if age is None else round(age, 3),
            "mode": mode, "authority": {"manual": "operator", "auto": "autonomy"}.get(mode, "none"), "allowed": [],
            "warnings": warnings, "battery_pct": bat, "height_m": si.get("tof_m") if down_valid else None,
            "height_source": "down_tof", "temp_high_c": temp,
            "attitude": {k: si.get(k) for k in ("roll_rad", "pitch_rad", "yaw_rad")},
            "front_tof": {"m": None, "valid": False, "reason": "not_recorded", "age_s": None},
            "down_tof": {"m": si.get("tof_m"), "valid": down_valid, "reason": si.get("tof_reason")},
            "pose": self.autonomy.pose_state(), "mission": self.autonomy.mission(),
            "autonomy": {**auto, "ground_origin": joined[-1].get("ground_origin") if joined else None, "mission_active": None},
            "video": {"enabled": self.video is not None, "on": video_on, "reason": video_reason, "ground_s": None,
                      "ground_off_s": None, "available": video_ok, "fps": vstats["fps"] if video_ok else None,
                      "frame_age_s": vstats["frame_age_s"] if video_on and vstats else None},
            "motor_time_s": si.get("motor_time_s"), "last_transition": transition,
            "operator_interventions": None, "autonomy_used": bool(self.rec.auto_events),
            "detection": self.detections.status(), "overlay": self.overlay.status(), "map": self.map.status(),
            "preflight": None,
            "replay": {"run_id": self.rec.run_id, "pos_s": round(self.clock.pos(), 2), "duration_s": round(self.clock.duration, 2),
                       "speed": self.clock.speed, "loop": self.clock.loop, "ended": self.clock.ended(),
                       "has_video": bool(self.rec.parts)},
        }


class _Detections:
    """기록된 탐지를 재생 시각까지만 보여 준다. 발견은 기록된 finding_id로 묶는다(실시간과 같은 묶음)."""

    def __init__(self, gw: ReplayGateway):
        self.gw, self.rec = gw, gw.rec

    def _upto(self) -> int:
        return bisect.bisect_right(self.rec.det_t, self.gw.clock.wall())

    def _groups(self, n: int) -> "OrderedDict[str, dict]":
        groups: OrderedDict[str, dict] = OrderedDict()
        for r in self.rec.dets[:n]:
            fid, people = r.get("finding_id"), r["event"].get("detections") or []
            if not fid or not people:
                continue
            seen = r.get("frame_time") or r.get("received_at")
            g = groups.get(fid)
            if g is None:
                g = groups[fid] = {"finding_id": fid, "first_seen": seen, "events": 0, "max_people": 0, "best": None,
                                   "review": "unreviewed", "review_at": None}
            g["last_seen"] = seen
            g["events"] += 1
            g["max_people"] = max(g["max_people"], len(people))
            best = max(people, key=lambda d: d["confidence"])
            if g["best"] is None or best["confidence"] > g["best"]["confidence"]:
                ev = r["event"]
                g["best"] = {"event_id": ev["event_id"], "frame_id": ev.get("frame_id"), "confidence": best["confidence"],
                             "people": len(people), "image": ev.get("image"), "detections": people, "seen_at": seen,
                             "drone": r.get("drone"), "photo_url": f"/api/detections/{ev['event_id']}.jpg" if r.get("photo") else None}
        upto = bisect.bisect_right(self.rec.review_t, self.gw.clock.wall())   # 재생 시각까지의 확인·오탐(마지막 것이 지금 값)
        for r in self.rec.reviews[:upto]:
            g = groups.get(r["finding_id"])
            if g is not None and r.get("review"):
                g["review"], g["review_at"] = r["review"], r.get("at")
        return groups

    def finding_at(self, fid: str, n: int) -> dict | None:
        return self._groups(n).get(fid)

    def snapshot(self) -> dict:
        findings = list(reversed(self._groups(self._upto()).values()))
        return {"schema_version": SCHEMA_VERSION, "detection_schema": "person-detection-event/1.0",
                "findings": findings, "status": self.status()}

    def status(self) -> dict:
        t, n = self.gw.clock.wall(), self._upto()
        recent = [x for x in self.rec.det_t[:n] if t - x <= DETECT_ALIVE_S]
        last = self.rec.det_t[n - 1] if n else None
        rows = self.rec.dets[:n]
        return {"alive": last is not None and t - last <= DETECT_ALIVE_S,
                "events_per_s": round(len(recent) / DETECT_ALIVE_S, 1) if recent else 0.0,
                "last_event_age_s": None if last is None else round(t - last, 2), "events": n,
                "person_events": sum(1 for r in rows if r["event"].get("detections")), "rejected": 0,
                "findings": len({r.get("finding_id") for r in rows if r.get("finding_id")}),
                "model": rows[-1]["event"].get("model") if rows else None}

    def receive(self, event):
        return 409, {"ok": False, "reason": "replay_source"}

    def put_photo(self, event_id: str, jpeg: bytes):
        return 409, {"ok": False, "reason": "replay_source"}

    def set_review(self, finding_id: str, review):
        return 409, {"ok": False, "reason": "replay_source"}

    def photo_path(self, event_id: str) -> str | None:
        if not UUID.match(str(event_id)):
            return None
        path = os.path.join(self.rec.dir, "detections", f"{event_id}.jpg")
        return path if os.path.isfile(path) else None


class _Map:
    def __init__(self, gw: ReplayGateway):
        self.gw, self.rec = gw, gw.rec
        self._cache: tuple[str, dict] | None = None

    def latest(self) -> dict | None:
        path = _at(self.rec.map_t, self.rec.map_files, self.gw.clock.wall())[0]
        if path is None:
            return None
        if self._cache and self._cache[0] == path:
            return self._cache[1]
        opener = gzip.open if path.endswith(".gz") else open
        with opener(path, "rt", encoding="utf-8") as f:
            snap = json.load(f)
        self._cache = (path, snap)
        return snap

    def status(self) -> dict:
        t = self.gw.clock.wall()
        line_t, lines = self.rec.map_lines
        line, at = _at(line_t, lines, t)
        line = line or {}
        return {"alive": at is not None and t - at <= MAP_ALIVE_S, "last_message_age_s": None if at is None else round(t - at, 2),
                "upstream": "ok" if at is not None else None, "mapping_state": "replay" if at is not None else None,
                "source_url": line.get("source_url"), "revision": line.get("revision"), "segment_id": line.get("segment_id"),
                "points": line.get("points") or 0, "points_total": line.get("points_total"), "unit": line.get("unit"),
                "updates": bisect.bisect_right(line_t, t), "rejected": 0}

    def receive(self, msg):
        return 409, {"ok": False, "reason": "replay_source"}


class _Autonomy:
    def __init__(self, gw: ReplayGateway):
        self.gw, self.rec = gw, gw.rec

    def _connected(self, t: float):
        conn = [(x, e) for x, e in zip(self.rec.auto_t, self.rec.auto_events) if x <= t and e.get("kind") in ("connected", "disconnected", "hello")]
        state, node = False, None
        for _, e in conn:
            if e["kind"] == "connected":
                state, node = True, None
            elif e["kind"] == "disconnected":
                state = False
            elif e["kind"] == "hello":
                node = {"name": e.get("name"), "version": e.get("version")}
        return state, node

    def pose_state(self) -> dict | None:
        t = self.gw.clock.wall()
        p, at = _at(self.rec.pose_t, self.rec.poses, t)
        if p is None:
            return None
        age = t - at
        connected = self._connected(t)[0]
        ok = age <= POSE_STALE_S and connected
        return {**{k: p.get(k) for k in ("x_m", "y_m", "z_m", "yaw_rad", "sigma_m")}, "frame": "map", "age_s": round(age, 2),
                "valid": ok, "reason": None if ok else ("autonomy_disconnected" if not connected else "stale")}

    def mission(self) -> dict | None:
        t = self.gw.clock.wall()
        if not self._connected(t)[0]:
            return None
        last = None
        for x, e in zip(self.rec.auto_t, self.rec.auto_events):
            if x > t:
                break
            if e.get("kind") == "mission_state":
                last = {k: e.get(k) for k in ("state", "phase", "goal", "progress", "reason")}
        return last

    def status(self) -> dict:
        t = self.gw.clock.wall()
        connected, node = self._connected(t)
        people = _at(self.rec.people_t, self.rec.peoples, t)[0]
        plist = (people or {}).get("people") or []
        return {"connected": connected, "node": node, "connected_s": None, "cmd_vel_hz": None, "last_cmd_vel_age_s": None,
                "path_revision": bisect.bisect_right(self.rec.path_t, t), "people_revision": bisect.bisect_right(self.rec.people_t, t),
                "people": None if people is None else {"confirmed": sum(not q.get("gone") for q in plist),
                                                       "gone": sum(bool(q.get("gone")) for q in plist),
                                                       "unconfirmed": len(people.get("unconfirmed") or [])},
                "coverage": (people or {}).get("coverage"), "rejected": 0}

    def snapshot(self) -> dict:
        t = self.gw.clock.wall()
        path = _at(self.rec.path_t, self.rec.paths, t)[0]
        people = _at(self.rec.people_t, self.rec.peoples, t)[0]
        return {"schema_version": SCHEMA_VERSION, "type": "autonomy", "at": now_iso(t), "pose": self.pose_state(),
                "path": path, "people": people, "status": self.status()}
