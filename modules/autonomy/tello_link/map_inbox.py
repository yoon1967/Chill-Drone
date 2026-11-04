"""3D 지도 받기: 지도 연결기(gateway_map_bridge.py)가 보낸 최신 점군·카메라 경로를 들고 있다가 화면에 내준다 (표준 라이브러리만).

지도 파트 계약: contracts/mapping/ONLINE_SPATIAL_V1.md·PHONE_SFM_RESULT_V1.md(지도 서버 `/api/live/run`).
연결기가 점을 줄이고 필요한 칸만 골라 `gateway-map/0.1` 형식으로 보낸다. 게이트웨이는 좌표를 바꾸지 않는다
(지도 표시 좌표 그대로, 단위는 `scale.unit`. 온라인 지도는 보통 `arbitrary` = m가 아님).
- kind `snapshot`: 새 지도 버전. 최신 하나만 메모리에 든다. 화면에는 WS로 알림(`map`)만 보내고 점군은 GET /api/map으로 읽게 한다.
- kind `status`: 연결기가 본 지도 서버 상태(꺼짐·지도 없음·동작 중). 버전이 안 바뀌어도 연결기가 살아 있다는 근거.
회차 폴더: map/latest.json(덮어씀), map_updates.jsonl(버전마다 한 줄: 시각·버전·점 수·출처).
"""

from __future__ import annotations

import json
import math
import os
import threading
import time

from mode_arbiter import SCHEMA_VERSION
from tello_link import now_iso

SCHEMA = "gateway-map/0.1"
MAX_POINTS = 60_000          # 연결기 기본은 20,000. 지도 쪽 최대 융합 셀 50,000보다 조금 넉넉히
MAX_CAMERAS = 20_000
ALIVE_S = 5.0                # 연결기 소식이 이 시간 안에 왔으면 동작 중
UPSTREAM = ("ok", "no_map", "unreachable", "error")


class MapInbox:
    def __init__(self, *, run_dir: str | None = None, emit=None, clock=time.monotonic):
        self.run_dir = run_dir
        self.emit = emit or (lambda msg: None)
        self.clock = clock
        self._lock = threading.Lock()
        self._latest: dict | None = None
        self._key = None                     # (session, segment, revision): 같은 버전은 다시 저장하지 않는다
        self._last_mono: float | None = None
        self._upstream: dict | None = None
        self.updates = 0
        self.rejected = 0
        if run_dir:
            os.makedirs(os.path.join(run_dir, "map"), exist_ok=True)

    def receive(self, msg) -> tuple[int, dict]:
        """(HTTP 상태, 응답). 201 새 버전, 200 상태·같은 버전, 400 형식 오류, 500 저장 실패."""
        try:
            kind = _check(msg)
        except (ValueError, TypeError, KeyError) as e:
            with self._lock:
                self.rejected += 1
            return 400, {"ok": False, "reason": "invalid_map", "detail": str(e)}
        now = self.clock()
        with self._lock:
            self._last_mono = now
            self._upstream = {k: msg.get(k) for k in ("upstream", "source_url", "mapping_state", "detail")}
            if kind == "status":
                return 200, {"ok": True}
            src = msg["source"]
            key = (src.get("session_id"), src.get("segment_id"), src.get("revision"))
            if key == self._key:
                return 200, {"ok": True, "duplicate": True}
            self._key = key
            self.updates += 1
            snap = {**msg, "received_at": _iso(time.time())}
            self._latest = snap
            line = {"received_at": snap["received_at"], "revision": src.get("revision"),
                    "session_id": src.get("session_id"), "segment_id": src.get("segment_id"),
                    "points": len(msg["points"]), "points_total": msg.get("points_total"),
                    "cameras": len(msg["cameras"]), "unit": msg["scale"].get("unit"), "source_url": msg.get("source_url")}
        try:
            self._write(snap, line)
        except OSError as e:
            return 500, {"ok": False, "reason": "store_failed", "detail": str(e)}
        self.emit({"schema_version": SCHEMA_VERSION, "type": "map", "at": snap["received_at"], "revision": src.get("revision"),
                   "session_id": src.get("session_id"), "segment_id": src.get("segment_id"), "points": len(msg["points"])})
        return 201, {"ok": True, "duplicate": False, "revision": src.get("revision")}

    def latest(self) -> dict | None:
        with self._lock:
            return self._latest

    def status(self) -> dict:
        now = self.clock()
        with self._lock:
            last, up, snap = self._last_mono, self._upstream or {}, self._latest
            src = (snap or {}).get("source") or {}
            return {
                "alive": last is not None and now - last <= ALIVE_S,
                "last_message_age_s": None if last is None else round(now - last, 2),
                "upstream": up.get("upstream"),
                "mapping_state": up.get("mapping_state"),
                "source_url": up.get("source_url"),
                "revision": src.get("revision"),
                "segment_id": src.get("segment_id"),
                "points": len(snap["points"]) if snap else 0,
                "points_total": snap.get("points_total") if snap else None,
                "unit": snap["scale"].get("unit") if snap else None,
                "updates": self.updates,
                "rejected": self.rejected,
            }

    def _write(self, snap: dict, line: dict) -> None:
        if not self.run_dir:
            return
        path = os.path.join(self.run_dir, "map", "latest.json")
        with open(path + ".tmp", "w", encoding="utf-8") as f:
            json.dump(snap, f, ensure_ascii=False, separators=(",", ":"))
        os.replace(path + ".tmp", path)
        with open(os.path.join(self.run_dir, "map_updates.jsonl"), "a", encoding="utf-8") as f:
            f.write(json.dumps(line, ensure_ascii=False) + "\n")


def _check(msg) -> str:
    if not isinstance(msg, dict) or msg.get("schema_version") != SCHEMA:
        raise ValueError("unsupported_schema_version")
    kind = msg.get("kind")
    if kind not in ("snapshot", "status"):
        raise ValueError("invalid_kind")
    if msg.get("upstream") not in UPSTREAM:
        raise ValueError("invalid_upstream")
    if kind == "status":
        return kind
    if not isinstance(msg.get("source"), dict) or not isinstance(msg.get("scale"), dict):
        raise ValueError("missing_source_or_scale")
    points, cameras = msg.get("points"), msg.get("cameras")
    if not isinstance(points, list) or len(points) > MAX_POINTS:
        raise ValueError("invalid_points")
    for p in points:   # [x, y, z, r, g, b]
        if (not isinstance(p, list) or len(p) != 6 or not all(_num(v) for v in p[:3])
                or not all(isinstance(c, int) and 0 <= c <= 255 for c in p[3:])):
            raise ValueError("invalid_point")
    if not isinstance(cameras, list) or len(cameras) > MAX_CAMERAS:
        raise ValueError("invalid_cameras")
    for c in cameras:  # [x, y, z] 카메라 중심(표시 좌표), 시간 순
        if not isinstance(c, list) or len(c) != 3 or not all(_num(v) for v in c):
            raise ValueError("invalid_camera")
    up = msg.get("up")
    if up is not None and (not isinstance(up, list) or len(up) != 3 or not all(_num(v) for v in up)):
        raise ValueError("invalid_up")
    return kind


def _num(v) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def _iso(wall: float) -> str:
    return now_iso(wall)
