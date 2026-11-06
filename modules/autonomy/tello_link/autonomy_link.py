"""자율 비행 연결(/ws/autonomy): 자율이 보낸 위치·경로·찾은 사람을 들고, 자율에 보낼 메시지를 만든다 (표준 라이브러리만).

계약: contracts/api/AUTONOMY_BRIDGE_V0_1.md 3·4장(브랜치 docs/drone-control-contracts).
판정(속도 명령을 기체로 보낼지, 모드 전환, 끊김 0.5초·2초)은 ModeArbiter가 하고, 여기서는
받은 메시지 확인·보관·기록과 보낼 메시지 모양만 맡는다. WebSocket은 서버(gateway_server.py)가 연다.

- 한 번에 자율 하나만 받는다. 두 번째 연결은 `busy`.
- `pose`·`path`·`people`은 `frame: "map"`(자율 지도: 켠 자리 원점, x 앞·y 왼쪽·z 위, m, yaw 반시계 +). 좌표를 바꾸지 않는다.
- `people`은 0.1.0 초안에 없던 것으로, 자율 비행 쪽 `people.json`(찾은 사람) 모양을 그대로 받는다.
- 회차 폴더 autonomy/: events.jsonl(연결·control·ack·request·임무 상태 변화·찾은 사람 수 변화),
  pose.jsonl(받는 대로 전부), path.jsonl·people.jsonl(바뀔 때마다 전체, 기록 재생용), latest.json(경로·찾은 사람 최신, 덮어씀).
"""

from __future__ import annotations

import json
import math
import os
import threading
import time
from collections import deque

from mode_arbiter import SCHEMA_VERSION
from tello_link import now_iso

FROM_AUTONOMY = ("hello", "cmd_vel", "mission_state", "ack", "request", "pose", "path", "people")
POSE_STALE_S = 1.0          # 이보다 오래된 위치는 화면에 "오래됨"으로 보인다
POSE_LOG_S = 0.0            # 위치는 받는 대로 다 남긴다(기록 재생용, 10Hz면 시간당 약 5 MB)
MAX_PATH_POINTS = 5000
MAX_PEOPLE = 200
FLOAT_KEYS = ("baro", "agx", "agy", "agz")   # Tello 원문에서 소수 둘째 자리까지 오는 값


def _finite(v) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def _xyz(p) -> list[float]:
    if not isinstance(p, (list, tuple)) or len(p) not in (2, 3) or not all(_finite(v) for v in p):
        raise ValueError("bad_point")
    return [float(p[0]), float(p[1]), float(p[2]) if len(p) == 3 else None]


def _person(q: dict) -> dict:
    if not isinstance(q, dict) or not _finite(q.get("x")) or not _finite(q.get("y")):
        raise ValueError("bad_person")
    num = lambda k: float(q[k]) if _finite(q.get(k)) else None   # noqa: E731
    return {"x_m": float(q["x"]), "y_m": float(q["y"]), "conf": num("conf"), "hits": int(q["hits"]) if _finite(q.get("hits")) else None,
            "gone": bool(q.get("gone")), "t_first": num("t_first"), "top_m": num("top")}


def tello_raw_text(raw: dict) -> str:
    """TelloLink가 나눠 둔 원시값을 Tello 상태 문자열 모양으로 되돌린다(자율 쪽 해석기가 원문을 읽는다)."""
    parts = []
    for k, v in raw.items():
        if isinstance(v, list):
            v = ",".join(str(x) for x in v)
        elif isinstance(v, float):
            v = f"{v:.2f}"
        parts.append(f"{k}:{v}")
    return ";".join(parts) + ";"


class AutonomyLink:
    def __init__(self, *, run_dir: str | None = None, clock=time.monotonic):
        self.clock = clock
        self.run_dir = os.path.join(run_dir, "autonomy") if run_dir else None
        if self.run_dir:
            os.makedirs(self.run_dir, exist_ok=True)
        self._lock = threading.Lock()
        self.session: str | None = None
        self.node: dict | None = None
        self._since: float | None = None
        self._cmd_times: deque = deque(maxlen=40)
        self._pose: dict | None = None
        self._pose_mono: float | None = None
        self._pose_logged = -1e9
        self._path: dict | None = None
        self._people: dict | None = None
        self.path_revision = 0
        self.people_revision = 0
        self.rejected = 0
        self._mission_key = None

    # ---------- 연결 ----------
    def open(self, session: str) -> bool:
        with self._lock:
            if self.session is not None:
                return False
            self.session, self.node, self._since = session, None, self.clock()
            self._cmd_times.clear()
            # 새 자율은 새 지도(원점이 다를 수 있다). 지난 연결의 위치·경로·찾은 사람은 회차 폴더에만 남기고 화면에서는 비운다.
            self._pose = self._pose_mono = self._path = self._people = None
            self._mission_key = None
            self.path_revision += 1
            self.people_revision += 1
        self._log("events", {"kind": "connected", "session": session})
        return True

    def close(self, session: str) -> bool:
        """그 연결이 지금 자율이면 닫고 True. 위치·경로·찾은 사람은 마지막 값을 남긴다(화면이 오래됨으로 보인다)."""
        with self._lock:
            if self.session != session:
                return False
            self.session, self._since = None, None
        self._log("events", {"kind": "disconnected", "session": session})
        return True

    @property
    def connected(self) -> bool:
        return self.session is not None

    # ---------- 받기 ----------
    def receive(self, msg) -> str:
        """메시지 종류를 돌려준다. 확인·보관할 것은 여기서 하고, 판정할 것(cmd_vel·mission_state·ack·request)은
        게이트웨이가 판정기에 넘긴다. 형식이 틀리면 ValueError."""
        if not isinstance(msg, dict) or msg.get("schema_version") != SCHEMA_VERSION or msg.get("type") not in FROM_AUTONOMY:
            with self._lock:
                self.rejected += 1
            raise ValueError("unsupported_message")
        kind = msg["type"]
        try:
            if kind == "hello":
                with self._lock:
                    self.node = {"name": str(msg.get("node") or msg.get("name") or "")[:80] or None,
                                 "version": str(msg.get("version") or "")[:40] or None}
                self._log("events", {"kind": "hello", **self.node})
            elif kind == "cmd_vel":
                with self._lock:
                    self._cmd_times.append(self.clock())
            elif kind == "pose":
                self._receive_pose(msg)
            elif kind == "path":
                self._receive_path(msg)
            elif kind == "people":
                self._receive_people(msg)
            elif kind == "mission_state":
                key = (msg.get("state"), msg.get("phase"), msg.get("reason"))
                if key != self._mission_key:
                    self._mission_key = key
                    self._log("events", {"kind": "mission_state", **{k: msg.get(k) for k in ("state", "phase", "progress", "reason", "goal")}})
            elif kind in ("ack", "request"):
                self._log("events", {"kind": kind, **{k: msg.get(k) for k in ("id", "ok", "reason", "name")}})
        except (ValueError, TypeError, KeyError):
            with self._lock:
                self.rejected += 1
            raise ValueError("invalid_" + kind) from None
        return kind

    def _frame(self, msg) -> str:
        frame = msg.get("frame", "map")
        if frame != "map":
            raise ValueError("frame_must_be_map")
        return frame

    def _receive_pose(self, msg) -> None:
        self._frame(msg)
        if not all(_finite(msg.get(k)) for k in ("x_m", "y_m", "yaw_rad")):
            raise ValueError("bad_pose")
        pose = {"x_m": float(msg["x_m"]), "y_m": float(msg["y_m"]),
                "z_m": float(msg["z_m"]) if _finite(msg.get("z_m")) else None,
                "yaw_rad": float(msg["yaw_rad"]),
                "sigma_m": float(msg["sigma_m"]) if _finite(msg.get("sigma_m")) else None, "frame": "map"}
        now = self.clock()
        with self._lock:
            self._pose, self._pose_mono = pose, now
            log = now - self._pose_logged >= POSE_LOG_S
            if log:
                self._pose_logged = now
        if log:
            self._log("pose", pose)

    def _receive_path(self, msg) -> None:
        self._frame(msg)
        pts = msg.get("points")
        if not isinstance(pts, list) or len(pts) > MAX_PATH_POINTS:
            raise ValueError("bad_path")
        points = [_xyz(p) for p in pts]
        with self._lock:
            self.path_revision += 1
            self._path = {"points": points, "frame": "map", "revision": self.path_revision, "at": now_iso()}
            path = self._path
        self._log("path", path)
        self._save_latest()

    def _receive_people(self, msg) -> None:
        self._frame(msg)
        people, unconfirmed = msg.get("people", []), msg.get("unconfirmed", [])
        if not isinstance(people, list) or not isinstance(unconfirmed, list) or len(people) + len(unconfirmed) > MAX_PEOPLE:
            raise ValueError("bad_people")
        value = {"people": [_person(q) for q in people], "unconfirmed": [_person(q) for q in unconfirmed], "frame": "map",
                 "coverage": msg.get("coverage") if _finite(msg.get("coverage")) else None,
                 "explored_m2": msg.get("explored_m2") if _finite(msg.get("explored_m2")) else None}
        with self._lock:
            before = self._people
            changed = before is None or (len(before["people"]), sum(p["gone"] for p in before["people"]), len(before["unconfirmed"])) != \
                (len(value["people"]), sum(p["gone"] for p in value["people"]), len(value["unconfirmed"]))
            if before is None or value["people"] != before["people"] or value["unconfirmed"] != before["unconfirmed"]:
                self.people_revision += 1
            new = before is None or value["people"] != before["people"] or value["unconfirmed"] != before["unconfirmed"]
            self._people = {**value, "revision": self.people_revision, "at": now_iso()}
            people = self._people
        if new:
            self._log("people", people)
        if changed:
            self._log("events", {"kind": "people", "confirmed": len(value["people"]),
                                 "gone": sum(p["gone"] for p in value["people"]), "unconfirmed": len(value["unconfirmed"])})
        self._save_latest()

    # ---------- 화면으로 ----------
    def pose_state(self) -> dict | None:
        with self._lock:
            pose, t = self._pose, self._pose_mono
        if pose is None:
            return None
        age = self.clock() - t
        return {**pose, "age_s": round(age, 2), "valid": age <= POSE_STALE_S and self.connected,
                "reason": None if age <= POSE_STALE_S and self.connected else ("autonomy_disconnected" if not self.connected else "stale")}

    def status(self) -> dict:
        now = self.clock()
        with self._lock:
            times = list(self._cmd_times)
            people = self._people
            hz = None
            if len(times) >= 2 and now - times[-1] <= 1.0 and times[-1] > times[0]:
                hz = round((len(times) - 1) / (times[-1] - times[0]), 1)
            return {
                "connected": self.session is not None,
                "node": self.node,
                "connected_s": None if self._since is None else round(now - self._since, 1),
                "cmd_vel_hz": hz if self.session else None,
                "last_cmd_vel_age_s": round(now - times[-1], 2) if times else None,
                "path_revision": self.path_revision,
                "people_revision": self.people_revision,
                "people": None if people is None else {"confirmed": sum(not p["gone"] for p in people["people"]),
                                                       "gone": sum(p["gone"] for p in people["people"]),
                                                       "unconfirmed": len(people["unconfirmed"])},
                "coverage": people.get("coverage") if people else None,
                "rejected": self.rejected,
            }

    def snapshot(self) -> dict:
        """GET /api/autonomy: 경로·찾은 사람 전체(상태에는 버전만 싣는다)."""
        with self._lock:
            path, people = self._path, self._people
        return {"schema_version": SCHEMA_VERSION, "type": "autonomy", "at": now_iso(), "pose": self.pose_state(),
                "path": path, "people": people, "status": self.status()}

    # ---------- 자율로 ----------
    @staticmethod
    def tello_state_message(sample: dict) -> dict | None:
        raw = (sample or {}).get("raw")
        if not raw:
            return None
        return {"schema_version": SCHEMA_VERSION, "type": "tello_state", "at": sample.get("received_at") or now_iso(),
                "raw": tello_raw_text(raw), "age_s": sample.get("age_s")}

    @staticmethod
    def flight_status_message(snap: dict, connected: bool) -> dict:
        return {"schema_version": SCHEMA_VERSION, "type": "flight_status", "at": now_iso(), "mode": snap["mode"],
                "authority": snap["authority"], "flying": snap["flying"], "connected": connected,
                "landing_reason": snap["landing_reason"],
                "reason": (snap.get("last_transition") or {}).get("reason")}

    # ---------- 기록 ----------
    def event(self, kind: str, **detail) -> None:
        self._log("events", {"kind": kind, **detail})

    def _log(self, name: str, row: dict) -> None:
        if not self.run_dir:
            return
        try:
            with open(os.path.join(self.run_dir, name + ".jsonl"), "a", encoding="utf-8") as f:
                f.write(json.dumps({"at": now_iso(), **row}, ensure_ascii=False) + "\n")
        except OSError:
            pass   # 기록 실패로 비행 연결을 끊지 않는다

    def _save_latest(self) -> None:
        if not self.run_dir:
            return
        with self._lock:
            data = {"path": self._path, "people": self._people}
        tmp = os.path.join(self.run_dir, "latest.json.tmp")
        try:
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(data, f, ensure_ascii=False)
            os.replace(tmp, os.path.join(self.run_dir, "latest.json"))
        except OSError:
            pass
