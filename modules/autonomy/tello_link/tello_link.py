"""Tello EDU SDK 연결 모듈 (표준 라이브러리만 사용).

명령(UDP 8889) 송수신, 상태 패킷(UDP 8890) 수신·파싱, 회차별 로그를 맡는다.
ROS나 웹에 의존하지 않으므로 ROS 2 드라이버 노드나 점검 서버 안에서 그대로 import해 쓴다.

안전 기본값: 조회·세션 명령과 정지 명령(land, emergency, stop)만 허용한다.
이륙·이동·rc는 allow_flight=True, 기체에 저장되는 설정(ap, wifi 등)은 allow_config=True일 때만 보낸다.
"""

from __future__ import annotations

import json
import math
import os
import socket
import threading
import time
import uuid
from collections import deque
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone

DEFAULT_HOST = "192.168.10.1"
DEFAULT_CMD_PORT = 8889
DEFAULT_STATE_PORT = 8890
DEFAULT_VIDEO_PORT = 11111

QUERY_COMMANDS = {
    "battery?", "speed?", "time?", "wifi?", "sdk?", "sn?",
    "height?", "temp?", "attitude?", "baro?", "acceleration?", "tof?",
}
# RoboMaster TT 확장 키트(오픈소스 컨트롤러, 공장 펌웨어)에 묻는 조회 명령
EXT_QUERY_COMMANDS = {"ext tof?", "ext version?"}
# SDK 3.0 문서는 범위 밖을 8192라고 적었지만, 실기체(esp32v1.0.0.12)는 8190·8191을 보냈다(2026-10-27 실측).
EXT_TOF_OUT_OF_RANGE_MM = 8190
SESSION_COMMANDS = {"command", "streamon", "streamoff"}
STOP_COMMANDS = {"land", "emergency", "stop"}
# 기체에 남는 설정. 잘못 보내면 전원 버튼 5초 초기화가 필요할 수 있다.
CONFIG_COMMANDS = {
    "ap", "wifi", "multiwifi", "mon", "moff", "mdirection", "port",
    "setfps", "setbitrate", "setresolution", "downvision", "speed",
}

# 단위가 문서마다 다르게 적혀 있어 실측 전에는 SI로 바꾸지 않는 필드
UNVERIFIED_UNIT_FIELDS = ("vgx", "vgy", "vgz", "baro", "agx", "agy", "agz")
TOF_NO_RETURN_CM = 6553   # 반사 없음으로 관찰되는 값(실측 확인 필요)
TOF_MIN_CM = 10           # 최소 거리 이하·바닥 접지 시 관찰되는 값(실측 확인 필요)


def now_iso(t: float | None = None) -> str:
    return datetime.fromtimestamp(time.time() if t is None else t, timezone.utc).isoformat(timespec="milliseconds")


def classify(cmd: str) -> str:
    """명령을 query / session / stop / ext / config / flight 중 하나로 분류한다."""
    text = " ".join(cmd.strip().lower().split())
    head = text.split()[0] if text else ""
    if text in QUERY_COMMANDS or text in EXT_QUERY_COMMANDS:
        return "query"
    if head == "ext":
        # 확장 키트(오픈소스 컨트롤러) 명령. 표시용 LED·매트릭스만 허용하고 나머지는 설정으로 막는다.
        parts = text.split()
        if len(parts) >= 2 and parts[1] in ("led", "mled") and parts[2:3] not in (["sg"], ["sc"]):
            return "ext"
        return "config"
    if text in SESSION_COMMANDS:
        return "session"
    if text in STOP_COMMANDS:
        return "stop"
    if head in CONFIG_COMMANDS:
        return "config"
    return "flight"


def parse_ext_tof(response: str | None) -> dict:
    """`EXT tof?` 응답(`tof 481`, mm)을 전방 거리로 바꾼다. 범위 밖·오류는 0이 아니라 무효로 둔다."""
    if response is None:
        return {"front_tof_m": None, "valid": False, "reason": "no_response", "raw": None}
    parts = response.strip().split()
    if len(parts) != 2 or parts[0].lower() != "tof":
        low = response.lower()
        if low.startswith("unknown command"):
            reason = "unknown_command"  # 실기체에서 드물게(240회 중 2회) 이렇게 답했다
        elif low.startswith("error"):
            reason = "error_response"
        else:
            reason = "bad_format"
        return {"front_tof_m": None, "valid": False, "reason": reason, "raw": response}
    try:
        mm = int(float(parts[1]))
    except ValueError:
        return {"front_tof_m": None, "valid": False, "reason": "bad_format", "raw": response}
    if mm >= EXT_TOF_OUT_OF_RANGE_MM:
        return {"front_tof_m": None, "valid": False, "reason": "out_of_range", "raw": response}
    return {"front_tof_m": mm / 1000.0, "valid": True, "reason": None, "raw": response}


@dataclass
class CommandResult:
    cmd: str
    category: str
    sent_at: str | None
    response: str | None
    ok: bool
    valid: bool
    reason: str | None
    rtt_ms: float | None
    blocked: bool = False
    source: str | None = None  # 입력 출처: keyboard, script, safety, check 등

    def to_dict(self) -> dict:
        return asdict(self)


@dataclass
class StateSample:
    received_at: str
    received_mono: float
    time_source: str
    raw: dict
    si: dict
    valid: bool
    reasons: list = field(default_factory=list)

    def to_dict(self, now_mono: float | None = None) -> dict:
        d = asdict(self)
        d.pop("received_mono")
        if now_mono is not None:
            d["age_s"] = round(now_mono - self.received_mono, 3)
        return d


def _num(text: str):
    try:
        value = float(text)
    except ValueError:
        return text
    return int(value) if value.is_integer() and "." not in text else value


def parse_state(packet: str, received: float | None = None, received_mono: float | None = None) -> StateSample:
    """`pitch:0;roll:0;...;\\r\\n` 형식의 상태 패킷을 원시값과 SI 값으로 나눈다."""
    received = time.time() if received is None else received
    received_mono = time.monotonic() if received_mono is None else received_mono
    raw: dict = {}
    reasons: list[str] = []
    for part in packet.strip().strip(";").split(";"):
        if ":" not in part:
            continue
        key, value = part.split(":", 1)
        key = key.strip()
        if key == "mpry":
            raw[key] = [_num(v) for v in value.split(",")]
        else:
            raw[key] = _num(value.strip())

    required = ("pitch", "roll", "yaw", "h", "bat", "tof", "templ", "temph")
    missing = [k for k in required if not isinstance(raw.get(k), (int, float))]
    if missing:
        reasons.append("missing_or_non_numeric:" + ",".join(missing))

    def get(key):
        v = raw.get(key)
        return v if isinstance(v, (int, float)) else None

    si: dict = {
        "roll_rad": None if get("roll") is None else math.radians(get("roll")),
        "pitch_rad": None if get("pitch") is None else math.radians(get("pitch")),
        "yaw_rad": None if get("yaw") is None else math.radians(get("yaw")),
        "height_m": None if get("h") is None else get("h") / 100.0,
        "battery_pct": get("bat"),
        "temp_low_c": get("templ"),
        "temp_high_c": get("temph"),
        "motor_time_s": get("time"),
    }
    tof = get("tof")
    if tof is None:
        si["tof_m"], si["tof_valid"], si["tof_reason"] = None, False, "missing"
    elif tof >= TOF_NO_RETURN_CM:
        si["tof_m"], si["tof_valid"], si["tof_reason"] = None, False, "no_return"
    elif tof <= TOF_MIN_CM:
        si["tof_m"], si["tof_valid"], si["tof_reason"] = None, False, "below_min_or_on_ground"
    else:
        si["tof_m"], si["tof_valid"], si["tof_reason"] = tof / 100.0, True, None
    si["unverified_unit_fields"] = [k for k in UNVERIFIED_UNIT_FIELDS if k in raw]

    return StateSample(
        received_at=now_iso(received),
        received_mono=received_mono,
        time_source="server_receive",
        raw=raw,
        si=si,
        valid=not missing,
        reasons=reasons,
    )


class RunLog:
    """회차 폴더에 manifest·commands·sensor·events·metrics를 남긴다."""

    def __init__(self, root: str, run_id: str, manifest: dict):
        self.run_id = run_id
        self.dir = os.path.join(root, run_id)
        os.makedirs(self.dir, exist_ok=True)
        self._lock = threading.Lock()
        self._files = {
            name: open(os.path.join(self.dir, f"{name}.jsonl"), "a", encoding="utf-8")
            for name in ("commands", "sensor", "events")
        }
        self._write_json("manifest.json", manifest)

    def _write_json(self, name: str, data: dict) -> None:
        with open(os.path.join(self.dir, name), "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)

    def append(self, stream: str, record: dict) -> None:
        line = json.dumps(record, ensure_ascii=False)
        with self._lock:
            f = self._files.get(stream)
            if f is None:
                return
            f.write(line + "\n")
            f.flush()

    def close(self, metrics: dict) -> None:
        self._write_json("metrics.json", metrics)
        with self._lock:
            for f in self._files.values():
                f.close()
            self._files = {}


class TelloLink:
    def __init__(
        self,
        host: str = DEFAULT_HOST,
        cmd_port: int = DEFAULT_CMD_PORT,
        local_cmd_port: int = DEFAULT_CMD_PORT,
        state_port: int = DEFAULT_STATE_PORT,
        bind_ip: str = "0.0.0.0",
        allow_flight: bool = False,
        allow_config: bool = False,
        default_timeout: float = 7.0,
        log_root: str | None = None,
        run_id: str | None = None,
        source: str = "tello",
    ):
        self.host = host
        self.cmd_port = cmd_port
        self.local_cmd_port = local_cmd_port
        self.state_port = state_port
        self.bind_ip = bind_ip
        self.allow_flight = allow_flight
        self.allow_config = allow_config
        self.default_timeout = default_timeout
        self.source = source
        self.run_id = run_id or datetime.now().strftime("tello_%Y%m%d_%H%M%S_") + uuid.uuid4().hex[:6]
        self.log_root = log_root
        self.log: RunLog | None = None

        self._cmd_sock: socket.socket | None = None
        self._state_sock: socket.socket | None = None
        self._cmd_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._stop = threading.Event()
        self._state_thread: threading.Thread | None = None

        self._latest_state: StateSample | None = None
        self._state_times: deque = deque(maxlen=50)
        self.state_count = 0
        self.state_bad_count = 0
        self.history: deque = deque(maxlen=200)
        self._counts = {"sent": 0, "blocked": 0, "timeout": 0}
        self._rtts: list[float] = []
        self.sdk_mode = False

    # ---------- 수명 ----------
    def start(self) -> "TelloLink":
        self._cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._cmd_sock.bind((self.bind_ip, self.local_cmd_port))
        self._state_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._state_sock.bind((self.bind_ip, self.state_port))
        self._state_sock.settimeout(0.5)
        if self.log_root:
            self.manifest = self._manifest()
            self.log = RunLog(self.log_root, self.run_id, self.manifest)
        self.event("link_started", {"host": self.host, "cmd_port": self.cmd_port, "state_port": self.state_port})
        self._stop.clear()
        self._state_thread = threading.Thread(target=self._state_loop, name="tello-state", daemon=True)
        self._state_thread.start()
        return self

    def close(self) -> None:
        self._stop.set()
        if self._state_thread:
            self._state_thread.join(timeout=2)
        for s in (self._cmd_sock, self._state_sock):
            if s:
                s.close()
        self.event("link_closed", {})
        if self.log:
            self.log.close(self.metrics())
            self.log = None

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.close()

    def _manifest(self) -> dict:
        return {
            "run_id": self.run_id,
            "created_at": now_iso(),
            "tool": "modules/autonomy/tello_link",
            "jira": "S15P31B205-156",
            "source": self.source,
            "drone": {"host": self.host, "cmd_port": self.cmd_port, "state_port": self.state_port},
            "allow_flight": self.allow_flight,
            "allow_config": self.allow_config,
            "time_source": "server_receive",
            "units": {
                "si": "m, rad, °C, %, s",
                "raw": "Tello SDK 원시값 (h·tof cm, 자세 deg)",
                "unverified": list(UNVERIFIED_UNIT_FIELDS),
            },
            "files": {
                "commands": "commands.jsonl",
                "sensor": "sensor.jsonl (상태 패킷)",
                "events": "events.jsonl",
                "metrics": "metrics.json (종료 시)",
                "pose": None,
            },
            # 사람이 조종한 회차는 자율 성공으로 세지 않는다. 조작을 시작하면 true로 바뀐다.
            "operator_controlled": False,
            "notes": "pose는 만들지 않는다. 운용자가 보낸 명령은 commands.jsonl에 source와 함께 모두 남는다.",
        }

    def update_manifest(self, **fields) -> None:
        if not self.log:
            return
        self.manifest.update(fields)
        self.log._write_json("manifest.json", self.manifest)

    def event(self, kind: str, detail: dict) -> None:
        if self.log:
            self.log.append("events", {"t": now_iso(), "type": kind, "detail": detail})

    # ---------- 명령 ----------
    def is_allowed(self, cmd: str) -> tuple[bool, str | None]:
        category = classify(cmd)
        if category == "flight" and not self.allow_flight:
            return False, "flight_blocked"
        if category == "config" and not self.allow_config:
            return False, "config_blocked"
        return True, None

    def send(self, cmd: str, timeout: float | None = None, source: str | None = None) -> CommandResult:
        cmd = cmd.strip()
        category = classify(cmd)
        allowed, reason = self.is_allowed(cmd)
        if not allowed:
            result = CommandResult(cmd, category, None, None, False, False, reason, None, blocked=True, source=source)
            self._record(result)
            return result
        if self._cmd_sock is None:
            raise RuntimeError("start()를 먼저 호출해야 한다")

        timeout = self.default_timeout if timeout is None else timeout
        with self._cmd_lock:
            self._drain()
            sent = time.time()
            t0 = time.perf_counter()  # Windows에서 monotonic은 약 15.6ms 단위라 왕복 시간 측정에 쓰지 않는다
            self._cmd_sock.sendto(cmd.encode("utf-8"), (self.host, self.cmd_port))
            response, reason = self._recv_response(timeout)
            rtt = (time.perf_counter() - t0) * 1000.0

        if response is None:
            result = CommandResult(cmd, category, now_iso(sent), None, False, False, reason, None, source=source)
        else:
            lowered = response.lower()
            if category == "query":
                ok = bool(response) and not lowered.startswith("error")
            else:
                ok = lowered == "ok"
            result = CommandResult(cmd, category, now_iso(sent), response, ok, True,
                                   None if ok else "error_response", round(rtt, 1), source=source)
        if cmd == "command":
            self.sdk_mode = result.ok
        self._record(result)
        return result

    def rc(self, lr: int, fb: int, ud: int, yaw: int, source: str | None = None) -> CommandResult:
        """rc 명령은 응답이 없으므로 보내기만 한다. allow_flight=True일 때만 동작한다."""
        vals = [max(-100, min(100, int(v))) for v in (lr, fb, ud, yaw)]
        cmd = "rc " + " ".join(str(v) for v in vals)
        if not self.allow_flight:
            result = CommandResult(cmd, "flight", None, None, False, False, "flight_blocked", None,
                                   blocked=True, source=source)
            self._record(result)
            return result
        with self._cmd_lock:
            sent = time.time()
            self._cmd_sock.sendto(cmd.encode("utf-8"), (self.host, self.cmd_port))
        result = CommandResult(cmd, "flight", now_iso(sent), None, True, True, "no_response_expected", None,
                               source=source)
        self._record(result)
        return result

    def emergency_now(self, source: str | None = None) -> CommandResult:
        """`emergency`를 잠금을 기다리지 않고 바로 보낸다(모터 즉시 정지).

        이륙처럼 응답을 오래 기다리는 명령이 잠금을 잡고 있어도 막히지 않게 한다.
        응답은 기다리지 않는다. 늦게 온 `ok`는 다음 명령 전에 버려진다.
        """
        sent = time.time()
        for _ in range(2):  # UDP 유실에 대비해 두 번 보낸다
            self._cmd_sock.sendto(b"emergency", (self.host, self.cmd_port))
        result = CommandResult("emergency", "stop", now_iso(sent), None, True, True, "sent_without_wait", None,
                               source=source)
        self.event("emergency", {"source": source})
        self._record(result)
        return result

    def _drain(self) -> None:
        """늦게 도착한 이전 응답을 버려서 다음 응답과 섞이지 않게 한다."""
        self._cmd_sock.setblocking(False)
        try:
            while True:
                data = self._cmd_sock.recv(2048)
                self.event("stale_response_dropped", {"data": data.decode("utf-8", "replace").strip()})
        except (BlockingIOError, OSError):
            pass
        finally:
            self._cmd_sock.setblocking(True)

    def _recv_response(self, timeout: float) -> tuple[str | None, str | None]:
        self._cmd_sock.settimeout(timeout)
        try:
            data, _addr = self._cmd_sock.recvfrom(2048)
        except socket.timeout:
            return None, "timeout"
        except ConnectionResetError:
            # Windows는 상대 포트가 닫혀 있으면 ICMP를 이 예외로 돌려준다.
            return None, "connection_reset"
        except OSError:
            return None, "socket_closed"
        try:
            return data.decode("utf-8").strip(), None
        except UnicodeDecodeError:
            return None, "bad_packet"

    def _record(self, result: CommandResult) -> None:
        d = result.to_dict()
        d["recorded_at"] = now_iso()
        self.history.append(d)
        self._counts["blocked" if result.blocked else "sent"] += 1
        if result.reason == "timeout":
            self._counts["timeout"] += 1
        if result.rtt_ms is not None:
            self._rtts.append(result.rtt_ms)
        if self.log:
            self.log.append("commands", d)

    # ---------- 상태 ----------
    def _state_loop(self) -> None:
        while not self._stop.is_set():
            try:
                data, _addr = self._state_sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                if self._stop.is_set():
                    break
                continue
            t, mono = time.time(), time.monotonic()
            try:
                text = data.decode("ascii")
            except UnicodeDecodeError:
                self.state_bad_count += 1
                if self.log:
                    self.log.append("sensor", {"received_at": now_iso(t), "time_source": "server_receive",
                                               "valid": False, "reasons": ["bad_packet"]})
                continue
            sample = parse_state(text, t, mono)
            with self._state_lock:
                self._latest_state = sample
                self._state_times.append(mono)
                self.state_count += 1
                if not sample.valid:
                    self.state_bad_count += 1
            if self.log:
                self.log.append("sensor", sample.to_dict())

    def latest_state(self) -> dict | None:
        with self._state_lock:
            sample = self._latest_state
        return None if sample is None else sample.to_dict(time.monotonic())

    def state_rate_hz(self) -> float | None:
        with self._state_lock:
            times = list(self._state_times)
        if len(times) < 2:
            return None
        if time.monotonic() - times[-1] > 2.0:
            return 0.0
        return round((len(times) - 1) / (times[-1] - times[0]), 1) if times[-1] > times[0] else None

    def metrics(self) -> dict:
        rtts = self._rtts
        return {
            "run_id": self.run_id,
            "closed_at": now_iso(),
            "commands_sent": self._counts["sent"],
            "commands_blocked": self._counts["blocked"],
            "commands_timeout": self._counts["timeout"],
            "rtt_ms_median": sorted(rtts)[len(rtts) // 2] if rtts else None,
            "rtt_ms_max": max(rtts) if rtts else None,
            "state_packets": self.state_count,
            "state_bad_packets": self.state_bad_count,
        }
