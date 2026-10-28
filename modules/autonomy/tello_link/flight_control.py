"""수동 조작용 비행 제어 (표준 라이브러리만 사용).

TelloLink 위에서 이륙 전 점검, 손 떼면 제자리(데드맨), 속도·높이 상한, 이상 시 자동 착륙을 맡는다.
키보드든 스크립트든 `set_input()`으로 조종 입력을 넣고, rc는 이 클래스가 20Hz로 보낸다.
rc 값은 조종 입력 비율이지 속도가 아니다. 실제 속도는 상태값·실측으로 따로 구한다.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass

try:
    from .tello_link import TelloLink, now_iso
except ImportError:  # 직접 실행하는 check_server.py와의 호환
    from tello_link import TelloLink, now_iso

RC_LIMIT = 30              # 처음 비행 상한 (±100 중 30)
HEIGHT_LIMIT_M = 1.5       # 이 높이 이상이면 상승 입력을 무시한다(하방 tof 기준, 아래 _height_check)
AIRBORNE_M = 0.2           # takeoff 응답이 없을 때 이보다 높으면 떠 있다고 본다
RC_RATE_HZ = 20
DEADMAN_S = 0.3            # 입력이 이 시간 동안 없으면 제자리
STATE_STALE_S = 1.0        # 비행 중 상태가 이보다 오래 끊기면 자동 착륙
BATTERY_MIN_TAKEOFF = 30   # 이륙 전 최소 배터리(%)
BATTERY_LAND = 15          # 비행 중 이 아래면 자동 착륙
TEMP_MAX_TAKEOFF_C = 80    # 이륙 전 최고 온도
TEMP_LAND_C = 85           # 비행 중 이 이상이면 자동 착륙
TAKEOFF_TIMEOUT_S = 20
LAND_TIMEOUT_S = 15


@dataclass
class Check:
    key: str
    title: str
    ok: bool
    detail: str


class FlightController:
    def __init__(self, link: TelloLink, rc_limit: int = RC_LIMIT, height_limit_m: float = HEIGHT_LIMIT_M):
        if not link.allow_flight:
            raise ValueError("FlightController는 allow_flight=True인 TelloLink가 필요하다")
        self.link = link
        self.rc_limit = rc_limit
        self.height_limit_m = height_limit_m
        self.flying = False
        self.busy: str | None = None          # takeoff / land 진행 중
        self.operator_confirmed = False       # 주변 비움·가드·안전 담당 확인
        self.last_auto_land: dict | None = None
        self.last_rc: tuple[int, int, int, int] = (0, 0, 0, 0)
        self._input = (0, 0, 0, 0)
        self._input_source = "none"
        self._input_mono = 0.0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    # ---------- 수명 ----------
    def start(self) -> "FlightController":
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, name="tello-flight", daemon=True)
        self._thread.start()
        return self

    def close(self) -> None:
        if self.flying:
            self.land(source="shutdown")
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2)

    # ---------- 이륙 전 점검 ----------
    def preflight(self) -> list[Check]:
        state = self.link.latest_state()
        si = (state or {}).get("si") or {}
        age = (state or {}).get("age_s")
        bat = si.get("battery_pct")
        temp = si.get("temp_high_c")
        return [
            Check("sdk", "SDK 모드", self.link.sdk_mode, "command → ok" if self.link.sdk_mode else "[점검 시작]을 먼저 누르세요"),
            Check("state", "상태 수신", state is not None and age is not None and age < STATE_STALE_S,
                  "없음" if age is None else f"{age:.1f}초 전"),
            Check("battery", f"배터리 {BATTERY_MIN_TAKEOFF}% 이상", isinstance(bat, (int, float)) and bat >= BATTERY_MIN_TAKEOFF,
                  "없음" if bat is None else f"{bat}%"),
            Check("temp", f"온도 {TEMP_MAX_TAKEOFF_C}°C 미만", isinstance(temp, (int, float)) and temp < TEMP_MAX_TAKEOFF_C,
                  "없음" if temp is None else f"{temp}°C"),
            Check("operator", "주변 비움·가드 장착·안전 담당 확인", self.operator_confirmed,
                  "확인함" if self.operator_confirmed else "체크 필요"),
        ]

    def can_takeoff(self) -> tuple[bool, list[Check]]:
        checks = self.preflight()
        return all(c.ok for c in checks) and not self.flying and self.busy is None, checks

    def confirm_operator(self, confirmed: bool) -> None:
        self.operator_confirmed = confirmed
        self.link.event("operator_confirm", {"confirmed": confirmed})

    # ---------- 명령 ----------
    def takeoff(self, source: str = "operator") -> dict:
        ok, checks = self.can_takeoff()
        if not ok:
            failed = [c.key for c in checks if not c.ok]
            self.link.event("takeoff_refused", {"failed": failed, "flying": self.flying, "busy": self.busy})
            return {"ok": False, "reason": "preflight_failed", "failed": failed}
        self.link.update_manifest(operator_controlled=True)
        self.busy = "takeoff"
        try:
            r = self.link.send("takeoff", timeout=TAKEOFF_TIMEOUT_S, source=source)
        finally:
            self.busy = None
        if r.ok:
            self.flying = True
            self.set_input(0, 0, 0, 0, source="takeoff")
        else:
            # 응답이 없어도 실제로는 떠 있을 수 있다. 높이로 한 번 더 판단한다.
            # h는 떠 있어도 0~10cm로 나온 적이 있어(허병우 10/27) 하방 tof와 둘 중 하나라도 높으면 떠 있다고 본다.
            si = (self.link.latest_state() or {}).get("si") or {}
            heights = [si.get("height_m"), si.get("tof_m") if si.get("tof_valid") else None]
            if any(isinstance(v, (int, float)) and v > AIRBORNE_M for v in heights):
                self.flying = True
        return {"ok": r.ok, "flying": self.flying, "result": r.to_dict()}

    def land(self, source: str = "operator") -> dict:
        self.set_input(0, 0, 0, 0, source=source)
        self.busy = "land"
        try:
            r = self.link.send("land", timeout=LAND_TIMEOUT_S, source=source)
        finally:
            self.busy = None
        if r.ok:
            self.flying = False
            self.last_rc = (0, 0, 0, 0)
            self.operator_confirmed = False  # 다음 이륙 전에 주변을 다시 확인하게 한다
        return {"ok": r.ok, "flying": self.flying, "result": r.to_dict()}

    def emergency(self, source: str = "operator") -> dict:
        r = self.link.emergency_now(source=source)
        self.flying = False
        self.last_rc = (0, 0, 0, 0)
        self.operator_confirmed = False  # 다음 이륙 전에 주변을 다시 확인하게 한다
        self.set_input(0, 0, 0, 0, source=source)
        return {"ok": True, "result": r.to_dict()}

    def set_input(self, lr: float, fb: float, ud: float, yaw: float, source: str = "keyboard") -> None:
        """조종 입력을 -1~1로 넣는다(좌우·앞뒤·상하·회전). 1은 rc 100이고, rc_limit로 잘린다."""
        def scale(v):
            v = max(-1.0, min(1.0, float(v))) * 100
            return int(max(-self.rc_limit, min(self.rc_limit, round(v))))
        with self._lock:
            self._input = tuple(scale(v) for v in (lr, fb, ud, yaw))
            self._input_source = source
            self._input_mono = time.monotonic()

    # ---------- 루프 ----------
    def _current_rc(self) -> tuple[tuple[int, int, int, int], str]:
        with self._lock:
            vals, source, stamp = self._input, self._input_source, self._input_mono
        if time.monotonic() - stamp > DEADMAN_S:
            return (0, 0, 0, 0), "deadman"
        lr, fb, ud, yaw = vals
        if ud > 0:
            blocked = self._height_check()
            if blocked:
                ud = 0
                source = source + "+" + blocked
        return (lr, fb, ud, yaw), source

    def _height_check(self) -> str | None:
        """상승을 막아야 하면 이유를 돌려준다.

        기준은 하방 tof(바닥까지 거리)다. `h`는 실기체에서 0.78m에 떠 있는데 0~10cm로 나온 적이 있어
        혼자 믿지 않는다. 하방 tof가 무효면 높이를 모르는 것으로 보고 상승을 막는다(하강은 허용).
        `h`가 상한 이상이어도 막는다. 둘 중 높은 쪽을 따르는 보수적 판단이다.
        """
        si = (self.link.latest_state() or {}).get("si") or {}
        if not si.get("tof_valid") or not isinstance(si.get("tof_m"), (int, float)):
            return "height_unknown"
        h = si.get("height_m")
        if si["tof_m"] >= self.height_limit_m or (isinstance(h, (int, float)) and h >= self.height_limit_m):
            return "height_limit"
        return None

    def _safety_reason(self) -> str | None:
        state = self.link.latest_state()
        if state is None or state.get("age_s", 99) > STATE_STALE_S:
            return "state_lost"
        si = state.get("si") or {}
        bat, temp = si.get("battery_pct"), si.get("temp_high_c")
        if isinstance(bat, (int, float)) and bat < BATTERY_LAND:
            return "battery_low"
        if isinstance(temp, (int, float)) and temp >= TEMP_LAND_C:
            return "overheat"
        return None

    def _loop(self) -> None:
        period = 1.0 / RC_RATE_HZ
        while not self._stop.is_set():
            t0 = time.monotonic()
            try:
                if self.flying and self.busy is None:
                    reason = self._safety_reason()
                    if reason:
                        self.last_auto_land = {"at": now_iso(), "reason": reason}
                        self.link.event("auto_land", {"reason": reason})
                        self.land(source="safety:" + reason)
                    else:
                        rc, source = self._current_rc()
                        self.link.rc(*rc, source=source)
                        self.last_rc = rc
            except OSError as e:  # 종료 중 소켓이 닫힌 경우
                if self._stop.is_set():
                    break
                self.link.event("flight_loop_error", {"error": str(e)})
            self._stop.wait(max(0.0, period - (time.monotonic() - t0)))

    def status(self) -> dict:
        ok, checks = self.can_takeoff()
        return {
            "flying": self.flying,
            "busy": self.busy,
            "can_takeoff": ok,
            "preflight": [c.__dict__ for c in checks],
            "rc_limit": self.rc_limit,
            "height_limit_m": self.height_limit_m,
            "height_source": "down_tof",
            "last_rc": list(self.last_rc),
            "input_source": self._input_source,
            "last_auto_land": self.last_auto_land,
        }
