"""수동 조작용 비행 제어 (표준 라이브러리만 사용).

TelloLink 위에서 이륙 전 점검, 손 떼면 제자리(데드맨), 속도·높이 상한, 이상 시 자동 착륙을 맡는다.
키보드든 스크립트든 `set_input()`으로 조종 입력을 넣고, rc는 이 클래스가 20Hz로 보낸다.
rc 값은 조종 입력 비율이지 속도가 아니다. 실제 속도는 상태값·실측으로 따로 구한다.
"""

from __future__ import annotations

import math
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
BATTERY_MIN_TAKEOFF = 20   # 이륙 전 최소 배터리(%). Tello 앱에는 없는 기준이라 실내 짧은 비행 여유만 둔다
BATTERY_LAND = 15          # 비행 중 이 아래면 자동 착륙. 기체도 약 10%에서 스스로 착륙한다(사용자 보고)
# 온도는 막지 않고 알리기만 한다. Tello는 팬이 없어 지상에서 뜨거워지고 날면 식으므로, 뜨거울 때 이륙을 막으면 역효과다.
# 위험한 온도에서는 기체가 스스로 꺼진다(임계값 비공개). Tello 앱도 온도로 이륙을 막지 않는다.
TEMP_WARN_C = 75           # 이 이상이면 이륙 전 점검에 경고(이륙은 허용)
TEMP_LAND_C = None         # 비행 중 고온 자동 착륙. None이면 끔(기체 자체 보호에 맡긴다)
MAPPING_TEMP_LAND_C = 85  # 통합 조종 모드의 보수적 착륙 기준
TAKEOFF_TIMEOUT_S = 20
LAND_TIMEOUT_S = 15


@dataclass
class Check:
    key: str
    title: str
    ok: bool
    detail: str
    blocking: bool = True  # False면 알리기만 하고 이륙은 막지 않는다


class FlightController:
    def __init__(self, link: TelloLink, rc_limit: int = RC_LIMIT, height_limit_m: float = HEIGHT_LIMIT_M, *,
                 block_on_heat=False):
        if not link.allow_flight:
            raise ValueError("FlightController는 allow_flight=True인 TelloLink가 필요하다")
        self.link = link
        self.rc_limit = rc_limit
        self.height_limit_m = height_limit_m
        self.block_on_heat = block_on_heat
        self.flying = False
        self.busy: str | None = None          # takeoff / land 진행 중
        self.operator_confirmed = False       # 주변 비움·가드·안전 담당 확인
        self.last_auto_land: dict | None = None
        self.last_rc: tuple[int, int, int, int] = (0, 0, 0, 0)
        self._input = (0, 0, 0, 0)
        self._input_source = "none"
        self._input_mono = 0.0
        self._emergency_epoch = 0
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
        self.set_input(0, 0, 0, 0, source="shutdown")
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=LAND_TIMEOUT_S + 2)
        if self.flying:
            self.link.rc(0, 0, 0, 0, source="shutdown")
            self.land(source="shutdown")

    # ---------- 이륙 전 점검 ----------
    def preflight(self) -> list[Check]:
        state = self.link.latest_state()
        si = (state or {}).get("si") or {}
        age = (state or {}).get("age_s")
        bat = si.get("battery_pct")
        temp = si.get("temp_high_c")
        checks = [
            Check("sdk", "SDK 모드", self.link.sdk_mode, "command → ok" if self.link.sdk_mode else "[점검 시작]을 먼저 누르세요"),
            Check("state", "상태 수신", state is not None and state.get("valid") is True and age is not None and age < STATE_STALE_S,
                  "없음" if age is None else f"{age:.1f}초 전"),
            Check("battery", f"배터리 {BATTERY_MIN_TAKEOFF}% 이상",
                  isinstance(bat, (int, float)) and math.isfinite(bat) and bat >= BATTERY_MIN_TAKEOFF,
                  "없음" if bat is None else f"{bat}%"),
            Check("temp", f"온도 {TEMP_WARN_C}°C 미만" + ("" if self.block_on_heat else "(권장)"),
                  isinstance(temp, (int, float)) and math.isfinite(temp) and temp < TEMP_WARN_C,
                  "없음" if temp is None else f"{temp}°C", blocking=self.block_on_heat),
            Check("operator", "주변 비움·가드 장착·안전 담당 확인", self.operator_confirmed,
                  "확인함" if self.operator_confirmed else "체크 필요"),
        ]
        if self.block_on_heat:
            height = si.get("height_m")
            on_ground = (si.get("tof_reason") == "below_min_or_on_ground" and
                         isinstance(height, (int, float)) and 0 <= height <= AIRBORNE_M)
            checks.append(Check("ground", "기체 지상 확인", on_ground, "지상" if on_ground else "높이 확인 필요"))
        return checks

    def can_takeoff(self) -> tuple[bool, list[Check]]:
        checks = self.preflight()
        return all(c.ok for c in checks if c.blocking) and not self.flying and self.busy is None, checks

    def confirm_operator(self, confirmed: bool) -> None:
        with self._lock:
            self.operator_confirmed = confirmed
        self.link.event("operator_confirm", {"confirmed": confirmed})

    # ---------- 명령 ----------
    def takeoff(self, source: str = "operator") -> dict:
        with self._lock:
            ok, checks = self.can_takeoff()
            if not ok:
                failed = [c.key for c in checks if c.blocking and not c.ok]
                self.link.event("takeoff_refused", {"failed": failed, "flying": self.flying, "busy": self.busy})
                return {"ok": False, "reason": "preflight_failed", "failed": failed}
            self.busy = "takeoff"
            emergency_epoch = self._emergency_epoch
        try:
            self.link.update_manifest(operator_controlled=True)
            r = self.link.send("takeoff", timeout=TAKEOFF_TIMEOUT_S, source=source,
                               may_send=lambda: self._emergency_epoch == emergency_epoch
                               and not self._stop.is_set()
                               and all(check.ok for check in self.preflight() if check.blocking))
            state = self.link.latest_state() or {}
            si = state.get("si") or {}
            heights = [si.get("height_m"), si.get("tof_m") if si.get("tof_valid") else None]
            with self._lock:
                if self._emergency_epoch != emergency_epoch:
                    return {"ok": False, "reason": "emergency"}
                uncertain = r.sent_at is not None and r.response is None and (
                    state.get("valid") is not True or state.get("age_s", 99) > STATE_STALE_S)
                self.flying = r.ok or uncertain or any(isinstance(v, (int, float)) and v > AIRBORNE_M for v in heights)
                self._input = (0, 0, 0, 0)
                return {"ok": r.ok, "flying": self.flying, "result": r.to_dict()}
        finally:
            self.busy = None

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
        with self._lock:
            self._emergency_epoch += 1
            self.flying = False
            self.operator_confirmed = False
            self._input = (0, 0, 0, 0)
        r = self.link.emergency_now(source=source)
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
        tof = si.get("tof_m")
        if not si.get("tof_valid") or not isinstance(tof, (int, float)) or not math.isfinite(tof):
            return "height_unknown"
        h = si.get("height_m")
        if tof >= self.height_limit_m or (isinstance(h, (int, float)) and h >= self.height_limit_m):
            return "height_limit"
        return None

    def _safety_reason(self) -> str | None:
        state = self.link.latest_state()
        if state is None or state.get("valid") is not True or state.get("age_s", 99) > STATE_STALE_S:
            return "state_lost"
        si = state.get("si") or {}
        bat, temp = si.get("battery_pct"), si.get("temp_high_c")
        if isinstance(bat, (int, float)) and bat < BATTERY_LAND:
            return "battery_low"
        temp_limit = MAPPING_TEMP_LAND_C if self.block_on_heat else TEMP_LAND_C
        if temp_limit is not None and isinstance(temp, (int, float)) and temp >= temp_limit:
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
