"""수동 조작용 비행 제어 (표준 라이브러리만 사용).

TelloLink 위에서 이륙 전 점검, 손 떼면 제자리(데드맨), 속도·높이 상한, 이상 시 자동 착륙을 맡는다.
키보드든 스크립트든 `set_input()`으로 조종 입력을 넣고, rc는 이 클래스가 20Hz로 보낸다.
rc 값은 조종 입력 비율이지 속도가 아니다. 실제 속도는 상태값·실측으로 따로 구한다.
"""

from __future__ import annotations

import math
import threading
import time
from collections import deque
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
# 배터리·온도 기준은 FlightController를 만드는 쪽이 넘긴다. 아래는 넘기지 않았을 때의 기본값이다(수동 조종 게이트웨이·점검 서버).
# 자율비행 드라이버(tello_driver)는 자기 값을 넘긴다: 배터리는 이륙 기준 없이 비행 중 10% 이하일 때만 그 자리에 착륙
# (2026-11-02 사용자 결정), 온도는 이륙 80°C 미만·비행 중 85°C 이상이면 착륙
BATTERY_MIN_TAKEOFF = 20   # 이륙 전 최소 배터리(%). Tello 앱에는 없는 기준이라 실내 짧은 비행 여유만 둔다. 0이면 기준 없음
BATTERY_LAND = 15          # 비행 중 이 값 이하면 자동 착륙. 기체도 약 10%에서 스스로 착륙한다(사용자 보고)
# 온도는 기본으로 막지 않고 알리기만 한다. Tello는 팬이 없어 지상에서 뜨거워지고 날면 식으므로, 뜨거울 때 이륙을 막으면 역효과다.
# 위험한 온도에서는 기체가 스스로 꺼진다(임계값 비공개). Tello 앱도 온도로 이륙을 막지 않는다.
TEMP_WARN_C = 75           # 이 이상이면 이륙 전 점검에 경고(이륙은 허용)
TEMP_MAX_TAKEOFF_C = None  # 이륙 전 최고 온도. None이면 막지 않고 TEMP_WARN_C로 알리기만 한다
TEMP_LAND_C = None         # 비행 중 고온 자동 착륙. None이면 끔(기체 자체 보호에 맡긴다)
MAPPING_TEMP_LAND_C = 85  # 통합 조종 모드의 보수적 착륙 기준
TAKEOFF_TIMEOUT_S = 20
LAND_TIMEOUT_S = 15
# 착륙 전 스틱 0: Tello는 마지막에 받은 rc를 내려오는 동안에도 그대로 따른다. 2026-11-12 13:58 실기체는 뒤쪽 제동(rc 0 -36 0 0)
# 중에 착륙 요청을 받았는데, land()가 입력을 0으로 정해 두기만 하고 rc 루프를 세워 0이 나가지 않았다 → 내려오는 3.5초 동안
# 0.28 m/s로 0.83 m 뒤로 갔다. land를 보내기 전에 rc 0 0 0 0을 직접 보낸다 (rc는 응답 없는 UDP라 한 번은 잃을 수 있다)
LAND_ZERO_RC = 3           # 이만큼
LAND_ZERO_GAP_S = 0.02     #   이 간격으로
# 충돌 감지: 2026-10-29 모니터에 부딪힌 기체는 pitch 40°로 기울며 떨어져 하방 tof가 바닥(10 cm)이 됐는데, 떠 있다고 믿고
# 2초 넘게 회전 명령을 보냈다. 정상 비행 기울기는 rc 80에서도 3° 안팎이었다
CRASH_TILT_DEG = 30        # 이보다 기울면 부딪혔거나 뒤집히는 중
CRASH_TILT_WINDOW_S = 3.0  # 기운 뒤 이 시간 안에
CRASH_GROUND_S = 0.5       #   하방 tof가 이만큼 바닥이면 떨어진 것
FELL_GROUND_S = 1.5        # 기운 것을 못 봤어도 이만큼 바닥이면 떨어진 것 (상태값 10 Hz라 기운 순간을 놓칠 수 있다)
# 뒤집힘(flipped): 2026-11-03 14:32 선반에 닿아 roll −173°로 뒤집혔는데, 하방 tof가 위를 봐 0.69 m로 읽혀 바닥 판정이 안 났고
# land는 error로 끝나 운용자가 Esc를 누를 때까지 6초 걸렸다. 이만큼 기운 채 이어지면 바닥을 기다리지 않고 모터를 끈다
FLIP_TILT_DEG = 90
FLIP_S = 0.5
# 내려앉음(grounded): 기울지 않았는데 바닥. 2026-10-30 복도에서 상승 입력 rc 30을 보내는데도 0.5 m/s로 내려가 바닥에 닿았다(충돌 아님).
# 그때 1.5 s 동안 전진 입력을 그대로 보내 카펫 위에서 밀었다
GROUNDED_AGREE_S = 0.8     # 하방 tof와 h가 둘 다 바닥이면 이만큼만 기다린다 (tof만이면 FELL_GROUND_S: 낮은 가구 위를 스칠 수 있다)
GROUND_HOLD_S = 0.3        # 떠 있다고 믿는데 바닥이 이만큼 이어지면 판정 전이라도 입력을 0으로 (밀거나 돌리지 않는다)
SINK_WINDOW_S = 0.6        # 하강 감시: 이 시간 동안 하방 tof가 SINK_DROP_M 넘게 떨어지는데
SINK_DROP_M = 0.2          #   상승 입력(rc SINK_UD_MIN 이상)을 보내고 있으면 경고를 남긴다
SINK_H_AGREE = 0.5         #   단 Tello h도 그 ToF 감소의 이 비율 넘게 같이 내려갔을 때만 (아니면 밑에 가구·바닥 턱이 들어온 것)
SINK_UD_MIN = 15
SINK_WARN_GAP_S = 2.0      # 같은 경고는 이 간격으로만
# 천장 눌림(climb_stalled): 2026-11-09 17:07 실기체가 상승 입력 rc 37을 2.7초 보내는 동안 Tello h가 1.9 m에서 오르지 않았다
# (높이 상한 2.0 m 아래에서 천장·매달린 것에 눌림). 상한을 실제 천장에 맞추지 않았으면 이렇게 눌린 것을 보고
# 이번 비행의 상한을 그 아래로 낮추고 잠깐 내려온다. rc 37에서 h는 약 0.2 m/s로 올랐다
CLIMB_STALL_UD = 25        # 이 이상 상승 입력을
CLIMB_STALL_S = 2.0        #   이만큼 보내는 동안
CLIMB_STALL_RISE_M = 0.2   #   Tello h가 이만큼도 오르지 않으면 (h는 0.1 m 단위라 두 칸)
CLIMB_STALL_MIN_H = 1.2    #   h가 이 이상일 때만 본다 (이륙 직후·낮은 곳은 빼고)
CEILING_MARGIN_M = 0.3     # 그때 h에서 이만큼 아래를 이번 비행의 높이 상한으로
CEILING_DROP_S = 1.0       # 그리고 이 시간 동안
CEILING_DROP_UD = -20      #   이 입력으로 내려온다


@dataclass
class Check:
    key: str
    title: str
    ok: bool
    detail: str
    blocking: bool = True  # False면 알리기만 하고 이륙은 막지 않는다


class FlightController:
    def __init__(self, link: TelloLink, rc_limit: int = RC_LIMIT, height_limit_m: float = HEIGHT_LIMIT_M,
                 operator_controlled: bool = True, temp_max_takeoff_c: float | None = TEMP_MAX_TAKEOFF_C,
                 temp_land_c: float | None = TEMP_LAND_C, battery_min_takeoff_pct: float = BATTERY_MIN_TAKEOFF,
                 battery_land_pct: float = BATTERY_LAND, *, block_on_heat=False):
        if not link.allow_flight:
            raise ValueError("FlightController는 allow_flight=True인 TelloLink가 필요하다")
        self.link = link
        self.rc_limit = rc_limit
        self.height_limit_m = height_limit_m
        self.temp_max_takeoff_c = temp_max_takeoff_c   # 이륙 전 최고 온도. None이면 막지 않고 알리기만(block_on_heat면 TEMP_WARN_C로 막음)
        self.temp_land_c = temp_land_c                 # 비행 중 이 이상이면 자동 착륙. None이면 끔
        self.battery_min_takeoff_pct = battery_min_takeoff_pct   # 이륙 전 최소 배터리(%), 0이면 기준 없음
        self.battery_land_pct = battery_land_pct                 # 비행 중 이 값 이하면 자동 착륙
        # 이륙하면 manifest의 operator_controlled를 이 값으로 남긴다. 자율비행 드라이버(tello_driver)는 False로 만든다
        self.operator_controlled = operator_controlled
        self.block_on_heat = block_on_heat             # 통합 조종 모드: 뜨거우면 이륙을 막고 MAPPING_TEMP_LAND_C에서 착륙
        self.flying = False
        self.busy: str | None = None          # takeoff / land 진행 중
        self.operator_confirmed = False       # 주변 비움·가드·안전 담당 확인
        self.last_auto_land: dict | None = None
        self.last_rc: tuple[int, int, int, int] = (0, 0, 0, 0)
        self._input = (0, 0, 0, 0)
        self._input_source = "none"
        self._input_mono = 0.0
        self._emergency_epoch = 0
        self._tilt_mono: float | None = None     # 마지막으로 크게 기운 때
        self._flip_mono: float | None = None     # 뒤집힌 채(FLIP_TILT_DEG 넘게) 이어지기 시작한 때
        self._ground_mono: float | None = None   # 떠 있는 동안 하방 tof가 바닥이 된 때
        self._tof_hist: deque[tuple[float, float, float | None]] = deque()   # (monotonic, 하방 tof m, h m) 최근 SINK_WINDOW_S
        self.last_warning: dict | None = None    # 착륙시키지는 않는 이상 신호 (sink_against_climb, climb_stalled)
        self._warn_mono = 0.0
        self.ceiling_cap_m: float | None = None  # 천장에 눌린 것을 보고 낮춘 이번 비행의 높이 상한 (Tello h 기준)
        self._climb_mono: float | None = None    # 강한 상승 입력이 이어지기 시작한 때
        self._climb_h0 = 0.0                     #   그때의 h
        self._drop_until = 0.0                   # 천장에서 내려오는 중이면 그 끝 (monotonic)
        self._lock = threading.Lock()
        self._rc_lock = threading.Lock()         # rc 루프의 '입력 읽고 보내기'와 land()의 '루프 세우기'가 섞이지 않게
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
            Check("battery", self._battery_title(),
                  isinstance(bat, (int, float)) and math.isfinite(bat)
                  and bat >= self.battery_min_takeoff_pct and bat > self.battery_land_pct,
                  "없음" if bat is None else f"{bat}%"),
            self._temp_check(temp),
            Check("operator", "주변 비움·가드 장착·안전 담당 확인", self.operator_confirmed,
                  "확인함" if self.operator_confirmed else "체크 필요"),
        ]
        if self.block_on_heat:
            height = si.get("height_m")
            on_ground = (si.get("tof_reason") == "below_min_or_on_ground" and
                         isinstance(height, (int, float)) and 0 <= height <= AIRBORNE_M)
            checks.append(Check("ground", "기체 지상 확인", on_ground, "지상" if on_ground else "높이 확인 필요"))
        return checks

    def _temp_check(self, temp) -> Check:
        """이륙 전 온도 점검. 이륙 최고 온도를 받았으면(자율비행 드라이버) 그 미만이어야 이륙하고,
        아니면 TEMP_WARN_C로 알리기만 한다(통합 조종 모드 block_on_heat면 막는다)."""
        known = isinstance(temp, (int, float)) and math.isfinite(temp)
        detail = "없음" if temp is None else f"{temp}°C"
        if self.temp_max_takeoff_c is not None:
            return Check("temp", f"온도 {self.temp_max_takeoff_c:g}°C 미만", known and temp < self.temp_max_takeoff_c, detail)
        return Check("temp", f"온도 {TEMP_WARN_C}°C 미만" + ("" if self.block_on_heat else "(권장)"),
                     known and temp < TEMP_WARN_C, detail, blocking=self.block_on_heat)

    def _battery_title(self) -> str:
        """이륙 점검 문구. 이륙 기준이 착륙 기준보다 높을 때만 그 기준을 말한다 (자율비행 드라이버는 기준 없음)."""
        if self.battery_min_takeoff_pct > self.battery_land_pct:
            return f"배터리 {self.battery_min_takeoff_pct:g}% 이상"
        return f"배터리 {self.battery_land_pct:g}% 초과 (이하면 바로 착륙)"

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
            self.link.update_manifest(operator_controlled=self.operator_controlled)
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
                # 응답이 없어도 실제로는 떠 있을 수 있다. h는 떠 있어도 0~10cm로 나온 적이 있어(허병우 10/27)
                # 하방 tof와 둘 중 하나라도 높으면 떠 있다고 본다.
                self.flying = r.ok or uncertain or any(isinstance(v, (int, float)) and v > AIRBORNE_M for v in heights)
                self._input, self._input_source, self._input_mono = (0, 0, 0, 0), "takeoff", time.monotonic()
                if self.flying:
                    self._reset_ground()   # 지난 비행의 바닥 판정·낮춘 높이 상한이 이번 이륙에 남지 않게
                return {"ok": r.ok, "flying": self.flying, "result": r.to_dict()}
        finally:
            self.busy = None

    def land(self, source: str = "operator") -> dict:
        """스틱을 0으로 되돌린 뒤 land를 보낸다 (LAND_ZERO_* 설명: 마지막 rc가 남으면 그 방향으로 가며 내려온다)."""
        with self._rc_lock:    # 이 뒤로 rc 루프는 쉰다. 루프가 읽어 둔 옛 입력이 아래 0보다 늦게 나가지 않는다
            self.set_input(0, 0, 0, 0, source=source)
            self.busy = "land"
        try:
            for i in range(LAND_ZERO_RC):
                if i:
                    time.sleep(LAND_ZERO_GAP_S)
                self.link.rc(0, 0, 0, 0, source=source)
            self.last_rc = (0, 0, 0, 0)
            r = self.link.send("land", timeout=LAND_TIMEOUT_S, source=source)
        finally:
            self.busy = None
        landed = r.ok
        if not r.ok:
            # 부딪혀 떨어진 뒤에는 Tello가 land에 error로 답하고, 떠 있다고 믿은 채 land를 끝없이 다시 보냈다
            # (2026-10-29 모니터 충돌). 높이(하방 tof·h)가 둘 다 바닥이면 이미 내려앉은 것으로 본다
            si = (self.link.latest_state() or {}).get("si") or {}
            tof = si.get("tof_m") if si.get("tof_valid") else 0.0 if si.get("tof_reason") == "below_min_or_on_ground" else None
            heights = [si.get("height_m"), tof]
            if all(isinstance(v, (int, float)) and v <= AIRBORNE_M for v in heights):
                landed = True
                self.link.event("land_assumed_on_ground", {"response": r.response, "heights": heights})
        if landed:
            self.flying = False
            self.last_rc = (0, 0, 0, 0)
            self._reset_ground()
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
        self._reset_ground()
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
        if self._ground_mono is not None and time.monotonic() - self._ground_mono >= GROUND_HOLD_S:
            # 떠 있다고 믿는데 바닥이 이어진다: crash/grounded 판정 전이라도 밀거나 돌리지 않는다
            return (0, 0, 0, 0), source + "+ground_hold"
        if ud > 0:
            blocked = self._height_check()
            if blocked:
                ud = 0
                source = source + "+" + blocked
        ud, source = self._climb_watch(ud, source)
        return (lr, fb, ud, yaw), source

    def _climb_watch(self, ud: int, source: str) -> tuple[int, str]:
        """강한 상승 입력에도 h가 오르지 않으면 천장에 눌린 것으로 보고 상한을 낮추고 잠깐 내려온다 (CLIMB_STALL_*)."""
        now = time.monotonic()
        if now < self._drop_until:
            return min(ud, CEILING_DROP_UD), source + "+ceiling_drop"
        h = ((self.link.latest_state() or {}).get("si") or {}).get("height_m")
        if ud < CLIMB_STALL_UD or not isinstance(h, (int, float)) or h < CLIMB_STALL_MIN_H:
            self._climb_mono = None
            return ud, source
        if self._climb_mono is None or h - self._climb_h0 >= CLIMB_STALL_RISE_M:
            self._climb_mono, self._climb_h0 = now, float(h)   # 오르고 있다: 지금부터 다시 잰다
            return ud, source
        if now - self._climb_mono < CLIMB_STALL_S:
            return ud, source
        cap = round(float(h) - CEILING_MARGIN_M, 2)
        self.ceiling_cap_m = cap if self.ceiling_cap_m is None else min(self.ceiling_cap_m, cap)
        self._climb_mono = None
        self._drop_until = now + CEILING_DROP_S
        detail = {"h_m": round(float(h), 2), "ud": ud, "stall_s": CLIMB_STALL_S, "cap_m": self.ceiling_cap_m}
        self.last_warning = {"at": now_iso(), "type": "climb_stalled", "detail": detail}
        self.link.event("climb_stalled", detail)
        return CEILING_DROP_UD, source + "+ceiling_drop"

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
        if self.ceiling_cap_m is not None and isinstance(h, (int, float)) and h >= self.ceiling_cap_m:
            return "ceiling_cap"   # 이번 비행에서 천장에 눌린 높이 아래로 낮춘 상한 (_climb_watch)
        return None

    def _safety_reason(self) -> str | None:
        state = self.link.latest_state()
        if state is None or state.get("valid") is not True or state.get("age_s", 99) > STATE_STALE_S:
            return "state_lost"
        si = state.get("si") or {}
        bat, temp = si.get("battery_pct"), si.get("temp_high_c")
        if isinstance(bat, (int, float)) and self.battery_land_pct > 0 and bat <= self.battery_land_pct:
            return "battery_low"
        temp_limit = MAPPING_TEMP_LAND_C if self.block_on_heat else self.temp_land_c
        if temp_limit is not None and isinstance(temp, (int, float)) and temp >= temp_limit:
            return "overheat"
        self._sink_check(si)
        return self._crash_check(si)

    def _reset_ground(self) -> None:
        """이륙·착륙 때 바닥·기울기·하강 기록을 비운다 (지난 비행의 바닥 판정이 다음 이륙에 남지 않게)."""
        self._tilt_mono = self._ground_mono = self._flip_mono = None
        self._tof_hist.clear()
        self.ceiling_cap_m, self._climb_mono, self._drop_until = None, None, 0.0

    def _sink_check(self, si: dict) -> None:
        """상승 입력을 보내는데 하방 tof가 빠르게 떨어지면 경고를 남긴다 (착륙시키지는 않는다: 바닥에 닿으면 grounded가 잡는다).

        2026-10-30 복도: rc ud 30(0.3 m/s 상승)을 보내는 동안 0.5 m/s로 내려가 바닥에 닿았다.
        하방 tof만 보면 가구 위로 넘으려 오르는 동안(상승 입력) 밑에 가구·바닥 턱이 들어온 것도 '하강'이 된다: 11/05–11/09 실기체
        경고 10번 모두 tof는 0.23–0.64 m 줄었는데 Tello h는 그대로이거나 0.1–0.2 m만 움직였다. 그래서 h도 같이 내려갔을 때만 남긴다
        (h는 바닥 기준은 아니지만 변화는 따라간다). h가 없으면 tof만 본다.
        """
        now = time.monotonic()
        tof = si.get("tof_m") if si.get("tof_valid") else None
        if not isinstance(tof, (int, float)):
            return
        h = si.get("height_m")
        self._tof_hist.append((now, float(tof), float(h) if isinstance(h, (int, float)) else None))
        while self._tof_hist and now - self._tof_hist[0][0] > SINK_WINDOW_S:
            self._tof_hist.popleft()
        if self.last_rc[2] < SINK_UD_MIN or now - self._warn_mono < SINK_WARN_GAP_S:
            return
        drop = self._tof_hist[0][1] - float(tof)
        h0 = self._tof_hist[0][2]
        h_drop = None if h0 is None or not isinstance(h, (int, float)) else h0 - float(h)
        if drop >= SINK_DROP_M and (h_drop is None or h_drop >= SINK_H_AGREE * drop):
            self._warn_mono = now
            detail = {"drop_m": round(drop, 2), "window_s": SINK_WINDOW_S, "ud": self.last_rc[2],
                      "h_drop_m": None if h_drop is None else round(h_drop, 2)}
            self.last_warning = {"at": now_iso(), "type": "sink_against_climb", "detail": detail}
            self.link.event("sink_against_climb", detail)

    def _crash_check(self, si: dict) -> str | None:
        """떠 있다고 믿는 동안 바닥에 닿아 있으면 착륙 처리로 명령을 끊는다. 이유는 둘로 나눈다.

        - crash: 크게 기운(CRASH_TILT_DEG) 뒤 바닥 — 부딪혀 떨어진 것 (2026-10-29 모니터)
        - grounded: 기울지 않고 바닥 — 스스로 내려앉은 것 (2026-10-30 복도). 충돌이 아니므로 그렇게 부르지 않는다.
        - flipped: FLIP_TILT_DEG 넘게 기운 채 FLIP_S — 뒤집힌 것. 바닥을 기다리지 않는다(뒤집히면 하방 tof가 위를 본다)
          하방 tof와 h가 둘 다 바닥이면 GROUNDED_AGREE_S, tof만이면 FELL_GROUND_S 뒤에 판정한다
        land에 Tello가 error로 답해도 높이가 바닥이면 land()가 내려앉은 것으로 본다.
        """
        now = time.monotonic()
        tilts = [abs(math.degrees(v)) for v in (si.get("pitch_rad"), si.get("roll_rad")) if isinstance(v, (int, float))]
        if tilts and max(tilts) >= CRASH_TILT_DEG:
            self._tilt_mono = now
        if tilts and max(tilts) >= FLIP_TILT_DEG:
            if self._flip_mono is None:
                self._flip_mono = now
            if now - self._flip_mono >= FLIP_S:
                self._reset_ground()
                return "flipped"
        else:
            self._flip_mono = None
        on_ground = si.get("tof_reason") == "below_min_or_on_ground" or (
            si.get("tof_valid") and isinstance(si.get("tof_m"), (int, float)) and si["tof_m"] <= AIRBORNE_M)
        if not on_ground:
            self._ground_mono = None
            return None
        if self._ground_mono is None:
            self._ground_mono = now
        grounded = now - self._ground_mono
        tilted = self._tilt_mono is not None and now - self._tilt_mono <= CRASH_TILT_WINDOW_S
        if tilted and grounded >= CRASH_GROUND_S:
            self._reset_ground()
            return "crash"
        h = si.get("height_m")
        agree = isinstance(h, (int, float)) and h <= AIRBORNE_M   # h도 바닥이라고 한다
        if grounded >= (GROUNDED_AGREE_S if agree else FELL_GROUND_S):
            self._reset_ground()
            return "grounded"
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
                        if reason == "flipped":   # 뒤집힌 채 land는 error로 끝난다: 모터를 바로 끈다
                            self.emergency(source="safety:" + reason)
                        else:
                            self.land(source="safety:" + reason)
                    else:
                        with self._rc_lock:
                            if self.busy is None:   # 그 사이 land()가 시작했으면 보내지 않는다 (land가 0을 직접 보낸다)
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
            "ceiling_cap_m": self.ceiling_cap_m,
            "height_source": "down_tof",
            "last_rc": list(self.last_rc),
            "input_source": self._input_source,
            "last_auto_land": self.last_auto_land,
            "last_warning": self.last_warning,
        }
