"""Tello 자율비행 드라이버에서 ROS와 무관한 부분 (표준 라이브러리만 사용).

ROS 2 노드(tello_driver_node.py)가 이것을 감싼다. 실기체와 Gazebo 시뮬레이션(simulation/tt-sim의 gz_tello.py,
UDP로 Tello SDK를 흉내 냄)이 같은 코드를 지난다.

  속도 명령 (m/s, rad/s. 기체 기준 x 앞·y 왼쪽·z 위, 반시계 +)
    → rc 입력 비율 (-1~1) → FlightController가 20Hz로 `rc a b c d` 전송 (손 떼면 0.3초 뒤 제자리)
  이륙·착륙·비상 정지 요청 → FlightController. 이륙은 응답까지 약 8.6초 걸려 따로 도는 스레드에서 보낸다
  상태 패킷 → Tello가 보낸 형식의 문자열로 되돌려 위치 추정 노드에 넘긴다
  비행 중 속도 명령이 cmd_timeout 넘게 끊기면(비행 노드가 죽음) 착륙한다
  (front_tof) 떠 있는 동안 확장 키트 전방 ToF를 `EXT tof?`로 물어 새 값만 넘긴다 (m, 범위 밖은 inf)
"""

from __future__ import annotations

import math
import os
import sys
import threading
import time
from dataclasses import dataclass

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tello_link"))

from flight_control import FlightController  # noqa: E402
from tello_link import TelloLink, parse_ext_tof  # noqa: E402
from tof_fresh import FreshFilter  # noqa: E402

CONNECT_RETRY_S = 2.0
CMD_TIMEOUT_S = 2.0
# 전방 ToF 값은 약 0.2초마다 바뀐다(docs/drone/TELLO_EDU_IO.md 10.3). 그 두 배로 물어 새 값을 늦게 잡지 않는다
FRONT_TOF_PERIOD_S = 0.1
# 실측 왕복 최대 507 ms. 늦게 온 응답이 다음 명령(land 등)의 응답으로 읽히지 않게 넉넉히 기다린다
FRONT_TOF_TIMEOUT_S = 0.5
# 땅에서 명령이 없으면 약 85–100초 뒤 상태 패킷이 끊겼다(2026-10-27 확장 키트 장착 기체, Wi-Fi는 그대로).
# 떠 있지 않을 때 이 간격으로 `command`를 다시 보내 SDK 모드를 유지한다(끊겼으면 다시 들어간다)
KEEPALIVE_S = 5.0
AUTONOMY_TEMP_LIMITS = (80.0, 85.0)      # 자율비행 온도 기준 (이륙 미만 °C, 비행 중 이 이상이면 착륙 °C)
AUTONOMY_BATTERY_LIMITS = (0.0, 10.0)    # 자율비행 배터리 기준 (이륙 최소 %: 0이면 없음, 비행 중 이 값 이하면 착륙 %)


@dataclass
class RcScale:
    """rc 100(조종 입력 최대)일 때의 속도. 모두 가정값이라 실기체 시험으로 바꾼다.

    v_full: 2026-10-23 첫 수동 비행에서 rc ±30 동안 vgx가 -2~3(dm/s로 보면 0.2~0.3 m/s)이었다. 줄자 측정 전.
    """
    v_full: float = 1.0                      # 수평 (m/s)
    vz_full: float = 1.0                     # 상하 (m/s)
    yaw_full: float = math.radians(100.0)    # 회전 (rad/s)
    # 수평 rc 무시 구간 (rc 100 = 1). 실기체 Tello는 작은 rc에 덜 움직인다: 2026-10-29 rc 20에 약 0.12 m/s,
    # rc 40–42에 0.3–0.4 m/s. 둘을 이으면 rc ≈ 9.5에서 0, 기울기 0.0114 m/s/rc → dead 0.095, v_full 1.03.
    # 선형(dead 0, v_full 0.6)으로 보내면 speed 0.25를 rc 42로 보내 실제 0.4 m/s로 날아 모니터에 닿았다
    v_dead: float = 0.0
    # 상하 rc 무시 구간. 실기체 Tello는 rc 상하 3–7을 무시해(2026-10-27) 비행 높이 1.0 m에 8 cm 모자란 0.92 m에서 멈췄다
    # (비행 노드는 0.8 × 0.08 = 0.064 m/s = rc 6을 냈다, 11/02 비행 하방 ToF 중앙 0.92 m). 0이면 보정 안 함
    vz_dead: float = 0.0


VZ_HOLD = 0.03   # 상하 명령이 이보다 작으면(높이 차 약 4 cm 안) 0으로 둔다: 무시 구간을 건너뛰어 보내면 그 근처에서 오르내린다


RC_SLEW_PER_S = 1.5   # rc 크기를 키울 때 초당 최대 변화 (rc 100 = 1): 0 → rc 45까지 0.3 s. 줄이기·반대로 밀기는 바로


def slew(prev: float, target: float, step: float) -> float:
    """rc 한 축을 prev에서 target으로: 크기를 키우는 쪽만 step만큼씩 간다. 줄이거나(정지·제동) 부호가 바뀌면 바로.

    비행 노드는 0 → rc 27, 회전 0 → rc 80처럼 한 번에 바꿔 기체가 덜컥거렸다(10/30 실기체). 멈추는 쪽은 늦추지 않는다.
    """
    if abs(target) <= abs(prev) or prev * target < 0:
        return target
    return prev + max(-step, min(step, target - prev))


def to_input(scale: RcScale, rc_limit: int, vx: float, vy: float, vz: float, wz: float):
    """속도 명령 → FlightController.set_input 값 (lr, fb, ud, yaw; 1 = rc 100).

    Tello rc는 lr 오른쪽 +, fb 앞 +, ud 위 +, yaw 시계 방향 +이다. 우리 좌표는 y 왼쪽 +, 반시계 +.
    수평은 rc_limit 안으로 방향을 지킨 채 줄인다. 축마다 따로 자르면 비스듬히 갈 때 방향이 틀어진다.
    """
    fb = vx / scale.v_full
    lr = -vy / scale.v_full
    h = math.hypot(fb, lr)
    if scale.v_dead > 0 and h > 1e-3:   # 무시 구간을 건너뛰도록 크기만 옮긴다 (방향은 그대로)
        k = (scale.v_dead + h * (1 - scale.v_dead)) / h
        fb, lr = fb * k, lr * k
    ud = vz / scale.vz_full
    if scale.vz_dead > 0:   # 무시 구간을 건너뛴다 (작은 명령은 0)
        ud = 0.0 if abs(ud) < VZ_HOLD else math.copysign(scale.vz_dead + abs(ud) * (1 - scale.vz_dead), ud)
    yaw = -wz / scale.yaw_full
    cap = rc_limit / 100.0
    m = max(abs(fb), abs(lr))
    if m > cap:
        fb, lr = fb * cap / m, lr * cap / m
    ud = max(-cap, min(cap, ud))
    yaw = max(-cap, min(cap, yaw))
    return lr, fb, ud, yaw


def raw_to_text(raw: dict) -> str:
    """TelloLink가 파싱한 원시값 → Tello가 보낸 형식의 상태 문자열 (`pitch:0;roll:0;...;`)."""
    parts = []
    for key, value in raw.items():
        if isinstance(value, (list, tuple)):
            value = ",".join(str(v) for v in value)
        parts.append(f"{key}:{value}")
    return ";".join(parts) + ";"


class TelloDriver:
    """TelloLink + FlightController 위에서 자율비행 노드의 명령을 받는다."""

    def __init__(self, link: TelloLink, scale: RcScale | None = None, rc_limit: int = 30,
                 height_limit_m: float = 1.5, cmd_timeout: float = CMD_TIMEOUT_S, log=print,
                 front_tof: bool = False, on_front_tof=None, temp_limits: tuple[float, float] | None = None,
                 battery_limits: tuple[float, float] | None = None, rc_slew: float = RC_SLEW_PER_S):
        self.link = link
        self.scale = scale or RcScale()
        self.rc_limit = rc_limit
        self.rc_slew = rc_slew                   # 초당 rc 상승 한도 (rc 100 = 1). 0이면 바로 바꾼다
        self._rc_prev = (0.0, 0.0, 0.0, 0.0)
        self._rc_mono: float | None = None
        self.cmd_timeout = cmd_timeout
        self.log = log
        # FlightController의 기본 기준은 수동 조종 게이트웨이용(배터리 이륙 20%·15% 이하 착륙, 온도는 경고만)이라
        # 자율비행 기준을 늘 넘긴다: 온도 이륙 80°C 미만·85°C 이상 착륙, 배터리 이륙 기준 없음·10% 이하 착륙(2026-11-02)
        temp_takeoff, temp_land = AUTONOMY_TEMP_LIMITS if temp_limits is None else temp_limits
        battery_takeoff, battery_land = AUTONOMY_BATTERY_LIMITS if battery_limits is None else battery_limits
        self.ctrl = FlightController(link, rc_limit=rc_limit, height_limit_m=height_limit_m, operator_controlled=False,
                                     temp_max_takeoff_c=temp_takeoff, temp_land_c=temp_land,
                                     battery_min_takeoff_pct=battery_takeoff, battery_land_pct=battery_land)
        self.connected = False
        self.last_cmd_mono = 0.0
        self.last_result: dict | None = None     # 마지막 이륙·착륙 요청 결과
        self.landing_reason: str | None = None   # 드라이버가 스스로 착륙시킨 이유
        self.front_tof = front_tof
        self.on_front_tof = on_front_tof or (lambda range_m, mono: None)   # (거리 m 또는 inf, 잰 순간 monotonic)
        self.front_tof_last: dict | None = None  # 마지막 parse_ext_tof 결과
        self._fresh = FreshFilter()
        self._keepalive_mono = 0.0
        self._keepalive: threading.Thread | None = None
        self._seen_states = 0
        self._job: threading.Thread | None = None
        self._stop = threading.Event()

    # ---------- 수명 ----------
    def start(self) -> "TelloDriver":
        self.ctrl.start()
        threading.Thread(target=self._connect, name="tello-connect", daemon=True).start()
        if self.front_tof:
            threading.Thread(target=self._front_tof_loop, name="tello-front-tof", daemon=True).start()
        return self

    def _connect(self) -> None:
        """`command`가 ok일 때까지 되풀이한다 (드론·시뮬레이터가 늦게 켜질 수 있다)."""
        tries = 0
        while not self._stop.is_set():
            tries += 1
            r = self.link.send("command", timeout=CONNECT_RETRY_S, source="driver")
            if r.ok:
                self.connected = True
                self.log(f"SDK 모드 연결 ({tries}번째 시도, {r.rtt_ms} ms)")
                if self.front_tof:   # 이륙 전에 한 번 물어 확장 키트가 답하는지 보여 준다
                    reading = parse_ext_tof(self.link.send("EXT tof?", timeout=FRONT_TOF_TIMEOUT_S,
                                                           source="driver").response)
                    self.front_tof_last = reading
                    self.log("전방 ToF(확장 키트): " + (f"{reading['front_tof_m']:.3f} m" if reading["valid"]
                                                      else f"값 없음 ({reading['reason']}, 응답 {reading['raw']!r})"))
                return
            if tries == 1 or tries % 5 == 0:
                self.log(f"연결 대기 중: command → {r.reason or r.response} ({tries}번째)")

    def close(self) -> None:
        self._stop.set()
        self.ctrl.close()        # 떠 있으면 착륙한다
        self.link.close()

    # ---------- 입력 ----------
    def on_cmd_vel(self, vx: float, vy: float, vz: float, wz: float) -> None:
        now = time.monotonic()
        self.last_cmd_mono = now
        if not (self.ctrl.flying and self.ctrl.busy is None):
            self._rc_prev, self._rc_mono = (0.0, 0.0, 0.0, 0.0), None
            return
        vals = to_input(self.scale, self.rc_limit, vx, vy, vz, wz)
        if self.rc_slew > 0:
            dt = 0.1 if self._rc_mono is None else min(0.2, now - self._rc_mono)
            vals = tuple(slew(p, t, self.rc_slew * dt) for p, t in zip(self._rc_prev, vals))
        self._rc_prev, self._rc_mono = vals, now
        self.ctrl.set_input(*vals, source="autonomy")

    def request(self, cmd: str, source: str = "autonomy") -> bool:
        """takeoff / land / emergency / confirm / unconfirm. 받아들였으면 True (결과는 status()의 last_result).

        confirm·unconfirm은 운용자 확인(주변 3 m 비움·가드·안전 담당)을 켜고 끈다. 웹 화면의 체크박스가 보낸다.
        FlightController가 착륙·비상 정지 뒤에 다시 끄므로 매번 다시 확인해야 한다.
        """
        if cmd == "emergency":
            self.last_result = {"cmd": cmd, **self.ctrl.emergency(source=source)}
            return True
        if cmd in ("confirm", "unconfirm"):
            if cmd == "confirm" and (self.ctrl.flying or self.ctrl.busy):
                return False
            self.ctrl.confirm_operator(cmd == "confirm")
            return True
        if cmd not in ("takeoff", "land"):
            return False
        if cmd == "takeoff" and (self.ctrl.flying or self.ctrl.busy or not self.connected):
            return False
        if cmd == "land" and not self.ctrl.flying and self.ctrl.busy != "takeoff":
            return False
        if self._job and self._job.is_alive() and cmd == "takeoff":
            return False
        self._job = threading.Thread(target=self._run, args=(cmd, source), name=f"tello-{cmd}", daemon=True)
        self._job.start()
        return True

    def _run(self, cmd: str, source: str) -> None:
        if cmd == "takeoff":
            r = self.ctrl.takeoff(source=source)
            self.last_cmd_mono = time.monotonic()   # 이륙에 걸린 시간을 명령 끊김으로 세지 않는다
            if self.ctrl.flying:
                self.landing_reason = None
        else:
            r = self.ctrl.land(source=source)
        self.last_result = {"cmd": cmd, "source": source, **r}
        self.log(f"{cmd} 결과: {'성공' if r.get('ok') else '실패'} (떠 있음: {self.ctrl.flying})"
                 + ("" if r.get("ok") else f" {r.get('reason') or (r.get('result') or {}).get('reason')}"))

    # ---------- 주기 처리 ----------
    def poll_state(self) -> str | None:
        """새 상태 패킷이 왔으면 Tello 형식 문자열, 아니면 None."""
        count = self.link.state_count
        if count == self._seen_states:
            return None
        self._seen_states = count
        state = self.link.latest_state()
        return raw_to_text(state["raw"]) if state else None

    def _front_tof_loop(self) -> None:
        while not self._stop.wait(FRONT_TOF_PERIOD_S):
            # 이륙·착륙 중에는 묻지 않는다. 늦게 온 tof 응답을 takeoff·land의 응답으로 읽으면 떠 있는지 잘못 안다
            if self.connected and self.ctrl.flying and self.ctrl.busy is None:
                self.poll_front_tof()

    def poll_front_tof(self) -> float | None:
        """`EXT tof?` 한 번. 새 값이면 on_front_tof로 넘기고 거리(m, 범위 밖 inf)를 돌려준다.

        같은 값은 0.2초에 한 번만 넘긴다(tof_fresh). 비행 노드는 두 번 연속 가까워야 장애물로 보므로,
        같은 응답을 두 번 넘기면 한 번 튄 값도 장애물이 된다. 범위 밖만 inf로 넘기고 오류·무응답은 버린다.
        """
        t0 = time.monotonic()
        r = self.link.send("EXT tof?", timeout=FRONT_TOF_TIMEOUT_S, source="driver")
        t1 = time.monotonic()
        reading = parse_ext_tof(r.response)
        self.front_tof_last = reading
        if self._fresh.update(reading, t1) is None:
            return None
        if reading["valid"]:
            range_m = reading["front_tof_m"]
        elif reading["reason"] == "out_of_range":
            range_m = math.inf
        else:
            return None
        self.on_front_tof(range_m, (t0 + t1) / 2)   # 잰 순간은 왕복의 가운데로 본다
        return range_m

    def tick(self) -> None:
        """비행 중 속도 명령이 끊기면 착륙한다. 제자리 유지(데드맨)는 FlightController가 이미 한다.

        땅에 있을 때는 KEEPALIVE_S마다 `command`를 보내 SDK 모드를 유지한다(떠 있으면 rc가 그 역할을 한다).
        """
        now = time.monotonic()
        if (self.connected and not self.ctrl.flying and self.ctrl.busy is None and now - self._keepalive_mono > KEEPALIVE_S
                and not (self._keepalive and self._keepalive.is_alive())
                and not (self._job and self._job.is_alive())):
            self._keepalive_mono = now
            self._keepalive = threading.Thread(target=self.link.send, args=("command",),
                                               kwargs={"timeout": 1.0, "source": "driver:keepalive"},
                                               name="tello-keepalive", daemon=True)
            self._keepalive.start()
        if not self.ctrl.flying or self.ctrl.busy is not None or self.landing_reason:
            return
        if time.monotonic() - self.last_cmd_mono > self.cmd_timeout:
            self.landing_reason = "cmd_timeout"
            self.log(f"속도 명령이 {self.cmd_timeout:.1f}초 넘게 끊겼습니다. 착륙합니다.")
            self.request("land", source="safety:cmd_timeout")

    def status(self) -> dict:
        s = self.ctrl.status()
        s.update(connected=self.connected, last_result=self.last_result, landing_reason=self.landing_reason,
                 state_rate_hz=self.link.state_rate_hz(), source=self.link.source)
        s.update(battery_land_pct=self.ctrl.battery_land_pct, temp_land_c=self.ctrl.temp_land_c)   # 웹 화면 안내문
        if self.front_tof:
            s["front_tof"] = self.front_tof_last
        return s
