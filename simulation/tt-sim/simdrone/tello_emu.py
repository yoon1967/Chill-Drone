"""(시뮬레이션 전용) Tello EDU 흉내 중 ROS·Gazebo와 무관한 부분. 시각은 모두 시뮬레이션 시각(초)을 받는다.

StateSynth  Gazebo 정답 → Tello 상태 문자열. 잡음·속도 배율 오차·yaw 누적을 넣는다
            (sim_tello_state.py와 gz_tello.py가 같이 쓴다)
TelloEmu    SDK 명령 처리와 비행 단계(땅 → 이륙 → 비행 → 착륙), rc → 기체 속도

가정값 (실기체로 다시 잰다):
  rc 100 = 수평 1.0 m/s, 상하 1.0 m/s, 회전 100°/s. 속도는 시상수 0.3 s(회전 0.15 s)로 따라간다
  이륙: 모터 2 s → 0.25 m/s로 0.8 m까지 → 0.5 s 멈춘 뒤 'ok' (실측: 응답까지 8.6 s, 약 80 cm에서 멈춤)
  착륙: 0.3 m/s로 바닥까지 내려간 뒤 'ok' (실측: 1.3 m 안팎에서 3.2–4.6 s)
  비행 중 15 s 동안 명령이 없으면 스스로 착륙 (Tello 동작). rc는 떠 있을 때만 듣는다
  `EXT tof?`(확장 키트 전방 ToF): front_tof_m을 `tof <mm>`로, 1.2 m 밖은 실측처럼 `tof 8190`으로 답한다.
  front_tof_m이 None이면(ToF를 안 받음) 키트가 없는 것처럼 답하지 않는다
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass

from autonomy import tello_state
from autonomy.tello_state import wrap

GROUND, TAKEOFF, FLYING, LANDING, FALLING = "ground", "takeoff", "flying", "landing", "falling"
QUERY_REPLIES = {"sdk?": "30", "sn?": "SIMTELLO00001", "wifi?": "90", "speed?": "10.0", "temp?": "60~63C"}
EXT_TOF_MAX_M = 1.2
EXT_TOF_OUT_OF_RANGE = 8190   # 실기체(esp32v1.0.0.12) 범위 밖 응답


# 닿음 반력 (11/13, autonomy/contact.py를 Gazebo에서 보려고): 실기체 Tello의 agx·agy는 기체 기준 비중력 가속이라 자유롭게 날 때는
# 작고(최대 93 mg), 무엇에 닿으면 그 반력이 나온다(부딪힘 0.65–3.1 g, 기대어 밀 때 0.1–0.4 g). Gazebo는 기체를 속도로 움직이므로
# 걸린 만큼 '명령한 속도 − 실제 속도'가 생긴다. 그 차이(CONTACT_DEAD 넘는 몫)에 CONTACT_GAIN을 곱해 반대쪽 가속으로 낸다.
# 0.2 m/s로 가다 막히면 0.375 g. Tello가 제자리를 지키려 계속 미는 것(실기체 11/13)은 흉내 내지 않는다
CONTACT_GAIN = 2.5     # g / (m/s)
CONTACT_DEAD = 0.05    # m/s: 속도 추종 지연·잡음
ACC_NOISE = 10.0       # mg


def contact_accel(cmd, actual) -> tuple[float, float]:
    """기체 기준 수평 속도 명령·실제 (앞, 왼쪽 m/s) → Tello agx(앞 +)·agy(오른쪽 +) (0.001 g)."""
    ex, ey = cmd[0] - actual[0], cmd[1] - actual[1]
    n = math.hypot(ex, ey)
    if n <= CONTACT_DEAD:
        return 0.0, 0.0
    k = CONTACT_GAIN * 1000.0 * (n - CONTACT_DEAD) / n
    return -k * ex, k * ey   # 반력은 미는 쪽의 반대 (왼쪽 +인 ey를 오른쪽 +로 바꾸면 부호가 한 번 더 뒤집힌다)


def ext_tof_reply(front_tof_m: float | None) -> str | None:
    """전방 거리(m, 못 보면 inf) → `EXT tof?` 응답. None이면 응답 없음."""
    if front_tof_m is None:
        return None
    if not math.isfinite(front_tof_m) or front_tof_m > EXT_TOF_MAX_M:
        return f"tof {EXT_TOF_OUT_OF_RANGE}"
    return f"tof {round(front_tof_m * 1000)}"


class StateSynth:
    """Gazebo 정답 위치 → Tello 상태 문자열. 값은 sim_tello_state.py에 있던 흉내 그대로다.

    초당 10번, 속도 0.1 m/s 단위 정수 + 잡음 0.03 m/s + 비행마다 배율 오차(1σ 3%),
    yaw 1° 단위 + 잡음 0.3° + 누적 1°/분 (0도 방향은 전원 켤 때 방향이라 임의), 높이 h 1 cm 단위 + 잡음 2 cm.
    """

    def __init__(self, seed: int = 0, vel_noise: float = 0.03, scale_sigma: float = 0.03, yaw_noise_deg: float = 0.3,
                 yaw_drift_deg_per_min: float = 1.0, h_noise: float = 0.02):
        self.rng = random.Random(int(seed))
        self.vel_noise = vel_noise
        self.scale = 1.0 + self.rng.gauss(0, scale_sigma)              # 이 비행의 속도 배율 오차
        self.yaw_noise = math.radians(yaw_noise_deg)
        self.yaw_drift = math.radians(yaw_drift_deg_per_min) / 60.0
        self.h_noise = h_noise
        self.yaw_zero = self.rng.uniform(-math.pi, math.pi)              # Tello yaw 0이 가리키는 지도 방향
        self.prev = None
        self.t0 = None

    def sample(self, t: float, x: float, y: float, z: float, yaw: float, z0: float, tof_down: float | None,
               extra: dict | None = None, cmd: tuple | None = None) -> str:
        """cmd: 지금 기체에 건 수평 속도 명령 (기체 기준 앞·왼쪽 m/s). 주면 닿음 반력 가속(agx·agy)을 흉내 낸다 (contact_accel)."""
        if self.t0 is None:
            self.t0 = t
        vx_w = vy_w = vz = 0.0
        if self.prev is not None and t - self.prev[0] > 1e-3:
            dt = t - self.prev[0]
            vx_w, vy_w, vz = (x - self.prev[1]) / dt, (y - self.prev[2]) / dt, (z - self.prev[3]) / dt
        self.prev = (t, x, y, z)
        if cmd is not None:
            agx, agy = contact_accel(cmd, (math.cos(yaw) * vx_w + math.sin(yaw) * vy_w, -math.sin(yaw) * vx_w + math.cos(yaw) * vy_w))
            extra = {**(extra or {}), "agx": agx + self.rng.gauss(0, ACC_NOISE), "agy": agy + self.rng.gauss(0, ACC_NOISE)}
        # takeoff: 실기체처럼 Tello가 yaw 0이라고 믿는 방향 기준 (yaw가 흐르면 그 기준도 같이 흐른다)
        frame_yaw = yaw if tello_state.CONVENTION["vel_frame"] == "body" else self.yaw_zero - self.yaw_drift * (t - self.t0)
        c, s = math.cos(frame_yaw), math.sin(frame_yaw)
        vx = (c * vx_w + s * vy_w) * self.scale + self.rng.gauss(0, self.vel_noise)
        vy = (-s * vx_w + c * vy_w) * self.scale + self.rng.gauss(0, self.vel_noise)
        yaw_reading = wrap(yaw - self.yaw_zero + self.yaw_drift * (t - self.t0) + self.rng.gauss(0, self.yaw_noise))
        h = max(0.0, z - z0 + self.rng.gauss(0, self.h_noise))
        bat = max(0.0, 100.0 - (t - self.t0) / 60.0 * 8.0)
        return tello_state.encode(tello_state.from_si(yaw_reading, vx, vy, vz, h, tof_down, bat, extra=extra))


@dataclass
class EmuParams:
    v_full: float = 1.0                    # rc 100일 때 수평 속도 (m/s)
    vz_full: float = 1.0                   # rc 100일 때 상하 속도 (m/s)
    yaw_full: float = math.radians(100.0)  # rc 100일 때 회전 속도 (rad/s)
    tau: float = 0.3                       # 수평·상하 속도가 rc를 따라가는 시상수 (s)
    tau_yaw: float = 0.15
    spinup: float = 2.0                    # 이륙 명령 뒤 모터가 도는 시간 (s)
    takeoff_height: float = 0.8
    takeoff_speed: float = 0.25
    takeoff_settle: float = 0.5
    land_speed: float = 0.3
    idle_land: float = 15.0                # 비행 중 이 시간 동안 명령이 없으면 착륙
    ground_eps: float = 0.03               # 바닥에 닿았다고 볼 높이 (m)


class TelloEmu:
    """명령 문자열을 받아 응답하고, 매 주기 기체 속도(기체 기준 vx, vy, vz, yaw rate)를 정한다.

    takeoff·land는 동작이 끝난 뒤에 'ok'를 돌려준다(실기체와 같음): update()가 돌려주는 목록에 담긴다.
    """

    def __init__(self, p: EmuParams | None = None):
        self.p = p or EmuParams()
        self.phase = GROUND
        self.phase_t = 0.0
        self.reached_t: float | None = None
        self.rc = (0, 0, 0, 0)
        self.vel = [0.0, 0.0, 0.0, 0.0]
        self.last_cmd_t = 0.0
        self.last_t: float | None = None
        self.sdk = False
        self.waiting: str | None = None    # 'ok'를 아직 안 보낸 takeoff / land
        self.front_tof_m: float | None = None   # 확장 키트 전방 ToF (gz_tello.py가 Gazebo 값으로 채운다)
        self.log: list[tuple[float, str]] = []

    def _set_phase(self, t: float, phase: str) -> None:
        self.phase, self.phase_t, self.reached_t = phase, t, None
        self.log.append((t, phase))

    def command(self, t: float, cmd: str) -> str | None:
        """명령 하나를 처리한다. 바로 보낼 응답, 또는 None(rc처럼 응답이 없거나 동작이 끝난 뒤 응답)."""
        cmd = cmd.strip()
        self.last_cmd_t = t
        if cmd.startswith("rc "):
            try:
                vals = tuple(max(-100, min(100, int(v))) for v in cmd.split()[1:5])
            except ValueError:
                return None
            if len(vals) == 4 and self.phase == FLYING:
                self.rc = vals
            return None
        if cmd == "command":
            self.sdk = True
            return "ok"
        if not self.sdk:
            return None                    # SDK 모드 전에는 응답하지 않는다
        if cmd in ("streamon", "streamoff"):
            return "ok"
        if cmd == "emergency":
            self.rc, self.waiting = (0, 0, 0, 0), None
            if self.phase != GROUND:
                self._set_phase(t, FALLING)
            return "ok"
        if cmd == "takeoff":
            if self.phase != GROUND:
                return "error"
            self.rc, self.waiting = (0, 0, 0, 0), "takeoff"
            self._set_phase(t, TAKEOFF)
            return None
        if cmd == "land":
            if self.phase not in (FLYING, TAKEOFF):
                return "error"
            # 실기체는 land를 받아도 마지막 rc를 내려오는 동안 그대로 따른다 (2026-11-12 13:58: rc 0 -36 0 0이 남아 3.5초에
            # 0.83 m 뒤로 갔다). 그래서 rc를 지우지 않는다. 0으로 되돌리는 것은 드라이버(FlightController.land)가 한다
            self.waiting = "land"
            self._set_phase(t, LANDING)
            return None
        if cmd == "stop":
            self.rc = (0, 0, 0, 0)
            return "ok"
        if cmd == "battery?":
            return "87"
        if cmd.lower() == "ext tof?":
            return ext_tof_reply(self.front_tof_m)
        return QUERY_REPLIES.get(cmd, "ok")

    def update(self, t: float, h: float) -> tuple[tuple[float, float, float, float], list[str]]:
        """t: 지금 시각, h: 바닥에서 높이(m). → (기체 속도 명령, 지금 보낼 늦은 응답들)."""
        dt = 0.0 if self.last_t is None else max(0.0, t - self.last_t)
        self.last_t = t
        p, replies = self.p, []
        target = [0.0, 0.0, 0.0, 0.0]
        direct = False                      # 이륙·착륙은 기체가 스스로 하므로 지연 없이 바로 그 속도
        if self.phase == TAKEOFF:
            direct = True
            if t - self.phase_t >= p.spinup:
                if h < p.takeoff_height:
                    target[2] = p.takeoff_speed
                elif self.reached_t is None:
                    self.reached_t = t
                elif t - self.reached_t >= p.takeoff_settle:
                    self._set_phase(t, FLYING)
                    self.last_cmd_t = t
                    if self.waiting == "takeoff":
                        replies.append("ok")
                        self.waiting = None
        elif self.phase in (LANDING, FALLING):
            direct = True
            if h > p.ground_eps:
                target[2] = -p.land_speed if self.phase == LANDING else -1.5
                if self.phase == LANDING:   # 마지막 rc의 수평·회전은 남는다 (내려오는 중에 온 rc는 듣지 않는다고 본다)
                    lr, fb, _, yaw = self.rc
                    target[0], target[1], target[3] = fb / 100 * p.v_full, -lr / 100 * p.v_full, -yaw / 100 * p.yaw_full
            else:
                was_landing = self.phase == LANDING
                self.rc = (0, 0, 0, 0)
                self._set_phase(t, GROUND)
                if was_landing and self.waiting == "land":
                    replies.append("ok")
                self.waiting = None
        elif self.phase == FLYING:
            if t - self.last_cmd_t > p.idle_land:
                self.rc = (0, 0, 0, 0)
                self._set_phase(t, LANDING)
            else:
                lr, fb, ud, yaw = self.rc
                target = [fb / 100 * p.v_full, -lr / 100 * p.v_full, ud / 100 * p.vz_full, -yaw / 100 * p.yaw_full]
        if direct or self.phase == GROUND:
            self.vel = target
        else:
            for i, tau in enumerate((p.tau, p.tau, p.tau, p.tau_yaw)):
                a = 1.0 - math.exp(-dt / tau) if tau > 0 else 1.0
                self.vel[i] += a * (target[i] - self.vel[i])
        return tuple(self.vel), replies
