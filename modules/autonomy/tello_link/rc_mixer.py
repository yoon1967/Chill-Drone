"""조종 입력 → Tello rc 변환 (표준 라이브러리만 사용).

계약: contracts/api/AUTONOMY_BRIDGE_V0_1.md 5장, DRONE_CONTROL_V0_1.md 6.3절.
- 자율 속도 명령(m/s, rad/s, 기체 기준)을 rc로 바꾼다. 수평은 같은 비율로 줄여 대각선 방향을 지킨다.
- 사람 입력(-1~1)은 데드존을 빼고 rc 상한 안으로 늘린다.
- 가감속 제한: 커지는 쪽만 천천히, 줄어드는 쪽(멈춤)은 바로.
rc 값은 조종 입력 비율이지 속도가 아니다. K_LIN·K_YAW는 보정 실험 전 가정값이다.
"""

from __future__ import annotations

import math

RC_LIMIT = 30              # 처음 비행 상한 (±100 중 30)
K_LIN = 100.0              # rc / (m/s). 가정값, rc 보정 실험으로 확정(-274)
K_YAW = 57.0               # rc / (rad/s). 드라이버 가정(rc 100 = 100°/s), 실측 전
DEADZONE = 0.15            # 사람 입력 데드존. 이 밖이면 자율 중 넘겨받기로 본다
SLEW_RC_PER_S = 150.0      # 초당 rc 변화 상한(커지는 쪽만). 0 → 30이 0.2초. 비행으로 조정

Rc = tuple[int, int, int, int]  # (좌우, 앞뒤, 상하, 회전). Tello rc 순서·부호(오른쪽·앞·위·시계 +)


def max_speed_mps(rc_limit: int = RC_LIMIT, k_lin: float = K_LIN) -> float:
    return rc_limit / k_lin


def _finite(*values) -> bool:
    return all(isinstance(v, (int, float)) and math.isfinite(v) for v in values)


def _clamp(v: float, limit: int) -> int:
    return int(max(-limit, min(limit, round(v))))


def cmd_vel_to_rc(vx_mps: float, vy_mps: float, vz_mps: float, yaw_rate_rps: float, *,
                  rc_limit: int = RC_LIMIT, k_lin: float = K_LIN, k_yaw: float = K_YAW) -> Rc | None:
    """자율 속도 명령을 rc로 바꾼다. 숫자가 아니거나 무한이면 None(버린다).

    좌표계: x 앞, y 왼쪽, z 위, yaw 반시계 +. Tello rc는 오른쪽·시계 방향이 +라 y·yaw 부호를 뒤집는다.
    """
    if not _finite(vx_mps, vy_mps, vz_mps, yaw_rate_rps):
        return None
    speed = math.hypot(vx_mps, vy_mps)
    vmax = max_speed_mps(rc_limit, k_lin)
    s = 1.0 if speed <= vmax or speed == 0 else vmax / speed
    return (
        _clamp(-vy_mps * s * k_lin, rc_limit),
        _clamp(vx_mps * s * k_lin, rc_limit),
        _clamp(vz_mps * k_lin, rc_limit),
        _clamp(-yaw_rate_rps * k_yaw, rc_limit),
    )


def apply_deadzone(axes, deadzone: float = DEADZONE) -> tuple[float, float, float, float]:
    """-1~1 입력에서 데드존을 빼고 남은 범위를 다시 0~1로 늘린다. 숫자가 아닌 값은 0."""
    out = []
    for v in axes:
        v = float(v) if _finite(v) else 0.0
        v = max(-1.0, min(1.0, v))
        mag = abs(v)
        out.append(0.0 if mag <= deadzone else math.copysign((mag - deadzone) / (1.0 - deadzone), v))
    return tuple(out)


def outside_deadzone(axes, deadzone: float = DEADZONE) -> bool:
    return any(v != 0.0 for v in apply_deadzone(axes, deadzone))


def operator_axes_to_rc(axes, *, rc_limit: int = RC_LIMIT, deadzone: float = DEADZONE) -> Rc:
    """사람 입력 [좌우, 앞뒤, 상하, 회전](-1~1, 오른쪽·앞·위·시계 +)을 rc로. 끝까지 밀면 rc_limit."""
    return tuple(_clamp(v * rc_limit, rc_limit) for v in apply_deadzone(axes, deadzone))


class SlewLimiter:
    """rc가 갑자기 커지지 않게 한다. 멈추거나 줄이는 쪽은 바로 따른다(손 떼면 바로 멈춤)."""

    def __init__(self, rate_rc_per_s: float = SLEW_RC_PER_S):
        self.rate = rate_rc_per_s
        self.last: Rc = (0, 0, 0, 0)
        self._last_t: float | None = None

    def reset(self) -> None:
        self.last, self._last_t = (0, 0, 0, 0), None

    def step(self, target: Rc, now: float) -> Rc:
        dt = 0.0 if self._last_t is None else max(0.0, now - self._last_t)
        self._last_t = now
        if self.rate <= 0:
            self.last = tuple(target)
            return self.last
        max_step = self.rate * dt
        out = []
        for prev, tgt in zip(self.last, target):
            if tgt == 0 or (prev != 0 and (tgt > 0) != (prev > 0)) or abs(tgt) <= abs(prev):
                # 멈춤·방향 반대·줄어듦: 줄이는 건 바로. 반대 방향은 0에서 다시 키운다.
                nxt = 0 if (prev != 0 and tgt != 0 and (tgt > 0) != (prev > 0)) else tgt
            else:
                nxt = prev + math.copysign(min(abs(tgt) - abs(prev), max_step), tgt)
            out.append(int(round(nxt)))
        self.last = tuple(out)
        return self.last
