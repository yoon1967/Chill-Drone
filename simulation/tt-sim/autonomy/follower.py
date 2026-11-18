"""경로 점을 하나씩 따라가는 속도 명령 계산. ROS와 무관한 순수 함수라 실기체 어댑터에도 그대로 쓴다.

TT는 카메라와 ToF가 앞쪽만 보므로, 옆이나 뒤로 미끄러지지 않고
먼저 진행 방향으로 기수를 돌린 뒤 앞으로 간다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class FollowerParams:
    speed: float = 0.4            # 수평 최대 속도 (m/s)
    vz_max: float = 0.3           # 수직 최대 속도 (m/s)
    yaw_rate_max: float = 0.8     # 최대 회전 속도 (rad/s)
    k_pos: float = 0.8            # 거리 → 속도 비례 이득 (목적지 근처에서 감속)
    k_yaw: float = 1.5            # 방향 오차 → 회전 속도 이득
    turn_first_deg: float = 25.0  # 방향 오차가 이보다 크면 제자리에서 먼저 돈다 (35°였다: 11/03 옆으로 미끄러짐이 보여 줄임)
    lateral_max: float = 0.05     # 옆(기체 y) 속도 상한 (m/s). 옆은 보는 센서가 없어 미끄러지지 않고 기수를 돌려 앞으로 간다
    accept: float = 0.15          # 마지막 점 도착 판정 거리 (m)
    accept_pass: float = 0.15     # 중간 경유점 통과 판정 거리 (m). 크면 모서리를 질러가 장애물에 붙는다


@dataclass
class Command:
    """기체 기준 속도 (x 앞, y 왼쪽, z 위, 요 회전). /tt/cmd_vel에 그대로 쓴다."""
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    yaw_rate: float = 0.0


def wrap(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


def _clip(v: float, limit: float) -> float:
    return max(-limit, min(limit, v))


def follow(pose, target, p: FollowerParams, final: bool) -> tuple[Command, bool]:
    """pose (x, y, z, yaw)에서 target (x, y, z)로 가는 명령과 도착 여부."""
    x, y, z, yaw = pose
    dx, dy, dz = target[0] - x, target[1] - y, target[2] - z
    dist_xy = math.hypot(dx, dy)
    if math.hypot(dist_xy, dz) < (p.accept if final else p.accept_pass):
        return Command(), True

    yaw_err = wrap(math.atan2(dy, dx) - yaw) if dist_xy > 0.1 else 0.0
    cmd = Command(yaw_rate=_clip(p.k_yaw * yaw_err, p.yaw_rate_max))

    if abs(yaw_err) <= math.radians(p.turn_first_deg) and dist_xy > 1e-6:
        speed = min(p.speed, p.k_pos * dist_xy) if final else p.speed
        speed *= math.cos(yaw_err)
        wx, wy = speed * dx / dist_xy, speed * dy / dist_xy          # 월드 기준 속도
        cmd.vx = math.cos(yaw) * wx + math.sin(yaw) * wy             # 기체 기준으로 바꾼다
        cmd.vy = _clip(-math.sin(yaw) * wx + math.cos(yaw) * wy, p.lateral_max)
    cmd.vz = _clip(p.k_pos * dz, p.vz_max)
    return cmd, False
