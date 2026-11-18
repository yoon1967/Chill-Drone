"""앞이 확실히 비어 있을 때만 평소 속도(speed)보다 빠르게(fast) 가기. 2026-10-30 저녁 요청.

'확실히 비었다'는 세 가지가 다 멀 때다: 카메라 장면 정면 ±15°의 가까운 거리, 정면 ToF(범위 밖이면 무한), 경로(탐색)나 띠·울타리
(둘러보기)가 기수 방향으로 곧게 이어지는 길이. 그중 가장 짧은 것이 FAST_FROM부터 FAST_FULL까지 늘어나는 동안 속도를 speed에서
fast까지 선형으로 올린다. 카메라 장면이 없으면(ToF만) 올리지 않는다: ToF는 1.2 m밖에 못 보고, 10/29에 0.4 m/s로 0.62 m에서
처음 본 모니터에 닿았다.
"""
from __future__ import annotations

import math

FAST_FROM = 1.5                 # 이보다 가까이 무엇이 있으면 평소 속도 (m)
FAST_FULL = 3.0                 # 이보다 멀리까지 비어 있으면 fast 그대로 (m)
FAST_FOV = math.radians(15)     # 카메라 장면에서 '정면'으로 보는 각 (±)
STRAIGHT_TURN = math.radians(20)   # 경로가 기수에서 이보다 꺾이면 거기까지만 곧은 길로 센다


def fast_factor(clear: float | None, lo: float = FAST_FROM, hi: float = FAST_FULL) -> float:
    """비어 있는 거리 → 0(평소 속도)–1(fast)."""
    if clear is None or not math.isfinite(clear):
        return 1.0 if clear is not None and clear > hi else 0.0
    return max(0.0, min(1.0, (clear - lo) / (hi - lo)))


def boosted(v: float, base: float, fast: float, k: float) -> float:
    """평소 속도 base로 낸 명령 v를 k(0–1)만큼 fast 쪽으로 올린다. fast가 base보다 작거나 v가 0이면 그대로."""
    if fast <= base or v <= 0.0 or base <= 0.0 or k <= 0.0:
        return v
    return min(v * (1.0 + k * (fast / base - 1.0)), fast)


def straight_ahead(x: float, y: float, yaw: float, points, max_turn: float = STRAIGHT_TURN) -> float:
    """(x, y)에서 경유점들을 따라 기수 yaw 방향에서 max_turn 안으로 곧게 이어지는 길이 (m). 꺾이는 점에서 멈춘다."""
    total, px, py = 0.0, x, y
    for q in points:
        dx, dy = float(q[0]) - px, float(q[1]) - py
        seg = math.hypot(dx, dy)
        if seg < 1e-6:
            continue
        d = math.atan2(dy, dx) - yaw
        if abs(math.atan2(math.sin(d), math.cos(d))) > max_turn:
            break
        total += seg
        px, py = float(q[0]), float(q[1])
    return total
