"""가까운 장애물 앞에서 움직이는 규칙 (ROS와 무관).

2026-11-02 17:33 실기체(기록 20261102_173332)는 유리와 유리 사이로 들어가다 추락했다.
- 유리는 카메라 깊이에 안 보이고(뒤가 보인다) 정면 ToF도 1.2 m 안이 비어 있다가 0.18 m에서 처음 잡았다.
- 그 자리에서 회전 rc 80으로 제자리 회전했고(기체 20×20 cm라 모서리가 중심에서 14 cm까지 돈다),
- 비상 정지 뒤 '지금 기수의 반대쪽'으로 물러났는데 돌고 난 뒤라 들어온 길이 아니었다. 기수가 저절로 142° → 69°로 돌며
  (옆 유리에 닿음) 뒤집혔다.
그래서: 늦게 잡힌 ToF는 유리로 보고 넓게 막고, 가까우면 돌기 전에 들어온 길로 천천히 물러나고, 물러나는 동안 기수가
저절로 돌거나 기울면 멈춘다. 틈은 폭 30 cm·높이 20 cm보다 좁으면 지나가지 않는다(사용자 기준).
좁은 곳에 들어갔다가 나올 때(11/03, 사용자: '정확한 각도를 만들어 나왔으면 괜찮았을 것'): 제자리에서 돌지 않고 들어온 자취를
그대로 거꾸로 되짚는다(trail_cursor·trail_target). 지나온 선은 실제로 비어 있던 곳이라 카메라·ToF가 못 보는 유리가 있어도 안전하다.
"""
from __future__ import annotations

import math

GAP_MIN_WIDTH = 0.30              # 이보다 좁은 틈은 지나가지 않는다 (m, 기체는 가드 포함 20×20×10 cm) — 경로 여유 하한은 그 절반
GAP_MIN_HEIGHT = 0.20
EXIT_WIDTH = 0.8                  # 지도에서 양옆 빈 폭이 이보다 좁으면 '좁은 곳' (문 0.8–0.9 m는 아님). 유리는 지도에 없을 수 있다
TRAIL_STEP = 0.1                  # 지나온 자취를 남기는 간격 (m)
TRIM_RISE = 0.15                  # 자취를 자를 점을 찾을 때, 거리가 가장 가까웠던 값보다 이만큼 다시 멀어지면 그 다리는 끝난 것 (m)


def forward_speed(history, window: float = 0.3) -> float:
    """최근 위치 history[(시각, (x, y, z, yaw)), …](오래된 것 → 최근)로 잰 기체 앞 방향 속도 (m/s, 뒤로면 음수).
    window 초 앞 위치와 지금 위치의 차이를 지금 기수로 본다. 위치가 모자라면 0."""
    if len(history) < 2:
        return 0.0
    t1, p1 = history[-1]
    k = len(history) - 1
    while k > 0 and t1 - history[k][0] < window:
        k -= 1
    t0, p0 = history[k]
    dt = t1 - t0
    if dt < window / 2:
        return 0.0
    c, s = math.cos(p1[3]), math.sin(p1[3])
    return ((p1[0] - p0[0]) * c + (p1[1] - p0[1]) * s) / dt


def turn_brake(forward: float, start: float, gain: float, low: float, high: float) -> float | None:
    """제자리에서 돌기 전 제동: 앞 속도 forward(m/s)가 start보다 크면 낼 뒤 속도(음수, low–high 크기), 아니면 None.

    11/05 13:04 실기체: 0.2 m/s로 가다 크게 틀어야 하면 앞 명령을 0으로 끊고 바로 돌았는데, Tello는 바로 서지 않아 도는 동안
    0.10 m/s(빠를 때 0.27) 앞으로 밀려갔다(그런 회전 17초). 3:12에는 45°→91°로 돌며 0.2 m 밀려 나무 문짝 0.20 m 앞에서 비상 정지했다.
    물러나기 시작 때의 제동과 같이, 먼저 뒤로 짧게 밀어 관성을 끊은 뒤 돈다. low는 rc 무시 구간(실기체 약 0.095 m/s)보다 크게."""
    if forward <= start:
        return None
    return -min(high, max(low, gain * forward))


def late_hit(prev: float, front: float, near: float, max_range: float) -> bool:
    """정면 ToF가 비어 있다가(prev가 범위 밖) near 안에서 처음 잡혔는가. 다가오는 벽은 보통 max_range 근처에서 먼저 잡힌다."""
    return not (math.isfinite(prev) and prev < max_range - 0.05) and math.isfinite(front) and front < near


def retreat_direction(past_xy, here, min_dist: float):
    """들어온 길로 물러날 방향 (단위 벡터). past_xy는 최근 위치(오래된 것 → 최근 순). here에서 min_dist 넘게 떨어진
    가장 최근 위치 쪽이다 — 방금 지나온 곳이라 비어 있다. 그런 곳이 없으면(제자리에만 있었으면) None."""
    hx, hy = here
    for px, py in reversed(list(past_xy)):
        d = math.hypot(px - hx, py - hy)
        if d >= min_dist:
            return (px - hx) / d, (py - hy) / d
    return None


def world_to_body(vx_w: float, vy_w: float, yaw: float) -> tuple[float, float]:
    """세계 좌표 속도 → 기체 기준 (앞, 왼쪽)."""
    c, s = math.cos(yaw), math.sin(yaw)
    return c * vx_w + s * vy_w, -s * vx_w + c * vy_w


def limit_turn(rate: float, front: float, slow_range: float, slow_rate: float, force: bool = False) -> float:
    """정면이 slow_range 안이면(또는 force) 회전 속도를 slow_rate로 줄인다."""
    if force or (math.isfinite(front) and front < slow_range):
        return max(-slow_rate, min(slow_rate, rate))
    return rate


def contact(yaw_drift: float, tilt_deg: float, max_drift: float, max_tilt: float) -> str | None:
    """회전 명령 없이 물러나는데 기수가 저절로 돌았거나 크게 기울었으면 닿은 것으로 본다. 이유 문장, 아니면 None."""
    if tilt_deg >= max_tilt:
        return f"기울기 {tilt_deg:.0f}°"
    if abs(yaw_drift) >= max_drift:
        return f"기수가 저절로 {math.degrees(abs(yaw_drift)):.0f}° 돎"
    return None


def free_width(occupied_at, here, limit: float, step: float = 0.05, directions: int = 12) -> float:
    """here(x, y, z)에서 서로 반대인 두 방향으로 막힌 칸까지 거리를 더한 폭 중 가장 좁은 것 (m, 한쪽은 limit까지만 잰다).
    한쪽만 막힌 벽 옆은 limit 넘게, 양옆이 막힌 복도·틈은 그 폭이 나온다."""
    x, y, z = here
    best = 2 * limit
    for k in range(directions):
        a = math.pi * k / directions
        width = 0.0
        for sgn in (1, -1):
            d = step
            while d < limit and not occupied_at((x + sgn * d * math.cos(a), y + sgn * d * math.sin(a), z)):
                d += step
            width += min(d, limit)
        best = min(best, width)
    return best


def room_around(occupied_at, here, limit: float, step: float = 0.05, directions: int = 16) -> float:
    """here(x, y, z)에서 가장 가까운 막힌 곳까지 거리 (m, directions 방향, limit까지만). 제자리에서 돌 자리가 있는지 본다 —
    기체 모서리는 중심에서 0.14 m까지 돈다(20×20 cm), 옆은 보는 센서가 없다."""
    x, y, z = here
    best = limit
    for k in range(directions):
        a = 2 * math.pi * k / directions
        d = step
        while d < best and not occupied_at((x + d * math.cos(a), y + d * math.sin(a), z)):
            d += step
        best = min(best, d)
    return best


def sector_clear(occupied_at, here, yaw: float, side: int, sector=(math.radians(20), math.radians(70)),
                 limit: float = 1.0, rays: int = 6, step: float = 0.05) -> float:
    """정면 왼쪽(side +1)·오른쪽(−1) 부채꼴 sector(기수에서의 각 범위)에서 가장 가까운 막힌 곳까지 거리 (m, limit까지만)."""
    x, y, z = here
    best = limit
    a0, a1 = sector
    for k in range(rays):
        a = yaw + side * (a0 + (a1 - a0) * k / max(rays - 1, 1))
        c, s = math.cos(a), math.sin(a)
        d = 2 * step
        while d < best and not occupied_at((x + d * c, y + d * s, z)):
            d += step
        best = min(best, d)
    return best


def pocket(occupied_at, here, yaw: float, turn_room: float, limit: float = 1.0,
           sector=(math.radians(20), math.radians(70))) -> tuple[bool, float, float, float]:
    """막힌 주머니인가: 돌 자리가 없고(둘레 turn_room 안이 막힘) 정면 왼쪽·오른쪽 부채꼴 limit 안이 모두 막혔다
    (11/03 사용자: '정면 좌·우 1 m 안에 장애물이 모두 있을 때만 후진'). (주머니, 둘레 빈 거리, 왼쪽, 오른쪽)."""
    room = room_around(occupied_at, here, 1.0)
    left, right = sector_clear(occupied_at, here, yaw, 1, sector, limit), sector_clear(occupied_at, here, yaw, -1, sector, limit)
    return room < turn_room and left < limit and right < limit, room, left, right


def rear_unknown(known_free_at, here, yaw: float, length: float, half_width: float, start: float = 0.1,
                 step: float = 0.05) -> float | None:
    """기체 뒤쪽으로 곧게 물러날 띠(중심에서 start–length m, 폭 ±half_width)에서 '본 빈 칸'·'지나온 칸'이 아닌 첫 곳까지
    거리 (m). 띠가 모두 그런 칸이면 None.

    뒤를 보는 센서가 없으니 카메라가 본 적 있거나 지나온 곳으로만 물러난다. 11/09 13:03 실기체: 이륙 직후 제자리에서 둘러보다
    (79° 돌았을 때) 옆 의자에 앉은 사람이 0.44 m 앞에 잡혀 '들어온 길로 0.3 m 물러나기'를 했는데, 막 떠서 들어온 길이 없었고
    카메라가 한 번도 보지 못한 뒤쪽으로 곧게 후진하다 2.3초 만에 무언가에 닿아 뒤집혀 떨어졌다."""
    x, y, z = here
    bx, by = -math.cos(yaw), -math.sin(yaw)    # 뒤쪽
    sx, sy = -by, bx                           # 옆
    d = start
    while d <= length + 1e-9:
        for w in (-half_width, 0.0, half_width):
            if not known_free_at((x + d * bx + w * sx, y + d * by + w * sy, z)):
                return d
        d += step
    return None


def retreat_body(direction, yaw: float, speed: float, room: float, turn_room: float, lateral_max: float,
                 sideways: float) -> tuple[float, float]:
    """물러나거나 후진할 때의 기체 기준 속도 (앞, 왼쪽). 옆은 보는 센서가 없으므로 기체 뒤쪽으로 곧게 가고 옆은 lateral_max까지만
    보정한다. 들어온 자취(direction, 세계 단위 벡터)가 뒤쪽에서 sideways 넘게 벗어났으면(돌고 난 뒤) 돌 자리가 없을 때(room < turn_room)만
    그 자취 방향으로 간다 — 지나온 선이 유일하게 비어 있다고 아는 곳이다. direction이 없으면 곧게 뒤로."""
    if direction is None:
        return -speed, 0.0
    bx, by = world_to_body(direction[0], direction[1], yaw)
    off_tail = abs((math.atan2(by, bx) - math.pi + math.pi) % (2 * math.pi) - math.pi)
    if off_tail <= sideways or room >= turn_room:
        return -speed, max(-lateral_max, min(lateral_max, speed * by))
    return speed * bx, speed * by


def trail_cursor(trail, here, near: float):
    """되짚기 시작점: 자취(오래된 것 → 최근)에서 here에 near 안으로 가까운 가장 최근 점의 번호. 없으면 가장 가까운 점,
    그것도 1 m 넘게 멀면 None(여기 자취가 없다)."""
    best, best_d = None, math.inf
    for i in range(len(trail) - 1, -1, -1):
        d = math.hypot(trail[i][0] - here[0], trail[i][1] - here[1])
        if d <= near:
            return i
        if d < best_d:
            best, best_d = i, d
    return best if best_d <= 1.0 else None


def trail_fold(trail, cursor: int, max_turn: float) -> int:
    """trail[:cursor + 1]을 거꾸로 훑어 처음 접힌 점(이어진 두 구간이 max_turn 넘게 꺾임 — U턴)의 번호. 없으면 0.
    되짚기는 거기까지만 한다: 그 너머는 전에 들어갔다 나온 막다른 곳이라, 지나면 되짚는 방향이 뒤집혀 앞뒤로 흔들렸다."""
    for i in range(cursor - 1, 0, -1):
        ax, ay = trail[i][0] - trail[i - 1][0], trail[i][1] - trail[i - 1][1]
        bx, by = trail[i + 1][0] - trail[i][0], trail[i + 1][1] - trail[i][1]
        na, nb = math.hypot(ax, ay), math.hypot(bx, by)
        if na < 1e-6 or nb < 1e-6:
            continue
        if (ax * bx + ay * by) / (na * nb) < math.cos(max_turn):
            return i
    return 0


def trail_trim(trail, here, window: int, near: float) -> int:
    """물러나거나 되짚은 뒤: 최근 window점 중 here에 가장 가까운 점(near 안)까지만 남기도록 자를 길이(지울 뒤쪽 점 수). 없으면 0.

    가장 가까운 점은 **가장 최근 다리**에서만 찾는다: 끝에서부터 거꾸로 훑다가 거리가 가장 가까웠던 값보다 TRIM_RISE 넘게 다시
    멀어지면 멈춘다. 11/11 Gazebo 주머니(폭 1 m 막다른 통로): 서쪽으로 나오다 U턴해 0.7 m 되들어간 뒤 물러났을 때, 같은 통로에
    겹쳐 있던 **옛 다리**(서쪽으로 나오던 길)의 점이 0.04 m로 더 가까워 거기서 잘랐다 — U턴과 방금 들어온 다리가 통째로 지워져
    '들어온 자취로 후진'이 통로 안쪽(기수 오른쪽 앞)을 겨눴고, 센서 없는 옆으로 0.3 m 밀려 벽에 닿았다."""
    best, best_d = None, near
    n = len(trail)
    for i in range(n - 1, max(n - window, 0) - 1, -1):
        d = math.hypot(trail[i][0] - here[0], trail[i][1] - here[1])
        if d < best_d:
            best, best_d = i, d
        elif best is not None and d > best_d + TRIM_RISE:
            break
    return 0 if best is None else n - 1 - best


def off_tail(direction, yaw: float) -> float:
    """세계 좌표 방향(direction)이 기체 뒤쪽에서 벗어난 각 (rad): 0 = 똑바로 뒤, π/2 = 옆, π = 정면."""
    bx, by = world_to_body(direction[0], direction[1], yaw)
    return abs(math.atan2(by, bx) % (2 * math.pi) - math.pi)


def trail_target(trail, cursor: int, here, lookback: float):
    """자취 trail[:cursor + 1]을 거꾸로 되짚을 목표점. 지나친 점(뒤 점이 더 가까움)은 넘기고, here에서 lookback 넘게 떨어진
    첫 점을 겨눈다 — 옆으로 밀렸어도 선 위로 돌아온다. (목표점, 새 cursor, 자취 끝에 닿았는가)를 돌려준다."""
    def dist(i: int) -> float:
        return math.hypot(trail[i][0] - here[0], trail[i][1] - here[1])
    while cursor > 0 and dist(cursor - 1) <= dist(cursor):
        cursor -= 1
    j = cursor
    while j > 0 and dist(j) < lookback:
        j -= 1
    return trail[j], cursor, cursor == 0 and dist(0) < lookback / 2


def side_near(centers, ranges, cone: float, near: float, run: int) -> list[float]:
    """정면 ToF 원뿔(±cone) 밖 양옆에서 near 안으로 읽힌 열이 run개 넘게 이어진 쪽의 가운데 각(기수 기준 rad, 왼쪽 +).
    왼쪽·오른쪽 가운데 걸린 쪽만 돌려준다 (0–2개). centers·ranges는 shape.scene_ranges 결과(ranges는 배율을 곱하기 전 값).

    단안 깊이는 1.3 m 안의 무늬 없는 면을 거리와 상관없이 0.6–0.7 m로 읽는다 (11/13 17:44 실기체: 0.2 m 앞 흰 기둥 모서리도,
    1.2 m 앞 흰 벽도 0.6–0.7). 그래서 이 값만으로는 막지 않고, 그쪽으로 기수를 돌려 정면 ToF로 재 볼 곳을 고르는 데만 쓴다.
    정면 ToF 원뿔은 기체 바로 앞에서 좁아(0.2 m 앞 ±4 cm) 기체 폭(±10 cm)의 양 끝은 돌려 겨눠야 잰다."""
    out = []
    for sign in (1, -1):
        pick = [(float(c), float(r)) for c, r in zip(centers, ranges) if sign * c > cone]
        pick.sort(key=lambda cr: abs(cr[0]))
        best, cur = [], []
        for c, r in pick:
            cur = cur + [c] if r <= near else []
            if len(cur) > len(best):
                best = cur
        if len(best) >= run:
            out.append(sum(best) / len(best))
    return out
