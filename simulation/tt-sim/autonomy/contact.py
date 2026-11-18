"""닿음 감지와 떼어 낼 방향 (11/13 13:51 실기체).

그 비행은 세 번 부딪혔는데 비행 노드는 한 번(물러나다 기수가 저절로 돎)만 알았다. 마지막은 문 앞에서 0.8 rad/s로 돌다
왼쪽 흰 벽에 부딪혔고(91.6초, 수평 가속 0.65 g), 그 뒤 12초 동안 기체 왼쪽이 벽에 닿아 있었다. 우리는 옆·앞뒤 rc 0만
보냈는데 Tello가 스스로 제자리를 지키려 계속 왼쪽(벽 쪽)으로 밀어 롤이 −3°에서 −60°까지 커졌고, 왼쪽으로 돌라는
명령(rc 요 −34~−80)을 6초 보내도 기수가 1°도 안 돌았다. 운용자가 착륙시켰다. 그래서 닿으면 입력 0만 보내지 않고
닿은 반대쪽으로 짧게 밀어 떼어 낸다 (Tello가 지키는 자리를 벽 밖으로 옮긴다).

Tello 상태의 agx·agy는 기체 기준 비중력 가속이다 (0.001 g, agx 앞 +, agy 오른쪽 +). 자유롭게 날 때는 추력이 기체 z축이라
작고(정상 비행 10판 11/05–11/12에서 가장 큰 값 93 mg), 무엇에 닿으면 그 반력이 그대로 나온다(부딪힘 0.65–3.1 g,
벽에 기대어 밀 때 0.1–0.4 g). 반력은 닿은 물건에서 멀어지는 쪽이다. 기울기는 미는 쪽으로 기운다:
pitch는 앞으로 갈 때 −, roll은 오른쪽으로 갈 때 + (실기체 rc와 맞춰 봄, 11/13 벽에 붙은 동안 롤 −·agy +·벽은 왼쪽).

가라는데 안 감 (PushWatch, 11/14): Gazebo의 안 보이는 벽 판에서 문을 지나던 드론이 벽에 10.8초 붙어 있었는데 못 잡았다.
앞으로 가기와 제자리 돌기를 번갈아 해 밀리는 힘이 띄엄띄엄 나서 '0.6초 이어서 밀림'에 안 걸렸고, 기수가 95–109°로 흔들려
'제자리에 걸림'(6°만 돌아도 움직인 것으로 셈)도 늦었다. 그래서 보낸 수평 속도 명령을 지도 방향으로 모은 거리와 그 사이 추정
위치가 그쪽으로 간 거리를 비교한다. 실기체 기록 56분(10/30–11/13, 떠 있던 시간)에 대 보니 Tello는 돌거나 멈춘 직후 앞으로
가라고 해도 1.5–2.5초 동안 안 움직이는 일이 흔하다(11/03 16:12, 11/05 13:04). 그래서 반력 없이는 4초 동안 0.5 m를 가라 했는데
10%도 못 갔을 때만, 2–3초로 짧게 볼 때는 명령 반대쪽 반력 평균이 60 mg(정상 비행 90%가 24 mg 안)을 넘을 때만 닿음으로 본다.
제동(지금 움직이는 쪽의 반대로 명령)은 갈 것이 아니므로 판정을 비운다. 이 값으로 그 기록에서 걸린 곳은 11/02 17:33에 기체가
뒤집혀 상태값이 멈춘 한 곳뿐이었다.
"""
from __future__ import annotations

import math
from collections import deque

IMPACT_MG = 300.0      # 수평 가속이 한 번이라도 이만큼(0.001 g) 넘으면 부딪힘 (정상 비행 최대 93 mg)
PRESS_MG = 120.0       # 이만큼 넘는 수평 가속이
PRESS_TIME = 0.6       #   이만큼(s) 이어지면 무엇에 기대어 밀고 있음
TILT_MAX = 15.0        # 기울기(°)가 이만큼 넘으면 닿음. 정상 비행은 rc 80에서도 3° 안팎
TURN_RATE = 0.25       # 이만큼(rad/s, rc 약 29) 넘게 같은 쪽으로 돌라는 명령을
TURN_TIME = 1.2        #   이만큼(s) 이어서 냈는데
TURN_MIN = math.radians(4)   # 기수가 이만큼도 안 돌았으면 걸린 것 (정상 비행 10판 0번, 11/13은 92.7초부터 5.9초 동안 2°)
DIR_MG = 80.0          # 떼어 낼 방향: 수평 가속이 이만큼 넘으면 그 방향(반력)으로,
DIR_TILT = 3.0         #   아니면 기울기(°)가 이만큼 넘으면 기운 쪽의 반대로. 둘 다 작으면 모른다
# 가라는데 안 감 (PushWatch). 규칙마다 (창 s, 그동안 가라 한 거리 m, 간 비율 미만, 명령 반대쪽 반력 평균 mg 이상 — None이면 안 봄).
# 실기체 기록 56분에서: 반력 없이 4초 0.4 m로 낮추면 정상 비행 3곳이 걸렸고, 반력 60 mg을 붙이면 2초 0.2 m·3초 0.15 m(앞으로
# 가기와 돌기를 반씩 번갈아 0.15 m/s로 밀 때)에서도 0곳 (45 mg로 낮춰도 0곳). 빠른 것부터 본다
STALL_RULES = ((2.0, 0.2, 0.25, 60.0), (3.0, 0.15, 0.25, 60.0), (4.0, 0.5, 0.10, None))
STALL_LAG = 0.4        # 명령은 이만큼(s) 앞당겨 모은다 (Tello가 따라오는 시간)
STALL_ALIGN = 0.8      # 모은 명령 크기 / 명령 길이 합이 이 이상: 한쪽으로 가라고 한 것만 본다 (지그재그·돌면서 가기는 안 봄)
STALL_BRAKE = 0.08     # 최근 0.3초에 이보다 빨리(m/s) 움직이는데 명령이 그 반대쪽이면 제동: 판정을 비운다
# 닿은 쪽을 모를 때(돌라는데 안 돎인데 가속·기울기가 작음) 쓸 쪽 (PushWatch.hint, 11/14 Gazebo 반력 0 벽 판): 벽에 닿은 채 돌면서
# 0.12 m/s로 나아가려다 '돌라는데 안 돎'이 쪽을 모른 채 1.2초마다 걸려 머물기만 하다 그 자리에 내렸다. 바로 앞 HINT_WINDOW초에
# HINT_CMD m 넘게 한쪽으로 가라 했는데 HINT_FRAC도 못 갔으면 그쪽에 닿은 것으로 본다
HINT_WINDOW = 2.0
HINT_CMD = 0.08
HINT_FRAC = 0.25

SIDES = ("앞", "왼쪽 앞", "왼쪽", "왼쪽 뒤", "뒤", "오른쪽 뒤", "오른쪽", "오른쪽 앞")


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def side_name(angle: float) -> str:
    """기체 기준 방향(rad, 0 앞, + 왼쪽)의 이름 (45°씩 여덟 쪽)."""
    return SIDES[int(round(wrap(angle) / (math.pi / 4))) % 8]


class ContactWatch:
    """Tello 상태(가속·기울기)와 보낸 회전 명령·기수로 닿음을 본다. 닿았으면 이유 문장을 돌려준다."""

    def __init__(self):
        self.last = None    # 마지막 상태 (agx, agy, pitch, roll)
        self.reset()

    def reset(self) -> None:
        """떼어 내기·머물기를 시작할 때 이어지던 판단을 버린다."""
        self.press_since = None
        self.turn = None    # (시작 시각, 그때 기수, 도는 쪽)

    def state(self, t: float, agx: float, agy: float, pitch: float, roll: float) -> str | None:
        """Tello 상태 하나 (초당 약 10번). 부딪힘·기대어 밂·크게 기욺이면 이유."""
        self.last = (float(agx), float(agy), float(pitch), float(roll))
        a = math.hypot(agx, agy)
        tilt = max(abs(pitch), abs(roll))
        if a >= IMPACT_MG:
            return f"부딪힘 (수평 가속 {a / 1000:.2f} g)"
        if tilt >= TILT_MAX:
            return f"기울기 {tilt:.0f}°"
        if a < PRESS_MG:
            self.press_since = None
            return None
        if self.press_since is None:
            self.press_since = t
        elif t - self.press_since >= PRESS_TIME:
            return f"무엇에 기대어 밀림 (수평 가속 {a / 1000:.2f} g가 {t - self.press_since:.1f}초)"
        return None

    def turning(self, t: float, rate: float, yaw: float) -> str | None:
        """보낸 회전 명령(rad/s, + 반시계)과 지금 기수(rad). 돌라는데 TURN_TIME 넘게 TURN_MIN도 못 돌았으면 이유."""
        if abs(rate) < TURN_RATE:
            self.turn = None
            return None
        sign = 1 if rate > 0 else -1
        if self.turn is None or self.turn[2] != sign or abs(wrap(yaw - self.turn[1])) >= TURN_MIN:
            self.turn = (t, yaw, sign)   # 새로 돌기 시작했거나 잘 돌고 있다: 여기서부터 다시 잰다
            return None
        if t - self.turn[0] >= TURN_TIME:
            return (f"돌라는데 {t - self.turn[0]:.1f}초째 기수가 "
                    f"{math.degrees(abs(wrap(yaw - self.turn[1]))):.0f}°밖에 안 돎")
        return None

    def away(self) -> float | None:
        """떼어 낼 방향 (기체 기준 rad, 0 앞, + 왼쪽). 모르면 None."""
        if self.last is None:
            return None
        agx, agy, pitch, roll = self.last
        if math.hypot(agx, agy) >= DIR_MG:
            return math.atan2(-agy, agx)     # 반력 쪽: 앞 agx, 왼쪽 −agy
        if max(abs(pitch), abs(roll)) >= DIR_TILT:
            return math.atan2(roll, pitch)   # 미는 쪽(앞 −pitch, 왼쪽 −roll)의 반대
        return None


def react_world(last, yaw: float) -> tuple[float, float]:
    """ContactWatch.last의 가속을 지도 방향 반력 벡터(mg)로: 기체 기준 (앞 agx, 왼쪽 −agy)를 기수 yaw만큼 돌린다. 없으면 0."""
    if last is None:
        return 0.0, 0.0
    f, left = last[0], -last[1]
    c, s = math.cos(yaw), math.sin(yaw)
    return c * f - s * left, s * f + c * left


class PushWatch:
    """가라는데 안 감: 비행 노드가 틱마다 (시각, 지난 틱에 보낸 수평 속도 명령의 지도 방향 벡터, 추정 위치, 지도 방향 반력)을
    넣는다. 위치와 명령은 같은 좌표(보정 전 위치 추정)로 넣는다."""

    def __init__(self, rules=STALL_RULES):
        self.rules = rules
        self.keep = max(r[0] for r in rules) + STALL_LAG + 0.5
        self.hist: deque = deque()   # (t, 명령 x, 명령 y, 위치 x, 위치 y, 반력 x, 반력 y)

    def reset(self) -> None:
        self.hist.clear()

    def add(self, t: float, cmd, pos, react=(0.0, 0.0)) -> None:
        h = self.hist
        if h and t - h[-1][0] > 0.5:
            h.clear()   # 한동안 안 넣었다 (다른 단계였다): 이어 보지 않는다
        old = next((e for e in h if e[0] >= t - 0.3), None)
        if old is not None and t - old[0] > 0.15:
            vx, vy = (pos[0] - old[3]) / (t - old[0]), (pos[1] - old[4]) / (t - old[0])
            if math.hypot(vx, vy) > STALL_BRAKE and math.hypot(cmd[0], cmd[1]) > 0.02 and vx * cmd[0] + vy * cmd[1] < 0:
                h.clear()   # 제동 중: 여기서부터 다시 잰다
        h.append((t, float(cmd[0]), float(cmd[1]), float(pos[0]), float(pos[1]), float(react[0]), float(react[1])))
        while h and h[0][0] < t - self.keep:
            h.popleft()

    @staticmethod
    def _span(h: list, t: float, window: float):
        """창 window초: (모은 명령 크기, 그 방향 단위 벡터 x, y, 그쪽으로 간 거리, 위치 창 시작 번호). 한쪽으로 모이지 않았으면 None."""
        cx = cy = length = 0.0
        for k in range(len(h) - 1):
            tk = h[k][0]
            if tk < t - window - STALL_LAG:
                continue
            if tk >= t - STALL_LAG:
                break
            d = min(h[k + 1][0] - tk, 0.2)
            cx += h[k][1] * d
            cy += h[k][2] * d
            length += math.hypot(h[k][1], h[k][2]) * d
        c = math.hypot(cx, cy)
        if c < 1e-6 or c < STALL_ALIGN * length:
            return None
        ux, uy = cx / c, cy / c
        j = next(k for k, e in enumerate(h) if e[0] >= t - window)
        return c, ux, uy, (h[-1][3] - h[j][3]) * ux + (h[-1][4] - h[j][4]) * uy, j

    def hint(self, t: float, window: float = HINT_WINDOW, need: float = HINT_CMD, frac: float = HINT_FRAC):
        """닿은 쪽을 모를 때 쓸 쪽: 바로 앞 window초에 need m 넘게 한쪽으로 가라 했는데 frac도 못 갔으면 그 지도 방향, 아니면 None.
        창이 다 차지 않아도(머문 뒤 다시 가려던 참) 있는 만큼 본다."""
        h = list(self.hist)
        if len(h) < 2:
            return None
        s = self._span(h, t, window)
        if s is None or s[0] < need or s[3] >= frac * s[0]:
            return None
        return math.atan2(s[2], s[1])

    def check(self, t: float):
        """걸렸으면 (이유 문장, 가라 한 지도 방향 rad), 아니면 None."""
        h = list(self.hist)
        if len(h) < 2:
            return None
        for window, need, frac, mg in self.rules:
            if h[0][0] > t - window - STALL_LAG:
                continue   # 창이 아직 다 안 찼다
            s = self._span(h, t, window)
            if s is None or s[0] < need:
                continue
            c, ux, uy, went, j = s
            if went >= frac * c:
                continue
            what = f"가라는데 안 감 ({window:.0f}초 동안 {c:.2f} m 가라 했는데 {max(went, 0.0):.2f} m"
            if mg is not None:
                back = -sum(e[5] * ux + e[6] * uy for e in h[j:]) / len(h[j:])
                if back < mg:
                    continue
                what += f", 반대쪽으로 밀림 {back / 1000:.2f} g"
            return what + ")", math.atan2(uy, ux)
        return None
