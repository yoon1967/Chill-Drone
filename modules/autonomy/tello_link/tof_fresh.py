"""전방 ToF 새 값 판정 (표준 라이브러리만 사용).

계약: contracts/api/AUTONOMY_BRIDGE_V0_1.md 3장 "전방 ToF: 새 값일 때만 보낸다".
센서는 약 5Hz로 바뀌는데 게이트웨이는 더 자주 묻는다. 같은 값을 두 번 보내면 자율의 "두 번 연속 가까움" 필터를
튄 값이 통과하므로, 값이 바뀌었거나 같은 값이 hold_s 이상 이어질 때만 새 값으로 본다.
멈춰 있는 장애물(값 그대로)도 약 1/hold_s Hz로 계속 전달된다.
"""

from __future__ import annotations

TOF_HOLD_S = 0.2  # 실측 센서 갱신 간격 약 195ms
JITTER_S = 0.02   # 조회 간격 흔들림 허용. 0.2초 간격이 0.199초로 잡혀 한 번 건너뛰지 않게


class FreshFilter:
    def __init__(self, hold_s: float = TOF_HOLD_S, jitter_s: float = JITTER_S):
        self.hold_s = hold_s
        self.jitter_s = jitter_s
        self.seq = 0
        self._last_key = None
        self._last_t: float | None = None

    def update(self, reading: dict, now: float) -> int | None:
        """`parse_ext_tof()` 결과를 넣는다. 새 값이면 올린 seq, 아니면 None."""
        key = (bool(reading.get("valid")), reading.get("front_tof_m"), reading.get("reason"))
        if key == self._last_key and self._last_t is not None and now - self._last_t < self.hold_s - self.jitter_s:
            return None
        self._last_key, self._last_t = key, now
        self.seq += 1
        return self.seq
