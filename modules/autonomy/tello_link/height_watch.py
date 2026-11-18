"""자율 비행 높이 지켜보기(알리기만): 드론 아래가 가까운가, 자율 비행 높이 추정이 잰 높이와 어긋나는가.

11/17 10:23 실기체 충돌(4drone parts/drafts/crash-1117.md): 비행 노드 높이 추정이 실제보다 0.4~0.9 m 높게 굳어
드론을 바닥 위 0.4 m까지 내렸고, 거기서 오르다 기체 왼쪽 뒤가 닿아 뒤집혔다. 그동안 관제 화면에는 아무 알림이 없었다.
게이트웨이(자율 비행 보기)는 비행에 끼어들지 않는다(병우님 코드·기준 그대로). 운용자에게 알리기만 한다.

- 아래가 가깝다(low): 자율 비행 중(이륙·착륙 단계 아님) 최근 2초 하방 ToF 중앙값이 0.5 m보다 짧으면.
  0.6 m 넘으면 풀린다. 하방 ToF가 0.1 m 아래라 Tello가 무효로 준 것도 가까운 것으로 본다.
- 높이 어긋남(gap): 최근 3초 동안 자율 비행 추정 높이(/tt/pose_est z)와 잰 높이 차이의 중앙값이 0.4 m를 넘으면.
  0.25 m 안으로 들어오면 풀린다. 잰 높이 = Tello 기압 − 바닥 기압.
  바닥 기압은 뜬 뒤 60초 동안, 추정과 하방 ToF가 맞을 때(바닥 위)의 (기압 − 하방 ToF) 중앙값이다
  (비행 노드 위치 추정이 바닥 기압을 잡는 방식과 같다). 기압은 ±0.1 m쯤 흔들려 한 값이 아니라 중앙값으로 본다.
  가구 위에서는 하방 ToF만 짧아진다. 그래서 "높게 봄"은 기압으로만 보고, "낮게 봄"은 기압·하방 둘 다 추정보다 높을 때만 본다.
- 11/16·11/17 실기체 세 비행으로 맞췄다: 정상 비행의 어긋남은 최대 0.19~0.33 m. 10:23 충돌 비행에서는 닿기 약 13초 전에 켜진다.
"""

from __future__ import annotations

import math
from collections import deque
from statistics import median

LOW_M = 0.5            # 최근 하방 ToF 중앙값이 이보다 짧으면 아래가 가깝다
LOW_CLEAR_M = 0.6      # 이보다 멀어지면 풀린다(오락가락 막기)
LOW_S = 2.0            # 이만큼의 중앙값을 본다
GAP_M = 0.4            # 추정 높이와 잰 높이 차이의 중앙값이 이보다 크면
GAP_CLEAR_M = 0.25     # 이 안으로 들어오면 풀린다
GAP_S = 3.0            # 이만큼의 중앙값을 본다
SPAN = 0.8             # 창의 이만큼(비율)은 값이 쌓여야 판단한다(막 뜬 뒤·값이 끊긴 뒤)
BARO_REF_S = 60.0      # 뜬 뒤 이 시간 동안 바닥 기압을 모은다
BARO_REF_MIN = 10      # 이만큼 모이기 전에는 어긋남을 보지 않는다
BARO_AGREE_M = 0.2     # 바닥 기압을 모을 때: 추정과 하방 ToF가 이 안이면 바닥 위로 본다(가구 위는 뺀다)
SHORT_M = 0.05         # 하방 ToF가 0.1 m 아래라 무효일 때 쓰는 값


def _num(v) -> float | None:
    return float(v) if isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) else None


def _window(q: deque, now: float, length: float) -> bool:
    """오래된 값을 버리고, 창이 판단할 만큼 찼는지."""
    while q and q[0][0] < now - length:
        q.popleft()
    return len(q) >= 2 and q[-1][0] - q[0][0] >= length * SPAN


class HeightWatch:
    """한 비행 동안의 높이 지켜보기. update()를 Tello 상태마다 부르고, 켜짐·꺼짐이 바뀌면 그 일을 돌려받는다."""

    def __init__(self):
        self.reset()

    def reset(self) -> None:
        self.air_since: float | None = None
        self._ref: list[float] = []
        self._lows: deque = deque()   # (때, 하방 ToF)
        self._gaps: deque = deque()   # (때, 추정 − 기압 높이, 낮게 봄 값 또는 None)
        self.low: dict | None = None  # 켜져 있으면 {tof_m}
        self.gap: dict | None = None  # 켜져 있으면 {gap_m, est_m, baro_m, tof_m}

    def baro_ref(self) -> float | None:
        return median(self._ref) if len(self._ref) >= BARO_REF_MIN else None

    def update(self, now: float, flying: bool, tof_m, tof_short: bool, baro, est_z) -> list[dict]:
        """flying: 자율 비행이 날고 있다(이륙·착륙 단계 아님). tof_m: 하방 ToF(m, 무효면 None).
        tof_short: 하방 ToF가 0.1 m 아래라 무효. baro: Tello 기압 높이(m). est_z: 자율 비행 추정 높이(새 값이 아니면 None)."""
        out: list[dict] = []
        if not flying:   # 땅·착륙 단계: 알림은 끝(end, 다시 정상이 된 것은 아니다)
            if self.low:
                out.append({"what": "low", "on": False, "end": True})
            if self.gap:
                out.append({"what": "gap", "on": False, "end": True})
            self.reset()   # 다음 비행은 바닥 기압부터 새로
            return out
        tof, baro, est = _num(tof_m), _num(baro), _num(est_z)
        if self.air_since is None:
            self.air_since = now
        if (tof is not None and baro is not None and est is not None and now - self.air_since <= BARO_REF_S
                and abs(est - tof) <= BARO_AGREE_M):
            self._ref.append(baro - tof)
        out += self._watch_low(now, SHORT_M if tof_short else tof)
        out += self._watch_gap(now, tof, baro, est)
        return out

    def _watch_low(self, now: float, tof: float | None) -> list[dict]:
        if tof is not None:
            self._lows.append((now, tof))
        if not _window(self._lows, now, LOW_S):
            return []
        m = round(median(v for _, v in self._lows), 2)
        if self.low is None and m < LOW_M:
            self.low = {"tof_m": m}
            return [{"what": "low", "on": True, "tof_m": m}]
        if self.low is not None:
            self.low["tof_m"] = m
            if m > LOW_CLEAR_M:
                self.low = None
                return [{"what": "low", "on": False, "tof_m": m}]
        return []

    def _watch_gap(self, now: float, tof: float | None, baro: float | None, est: float | None) -> list[dict]:
        ref = self.baro_ref()
        if ref is not None and baro is not None and est is not None:
            d = est - (baro - ref)
            low = max(d, est - tof) if tof is not None else None   # 낮게 봄: 기압·하방 둘 다 추정보다 높아야
            self._gaps.append((now, d, low, est, baro - ref, tof))
        if not _window(self._gaps, now, GAP_S):
            return []   # 모르면 그대로 둔다(켜진 것도 끄지 않는다)
        d = median(g[1] for g in self._gaps)
        lows = [g[2] for g in self._gaps if g[2] is not None]
        too_low = len(lows) * 2 >= len(self._gaps) and median(lows) < -GAP_M
        _, _, _, est, h, tof = self._gaps[-1]
        values = {"gap_m": round(d, 2), "est_m": round(est, 2), "baro_m": round(h, 2),
                  "tof_m": None if tof is None else round(tof, 2)}
        if self.gap is None and (d > GAP_M or too_low):
            self.gap = dict(values)
            return [{"what": "gap", "on": True, **values}]
        if self.gap is not None:
            self.gap.update(values)
            if abs(d) < GAP_CLEAR_M:
                self.gap = None
                return [{"what": "gap", "on": False, **values}]
        return []

    def state(self) -> dict:
        ref = self.baro_ref()
        return {"low": None if self.low is None else dict(self.low), "gap": None if self.gap is None else dict(self.gap),
                "baro_ref_m": None if ref is None else round(ref, 2)}
