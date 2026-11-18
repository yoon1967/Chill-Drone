"""AI 합성 영상 받기: 탐지 프로그램이 그려 보낸 최신 JPEG 1장만 들고 있다 (표준 라이브러리만 사용).

- 탐지 쪽(gateway_ai_client.py)이 게이트웨이 `/snapshot.jpg`를 받아 사람 윤곽선·장면 분할을 그린 뒤 PUT으로 보낸다.
  헤더 X-Input-Id·X-Frame-Id는 원본 프레임 번호다. 화면은 이것으로 원본보다 몇 장 늦었는지 본다.
- 저장하지 않는다(사진 기록은 detection_inbox가 한다). 쌓지 않고 최신 1장만 바꿔 끼운다.
- ALIVE_S 넘게 안 오면 꺼진 것으로 본다(탐지 프로그램 종료·멈춤). 그때 화면은 원본 영상으로 돌아간다.
"""

from __future__ import annotations

import threading
import time
from collections import deque

ALIVE_S = 2.0
MAX_IMAGE_BYTES = 3_000_000


class OverlayStore:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self._lock = threading.Lock()
        self._jpeg: bytes | None = None
        self._meta: dict | None = None
        self._recent: deque = deque(maxlen=64)   # 받은 시각(장/s 계산)
        self.received = 0
        self.rejected = 0

    def put(self, jpeg: bytes, input_id: str | None, frame_id: str | None) -> tuple[int, dict]:
        if len(jpeg) > MAX_IMAGE_BYTES:
            return self._reject(413, "too_large")
        if not jpeg.startswith(b"\xff\xd8"):
            return self._reject(400, "not_jpeg")
        now = self.clock()
        with self._lock:
            self._jpeg = jpeg
            self._meta = {"input_id": input_id or None, "frame_id": frame_id or None, "mono": now}
            self._recent.append(now)
            self.received += 1
        return 200, {"ok": True}

    def latest(self) -> bytes | None:
        """살아 있을 때만 최신 JPEG. 같은 장이면 같은 객체를 돌려준다(스트림이 중복 전송을 거른다)."""
        with self._lock:
            if self._meta is None or self.clock() - self._meta["mono"] > ALIVE_S:
                return None
            return self._jpeg

    def status(self) -> dict:
        now = self.clock()
        with self._lock:
            meta = self._meta
            recent = [t for t in self._recent if now - t <= ALIVE_S]
            alive = meta is not None and now - meta["mono"] <= ALIVE_S
            return {
                "alive": alive,
                "fps": round(len(recent) / ALIVE_S, 1) if alive else None,
                "age_s": None if meta is None else round(now - meta["mono"], 2),
                "frame_id": meta["frame_id"] if meta else None,
                "input_id": meta["input_id"] if meta else None,
                "received": self.received,
                "rejected": self.rejected,
            }

    def _reject(self, status: int, reason: str) -> tuple[int, dict]:
        with self._lock:
            self.rejected += 1
        return status, {"ok": False, "reason": reason}
