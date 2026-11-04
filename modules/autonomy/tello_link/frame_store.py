"""게이트웨이가 내준 영상 프레임에 번호를 붙이고 최근 것만 보관한다 (표준 라이브러리만 사용).

탐지처럼 `/snapshot.jpg`로 프레임을 가져가 처리하는 쪽은 결과에 `input_id`·`frame_id`만 담아 보내면 된다.
게이트웨이는 그 번호로 같은 사진과 수신 시각을 다시 찾는다(사진을 다시 올리지 않아도 된다).
- `input_id`: 영상 세션. 영상을 켤 때마다 새 값(`<회차 id>-video<n>`)
- `frame_id`: 세션 안에서 유일한 번호(`frame-000001`). 같은 프레임을 다시 가져가면 같은 번호
- 시각은 게이트웨이가 디코딩한 시각(`time_source=gateway_receive`). 카메라 촬영 시각이 아니다.
"""

from __future__ import annotations

import threading
import time
from collections import OrderedDict

MAX_FRAMES = 64          # 탐지가 초당 10장 가져가도 약 6초 분량
MAX_AGE_S = 10.0


class FrameStore:
    def __init__(self, run_id: str, *, max_frames: int = MAX_FRAMES, max_age_s: float = MAX_AGE_S,
                 clock=time.monotonic):
        self.run_id = run_id
        self.max_frames = max_frames
        self.max_age_s = max_age_s
        self.clock = clock
        self._lock = threading.Lock()
        self._session = 0
        self._count = 0
        self._frames: OrderedDict[str, dict] = OrderedDict()   # frame_id → 프레임
        self._last_stamp: float | None = None

    @property
    def input_id(self) -> str | None:
        return f"{self.run_id}-video{self._session}" if self._session else None

    def new_session(self) -> str:
        """영상을 켤 때 부른다. 이전 세션 프레임은 비운다."""
        with self._lock:
            self._session += 1
            self._count = 0
            self._frames.clear()
            self._last_stamp = None
            return self.input_id

    def add(self, jpeg: bytes, stamp: float, wall: float, size: tuple[int, int] | None = None,
            drone: dict | None = None) -> dict | None:
        """영상 수신기의 (jpeg, 디코딩 monotonic 시각, wall 시각). 같은 프레임이면 이미 붙인 번호를 돌려준다.

        drone: 내줄 때의 드론 상태 요약(탐지 결과에 그때 드론 상태를 붙이는 데 쓴다).
        """
        with self._lock:
            if not self._session:
                return None
            if stamp == self._last_stamp and self._frames:
                return next(reversed(self._frames.values()))
            self._count += 1
            frame = {
                "input_id": self.input_id,
                "frame_id": f"frame-{self._count:06d}",
                "jpeg": jpeg,
                "mono": stamp,
                "wall": wall,
                "width": size[0] if size else None,
                "height": size[1] if size else None,
                "drone": drone,
            }
            self._last_stamp = stamp
            self._frames[frame["frame_id"]] = frame
            self._prune()
            return frame

    def get(self, input_id: str, frame_id: str) -> dict | None:
        """보관 중인 프레임. 세션이 다르거나 오래돼 지웠으면 None."""
        with self._lock:
            if input_id != self.input_id:
                return None
            self._prune()
            return self._frames.get(frame_id)

    def _prune(self) -> None:
        now = self.clock()
        while self._frames:
            oldest = next(iter(self._frames.values()))
            if len(self._frames) > self.max_frames or now - oldest["mono"] > self.max_age_s:
                self._frames.popitem(last=False)
            else:
                break
