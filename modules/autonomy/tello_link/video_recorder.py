"""드론 영상 원본 녹화: Tello가 보낸 H.264 UDP 바이트를 받은 그대로 회차 폴더에 남긴다 (표준 라이브러리만).

기록 재생(`--replay`)과 화면 시험용 목업 데이터를 만들려고 둔다. 영상 수신 스레드에서는 디스크에 쓰지 않는다.
`TelloVideo.packet_sink`로 받은 바이트를 큐에 넣고, 작업 스레드 하나가 파일에 쓴다. 큐가 넘치면(디스크가 느릴 때)
그 조각은 거기까지만 남기고 `partial`로 적는다(영상 수신·비행은 영향 없음).

회차 폴더 video/:
- part_NNNN.h264   영상을 켤 때마다 한 조각. 받은 바이트를 이어 붙인 H.264 기본 스트림(ffplay·PyAV로 바로 열림)
- part_NNNN.jsonl  받은 UDP 패킷마다 한 줄 {offset, size, timestamp_s(게이트웨이 monotonic), wall(UTC epoch 초)}
                   지도 파트 녹화(modules/mapping/data/live_recording.py)와 같은 칸에 wall만 더했다
- parts.jsonl      조각마다 시작·끝 한 줄(영상 세션 input_id, 켠 이유, 바이트·패킷 수, 상태 captured/partial)
"""

from __future__ import annotations

import json
import os
import threading
import time
from collections import deque

QUEUE_BYTES = 8 << 20      # 쓰기 대기 상한 8 MB(약 40초 분량). 넘치면 그 조각은 거기까지
MAX_PARTS = 256


def _iso(t: float) -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%S", time.gmtime(t)) + f".{int(t % 1 * 1000):03d}+00:00"


class VideoRecorder:
    def __init__(self, run_dir: str, *, queue_bytes: int = QUEUE_BYTES):
        self.dir = os.path.join(run_dir, "video")
        os.makedirs(self.dir, exist_ok=True)
        self.limit = queue_bytes
        self._cond = threading.Condition()
        self._jobs: deque = deque()
        self._queued = 0
        self._parts = 0
        self._current: dict | None = None     # 지금 받는 조각(수신 스레드가 보는 것)
        self.error: str | None = None
        self.bytes_written = 0
        self._closed = False
        self._worker = threading.Thread(target=self._run, name="video-recorder", daemon=True)
        self._worker.start()

    # ---------- 게이트웨이가 부른다 ----------
    def start_part(self, input_id: str | None, reason: str | None):
        """새 조각을 열고 TelloVideo.packet_sink에 넣을 함수를 돌려준다. 열 수 없으면 None."""
        with self._cond:
            if self._closed or self._parts >= MAX_PARTS:
                return None
            self._end_locked()
            part = {"id": self._parts, "input_id": input_id, "reason": reason, "started_at": _iso(time.time()),
                    "bytes": 0, "packets": 0, "status": "recording"}
            self._parts += 1
            self._current = part
            self._jobs.append(("open", part))
            self._cond.notify()
        return lambda data, mono, part=part: self._offer(part, data, mono)

    def end_part(self) -> None:
        with self._cond:
            self._end_locked()

    def close(self, timeout: float = 5.0) -> None:
        with self._cond:
            self._end_locked()
            self._closed = True
            self._jobs.append(("stop",))
            self._cond.notify()
        self._worker.join(timeout=timeout)

    def stats(self) -> dict:
        with self._cond:
            cur = self._current
            return {"recording": cur is not None, "part": cur["id"] if cur else None, "parts": self._parts,
                    "queued_bytes": self._queued, "bytes_written": self.bytes_written, "error": self.error}

    # ---------- 수신 스레드 ----------
    def _offer(self, part: dict, data: bytes, mono: float) -> bool:
        wall = time.time()
        with self._cond:
            if part is not self._current or part["status"] != "recording":
                return False
            if self._queued + len(data) > self.limit:
                part["status"] = "partial"      # 디스크가 못 따라온다. 앞부분은 남긴다
                part["reason_partial"] = "queue_overflow"
                self.error = "queue_overflow"
                self._cond.notify()
                return False
            self._queued += len(data)
            self._jobs.append(("data", part, bytes(data), mono, wall))
            self._cond.notify()
            return True

    def _end_locked(self) -> None:
        if self._current is not None:
            self._jobs.append(("close", self._current))
            self._current = None
            self._cond.notify()

    # ---------- 작업 스레드 ----------
    def _run(self) -> None:
        files: dict[int, tuple] = {}
        while True:
            with self._cond:
                self._cond.wait_for(lambda: bool(self._jobs), timeout=0.5)
                job = self._jobs.popleft() if self._jobs else None
                if job and job[0] == "data":
                    self._queued -= len(job[2])
            if job is None:
                continue
            kind = job[0]
            try:
                if kind == "open":
                    part = job[1]
                    name = os.path.join(self.dir, f"part_{part['id']:04d}")
                    files[part["id"]] = (open(name + ".h264", "xb"), open(name + ".jsonl", "x", encoding="utf-8"))
                    self._line({"event": "start", **{k: part[k] for k in ("id", "input_id", "reason", "started_at")}})
                elif kind == "data":
                    _, part, data, mono, wall = job
                    f = files.get(part["id"])
                    if f is None:
                        continue
                    raw, index = f
                    index.write(json.dumps({"offset": part["bytes"], "size": len(data), "timestamp_s": round(mono, 6),
                                            "wall": round(wall, 6)}) + "\n")
                    raw.write(data)
                    part["bytes"] += len(data)
                    part["packets"] += 1
                    self.bytes_written += len(data)
                elif kind == "close":
                    part = job[1]
                    for stream in files.pop(part["id"], ()):
                        stream.close()
                    if part["status"] == "recording":
                        part["status"] = "captured"
                    self._line({"event": "end", "id": part["id"], "ended_at": _iso(time.time()), "bytes": part["bytes"],
                                "packets": part["packets"], "status": part["status"], "reason_partial": part.get("reason_partial")})
                elif kind == "stop":
                    for streams in files.values():
                        for stream in streams:
                            stream.close()
                    return
            except OSError as e:   # 디스크 오류: 녹화만 멈춘다(영상·비행은 계속)
                with self._cond:
                    self.error = f"{type(e).__name__}: {e}"[:200]
                    if self._current is not None:
                        self._current["status"] = "partial"
                        self._current["reason_partial"] = "write_error"

    def _line(self, row: dict) -> None:
        with open(os.path.join(self.dir, "parts.jsonl"), "a", encoding="utf-8") as f:
            f.write(json.dumps(row, ensure_ascii=False) + "\n")
