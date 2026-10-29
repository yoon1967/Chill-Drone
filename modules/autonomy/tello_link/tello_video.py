"""Tello 영상(UDP 11111, H.264) 수신. 최신 프레임 1장만 유지한다.

PyAV(av)로 원시 UDP 바이트를 직접 파싱·디코딩한다. JPEG 변환에는 OpenCV를 쓴다.
`streamon` 명령은 TelloLink로 따로 보낸다. 이 클래스는 받기만 한다.
"""

from __future__ import annotations

import socket
import threading
import time
from collections import deque


class TelloVideo:
    def __init__(self, port: int = 11111, bind_ip: str = "0.0.0.0"):
        self.port = port
        self.bind_ip = bind_ip
        self._sock: socket.socket | None = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._jpeg_lock = threading.Lock()
        self._frame = None
        self._frame_mono: float | None = None
        self._frame_wall: float | None = None
        self._frame_times: deque = deque(maxlen=60)
        self._jpeg_cache: tuple[float, int, bytes] | None = None
        self.started_mono: float | None = None
        self.packets = 0
        self.bytes = 0
        self.frames = 0
        self.decode_errors = 0
        self.last_packet_mono: float | None = None
        self.size: tuple[int, int] | None = None
        # Optional nonblocking observer for exact received H.264 bytes. It must
        # enqueue work; disk I/O belongs to the caller's recording worker.
        self.packet_sink = None
        self.packet_sink_errors = 0

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self) -> None:
        if self.running:
            return
        import av  # 지연 import: 영상이 필요 없는 환경에서도 tello_link는 동작하게 한다

        self._codec = av.CodecContext.create("h264", "r")
        # FFmpeg 자동 스레드를 끈다. 실기체 스트림에서 프로세스 스레드가 수천 개로 불어난 적이 있다.
        # 960×720 디코딩은 단일 스레드로 충분하다.
        self._codec.thread_count = 1
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        self._sock.bind((self.bind_ip, self.port))
        self._sock.settimeout(0.5)
        self._stop.clear()
        self.started_mono = time.monotonic()
        self._thread = threading.Thread(target=self._loop, name="tello-video", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2)
        if self._sock:
            self._sock.close()
        self._thread = None
        self._sock = None

    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                data, _ = self._sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            self.packets += 1
            self.bytes += len(data)
            self.last_packet_mono = time.monotonic()
            if self.packet_sink is not None:
                try:
                    self.packet_sink(data, self.last_packet_mono)
                except Exception:
                    self.packet_sink_errors += 1
            try:
                for packet in self._codec.parse(data):
                    for frame in self._codec.decode(packet):
                        now = time.monotonic()
                        with self._lock:
                            self._frame = frame
                            self._frame_mono = now
                            self._frame_wall = time.time()
                            self._frame_times.append(now)
                            self.frames += 1
                            self.size = (frame.width, frame.height)
            except Exception:  # 스트림 시작 직후 키프레임 전 패킷 등은 디코딩 실패가 정상이다
                self.decode_errors += 1

    def jpeg(self, quality: int = 75) -> bytes | None:
        snapshot = self.jpeg_snapshot(quality)
        return snapshot[0] if snapshot else None

    def jpeg_snapshot(self, quality: int = 75) -> tuple[bytes, float, float] | None:
        """Return JPEG plus stable receive timestamps for de-duplication and alignment."""
        # PyAV는 같은 프레임을 여러 스레드에서 동시에 변환하면 프로세스가 죽는다(재현 확인).
        # 변환은 한 번에 하나만 하고, 같은 프레임이면 캐시를 돌려준다.
        with self._jpeg_lock:
            with self._lock:
                frame, stamp, wall = self._frame, self._frame_mono, self._frame_wall
                cache = self._jpeg_cache
            if frame is None:
                return None
            if cache and cache[:2] == (stamp, quality):
                return cache[2], stamp, wall
            import cv2
            ok, encoded = cv2.imencode('.jpg', frame.to_ndarray(format='bgr24'),
                                       [cv2.IMWRITE_JPEG_QUALITY, quality])
            if not ok:
                return None
            data = encoded.tobytes()
            with self._lock:
                self._jpeg_cache = (stamp, quality, data)
            return data, stamp, wall

    def stats(self) -> dict:
        now = time.monotonic()
        with self._lock:
            times = list(self._frame_times)
            frame_mono = self._frame_mono
        fps = None
        if len(times) >= 2 and now - times[-1] < 2.0 and times[-1] > times[0]:
            fps = round((len(times) - 1) / (times[-1] - times[0]), 1)
        elif times:
            fps = 0.0
        return {
            "running": self.running,
            "port": self.port,
            "packets": self.packets,
            "bytes": self.bytes,
            "frames": self.frames,
            "decode_errors": self.decode_errors,
            "packet_sink_errors": self.packet_sink_errors,
            "fps": fps,
            "frame_age_s": None if frame_mono is None else round(now - frame_mono, 3),
            "packet_age_s": None if self.last_packet_mono is None else round(now - self.last_packet_mono, 3),
            "since_start_s": None if self.started_mono is None else round(now - self.started_mono, 1),
            "width": self.size[0] if self.size else None,
            "height": self.size[1] if self.size else None,
            "time_source": "server_receive",
        }
