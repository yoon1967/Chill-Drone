"""Tello 영상(UDP 11111, H.264) 수신. 최신 프레임 1장만 유지한다.

PyAV(av)로 원시 UDP 바이트를 직접 파싱·디코딩한다. JPEG 변환에는 OpenCV를 쓴다(없으면 Pillow).
`streamon` 명령은 TelloLink로 따로 보낸다. 이 클래스는 받기만 한다.

record_dir를 주면 받은 H.264 바이트를 그대로 video.h264에, 풀린 프레임마다 받은 시각을 video_frames.csv에 남긴다.
다시 인코딩하지 않아 CPU를 거의 쓰지 않는다. video.h264는 VLC로 바로 볼 수 있고, 프레임 시각으로 비행 기록과 맞춘다
(simulation/tt-sim/tools/flight_viewer.py가 mp4로 옮긴다).
packet_sink를 주면(콘솔 앱) 받은 바이트를 그 함수에도 넘긴다. 막히면 안 되므로 큐에 넣기만 해야 한다.
"""

from __future__ import annotations

import json
import socket
import threading
import time
from collections import deque
from pathlib import Path


def mjpeg_part(jpeg, pose=None, metadata=None):
    headers = b"--frame\r\nContent-Type: image/jpeg\r\n"
    if pose is not None:
        headers += b"X-Tello-Pose: " + json.dumps(pose, separators=(',', ':'), allow_nan=False).encode('ascii') + b"\r\n"
    if metadata is not None:
        headers += b"X-Tello-Frame: " + json.dumps(metadata, separators=(',', ':'), allow_nan=False).encode('ascii') + b"\r\n"
    return headers + b"\r\n" + jpeg + b"\r\n"


class TelloVideo:
    def __init__(self, port: int = 11111, bind_ip: str = "0.0.0.0", record_dir: str | Path | None = None):
        self.port = port
        self.bind_ip = bind_ip
        self.record_dir = Path(record_dir) if record_dir else None
        self._raw = None       # video.h264 (받은 바이트 그대로)
        self._times = None     # video_frames.csv (프레임 번호, 받은 시각)
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
        if self.record_dir:
            self.record_dir.mkdir(parents=True, exist_ok=True)
            self._raw = (self.record_dir / "video.h264").open("ab")
            self._times = (self.record_dir / "video_frames.csv").open("a", encoding="utf-8", buffering=1)
            self._times.write("frame,wall_time,mono_time,bytes_so_far\n")
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
        for f in (self._raw, self._times):
            if f:
                f.close()
        self._raw = self._times = None
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
            if self._raw:
                self._raw.write(data)
            if self.packet_sink is not None:
                try:
                    self.packet_sink(data, self.last_packet_mono)
                except Exception:
                    self.packet_sink_errors += 1
            try:
                for packet in self._codec.parse(data):
                    for frame in self._codec.decode(packet):
                        now = time.monotonic()
                        wall = time.time()
                        with self._lock:
                            self._frame = frame
                            self._frame_mono = now
                            self._frame_wall = wall
                            self._frame_times.append(now)
                            self.frames += 1
                            self.size = (frame.width, frame.height)
                        if self._times:
                            self._times.write(f"{self.frames},{wall:.4f},{now:.4f},{self.bytes}\n")
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
            data = _encode_jpeg(frame, quality)
            if data is None:
                return None
            with self._lock:
                self._jpeg_cache = (stamp, quality, data)
            return data, stamp, wall

    def rgb(self, width: int = 0):
        """최신 프레임을 (H×W×3 uint8 RGB 배열, 받은 monotonic 시각)으로. 없으면 None. ROS 드라이버가 쓴다.

        width > 0이면 가로세로 비율을 지켜 그 폭으로 줄인다. jpeg()와 같은 잠금 아래에서 변환한다(PyAV 동시 변환 문제).
        """
        with self._jpeg_lock:
            with self._lock:
                frame, stamp = self._frame, self._frame_mono
            if frame is None:
                return None
            if width and width < frame.width:
                height = int(round(frame.height * width / frame.width / 2)) * 2
                return frame.to_ndarray(format="rgb24", width=width, height=height), stamp
            return frame.to_ndarray(format="rgb24"), stamp

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


def _encode_jpeg(frame, quality: int) -> bytes | None:
    """PyAV 프레임 → JPEG. OpenCV가 빠르고(develop 콘솔), 없으면 Pillow."""
    try:
        import cv2
    except ImportError:
        import io

        buf = io.BytesIO()
        frame.to_image().save(buf, format="JPEG", quality=quality)
        return buf.getvalue()
    ok, encoded = cv2.imencode(".jpg", frame.to_ndarray(format="bgr24"), [cv2.IMWRITE_JPEG_QUALITY, quality])
    return encoded.tobytes() if ok else None
