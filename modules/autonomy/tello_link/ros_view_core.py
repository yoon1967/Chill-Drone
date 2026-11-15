"""자율 비행(ROS) 보기 노드에서 ROS와 무관한 부분 (표준 라이브러리 + 영상 JPEG는 OpenCV 또는 Pillow).

자율 비행 프로그램(ROS 2, WSL)은 그대로 두고, 그 토픽을 구독해 게이트웨이가 가져가게 내준다(보기 연결).
ROS 노드(ros_view_node.py)가 받은 값을 ViewState에 넣고, 여기 HTTP 서버가 내준다. 게이트웨이(ros_view.py)가 붙는다.

- WSL 안에서 127.0.0.1에만 연다. Windows 게이트웨이는 127.0.0.1로 붙는다(WSL 기본 localhost 넘겨주기).
  WSL 주소·드론 Wi-Fi로는 보이지 않고, Windows 방화벽 규칙이 필요 없다(11/13 확인).
- 시각: 보기 노드(WSL)와 게이트웨이(Windows)는 시계가 달라 절대 시각 대신 "받은 지 몇 초"(age_s)를 준다.
- 영상: 받은 그림(rgb8·bgr8)은 그대로 두고, 게이트웨이가 가져갈 때 JPEG로 바꾼다(아무도 안 보면 바꾸지 않는다).
- 명령은 아직 받지 않는다(보기만, 1단계).

    GET /api/state    Tello 상태 원문·자율 드라이버 상태·전방 ToF·영상 정보·토픽별 받은 수 (ros-view/0.1)
    GET /frame.jpg    최신 장 하나 (헤더 X-Frame-Seq, X-Frame-Age-Ms)
    GET /video.mjpg   새 장이 올 때마다 한 장씩 (multipart, 같은 헤더)
"""

from __future__ import annotations

import io
import json
import math
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit

SCHEMA = "ros-view/0.1"
DEFAULT_PORT = 8094
JPEG_QUALITY = 75
BOUNDARY = "rosviewframe"


def encode_jpeg(width: int, height: int, encoding: str, data: bytes, step: int | None = None,
                quality: int = JPEG_QUALITY) -> bytes | None:
    """sensor_msgs/Image(rgb8·bgr8) 내용 → JPEG. OpenCV가 있으면 OpenCV, 없으면 Pillow. 모르는 형식·크기 오류는 None."""
    enc = (encoding or "").lower()
    row = step or width * 3
    if enc not in ("rgb8", "bgr8") or width <= 0 or height <= 0 or row < width * 3 or len(data) < row * height:
        return None
    try:
        import cv2
        import numpy as np
    except ImportError:
        return _encode_pillow(width, height, enc, data, row, quality)
    img = np.frombuffer(data, dtype=np.uint8, count=row * height).reshape(height, row)[:, :width * 3]
    img = img.reshape(height, width, 3)
    if enc == "rgb8":
        img = img[:, :, ::-1]   # OpenCV는 BGR
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, quality])
    return buf.tobytes() if ok else None


def _encode_pillow(width, height, enc, data, row, quality):
    try:
        from PIL import Image
    except ImportError:
        return None
    packed = data if row == width * 3 else b"".join(data[y * row:y * row + width * 3] for y in range(height))
    img = Image.frombytes("RGB", (width, height), bytes(packed[:width * height * 3]), "raw", "RGB" if enc == "rgb8" else "BGR")
    out = io.BytesIO()
    img.save(out, "JPEG", quality=quality)
    return out.getvalue()


def scan_range(ranges) -> float | None:
    """전방 ToF LaserScan(한 줄)의 거리 m. 범위 밖은 inf 그대로(자율 드라이버가 inf로 낸다), 없거나 NaN이면 None."""
    if not ranges:
        return None
    r = float(ranges[0])
    return None if math.isnan(r) else r


class ViewState:
    """ROS 콜백이 넣고 HTTP 서버가 꺼내 가는 최신 값. 스레드 안전."""

    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.started = clock()
        self._lock = threading.Lock()
        self._new_frame = threading.Condition(self._lock)
        self._state: tuple[str, float | None, float] | None = None      # (원문, ROS 시각, 받은 때)
        self._flight: tuple[dict, float] | None = None                  # (자율 드라이버 상태, 받은 때)
        self._tof: tuple[float | None, float] | None = None             # (m 또는 inf, 받은 때)
        self._frame: tuple | None = None                                # (seq, 받은 때, w, h, encoding, step, data)
        self._jpeg: tuple[int, bytes] | None = None                     # (seq, JPEG) 마지막으로 바꾼 장
        self._seq = 0
        self._frame_times: deque = deque(maxlen=31)
        self._topics: dict[str, list] = {}                               # 이름 → [받은 수, 마지막 받은 때]
        self.bad = 0                                                     # 읽지 못한 메시지 수

    def _seen(self, topic: str, now: float) -> None:
        t = self._topics.setdefault(topic, [0, now])
        t[0] += 1
        t[1] = now

    # ---------- ROS 콜백이 부른다 ----------
    def on_state(self, data: str, topic: str = "/tt/state") -> None:
        """자율 드라이버의 `/tt/state`: JSON {stamp, raw}(raw = Tello 상태 원문)."""
        now = self.clock()
        try:
            msg = json.loads(data)
            raw = msg["raw"]
            if not isinstance(raw, str) or not raw:
                raise ValueError("raw")
        except (ValueError, KeyError, TypeError):
            with self._lock:
                self.bad += 1
            return
        stamp = msg.get("stamp")
        with self._lock:
            self._state = (raw, float(stamp) if isinstance(stamp, (int, float)) else None, now)
            self._seen(topic, now)

    def on_flight_status(self, data: str, topic: str = "/tt/flight_status") -> None:
        """자율 드라이버의 `/tt/flight_status`: JSON(떠 있음·진행 중·이륙 전 점검·자동 착륙 까닭 등)."""
        now = self.clock()
        try:
            msg = json.loads(data)
            if not isinstance(msg, dict):
                raise ValueError("not a dict")
        except ValueError:
            with self._lock:
                self.bad += 1
            return
        with self._lock:
            self._flight = (msg, now)
            self._seen(topic, now)

    def on_front_tof(self, range_m: float | None, topic: str = "/tt/tof_front") -> None:
        now = self.clock()
        with self._lock:
            self._tof = (range_m, now)
            self._seen(topic, now)

    def on_image(self, width: int, height: int, encoding: str, data: bytes, step: int | None = None,
                 topic: str = "/tt/camera/image_raw") -> None:
        """그림을 받기만 한다(JPEG는 가져갈 때). 받은 순서대로 seq를 올린다."""
        now = self.clock()
        with self._new_frame:
            self._seq += 1
            self._frame = (self._seq, now, int(width), int(height), encoding, step, bytes(data))
            self._frame_times.append(now)
            self._seen(topic, now)
            self._new_frame.notify_all()

    # ---------- HTTP 서버가 부른다 ----------
    def jpeg(self) -> tuple[int, bytes, float] | None:
        """최신 장의 (seq, JPEG, 받은 지 초). 같은 장은 한 번만 바꾼다. 그림이 없거나 못 바꾸면 None."""
        with self._lock:
            frame, cached = self._frame, self._jpeg
        if frame is None:
            return None
        seq, mono, w, h, encoding, step, data = frame
        if cached is not None and cached[0] == seq:
            return seq, cached[1], self.clock() - mono
        out = encode_jpeg(w, h, encoding, data, step)
        if out is None:
            with self._lock:
                self.bad += 1
            return None
        with self._lock:
            if self._jpeg is None or self._jpeg[0] < seq:
                self._jpeg = (seq, out)
        return seq, out, self.clock() - mono

    def wait_frame(self, after_seq: int, timeout: float) -> bool:
        """after_seq보다 새 장이 오면 True(timeout이 지나면 False)."""
        with self._new_frame:
            return self._new_frame.wait_for(lambda: self._seq > after_seq, timeout)

    def snapshot(self) -> dict:
        now = self.clock()
        age = lambda t: round(now - t, 3)   # noqa: E731
        with self._lock:
            state, flight, tof, frame = self._state, self._flight, self._tof, self._frame
            times = list(self._frame_times)
            topics = {k: {"count": v[0], "age_s": age(v[1])} for k, v in self._topics.items()}
            bad = self.bad
        fps = None
        if len(times) >= 2 and times[-1] > times[0]:
            fps = round((len(times) - 1) / (times[-1] - times[0]), 1)
        tof_m = None if tof is None else tof[0]
        return {
            "schema_version": SCHEMA, "up_s": age(self.started), "bad": bad, "topics": topics,
            "state": None if state is None else {"raw": state[0], "ros_stamp": state[1], "age_s": age(state[2])},
            "flight_status": None if flight is None else {**flight[0], "age_s": age(flight[1])},
            "front_tof": None if tof is None else {
                "m": tof_m if tof_m is not None and math.isfinite(tof_m) else None,
                "inf": tof_m is not None and math.isinf(tof_m), "age_s": age(tof[1])},
            "video": None if frame is None else {"seq": frame[0], "age_s": age(frame[1]), "width": frame[2],
                                                 "height": frame[3], "encoding": frame[4], "fps": fps},
        }


def make_server(view: ViewState, host: str = "127.0.0.1", port: int = DEFAULT_PORT) -> ThreadingHTTPServer:
    """보기 노드 HTTP 서버. serve_forever()로 돌리고, 끌 때는 server.stopping.set() 뒤 shutdown()."""

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):   # 10Hz로 묻는다. 줄마다 찍지 않는다
            pass

        def _send(self, code: int, body: bytes, ctype: str, extra: dict | None = None) -> None:
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            for k, v in (extra or {}).items():
                self.send_header(k, v)
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = urlsplit(self.path).path
            if path == "/api/state":
                self._send(200, json.dumps(view.snapshot(), ensure_ascii=False).encode("utf-8"), "application/json")
            elif path == "/frame.jpg":
                f = view.jpeg()
                if f is None:
                    self._send(503, b'{"reason": "no_frame"}', "application/json")
                else:
                    self._send(200, f[1], "image/jpeg", {"X-Frame-Seq": str(f[0]), "X-Frame-Age-Ms": str(round(f[2] * 1000))})
            elif path == "/video.mjpg":
                self._stream()
            elif path == "/":
                self._send(200, json.dumps({"schema_version": SCHEMA, "paths": ["/api/state", "/frame.jpg", "/video.mjpg"]}).encode(),
                           "application/json")
            else:
                self._send(404, b'{"reason": "not_found"}', "application/json")

        def _stream(self) -> None:
            self.send_response(200)
            self.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={BOUNDARY}")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            seq = 0
            try:
                while not server.stopping.is_set():
                    if not view.wait_frame(seq, 1.0):
                        continue
                    f = view.jpeg()
                    if f is None or f[0] <= seq:
                        continue
                    seq = f[0]
                    head = (f"--{BOUNDARY}\r\nContent-Type: image/jpeg\r\nContent-Length: {len(f[1])}\r\n"
                            f"X-Frame-Seq: {f[0]}\r\nX-Frame-Age-Ms: {round(f[2] * 1000)}\r\n\r\n")
                    self.wfile.write(head.encode("ascii") + f[1] + b"\r\n")
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                pass   # 게이트웨이가 끊었다

    server = ThreadingHTTPServer((host, port), Handler)
    server.daemon_threads = True
    server.stopping = threading.Event()
    return server
