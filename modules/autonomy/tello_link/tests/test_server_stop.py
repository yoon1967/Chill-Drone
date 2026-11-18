"""게이트웨이를 끌 때(Ctrl+C) 영상 스트림(MJPEG)이 열려 있어도 곧 끝나는지 본다(fastapi·uvicorn이 있는 게이트웨이 환경에서만).

    python -B -m unittest discover -s modules/autonomy/tello_link/tests -p "test_server_stop.py" -v
"""

import asyncio
import http.client
import os
import signal
import socket
import sys
import threading
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    from fastapi import FastAPI, Request
    from fastapi.responses import StreamingResponse
    import uvicorn  # noqa: F401
    import gateway_server  # noqa: E402
except ImportError:  # fastapi·uvicorn 없는 환경
    gateway_server = None


class Frames:
    """부를 때마다 새 JPEG 바이트를 준다(pose_at이 있으면 (JPEG, 프레임 시각))."""

    def __init__(self, timed=False):
        self.n = 0
        self.timed = timed

    def __call__(self):
        self.n += 1
        jpeg = b"\xff\xd8" + self.n.to_bytes(4, "big") + b"\xff\xd9"
        return (jpeg, 100.0 + self.n) if self.timed else jpeg


class Client:
    """손님 연결 흉내(Request.is_disconnected만 쓴다)."""

    def __init__(self):
        self.closed = False

    async def is_disconnected(self):
        return self.closed


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@unittest.skipIf(gateway_server is None, "fastapi·uvicorn이 없는 환경")
class MjpegFramesTest(unittest.TestCase):
    def collect(self, frames, until):
        async def run():
            parts = []
            async for part in frames:
                parts.append(part)
                until(parts)
            return parts
        return asyncio.run(asyncio.wait_for(run(), 3))

    def test_ends_when_gateway_stops(self):
        stopping = threading.Event()
        parts = self.collect(gateway_server.mjpeg_frames(Client(), Frames(), lambda: True, stopping),
                             lambda p: len(p) == 2 and stopping.set())
        self.assertEqual(len(parts), 2)

    def test_ends_when_client_leaves_or_video_turns_off(self):
        client = Client()
        parts = self.collect(gateway_server.mjpeg_frames(client, Frames(), lambda: True, threading.Event()),
                             lambda p: len(p) == 3 and setattr(client, "closed", True))
        self.assertEqual(len(parts), 3)
        video_on = [True]
        parts = self.collect(gateway_server.mjpeg_frames(Client(), Frames(), lambda: video_on[0], threading.Event()),
                             lambda p: video_on.__setitem__(0, False))
        self.assertEqual(len(parts), 1)

    def test_same_frame_is_sent_once(self):   # 최신 프레임만, 같은 장은 다시 안 보낸다
        jpeg = b"\xff\xd8same\xff\xd9"
        stopping = threading.Event()
        calls = []

        def latest():
            calls.append(1)
            if len(calls) >= 5:
                stopping.set()
            return jpeg
        parts = self.collect(gateway_server.mjpeg_frames(Client(), latest, lambda: True, stopping), lambda p: None)
        self.assertEqual(len(parts), 1)

    def test_mapping_part_carries_pose_at_frame_time(self):
        stopping = threading.Event()
        asked = []

        def pose_at(t):
            asked.append(t)
            return {"t": t}
        parts = self.collect(gateway_server.mjpeg_frames(Client(), Frames(timed=True), lambda: True, stopping, pose_at),
                             lambda p: stopping.set())
        self.assertEqual(asked, [101.0])
        self.assertIn(b'X-Tello-Pose: {"t":101.0}', parts[0])


@unittest.skipIf(gateway_server is None, "fastapi·uvicorn이 없는 환경")
class ServerStopTest(unittest.TestCase):
    """실제 uvicorn 서버에 영상 스트림을 열어 둔 채 Ctrl+C(handle_exit)를 보낸다."""

    def start(self, ignore_stop=False):
        app = FastAPI()
        app.state.stopping = threading.Event()
        never = threading.Event()   # 끄기 신호를 안 보는 스트림(안전망 시험용)

        @app.get("/video.mjpg")
        async def video(request: Request):
            frames = gateway_server.mjpeg_frames(request, Frames(), lambda: True,
                                                 never if ignore_stop else app.state.stopping)
            return StreamingResponse(frames, media_type="multipart/x-mixed-replace; boundary=frame")

        port = free_port()
        server = gateway_server.make_server(app, "127.0.0.1", port)
        thread = threading.Thread(target=server.run, daemon=True)
        thread.start()
        deadline = time.monotonic() + 5
        while not server.started and time.monotonic() < deadline:
            time.sleep(0.02)
        self.assertTrue(server.started, "서버가 켜져야 한다")
        conn = http.client.HTTPConnection("127.0.0.1", port, timeout=10)
        self.addCleanup(conn.close)
        conn.request("GET", "/video.mjpg")
        resp = conn.getresponse()
        self.assertEqual(resp.status, 200)
        self.assertTrue(resp.read(16).startswith(b"--frame"))   # 스트림이 열려 있다
        return app, server, thread

    def stop(self, server, thread):
        started = time.monotonic()
        server.handle_exit(signal.SIGINT, None)   # Ctrl+C
        thread.join(gateway_server.GRACEFUL_STOP_S + 5)
        return time.monotonic() - started

    def test_ctrl_c_ends_open_video_stream_quickly(self):
        app, server, thread = self.start()
        took = self.stop(server, thread)
        self.assertFalse(thread.is_alive(), "영상 스트림이 열려 있어도 서버가 끝나야 한다")
        self.assertTrue(app.state.stopping.is_set())
        self.assertLess(took, 2.0)

    def test_request_that_ignores_stop_is_cut_after_grace(self):   # 안전망: 끝나지 않는 응답도 기다림에 끝이 있다
        _, server, thread = self.start(ignore_stop=True)
        with self.assertLogs("uvicorn.error", "ERROR"):   # "Cancel 1 running task(s), timeout graceful shutdown exceeded"
            took = self.stop(server, thread)
        self.assertFalse(thread.is_alive())
        self.assertGreaterEqual(took, gateway_server.GRACEFUL_STOP_S - 0.5)
        self.assertLess(took, gateway_server.GRACEFUL_STOP_S + 3)


if __name__ == "__main__":
    unittest.main()
