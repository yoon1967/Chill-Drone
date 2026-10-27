"""드론 없이 점검 화면과 tello_link를 시험하기 위한 가짜 Tello (UDP).

실제 Tello처럼 명령에 응답하고, `command` 이후 상태 패킷을 10Hz로 보내고,
`streamon` 이후 H.264 테스트 영상을 보낸다(PyAV가 있을 때만).
값은 모두 지어낸 것이므로 sn과 상태에 FAKE 표시를 남긴다. 실기체 결과로 쓰지 않는다.

    python fake_tello.py --cmd-port 18889 --state-port 18890 --video-port 11112
"""

from __future__ import annotations

import argparse
import math
import socket
import threading
import time

REPLIES = {
    "battery?": "87",
    "sdk?": "20",
    "sn?": "FAKE0TQZK7",
    "wifi?": "90",
    "speed?": "10.0",
    "time?": "0s",
    "height?": "0dm",
    "temp?": "58~61C",
}


class FakeTello:
    def __init__(self, bind_ip="127.0.0.1", cmd_port=18889, state_port=18890, video_port=11112,
                 send_state=True, send_video=True, reply_delay=0.02):
        self.bind_ip = bind_ip
        self.cmd_port = cmd_port
        self.state_port = state_port
        self.video_port = video_port
        self.send_state = send_state
        self.send_video = send_video
        self.reply_delay = reply_delay
        self.received: list[str] = []
        # 비행 흉내: 이륙하면 80cm까지 오르고, rc 상하 입력으로 높이가 바뀌며, 15초간 명령이 없으면 착륙한다.
        self.flying = False
        self.h_cm = 0.0
        self.bat = 87
        self.temp_override: int | None = None
        self.ext_kit = False             # True면 확장 키트가 달린 것처럼 EXT 명령에 응답한다(값은 지어낸 것)
        self.front_tof_mm = 481
        self.motion_delay = 1.0          # 이륙·착륙에 걸리는 시간(응답이 늦게 온다)
        self._rc = (0, 0, 0, 0)
        self._last_cmd_mono = time.monotonic()
        self._client_ip: str | None = None
        self._stop = threading.Event()
        self._streaming = threading.Event()
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((bind_ip, cmd_port))
        self._sock.settimeout(0.3)
        self._threads: list[threading.Thread] = []

    def start(self) -> "FakeTello":
        for target in (self._cmd_loop, self._state_loop, self._video_loop):
            t = threading.Thread(target=target, daemon=True)
            t.start()
            self._threads.append(t)
        return self

    def close(self) -> None:
        self._stop.set()
        for t in self._threads:
            t.join(timeout=2)
        self._sock.close()

    def _cmd_loop(self) -> None:
        while not self._stop.is_set():
            try:
                data, addr = self._sock.recvfrom(1024)
            except (socket.timeout, ConnectionResetError):
                continue
            except OSError:
                break
            cmd = data.decode("utf-8", "replace").strip()
            self.received.append(cmd)
            self._last_cmd_mono = time.monotonic()
            if cmd.startswith("rc "):
                try:
                    self._rc = tuple(int(v) for v in cmd.split()[1:5])
                except ValueError:
                    pass
                continue  # 실제 Tello도 rc에는 응답하지 않는다
            if cmd == "command":
                self._client_ip = addr[0]
            elif cmd == "streamon":
                self._streaming.set()
            elif cmd == "streamoff":
                self._streaming.clear()
            elif cmd == "emergency":
                self.flying, self.h_cm, self._rc = False, 0.0, (0, 0, 0, 0)
            elif cmd in ("takeoff", "land"):
                # 실제처럼 동작이 끝난 뒤 응답한다. 그동안 다른 명령(emergency)은 계속 받는다.
                threading.Thread(target=self._move, args=(cmd, addr), daemon=True).start()
                continue
            if cmd.lower().startswith("ext "):
                reply = self._ext_reply(cmd) if self.ext_kit else "error"
            else:
                reply = REPLIES.get(cmd, "ok")
            time.sleep(self.reply_delay)
            self._sock.sendto(reply.encode(), addr)

    def _ext_reply(self, cmd: str) -> str:
        low = cmd.lower()
        if low == "ext version?":
            return "esp32vFAKE.0.0.0"
        if low == "ext tof?":
            return f"tof {self.front_tof_mm}"
        if low.startswith("ext mled"):
            return "mled ok"
        if low.startswith("ext led"):
            return "led ok"
        return "error"

    def _move(self, cmd: str, addr) -> None:
        if cmd == "takeoff":
            self.flying = True
            self._rc = (0, 0, 0, 0)
            target = 80.0
        else:
            target = 0.0
        start, t0 = self.h_cm, time.monotonic()
        while time.monotonic() - t0 < self.motion_delay and not self._stop.is_set():
            if cmd == "takeoff" and not self.flying:  # 이륙 중 emergency
                return
            frac = (time.monotonic() - t0) / self.motion_delay
            self.h_cm = start + (target - start) * frac
            time.sleep(0.05)
        self.h_cm = target
        if cmd == "land":
            self.flying = False
        try:
            self._sock.sendto(b"ok", addr)
        except OSError:
            pass

    def _state_loop(self) -> None:
        out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        t0 = time.monotonic()
        while not self._stop.is_set():
            time.sleep(0.1)
            if not (self.send_state and self._client_ip):
                continue
            t = time.monotonic() - t0
            if self.flying:
                self.h_cm = max(0.0, self.h_cm + self._rc[2] / 100 * 100 * 0.1)  # rc 100 ≈ 1m/s로 가정
                if time.monotonic() - self._last_cmd_mono > 15:  # 실제 Tello의 15초 자동 착륙 흉내
                    self.flying, self.h_cm = False, 0.0
            yaw = int(5 * math.sin(t / 3))
            temp = self.temp_override if self.temp_override is not None else 58 + min(int(t / 20), 25)
            h = int(self.h_cm)
            tof = h + 10 if h > 0 else 10
            packet = (
                f"mid:-1;x:-100;y:-100;z:-100;mpry:0,0,0;pitch:0;roll:1;yaw:{yaw};"
                f"vgx:0;vgy:0;vgz:0;templ:{temp};temph:{temp + 3};tof:{tof};h:{h};bat:{self.bat};"
                f"baro:{12.5 + h / 100 + 0.02 * math.sin(t):.2f};time:0;agx:-2.00;agy:1.00;agz:-998.00;\r\n"
            )
            try:
                out.sendto(packet.encode(), (self._client_ip, self.state_port))
            except OSError:
                pass
        out.close()

    def _video_loop(self) -> None:
        try:
            import av
            import numpy as np  # noqa: F401  (PyAV 프레임 생성에 필요)
        except ImportError:
            return
        out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        encoder = None
        n = 0
        while not self._stop.is_set():
            if not (self.send_video and self._streaming.is_set() and self._client_ip):
                encoder = None
                time.sleep(0.1)
                continue
            if encoder is None:
                encoder = av.CodecContext.create("libx264", "w")
                encoder.width, encoder.height = 960, 720
                encoder.pix_fmt = "yuv420p"
                encoder.framerate = 30
                encoder.options = {"tune": "zerolatency", "preset": "ultrafast", "g": "30"}
            frame = av.VideoFrame.from_ndarray(_pattern(n), format="rgb24")
            n += 1
            for packet in encoder.encode(frame):
                data = bytes(packet)
                for i in range(0, len(data), 1460):  # 실제 Tello처럼 1460바이트 단위로 쪼갠다
                    try:
                        out.sendto(data[i:i + 1460], (self._client_ip, self.video_port))
                    except OSError:
                        pass
            time.sleep(1 / 30)
        out.close()


def _pattern(n: int):
    import numpy as np
    h, w = 720, 960
    img = np.zeros((h, w, 3), dtype=np.uint8)
    bars = [(200, 200, 200), (200, 200, 0), (0, 200, 200), (0, 200, 0), (200, 0, 200), (200, 0, 0), (0, 0, 200)]
    for i, color in enumerate(bars):
        img[:, i * w // len(bars):(i + 1) * w // len(bars)] = color
    x = (n * 8) % (w - 80)
    img[h // 2 - 40:h // 2 + 40, x:x + 80] = (255, 255, 255)
    return img


def main() -> None:
    ap = argparse.ArgumentParser(description="가짜 Tello (UDP)")
    ap.add_argument("--bind", default="127.0.0.1")
    ap.add_argument("--cmd-port", type=int, default=18889)
    ap.add_argument("--state-port", type=int, default=18890, help="상태를 보낼 PC 쪽 포트")
    ap.add_argument("--video-port", type=int, default=11112, help="영상을 보낼 PC 쪽 포트")
    ap.add_argument("--no-state", action="store_true", help="방화벽으로 상태가 막힌 상황 재현")
    ap.add_argument("--no-video", action="store_true", help="방화벽으로 영상이 막힌 상황 재현")
    args = ap.parse_args()
    fake = FakeTello(args.bind, args.cmd_port, args.state_port, args.video_port,
                     send_state=not args.no_state, send_video=not args.no_video).start()
    print(f"가짜 Tello: 명령 {args.bind}:{args.cmd_port}, 상태→{args.state_port}, 영상→{args.video_port} (Ctrl+C 종료)")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        fake.close()


if __name__ == "__main__":
    main()
