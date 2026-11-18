"""Windows에서 돌리는 Tello UDP 중계 (표준 라이브러리만). WSL2 기본(NAT) 네트워크용.

Tello는 상태(8890)·영상(11111)을 명령을 보낸 PC 주소로 보낸다. ROS 2 드라이버가 WSL 안에 있으면 그 패킷이
Windows에서 멈추고 WSL까지 오지 않는다. 이 중계가 Windows에서 Tello와 이야기하고 WSL 드라이버와 패킷을 주고받는다.

  WSL 드라이버 ──명령──▶ Windows:9889 ──▶ Tello 192.168.10.1:8889
  WSL 드라이버 ◀──응답── Windows       ◀── Tello
  WSL 드라이버:8890  ◀──상태── Windows:8890  ◀── Tello (10Hz)
  WSL 드라이버:11111 ◀──영상── Windows:11111 ◀── Tello (streamon 뒤)

드라이버 주소는 처음 받은 명령의 보낸 주소로 안다. 명령은 그대로 넘기므로 차단·안전 장치는 드라이버(tello_link)가 맡는다.

Tello에는 늘 같은 포트(tello_local_port, 기본 8889)에서 보낸다. 실기체는 켠 뒤 처음 `command`를 보낸 포트로 응답을 보내는
것으로 보였다(2026-10-26: 다른 포트에서 먼저 보낸 뒤로는 새 포트에 응답이 오지 않고 상태만 왔다). 그래서 중계를 다시 켜도
같은 포트를 쓰게 하고, 응답이 안 오면 드론을 껐다 켠다. 이 포트는 점검 화면(check_server.py)과 같으므로 동시에 켜지 않는다.

  python tello_relay.py                     # 실기체 (PC Wi-Fi를 TELLO-xxxxxx에 연결한 뒤)
  python tello_relay.py --tello 127.0.0.1 --tello-cmd-port 18889 --state-port 18890 --video-port 11112   # 가짜 Tello로 시험

처음 실행하면 Windows 방화벽이 Python 허용을 물을 수 있다. 개인·공용 네트워크 모두 허용해야 Tello와 WSL 패킷이 들어온다.
"""

from __future__ import annotations

import argparse
import socket
import threading
import time


class TelloRelay:
    def __init__(self, tello: str = "192.168.10.1", tello_cmd_port: int = 8889, driver_port: int = 9889,
                 state_port: int = 8890, video_port: int = 11111, bind: str = "0.0.0.0",
                 driver_state_port: int = 8890, driver_video_port: int = 11111, tello_local_port: int = 8889, log=print):
        self.tello = (tello, tello_cmd_port)
        self.driver_state_port = driver_state_port
        self.driver_video_port = driver_video_port
        self.log = log
        self.driver: tuple[str, int] | None = None   # 명령을 보낸 WSL 드라이버 주소
        self.counts = {"cmd": 0, "reply": 0, "state": 0, "video": 0}
        self._stop = threading.Event()
        self.from_driver = self._sock(bind, driver_port)        # WSL 드라이버가 명령을 보내는 곳
        self.to_tello = self._sock(bind, tello_local_port)      # Tello와 명령·응답을 주고받는 곳 (포트 고정)
        self.state_in = self._sock(bind, state_port)            # Tello 상태
        self.video_in = self._sock(bind, video_port, rcvbuf=1 << 20)
        self.out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    @staticmethod
    def _sock(bind: str, port: int, rcvbuf: int | None = None) -> socket.socket:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if rcvbuf:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, rcvbuf)
        s.bind((bind, port))
        s.settimeout(0.5)
        return s

    def start(self) -> "TelloRelay":
        for target in (self._commands, self._replies, self._state, self._video):
            threading.Thread(target=target, daemon=True).start()
        return self

    def close(self) -> None:
        self._stop.set()
        for s in (self.from_driver, self.to_tello, self.state_in, self.video_in, self.out):
            s.close()

    def _recv(self, sock: socket.socket, size: int):
        while not self._stop.is_set():
            try:
                return sock.recvfrom(size)
            except socket.timeout:
                continue
            except ConnectionResetError:   # Windows: 상대 포트가 닫혀 있으면 ICMP가 이 예외로 온다
                continue
            except OSError as e:
                if self._stop.is_set():
                    return None
                # Wi-Fi를 바꾸는 순간 같은 네트워크 오류로 받는 스레드가 조용히 끝나 중계가 멈춘 적이 있다. 계속 받는다
                self.log(f"받기 오류, 계속합니다: {e}")
                time.sleep(0.5)
        return None

    def _commands(self) -> None:
        while (got := self._recv(self.from_driver, 2048)) is not None:
            data, addr = got
            if self.driver != addr:
                self.driver = addr
                self.log(f"드라이버 {addr[0]}:{addr[1]} 연결. 상태는 :{self.driver_state_port}, 영상은 :{self.driver_video_port}로 보냅니다.")
            self.counts["cmd"] += 1
            try:
                self.to_tello.sendto(data, self.tello)
            except OSError as e:
                self.log(f"Tello로 보내지 못했습니다: {e} (PC Wi-Fi가 TELLO에 연결돼 있는지 확인)")

    def _forward(self, data: bytes, port: int | None = None) -> None:
        if self.driver is None:
            return
        try:
            self.out.sendto(data, (self.driver[0], port) if port else self.driver)
        except OSError:
            pass

    def _replies(self) -> None:
        while (got := self._recv(self.to_tello, 2048)) is not None:
            self.counts["reply"] += 1
            if self.driver is not None:
                try:
                    self.from_driver.sendto(got[0], self.driver)   # 드라이버가 보낸 포트(9889 상대)로 돌려준다
                except OSError:
                    pass

    def _state(self) -> None:
        while (got := self._recv(self.state_in, 2048)) is not None:
            self.counts["state"] += 1
            self._forward(got[0], self.driver_state_port)

    def _video(self) -> None:
        while (got := self._recv(self.video_in, 4096)) is not None:
            self.counts["video"] += 1
            self._forward(got[0], self.driver_video_port)


def main() -> None:
    ap = argparse.ArgumentParser(description="Tello ↔ WSL 드라이버 UDP 중계 (Windows에서 실행)")
    ap.add_argument("--tello", default="192.168.10.1")
    ap.add_argument("--tello-cmd-port", type=int, default=8889)
    ap.add_argument("--driver-port", type=int, default=9889, help="WSL 드라이버가 명령을 보낼 Windows 포트")
    ap.add_argument("--state-port", type=int, default=8890, help="Tello 상태를 받을 Windows 포트")
    ap.add_argument("--video-port", type=int, default=11111, help="Tello 영상을 받을 Windows 포트")
    ap.add_argument("--tello-local-port", type=int, default=8889, help="Tello에 명령을 보낼 Windows 쪽 포트 (고정)")
    args = ap.parse_args()
    relay = TelloRelay(args.tello, args.tello_cmd_port, args.driver_port, args.state_port, args.video_port,
                       tello_local_port=args.tello_local_port).start()
    print(f"Tello 중계: 드라이버 → Windows:{args.driver_port} → Tello {args.tello}:{args.tello_cmd_port}, "
          f"상태 :{args.state_port}, 영상 :{args.video_port} (Ctrl+C 종료)")
    try:
        last = dict(relay.counts)
        while True:
            time.sleep(5)
            c = relay.counts
            print(f"  5초 동안 명령 {c['cmd'] - last['cmd']}, 응답 {c['reply'] - last['reply']}, "
                  f"상태 {c['state'] - last['state']}, 영상 패킷 {c['video'] - last['video']}")
            last = dict(c)
    except KeyboardInterrupt:
        relay.close()


if __name__ == "__main__":
    main()
