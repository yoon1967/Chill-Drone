"""Tello 연결 점검·조작 웹 화면.

    python check_server.py                   # 연결 점검만 (이륙 차단)
    python check_server.py --flight          # 조작 화면 켜기 (이륙·rc 허용, 안전 장치 포함)
    python check_server.py --fake --flight   # 드론 없이 가짜 Tello로 조작 화면 확인
    브라우저: http://127.0.0.1:8010

--flight 없이는 비행 명령이 tello_link에서 막힌다. 설정 명령(ap, wifi 등)은 항상 막는다.
"""

from __future__ import annotations

import argparse
import asyncio
import os
import re
import socket
import subprocess
import sys
import threading
import time
from contextlib import asynccontextmanager

from fastapi import Body, FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, Response, StreamingResponse

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from flight_control import FlightController  # noqa: E402
from tello_link import DEFAULT_HOST, TelloLink, classify, now_iso  # noqa: E402
from tello_video import TelloVideo  # noqa: E402

REPO_ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
DEFAULT_LOG_ROOT = os.path.join(REPO_ROOT, "data", "outputs", "tello")
CHECK_QUERIES = ["battery?", "sdk?", "sn?", "wifi?"]

BATTERY_WARN_PCT = 20
# Tello는 지상에서 켜 둔 채로 두면 과열로 꺼진다. 임계값은 실측 전 가정값이다.
TEMP_WARN_C = 75
TEMP_DANGER_C = 85


def wifi_ssid() -> str | None:
    if os.name != "nt":
        return None
    try:
        out = subprocess.run(["netsh", "wlan", "show", "interfaces"], capture_output=True, timeout=3).stdout
    except (OSError, subprocess.TimeoutExpired):
        return None
    text = out.decode("utf-8", "replace") if b"SSID" in out else out.decode("cp949", "replace")
    m = re.search(r"^\s*SSID\s*:\s*(.+?)\s*$", text, re.MULTILINE)
    return m.group(1) if m else None


def local_ip_toward(host: str) -> str | None:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect((host, 9))  # 패킷을 보내지 않고 경로만 고른다
        return s.getsockname()[0]
    except OSError:
        return None
    finally:
        s.close()


class Checker:
    def __init__(self, args):
        self.fake_mode = args.fake
        self.fake = None
        if args.fake:
            from fake_tello import FakeTello
            self.fake = FakeTello(cmd_port=18889, state_port=18890, video_port=11112,
                                  send_state=not args.fake_no_state, send_video=not args.fake_no_video).start()
            host, cmd_port, local_port, state_port, video_port, bind = "127.0.0.1", 18889, 0, 18890, 11112, "127.0.0.1"
        else:
            host, cmd_port, local_port, state_port, video_port, bind = args.drone_ip, 8889, 8889, 8890, 11111, "0.0.0.0"
        self.host = host
        self.flight_mode = args.flight
        self.link = TelloLink(host=host, cmd_port=cmd_port, local_cmd_port=local_port, state_port=state_port,
                              bind_ip=bind, log_root=args.log_root, source="fake" if args.fake else "tello",
                              allow_flight=args.flight).start()
        self.flight = FlightController(self.link).start() if args.flight else None
        self.video = TelloVideo(port=video_port, bind_ip=bind)
        self.stream_requested_mono: float | None = None
        self.stream_on = False
        self.last_check: dict | None = None
        self.info: dict = {}
        self._net_cache: tuple[float, dict] | None = None
        self._lock = threading.Lock()

    def close(self):
        if self.flight:
            self.flight.close()  # 비행 중이면 착륙부터
        self.video.stop()
        self.link.close()
        if self.fake:
            self.fake.close()

    # ---------- 동작 ----------
    def run_check(self) -> dict:
        with self._lock:
            results = [self.link.send("command", timeout=3, source="check").to_dict()]
            if results[0]["ok"]:
                for q in CHECK_QUERIES:
                    r = self.link.send(q, timeout=3, source="check").to_dict()
                    results.append(r)
                    if r["ok"]:
                        self.info[q] = r["response"]
            self.last_check = {"at": now_iso(), "results": results}
            self.link.event("check_run", {"ok": [r["ok"] for r in results]})
            return self.last_check

    def video_on(self) -> dict:
        self.video.start()
        r = self.link.send("streamon", timeout=3, source="check").to_dict()
        if r["ok"]:
            self.stream_on = True
            self.stream_requested_mono = time.monotonic()
        self.link.event("video_on", {"ok": r["ok"]})
        return r

    def video_off(self) -> dict:
        r = self.link.send("streamoff", timeout=3, source="check").to_dict()
        self.stream_on = False
        self.stream_requested_mono = None
        self.video.stop()
        self.link.event("video_off", {"ok": r["ok"]})
        return r

    # ---------- 판정 ----------
    def network(self) -> dict:
        now = time.monotonic()
        if self._net_cache and now - self._net_cache[0] < 3:
            return self._net_cache[1]
        if self.fake_mode:
            info = {"mode": "fake", "ssid": None, "local_ip": "127.0.0.1", "on_tello_subnet": True}
        else:
            ip = local_ip_toward(self.host)
            info = {"mode": "tello", "ssid": wifi_ssid(), "local_ip": ip,
                    "on_tello_subnet": bool(ip and ip.startswith("192.168.10."))}
        self._net_cache = (now, info)
        return info

    def steps(self, net: dict, state: dict | None, rate: float | None, vstats: dict) -> list[dict]:
        steps = []

        # 0. 네트워크
        if net["mode"] == "fake":
            steps.append(_step("network", "네트워크", "ok", "가짜 Tello(이 PC 안)에 연결 중"))
        elif net["on_tello_subnet"]:
            steps.append(_step("network", "네트워크", "ok", f"Tello 망에 있음 · PC {net['local_ip']} · SSID {net['ssid'] or '알 수 없음'}"))
        else:
            steps.append(_step("network", "네트워크", "fail",
                               f"PC가 Tello 망(192.168.10.x)에 없음 · 현재 SSID {net['ssid'] or '알 수 없음'}",
                               "PC Wi-Fi를 TELLO-xxxxxx에 연결하세요. 폰 Tello 앱은 끄고 폰 Wi-Fi 연결도 끊으세요."))

        # 1. SDK 진입
        cmd = next((h for h in reversed(self.link.history) if h["cmd"] == "command"), None)
        if cmd is None:
            steps.append(_step("sdk", "SDK 모드 진입", "idle", "아직 시도하지 않음", "[점검 시작]을 누르세요."))
        elif cmd["ok"]:
            steps.append(_step("sdk", "SDK 모드 진입", "ok", f"command → ok · 왕복 {cmd['rtt_ms']} ms"))
        else:
            steps.append(_step("sdk", "SDK 모드 진입", "fail", f"command → {cmd['response'] or cmd['reason']}",
                               "드론 전원·Wi-Fi 연결을 확인하고, 폰 앱이 켜져 있으면 끄세요."))

        # 2. 기체 정보
        if not self.info:
            steps.append(_step("info", "기체 정보", "idle", "아직 조회하지 않음"))
        else:
            bat = _to_int(self.info.get("battery?"))
            detail = (f"배터리 {self.info.get('battery?', '없음')}% · SDK {self.info.get('sdk?', '없음')} · "
                      f"SN {self.info.get('sn?', '없음')} · Wi-Fi 신호 {self.info.get('wifi?', '없음')}")
            missing = [q for q in CHECK_QUERIES if q not in self.info]
            if bat is not None and bat < BATTERY_WARN_PCT:
                steps.append(_step("info", "기체 정보", "warn", detail, f"배터리 {BATTERY_WARN_PCT}% 미만입니다. 충전 후 비행하세요."))
            elif missing:
                steps.append(_step("info", "기체 정보", "warn", detail, "응답 없는 항목: " + ", ".join(missing)))
            else:
                steps.append(_step("info", "기체 정보", "ok", detail))

        # 3. 상태 수신
        sdk_ok = bool(cmd and cmd["ok"])
        if state is None:
            if sdk_ok and cmd and _age_since(cmd["sent_at"]) > 3:
                steps.append(_step("state", "상태 수신 (UDP 8890)", "fail", "SDK 모드인데 상태 패킷이 오지 않음",
                                   "Windows 방화벽이 Python의 UDP 8890 수신을 막고 있을 가능성이 큽니다. "
                                   "Python을 '개인 네트워크'와 '공용 네트워크' 모두 허용하세요."))
            else:
                steps.append(_step("state", "상태 수신 (UDP 8890)", "idle", "SDK 모드 진입 후 자동으로 들어옵니다"))
        else:
            age = state.get("age_s", 0)
            detail = f"{rate if rate is not None else '-'} Hz · 마지막 패킷 {age:.1f}초 전 · 누적 {self.link.state_count}개"
            if age > 2:
                steps.append(_step("state", "상태 수신 (UDP 8890)", "fail", detail,
                                   "상태가 끊겼습니다. 드론 전원이 꺼졌거나 Wi-Fi가 끊겼는지 확인하세요."))
            elif rate is not None and rate < 5:
                steps.append(_step("state", "상태 수신 (UDP 8890)", "warn", detail, "평소(약 10Hz)보다 느립니다."))
            elif not state["valid"]:
                steps.append(_step("state", "상태 수신 (UDP 8890)", "warn", detail, "형식이 이상한 패킷: " + ", ".join(state["reasons"])))
            else:
                steps.append(_step("state", "상태 수신 (UDP 8890)", "ok", detail))

        # 4. 영상
        if not self.stream_on:
            steps.append(_step("video", "영상 (UDP 11111)", "idle", "영상 꺼짐", "[영상 켜기]를 누르세요."))
        else:
            waited = time.monotonic() - (self.stream_requested_mono or time.monotonic())
            fps = vstats["fps"]
            res = f"{vstats['width']}×{vstats['height']}" if vstats["width"] else "해상도 미확인"
            detail = (f"{fps if fps is not None else '-'} fps · {res} · "
                      f"프레임 나이 {vstats['frame_age_s'] if vstats['frame_age_s'] is not None else '-'}초 · "
                      f"패킷 {vstats['packets']}개")
            if vstats["packets"] == 0 and waited > 5:
                steps.append(_step("video", "영상 (UDP 11111)", "fail", "streamon은 ok인데 영상 패킷이 오지 않음",
                                   "Windows 방화벽이 Python의 UDP 11111 수신을 막고 있을 가능성이 큽니다."))
            elif vstats["frames"] == 0:
                status = "fail" if waited > 8 else "idle"
                steps.append(_step("video", "영상 (UDP 11111)", status,
                                   f"패킷 {vstats['packets']}개 수신, 아직 디코딩된 프레임 없음",
                                   "키프레임을 기다리는 중입니다. 오래 걸리면 영상을 껐다 켜세요." if status == "idle" else
                                   "패킷은 오는데 디코딩이 안 됩니다. 영상을 껐다 켜 보세요."))
            elif vstats["frame_age_s"] is not None and vstats["frame_age_s"] > 2:
                steps.append(_step("video", "영상 (UDP 11111)", "fail", detail, "영상이 끊겼습니다."))
            elif fps is not None and fps < 10:
                steps.append(_step("video", "영상 (UDP 11111)", "warn", detail, "프레임이 느립니다. Wi-Fi 간섭이나 거리를 확인하세요."))
            else:
                steps.append(_step("video", "영상 (UDP 11111)", "ok", detail))
        return steps

    def warnings(self, state: dict | None) -> list[str]:
        out = []
        if state and state.get("si"):
            th = state["si"].get("temp_high_c")
            if isinstance(th, (int, float)):
                if th >= TEMP_DANGER_C:
                    out.append(f"기체 온도 {th}°C: 곧 과열로 꺼질 수 있습니다. 전원을 끄고 식히세요.")
                elif th >= TEMP_WARN_C:
                    out.append(f"기체 온도 {th}°C: 지상에서 오래 켜 두면 과열됩니다.")
        return out

    def status(self) -> dict:
        net = self.network()
        state = self.link.latest_state()
        rate = self.link.state_rate_hz()
        vstats = self.video.stats()
        return {
            "now": now_iso(),
            "mode": "fake" if self.fake_mode else "tello",
            "run_id": self.link.run_id,
            "log_dir": self.link.log.dir if self.link.log else None,
            "flight_blocked": not self.link.allow_flight,
            "network": net,
            "steps": self.steps(net, state, rate, vstats),
            "warnings": self.warnings(state),
            "state": state,
            "state_rate_hz": rate,
            "state_count": self.link.state_count,
            "video": vstats,
            "stream_on": self.stream_on,
            "info": self.info,
            "history": [h for h in self.link.history if not h["cmd"].startswith("rc ")][-30:],
            "flight": self.flight.status() if self.flight else None,
        }


def _step(key, title, status, detail, hint=None):
    return {"key": key, "title": title, "status": status, "detail": detail, "hint": hint}


def _to_int(text):
    try:
        return int(float(text))
    except (TypeError, ValueError):
        return None


def _age_since(iso):
    from datetime import datetime
    try:
        return time.time() - datetime.fromisoformat(iso).timestamp()
    except (TypeError, ValueError):
        return 0.0


def build_app(args) -> FastAPI:
    checker = Checker(args)

    @asynccontextmanager
    async def lifespan(_app):
        yield
        checker.close()

    app = FastAPI(title="Tello 연결 점검·조작", lifespan=lifespan)
    app.state.checker = checker

    @app.get("/")
    def index():
        return FileResponse(os.path.join(HERE, "static", "index.html"))

    @app.get("/api/status")
    def status():
        return checker.status()

    @app.post("/api/check")
    def run_check():
        return checker.run_check()

    @app.post("/api/command")
    def command(body: dict = Body(...)):
        cmd = str(body.get("cmd", "")).strip()
        if not cmd:
            raise HTTPException(400, "cmd가 비어 있습니다")
        if classify(cmd) == "flight":
            # 비행 명령은 이륙 전 점검·데드맨을 거치도록 조작 패널(/api/flight/*)로만 보낸다.
            raise HTTPException(403, {"message": "비행 명령은 조작 패널에서만 보냅니다"})
        r = checker.link.send(cmd, timeout=3, source="manual").to_dict()
        if r["ok"] and cmd in ("battery?", "sdk?", "sn?", "wifi?"):
            checker.info[cmd] = r["response"]
        if r["blocked"]:
            raise HTTPException(403, {"message": "점검 화면에서는 비행·설정 명령을 보내지 않습니다", "result": r})
        return r

    @app.post("/api/video/on")
    def video_on():
        return checker.video_on()

    @app.post("/api/video/off")
    def video_off():
        return checker.video_off()

    # ---------- 조작 (--flight일 때만) ----------
    def flight():
        if checker.flight is None:
            raise HTTPException(403, "조작 모드가 꺼져 있습니다. --flight로 실행하세요")
        return checker.flight

    @app.post("/api/flight/confirm")
    def flight_confirm(body: dict = Body(...)):
        flight().confirm_operator(bool(body.get("confirmed")))
        return flight().status()

    @app.post("/api/flight/takeoff")
    def flight_takeoff():
        return flight().takeoff(source="web")

    @app.post("/api/flight/land")
    def flight_land():
        return flight().land(source="web")

    @app.post("/api/flight/emergency")
    async def flight_emergency():
        # 작업 스레드 대기열을 거치지 않도록 비동기로 바로 보낸다.
        return flight().emergency(source="web")

    @app.websocket("/ws/control")
    async def control_ws(ws: WebSocket):
        await ws.accept()
        ctrl = checker.flight
        if ctrl is None:
            await ws.close(code=1008)
            return
        try:
            while True:
                msg = await ws.receive_json()
                if msg.get("emergency"):
                    ctrl.emergency(source="keyboard")
                elif "rc" in msg:
                    lr, fb, ud, yaw = (float(v) for v in msg["rc"])
                    ctrl.set_input(lr, fb, ud, yaw, source="keyboard")
        except (WebSocketDisconnect, ValueError, TypeError):
            pass
        finally:
            ctrl.set_input(0, 0, 0, 0, source="ws_closed")  # 연결이 끊기면 제자리

    @app.get("/snapshot.jpg")
    def snapshot():
        data = checker.video.jpeg()
        if data is None:
            raise HTTPException(404, "아직 프레임이 없습니다")
        return Response(data, media_type="image/jpeg", headers={"Cache-Control": "no-store"})

    @app.get("/video.mjpg")
    async def video_mjpg(request: Request):
        # 비동기 생성기로 둔다. 동기 생성기는 브라우저가 끊겨도 작업 스레드를 계속 잡고 있어서
        # 연결이 쌓이면 /api/status까지 멈춘다.
        async def gen():
            last = None
            while checker.video.running and not await request.is_disconnected():
                data = await asyncio.to_thread(checker.video.jpeg)
                if data is not None and data is not last:
                    last = data
                    yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + data + b"\r\n"
                await asyncio.sleep(0.04)
        return StreamingResponse(gen(), media_type="multipart/x-mixed-replace; boundary=frame")

    return app


def main():
    ap = argparse.ArgumentParser(description="Tello 연결 점검·조작 웹 화면")
    ap.add_argument("--fake", action="store_true", help="드론 없이 가짜 Tello로 실행")
    ap.add_argument("--flight", action="store_true", help="조작 화면 켜기 (이륙·rc 허용, 안전 장치 포함)")
    ap.add_argument("--fake-no-state", action="store_true", help="가짜: 상태 패킷 차단 상황 재현")
    ap.add_argument("--fake-no-video", action="store_true", help="가짜: 영상 패킷 차단 상황 재현")
    ap.add_argument("--drone-ip", default=DEFAULT_HOST)
    ap.add_argument("--log-root", default=DEFAULT_LOG_ROOT, help="회차 로그 폴더 (기본: data/outputs/tello)")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8010)
    args = ap.parse_args()

    import uvicorn
    app = build_app(args)
    print(f"Tello 연결 점검: http://{args.host}:{args.port}  (모드: {'가짜' if args.fake else '실제'}"
          f"{', 조작 켜짐' if args.flight else ', 이륙 차단'})")
    print(f"회차 로그: {app.state.checker.link.log.dir}")
    uvicorn.run(app, host=args.host, port=args.port, log_level="warning")


if __name__ == "__main__":
    main()
