"""드론 게이트웨이 웹 서버: 관제 화면(apps/controller)과 상태·명령을 주고받는다.

    python gateway_server.py --fake      # 드론 없이 가짜 Tello로 실행
    python gateway_server.py             # 실제 Tello (드론 Wi-Fi에 연결된 PC에서)
    브라우저: http://127.0.0.1:8010  (실시간 감독 #/live, 현장 조작 #/control)

계약: contracts/api/DRONE_CONTROL_V0_1.md
- GET /api/drone/state    최신 drone_state 1개
- WS  /ws/drone           서버 → 화면: drone_state 약 10Hz, event, result / 화면 → 서버: cmd, rc
- GET /video.mjpg          영상(최신 프레임만)
- GET /snapshot.jpg        최신 프레임 1장 + 헤더 X-Input-Id·X-Frame-Id·X-Frame-Time (탐지 결과 연결용)
- POST /api/detections    사람 탐지 이벤트(person-detection-event/1.0). 기본은 같은 PC에서만 받는다
- GET  /api/detections    발견 목록·탐지 상태, GET·PUT /api/detections/<event_id>.jpg 사진
이륙·rc가 허용된 상태로 뜨므로, 실제 기체에서는 주변을 비우고 실행한다. 이륙은 화면에서 주변 확인 뒤에만 된다.
check_server.py(연결 점검·첫 조작 화면)와 같은 포트를 쓰므로 동시에 켜지 않는다. Tello는 처음 command를 보낸 쪽에만 답한다.
"""

from __future__ import annotations

import argparse
import asyncio
import itertools
import json
import os
import sys
import time
from contextlib import asynccontextmanager

from fastapi import FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from gateway import Gateway  # noqa: E402
from tello_link import DEFAULT_HOST, TelloLink, now_iso  # noqa: E402

REPO_ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
DEFAULT_LOG_ROOT = os.path.join(REPO_ROOT, "data", "outputs", "tello")
CONTROLLER_DIR = os.path.join(REPO_ROOT, "apps", "controller")
STATE_PERIOD_S = 0.1
MAX_EVENT_BYTES = 1_000_000
LOCAL_HOSTS = ("127.0.0.1", "::1", "localhost")


def build_gateway(args):
    fake = None
    if args.fake:
        from fake_tello import FakeTello
        fake = FakeTello(cmd_port=18889, state_port=18890, video_port=11112, send_video=not args.no_video)
        fake.temp_override = 50  # 가짜는 온도가 계속 올라 이륙 전 점검에 걸리므로 고정한다
        fake.start()
        host, cmd_port, local_port, state_port, video_port, bind = "127.0.0.1", 18889, 0, 18890, 11112, "127.0.0.1"
    else:
        host, cmd_port, local_port, state_port, video_port, bind = args.drone_ip, 8889, 8889, 8890, 11111, "0.0.0.0"
    link = TelloLink(host=host, cmd_port=cmd_port, local_cmd_port=local_port, state_port=state_port, bind_ip=bind,
                     log_root=args.log_root, source="fake" if args.fake else "tello", allow_flight=True).start()
    video = None
    if not args.no_video:
        from tello_video import TelloVideo
        video = TelloVideo(port=video_port, bind_ip=bind)
    return Gateway(link, source="fake" if args.fake else "tello", video=video), fake


def build_app(args) -> FastAPI:
    gw, fake = build_gateway(args)
    clients: dict[str, tuple[asyncio.Queue, asyncio.AbstractEventLoop]] = {}
    ids = itertools.count(1)

    def broadcast(msg: dict) -> None:  # 게이트웨이 스레드에서 불린다
        for q, loop in list(clients.values()):
            loop.call_soon_threadsafe(q.put_nowait, msg)

    gw.listeners.append(broadcast)

    # 영상은 자동으로 켜지 않는다(지상 발열). 화면의 video_on 또는 이륙 성공 시 게이트웨이가 켠다.
    @asynccontextmanager
    async def lifespan(_app):
        gw.start()
        yield
        gw.close()
        if gw.video is not None:
            gw.video.stop()
        gw.link.close()
        if fake:
            fake.close()

    app = FastAPI(title="드론 게이트웨이", lifespan=lifespan)
    app.state.gateway = gw

    @app.get("/api/drone/state")
    def drone_state():
        return gw.drone_state()

    @app.websocket("/ws/drone")
    async def ws_drone(ws: WebSocket):
        await ws.accept()
        key = f"ws-{next(ids)}"
        queue: asyncio.Queue = asyncio.Queue(maxsize=200)
        clients[key] = (queue, asyncio.get_running_loop())

        async def sender():
            while True:
                try:
                    msg = await asyncio.wait_for(queue.get(), timeout=STATE_PERIOD_S)
                except asyncio.TimeoutError:
                    msg = gw.drone_state()
                await ws.send_json(msg)

        send_task = asyncio.create_task(sender())
        try:
            await ws.send_json(gw.drone_state())
            while True:
                msg = await ws.receive_json()
                if not isinstance(msg, dict):
                    continue
                result = await asyncio.to_thread(gw.handle_message, msg, key)
                if result is not None:
                    await ws.send_json(result)
                    await ws.send_json(gw.drone_state())  # 버튼 잠금이 바로 바뀌게
        except (WebSocketDisconnect, ValueError, RuntimeError):
            pass
        finally:
            send_task.cancel()
            clients.pop(key, None)
            gw.client_closed(key)  # 그 화면의 rc는 바로 0

    @app.get("/snapshot.jpg")
    def snapshot():
        """최신 프레임 1장. 탐지 결과는 헤더의 X-Input-Id·X-Frame-Id로 이 프레임을 가리킨다."""
        frame, reason = gw.video_frame()
        if frame is None:  # video_disabled(--no-video)는 404, 꺼짐·끊김·첫 프레임 전은 503
            raise HTTPException(404 if reason == "video_disabled" else 503, {"reason": reason})
        headers = {
            "Cache-Control": "no-store",
            "X-Input-Id": frame["input_id"],
            "X-Frame-Id": frame["frame_id"],
            "X-Frame-Time": now_iso(frame["wall"]),          # 게이트웨이 디코딩 시각(UTC), 촬영 시각 아님
            "X-Frame-Age-Ms": str(round((time.monotonic() - frame["mono"]) * 1000)),
        }
        if frame["width"]:
            headers["X-Image-Size"] = f"{frame['width']}x{frame['height']}"
        return Response(frame["jpeg"], media_type="image/jpeg", headers=headers)

    def detection_sender_ok(request: Request) -> bool:
        # 인증이 없으므로 기본은 같은 PC의 탐지 프로그램만. 다른 PC에서 돌리면 --allow-remote-detections
        return getattr(args, "allow_remote_detections", False) or (request.client is not None and request.client.host in LOCAL_HOSTS)

    @app.post("/api/detections")
    async def post_detection(request: Request):
        if not detection_sender_ok(request):
            return JSONResponse({"ok": False, "reason": "not_local"}, 403)
        body = await request.body()
        if len(body) > MAX_EVENT_BYTES:
            return JSONResponse({"ok": False, "reason": "too_large"}, 413)
        try:
            event = json.loads(body)
        except ValueError:
            return JSONResponse({"ok": False, "reason": "invalid_json"}, 400)
        status, out = await asyncio.to_thread(gw.detections.receive, event)
        return JSONResponse(out, status)

    @app.get("/api/detections")
    def list_detections():
        return gw.detections.snapshot()

    @app.get("/api/detections/{event_id}.jpg")
    def detection_photo(event_id: str):
        path = gw.detections.photo_path(event_id)
        if path is None:
            raise HTTPException(404, {"reason": "no_photo"})
        return FileResponse(path, media_type="image/jpeg")

    @app.put("/api/detections/{event_id}.jpg")
    async def put_detection_photo(event_id: str, request: Request):
        if not detection_sender_ok(request):
            return JSONResponse({"ok": False, "reason": "not_local"}, 403)
        status, out = await asyncio.to_thread(gw.detections.put_photo, event_id, await request.body())
        return JSONResponse(out, status)

    @app.get("/video.mjpg")
    async def video_mjpg(request: Request):
        if gw.video is None:
            raise HTTPException(404, "영상이 꺼져 있습니다")

        async def gen():  # 최신 프레임만 보낸다. 쌓지 않는다.
            last = None
            while gw.video.running and not await request.is_disconnected():
                data = await asyncio.to_thread(gw.video.jpeg)
                if data is not None and data is not last:
                    last = data
                    yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + data + b"\r\n"
                await asyncio.sleep(0.04)
        return StreamingResponse(gen(), media_type="multipart/x-mixed-replace; boundary=frame")

    if os.path.isdir(CONTROLLER_DIR):
        app.mount("/", StaticFiles(directory=CONTROLLER_DIR, html=True), name="controller")
    return app


def main():
    ap = argparse.ArgumentParser(description="드론 게이트웨이 (관제 화면 ↔ Tello)")
    ap.add_argument("--fake", action="store_true", help="드론 없이 가짜 Tello로 실행")
    ap.add_argument("--no-video", action="store_true", help="영상 수신 끄기")
    ap.add_argument("--drone-ip", default=DEFAULT_HOST)
    ap.add_argument("--log-root", default=DEFAULT_LOG_ROOT, help="회차 로그 폴더 (기본: data/outputs/tello)")
    ap.add_argument("--host", default="127.0.0.1", help="폰에서 열려면 0.0.0.0 (같은 망 안에서만)")
    ap.add_argument("--port", type=int, default=8010)
    ap.add_argument("--allow-remote-detections", action="store_true",
                    help="다른 PC의 탐지 결과도 받기 (인증이 없으므로 같은 망 시험에서만)")
    args = ap.parse_args()

    import uvicorn
    app = build_app(args)
    print(f"드론 게이트웨이: http://{args.host}:{args.port}  (모드: {'가짜' if args.fake else '실제'})")
    print(f"회차 로그: {app.state.gateway.link.log.dir}")
    uvicorn.run(app, host=args.host, port=args.port, log_level="warning")


if __name__ == "__main__":
    main()
