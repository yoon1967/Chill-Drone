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
- PUT /api/perception/overlay.jpg  AI 합성 영상 1장(헤더 X-Input-Id·X-Frame-Id = 원본 프레임). 같은 PC에서만
- GET /overlay.mjpg        AI 합성 영상(최신 장만). 2초 넘게 안 오면 보내지 않는다
- POST /api/map           3D 지도(gateway-map/0.1, 지도 연결기 gateway_map_bridge.py). 탐지와 같이 기본은 같은 PC에서만 받는다
- GET  /api/map           최신 3D 지도(점군·카메라 경로). 없으면 404
- WS  /ws/autonomy        자율 비행 하나(AUTONOMY_BRIDGE_V0_1). 게이트웨이 → hello·tello_state·flight_status·control,
                          자율 → hello·cmd_vel·mission_state·ack·request·pose·path·people. 기본은 같은 PC만, 두 번째는 busy
- GET  /api/autonomy      자율 위치·경로·찾은 사람 전체 (시험용 가짜 자율: fake_autonomy.py)
--replay <회차 폴더>: 기체 대신 회차 기록(상태·모드·영상·탐지·지도·자율)을 시간에 맞춰 내보낸다(replay.py). 화면은 그대로, 명령은 거절.
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
from urllib.parse import urlsplit

from fastapi import FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from gateway import Gateway  # noqa: E402
from tello_link import DEFAULT_HOST, TelloLink, now_iso  # noqa: E402
from tello_video import mjpeg_part  # noqa: E402

REPO_ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
DEFAULT_LOG_ROOT = os.path.join(REPO_ROOT, "data", "outputs", "tello")
CONTROLLER_DIR = os.path.join(REPO_ROOT, "apps", "controller")
STATE_PERIOD_S = 0.1
MAX_EVENT_BYTES = 1_000_000
MAX_MAP_BYTES = 16_000_000   # 점 6만 개 × 한 줄 약 60바이트 + 여유
LOCAL_HOSTS = ("127.0.0.1", "::1", "localhost")


class ControllerFiles(StaticFiles):
    """관제 화면 파일. 브라우저가 매번 바뀌었는지 물어보게 한다(no-cache: 그대로면 304라 가볍다).

    캐시 지시가 없으면 브라우저가 옛 app.js를 한동안 그대로 써서, 게이트웨이를 새 코드로 다시 켜도
    새 화면이 안 보였다(10/30, 11/05 지도 콘솔 화면).
    """
    async def get_response(self, path, scope):
        response = await super().get_response(path, scope)
        response.headers["Cache-Control"] = "no-cache"
        return response


def build_gateway(args):
    fake = None
    if getattr(args, "replay", None):   # 기록 재생: 기체 없이 회차 폴더를 시간에 맞춰 내보낸다(명령은 모두 거절)
        from replay import ReplayGateway
        return ReplayGateway(args.replay, speed=args.replay_speed, loop=args.replay_loop, start_s=args.replay_start,
                             video=not args.no_video), None
    if args.fake:
        from fake_tello import FakeTello
        fake = FakeTello(cmd_port=18889, state_port=18890, video_port=11112, send_video=not args.no_video)
        fake.temp_override = 50  # 가짜는 온도가 계속 올라 이륙 전 점검에 걸리므로 고정한다
        fake.start()
        host, cmd_port, local_port, state_port, video_port, bind = "127.0.0.1", 18889, 0, 18890, 11112, "127.0.0.1"
    else:
        host, cmd_port, local_port, state_port, video_port, bind = args.drone_ip, 8889, 8889, 8890, 11111, "0.0.0.0"
    link = TelloLink(host=host, cmd_port=cmd_port, local_cmd_port=local_port, state_port=state_port, bind_ip=bind,
                     log_root=args.log_root, source="fake" if args.fake else "tello", allow_flight=True)
    try:
        link.start()
    except OSError:
        link.close()
        if fake:
            fake.close()
        raise
    video = None
    if not args.no_video:
        from tello_video import TelloVideo
        video = TelloVideo(port=video_port, bind_ip=bind)
    return Gateway(link, source="fake" if args.fake else "tello", video=video,
                   mapping_console_port=getattr(args, "mapping_console_port", None),
                   record_video=not getattr(args, "no_record_video", False)), fake


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
        try:
            yield
        finally:
            try:
                gw.close()
            finally:
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
        origin = ws.headers.get("origin")
        host = ws.headers.get("host", "")
        try:
            hostname = urlsplit(f"//{host}").hostname
        except ValueError:
            hostname = None
        scheme = "https" if ws.url.scheme == "wss" else "http"
        if (origin is not None and origin != f"{scheme}://{host}") or (
                getattr(gw, "mapping_console_port", None) is not None and hostname not in LOCAL_HOSTS):
            await ws.close(code=1008)
            return
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

    def autonomy_sender_ok(ws: WebSocket) -> bool:
        # 인증이 없으므로 기본은 같은 PC의 자율(가짜 자율, mirrored 네트워크의 WSL2)만. 다리 노드 단계에서 --allow-remote-autonomy
        return getattr(args, "allow_remote_autonomy", False) or (ws.client is not None and ws.client.host in LOCAL_HOSTS)

    @app.websocket("/ws/autonomy")
    async def ws_autonomy(ws: WebSocket):
        """자율 비행 하나(AUTONOMY_BRIDGE_V0_1). 두 번째 연결은 busy로 닫는다."""
        await ws.accept()
        if not autonomy_sender_ok(ws):
            await ws.send_json({"schema_version": "0.1.0", "type": "error", "reason": "not_local"})
            await ws.close(code=1008)
            return
        key = f"au-{next(ids)}"
        loop = asyncio.get_running_loop()
        queue: asyncio.Queue = asyncio.Queue(maxsize=200)

        def put(msg):   # 게이트웨이 스레드에서 불린다. 자율이 못 받아 쌓이면 버린다(오래된 상태는 쓸모없다)
            if not queue.full():
                queue.put_nowait(msg)

        if not await asyncio.to_thread(gw.autonomy_open, key, lambda msg: loop.call_soon_threadsafe(put, msg)):
            await ws.send_json({"schema_version": "0.1.0", "type": "error", "reason": "busy"})
            await ws.close(code=1013)
            return

        async def sender():
            while True:
                await ws.send_json(await queue.get())

        send_task = asyncio.create_task(sender())
        try:
            while True:
                msg = await ws.receive_json()
                reply = await asyncio.to_thread(gw.autonomy_message, key, msg)
                if reply is not None:
                    put(reply)
        except (WebSocketDisconnect, ValueError, RuntimeError):
            pass
        finally:
            send_task.cancel()
            await asyncio.to_thread(gw.autonomy_closed, key)   # auto였으면 바로 paused

    @app.get("/api/autonomy")
    def get_autonomy():
        """자율 위치·경로·찾은 사람 전체. 상태(drone_state.autonomy)에는 버전만 싣는다."""
        return gw.autonomy.snapshot()

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

    @app.post("/api/map")
    async def post_map(request: Request):
        if not detection_sender_ok(request):
            return JSONResponse({"ok": False, "reason": "not_local"}, 403)
        body = await request.body()
        if len(body) > MAX_MAP_BYTES:
            return JSONResponse({"ok": False, "reason": "too_large"}, 413)
        try:
            msg = json.loads(body)
        except ValueError:
            return JSONResponse({"ok": False, "reason": "invalid_json"}, 400)
        status, out = await asyncio.to_thread(gw.map.receive, msg)
        return JSONResponse(out, status)

    @app.get("/api/map")
    def get_map():
        snap = gw.map.latest()
        if snap is None:
            return JSONResponse({"available": False, "status": gw.map.status()}, 404)
        return snap

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

    @app.put("/api/perception/overlay.jpg")
    async def put_overlay(request: Request):
        if not detection_sender_ok(request):
            return JSONResponse({"ok": False, "reason": "not_local"}, 403)
        h = request.headers
        status, out = gw.overlay.put(await request.body(), h.get("x-input-id"), h.get("x-frame-id"))
        return JSONResponse(out, status)

    def mjpeg(request: Request, latest, running, *, mapping=False):
        async def gen():  # 최신 프레임만 보낸다. 쌓지 않는다.
            last = None
            while running() and not await request.is_disconnected():
                frame = await asyncio.to_thread(latest)
                data = frame[0] if frame is not None and mapping else frame
                if data is not None and data is not last:
                    last = data
                    pose = gw.link.sensor_pose_at(frame[1]) if mapping else None
                    yield mjpeg_part(data, pose)
                await asyncio.sleep(0.04)
        return StreamingResponse(gen(), media_type="multipart/x-mixed-replace; boundary=frame",
                                 headers={"X-Tello-Sensor-Pose": "1"} if mapping else None)

    @app.get("/video.mjpg")
    async def video_mjpg(request: Request, mapping: bool = False):
        if gw.video is None:
            raise HTTPException(404, "영상이 꺼져 있습니다")
        mapping = mapping and hasattr(gw.link, "sensor_pose_at")
        latest = gw.video.jpeg_snapshot if mapping else gw.video.jpeg
        return mjpeg(request, latest, lambda: gw.video.running, mapping=mapping)

    @app.get("/overlay.mjpg")
    async def overlay_mjpg(request: Request):
        # 탐지 프로그램이 멈추면 화면이 원본으로 돌아가며 이 연결을 끊는다. 여기서는 끊지 않는다.
        return mjpeg(request, gw.overlay.latest, lambda: True)

    if os.path.isdir(CONTROLLER_DIR):
        app.mount("/", ControllerFiles(directory=CONTROLLER_DIR, html=True), name="controller")
    return app


def main():
    ap = argparse.ArgumentParser(description="드론 게이트웨이 (관제 화면 ↔ Tello)")
    ap.add_argument("--fake", action="store_true", help="드론 없이 가짜 Tello로 실행")
    ap.add_argument("--replay", metavar="RUN_DIR",
                    help="기록 재생: 게이트웨이 회차 폴더(data/outputs/tello/<회차>)를 시간에 맞춰 다시 내보낸다. 기체에 붙지 않는다")
    ap.add_argument("--replay-speed", type=float, default=1.0, help="재생 속도 배율(기본 1)")
    ap.add_argument("--replay-loop", action="store_true", help="끝나면 처음부터 다시")
    ap.add_argument("--replay-start", type=float, default=0.0, help="재생 시작 위치(초)")
    ap.add_argument("--no-video", action="store_true", help="영상 수신 끄기")
    ap.add_argument("--no-record-video", action="store_true",
                    help="영상 원본을 회차 폴더(video/)에 남기지 않는다. 기본은 영상을 켤 때마다 남긴다(분당 약 10 MB)")
    ap.add_argument("--drone-ip", default=DEFAULT_HOST)
    ap.add_argument("--log-root", default=DEFAULT_LOG_ROOT, help="회차 로그 폴더 (기본: data/outputs/tello)")
    ap.add_argument("--host", default="127.0.0.1", help="폰에서 열려면 0.0.0.0 (같은 망 안에서만)")
    ap.add_argument("--port", type=int, default=8010)
    ap.add_argument("--allow-remote-autonomy", action="store_true",
                    help="다른 주소(WSL2 NAT 등)의 자율 연결도 받는다. 인증이 없으니 같은 PC·같은 망 시험에서만")
    ap.add_argument("--allow-remote-detections", action="store_true",
                    help="다른 PC의 탐지 결과·3D 지도도 받기 (인증이 없으므로 같은 망 시험에서만)")
    args = ap.parse_args()

    import uvicorn
    app = build_app(args)
    mode = "기록 재생" if args.replay else "가짜" if args.fake else "실제"
    print(f"드론 게이트웨이: http://{args.host}:{args.port}  (모드: {mode})")
    if args.replay:
        rec = app.state.gateway.rec
        print(f"재생: {rec.dir} · {rec.t1 - rec.t0:.0f}초 · 영상 조각 {len(rec.parts)}개 · 탐지 {len(rec.dets)}개 (원래 폴더에는 쓰지 않음)")
    else:
        print(f"회차 로그: {app.state.gateway.link.log.dir}")
    uvicorn.run(app, host=args.host, port=args.port, log_level="warning")


if __name__ == "__main__":
    main()
