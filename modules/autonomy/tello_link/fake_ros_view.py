"""가짜 자율 비행 보기 노드(시험용): ROS 없이 보기 노드(ros_view_node.py)와 같은 HTTP를 낸다.

진짜 노드와 같은 ros_view_core(값 보관·JPEG·HTTP)를 쓰고, 자율 드라이버 토픽 대신 지어낸 값을 넣는다.
땅 8초 → 이륙(2초) → 비행 30초(천천히 돈다) → 착륙(2초)을 되풀이한다. 그림은 480×360(자율 launch 기본)·15장/초.

    python modules/autonomy/tello_link/fake_ros_view.py                 # 127.0.0.1:8094
    python modules/autonomy/tello_link/gateway_server.py --ros-view http://127.0.0.1:8094 --port 8079

필요: 그림을 만들고 JPEG로 바꾸려면 numpy·OpenCV(게이트웨이 venv에 있다). --no-video면 없어도 된다.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from ros_view_core import DEFAULT_PORT, ViewState, make_server  # noqa: E402

GROUND_S, TAKEOFF_S, FLY_S, LAND_S = 8.0, 2.0, 30.0, 2.0
WIDTH, HEIGHT = 480, 360


def phase_at(t: float) -> tuple[str, float]:
    """되풀이 안에서 지금 단계(ground·takeoff·flying·land)와 그 단계에 들어선 지 몇 초."""
    t %= GROUND_S + TAKEOFF_S + FLY_S + LAND_S
    for name, length in (("ground", GROUND_S), ("takeoff", TAKEOFF_S), ("flying", FLY_S), ("land", LAND_S)):
        if t < length:
            return name, t
        t -= length
    return "ground", 0.0


def tello_raw(t: float, phase: str, into: float) -> str:
    """Tello 상태 원문 한 줄(자율 드라이버가 /tt/state의 raw로 넘기는 모양)."""
    up = {"ground": 0.0, "takeoff": into / TAKEOFF_S, "flying": 1.0, "land": 1.0 - into / LAND_S}[phase]
    tof = max(10, round(10 + 80 * up + (3 * math.sin(t) if phase == "flying" else 0)))   # cm, 땅이면 10
    h = round(90 * up)
    yaw = round((into * 6) % 360 - 180) if phase == "flying" else 0
    bat = max(20, 95 - int(t / 10))
    temp = 55 + min(20, int(t / 30))
    return (f"mid:-1;x:-100;y:-100;z:-100;mpry:0,0,0;pitch:{round(2 * math.sin(t))};roll:1;yaw:{yaw};"
            f"vgx:0;vgy:0;vgz:0;templ:{temp};temph:{temp + 3};tof:{tof};h:{h};bat:{bat};baro:{12.5 + h / 100:.2f};"
            f"time:{int(t) if phase != 'ground' else 0};agx:0.00;agy:0.00;agz:-1000.00;")


def flight_status(phase: str, bat: int, temp: int) -> dict:
    """자율 드라이버 /tt/flight_status 모양(FlightController.status + 드라이버가 더한 것). 값은 지어낸 것."""
    checks = [
        {"key": "sdk", "title": "SDK 모드", "ok": True, "detail": "command → ok", "blocking": True},
        {"key": "state", "title": "상태 수신", "ok": True, "detail": "0.1초 전", "blocking": True},
        {"key": "battery", "title": "배터리 10% 초과 (이하면 바로 착륙)", "ok": bat > 10, "detail": f"{bat}%", "blocking": True},
        {"key": "temp", "title": "온도 80°C 미만", "ok": temp < 80, "detail": f"{temp}°C", "blocking": True},
        {"key": "operator", "title": "주변 비움·가드 장착·안전 담당 확인", "ok": phase != "ground", "detail": "자율 비행 웹에서",
         "blocking": True},
    ]
    # FlightController처럼 이륙 명령이 끝나기 전에는 떠 있지 않다(busy만 takeoff), 착륙 중에는 아직 떠 있다
    return {"flying": phase in ("flying", "land"), "busy": phase if phase in ("takeoff", "land") else None,
            "can_takeoff": False, "preflight": checks, "rc_limit": 80, "height_limit_m": 1.5, "ceiling_cap_m": None,
            "height_source": "down_tof", "last_rc": [0, 0, 0, 0], "input_source": "autonomy", "last_auto_land": None,
            "last_warning": None, "connected": True, "last_result": None, "landing_reason": None, "state_rate_hz": 10.0,
            "source": "fake", "battery_land_pct": 10.0, "temp_land_c": 85.0}


def frames(view: ViewState, stop: threading.Event, fps: float = 15.0) -> None:
    import numpy as np

    bars = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
    colors = [(200, 200, 200), (200, 200, 60), (60, 200, 200), (60, 200, 60), (200, 60, 200), (200, 60, 60), (60, 60, 200)]
    for i, c in enumerate(colors):   # RGB
        bars[:, i * WIDTH // len(colors):(i + 1) * WIDTH // len(colors)] = c
    t0, n = time.monotonic(), 0
    while not stop.wait(max(0.0, t0 + n / fps - time.monotonic())):
        img = bars.copy()
        x = int((WIDTH - 60) * (0.5 + 0.5 * math.sin(n / fps)))
        img[HEIGHT // 2 - 30:HEIGHT // 2 + 30, x:x + 60] = (255, 255, 255)
        view.on_image(WIDTH, HEIGHT, "rgb8", img.tobytes(), WIDTH * 3)
        n += 1


def feed(view: ViewState, stop: threading.Event) -> None:
    t0 = time.monotonic()
    i = 0
    while not stop.wait(0.1):
        t = time.monotonic() - t0
        phase, into = phase_at(t)
        raw = tello_raw(t, phase, into)
        view.on_state(json.dumps({"stamp": round(t, 3), "raw": raw}))
        if i % 2 == 0:   # 드라이버는 0.2초마다
            bat = max(20, 95 - int(t / 10))
            view.on_flight_status(json.dumps(flight_status(phase, bat, 55 + min(20, int(t / 30)))))
            view.on_front_tof(float("inf") if phase != "flying" or (int(t) % 10) < 3 else 0.6 + 0.4 * math.sin(t / 2))
        i += 1


def main() -> None:
    ap = argparse.ArgumentParser(description="가짜 자율 비행 보기 노드(게이트웨이 --ros-view 시험용)")
    ap.add_argument("--bind", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--no-video", action="store_true", help="그림을 만들지 않는다(numpy·OpenCV 없이)")
    args = ap.parse_args()
    view, stop = ViewState(), threading.Event()
    threading.Thread(target=feed, args=(view, stop), daemon=True).start()
    if not args.no_video:
        threading.Thread(target=frames, args=(view, stop), daemon=True).start()
    server = make_server(view, args.bind, args.port)
    print(f"가짜 자율 비행 보기 노드: http://{args.bind}:{args.port}  (땅 {GROUND_S:.0f}초 → 비행 {FLY_S:.0f}초 되풀이)", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        server.stopping.set()
        server.server_close()


if __name__ == "__main__":
    main()
