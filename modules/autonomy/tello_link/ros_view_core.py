"""자율 비행(ROS) 보기 노드에서 ROS와 무관한 부분 (표준 라이브러리 + 영상 JPEG는 OpenCV 또는 Pillow).

자율 비행 프로그램(ROS 2, WSL)은 그대로 두고, 그 토픽을 구독해 게이트웨이가 가져가게 내준다(보기 연결).
ROS 노드(ros_view_node.py)가 받은 값을 ViewState에 넣고, 여기 HTTP 서버가 내준다. 게이트웨이(ros_view.py)가 붙는다.

- WSL 안에서 127.0.0.1에만 연다. Windows 게이트웨이는 127.0.0.1로 붙는다(WSL 기본 localhost 넘겨주기).
  WSL 주소·드론 Wi-Fi로는 보이지 않고, Windows 방화벽 규칙이 필요 없다(11/13 확인).
- 시각: 보기 노드(WSL)와 게이트웨이(Windows)는 시계가 달라 절대 시각 대신 "받은 지 몇 초"(age_s)를 준다.
- 영상: 받은 그림(rgb8·bgr8)은 그대로 두고, 게이트웨이가 가져갈 때 JPEG로 바꾼다(아무도 안 보면 바꾸지 않는다).
- 자율 정보(2단계): 위치(`/tt/pose_est`)·계획 경로(`/tt/path`)·임무 단계(`/tt/mission_state`)·찍은 자리(`/tt/operator_goal`),
  찾은 사람(자율 비행 기록 폴더 flights/<시각>/people.json 중 가장 새 회차, 자율 비행 웹과 같은 파일),
  지금 하는 일과 이유(`/tt/mission_note`, 11/17: 남은 탐색 시간·출발점으로 돌아가는 이유·내린 이유, 1초마다).
- 지도(11/17): 비행 노드가 카메라·ToF로 만드는 지도의 비행 높이 단면(`/tt/obstacle_grid`)과 찾은 장애물(같은 회차 폴더의
  discovered.json). 자율 비행 웹(/api/grid)과 같은 모양으로 줄여 둔다: 칸마다 1바이트 0 모름·1 본 빈 곳·2 의심·3 막힘.
  자율 비행의 `world` 좌표(켠 자리 원점, x 앞·y 왼쪽, m)는 게이트웨이 `map`과 같은 약속이라 바꾸지 않는다.
- 명령(3단계): 자율 비행 웹(autonomy_web.command)과 **같은 명령을 같은 토픽·문자열로** 낸다. 관제 화면 버튼 → 게이트웨이 → 여기.
  /tt/mission_cmd: start · land · goto <x> <y> · goto cancel, /tt/flight_cmd: emergency(두 번) · confirm · unconfirm.
  착륙은 웹처럼, 떠 있는데 임무가 없거나 끝·중단이면 드라이버에도 land를 보낸다. 받을지 말지는 비행 노드·드라이버가 정한다.
  publish를 주지 않으면(보기만, commands:=false) 명령을 받지 않는다.
  YOLO 준비(/tt/detections가 3초 안에 옴)와 need_yolo를 알려 준다(웹이 시작 버튼을 켜는 조건과 같게 게이트웨이가 쓴다).

    GET  /api/state    Tello 상태 원문·자율 드라이버 상태·전방 ToF·영상·자율 정보 요약·YOLO·명령·토픽별 받은 수 (ros-view/0.1)
                       ?states_after=번호: 놓친 Tello 상태까지, ?log_after=번호: 그 뒤 자율 비행 노드 경고·오류(/rosout WARN 이상)
    GET  /api/path     계획 경로 점 전체(version이 오르면 다시 가져간다)
    GET  /api/people   찾은 사람 전체(people.json 모양 그대로 + version·file)
    GET  /api/grid     지도 단면 {ver, res, w, h, x0, y0, cells(base64, 칸마다 0 모름·1 본 곳·2 의심·3 막힘), counts}
                       + 높이 층이 있으면 heights(base64, 칸마다 0 없음·1 낮은 물체·2 벽·높은 것·3 높이 모름)·heights_counts·heights_age_s
    GET  /api/discovered  찾은 장애물(discovered.json 모양 그대로 + version·file)
    GET  /api/map      자율 비행 웹이 그리는 지도 파일(load_map과 같은 키: bounds·spawn·goal·walls·obstacles·open 등 + version·file).
                       시뮬레이터는 세계 지도(world_dir/known_map.json: 벽·가구·목적지), 실기체는 가장 새 회차 map.json(빈 지도, 출발만)
    GET  /frame.jpg    최신 장 하나 (헤더 X-Frame-Seq, X-Frame-Age-Ms)
    GET  /video.mjpg   새 장이 올 때마다 한 장씩 (multipart, 같은 헤더)
    POST /api/<명령>   mission/start · mission/land · mission/goto {x, y} · mission/goto_cancel · flight/emergency ·
                       flight/confirm {confirmed} (자율 비행 웹 API와 같은 이름). 헤더 X-Ros-View-Client: 1, Host 127.0.0.1만.
"""

from __future__ import annotations

import base64
import io
import json
import math
import os
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlsplit

SCHEMA = "ros-view/0.1"
DEFAULT_PORT = 8094
JPEG_QUALITY = 75
BOUNDARY = "rosviewframe"
MISSION_CMD = "/tt/mission_cmd"   # 자율 비행 웹과 같은 토픽(std_msgs/String)
FLIGHT_CMD = "/tt/flight_cmd"
COMMANDS = ("mission/start", "mission/land", "mission/goto", "mission/goto_cancel", "flight/emergency", "flight/confirm")
CLIENT_HEADER = "X-Ros-View-Client"   # 브라우저 페이지가 몰래 POST하지 못하게(직접 단 헤더는 CORS 사전 확인이 필요하다)
LOCAL_HOSTS = ("127.0.0.1", "localhost")
MAX_BODY = 4096
YOLO_FRESH_S = 3.0   # 자율 비행 웹: YOLO 결과가 3초 안에 왔으면 준비됨
# 최근 Tello 상태를 번호와 함께 모아 둔다(약 13초). 게이트웨이가 GET /api/state?states_after=번호로 놓친 것까지 받는다:
# 마지막 상태 하나만 주면 게이트웨이 한 바퀴가 0.35초 넘게 늦을 때 상태가 빠져 지도용 센서 자세의 원점이 새로 잡혔다
# (11/16 배터리, 드라이버 쪽 끊김 0번인데 게이트웨이 쪽 15번)
STATE_HISTORY = 128
# 지도 단면(nav_msgs/OccupancyGrid int8) → 칸 코드. 비행 노드 publish_obstacle_grid가 내는 값은 -1 모름·0 본 빈 곳·50 의심·100 막힘이고,
# 자율 비행 웹 on_grid와 같게 0 모름·1 본 곳·2 의심·3 막힘으로 바꾼다(그 밖의 값은 모름). int8을 바이트로 읽은 값(−1 = 255)의 표
GRID_CODES = bytes(1 if v == 0 else 2 if v == 50 else 3 if v == 100 else 0 for v in range(256))
MAX_GRID_CELLS = 4_000_000   # 지도 한 장 상한(비행 노드 기본 200×200, 넓혀도 이 안)
# 이륙 전 기수 값 확인(11/17 비행 노드 d95a9aa): 땅에서 Tello yaw 값이 흐르는 동안 takeoff를 보내면 Tello가 error로 거절해 왔다
# (자율 비행 실기체 기록 요청 52번 중 거절 23번 모두, 우리 11/16·11/17 첫 이륙 거절도). 비행 노드는 흐르는 동안 시작을 받지 않고,
# 30초(TAKEOFF_AGAIN) 안에 한 번 더 누르면 그대로 요청한다. 게이트웨이(ros_view.py)는 막지 않고 화면 시작 칸에 알린다.
# 잣대는 비행 노드 start_blocked·자율 비행 웹 시작 칸과 같다(simulation/tt-sim/autonomy/tello_state.py yaw_span)
YAW_SETTLE_WINDOW_S = 10.0   # 이 시간 동안
YAW_SETTLE_DEG = 2.0         #   기수 값이 이만큼(°) 이상 바뀌었으면 흐르는 중
YAW_SETTLE_MIN_S = 3.0       # 상태가 이만큼도 안 쌓였으면 아직 모른다
TAKEOFF_AGAIN_S = 30.0       # 비행 노드: 흘러서 받지 않은 시작을 이 안에 한 번 더 누르면 그대로 요청한다
LOG_WARN = 30                # rcl_interfaces/Log 등급: WARN 30, ERROR 40, FATAL 50(그 아래 INFO·DEBUG는 안 모은다)
LOG_HISTORY = 200            # 자율 비행 노드 경고·오류를 이만큼 모아 둔다(게이트웨이가 번호로 놓친 것까지 가져간다)


def yaw_span(samples, now: float, window: float = YAW_SETTLE_WINDOW_S) -> float | None:
    """최근 window초 동안 기수 값(°)이 움직인 폭. 비행 노드 tello_state.yaw_span과 같다(±180° 넘김은 이어 붙인다).
    samples는 (받은 때, °). 그 사이 상태가 YAW_SETTLE_MIN_S초 미만이면 None(아직 모른다)."""
    recent = [(t, y) for t, y in samples if now - window <= t <= now]
    if len(recent) < 2 or recent[-1][0] - recent[0][0] < YAW_SETTLE_MIN_S:
        return None
    lo = hi = acc = 0.0
    for (_, a), (_, b) in zip(recent, recent[1:]):
        acc += (b - a + 180.0) % 360.0 - 180.0
        lo, hi = min(lo, acc), max(hi, acc)
    return hi - lo


def grid_codes(raw: bytes) -> bytes:
    """OccupancyGrid data(int8을 바이트로) → 칸 코드 바이트."""
    return bytes(raw).translate(GRID_CODES)


# 지도 높이 층(11/17): 지도 단면(/tt/obstacle_grid)은 비행 높이 ±0.35 m를 자른 것이라 높이가 없어, 벽과 책상·의자가 같은 '막힘'이었다.
# 비행 노드는 3D 카메라 지도(바닥~2.5 m, 0.1 m 칸)를 회차 폴더 camera_map.npz로 30초마다·끝날 때 저장한다(mission_node save_camera_map).
# 칸 기둥마다 꼭대기(0.3–2.4 m 안에서 막힘 점수가 의심 0.3을 넘는 가장 높은 칸의 윗면)로 나눈다. 바닥 0.3 m·의심 0.3은 비행 노드
# camera_map.py(floor·suspect)와 같은 잣대다. 기준값은 11/17 실기체 두 비행으로 정했다(4drone parts/drafts/map2d-obstacles-plan-1117.md).
HEIGHT_NONE, HEIGHT_LOW, HEIGHT_TALL, HEIGHT_UNSURE = 0, 1, 2, 3   # 칸 코드: 없음·낮은 물체·벽·높은 것·높이 모름
HEIGHT_FLOOR_M = 0.3        # 이 아래는 바닥(비행 노드 floor)
HEIGHT_CEILING_M = 2.4      # 이 위는 천장(비행 노드 천장 2.5 m)
HEIGHT_SUSPECT = 0.3        # 이 점수(log-odds)를 넘으면 막힌 칸으로 본다(비행 노드 suspect)
HEIGHT_TALL_M = 1.6         # 꼭대기가 이 이상이면 벽·높은 것(벽·칸막이·높은 장·서 있는 사람)
HEIGHT_LOW_M = 1.4          # 꼭대기가 이 이하이고
HEIGHT_FREE_ABOVE_M = 0.6   #   그 바로 위 이 높이 안에
HEIGHT_FREE_MIN = 2         #   카메라가 비어 있다고 본 칸이 이만큼 있으면 낮은 물체(책상·의자·탁자). 나머지는 높이 모름
HEIGHT_SETTLE_S = 1.0       # 파일을 고친 뒤 이만큼 그대로면 읽는다(비행 노드는 따로 스레드에서 압축해 쓴다)
HEIGHT_GIVE_UP_S = 10.0     # 고친 지 이만큼 지나도 못 읽으면 그 판은 버린다(다음 저장을 기다린다)


def camera_map_heights(path: str) -> tuple[bytes, dict]:
    """비행 노드 camera_map.npz → (칸마다 높이 층 바이트, {w, h, x0, y0, res, counts}). 칸 순서는 /api/grid cells와 같다(y 줄마다 x).
    numpy는 여기서만 불러온다(없으면 ImportError). 읽지 못하거나 모양이 이상하면 예외를 그대로 낸다(부르는 쪽이 다음에 다시)."""
    import numpy as np
    with open(path, "rb") as fh, np.load(fh) as z:   # 파일은 직접 열고 닫는다(읽다 실패해도 비행 노드가 쓰는 파일을 붙잡지 않게)
        lo, seen = z["logodds"], z["seen"]
        origin, res = np.asarray(z["origin"], dtype=float).reshape(-1), float(z["res"])
    if lo.ndim != 3 or seen.shape != lo.shape or origin.size != 3 or not (res > 0 and math.isfinite(res)):
        raise ValueError("camera_map shape")
    nx, ny, nz = lo.shape
    sus = lo > HEIGHT_SUSPECT
    k0 = max(0, int(math.floor((HEIGHT_FLOOR_M - origin[2]) / res + 1e-9)))
    k1 = min(nz, int(math.floor((HEIGHT_CEILING_M - origin[2]) / res + 1e-9)))
    if k1 > k0:
        top = np.where(sus[:, :, k0:k1], np.arange(k0, k1)[None, None, :], -1).max(axis=2)   # 기둥마다 가장 높은 막힌 칸
    else:
        top = np.full((nx, ny), -1)
    top_m = origin[2] + (top + 1) * res                                                    # 그 칸의 윗면
    free = seen & ~sus
    above = np.zeros((nx, ny), dtype=np.int32)                                             # 꼭대기 바로 위에서 본 빈 칸 수
    for dk in range(1, int(round(HEIGHT_FREE_ABOVE_M / res)) + 1):
        k = top + dk
        ok = (top >= 0) & (k < k1)
        above += ok & np.take_along_axis(free, np.clip(k, 0, nz - 1)[:, :, None], axis=2)[:, :, 0]
    has = top >= 0
    cls = np.zeros((nx, ny), dtype=np.uint8)
    cls[has] = HEIGHT_UNSURE
    cls[has & (top_m >= HEIGHT_TALL_M - 1e-6)] = HEIGHT_TALL
    cls[has & (top_m <= HEIGHT_LOW_M + 1e-6) & (above >= HEIGHT_FREE_MIN)] = HEIGHT_LOW
    info = {"w": int(nx), "h": int(ny), "x0": float(origin[0]), "y0": float(origin[1]), "res": res,
            "counts": {"low": int((cls == HEIGHT_LOW).sum()), "tall": int((cls == HEIGHT_TALL).sum()),
                       "unsure": int((cls == HEIGHT_UNSURE).sum())}}
    return cls.T.tobytes(), info   # [x, y] → y 줄마다 x(OccupancyGrid와 같은 순서)


def same_grid(a: dict, b: dict) -> bool:
    """두 지도(크기·원점·칸 크기)가 같은 칸을 가리키나. 높이 층은 지도 단면과 같을 때만 싣는다(시뮬레이터·다른 회차 파일을 막는다)."""
    return ((a.get("w"), a.get("h")) == (b.get("w"), b.get("h"))
            and all(isinstance(a.get(k), (int, float)) and isinstance(b.get(k), (int, float)) and abs(a[k] - b[k]) < 1e-6
                    for k in ("x0", "y0", "res")))


def goto_command(body: dict) -> str | None:
    """지도에서 찍은 자리 {x, y}(m) → 비행 노드 명령 `goto <x> <y>`. 숫자가 아니거나 터무니없으면 None(자율 비행 웹과 같은 식)."""
    x, y = body.get("x"), body.get("y")
    ok = all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) and abs(v) <= 1000 for v in (x, y))
    return f"goto {x:.2f} {y:.2f}" if ok else None


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


def quat_yaw(x: float, y: float, z: float, w: float) -> float:
    """사원수 → yaw(rad, 반시계 +). 자율 비행 노드와 같은 식."""
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def pose_sigma(cov) -> float | None:
    """Odometry 위치 공분산 → 불확실성 m. 위치 추정 노드는 모르면 −1을 낸다(그때 None, 0으로 채우지 않는다)."""
    if cov is None or len(cov) < 8:
        return None
    var = max(float(cov[0]), float(cov[7]))
    return None if math.isnan(var) or var < 0 else math.sqrt(var)


def newest_run_file(root: str, name: str) -> str | None:
    """자율 비행 기록 폴더(root/<시각>/) 중 가장 새 회차의 그 파일. 그 회차에 아직 없으면 None(지난 회차 것을 쓰지 않는다).
    회차 폴더 이름이 시각(YYYYmmdd_HHMMSS)이라 이름 순이 시간 순이다."""
    try:
        runs = [d for d in os.listdir(root) if os.path.isdir(os.path.join(root, d))]
    except OSError:
        return None
    if not runs:
        return None
    f = os.path.join(root, max(runs), name)
    return f if os.path.isfile(f) else None


def newest_people_file(root: str) -> str | None:
    return newest_run_file(root, "people.json")


def watch_json(on_change, name: str, root: str | None, path: str | None, stop: threading.Event, period: float = 1.0) -> None:
    """가장 새 회차의 그 JSON 파일을 지켜본다(자율 비행 웹과 같은 방식: 바뀌면 다시 읽는다). path를 주면 그 파일만.
    on_change(data, file): 새 내용. 새 회차가 시작돼 아직 파일이 없으면 on_change(None, None)."""
    seen = None   # (파일, 고친 시각)
    while not stop.is_set():
        f = path or (newest_run_file(root, name) if root else None)
        if f is None:
            if seen is not None:
                on_change(None, None)
                seen = None
        else:
            try:
                mtime = os.path.getmtime(f)
                if (f, mtime) != seen:
                    with open(f, encoding="utf-8") as fh:
                        data = json.load(fh)
                    if isinstance(data, dict):
                        on_change(data, f)
                    seen = (f, mtime)
            except (OSError, ValueError):
                pass   # 쓰는 중이면 다음에 다시 읽는다
        stop.wait(period)


def watch_people(view: "ViewState", root: str | None, path: str | None, stop: threading.Event, period: float = 1.0) -> None:
    """찾은 사람 파일(people.json)을 지켜본다."""
    watch_json(view.on_people, "people.json", root, path, stop, period)


def watch_discovered(view: "ViewState", root: str | None, path: str | None, stop: threading.Event, period: float = 1.0) -> None:
    """비행 노드가 찾은 장애물 파일(discovered.json, 자율 비행 웹이 지도에 빨갛게 그리는 것)을 지켜본다."""
    watch_json(view.on_discovered, "discovered.json", root, path, stop, period)


# 자율 비행 웹 load_map이 남기는 키(그 밖은 버린다)
MAP_KEYS = ("stage", "description", "bounds", "flight_height", "spawn", "goal", "walls", "obstacles", "open")


def watch_world_map(view: "ViewState", root: str | None, path: str | None, stop: threading.Event, period: float = 2.0) -> None:
    """자율 비행 웹이 그리는 지도 파일을 지켜본다(11/17 "똑같이"): 시뮬레이터는 path로 세계 지도(world_dir/known_map.json),
    실기체는 가장 새 회차의 map.json(real.launch가 남기는 빈 지도: 출발만, 벽·가구 없음)."""
    watch_json(view.on_world_map, "map.json", root, path, stop, period)


def watch_camera_map(view: "ViewState", root: str | None, path: str | None, stop: threading.Event, period: float = 2.0,
                     settle: float = HEIGHT_SETTLE_S, log=None) -> None:
    """비행 노드 3D 카메라 지도(camera_map.npz)를 지켜보고 높이 층을 넣는다(찾은 사람 파일과 같은 방식: 가장 새 회차, path면 그 파일).
    고친 시각이 바뀌고 settle초 그대로면 읽는다. 쓰는 중이라 못 읽으면 다음에 다시. 새 회차에 아직 없으면 높이 층을 비운다.
    numpy가 없으면 높이 층 없이 그만둔다(나머지는 그대로 돈다)."""
    try:
        import numpy  # noqa: F401
    except ImportError:
        if log:
            log("numpy가 없어 지도 높이 층(벽·낮은 물체)을 만들지 않습니다. 나머지는 그대로입니다")
        return
    seen = None   # (파일, 고친 시각): 읽었거나 버린 판
    while not stop.is_set():
        f = path or (newest_run_file(root, "camera_map.npz") if root else None)
        if f is None or not os.path.isfile(f):
            if seen is not None:
                view.on_heights(None, None)
                seen = None
        else:
            try:
                mtime = os.path.getmtime(f)
                if (f, mtime) != seen and time.time() - mtime >= settle:
                    try:
                        codes, info = camera_map_heights(f)
                    except Exception:   # 압축이 덜 끝났거나 모양이 다르다: 조금 기다려도 안 되면 이 판은 버린다
                        if time.time() - mtime >= HEIGHT_GIVE_UP_S:
                            seen = (f, mtime)
                    else:
                        view.on_heights(codes, info, f)
                        seen = (f, mtime)
            except OSError:
                pass
        stop.wait(period)


class ViewState:
    """ROS 콜백이 넣고 HTTP 서버가 꺼내 가는 최신 값. 스레드 안전."""

    def __init__(self, clock=time.monotonic, publish=None, need_yolo: bool = False):
        self.clock = clock
        self.started = clock()
        self.publish = publish        # (토픽, 문자열) → 자율 비행 프로그램에 낸다. None이면 명령을 받지 않는다(보기만)
        self.need_yolo = need_yolo    # 자율 비행 웹 need_yolo와 같은 뜻: YOLO가 준비돼야 시작
        self._yolo: float | None = None                                   # /tt/detections를 마지막으로 받은 때
        self._commands: deque = deque(maxlen=20)                         # (이름, 낸 문자열들, 받은 때)
        self.commands = 0
        self._lock = threading.Lock()
        self._new_frame = threading.Condition(self._lock)
        self._state: tuple[str, float | None, float] | None = None      # (원문, ROS 시각, 받은 때)
        self._states: deque = deque(maxlen=STATE_HISTORY)               # (번호, 원문, ROS 시각, 받은 때) 최근 것들
        self._state_seq = 0
        self._flight: tuple[dict, float] | None = None                  # (자율 드라이버 상태, 받은 때)
        self._tof: tuple[float | None, float] | None = None             # (m 또는 inf, 받은 때)
        self._frame: tuple | None = None                                # (seq, 받은 때, w, h, encoding, step, data)
        self._jpeg: tuple[int, bytes] | None = None                     # (seq, JPEG) 마지막으로 바꾼 장
        self._seq = 0
        self._frame_times: deque = deque(maxlen=31)
        self._topics: dict[str, list] = {}                               # 이름 → [받은 수, 마지막 받은 때]
        self._pose: tuple[dict, float] | None = None                    # 자율 위치(map 좌표), 받은 때
        self._path: tuple[list, float] | None = None                    # 계획 경로 점, 받은 때
        self._path_version = 0
        self._mission: tuple[str, float] | None = None                  # 임무 단계 이름(WAIT_POSE·FOLLOW 등), 받은 때
        self._goal: tuple[dict, float] | None = None                    # 찍은 자리 상태, 받은 때
        self._note: tuple[dict, float] | None = None                    # 지금 하는 일과 이유, 받은 때
        self._logs: deque = deque(maxlen=LOG_HISTORY)                     # 자율 비행 노드 경고·오류 (번호, 등급, 노드, 글, 받은 때)
        self._log_seq = 0
        self._people: tuple[dict, str | None, float] | None = None      # people.json 내용, 파일, 읽은 때
        self._people_version = 0
        self._grid: tuple[dict, bytes, float] | None = None             # 지도 단면(크기·원점·칸 수), 칸 코드, 받은 때
        self._grid_version = 0
        self._heights: tuple[bytes, dict, str | None, float] | None = None   # 지도 높이 층(camera_map_heights), 파일, 읽은 때
        self._found: tuple[dict, str | None, float] | None = None       # discovered.json 내용, 파일, 읽은 때
        self._found_version = 0
        self._world_map: tuple[dict, str | None, float] | None = None   # 자율 비행 웹 지도 파일(MAP_KEYS만), 파일, 읽은 때
        self._world_map_version = 0
        self._truth: tuple[dict, float] | None = None                   # 시뮬레이션 정답 위치(/tt/odometry_gt), 받은 때
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
        stamp = float(stamp) if isinstance(stamp, (int, float)) else None
        with self._lock:
            self._state = (raw, stamp, now)
            self._state_seq += 1
            self._states.append((self._state_seq, raw, stamp, now))
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

    def on_pose(self, x: float, y: float, z: float, yaw: float, sigma: float | None, topic: str = "/tt/pose_est") -> None:
        """자율 위치 추정(Odometry). world 좌표 = 게이트웨이 map 좌표(켠 자리 원점, x 앞·y 왼쪽, m)."""
        now = self.clock()
        if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in (x, y, z, yaw)):
            with self._lock:
                self.bad += 1
            return
        sig = round(float(sigma), 3) if isinstance(sigma, (int, float)) and math.isfinite(sigma) else None
        pose = {"x_m": round(float(x), 3), "y_m": round(float(y), 3), "z_m": round(float(z), 3),
                "yaw_rad": round(float(yaw), 4), "sigma_m": sig}
        with self._lock:
            self._pose = (pose, now)
            self._seen(topic, now)

    def on_path(self, points, topic: str = "/tt/path") -> None:
        """계획 경로(nav_msgs/Path의 점들). 바뀌었을 때만 version을 올린다."""
        now = self.clock()
        pts = [[round(float(p[0]), 3), round(float(p[1]), 3), round(float(p[2]), 3)] for p in points
               if len(p) >= 3 and all(isinstance(v, (int, float)) and math.isfinite(v) for v in p[:3])]
        with self._lock:
            if self._path is None or self._path[0] != pts:
                self._path_version += 1
            self._path = (pts, now)
            self._seen(topic, now)

    def on_mission_state(self, text: str, topic: str = "/tt/mission_state") -> None:
        """비행 노드의 지금 단계 이름(바뀔 때와 1초마다 온다)."""
        now = self.clock()
        state = (text or "").strip()[:40]
        if not state:
            return
        with self._lock:
            self._mission = (state, now)
            self._seen(topic, now)

    def on_goal(self, data: str, topic: str = "/tt/operator_goal") -> None:
        """찍은 자리 상태 JSON {seq, t, state(none·pending·going·arrived), text, click, goal}."""
        now = self.clock()
        try:
            goal = json.loads(data)
            if not isinstance(goal, dict):
                raise ValueError("not a dict")
        except ValueError:
            with self._lock:
                self.bad += 1
            return
        with self._lock:
            self._goal = (goal, now)
            self._seen(topic, now)

    def on_note(self, data: str, topic: str = "/tt/mission_note") -> None:
        """비행 노드의 지금 하는 일과 이유 JSON {t, state, search{elapsed, limit}, returning, why_return, return_home, why_end}.
        단계가 바뀔 때와 1초마다 온다. 그대로 두고 게이트웨이가 가져간다(자율 비행 웹처럼 3초 넘게 안 오면 게이트웨이가 낡은 것으로 본다)."""
        now = self.clock()
        try:
            note = json.loads(data)
            if not isinstance(note, dict):
                raise ValueError("not a dict")
        except ValueError:
            with self._lock:
                self.bad += 1
            return
        with self._lock:
            self._note = (note, now)
            self._seen(topic, now)

    def on_log(self, level: int, node: str, text: str, topic: str = "/rosout") -> None:
        """자율 비행 노드들의 경고·오류(/rosout, WARN 이상). 번호를 붙여 모아 두고 게이트웨이가 ?log_after=번호로 놓친 것까지 가져간다.
        11/17 실기체 충돌: "바로 밑 0.32 m에 무엇이 있습니다", "부딪힘 … 닿은 쪽" 같은 비행 노드 경고가 화면에 안 보였다."""
        if not isinstance(level, int) or level < LOG_WARN or not isinstance(text, str) or not text.strip():
            return
        now = self.clock()
        with self._lock:
            self._log_seq += 1
            self._logs.append((self._log_seq, level, str(node)[:60], text.strip()[:300], now))
            self._seen(topic, now)

    def on_people(self, data: dict | None, file: str | None) -> None:
        """찾은 사람 파일 내용(None이면 지금 회차에 아직 없음). 바뀌었을 때만 version을 올린다."""
        now = self.clock()
        with self._lock:
            before = self._people[0] if self._people else None
            if data != before:
                self._people_version += 1
            self._people = None if data is None else (data, file, now)
            if data is not None:
                self._seen("people.json", now)

    def on_grid(self, res: float, width: int, height: int, x0: float, y0: float, data: bytes,
                topic: str = "/tt/obstacle_grid") -> None:
        """비행 노드의 지도 단면(nav_msgs/OccupancyGrid). data는 int8 칸 값을 바이트로(y 줄마다 x가 이어짐).
        크기가 맞지 않거나 숫자가 아니면 버린다."""
        now = self.clock()
        ok = (type(width) is int and type(height) is int and 0 < width and 0 < height and width * height <= MAX_GRID_CELLS
              and len(data) == width * height
              and all(isinstance(v, (int, float)) and math.isfinite(v) for v in (res, x0, y0)) and res > 0)
        if not ok:
            with self._lock:
                self.bad += 1
            return
        codes = grid_codes(data)
        info = {"res": float(res), "w": width, "h": height, "x0": float(x0), "y0": float(y0),
                "counts": {"blocked": codes.count(3), "suspect": codes.count(2), "seen": codes.count(1)}}
        with self._lock:
            self._grid_version += 1
            self._grid = (info, codes, now)
            self._seen(topic, now)

    def on_discovered(self, data: dict | None, file: str | None) -> None:
        """찾은 장애물 파일 내용(None이면 지금 회차에 아직 없음). 바뀌었을 때만 version을 올린다."""
        now = self.clock()
        with self._lock:
            before = self._found[0] if self._found else None
            if data != before:
                self._found_version += 1
            self._found = None if data is None else (data, file, now)
            if data is not None:
                self._seen("discovered.json", now)

    def on_world_map(self, data: dict | None, file: str | None) -> None:
        """자율 비행 웹이 그리는 지도 파일(None이면 없음). 웹 load_map과 같은 키만 남기고, 바뀌었을 때만 version을 올린다."""
        now = self.clock()
        keep = None if data is None else {k: data.get(k) for k in MAP_KEYS}
        with self._lock:
            before = self._world_map[0] if self._world_map else None
            if keep != before:
                self._world_map_version += 1
            self._world_map = None if keep is None else (keep, file, now)
            if keep is not None:
                self._seen("map.json", now)

    def world_map(self) -> dict | None:
        """GET /api/map: 자율 비행 웹 /api/map과 같은 내용(+ version·file·frame)."""
        with self._lock:
            if self._world_map is None:
                return None
            data, file, _ = self._world_map
            return {**data, "version": self._world_map_version, "file": file, "frame": "map"}

    def on_truth(self, x: float, y: float, z: float, yaw: float, topic: str = "/tt/odometry_gt") -> None:
        """시뮬레이션 정답 위치(자율 비행 웹 truth:=true와 같은 토픽). 실기체에는 없다."""
        now = self.clock()
        if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in (x, y, z, yaw)):
            with self._lock:
                self.bad += 1
            return
        with self._lock:
            self._truth = ({"x_m": round(float(x), 3), "y_m": round(float(y), 3), "z_m": round(float(z), 3),
                            "yaw_rad": round(float(yaw), 4)}, now)
            self._seen(topic, now)

    def on_heights(self, codes: bytes | None, info: dict | None, file: str | None = None) -> None:
        """지도 높이 층(camera_map_heights). None이면 지금 회차에 아직 없음. 바뀌면 지도 번호도 올린다(게이트웨이가 /api/grid를 다시 가져가게,
        착륙 뒤 지도 단면이 멈춰도 마지막 높이 층이 가도록)."""
        now = self.clock()
        with self._lock:
            before = self._heights[0] if self._heights else None
            if codes != before:
                self._grid_version += 1
            self._heights = None if codes is None or info is None else (codes, info, file, now)
            if self._heights is not None:
                self._seen("camera_map.npz", now)

    def grid(self) -> dict | None:
        """GET /api/grid: 자율 비행 웹 /api/grid와 같은 모양(+ counts·frame). 높이 층이 지도 단면과 같은 칸이면 heights도 싣는다."""
        with self._lock:
            if self._grid is None:
                return None
            info, codes, _ = self._grid
            ver = self._grid_version
            heights = self._heights
        out = {"ver": ver, **info, "frame": "map", "cells": base64.b64encode(codes).decode("ascii")}
        if heights is not None and same_grid(info, heights[1]) and len(heights[0]) == len(codes):
            out.update(heights=base64.b64encode(heights[0]).decode("ascii"), heights_counts=heights[1]["counts"],
                       heights_age_s=round(self.clock() - heights[3], 3))
        return out

    def discovered(self) -> dict | None:
        with self._lock:
            if self._found is None:
                return None
            data, file, _ = self._found
            return {**data, "version": self._found_version, "file": file, "frame": "map"}

    def path(self) -> dict:
        with self._lock:
            pts = list(self._path[0]) if self._path else []
            return {"version": self._path_version, "frame": "map", "points": pts}

    def people(self) -> dict | None:
        with self._lock:
            if self._people is None:
                return None
            data, file, _ = self._people
            return {**data, "version": self._people_version, "file": file, "frame": "map"}

    def on_detections(self, data: str = "", topic: str = "/tt/detections") -> None:
        """YOLO 탐지 노드 결과(JSON). 받은 때만 본다: 자율 비행 웹처럼 3초 안에 왔으면 YOLO 준비됨."""
        now = self.clock()
        with self._lock:
            self._yolo = now
            self._seen(topic, now)

    # ---------- 명령 (게이트웨이가 POST로 넘긴다) ----------
    def command(self, name: str, body: dict | None = None) -> dict:
        """자율 비행 웹(autonomy_web.command)과 같은 명령을 같은 토픽·문자열로 낸다. 받을지 말지는 비행 노드·드라이버가 정한다."""
        if self.publish is None:
            return {"ok": False, "reason": "commands_disabled"}
        body = body if isinstance(body, dict) else {}
        if name == "mission/start":
            out = [(MISSION_CMD, "start")]
        elif name == "mission/land":
            out = [(MISSION_CMD, "land")]
            with self._lock:
                mission = self._mission[0] if self._mission else None
                flying = bool(self._flight and self._flight[0].get("flying"))
            if flying and mission in (None, "DONE", "ABORT"):   # 비행 노드가 없거나 끝났는데 떠 있으면 드라이버에 바로(웹과 같다)
                out.append((FLIGHT_CMD, "land"))
        elif name == "mission/goto":
            cmd = goto_command(body)
            if cmd is None:
                return {"ok": False, "reason": "bad_xy"}
            out = [(MISSION_CMD, cmd)]
        elif name == "mission/goto_cancel":
            out = [(MISSION_CMD, "goto cancel")]
        elif name == "flight/emergency":
            out = [(FLIGHT_CMD, "emergency")] * 2   # 웹처럼 두 번
        elif name == "flight/confirm":
            out = [(FLIGHT_CMD, "confirm" if bool(body.get("confirmed")) else "unconfirm")]
        else:
            return {"ok": False, "reason": "unknown"}
        for topic, text in out:
            self.publish(topic, text)
        sent = [text for _, text in out]
        with self._lock:
            self._commands.append((name, sent, self.clock()))
            self.commands += 1
        return {"ok": True, "sent": sent}

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

    def snapshot(self, states_after: int | None = None, log_after: int | None = None) -> dict:
        """states_after: 그 번호 다음 Tello 상태를 모두(오래된 것부터, 받은 지 몇 초) 함께 준다. 모아 둔 것보다 앞이면 빠진 수.
        log_after: 그 번호 다음 자율 비행 노드 경고·오류를 함께 준다(logs). 번호는 늘 log.seq로 준다."""
        now = self.clock()
        age = lambda t: round(now - t, 3)   # noqa: E731
        with self._lock:
            state, flight, tof, frame = self._state, self._flight, self._tof, self._frame
            seq = self._state_seq
            history = [s for s in self._states if s[0] > states_after] if states_after is not None else None
            first = self._states[0][0] if self._states else seq + 1
            pose, path, mission, goal, people, note = self._pose, self._path, self._mission, self._goal, self._people, self._note
            path_version, people_version = self._path_version, self._people_version
            grid, grid_version, found, found_version = self._grid, self._grid_version, self._found, self._found_version
            world, world_version, truth = self._world_map, self._world_map_version, self._truth
            times = list(self._frame_times)
            topics = {k: {"count": v[0], "age_s": age(v[1])} for k, v in self._topics.items()}
            bad = self.bad
            yolo, last_cmd, n_cmd = self._yolo, (self._commands[-1] if self._commands else None), self.commands
            log_seq = self._log_seq
            logs = [x for x in self._logs if x[0] > log_after] if log_after is not None else None
        if logs is not None:
            extra_logs = {"logs": [{"seq": x[0], "level": x[1], "node": x[2], "text": x[3], "age_s": age(x[4])} for x in logs]}
        else:
            extra_logs = {}
        fps = None
        if len(times) >= 2 and times[-1] > times[0]:
            fps = round((len(times) - 1) / (times[-1] - times[0]), 1)
        tof_m = None if tof is None else tof[0]
        extra = {} if history is None else {
            "states": [{"seq": s[0], "raw": s[1], "ros_stamp": s[2], "age_s": age(s[3])} for s in history],
            "states_lost": max(0, first - states_after - 1) if 0 <= states_after < seq else 0}
        return {
            "schema_version": SCHEMA, "up_s": age(self.started), "bad": bad, "topics": topics,
            "state": None if state is None else {"raw": state[0], "ros_stamp": state[1], "age_s": age(state[2]), "seq": seq},
            **extra,
            "log": {"seq": log_seq}, **extra_logs,
            "flight_status": None if flight is None else {**flight[0], "age_s": age(flight[1])},
            "front_tof": None if tof is None else {
                "m": tof_m if tof_m is not None and math.isfinite(tof_m) else None,
                "inf": tof_m is not None and math.isinf(tof_m), "age_s": age(tof[1])},
            "video": None if frame is None else {"seq": frame[0], "age_s": age(frame[1]), "width": frame[2],
                                                 "height": frame[3], "encoding": frame[4], "fps": fps},
            "pose": None if pose is None else {**pose[0], "frame": "map", "age_s": age(pose[1])},
            "mission": None if mission is None else {"state": mission[0], "age_s": age(mission[1])},
            "goal": None if goal is None else {**goal[0], "age_s": age(goal[1])},
            "note": None if note is None else {**note[0], "age_s": age(note[1])},
            "path": None if path is None else {"version": path_version, "points": len(path[0]), "age_s": age(path[1])},
            "people": None if people is None else {
                "version": people_version, "file": people[1], "age_s": age(people[2]),
                "confirmed": len(people[0].get("people") or []), "unconfirmed": len(people[0].get("unconfirmed") or [])},
            # 지도는 번호·크기·칸 수만(칸 전체는 /api/grid, 번호가 바뀔 때만 가져간다)
            "grid": None if grid is None else {"ver": grid_version, "age_s": age(grid[2]), "w": grid[0]["w"], "h": grid[0]["h"],
                                               "res": grid[0]["res"], **grid[0]["counts"]},
            "discovered": None if found is None else {
                "version": found_version, "file": found[1], "age_s": age(found[2]),
                "obstacles": len(found[0].get("obstacles") or []) if isinstance(found[0].get("obstacles"), list) else 0},
            # 지도 파일은 번호와 크기만(내용은 /api/map, 번호가 바뀔 때만 가져간다). 정답 위치는 시뮬레이션만
            "world_map": None if world is None else {
                "version": world_version, "file": world[1], "age_s": age(world[2]), "open": bool(world[0].get("open")),
                "walls": len(world[0].get("walls") or []) if isinstance(world[0].get("walls"), list) else 0,
                "obstacles": len(world[0].get("obstacles") or []) if isinstance(world[0].get("obstacles"), list) else 0},
            "truth": None if truth is None else {**truth[0], "age_s": age(truth[1])},
            "yolo": None if yolo is None else {"age_s": age(yolo), "ready": now - yolo < YOLO_FRESH_S},
            "need_yolo": self.need_yolo,
            "commands": {"enabled": self.publish is not None, "count": n_cmd,
                         "last": None if last_cmd is None else {"name": last_cmd[0], "sent": last_cmd[1], "age_s": age(last_cmd[2])}},
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
            url = urlsplit(self.path)
            path = url.path
            if path == "/api/state":
                query = parse_qs(url.query)

                def number(name):
                    v = (query.get(name) or [None])[0]
                    return int(v) if v is not None and v.lstrip("-").isdigit() else None

                snap = view.snapshot(states_after=number("states_after"), log_after=number("log_after"))
                self._send(200, json.dumps(snap, ensure_ascii=False).encode("utf-8"), "application/json")
            elif path == "/frame.jpg":
                f = view.jpeg()
                if f is None:
                    self._send(503, b'{"reason": "no_frame"}', "application/json")
                else:
                    self._send(200, f[1], "image/jpeg", {"X-Frame-Seq": str(f[0]), "X-Frame-Age-Ms": str(round(f[2] * 1000))})
            elif path == "/video.mjpg":
                self._stream()
            elif path == "/api/path":
                self._send(200, json.dumps(view.path()).encode("utf-8"), "application/json")
            elif path == "/api/people":
                people = view.people()
                if people is None:
                    self._send(404, b'{"reason": "no_people_file"}', "application/json")
                else:
                    self._send(200, json.dumps(people, ensure_ascii=False).encode("utf-8"), "application/json")
            elif path == "/api/grid":
                grid = view.grid()
                if grid is None:
                    self._send(404, b'{"reason": "no_grid"}', "application/json")
                else:
                    self._send(200, json.dumps(grid).encode("utf-8"), "application/json")
            elif path == "/api/discovered":
                found = view.discovered()
                if found is None:
                    self._send(404, b'{"reason": "no_discovered_file"}', "application/json")
                else:
                    self._send(200, json.dumps(found, ensure_ascii=False).encode("utf-8"), "application/json")
            elif path == "/api/map":
                world = view.world_map()
                if world is None:
                    self._send(404, b'{"reason": "no_map_file"}', "application/json")
                else:
                    self._send(200, json.dumps(world, ensure_ascii=False).encode("utf-8"), "application/json")
            elif path == "/":
                paths = ["/api/state", "/api/path", "/api/people", "/api/grid", "/api/discovered", "/api/map", "/frame.jpg",
                         "/video.mjpg"] + [f"POST /api/{c}" for c in COMMANDS]
                self._send(200, json.dumps({"schema_version": SCHEMA, "paths": paths}).encode(), "application/json")
            else:
                self._send(404, b'{"reason": "not_found"}', "application/json")

        def do_POST(self):
            """게이트웨이가 넘긴 명령. 같은 PC의 게이트웨이만: Host가 127.0.0.1·localhost이고 X-Ros-View-Client: 1이 있어야 한다."""
            # 본문은 답하기 전에 읽는다: 안 읽은 채 답하고 닫으면 Windows가 연결을 끊어(RST) 상대가 403·404를 못 읽는다
            try:
                n = int(self.headers.get("Content-Length") or 0)
            except ValueError:
                n = -1
            raw = self.rfile.read(n) if 0 < n <= MAX_BODY else b""
            path = urlsplit(self.path).path
            name = path[len("/api/"):] if path.startswith("/api/") else ""
            host = (self.headers.get("Host") or "").rsplit(":", 1)[0].strip("[]")
            if self.headers.get(CLIENT_HEADER) != "1" or host not in LOCAL_HOSTS:
                return self._send(403, b'{"ok": false, "reason": "not_gateway"}', "application/json")
            if name not in COMMANDS:
                return self._send(404, b'{"ok": false, "reason": "unknown"}', "application/json")
            try:
                if n < 0 or n > MAX_BODY:
                    raise ValueError("size")
                body = json.loads(raw or b"{}") if n else {}
            except ValueError:
                return self._send(400, b'{"ok": false, "reason": "bad_body"}', "application/json")
            result = view.command(name, body)
            code = 200 if result.get("ok") else 503 if result.get("reason") == "commands_disabled" else 400
            self._send(code, json.dumps(result, ensure_ascii=False).encode("utf-8"), "application/json")

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
