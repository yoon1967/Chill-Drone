#!/usr/bin/env python3
"""자율비행 웹 화면: 카메라·지도·상태를 보면서 시작·착륙·비상 정지를 누른다.

실기체와 Gazebo 시뮬레이션이 같은 화면을 쓴다. ROS 2 토픽만 주고받으므로 비행 코드와 따로 켜고 끌 수 있다
(화면을 꺼도 비행은 계속되고, 드라이버의 안전 장치는 그대로다).
화면 모양, 이륙 전 점검 신호등, 운용자 확인, Esc 비상 정지는 tello_link 점검 화면(static/index.html)에서 가져왔다.

입력  camera_topic(/tt/camera/image_raw), det_topic(/tt/detections/image, YOLO를 켰을 때)
      /tt/detections(YOLO 결과 JSON: 켜졌는지·초당 몇 번·지금 사람 몇 명을 보인다)
      /tt/obstacle_grid(카메라 지도 단면, 탐색 4단계): 막힘·의심·본 곳을 지도에 그린다
      /tt/pose_est, /tt/odometry_gt(시뮬레이션 정답, truth:=true일 때만), /tt/path, /tt/mission_state,
      /tt/flight_status(Tello 드라이버), /tt/state(Tello 상태)
      /tt/operator_goal(비행 노드): 운용자가 지도에서 찍은 자리의 상태와 알림 글. 기록에 적고 지도에 그리며, 마지막 답은
                                    [여기로 보내기] 옆에 20초 보인다 (11/16: 거절된 이유가 맨 아래 기록에만 나와 못 봤다)
      /tt/mission_note(비행 노드): 지금 하는 일과 이유 — 남은 탐색 시간, 출발점으로 돌아가는 이유, 내린 이유를 화면 위 띠에 보인다 (11/16)
      discovered(파일): 비행 노드가 쓰는 '찾은 장애물' JSON(discovered_out). 바뀌면 다시 읽어 지도에 빨갛게 그린다
      people(파일): 비행 노드가 쓰는 '찾은 사람' JSON(people_out). 길을 막는 사람과 지나간 사람을 나눠 그린다
출력  /tt/mission_cmd  start | land | goto <x> <y> | goto cancel      /tt/flight_cmd  emergency | confirm | unconfirm
      goto: 탐색 중에 지도에서 찍은 자리를 다음 볼 자리로 보낸다 (가는 길·회피는 비행 노드가 그대로 한다, 11/11)
웹    http://localhost:8080  (WSL2에서 띄우면 Windows 브라우저에서 그대로 열린다)

  python3 autonomy_web.py --ros-args -p map:=<known_map.json> [-p discovered:=<discovered.json>] [-p truth:=true] [-p port:=8080]

필요: rclpy, OpenCV(cv2), numpy (ROS 2 Jazzy 설치에 들어 있다). 웹 서버는 표준 라이브러리만 쓴다.
"""
from __future__ import annotations

import base64
import json
import math
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from nav_msgs.msg import Path as PathMsg
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

STATIC = Path(__file__).resolve().parent / "static"
# 경고 기준은 tello_link 점검 화면과 같다 (실측 전 가정값)
BATTERY_WARN_PCT = 20
TEMP_WARN_C = 75
TEMP_DANGER_C = 85
STATE_STALE_S = 1.0
SIGMA_WARN_M = 0.4


def image_to_bgr(msg: Image):
    """sensor_msgs/Image → OpenCV BGR 배열. 모르는 형식이면 None."""
    h, w, step = msg.height, msg.width, msg.step
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1}.get(msg.encoding)
    if channels is None or buf.size < h * step:
        return None
    img = buf[:h * step].reshape(h, step)[:, :w * channels].reshape(h, w, channels) if channels > 1 \
        else buf[:h * step].reshape(h, step)[:, :w]
    conv = {"rgb8": cv2.COLOR_RGB2BGR, "rgba8": cv2.COLOR_RGBA2BGR, "bgra8": cv2.COLOR_BGRA2BGR,
            "mono8": cv2.COLOR_GRAY2BGR}.get(msg.encoding)
    return cv2.cvtColor(img, conv) if conv is not None else img


def parse_tello(raw: str) -> dict:
    out = {}
    for part in raw.strip().strip(";").split(";"):
        if ":" in part:
            k, v = part.split(":", 1)
            try:
                out[k.strip()] = float(v)
            except ValueError:
                out[k.strip()] = v.strip()
    return out


def goto_command(body: dict) -> str | None:
    """지도에서 찍은 자리 {x, y}(m) → 비행 노드 명령 `goto <x> <y>`. 숫자가 아니거나 터무니없으면 None."""
    x, y = body.get("x"), body.get("y")
    ok = all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) and abs(v) <= 1000 for v in (x, y))
    return f"goto {x:.2f} {y:.2f}" if ok else None


def note_events(old: dict | None, new: dict) -> list[str]:
    """비행 노드의 '지금 하는 일과 이유'(/tt/mission_note)가 바뀌었을 때 기록에 적을 말들.
    돌아가기 시작한 이유와 내리는(중단한) 이유만 — 1초마다 오는 나머지는 적지 않는다."""
    old = old or {}
    out = []
    if new.get("returning") and new.get("why_return") and not (old.get("returning") and old.get("why_return") == new.get("why_return")):
        out.append(f"비행 노드: {new['why_return']} 출발점으로 돌아갑니다 (더 보내려면 지도를 찍어 [여기로 보내기])")
    if new.get("why_end") and new.get("why_end") != old.get("why_end"):
        out.append(f"비행 노드: {'중단한' if new.get('state') == 'ABORT' else '내리는'} 이유 — {new['why_end']}")
    return out


def stamp_of(msg) -> float:
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def yaw_of(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Frames:
    """한 영상 토픽의 최신 JPEG 한 장. MJPEG 스트림들이 새 장을 기다린다."""

    def __init__(self) -> None:
        self.cond = threading.Condition()
        self.jpeg: bytes | None = None
        self.seq = 0
        self.times: deque = deque(maxlen=30)
        self.viewers = 0
        self.last_put = 0.0

    def put(self, jpeg: bytes) -> None:
        with self.cond:
            self.jpeg = jpeg
            self.seq += 1
            self.times.append(time.monotonic())
            self.cond.notify_all()

    def wait(self, seq: int, timeout: float):
        with self.cond:
            self.cond.wait_for(lambda: self.seq != seq, timeout)
            return self.seq, self.jpeg

    def stats(self) -> dict:
        now = time.monotonic()
        t = list(self.times)
        fps = round((len(t) - 1) / (t[-1] - t[0]), 1) if len(t) >= 2 and t[-1] > t[0] and now - t[-1] < 2 else 0.0
        return {"fps": fps, "age_s": round(now - t[-1], 2) if t else None, "viewers": self.viewers}


class AutonomyWeb(Node):
    def __init__(self) -> None:
        super().__init__("tt_autonomy_web")
        for name, value in (("port", 8080), ("bind", "0.0.0.0"), ("map", ""), ("discovered", ""), ("people", ""),
                            ("camera_topic", "/tt/camera/image_raw"),
                            ("det_topic", "/tt/detections/image"), ("truth", False), ("width", 640), ("fps", 10.0),
                            ("quality", 75), ("need_yolo", False)):
            self.declare_parameter(name, value)
        g = lambda k: self.get_parameter(k).value  # noqa: E731
        self.width, self.period, self.quality = int(g("width")), 1.0 / float(g("fps")), int(g("quality"))
        self.truth_on = bool(g("truth"))
        self.need_yolo = bool(g("need_yolo"))   # true면 YOLO 결과가 와야 [자율비행 시작]을 켠다 (실기체 탐색, 11/03)
        self.map_data = self.load_map(g("map"))
        self.found_path = Path(g("discovered")) if g("discovered") else None
        self.found_mtime = None
        self.found: list = []
        self.people_path = Path(g("people")) if g("people") else None
        self.people_mtime = None
        self.people: list = []
        self.yolo_times: deque = deque(maxlen=20)   # /tt/detections 받은 때 (켜졌는지·초당 몇 번)
        self.yolo_last: dict = {}
        self.grid: dict | None = None               # /tt/obstacle_grid를 줄여 둔 것 (/api/grid)
        self.lock = threading.Lock()
        self.frames = {"cam": Frames(), "det": Frames()}
        self.pose = None
        self.truth = None
        self.trail: deque = deque(maxlen=4000)
        self.path: list = []
        self.mission = None
        self.mission_since = None
        self.flight = None
        self.flight_at = None
        self.tello = None
        self.tello_at = None
        self.events: deque = deque(maxlen=80)
        self.operator: dict | None = None   # 비행 노드가 알린 '찍은 자리' 상태 (/tt/operator_goal)
        self.operator_at = None
        self.operator_said_at = None        # 그 알림 글이 처음 온 때 (버튼 옆에 20초 보인다)
        self.note: dict | None = None       # 비행 노드가 알린 '지금 하는 일과 이유' (/tt/mission_note)
        self.note_at = None
        self._seen_result = None
        self._seen_autoland = None

        self.mission_pub = self.create_publisher(String, "/tt/mission_cmd", 10)
        self.flight_pub = self.create_publisher(String, "/tt/flight_cmd", 10)
        self.create_subscription(Image, g("camera_topic"), lambda m: self.on_image("cam", m), qos_profile_sensor_data)
        self.create_subscription(Image, g("det_topic"), lambda m: self.on_image("det", m), 2)
        self.create_subscription(String, "/tt/detections", self.on_detections, 10)
        self.create_subscription(OccupancyGrid, "/tt/obstacle_grid", self.on_grid, 1)
        self.create_subscription(Odometry, "/tt/pose_est", self.on_pose, 10)
        if self.truth_on:
            self.create_subscription(Odometry, "/tt/odometry_gt", self.on_truth, 10)
        self.create_subscription(PathMsg, "/tt/path", self.on_path, 1)
        self.create_subscription(String, "/tt/mission_state", self.on_mission_state, 10)
        self.create_subscription(String, "/tt/operator_goal", self.on_operator_goal, 10)
        self.create_subscription(String, "/tt/mission_note", self.on_mission_note, 10)
        self.create_subscription(String, "/tt/flight_status", self.on_flight_status, 10)
        self.create_subscription(String, "/tt/state", self.on_state, 10)

        handler = type("Handler", (WebHandler,), {"node": self})
        self.server = ThreadingHTTPServer((g("bind"), int(g("port"))), handler)
        self.server.daemon_threads = True
        threading.Thread(target=self.server.serve_forever, name="autonomy-web", daemon=True).start()
        self.event("웹 화면 시작")
        self.get_logger().info(f"자율비행 웹 화면: http://localhost:{g('port')} (지도 {g('map') or '없음'}, "
                               f"카메라 {g('camera_topic')}, 정답 위치 {'표시' if self.truth_on else '안 함'})")

    # ---------- 입력 ----------
    def load_map(self, path: str) -> dict | None:
        if not path:
            return None
        try:
            data = json.loads(Path(path).read_text(encoding="utf-8"))
        except (OSError, ValueError) as e:
            self.get_logger().error(f"지도를 읽지 못했습니다: {path} ({e})")
            return None
        keys = ("stage", "description", "bounds", "flight_height", "spawn", "goal", "walls", "obstacles", "open")
        return {k: data.get(k) for k in keys}

    def read_found(self) -> list:
        """비행 노드가 찾은(지도에 없던) 장애물. 파일이 바뀌었을 때만 다시 읽는다."""
        if self.found_path is None:
            return []
        try:
            mtime = self.found_path.stat().st_mtime
            if mtime != self.found_mtime:
                data = json.loads(self.found_path.read_text(encoding="utf-8"))
                self.found = [{k: o[k] for k in ("x", "y", "sx", "sy") if k in o} | {"sources": o.get("sources", [])}
                              for o in data.get("obstacles", [])]
                self.found_mtime = mtime
        except (OSError, ValueError):
            pass   # 비행 노드가 쓰는 중이면 다음에 다시 읽는다
        return self.found

    def read_people(self) -> list:
        """비행 노드가 찾은 사람. gone이면 그 자리를 다시 봤는데 없어 길을 막지 않는 사람."""
        if self.people_path is None:
            return []
        try:
            mtime = self.people_path.stat().st_mtime
            if mtime != self.people_mtime:
                data = json.loads(self.people_path.read_text(encoding="utf-8"))
                self.people = [{"x": q["x"], "y": q["y"], "hits": q.get("hits", 1), "gone": bool(q.get("gone"))}
                               for q in data.get("people", [])]
                self.people_mtime = mtime
        except (OSError, ValueError, KeyError):
            pass
        return self.people

    def on_detections(self, msg: String) -> None:
        """YOLO가 도는지 화면에 보이려고 센다 (2026-10-30: 영상 기본 탭이 '원본'이라 YOLO가 도는지 몰랐다)."""
        try:
            dets = json.loads(msg.data).get("detections", [])
        except ValueError:
            return
        names = [d.get("name") for d in dets]
        with self.lock:
            self.yolo_times.append(time.monotonic())
            self.yolo_last = {"people": names.count("person"), "objects": len(names) - names.count("person")}

    def on_grid(self, msg: OccupancyGrid) -> None:
        """카메라 지도 단면을 칸마다 한 바이트(0 모름, 1 본 빈 곳, 2 의심, 3 막힘)로 줄여 둔다."""
        a = np.asarray(msg.data, dtype=np.int8)
        code = np.zeros(a.shape, dtype=np.uint8)
        code[a == 0] = 1
        code[a == 50] = 2
        code[a == 100] = 3
        with self.lock:
            ver = (self.grid or {}).get("ver", 0) + 1
            self.grid = {"ver": ver, "res": msg.info.resolution, "w": msg.info.width, "h": msg.info.height,
                         "x0": msg.info.origin.position.x, "y0": msg.info.origin.position.y,
                         "cells": base64.b64encode(code.tobytes()).decode("ascii"),
                         "counts": {"blocked": int((code == 3).sum()), "suspect": int((code == 2).sum())}}

    def yolo_status(self) -> dict | None:
        with self.lock:
            t, last = list(self.yolo_times), dict(self.yolo_last)
        if not t:
            return None
        now = time.monotonic()
        hz = round((len(t) - 1) / (t[-1] - t[0]), 1) if len(t) >= 2 and t[-1] > t[0] else 0.0
        return {"age_s": round(now - t[-1], 2), "hz": hz if now - t[-1] < 2 else 0.0, **last}

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def event(self, text: str) -> None:
        with self.lock:
            self.events.append({"t": time.strftime("%H:%M:%S"), "text": text})

    def on_image(self, src: str, msg: Image) -> None:
        f = self.frames[src]
        now = time.monotonic()
        if f.viewers == 0 or now - f.last_put < self.period:   # 보는 사람이 없으면 JPEG로 바꾸지 않는다
            return
        f.last_put = now
        img = image_to_bgr(msg)
        if img is None:
            return
        if img.shape[1] > self.width:
            img = cv2.resize(img, (self.width, round(img.shape[0] * self.width / img.shape[1])),
                             interpolation=cv2.INTER_AREA)
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        if ok:
            f.put(buf.tobytes())

    def on_pose(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        var = max(msg.pose.covariance[0], msg.pose.covariance[7])
        pose = {"x": p.x, "y": p.y, "z": p.z, "yaw": yaw_of(msg.pose.pose.orientation),
                # 위치 추정 노드는 오차를 모르면(속도 적분만) −1을 보낸다: 0(정확)이 아니라 None(모름)으로 둔다 (11/04)
                "sigma": math.sqrt(var) if var > 0 else (0.0 if var == 0 else None), "t": stamp_of(msg)}
        with self.lock:
            self.pose = pose
            last = self.trail[-1] if self.trail else None
            if last is None or math.hypot(p.x - last[0], p.y - last[1]) > 0.05:
                self.trail.append((round(p.x, 3), round(p.y, 3)))

    def on_truth(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        with self.lock:
            self.truth = {"x": p.x, "y": p.y, "z": p.z, "yaw": yaw_of(msg.pose.pose.orientation)}

    def on_path(self, msg: PathMsg) -> None:
        with self.lock:
            self.path = [[round(ps.pose.position.x, 3), round(ps.pose.position.y, 3), round(ps.pose.position.z, 3)]
                         for ps in msg.poses]

    def on_mission_state(self, msg: String) -> None:
        if msg.data != self.mission:
            self.event(f"비행 단계: {self.mission or '-'} → {msg.data}")
            with self.lock:
                self.mission, self.mission_since = msg.data, time.monotonic()

    def on_operator_goal(self, msg: String) -> None:
        """비행 노드가 알린 찍은 자리 상태. 가는 중에는 1초마다 같은 것이 다시 오므로 (seq, t)가 바뀐 것만 기록에 적는다."""
        try:
            info = json.loads(msg.data)
        except ValueError:
            return
        if not isinstance(info, dict):
            return
        with self.lock:
            old = self.operator
            new = old is None or (old.get("seq"), old.get("t")) != (info.get("seq"), info.get("t"))
            self.operator, self.operator_at = info, time.monotonic()
            if new:
                self.operator_said_at = self.operator_at
        if new and info.get("text"):
            self.event(f"비행 노드: {info['text']}")

    def on_mission_note(self, msg: String) -> None:
        """비행 노드가 알린 지금 하는 일과 이유. 1초마다 오므로 돌아가는 이유·내리는 이유가 바뀔 때만 기록에 적는다."""
        try:
            info = json.loads(msg.data)
        except ValueError:
            return
        if not isinstance(info, dict):
            return
        with self.lock:
            old = self.note
            self.note, self.note_at = info, time.monotonic()
        for text in note_events(old, info):
            self.event(text)

    def on_flight_status(self, msg: String) -> None:
        info = json.loads(msg.data)
        result, auto = info.get("last_result"), info.get("last_auto_land")
        if result and result != self._seen_result:
            self._seen_result = result
            ok = result.get("ok")
            why = "" if ok else f" ({', '.join(result.get('failed') or []) or result.get('reason') or ''})"
            self.event(f"드라이버 {result.get('cmd')}: {'성공' if ok else '실패'}{why}")
        if auto and auto != self._seen_autoland:
            self._seen_autoland = auto
            self.event(f"자동 착륙: {auto.get('reason')}")
        with self.lock:
            self.flight, self.flight_at = info, time.monotonic()

    def on_state(self, msg: String) -> None:
        data = json.loads(msg.data)
        with self.lock:
            self.tello, self.tello_at = parse_tello(data["raw"]), time.monotonic()

    # ---------- 웹 API ----------
    def status(self) -> dict:
        now = time.monotonic()
        with self.lock:
            trail = list(self.trail)
            step = max(1, len(trail) // 600)
            tello = self.tello
            s = {
                "mission": {"state": self.mission,
                            "since_s": round(now - self.mission_since, 1) if self.mission_since else None},
                "pose": self.pose, "truth": self.truth, "path": self.path, "trail": trail[::step],
                "flight": self.flight,
                "flight_age_s": round(now - self.flight_at, 2) if self.flight_at else None,
                "tello": tello, "tello_age_s": round(now - self.tello_at, 2) if self.tello_at else None,
                "events": list(self.events)[-30:],
                "sim": self.truth_on, "need_yolo": self.need_yolo,
                # 찍은 자리: 비행 노드가 받아 기다리거나(pending) 가는 중(going)·도착해 둘러보는 중(arrived)일 때만 지도에 그린다.
                # 그동안 비행 노드가 1초마다 다시 알리므로, 3초 넘게 소식이 없으면(비행 노드가 꺼짐) 그리지 않는다
                "operator": self.operator if self.operator and self.operator.get("state") in ("pending", "going", "arrived")
                and self.operator_at is not None and now - self.operator_at < 3.0 else None,
                # 찍은 자리에 대한 비행 노드의 마지막 답 (받음·거절·그만둠·도착): [여기로 보내기] 옆에 잠깐 보인다
                "operator_note": {"text": self.operator.get("text"), "warn": bool(self.operator.get("warn")),
                                  "age_s": round(now - self.operator_said_at, 1)}
                if self.operator and self.operator.get("text") and self.operator_said_at is not None else None,
                # 지금 하는 일과 이유: 비행 노드가 1초마다 알린다. 3초 넘게 소식이 없으면(비행 노드가 꺼짐) 내린 이유만 남긴다
                "note": None if self.note is None else self.note if now - self.note_at < 3.0
                else {"state": self.note.get("state"), "why_end": self.note.get("why_end"), "stale": True},
            }
        s["camera"] = {k: f.stats() for k, f in self.frames.items()}
        s["found"] = self.read_found()
        s["people"] = self.read_people()
        s["yolo"] = self.yolo_status()
        with self.lock:
            g = self.grid
        s["grid"] = {"ver": g["ver"], **g["counts"]} if g else None
        s["warnings"] = self.warnings(s)
        return s

    def warnings(self, s: dict) -> list[str]:
        out = []
        t = s["tello"]
        if t:
            th, bat = t.get("temph"), t.get("bat")
            if isinstance(th, float) and th >= TEMP_DANGER_C:
                out.append(f"기체 온도 {th:.0f}°C: 곧 과열로 꺼질 수 있습니다. 착륙하고 식히세요.")
            elif isinstance(th, float) and th >= TEMP_WARN_C:
                out.append(f"기체 온도 {th:.0f}°C: 지상에서 오래 켜 두면 과열됩니다.")
            if isinstance(bat, float) and bat < BATTERY_WARN_PCT:
                out.append(f"배터리 {bat:.0f}%: 10% 이하가 되면 그 자리에 착륙합니다. 다음 비행은 완충 배터리로.")
        if s["tello_age_s"] is not None and s["tello_age_s"] > STATE_STALE_S:
            out.append(f"Tello 상태가 {s['tello_age_s']:.1f}초째 끊겼습니다.")
        if s["pose"] and s["pose"]["sigma"] is not None and s["pose"]["sigma"] > SIGMA_WARN_M:
            out.append(f"위치 불확실성이 큽니다 (σ {s['pose']['sigma']:.2f} m).")
        return out

    def command(self, name: str, body: dict) -> dict:
        if name == "mission/start":
            self.mission_pub.publish(String(data="start"))
            self.event("웹: 자율비행 시작 요청")
        elif name == "mission/land":
            self.mission_pub.publish(String(data="land"))
            with self.lock:
                mission, flying = self.mission, bool(self.flight and self.flight.get("flying"))
            if flying and mission in (None, "DONE", "ABORT"):   # 비행 노드가 없거나 끝났는데 떠 있으면 드라이버에 바로
                self.flight_pub.publish(String(data="land"))
            self.event("웹: 착륙 요청")
        elif name == "mission/goto":
            cmd = goto_command(body)
            if cmd is None:
                return {"ok": False, "reason": "bad_xy"}
            self.mission_pub.publish(String(data=cmd))
            self.event(f"웹: 지도에서 찍은 자리 ({body['x']:.2f}, {body['y']:.2f})로 보내기")
        elif name == "mission/goto_cancel":
            self.mission_pub.publish(String(data="goto cancel"))
            self.event("웹: 찍은 자리 취소")
        elif name == "flight/emergency":
            for _ in range(2):
                self.flight_pub.publish(String(data="emergency"))
            self.event("웹: 비상 정지")
        elif name == "flight/confirm":
            on = bool(body.get("confirmed"))
            self.flight_pub.publish(String(data="confirm" if on else "unconfirm"))
            self.event(f"웹: 운용자 확인 {'켬' if on else '끔'}")
        else:
            return {"ok": False, "reason": "unknown"}
        return {"ok": True}

    def close(self) -> None:
        self.server.shutdown()
        for f in self.frames.values():
            with f.cond:
                f.cond.notify_all()


class WebHandler(BaseHTTPRequestHandler):
    node: AutonomyWeb

    def log_message(self, *_):   # 요청마다 찍지 않는다
        pass

    def send_json(self, data, code: int = 200) -> None:
        body = json.dumps(data, ensure_ascii=False, default=str).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:
        url = urlparse(self.path)
        if url.path in ("/", "/index.html"):
            body = (STATIC / "index.html").read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif url.path == "/api/status":
            self.send_json(self.node.status())
        elif url.path == "/api/map":
            self.send_json(self.node.map_data)
        elif url.path == "/api/grid":
            with self.node.lock:
                grid = self.node.grid
            self.send_json(grid)
        elif url.path == "/video.mjpg":
            self.stream(parse_qs(url.query).get("src", ["cam"])[0])
        else:
            self.send_error(404)

    def do_POST(self) -> None:
        url = urlparse(self.path)
        length = int(self.headers.get("Content-Length") or 0)
        try:
            body = json.loads(self.rfile.read(length) or b"{}") if length else {}
        except ValueError:
            body = {}
        if not url.path.startswith("/api/"):
            self.send_error(404)
            return
        self.send_json(self.node.command(url.path[len("/api/"):], body))

    def stream(self, src: str) -> None:
        frames = self.node.frames.get(src)
        if frames is None:
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        with frames.cond:
            frames.viewers += 1
        seq = -1
        try:
            while rclpy.ok():
                seq, jpeg = frames.wait(seq, 1.0)
                if jpeg is None:
                    continue
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(jpeg)).encode()
                                 + b"\r\n\r\n" + jpeg + b"\r\n")
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass   # 브라우저가 끊으면 이 스레드도 끝낸다
        finally:
            with frames.cond:
                frames.viewers -= 1


def main() -> None:
    rclpy.init()
    node = AutonomyWeb()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
