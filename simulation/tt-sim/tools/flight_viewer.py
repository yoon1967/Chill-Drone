#!/usr/bin/env python3
"""비행 기록 한 폴더를 한 페이지(viewer.html)로: 영상·깊이·지도 위 경로·시계열 그래프·사건·노드 로그를 같은 시각에 맞춰 본다.

  ~/venvs/tt/bin/python tools/flight_viewer.py                  # 가장 최근 비행
  ~/venvs/tt/bin/python tools/flight_viewer.py <out_dir> ...    # 그 비행들
  ~/venvs/tt/bin/python tools/flight_viewer.py --all            # 페이지가 없는 비행 전부
  ... --no-video   영상을 다시 만들지 않음 (있던 viewer/video.mp4는 그대로 씀)

real.launch는 launch가 꺼지면(Ctrl+C) 이 도구를 알아서 돌린다(viewer:=false로 끔, 출력은 기록 폴더의 viewer.log).
영상은 PyAV(~/venvs/tt), 깊이 영상·노드 로그(bag)는 ROS 환경(rosbag2_py)이 있어야 넣는다. 없으면 그 칸만 빈다.

나오는 것 (기록 폴더 안)
  viewer.html          Windows에서 \\\\wsl.localhost\\Ubuntu-24.04\\home\\ssafy\\ttrepo\\data\\outputs\\flights\\<비행>\\viewer.html
                       을 브라우저로 연다. 인터넷이 필요 없다 (드론 Wi-Fi에 붙은 채로 볼 수 있다)
  viewer/video.mp4     받은 영상(video.h264)을 폭 480 H.264로 (프레임 순서 그대로 24 fps → 페이지가 프레임마다 받은 시각을 안다)
  viewer/depth/*.jpg   bag의 깊이 영상 /tt/depth (가까움 빨강 → 멂 파랑, 초당 5장까지)
  (영상 위 상자)       bag의 YOLO 결과 /tt/detections를 같은 시각의 영상 위에 겹쳐 그린다
  viewer/summary.json  목록 페이지에 쓰는 요약
그리고 기록 폴더들이 모인 flights/에 index.html(모든 비행 목록)을 새로 쓴다.

시각: 모든 기록은 같은 WSL 시계다. 영상만 '받아서 푼 시각'이라 실제보다 늦다(Tello 영상 지연, 기본 0.3 s).
페이지에서 영상 지연을 바꾸면 영상과 다른 기록이 바로 다시 맞춰진다.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import re
import sys
import time
from datetime import datetime
from fractions import Fraction
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parents[1]
sys.path.insert(0, str(ROOT))
from autonomy.tello_state import describe_landing  # noqa: E402
FLIGHTS = REPO / "data" / "outputs" / "flights"
FPS = 24
VIDEO_WIDTH = 480
FLOG_HZ = 10           # 비행 기록은 이 빈도로 줄여 넣는다 (단계가 바뀌는 줄은 모두 남김)
DEPTH_HZ = 5
TOF_HALF = math.radians(12.5)
HFOV = math.radians(82.6)
LEVELS = {10: "DEBUG", 20: "INFO", 30: "WARN", 40: "ERROR", 50: "FATAL"}


def iso(t: str) -> float:
    return datetime.fromisoformat(t).timestamp()


def jsonl(p: Path) -> list[dict]:
    out = []
    if p.exists():
        for line in p.open(encoding="utf-8"):
            try:
                out.append(json.loads(line))
            except json.JSONDecodeError:
                pass
    return out


def num(v):
    try:
        f = float(v)
    except (TypeError, ValueError):
        return None
    return f if math.isfinite(f) else None


def r3(v):
    return None if v is None else round(v, 3)


# ---------------------------------------------------------------- 기록 읽기
def load_tello(d: Path) -> dict:
    """tello_log/*/: 보낸 명령·응답, 상태값, 사건. 세션이 여러 개면 모두 잇는다."""
    tof, rc, events, sensor = [], [], [], []
    for sess in sorted((d / "tello_log").glob("*/")):
        for c in jsonl(sess / "commands.jsonl"):
            cmd = str(c.get("cmd", ""))
            t = iso(c.get("sent_at") or c.get("recorded_at"))
            resp = str(c.get("response") or "")
            if cmd.startswith("EXT") and resp.startswith("tof"):
                try:
                    mm = int(resp.split()[1])
                except (IndexError, ValueError):
                    continue
                tof.append([round(iso(c.get("recorded_at") or c["sent_at"]), 3), None if mm >= 8000 else mm / 1000.0])
            elif cmd.startswith("rc "):
                try:
                    a, b, cc, dd = (int(x) for x in cmd.split()[1:5])
                except ValueError:
                    continue
                rc.append([round(t, 3), a, b, cc, dd])
            elif cmd not in ("command",) and not cmd.startswith("EXT"):
                ok = bool(c.get("ok"))
                got = resp or c.get("reason") or ""
                src = c.get("source") or ""
                events.append({"t": round(t, 3), "kind": "명령", "lvl": "info" if ok else "warn",
                               "text": f"{cmd} → {got}" + (f" ({src})" if src and src != "driver" else "")})
        for e in jsonl(sess / "events.jsonl"):
            det = e.get("detail") or {}
            short = ", ".join(f"{k} {v}" for k, v in list(det.items())[:3]) if isinstance(det, dict) else str(det)
            typ = str(e.get("type", ""))
            lvl = "warn" if any(w in typ for w in ("land", "lost", "error", "emergency", "dropped", "sink")) else "info"
            events.append({"t": round(iso(e["t"]), 3), "kind": "드라이버", "lvl": lvl, "text": f"{typ} {short}".strip()})
        for s in jsonl(sess / "sensor.jsonl"):
            raw = s.get("raw") or {}
            if not isinstance(raw, dict):
                continue
            g = lambda k: num(raw.get(k))  # noqa: E731
            vgx, vgy, vgz = g("vgx"), g("vgy"), g("vgz")
            down = g("tof")
            sensor.append([round(iso(s["received_at"]), 3),
                           r3(down / 100.0) if down is not None and down < 1000 else None,   # 하방 ToF (m). 6553 등은 못 잼
                           r3(g("h") / 100.0) if g("h") is not None else None,
                           g("bat"), g("temph"),
                           r3(vgx / 10.0) if vgx is not None else None,                # dm/s → m/s
                           r3(vgy / 10.0) if vgy is not None else None,
                           r3(vgz / 10.0) if vgz is not None else None,
                           g("yaw"), g("pitch"), g("roll")])
    tof.sort(key=lambda r: r[0])
    rc.sort(key=lambda r: r[0])
    sensor.sort(key=lambda r: r[0])
    return {"tof": tof, "rc": rc, "events": events, "sensor": sensor}


FLOG_COLS = ["x", "y", "z", "yaw", "sigma", "front_tof", "cam_ahead", "lane_free", "near_hit", "cmd_vx", "cmd_vy", "cmd_wz",
             "soft_target", "soft_src"]


def load_flight_log(d: Path) -> dict:
    p = d / "flight_log.csv"
    if not p.exists():
        return {"cols": [], "rows": []}
    rows = list(csv.DictReader(p.open(encoding="utf-8")))
    cols = [c for c in FLOG_COLS if rows and c in rows[0]]
    text_cols = {"soft_src"}
    out, last_t, last_state = [], -1e18, None
    for r in rows:
        t = num(r.get("t"))
        if t is None:
            continue
        st = r.get("state", "")
        if st == last_state and t - last_t < 1.0 / FLOG_HZ:
            continue
        last_t, last_state = t, st
        vals = [(r.get(c) or "") if c in text_cols else r3(num(r.get(c))) for c in cols]
        out.append([round(t, 3), st, *vals])
    return {"cols": cols, "rows": out}


def load_frames(d: Path) -> list[float]:
    p = d / "video" / "video_frames.csv"
    if not p.exists():
        return []
    return [float(r["wall_time"]) for r in csv.DictReader(p.open(encoding="utf-8"))]


def encode_video(d: Path, out: Path, n_frames: int) -> int:
    """video.h264 → H.264 mp4, 프레임 i의 pts = i (1/24 s). 만든 프레임 수."""
    import av

    out.parent.mkdir(parents=True, exist_ok=True)
    tmp = out.with_suffix(".part.mp4")
    n = 0
    with av.open(str(d / "video" / "video.h264"), format="h264") as src, \
            av.open(str(tmp), "w", options={"movflags": "+faststart"}) as dst:
        st = None
        for fr in src.decode(video=0):
            if n >= n_frames:
                break
            if st is None:
                st = dst.add_stream("libx264", rate=FPS, options={"crf": "27", "preset": "veryfast"})
                st.width = VIDEO_WIDTH
                st.height = int(round(VIDEO_WIDTH * fr.height / fr.width / 2)) * 2
                st.pix_fmt = "yuv420p"
                st.codec_context.time_base = Fraction(1, FPS)
            img = fr.reformat(width=st.width, height=st.height, format="yuv420p")
            img.pts = n
            img.time_base = Fraction(1, FPS)
            for pkt in st.encode(img):
                dst.mux(pkt)
            n += 1
        if st is not None:
            for pkt in st.encode():
                dst.mux(pkt)
    tmp.replace(out)
    return n


def turbo(x):
    """0(가까움)–1(멂) → 빨강·노랑·초록·파랑 (numpy 배열)."""
    import numpy as np

    stops = np.array([[0.85, 0.1, 0.1], [0.98, 0.75, 0.1], [0.3, 0.8, 0.35], [0.15, 0.45, 0.9], [0.1, 0.12, 0.45]])
    pos = np.clip(x, 0, 1) * (len(stops) - 1)
    i = np.minimum(pos.astype(int), len(stops) - 2)
    f = (pos - i)[..., None]
    return stops[i] * (1 - f) + stops[i + 1] * f


def read_bag(d: Path, out: Path, want_depth: bool) -> tuple[list, list, str, dict]:
    """bag에서 깊이 영상(→ jpg), /rosout(노드 로그), YOLO 결과 /tt/detections.
    (깊이 [(t, 파일, 가운데 거리)], 로그, 안내, YOLO {"n": 결과 수, "t": [처음, 끝], "dets": [[t, 폭, 높이, [[x1, y1, x2, y2, 이름, 신뢰도]..]]..]})"""
    yolo = {"n": 0, "t": None, "dets": []}
    bag = d / "bag"
    if not bag.exists():
        return [], [], "bag 없음", yolo
    if not (bag / "metadata.yaml").exists() and any(bag.glob("*.mcap")):
        # 창을 닫는 등으로 bag이 안 닫히면 목차(metadata.yaml)가 없다. mcap 파일에서 다시 만든다
        import subprocess
        try:
            subprocess.run(["ros2", "bag", "reindex", str(bag), "-s", "mcap"], capture_output=True, timeout=300)
        except (OSError, subprocess.TimeoutExpired):
            pass
    if not (bag / "metadata.yaml").exists():
        return [], [], "bag이 제대로 닫히지 않음 (ros2 bag reindex 실패)", yolo
    try:
        import numpy as np
        import rosbag2_py
        from PIL import Image as PImage
        from rclpy.serialization import deserialize_message
        from rcl_interfaces.msg import Log
        from sensor_msgs.msg import Image
        from std_msgs.msg import String
    except ImportError as e:
        return [], [], f"ROS 환경이 없어 bag을 못 읽음 ({e.name}) — source /opt/ros/jazzy/setup.bash 뒤 실행", yolo
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
    topics = {t.name for t in reader.get_all_topics_and_types()}
    want = [t for t in ("/tt/depth", "/rosout", "/tt/detections") if t in topics and (t != "/tt/depth" or want_depth)]
    if not want:
        return [], [], "bag에 /tt/depth·/rosout 없음", yolo
    reader.set_filter(rosbag2_py.StorageFilter(topics=want))
    depth, logs, last = [], [], -1e18
    ddir = out / "depth"
    if want_depth and "/tt/depth" in want:
        ddir.mkdir(parents=True, exist_ok=True)
        for old in ddir.glob("*.jpg"):
            old.unlink()
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == "/rosout":
            m = deserialize_message(data, Log)
            t = m.stamp.sec + m.stamp.nanosec * 1e-9
            logs.append([round(t, 3), LEVELS.get(int(m.level), str(m.level)), m.name, m.msg])
            continue
        if topic == "/tt/detections":
            try:
                p = json.loads(deserialize_message(data, String).data)
            except ValueError:
                continue
            t = float(p.get("stamp", 0.0))
            yolo["n"] += 1
            yolo["t"] = [t, t] if yolo["t"] is None else [min(yolo["t"][0], t), max(yolo["t"][1], t)]
            boxes = [[int(q["x1"]), int(q["y1"]), int(q["x2"]), int(q["y2"]), q.get("name", "?"), round(float(q.get("conf", 0)), 2)]
                     for q in p.get("detections", []) if "x1" in q]
            if boxes:
                yolo["dets"].append([round(t, 3), int(p.get("width", 960)), int(p.get("height", 720)), boxes])
            continue
        m = deserialize_message(data, Image)
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if t - last < 1.0 / DEPTH_HZ - 0.02 or m.encoding not in ("32FC1", ""):
            continue
        last = t
        dep = np.frombuffer(m.data, dtype=np.float32).reshape(m.height, m.width)
        ok = np.isfinite(dep) & (dep > 0.05)
        rgb = turbo((np.where(ok, dep, 4.0) - 0.3) / 3.7)
        rgb[~ok] = 0.15
        img = PImage.fromarray((rgb * 255).astype("uint8"))
        img = img.resize((160, int(160 * m.height / m.width)))
        name = f"{len(depth):05d}.jpg"
        img.save(ddir / name, quality=80)
        f = (m.width / 2) / math.tan(HFOV / 2)
        r = max(2, int(f * math.tan(TOF_HALF)))
        patch = dep[m.height // 2 - r:m.height // 2 + r, m.width // 2 - r:m.width // 2 + r]
        patch = patch[np.isfinite(patch) & (patch > 0.05)]
        depth.append([round(t, 3), name, r3(float(np.percentile(patch, 5))) if patch.size >= 10 else None])
    yolo["dets"].sort(key=lambda r: r[0])
    return depth, logs, "", yolo


LOG_RE = re.compile(r"\[?(DEBUG|INFO|WARN|WARNING|ERROR|FATAL)\]?\s*\[?(\d{9,11}\.\d+)\]?\s*\[([^\]]+)\]:\s?(.*)")
LOG_RE2 = re.compile(r"(\d{9,11}\.\d+)\s*\[?(DEBUG|INFO|WARN|WARNING|ERROR|FATAL)\]?\s*\[([^\]]+)\]:\s?(.*)")


def read_ros_log(d: Path) -> list:
    """ros_log/의 노드 로그 파일과 launch.log에서 '[수준] [시각] [노드]: 내용' 줄."""
    out, seen = [], set()
    for f in sorted((d / "ros_log").glob("*.log")):
        for line in f.open(encoding="utf-8", errors="replace"):
            m = LOG_RE.search(line)
            if m:
                lvl, t, node, msg = m.groups()
            else:
                m = LOG_RE2.search(line)
                if not m:
                    continue
                t, lvl, node, msg = m.groups()
            key = (t[:14], node, msg[:80])
            if key in seen:
                continue
            seen.add(key)
            out.append([round(float(t), 3), "WARN" if lvl == "WARNING" else lvl, node, msg.rstrip()])
    return out


def load_map(d: Path) -> dict:
    def boxes(items):
        return [[r3(o["x"]), r3(o["y"]), r3(o.get("sx", 0.2)), r3(o.get("sy", 0.2))] for o in items or []
                if "x" in o and "y" in o]

    m = {}
    if (d / "map.json").exists():
        mp = json.loads((d / "map.json").read_text(encoding="utf-8"))
        b = mp.get("bounds") or {}
        m = {"bounds": [b.get("xmin", 0), b.get("ymin", 0), b.get("xmax", 1), b.get("ymax", 1)],
             "walls": boxes(mp.get("walls")), "obstacles": boxes(mp.get("obstacles")),
             "spawn": mp.get("spawn"), "goal": mp.get("goal"), "flight_height": mp.get("flight_height"),
             "open": bool(mp.get("open"))}
        # 실습실 가구 어림 지도는 더 깔지 않는다 (11/2 사용자 요청: 드론이 모르는 가구를 그려 두면 지도와 헷갈린다)
    if (d / "discovered.json").exists():
        try:
            m["discovered"] = boxes(json.loads((d / "discovered.json").read_text(encoding="utf-8")).get("obstacles"))
        except (json.JSONDecodeError, AttributeError):
            pass
    if (d / "people.json").exists():
        try:
            pj = json.loads((d / "people.json").read_text(encoding="utf-8"))
            items = pj.get("people", pj) if isinstance(pj, dict) else pj
            m["people"] = [[r3(p["x"]), r3(p["y"]), bool(p.get("gone"))] for p in items if isinstance(p, dict) and "x" in p]
            m["explored_m2"] = pj.get("explored_m2") if isinstance(pj, dict) else None
        except (json.JSONDecodeError, TypeError):
            pass
    if (d / "camera_map.npz").exists():
        try:
            import numpy as np

            z = np.load(d / "camera_map.npz")
            lo, org, res = z["logodds"], z["origin"], float(z["res"])
            h = float(m.get("flight_height") or 1.0)
            k0, k1 = max(0, int((h - 0.35 - org[2]) / res)), min(lo.shape[2], int((h + 0.35 - org[2]) / res) + 1)
            top = lo[:, :, k0:k1].max(axis=2)
            cell = lambda mask: [[r3(org[0] + (i + 0.5) * res), r3(org[1] + (j + 0.5) * res)]  # noqa: E731
                                 for i, j in zip(*(a.tolist() for a in np.nonzero(mask)))]
            # 막힘(확정 0.5 넘음)과 의심(0.3–0.5: 경로 계획은 피하지만 위치 맞추기에는 안 씀)
            m["camera_cells"] = {"res": res, "xy": cell(top > 0.5), "sus": cell((top > 0.3) & (top <= 0.5))}
        except Exception as e:  # noqa: BLE001 — 지도 형식이 바뀌어도 페이지는 만든다
            m["camera_cells_error"] = str(e)
    return m


# ---------------------------------------------------------------- 요약
def coverage_stats(flog: dict, logs: list, w0: float, w1: float, explored_m2=None) -> dict | None:
    """같은 곳을 다시 갔는지 (10/30 저녁 목표 '찍은 곳은 다시 안 찍기'의 잣대).

    날아간 길(m), 지나온 0.5 m 칸 수, 15초 넘게 떠났다 다시 온 칸 수, 고른 볼 자리 수와 그중 0.3 m 안까지 닿은 수,
    새로 본 넓이(people.json의 explored_m2) ÷ 날아간 길.
    """
    cols = flog.get("cols") or []
    if "x" not in cols or not flog.get("rows"):
        return None
    ix, iy = cols.index("x") + 2, cols.index("y") + 2
    moving = ("FOLLOW", "REPLAN", "LOOK", "PLAN", "BACKOFF", "HOLD", "UNSTICK", "EXIT", "WANDER", "TURN", "GAP")
    rows = [r for r in flog["rows"] if w0 <= r[0] <= w1 and r[1] in moving and r[ix] is not None and r[iy] is not None]
    if len(rows) < 2:
        return None
    path = sum(math.hypot(b[ix] - a[ix], b[iy] - a[iy]) for a, b in zip(rows, rows[1:]))
    cells: dict = {}
    for r in rows:
        key = (math.floor(r[ix] / 0.5), math.floor(r[iy] / 0.5))
        visits = cells.setdefault(key, [])
        if not visits or r[0] - visits[-1][1] > 15.0:
            visits.append([r[0], r[0]])
        else:
            visits[-1][1] = r[0]
    revisited = sum(1 for v in cells.values() if len(v) >= 2)
    spots = []
    for t, _, _, msg in logs:
        if msg.startswith("다음 볼 자리") and w0 <= t <= w1:
            m = re.search(r"\(([-\d.]+), ([-\d.]+)\)", msg)
            if m:
                spots.append((t, float(m.group(1)), float(m.group(2))))
    reached = 0
    for i, (t, sx, sy) in enumerate(spots):
        t_end = spots[i + 1][0] if i + 1 < len(spots) else w1
        if any(math.hypot(r[ix] - sx, r[iy] - sy) < 0.3 for r in rows if t <= r[0] <= t_end):
            reached += 1
    return {"path_m": r3(path), "cells": len(cells), "revisited": revisited,
            "revisit_ratio": r3(revisited / len(cells)) if cells else None,
            "vp_chosen": len(spots), "vp_reached": reached,
            "area_per_m": r3(explored_m2 / path) if explored_m2 and path > 1.0 else None}


def summarize(d: Path, run: dict, tello: dict, flog: dict, frames: list, depth: list) -> dict:
    ev = tello["events"]
    tk = [e for e in sorted(ev, key=lambda e: e["t"]) if e["text"].startswith("takeoff")]
    takeoff = next((e["t"] for e in tk if e["lvl"] == "info"), tk[0]["t"] if tk else None)
    sensor = tello["sensor"]
    airborne = [s[0] for s in sensor if s[1] is not None and s[1] > 0.3]
    t_air_end = airborne[-1] if airborne else None
    times = [x for x in (flog["rows"][0][0] if flog["rows"] else None, sensor[0][0] if sensor else None,
                         frames[0] if frames else None) if x is not None]
    ends = [x for x in (flog["rows"][-1][0] if flog["rows"] else None, sensor[-1][0] if sensor else None,
                        frames[-1] if frames else None) if x is not None]
    t0 = min(times) if times else (ev[0]["t"] if ev else 0.0)
    t1 = max(ends) if ends else t0 + 1
    # 볼 구간: 이륙 5초 전 ~ 마지막으로 떠 있던 때 10초 뒤
    w0 = max(t0, takeoff - 5) if takeoff else t0
    w1 = min(t1, t_air_end + 10) if t_air_end and t_air_end > w0 else t1
    fly = [s for s in sensor if w0 <= s[0] <= w1]
    tof_air = [v for t, v in tello["tof"] if v is not None and (not airborne or airborne[0] <= t <= airborne[-1])]
    lands = [e for e in ev if e["text"].startswith("land")]
    land_fail = [e for e in lands if e["lvl"] != "info"]
    speeds = [math.hypot(s[5], s[6]) for s in fly if s[5] is not None and s[6] is not None]
    states = {}
    rows = flog["rows"]
    for a, b in zip(rows, rows[1:]):
        if w0 <= a[0] <= w1:
            states[a[1]] = states.get(a[1], 0.0) + min(b[0] - a[0], 1.0)
    args = run.get("args", {})
    warn = []
    if land_fail:
        warn.append(f"착륙 명령 실패 {len(land_fail)}번")
    near = sum(1 for v in tof_air if v < 0.35)
    if near:
        warn.append(f"정면 ToF 0.35 m 안 {near}번")
    auto = [e for e in ev if "auto_land" in e["text"]]
    if auto:
        reasons = ", ".join(describe_landing(e["text"].split("reason", 1)[-1].strip()) for e in auto)
        warn.append(f"드라이버 자동 착륙 {len(auto)}번 ({reasons})")
    sinks = [e for e in ev if e["text"].startswith("sink_against_climb")]
    if sinks:
        warn.append(f"상승 입력 중 하강 {len(sinks)}번 (11/09 전 기록은 밑에 가구·바닥 턱이 들어온 것도 셈)")
    stalls = [e for e in ev if e["text"].startswith("climb_stalled")]
    if stalls:
        warn.append(f"천장에 눌림 {len(stalls)}번 (상승 입력에도 Tello 높이가 안 오름)")
    if not frames:
        warn.append("영상 없음")
    flight_s = (t_air_end - takeoff) if takeoff and t_air_end and t_air_end > takeoff else None
    return {
        "name": d.name, "started": run.get("started") or time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(t0)),
        "mode": args.get("mode", "route") if run else None, "speed": args.get("speed"), "vision": args.get("vision"),
        "doors": args.get("doors"), "search_time": args.get("search_time"), "map": Path(run.get("map", "")).name,
        "code": (run.get("code") or {}).get("sha1"),
        "t0": round(t0, 3), "t1": round(t1, 3), "w0": round(w0, 3), "w1": round(w1, 3),
        "takeoff": takeoff, "flight_s": r3(flight_s),
        "min_tof": r3(min(tof_air)) if tof_air else None,
        "max_speed": r3(max(speeds)) if speeds else None,
        "battery": [fly[0][3], fly[-1][3]] if fly else None,
        "temp_max": max((s[4] for s in fly if s[4] is not None), default=None),
        "frames": len(frames), "depth_frames": len(depth),
        "depth_hz": r3(len(depth) / (depth[-1][0] - depth[0][0])) if len(depth) > 1 and depth[-1][0] > depth[0][0] else None,
        "states": {k: round(v, 1) for k, v in sorted(states.items(), key=lambda kv: -kv[1])},
        "warn": warn,
    }


# ---------------------------------------------------------------- 만들기
def build(d: Path, video: bool = True) -> dict:
    d = d.expanduser().resolve()
    out = d / "viewer"
    out.mkdir(exist_ok=True)
    notes = []
    run = json.loads((d / "run.json").read_text(encoding="utf-8")) if (d / "run.json").exists() else {}
    tello = load_tello(d)
    flog = load_flight_log(d)
    frames = load_frames(d)
    mp4 = out / "video.mp4"
    if frames and (d / "video" / "video.h264").exists():
        if video or not mp4.exists():
            try:
                n = encode_video(d, mp4, len(frames))
                frames = frames[:n]
            except ImportError:
                notes.append("영상: PyAV가 없어 못 만듦 (~/venvs/tt/bin/python으로 실행)")
                frames = []
            except Exception as e:  # noqa: BLE001 — 영상이 깨져도 나머지는 만든다
                notes.append(f"영상: 만들다 멈춤 ({type(e).__name__}: {e})")
                frames = frames if mp4.exists() else []
    else:
        frames = []
    depth, logs, bag_note, yolo = read_bag(d, out, want_depth=True)
    if bag_note:
        notes.append(bag_note)
    if not logs:
        logs = read_ros_log(d)
    logs.sort(key=lambda r: r[0])
    mp = load_map(d)
    if mp.get("open") and flog["rows"] and "x" in flog["cols"]:
        # 빈 지도(map:= 없이, 20 m)는 날아간 곳·본 사람·카메라 지도 둘레만 크게 보인다
        ix, iy = flog["cols"].index("x") + 2, flog["cols"].index("y") + 2
        pts = [(r[ix], r[iy]) for r in flog["rows"] if r[ix] is not None and r[iy] is not None]
        pts += [(q[0], q[1]) for q in mp.get("people", [])] + [tuple(q) for q in (mp.get("camera_cells") or {}).get("xy", [])]
        if pts:
            xs, ys = [q[0] for q in pts], [q[1] for q in pts]
            mp["bounds"] = [r3(min(xs) - 1.0), r3(min(ys) - 1.0), r3(max(xs) + 1.0), r3(max(ys) + 1.0)]
    summary = summarize(d, run, tello, flog, frames, depth)
    summary["notes"] = notes
    summary["coverage"] = coverage_stats(flog, logs, summary["w0"], summary["w1"], mp.get("explored_m2"))
    span = yolo["t"][1] - yolo["t"][0] if yolo["t"] else 0
    summary["yolo"] = {"msgs": yolo["n"], "hz": r3(yolo["n"] / span) if span > 0 else None,
                       "person_frames": sum(1 for r in yolo["dets"] if any(b[4] == "person" for b in r[3]))} if yolo["n"] else None
    data = {"summary": summary, "args": run.get("args", {}), "map": mp, "flog": flog,
            "tof": tello["tof"], "rc": tello["rc"], "sensor": tello["sensor"],
            "events": sorted(tello["events"], key=lambda e: e["t"]), "logs": logs[:30000],
            "frames": [round(f, 3) for f in frames], "fps": FPS, "depth": depth, "dets": yolo["dets"],
            "lag": 0.3}
    blob = json.dumps(data, ensure_ascii=False, separators=(",", ":")).replace("</", "<\\/")
    html = PAGE.replace("/*DATA*/null", blob).replace("{{TITLE}}", f"비행 기록 {d.name}")
    (d / "viewer.html").write_text(html, encoding="utf-8")
    (out / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=1), encoding="utf-8")
    return summary


def write_index(root: Path) -> Path | None:
    if not root.is_dir():
        return None
    items = []
    for f in sorted((p for p in root.iterdir() if p.is_dir()), reverse=True):
        s = {}
        if (f / "viewer" / "summary.json").exists():
            s = json.loads((f / "viewer" / "summary.json").read_text(encoding="utf-8"))
        elif (f / "run.json").exists():
            r = json.loads((f / "run.json").read_text(encoding="utf-8"))
            a = r.get("args", {})
            s = {"started": r.get("started"), "mode": a.get("mode"), "speed": a.get("speed"), "vision": a.get("vision")}
        elif not (f / "tello_log").exists():
            continue
        s["name"] = f.name
        s["page"] = (f / "viewer.html").exists()
        items.append(s)
    blob = json.dumps(items, ensure_ascii=False).replace("</", "<\\/")
    (root / "index.html").write_text(INDEX.replace("/*DATA*/null", blob), encoding="utf-8")
    return root / "index.html"


def main() -> int:
    ap = argparse.ArgumentParser(description="비행 기록을 한 페이지로 (viewer.html)")
    ap.add_argument("out_dir", type=Path, nargs="*")
    ap.add_argument("--all", action="store_true", help="flights/ 안에서 페이지가 없는 비행 전부")
    ap.add_argument("--no-video", action="store_true", help="영상을 다시 만들지 않는다")
    args = ap.parse_args()
    dirs = list(args.out_dir)
    if args.all:
        dirs += [p for p in sorted(FLIGHTS.iterdir()) if p.is_dir() and (p / "tello_log").exists()
                 and not (p / "viewer.html").exists()]
    if not dirs:
        cands = sorted(p for p in FLIGHTS.iterdir() if p.is_dir()) if FLIGHTS.exists() else []
        if not cands:
            print(f"비행 기록이 없습니다: {FLIGHTS}")
            return 1
        dirs = [cands[-1]]
    roots = set()
    for d in dirs:
        t = time.time()
        s = build(d, video=not args.no_video)
        print(f"{d.name}: 영상 {s['frames']}프레임, 깊이 {s['depth_frames']}장, 비행 {s['flight_s'] or '-'} s, "
              f"최소 ToF {s['min_tof'] or '-'} m, {time.time() - t:.0f}초" + (f" — {'; '.join(s['notes'])}" if s["notes"] else ""))
        print(f"  → {d.expanduser().resolve() / 'viewer.html'}")
        roots.add(d.expanduser().resolve().parent)
    for r in roots:
        if r.name == "flights":
            print(f"목록 → {write_index(r)}")
    return 0


PAGE = r"""<!doctype html>
<html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>{{TITLE}}</title>
<style>
:root{--bg:#f6f7f9;--panel:#fff;--ink:#1d232b;--muted:#66707c;--line:#dde1e6;--grid:#eceff2;--accent:#2563eb;
 --tof:#dc2626;--cam:#2563eb;--cmd:#7c3aed;--real:#059669;--est:#d97706;--warn:#b45309;--warnbg:#fef3c7;--err:#b91c1c;--errbg:#fee2e2;
 --obs:#9aa3ae;--ref:#cfd5dc;--disc:#f59e0b;--camcell:#a78bfa;--path:#2563eb;--pathold:#c3cad3}
@media (prefers-color-scheme:dark){:root:not([data-theme="light"]){--bg:#0f1318;--panel:#171c22;--ink:#e6e9ed;--muted:#97a1ad;
 --line:#2a313a;--grid:#20262d;--accent:#60a5fa;--tof:#f87171;--cam:#60a5fa;--cmd:#a78bfa;--real:#34d399;--est:#fbbf24;
 --warn:#fbbf24;--warnbg:#3b2f12;--err:#fca5a5;--errbg:#3f1d1d;--obs:#5b6470;--ref:#323a44;--disc:#f59e0b;--camcell:#7c6bd6;
 --path:#60a5fa;--pathold:#3a434e}}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);font:14px/1.45 system-ui,"Segoe UI","Malgun Gothic",sans-serif}
header{padding:12px 16px;border-bottom:1px solid var(--line);background:var(--panel)}
h1{font-size:18px;margin:0 0 4px}.meta{color:var(--muted);font-size:13px}.meta b{color:var(--ink);font-weight:600}
.chips{display:flex;flex-wrap:wrap;gap:6px;margin-top:6px}.chip{padding:2px 8px;border-radius:10px;font-size:12px;background:var(--grid)}
.chip.w{background:var(--warnbg);color:var(--warn)}
main{display:grid;grid-template-columns:minmax(0,1.15fr) minmax(0,1fr);gap:12px;padding:12px 16px}
@media (max-width:900px){main{grid-template-columns:minmax(0,1fr)}}
.card{background:var(--panel);border:1px solid var(--line);border-radius:8px;padding:10px;min-width:0}
.card h2{font-size:13px;margin:0 0 6px;color:var(--muted);font-weight:600}
.media{display:grid;grid-template-columns:minmax(0,1fr) 170px;gap:8px}
video{width:100%;background:#000;border-radius:6px;display:block}
.vwrap{position:relative}#vbox{position:absolute;left:0;top:0;width:100%;height:100%;pointer-events:none}
.depth img{width:100%;border-radius:4px;background:#000;min-height:40px;display:block}
.depth .cap{font-size:12px;color:var(--muted)}
.ctl{display:flex;flex-wrap:wrap;gap:8px;align-items:center;margin-top:8px;font-size:13px}
button,select,input{font:inherit;color:var(--ink);background:var(--panel);border:1px solid var(--line);border-radius:6px;padding:3px 8px}
button:hover{border-color:var(--accent)}input[type=number]{width:70px}
.now{display:grid;grid-template-columns:repeat(auto-fill,minmax(120px,1fr));gap:4px 10px;margin-top:8px;font-size:13px}
.now div span{color:var(--muted);font-size:11px;display:block}.now b{font-variant-numeric:tabular-nums}
#map{width:100%;aspect-ratio:1/1;max-height:520px;display:block;cursor:grab;touch-action:none}#map.dragging{cursor:grabbing}
.legend{display:flex;flex-wrap:wrap;gap:10px;font-size:12px;color:var(--muted);margin-top:4px}
.legend i{display:inline-block;width:10px;height:10px;border-radius:2px;margin-right:4px;vertical-align:-1px}
.full{grid-column:1/-1}
#charts canvas{width:100%;display:block}
.lists{display:grid;grid-template-columns:minmax(0,1fr) minmax(0,1.6fr);gap:12px}
@media (max-width:900px){.lists{grid-template-columns:minmax(0,1fr)}}
.list{height:300px;overflow:auto;font-size:12.5px;border:1px solid var(--line);border-radius:6px}
.row{display:grid;grid-template-columns:62px minmax(0,1fr);gap:6px;padding:2px 6px;border-bottom:1px solid var(--grid);cursor:pointer}
.row:hover{background:var(--grid)}.row .t{color:var(--muted);font-variant-numeric:tabular-nums}
.row.past{opacity:.55}.row.cur{background:var(--grid);opacity:1;box-shadow:inset 3px 0 var(--accent)}
.row.warn .m{color:var(--warn)}.row.ERROR .m,.row.FATAL .m{color:var(--err)}.row.WARN .m{color:var(--warn)}
.row .n{color:var(--muted)}.filters{display:flex;gap:6px;flex-wrap:wrap;margin-bottom:6px}
.notes{font-size:12px;color:var(--muted);margin-top:4px}
.tip{position:fixed;pointer-events:none;background:var(--panel);border:1px solid var(--line);border-radius:6px;padding:6px 8px;
 font-size:12px;box-shadow:0 4px 14px rgba(0,0,0,.15);display:none;z-index:9;white-space:nowrap;font-variant-numeric:tabular-nums}
.hint{font-size:12px;color:var(--muted)}
</style></head><body>
<header><h1 id="title"></h1><div class="meta" id="meta"></div><div class="chips" id="chips"></div><div class="notes" id="notes"></div></header>
<main>
 <section class="card"><h2>카메라 (받은 영상) · 깊이 추정</h2>
  <div class="media"><div class="vwrap"><video id="video" controls muted playsinline preload="auto"></video><canvas id="vbox"></canvas><div id="novideo" class="hint" style="display:none">영상 없음</div></div>
   <div class="depth"><img id="depth" alt=""><div class="cap" id="depthcap">깊이 영상 없음</div></div></div>
  <div class="ctl"><button id="play">▶ 재생</button>
   <label>속도 <select id="rate"><option>0.25</option><option>0.5</option><option selected>1</option><option>2</option><option>4</option></select></label>
   <label title="영상은 받아서 푼 시각이라 실제보다 늦다. 영상 속 움직임과 그래프가 맞을 때까지 바꿔 본다">영상 지연 <input id="lag" type="number" step="0.05" min="-1" max="2"> 초</label>
   <label title="bag의 YOLO 결과(/tt/detections)를 같은 시각의 영상 위에 그린다. 빨강 사람, 노랑 물체"><input type="checkbox" id="showdet" checked> YOLO 상자</label>
   <span class="hint">←/→ 1초, Shift+←/→ 한 프레임, Space 재생</span></div>
  <div class="now" id="now"></div>
 </section>
 <section class="card"><h2>지도 (위에서 봄, 추정 위치) <span class="hint">휠: 확대 · 끌기: 이동 · 두 번 클릭: 처음대로</span></h2><canvas id="map"></canvas>
  <div class="legend" id="maplegend"></div></section>
 <section class="card full"><h2>시계열 — 클릭: 그 시각으로, 끌기: 이동, Ctrl+휠: 확대·축소, 두 번 클릭: 비행 구간</h2>
  <div class="ctl" style="margin:0 0 6px"><button id="zfly">비행 구간</button><button id="zall">전체</button><span class="hint" id="range"></span></div>
  <div id="charts"></div></section>
 <section class="card full"><div class="lists">
  <div><h2>사건 (명령·드라이버)</h2><div class="list" id="events"></div></div>
  <div><h2>노드 로그</h2><div class="filters"><input id="q" placeholder="찾기 (예: 카메라, 비상)" style="flex:1;min-width:120px">
   <select id="lvl"><option value="">모든 수준</option><option value="WARN">경고 이상</option></select>
   <select id="node"><option value="">모든 노드</option></select><label><input type="checkbox" id="follow" checked> 따라가기</label></div>
   <div class="list" id="logs"></div></div></div></section>
</main>
<div class="tip" id="tip"></div>
<script>
const D=/*DATA*/null;
const $=id=>document.getElementById(id);
const css=n=>getComputedStyle(document.documentElement).getPropertyValue(n).trim();
const S=D.summary, T0=S.takeoff||S.t0;
const rel=t=>t-T0, fmt=(t,d=1)=>(rel(t)>=0?'':'−')+Math.abs(rel(t)).toFixed(d);
const f2=(v,d=2)=>v==null||Number.isNaN(v)?'–':(+v).toFixed(d);
let lag=D.lag;try{const v=localStorage.getItem('lag:'+S.name);if(v!=null&&v!=='')lag=+v}catch(e){}
let cur=S.w0, view=[S.w0,S.w1];
// ---------- 헤더
$('title').textContent=`비행 기록 ${S.name}`;
const A=D.args||{};
$('meta').innerHTML=[`시작 <b>${S.started||'-'}</b>`,`모드 <b>${S.mode||'-'}</b>`,`속도 <b>${S.speed??'-'} m/s</b>`,
 `카메라 근거 <b>${A.vision==='true'||S.mode==='explore'?'씀':'안 씀'}</b>`,S.doors==='true'?'문 지나기 <b>켬</b>':'',
 `지도 <b>${S.map||'-'}</b>`,`비행 <b>${S.flight_s!=null?S.flight_s.toFixed(1)+' s':'-'}</b>`,
 `최소 정면 ToF <b>${f2(S.min_tof)} m</b>`,`최대 수평 속도 <b>${f2(S.max_speed)} m/s</b>`,
 S.battery?`배터리 <b>${S.battery[0]}→${S.battery[1]}%</b>`:'',S.temp_max!=null?`최고 온도 <b>${S.temp_max}°C</b>`:'',
 `깊이 추정 <b>${S.depth_frames?S.depth_frames+'장'+(S.depth_hz?` (${S.depth_hz.toFixed(1)} Hz)`:''):'없음'}</b>`,
 `YOLO <b>${S.yolo?`${S.yolo.msgs}번`+(S.yolo.hz?` (초당 ${S.yolo.hz.toFixed(1)}번)`:'')+` · 사람 잡힌 장면 ${S.yolo.person_frames}`:'결과 없음'}</b>`,
 S.code?`코드 <b>${S.code}</b>`:''].filter(Boolean).join(' · ');
$('chips').innerHTML=S.warn.map(w=>`<span class="chip w">${w}</span>`).join('')+
 Object.entries(S.states||{}).slice(0,8).map(([k,v])=>`<span class="chip">${k} ${v.toFixed(0)} s</span>`).join('')+
 (S.coverage?[`다시 간 칸 ${S.coverage.revisited}/${S.coverage.cells}`+(S.coverage.revisit_ratio!=null?` (${(S.coverage.revisit_ratio*100).toFixed(0)}%)`:''),
  S.coverage.vp_chosen?`볼 자리 ${S.coverage.vp_reached}/${S.coverage.vp_chosen} 닿음`:null,`날아간 길 ${S.coverage.path_m.toFixed(1)} m`,
  S.coverage.area_per_m!=null?`새로 본 넓이 ${S.coverage.area_per_m.toFixed(1)} m²/m`:null].filter(Boolean).map(t=>`<span class="chip" title="같은 곳 다시 안 찍기 잣대: 0.5 m 칸을 15초 넘게 떠났다 다시 오면 다시 간 칸">${t}</span>`).join(''):'');
$('notes').textContent=(S.notes||[]).join(' · ');
// ---------- 시계열 준비
const FL=D.flog, FC=FL.cols, fi=c=>FC.indexOf(c)+2;
function col(rows,i,scale=1){return rows.map(r=>[r[0],r[i]==null?null:r[i]*scale])}
function deriv(pts){const o=[];for(let i=1;i<pts.length;i++){const[a,b]=[pts[i-1],pts[i]];const dt=b[0]-a[0];
 if(dt>0.02&&dt<1.5&&a[1]!=null&&b[1]!=null){let d=b[1]-a[1];if(Math.abs(d)>180)d-=Math.sign(d)*360;o.push([b[0],d/dt])}}return o}
const sensor=D.sensor;
const hspeed=sensor.map(s=>[s[0],s[5]==null||s[6]==null?null:Math.hypot(s[5],s[6])]);
const estSpeed=(()=>{const r=FL.rows,o=[],ix=fi('x'),iy=fi('y');if(ix<2)return o;
 for(let i=2;i<r.length;i+=2){const a=r[i-2],b=r[i],dt=b[0]-a[0];if(dt>0.05&&dt<1)o.push([b[0],Math.hypot(b[ix]-a[ix],b[iy]-a[iy])/dt])}return o})();
// Tello yaw는 시계 방향이 +일 수 있다(yaw_sign). 보낸 회전(반시계 +)과 같은 방향으로 맞춘다
const yawSign=parseFloat(A.yaw_sign)||1;
const yawRate=deriv(sensor.map(s=>[s[0],s[8]])).map(([t,v])=>[t,v*yawSign]);
const STATE_COLORS={};const PAL=['#60a5fa','#34d399','#fbbf24','#f87171','#a78bfa','#f472b6','#22d3ee','#a3e635','#fb923c','#94a3b8','#e879f9','#2dd4bf'];
const stateColor=s=>STATE_COLORS[s]||(STATE_COLORS[s]=PAL[Object.keys(STATE_COLORS).length%PAL.length]);
const panels=[];
function series(label,color,pts,opt={}){return{label,color,pts:pts.filter(p=>p[1]!=null||opt.nullMark),...opt}}
if(FL.rows.length)panels.push({name:'단계',h:34,band:true});
{const s=[];if(D.tof.length)s.push(series('정면 ToF (위 회색: 범위 밖)','--tof',D.tof,{dots:true,nullMark:true}));
 if(fi('cam_ahead')>=2)s.push(series('카메라 앞','--cam',col(FL.rows,fi('cam_ahead'))));
 if(fi('near_hit')>=2)s.push(series('띠 안 가까운 자리','--est',col(FL.rows,fi('near_hit'))));
 if(D.depth.some(d=>d[2]!=null))s.push(series('깊이 가운데','--camcell',D.depth.map(d=>[d[0],d[2]]),{dots:true}));
 if(s.length)panels.push({name:'앞 거리 (m)',h:150,series:s,ymin:0,ymax:1.5,lines:[[0.35,'멈춤 0.35']]});}
{const s=[];if(fi('cmd_vx')>=2)s.push(series('보낸 앞 속도','--cmd',col(FL.rows,fi('cmd_vx')),{step:true}));
 if(hspeed.length)s.push(series('Tello 수평 속도','--real',hspeed));
 if(estSpeed.length)s.push(series('추정 위치 속도','--est',estSpeed));
 if(s.length)panels.push({name:'속도 (m/s)',h:130,series:s,ymin:-0.1});}
{const s=[];if(fi('cmd_wz')>=2)s.push(series('보낸 회전','--cmd',col(FL.rows,fi('cmd_wz'),180/Math.PI),{step:true}));
 if(yawRate.length)s.push(series('Tello 회전','--real',yawRate));
 if(s.length)panels.push({name:'회전 (°/s)',h:110,series:s,sym:true});}
{const s=[];if(sensor.length)s.push(series('하방 ToF','--real',sensor.map(r=>[r[0],r[1]])));
 if(fi('z')>=2)s.push(series('추정 높이','--est',col(FL.rows,fi('z'))));
 if(s.length)panels.push({name:'높이 (m)',h:100,series:s,ymin:0,lines:D.map.flight_height?[[D.map.flight_height,'비행 높이']]:[]});}
if(D.rc.length)panels.push({name:'보낸 rc',h:110,sym:true,series:[series('앞뒤','--cmd',D.rc.map(r=>[r[0],r[2]]),{step:true}),
 series('좌우','--est',D.rc.map(r=>[r[0],r[1]]),{step:true}),series('회전','--real',D.rc.map(r=>[r[0],r[4]]),{step:true}),
 series('상하','--muted',D.rc.map(r=>[r[0],r[3]]),{step:true})]});
if(sensor.length)panels.push({name:'배터리 % · 온도 °C',h:90,ymin:0,ymax:100,series:[series('배터리','--real',sensor.map(r=>[r[0],r[3]])),
 series('온도','--tof',sensor.map(r=>[r[0],r[4]]))]});
// ---------- 그래프 그리기
const charts=$('charts'),LW=132,DPR=window.devicePixelRatio||1;
panels.forEach(p=>{const wrap=document.createElement('div');const c=document.createElement('canvas');c.height=p.h*DPR;c.style.height=p.h+'px';
 wrap.appendChild(c);charts.appendChild(wrap);p.c=c});
const eventTimes=D.events.map(e=>e.t);
function bs(arr,t,key=r=>r[0]){let lo=0,hi=arr.length-1,ans=-1;while(lo<=hi){const m=(lo+hi)>>1;if(key(arr[m])<=t){ans=m;lo=m+1}else hi=m-1}return ans}
function at(pts,t,maxAge=1.5){const i=bs(pts,t);if(i<0)return null;return t-pts[i][0]<=maxAge?pts[i][1]:null}
function drawCharts(){
 const W=charts.clientWidth;const tx=t=>LW+(t-view[0])/(view[1]-view[0])*(W-LW-8);
 for(const p of panels){const c=p.c;c.width=W*DPR;const g=c.getContext('2d');g.setTransform(DPR,0,0,DPR,0,0);g.clearRect(0,0,W,p.h);
  g.font='12px system-ui,sans-serif';g.fillStyle=css('--muted');g.textBaseline='top';g.fillText(p.name,4,4);
  if(p.band){const r=FL.rows;for(let i=0;i<r.length;i++){const a=r[i][0],b=i+1<r.length?r[i+1][0]:a+0.1;if(b<view[0]||a>view[1])continue;
    g.fillStyle=stateColor(r[i][1]);g.fillRect(tx(a),6,Math.max(1,tx(b)-tx(a)+0.5),16)}
   g.fillStyle=css('--ink');let lastX=-1e9;for(let i=0;i<r.length;i++){if(i&&r[i][1]===r[i-1][1])continue;const x=tx(r[i][0]);
    if(x<LW||x>W||x-lastX<70)continue;g.fillText(r[i][1],x+2,22);lastX=x}
   g.strokeStyle=css('--warn');for(const e of D.events){if(e.lvl!=='warn')continue;const x=tx(e.t);if(x<LW||x>W)continue;g.beginPath();g.moveTo(x,0);g.lineTo(x,6);g.stroke()}
  }else{
   // 축: 보이는 구간 값의 0.5–99.5% (튀는 값 몇 개로 눌리지 않게). 넘는 값은 가장자리에 붙여 그린다
   const vals=[];for(const s of p.series)for(const q of s.pts)if(q[1]!=null&&q[0]>=view[0]&&q[0]<=view[1])vals.push(q[1]);
   vals.sort((a,b)=>a-b);const qv=f=>vals[Math.round(f*(vals.length-1))];
   let lo=p.ymin??(vals.length?qv(0.005):0),hi=p.ymax??(vals.length?qv(0.995):1);
   if(p.ymin!=null&&p.ymax==null&&vals.length)hi=Math.max(hi,p.ymin+0.1);
   if(p.sym){const m=Math.max(Math.abs(lo),Math.abs(hi),1e-3);lo=-m;hi=m}
   if(hi-lo<1e-6)hi=lo+1;const pad=(hi-lo)*0.06;const y0=lo-(p.ymin==null?pad:0),y1=hi+(p.ymax==null?pad:0);
   const ty=v=>p.h-6-(Math.max(y0,Math.min(y1,v))-y0)/(y1-y0)*(p.h-14);
   g.strokeStyle=css('--grid');g.fillStyle=css('--muted');g.textBaseline='middle';g.textAlign='right';
   for(const v of [y0+(y1-y0)*0.1,(y0+y1)/2,y1-(y1-y0)*0.1]){g.beginPath();g.moveTo(LW,ty(v));g.lineTo(W,ty(v));g.stroke();g.fillText(v.toFixed(Math.abs(y1-y0)<3?2:0),LW-4,ty(v))}
   g.textAlign='left';
   for(const [v,l] of p.lines||[]){g.setLineDash([4,4]);g.strokeStyle=css('--muted');g.beginPath();g.moveTo(LW,ty(v));g.lineTo(W,ty(v));g.stroke();g.setLineDash([]);g.fillText(l,W-70,ty(v)-7)}
   g.save();g.beginPath();g.rect(LW,0,W-LW,p.h);g.clip();
   for(const s of p.series){const col=css(s.color);g.strokeStyle=col;g.fillStyle=col;g.lineWidth=1.5;
    const i0=Math.max(0,bs(s.pts,view[0])),i1=Math.min(s.pts.length-1,bs(s.pts,view[1])+1);const stride=Math.max(1,Math.floor((i1-i0)/(W*1.5)));
    if(s.dots){for(let i=i0;i<=i1;i+=stride){const q=s.pts[i];
      if(q[1]==null){g.fillStyle=css('--muted');g.fillRect(tx(q[0])-1,2,2,4);g.fillStyle=col;continue}
      g.fillRect(tx(q[0])-1.5,ty(q[1])-1.5,3,3)}}
    else{g.beginPath();let pen=false,prevT=null,prevY=null;for(let i=i0;i<=i1;i+=stride){const q=s.pts[i];
     if(q[1]==null||(prevT!=null&&q[0]-prevT>2)){pen=false}const x=tx(q[0]),y=ty(q[1]);
     if(q[1]!=null){if(!pen){g.moveTo(x,y);pen=true}else{if(s.step)g.lineTo(x,prevY);g.lineTo(x,y)}prevY=y}prevT=q[0]}g.stroke()}}
   g.restore();
   // 범례는 왼쪽 칸에 (그래프를 가리지 않게)
   g.textBaseline='top';g.font='11px system-ui,sans-serif';p.series.forEach((s,k)=>{const y=20+k*13;if(y>p.h-10)return;
    g.fillStyle=css(s.color);g.fillRect(4,y+2,8,8);g.fillStyle=css('--muted');let t=s.label;while(t.length>3&&g.measureText(t).width>LW-60)t=t.slice(0,-2);
    g.fillText(t===s.label?t:t+'…',15,y)});
  }
  g.strokeStyle=css('--line');g.beginPath();g.moveTo(LW,p.h-0.5);g.lineTo(W,p.h-0.5);g.stroke();
  const x=tx(cur);if(x>=LW&&x<=W){g.strokeStyle=css('--accent');g.lineWidth=1.5;g.beginPath();g.moveTo(x,0);g.lineTo(x,p.h);g.stroke()}
  if(hoverT!=null){const hx=tx(hoverT);g.strokeStyle=css('--muted');g.lineWidth=1;g.setLineDash([2,3]);g.beginPath();g.moveTo(hx,0);g.lineTo(hx,p.h);g.stroke();g.setLineDash([])}
 }
 $('range').textContent=`이륙 기준 ${fmt(view[0])} ~ ${fmt(view[1])} 초 (보이는 폭 ${(view[1]-view[0]).toFixed(1)} 초)`;
}
let hoverT=null;
function tAt(ev){const W=charts.clientWidth;const r=charts.getBoundingClientRect();const x=ev.clientX-r.left;return view[0]+(x-LW)/(W-LW-8)*(view[1]-view[0])}
let drag=null;
charts.addEventListener('mousedown',e=>{drag={x:e.clientX,v:[...view],moved:false}});
window.addEventListener('mouseup',e=>{if(drag&&!drag.moved&&e.target.closest('#charts'))seek(tAt(e));drag=null});
charts.addEventListener('mousemove',e=>{hoverT=tAt(e);
 if(drag){const W=charts.clientWidth-LW-8;const dx=(e.clientX-drag.x)/W*(drag.v[1]-drag.v[0]);if(Math.abs(e.clientX-drag.x)>3)drag.moved=true;
  view=[drag.v[0]-dx,drag.v[1]-dx]}
 showTip(e);drawCharts()});
charts.addEventListener('mouseleave',()=>{hoverT=null;$('tip').style.display='none';drawCharts()});
charts.addEventListener('wheel',e=>{if(!e.ctrlKey&&!e.metaKey)return;e.preventDefault();const t=tAt(e);const k=e.deltaY>0?1.25:0.8;view=[t-(t-view[0])*k,t+(view[1]-t)*k];drawCharts()},{passive:false});
charts.addEventListener('dblclick',()=>{view=[S.w0,S.w1];drawCharts()});
$('zfly').onclick=()=>{view=[S.w0,S.w1];drawCharts()};$('zall').onclick=()=>{view=[S.t0,S.t1];drawCharts()};
function showTip(e){const t=hoverT;const lines=[`<b>${fmt(t,2)} s</b>`];
 const st=FL.rows.length?FL.rows[Math.max(0,bs(FL.rows,t))]:null;if(st)lines.push('단계 '+st[1]);
 for(const p of panels)if(p.series)for(const s of p.series){const v=at(s.pts,t,s.dots?0.5:1.5);if(v!=null)lines.push(`${s.label} ${f2(v)}`)}
 const tip=$('tip');tip.innerHTML=lines.join('<br>');tip.style.display='block';
 tip.style.left=Math.min(e.clientX+14,innerWidth-tip.offsetWidth-8)+'px';tip.style.top=(e.clientY+14)+'px'}
// ---------- 지도
const mapC=$('map'),M=D.map,MV={zoom:1,px:0,py:0};
{const at=e=>{const r=mapC.getBoundingClientRect();return[e.clientX-r.left,e.clientY-r.top]};
 const zoomAt=(mx,my,f)=>{const W=mapC.clientWidth,H=mapC.clientHeight,z2=Math.max(0.5,Math.min(30,MV.zoom*f));f=z2/MV.zoom;
  MV.px=(mx-W/2)-(mx-W/2-MV.px)*f;MV.py=(my-H/2)-(my-H/2-MV.py)*f;MV.zoom=z2;drawMap()};
 mapC.addEventListener('wheel',e=>{e.preventDefault();const[mx,my]=at(e);zoomAt(mx,my,e.deltaY<0?1.2:1/1.2)},{passive:false});
 let last=null;mapC.addEventListener('pointerdown',e=>{mapC.setPointerCapture(e.pointerId);last=at(e);mapC.classList.add('dragging')});
 mapC.addEventListener('pointermove',e=>{if(!last)return;const p=at(e);MV.px+=p[0]-last[0];MV.py+=p[1]-last[1];last=p;drawMap()});
 const up=()=>{last=null;mapC.classList.remove('dragging')};mapC.addEventListener('pointerup',up);mapC.addEventListener('pointercancel',up);
 mapC.addEventListener('dblclick',()=>{MV.zoom=1;MV.px=0;MV.py=0;drawMap()})}
function drawMap(){
 const b=M.bounds||[0,0,1,1];const W=mapC.clientWidth,H=mapC.clientHeight;mapC.width=W*DPR;mapC.height=H*DPR;const g=mapC.getContext('2d');g.setTransform(DPR,0,0,DPR,0,0);
 const sx=(W-20)/(b[2]-b[0]),sy=(H-20)/(b[3]-b[1]);const s0=Math.min(sx,sy);const ox0=10+((W-20)-s0*(b[2]-b[0]))/2,oy0=10+((H-20)-s0*(b[3]-b[1]))/2;
 // 확대·이동 (11/02): 화면 가운데 기준으로 MV.zoom배 한 뒤 MV.px·py만큼 옮긴다
 const s=s0*MV.zoom,ox=W/2+(ox0-W/2)*MV.zoom+MV.px,oy=H/2-(H/2-oy0)*MV.zoom-MV.py;
 // 위에서 본 그림: +x 오른쪽, +y 위
 const px=x=>ox+(x-b[0])*s,py=y=>H-oy-(y-b[1])*s;
 const box=(o,fill,stroke)=>{g.fillStyle=fill;g.fillRect(px(o[0]-o[2]/2),py(o[1]+o[3]/2),o[2]*s,o[3]*s);if(stroke){g.strokeStyle=stroke;g.strokeRect(px(o[0]-o[2]/2),py(o[1]+o[3]/2),o[2]*s,o[3]*s)}};
 g.clearRect(0,0,W,H);g.strokeStyle=css('--line');g.strokeRect(px(b[0]),py(b[3]),(b[2]-b[0])*s,(b[3]-b[1])*s);
 g.strokeStyle=css('--grid');for(let x=Math.ceil(b[0]);x<=b[2];x++){g.beginPath();g.moveTo(px(x),py(b[1]));g.lineTo(px(x),py(b[3]));g.stroke()}
 for(let y=Math.ceil(b[1]);y<=b[3];y++){g.beginPath();g.moveTo(px(b[0]),py(y));g.lineTo(px(b[2]),py(y));g.stroke()}
 (M.walls||[]).forEach(o=>box(o,css('--obs')));(M.obstacles||[]).forEach(o=>box(o,css('--obs')));
 if(M.camera_cells){const r=M.camera_cells.res*s;g.fillStyle=css('--camcell');g.globalAlpha=.2;for(const [x,y] of M.camera_cells.sus||[])g.fillRect(px(x)-r/2,py(y)-r/2,r,r);
  g.globalAlpha=.6;for(const [x,y] of M.camera_cells.xy)g.fillRect(px(x)-r/2,py(y)-r/2,r,r);g.globalAlpha=1}
 (M.discovered||[]).forEach(o=>box(o,'transparent',css('--disc')));
 // 찾은 사람: 칠한 점은 길을 막은 사람, 점선 원은 그 자리를 다시 봤는데 없어 길을 막지 않게 된 사람
 (M.people||[]).forEach(p=>{g.beginPath();g.arc(px(p[0]),py(p[1]),Math.max(5,0.25*s),0,7);if(p[2]){g.strokeStyle=css('--tof');g.setLineDash([3,3]);g.lineWidth=1.5;g.stroke();g.setLineDash([])}else{g.fillStyle=css('--tof');g.fill()}});
 const r=FL.rows,ix=fi('x'),iy=fi('y'),iyaw=fi('yaw');
 if(ix>=2&&r.length){g.lineWidth=1.5;g.strokeStyle=css('--pathold');g.beginPath();r.forEach((q,i)=>{if(q[0]<S.w0-5||q[0]>S.w1+5)return;i?g.lineTo(px(q[ix]),py(q[iy])):g.moveTo(px(q[ix]),py(q[iy]))});g.stroke();
  const k=bs(r,cur);g.strokeStyle=css('--path');g.lineWidth=2.5;g.beginPath();let st=false;for(let i=0;i<=k;i++){const q=r[i];if(q[0]<S.w0-5)continue;st?g.lineTo(px(q[ix]),py(q[iy])):(g.moveTo(px(q[ix]),py(q[iy])),st=true)}g.stroke();
  if(k>=0){const q=r[k],x=px(q[ix]),y=py(q[iy]),a=q[iyaw]||0;const tof=at(D.tof,cur,0.6);
   if(tof!=null){g.strokeStyle=css('--tof');g.lineWidth=2;g.beginPath();g.moveTo(x,y);g.lineTo(x+Math.cos(a)*tof*s,y-Math.sin(a)*tof*s);g.stroke()}
   g.fillStyle=css('--accent');g.beginPath();g.moveTo(x+Math.cos(a)*12,y-Math.sin(a)*12);g.lineTo(x+Math.cos(a+2.5)*8,y-Math.sin(a+2.5)*8);g.lineTo(x+Math.cos(a-2.5)*8,y-Math.sin(a-2.5)*8);g.closePath();g.fill()}}
 if(M.spawn){g.strokeStyle=css('--muted');g.beginPath();g.arc(px(M.spawn.x),py(M.spawn.y),6,0,7);g.stroke()}
 if(M.goal){g.strokeStyle=css('--real');g.lineWidth=2;g.beginPath();g.arc(px(M.goal.x),py(M.goal.y),7,0,7);g.stroke()}
 g.fillStyle=css('--muted');g.font='11px system-ui';g.fillText('1 m 격자 · +x →, +y ↑',px(b[0])+4,py(b[1])-6);
}
$('maplegend').innerHTML=[['--path','지나온 길'],['--pathold','전체 경로'],['--tof','정면 ToF'],M.walls&&M.walls.length?['--obs','지도의 벽·가구']:null,
 M.camera_cells?['--camcell','카메라로 만든 지도(진하게 막힘, 옅게 의심)']:null,
 M.people&&M.people.length?['--tof','찾은 사람(점선: 다시 봤는데 없어 길을 막지 않음)']:null,
 M.discovered&&M.discovered.length?['--disc','찾은 장애물']:null].filter(Boolean).map(([c,l])=>`<span><i style="background:${css(c)}"></i>${l}</span>`).join('');
// ---------- 현재 값
function drawNow(){const r=FL.rows.length?FL.rows[Math.max(0,bs(FL.rows,cur))]:null;const s=sensor.length?sensor[Math.max(0,bs(sensor,cur))]:null;
 const rc=D.rc.length?D.rc[Math.max(0,bs(D.rc,cur))]:null;const v=(c)=>r&&fi(c)>=2?r[fi(c)]:null;
 const u=(x,unit,d=2)=>x==null?null:f2(x,d)+unit, deg=x=>x==null?null:(x*180/Math.PI).toFixed(0);
 const tof=at(D.tof,cur,0.6);
 const items=[['이륙 기준',fmt(cur,2)+' s'],['단계',r?r[1]:'–'],['정면 ToF',tof==null?'없음/범위 밖':f2(tof)+' m'],
  ['카메라 앞',u(v('cam_ahead'),' m')],['띠',v('lane_free')==null?null:(v('lane_free')?'비었음':'막힘')],['보낸 앞 속도',u(v('cmd_vx'),' m/s')],
  ['보낸 회전',v('cmd_wz')==null?null:deg(v('cmd_wz'))+' °/s'],['Tello 수평 속도',s&&s[5]!=null?u(Math.hypot(s[5],s[6]),' m/s'):null],
  ['하방 ToF',s?u(s[1],' m'):null],['추정 위치',r&&fi('x')>=2?`${f2(r[fi('x')])}, ${f2(r[fi('y')])}`:null],
  ['방향',v('yaw')==null?null:deg(v('yaw'))+'°'],['rc',rc?`${rc[1]} ${rc[2]} ${rc[3]} ${rc[4]}`:null],
  ['배터리',s&&s[3]!=null?s[3]+'%':null],['온도',s&&s[4]!=null?s[4]+'°C':null],
  ['목표 방향',v('soft_target')==null?null:deg(v('soft_target'))+'° '+(r[fi('soft_src')]||'')]];
 $('now').innerHTML=items.filter(([,x])=>x!=null).map(([k,x])=>`<div><span>${k}</span><b>${x}</b></div>`).join('')}
// ---------- 사건·로그
const evBox=$('events'),logBox=$('logs');
evBox.innerHTML=D.events.map((e,i)=>`<div class="row ${e.lvl}" data-t="${e.t}"><span class="t">${fmt(e.t)}</span><span class="m">${e.kind} · ${esc(e.text)}</span></div>`).join('')||'<div class="hint" style="padding:6px">없음</div>';
function esc(s){return String(s).replace(/[&<>]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;'}[c]))}
const nodes=[...new Set(D.logs.map(l=>l[2]))].sort();$('node').innerHTML+=nodes.map(n=>`<option>${esc(n)}</option>`).join('');
let shownLogs=[],shownT=[];
function renderLogs(){const q=$('q').value.trim(),lv=$('lvl').value,nd=$('node').value;
 shownLogs=D.logs.filter(l=>(!q||l[3].includes(q))&&(!lv||['WARN','ERROR','FATAL'].includes(l[1]))&&(!nd||l[2]===nd)).slice(0,8000);
 shownT=shownLogs.map(l=>l[0]);
 logBox.innerHTML=shownLogs.map(l=>`<div class="row ${l[1]}" data-t="${l[0]}"><span class="t">${fmt(l[0])}</span><span class="m"><span class="n">${esc(l[2])}</span> ${esc(l[3])}</span></div>`).join('')
  ||`<div class="hint" style="padding:6px">${D.logs.length?'맞는 줄 없음':'노드 로그 없음 (10/29 이전 비행은 화면에만 찍혔다)'}</div>`;lastLogI=-2;markLists()}
['q','lvl','node'].forEach(id=>$(id).addEventListener('input',renderLogs));
[evBox,logBox].forEach(b=>b.addEventListener('click',e=>{const r=e.target.closest('.row');if(r)seek(+r.dataset.t)}));
let lastLogI=-2,lastEvI=-2;
function markList(box,arr,last){const i=bs(arr,cur,x=>+x);if(i===last)return i;const rows=box.children;
 for(let k=0;k<rows.length;k++){rows[k].classList.toggle('past',k<i);rows[k].classList.toggle('cur',k===i)}
 if($('follow').checked&&i>=0&&rows[i]){const top=rows[i].getBoundingClientRect().top-box.getBoundingClientRect().top+box.scrollTop-box.clientHeight/2;box.scrollTop=top}return i}
function markLists(){lastEvI=markList(evBox,eventTimes,lastEvI);lastLogI=markList(logBox,shownT,lastLogI)}
// ---------- 영상·깊이·시각
const video=$('video'),F=D.frames;
if(F.length){video.src='viewer/video.mp4'}else{video.style.display='none';$('novideo').style.display='block'}
if(!D.depth.length){document.querySelector('.depth').style.display='none';document.querySelector('.media').style.gridTemplateColumns='minmax(0,1fr)'}
$('lag').value=lag;$('lag').addEventListener('input',e=>{lag=+e.target.value||0;try{localStorage.setItem('lag:'+S.name,lag)}catch(_){};sync()});
const frameT=i=>F[Math.max(0,Math.min(F.length-1,i))]-lag;
function videoTime(){return frameT(Math.floor(video.currentTime*D.fps+1e-3))}
// 페이지가 영상을 옮긴 것(progSeek)과 사람이 영상 막대를 옮긴 것을 나눈다. 앞의 것은 시각을 영상에 맞추지 않는다
let progSeek=0,pendingSeek=null;
function setVideoTime(x){progSeek++;if(video.readyState>=1)video.currentTime=x;else pendingSeek=x}
video.addEventListener('loadedmetadata',()=>{if(pendingSeek!=null){video.currentTime=pendingSeek;pendingSeek=null}});
function seek(t){cur=t;if(F.length){const i=Math.max(0,bs(F,t+lag,x=>x));if(t+lag>=F[0]-0.5&&t+lag<=F[F.length-1]+0.5)setVideoTime((i+0.5)/D.fps)}update()}
function sync(){if(F.length&&!video.paused)cur=videoTime();update()}
// 영상 위 YOLO 상자: 지금 보이는 프레임(받은 시각 cur+지연)의 탐지. 0.4초 넘게 지난 결과는 그리지 않는다
const DT=D.dets||[];
if(!DT.length)$('showdet').parentElement.style.display='none';
$('showdet').addEventListener('change',()=>drawBoxes());
function drawBoxes(){const c=$('vbox');const W=video.clientWidth,H=video.clientHeight;c.width=W*DPR;c.height=H*DPR;const g=c.getContext('2d');g.setTransform(DPR,0,0,DPR,0,0);g.clearRect(0,0,W,H);
 if(!$('showdet').checked||!DT.length||!F.length)return;const t=cur+lag,i=bs(DT,t);if(i<0||t-DT[i][0]>0.4)return;
 const [,w,h,boxes]=DT[i],sx=W/w,sy=H/h;g.lineWidth=2;g.font='12px system-ui,sans-serif';g.textBaseline='top';
 for(const [x1,y1,x2,y2,n,cf] of boxes){const col=n==='person'?css('--tof'):css('--disc');g.strokeStyle=col;g.strokeRect(x1*sx,y1*sy,(x2-x1)*sx,(y2-y1)*sy);
  const lab=`${n==='person'?'사람':n} ${cf.toFixed(2)}`,tw=g.measureText(lab).width+6,ly=Math.max(0,y1*sy-16);g.fillStyle=col;g.fillRect(x1*sx,ly,tw,16);g.fillStyle='#fff';g.fillText(lab,x1*sx+3,ly+2)}}
function update(){if(cur<view[0]||cur>view[1]){const w=view[1]-view[0];view=[cur-w*0.2,cur+w*0.8]}
 drawCharts();drawMap();drawNow();markLists();drawBoxes();
 const dp=D.depth;if(dp.length){const i=bs(dp,cur);const d=i>=0?dp[i]:null;if(d&&cur-d[0]<1.5){const src='viewer/depth/'+d[1];if(!$('depth').src.endsWith(src))$('depth').src=src;
   $('depthcap').textContent=`깊이 ${fmt(d[0],2)} s · 가운데 ${f2(d[2])} m · 가까움 빨강 → 멂 파랑`}else $('depthcap').textContent='이 시각 깊이 영상 없음'}}
let playing=false,lastWall=0;
function tick(ts){if(F.length){if(!video.paused)cur=videoTime()}else if(playing){cur+=(ts-lastWall)/1000*(+$('rate').value);if(cur>S.t1)playing=false}
 lastWall=ts;if(playing||(F.length&&!video.paused))update();requestAnimationFrame(tick)}
requestAnimationFrame(tick);
function togglePlay(){if(F.length){if(video.paused){if(Math.abs(videoTime()-cur)>0.2)seek(Math.max(cur,F[0]-lag));video.play()}else video.pause()}else playing=!playing;
 $('play').textContent=(F.length?!video.paused:playing)?'❚❚ 멈춤':'▶ 재생'}
$('play').onclick=togglePlay;video.addEventListener('pause',()=>{$('play').textContent='▶ 재생'});video.addEventListener('play',()=>{$('play').textContent='❚❚ 멈춤'});
video.addEventListener('seeked',()=>{if(progSeek>0){progSeek--;return}if(video.paused){cur=videoTime();update()}});
$('rate').addEventListener('change',e=>{video.playbackRate=+e.target.value});
window.addEventListener('keydown',e=>{if(e.target.tagName==='INPUT'||e.target.tagName==='SELECT')return;
 if(e.code==='Space'){e.preventDefault();togglePlay()}else if(e.key==='ArrowRight'||e.key==='ArrowLeft'){e.preventDefault();
  const d=(e.shiftKey?1/D.fps:1)*(e.key==='ArrowRight'?1:-1);video.pause();playing=false;seek(cur+d)}});
window.addEventListener('resize',update);
renderLogs();update();
</script></body></html>
"""

INDEX = r"""<!doctype html>
<html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>비행 기록 목록</title>
<style>
:root{--bg:#f6f7f9;--panel:#fff;--ink:#1d232b;--muted:#66707c;--line:#dde1e6;--accent:#2563eb;--warn:#b45309;--warnbg:#fef3c7}
@media (prefers-color-scheme:dark){:root:not([data-theme="light"]){--bg:#0f1318;--panel:#171c22;--ink:#e6e9ed;--muted:#97a1ad;--line:#2a313a;
 --accent:#60a5fa;--warn:#fbbf24;--warnbg:#3b2f12}}
body{margin:0;background:var(--bg);color:var(--ink);font:14px/1.45 system-ui,"Segoe UI","Malgun Gothic",sans-serif;padding:16px}
h1{font-size:18px;margin:0 0 4px}p{color:var(--muted);margin:0 0 12px;font-size:13px}
.wrap{overflow-x:auto;background:var(--panel);border:1px solid var(--line);border-radius:8px}
table{border-collapse:collapse;width:100%;font-size:13px}th,td{padding:6px 10px;border-bottom:1px solid var(--line);text-align:left;white-space:nowrap}
th{color:var(--muted);font-weight:600}td.n{text-align:right;font-variant-numeric:tabular-nums}a{color:var(--accent)}
.w{background:var(--warnbg);color:var(--warn);padding:1px 6px;border-radius:8px;font-size:12px;margin-right:4px}
code{font-size:12px}
</style></head><body>
<h1>비행 기록 목록</h1>
<p>각 비행의 영상·지도·시계열·로그를 한 페이지로 본다. 페이지가 없는 비행은
<code>~/venvs/tt/bin/python tools/flight_viewer.py --all</code>(simulation/tt-sim에서)로 만든다.</p>
<div class="wrap"><table><thead><tr><th>비행</th><th>시작</th><th>모드</th><th class="n">속도</th><th>카메라</th>
<th class="n">비행 s</th><th class="n">최소 ToF</th><th class="n">최대 속도</th><th class="n">깊이</th><th>주의</th></tr></thead><tbody id="b"></tbody></table></div>
<script>
const D=/*DATA*/null;const f=(v,d=2)=>v==null?'–':(+v).toFixed(d);
document.getElementById('b').innerHTML=D.map(s=>`<tr><td>${s.page?`<a href="${s.name}/viewer.html">${s.name}</a>`:s.name+' <span style="color:var(--muted)">(페이지 없음)</span>'}</td>
<td>${s.started||'–'}</td><td>${s.mode||'–'}</td><td class="n">${s.speed??'–'}</td><td>${s.vision==='true'||s.mode==='explore'?'씀':'–'}</td>
<td class="n">${f(s.flight_s,1)}</td><td class="n">${f(s.min_tof)}</td><td class="n">${f(s.max_speed)}</td>
<td class="n">${s.depth_frames?s.depth_frames+'장':'–'}</td><td>${(s.warn||[]).map(w=>`<span class="w">${w}</span>`).join('')}</td></tr>`).join('');
</script></body></html>
"""

if __name__ == "__main__":
    raise SystemExit(main())
