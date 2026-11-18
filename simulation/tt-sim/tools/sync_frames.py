#!/usr/bin/env python3
"""실기체 비행 기록을 한 시간축으로 맞춘다: 영상 프레임 ↔ 전방 ToF ↔ Tello 상태값 ↔ 보낸 rc ↔ 비행 노드 판단 (↔ 깊이 영상).

  ~/venvs/tt/bin/python tools/sync_frames.py ~/ttrepo/data/outputs/flights/20261029_105842
  ... --no-video          # timeline.csv만 (영상 다시 인코딩 안 함)

모든 기록은 같은 WSL 시계를 쓰지만, 영상은 '받아서 푼 시각'이라 실제 촬영보다 늦다(Tello 영상 지연 0.2–0.5 s, 기록 안 됨).
그 지연을 기록에서 잰다: 기체가 돌 때 영상이 옆으로 밀리는 양(앞뒤 프레임의 위상 상관)과 Tello yaw 회전 속도를
여러 지연으로 맞대어 가장 잘 맞는 지연을 고른다. 이 값을 빼서 '촬영 추정 시각'에 다른 기록을 붙인다.

나오는 것 (기록 폴더 안 sync/)
  timeline.csv      프레임마다: 받은 시각, 촬영 추정 시각, 그때의 비행 단계·추정 위치·방향·판단 근거(flight_log.csv),
                    직전 ToF 값과 그 나이, 하방 ToF·속도·자세(상태값), 직전 rc
  video_sync.mp4    영상 위에 ToF 원뿔 영역(±12.5°)과 ToF 거리, 단계·보낸 속도·높이를 겹쳐 그린 것
  depth_vs_tof.csv  (bag에 /tt/depth가 있으면) 깊이 영상 가운데의 가까운 쪽 거리 ↔ 같은 시각 ToF — 깊이 배율 확인용
  summary.txt       잰 영상 지연, 프레임·ToF·상태 개수, 깊이/ToF 비율
"""
from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
import statistics
from datetime import datetime
from pathlib import Path

import numpy as np

HFOV = math.radians(82.6)
TOF_HALF = math.radians(12.5)
LAGS = np.arange(-0.6, 1.21, 0.02)   # 찾아볼 영상 지연 (s). 끝에 걸리면 믿지 않는다


def iso(t: str) -> float:
    return datetime.fromisoformat(t).timestamp()


def jsonl(p: Path) -> list[dict]:
    return [json.loads(line) for line in p.open(encoding="utf-8")] if p.exists() else []


class Series:
    """시각 순 (t, 값) — 어떤 시각 직전 값과 그 나이."""

    def __init__(self, items):
        items = sorted(items, key=lambda kv: kv[0])
        self.t = [t for t, _ in items]
        self.v = [v for _, v in items]

    def before(self, t: float):
        i = bisect.bisect_right(self.t, t) - 1
        return (self.v[i], t - self.t[i]) if i >= 0 else (None, None)

    def __len__(self) -> int:
        return len(self.t)


def load(d: Path):
    logs = sorted((d / "tello_log").glob("*/commands.jsonl"))
    cmds = jsonl(logs[0]) if logs else []
    sens = jsonl(logs[0].parent / "sensor.jsonl") if logs else []
    tof, rc = [], []
    for c in cmds:
        if c["cmd"].startswith("EXT") and str(c.get("response", "")).startswith("tof"):
            mm = int(str(c["response"]).split()[1])
            t = iso(c.get("recorded_at") or c["sent_at"])
            tof.append((t, math.inf if mm >= 8000 else mm / 1000.0))
        elif c["cmd"].startswith("rc "):
            rc.append((iso(c["sent_at"]), c["cmd"][3:]))
    state = Series((iso(s["received_at"]), s["raw"]) for s in sens)
    fl = []
    if (d / "flight_log.csv").exists():
        fl = [(float(r["t"]), r) for r in csv.DictReader((d / "flight_log.csv").open(encoding="utf-8"))]
    frames = []
    vf = d / "video" / "video_frames.csv"
    if vf.exists():
        frames = [(int(r["frame"]), float(r["wall_time"])) for r in csv.DictReader(vf.open(encoding="utf-8"))]
    return Series(tof), Series(rc), state, Series(fl), frames


def image_shifts(h264: Path, frames: list, width: int = 160):
    """앞뒤 프레임의 가로 밀림(화소, 원본 폭 기준으로 환산)을 위상 상관으로. (받은 시각, 밀림) 목록."""
    import av

    out, prev = [], None
    with av.open(str(h264), format="h264") as c:
        for i, fr in enumerate(c.decode(video=0)):
            if i >= len(frames):
                break
            g = fr.to_ndarray(format="gray", width=width, height=int(width * fr.height / fr.width))
            g = g.astype(np.float32)
            g -= g.mean()
            if prev is not None:
                f1, f2 = np.fft.fft2(prev), np.fft.fft2(g)
                r = f1 * np.conj(f2)
                r /= np.abs(r) + 1e-9
                cc = np.abs(np.fft.ifft2(r))
                dy, dx = np.unravel_index(np.argmax(cc), cc.shape)
                dx = dx - cc.shape[1] if dx > cc.shape[1] // 2 else dx
                out.append((frames[i][1], dx * fr.width / width))
            prev = g
    return out


def estimate_latency(shifts, state: Series):
    """영상 가로 밀림 속도 ↔ Tello yaw 회전 속도를 지연별로 맞대어 상관이 가장 큰 지연. (지연, 상관) 또는 None."""
    if len(shifts) < 50 or len(state) < 50:
        return None
    ts = np.array([t for t, _ in shifts])
    vs = np.array([v for _, v in shifts])
    dt = np.diff(ts, prepend=ts[0])
    ok = dt > 1e-3
    img_rate = np.zeros_like(vs)
    img_rate[ok] = vs[ok] / dt[ok]                                  # 화소/초
    st_all = np.array(state.t)
    yaw_all = np.unwrap(np.radians([float(r["yaw"]) for r in state.v]))
    keep = np.concatenate([[True], np.diff(st_all) > 1e-3])          # 같은 시각에 받은 상태값은 하나만
    st, yaw = st_all[keep], yaw_all[keep]
    yaw_rate = np.gradient(yaw, st)
    # 영상 밀림은 프레임마다 흔들리므로 0.2초 창으로 고르고, 회전 속도도 같은 창으로
    k = np.ones(5) / 5
    img_rate = np.convolve(img_rate, np.ones(5) / 5, mode="same")
    yaw_rate = np.convolve(yaw_rate, k[:3] / k[:3].sum(), mode="same")
    grid = np.arange(max(ts[0], st[0]) + 1.5, min(ts[-1], st[-1]) - 1.5, 0.05)
    if len(grid) < 100:
        return None
    a = np.interp(grid, ts, img_rate)
    best = None
    for lag in LAGS:
        b = np.interp(grid - lag, st, yaw_rate)                     # 영상은 lag만큼 늦게 받는다
        if np.std(a) < 1e-6 or np.std(b) < 1e-6:
            continue
        c = abs(float(np.corrcoef(a, b)[0, 1]))
        if best is None or c > best[1]:
            best = (float(lag), c)
    return best


def depth_vs_tof(d: Path, tof: Series):
    """bag의 /tt/depth 가운데(ToF 원뿔) 가까운 쪽 거리 ↔ 같은 시각 ToF. 실패하면 빈 목록."""
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import Image
    except ImportError:
        return []
    bag = d / "bag"
    if not (bag / "metadata.yaml").exists():
        return []
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/tt/depth"]))
    rows = []
    while reader.has_next():
        _, data, _ = reader.read_next()
        m = deserialize_message(data, Image)
        dep = np.frombuffer(m.data, dtype=np.float32).reshape(m.height, m.width)
        f = (m.width / 2) / math.tan(HFOV / 2)
        r = int(f * math.tan(TOF_HALF))
        patch = dep[m.height // 2 - r:m.height // 2 + r, m.width // 2 - r:m.width // 2 + r]
        patch = patch[np.isfinite(patch) & (patch > 0.05)]
        if patch.size < 10:
            continue
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        v, age = tof.before(t)
        if v is not None and math.isfinite(v) and age < 0.25:
            rows.append((t, float(np.percentile(patch, 5)), v, age))
    return rows


def overlay(d: Path, frames, lat: float, timeline: list[dict], out: Path) -> int:
    import av
    from PIL import Image as PImage
    from PIL import ImageDraw

    n = 0
    with av.open(str(d / "video" / "video.h264"), format="h264") as inp, av.open(str(out), "w") as dst:
        st = None
        for i, fr in enumerate(inp.decode(video=0)):
            if i >= len(timeline):
                break
            if st is None:
                for codec in ("libx264", "h264", "mpeg4"):
                    try:
                        st = dst.add_stream(codec, rate=24)
                        break
                    except Exception:  # noqa: BLE001 — 설치된 인코더를 차례로 시도
                        continue
                st.width, st.height, st.pix_fmt = fr.width, fr.height, "yuv420p"
            img = fr.to_image()
            dr = ImageDraw.Draw(img)
            w, h = img.size
            f = (w / 2) / math.tan(HFOV / 2)
            r = f * math.tan(TOF_HALF)
            row = timeline[i]
            tofv = row["tof"]
            color = (255, 60, 60) if tofv and tofv < 0.5 else (255, 210, 0) if tofv and tofv < 1.2 else (80, 220, 80)
            dr.rectangle([w / 2 - r, h / 2 - r, w / 2 + r, h / 2 + r], outline=color, width=3)
            lines = [f"t {row['t_cap'] - timeline[0]['t_cap']:6.2f}s  frame {row['frame']}  (video lag {lat:.2f}s)",
                     f"ToF {('%.2f m' % tofv) if tofv else 'none'} (age {row['tof_age'] or 0:.2f}s)",
                     f"state {row['state']}  cmd vx {row['cmd_vx']} wz {row['cmd_wz']}",
                     f"h-tof {row['h_tof']} cm  vgx {row['vgx']}  yaw {row['yaw_tello']}  rc {row['rc']}"]
            for k, s in enumerate(lines):
                dr.text((10 + 1, 10 + 22 * k + 1), s, fill=(0, 0, 0))
                dr.text((10, 10 + 22 * k), s, fill=(255, 255, 255))
            for pkt in st.encode(av.VideoFrame.from_image(img)):
                dst.mux(pkt)
            n += 1
        if st is not None:
            for pkt in st.encode():
                dst.mux(pkt)
    return n


def main() -> int:
    ap = argparse.ArgumentParser(description="비행 기록을 영상 프레임 시간축으로 맞춘다")
    ap.add_argument("out_dir", type=Path)
    ap.add_argument("--no-video", action="store_true")
    ap.add_argument("--lag", type=float, default=None, help="영상 지연을 재지 않고 이 값(s)을 쓴다")
    args = ap.parse_args()
    d = args.out_dir.expanduser()
    out = d / "sync"
    out.mkdir(exist_ok=True)
    tof, rc, state, fl, frames = load(d)
    summary = [f"기록 {d.name}: 프레임 {len(frames)}, ToF {len(tof)}, 상태값 {len(state)}, rc {len(rc)}, 비행 기록 {len(fl)}줄"]

    lat, corr = args.lag, None
    if lat is None and frames:
        try:
            got = estimate_latency(image_shifts(d / "video" / "video.h264", frames), state)
        except ImportError:
            got = None
            summary.append("영상 지연: PyAV가 없어 못 잼 (~/venvs/tt/bin/python으로 실행)")
        if got and got[1] >= 0.5 and LAGS[0] < got[0] < LAGS[-1] and got[0] >= 0:
            lat, corr = got
        elif got:
            summary.append(f"영상 지연: 잰 값 {got[0]:.2f} s (상관 {got[1]:.2f})는 믿기 어려움 — 회전이 적거나 영상이 흔들림")
    if lat is None:
        lat = 0.3
        summary.append("영상 지연: 잴 회전이 부족해 0.30 s로 둠 (--lag로 줄 수 있음)")
    else:
        summary.append(f"영상 지연 {lat:.2f} s" + (f" (영상 밀림 ↔ yaw 회전 상관 {corr:.2f})" if corr is not None else " (지정)"))

    timeline = []
    for fno, t in frames:
        tc = t - lat
        tv, tage = tof.before(tc)
        sv, _ = state.before(tc)
        rv, _ = rc.before(tc)
        fv, _ = fl.before(tc)
        fv = fv or {}
        timeline.append({
            "frame": fno, "t_recv": t, "t_cap": tc,
            "tof": tv if tv is not None and math.isfinite(tv) else None, "tof_age": tage,
            "state": fv.get("state", ""), "x": fv.get("x", ""), "y": fv.get("y", ""), "yaw_est": fv.get("yaw", ""),
            "front_tof_node": fv.get("front_tof", ""), "cam_ahead": fv.get("cam_ahead", ""), "lane_free": fv.get("lane_free", ""),
            "cmd_vx": fv.get("cmd_vx", ""), "cmd_wz": fv.get("cmd_wz", ""),
            "h_tof": sv.get("tof") if sv else "", "vgx": sv.get("vgx") if sv else "", "vgy": sv.get("vgy") if sv else "",
            "yaw_tello": sv.get("yaw") if sv else "", "pitch": sv.get("pitch") if sv else "", "roll": sv.get("roll") if sv else "",
            "bat": sv.get("bat") if sv else "", "rc": rv or "",
        })
    if timeline:
        with (out / "timeline.csv").open("w", newline="", encoding="utf-8") as fh:
            w = csv.DictWriter(fh, fieldnames=list(timeline[0].keys()))
            w.writeheader()
            for row in timeline:
                w.writerow({k: (f"{v:.4f}" if isinstance(v, float) else v) for k, v in row.items()})
        summary.append(f"timeline.csv {len(timeline)}줄")

    dv = depth_vs_tof(d, tof)
    if dv:
        with (out / "depth_vs_tof.csv").open("w", newline="", encoding="utf-8") as fh:
            w = csv.writer(fh)
            w.writerow(["t", "depth_center_m", "tof_m", "tof_age"])
            w.writerows([[f"{a:.4f}", f"{b:.3f}", f"{c:.3f}", f"{e:.3f}"] for a, b, c, e in dv])
        ratios = [c / b for _, b, c, _ in dv if b > 0]
        summary.append(f"깊이 ↔ ToF {len(dv)}쌍: ToF/깊이 중앙 {statistics.median(ratios):.2f} "
                       f"(25–75% {np.percentile(ratios, 25):.2f}–{np.percentile(ratios, 75):.2f}) — 1보다 크면 깊이가 짧게 잰다")
    else:
        summary.append("깊이 ↔ ToF: bag에 /tt/depth가 없거나(카메라 근거를 안 썼거나 10/29 이전 기록) ToF가 1.2 m 안을 잰 때가 없음")

    if not args.no_video and frames:
        try:
            n = overlay(d, frames, lat, timeline, out / "video_sync.mp4")
            summary.append(f"video_sync.mp4 {n}프레임")
        except ImportError:
            summary.append("video_sync.mp4: PyAV·Pillow가 없어 못 만듦 (~/venvs/tt/bin/python으로 실행)")
    (out / "summary.txt").write_text("\n".join(summary) + "\n", encoding="utf-8")
    print("\n".join(summary))
    print(f"→ {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
