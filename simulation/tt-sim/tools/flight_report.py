#!/usr/bin/env python3
"""실기체 비행 기록 폴더(real.launch.py out_dir) 하나를 정리한다: report.md, path.png, (영상이 있으면) video.mp4.

  python3 tools/flight_report.py ~/ttrepo/data/outputs/flights/20261028_164101
  ~/venvs/tt/bin/python tools/flight_report.py <폴더>     # video.h264 → video.mp4 (PyAV가 있는 파이썬)

보는 것
- 실행 조건: run.json(인자·코드 지문), 이륙·착륙 응답, 단계 흐름
- 실제로 낸 속도: Tello 상태값(vgx·yaw)과 보낸 rc를 시각으로 맞춰, 앞으로·회전할 때 실제 속도와
  rc 100 환산값(v_full·yaw_full_deg 제안)을 낸다. 명령한 만큼 안 나가면 여기서 보인다
- 시간 쓰임: 움직임·제자리 회전·정지 (tools/eval_dwell.py와 같은 기준)
- 배터리·온도·높이(하방 ToF), 전방 ToF 응답, 찾은 장애물, 영상 프레임 수, bag 토픽별 메시지 수
기록이 빠진 부분(예전 비행에는 run.json·video·bag이 없다)은 건너뛴다.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import sys
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from eval_dwell import analyze as dwell  # noqa: E402

AIR = ("WANDER", "TURN", "BACKOFF", "HOLD", "UNSTICK", "EXIT", "PLAN", "FOLLOW", "REPLAN", "LOOK", "RELOC", "HOMESPIN")


def iso(t: str) -> float:
    return datetime.fromisoformat(t).timestamp()


def jsonl(path: Path) -> list[dict]:
    return [json.loads(line) for line in path.open(encoding="utf-8")] if path.exists() else []


def wrap_deg(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def main() -> int:
    ap = argparse.ArgumentParser(description="실기체 비행 기록 정리")
    ap.add_argument("out_dir", type=Path)
    ap.add_argument("--no-video", action="store_true", help="video.mp4를 만들지 않는다")
    args = ap.parse_args()
    d = args.out_dir.expanduser()
    lines: list[str] = [f"# 비행 기록 {d.name}", ""]
    add = lines.append

    run = json.loads((d / "run.json").read_text(encoding="utf-8")) if (d / "run.json").exists() else {}
    a = run.get("args", {})
    vx_sign = float(a.get("vx_sign", -1.0))
    if run:
        keys = ("mode", "map", "speed", "search_time", "flight_height", "video", "rc_limit", "v_full", "yaw_full_deg",
                "vx_sign", "yaw_sign", "vel_scale", "vel_frame")
        add(f"- 시작 {run['started']}, 코드 지문 `{run['code']['sha1']}` (가장 최근 파일 {run['code']['newest']}, "
            f"{run['code']['newest_mtime']})")
        add("- 인자: " + ", ".join(f"`{k}:={a[k]}`" for k in keys if a.get(k) not in (None, "")))
    else:
        add("- run.json 없음 (2026-10-28 이전 기록). vx_sign은 -1로 본다")

    logs = sorted((d / "tello_log").glob("*/commands.jsonl"))
    cmds = jsonl(logs[0]) if logs else []
    sens = jsonl(logs[0].parent / "sensor.jsonl") if logs else []
    events = jsonl(logs[0].parent / "events.jsonl") if logs else []
    for c in cmds:
        c["_t"] = iso(c["sent_at"])
    for s in sens:
        s["_t"] = iso(s["received_at"])

    # ---- 이륙·착륙·단계
    add("")
    add("## 흐름")
    for c in cmds:
        if c["cmd"] in ("takeoff", "land", "emergency"):
            rtt = f", {c['rtt_ms'] / 1000:.1f} s" if c.get("rtt_ms") else ""
            add(f"- {c['sent_at'][11:19]} UTC `{c['cmd']}` → {c['response']}{rtt} ({c.get('source')})")
    for e in events:
        if e["type"] not in ("link_started", "operator_confirm"):
            add(f"- {e['t'][11:19]} UTC 사건 `{e['type']}` {json.dumps(e.get('detail'), ensure_ascii=False)[:120]}")
    fl = list(csv.DictReader((d / "flight_log.csv").open(encoding="utf-8"))) if (d / "flight_log.csv").exists() else []
    if fl:
        seq, last, t0 = [], None, float(fl[0]["t"])
        for r in fl:
            if r["state"] != last:
                seq.append(f"{float(r['t']) - t0:.0f}s {r['state']}")
                last = r["state"]
        add("- 단계: " + " → ".join(seq))

    # ---- 실제 속도 (상태값) vs 보낸 rc
    rc = [(c["_t"], list(map(int, c["cmd"].split()[1:]))) for c in cmds if c["cmd"].startswith("rc ")]

    def rc_at(t: float):
        lo, hi, best = 0, len(rc) - 1, None
        while lo <= hi:
            mid = (lo + hi) // 2
            if rc[mid][0] <= t:
                best, lo = rc[mid][1], mid + 1
            else:
                hi = mid - 1
        return best

    add("")
    add("## 실제로 낸 속도")
    if rc and sens:
        # 앞으로 가는 속도 = (vgx, vgy)를 기수 방향에 내린 것. Tello 속도값은 yaw 0 방향 기준이고 앞으로 날 때
        # (vgx, vgy) ≈ −속도·(cos yaw, sin yaw)였다 (2026-10-29, autonomy/tello_state.py). 돌아선 뒤에도 맞게 잰다
        def forward(r):
            ps = math.radians(float(r["yaw"]))
            return vx_sign * (r["vgx"] * math.cos(ps) + r["vgy"] * math.sin(ps)) / 10.0
        fwd = [(forward(s["raw"]), v[1]) for s in sens if (v := rc_at(s["_t"])) and v[1] > 5 and abs(v[3]) <= 5]
        if fwd:
            speed = statistics.mean(v for v, _ in fwd)
            fb = statistics.mean(f for _, f in fwd)
            add(f"- 앞으로(rc fb 평균 {fb:.0f}) {len(fwd) / 10:.1f} s 동안 상태값 속도 평균 {speed:.2f} m/s "
                f"→ rc 100 환산 **v_full ≈ {speed / (fb / 100):.2f} m/s** (지금 값 {a.get('v_full', '1.0(옛 기본)')})")
            add("  - 상태값 속도는 0.1 m/s 단위라 느릴 때 거칠다. 줄자로 잰 거리 ÷ 시간이 더 정확하다")
        turns, prev = [], None
        for s in sens:
            v = rc_at(s["_t"])
            if v and abs(v[3]) > 10 and prev and s["_t"] - prev[0] < 0.5:
                turns.append((wrap_deg(s["raw"]["yaw"] - prev[1]) / (s["_t"] - prev[0]), abs(v[3])))
            prev = (s["_t"], s["raw"]["yaw"])
        if turns:
            rate = statistics.mean(abs(r) for r, _ in turns)
            yr = statistics.mean(y for _, y in turns)
            add(f"- 회전(rc yaw 평균 {yr:.0f}) {len(turns) / 10:.1f} s 동안 {rate:.0f}°/s "
                f"→ rc 100 환산 **yaw_full_deg ≈ {rate / (yr / 100):.0f}** (지금 값 {a.get('yaw_full_deg', '100(옛 기본)')})")
        mx = [max(abs(v[k]) for _, v in rc) for k in range(4)]
        add(f"- 보낸 rc 최대 |좌우 앞뒤 상하 회전| = {mx}, rc {len(rc)}개")
    else:
        add("- rc 또는 상태값 기록이 없다")
    if fl:
        r = dwell(d / "flight_log.csv")
        air = r["air"] or 1e-9
        add(f"- 떠 있는 동안 {air:.0f} s: 움직임 {r['move'] / air:.0%}, 제자리 회전 {r['turn'] / air:.0%}, "
            f"정지 {r['still'] / air:.0%} (가장 긴 정지 {r['longest_still']:.1f} s)")
        pts = [(float(r["x"]), float(r["y"])) for r in fl if r["state"] in AIR]
        if pts:
            dist = sum(math.dist(p, q) for p, q in zip(pts, pts[1:]))
            add(f"- 추정 이동 거리 {dist:.2f} m (속도 적분이라 배율·부호가 틀리면 같이 틀린다)")

    # ---- 기체 상태
    add("")
    add("## 기체")
    if sens:
        r0, r1 = sens[0]["raw"], sens[-1]["raw"]
        flying = [s["raw"] for s in sens if s["raw"]["tof"] > 30]
        add(f"- 상태값 {len(sens)}줄, {sens[0]['received_at'][11:19]}–{sens[-1]['received_at'][11:19]} UTC")
        add(f"- 배터리 {r0['bat']} → {r1['bat']}%, 온도 {r0['templ']}–{r0['temph']} → 최고 "
            f"{max(s['raw']['temph'] for s in sens)}°C")
        if flying:
            tofs = [f["tof"] for f in flying]
            add(f"- 떠 있을 때 하방 ToF {min(tofs)}–{max(tofs)} cm (중앙 {statistics.median(tofs):.0f})")
    ext = [c for c in cmds if c["cmd"].startswith("EXT")]
    if ext:
        bad = [c for c in ext if not str(c["response"]).startswith("tof")]
        rtt = [c["rtt_ms"] for c in ext if c.get("rtt_ms")]
        add(f"- 전방 ToF 질의 {len(ext)}번, 이상 응답 {len(bad)}번, 응답 중앙 {statistics.median(rtt):.0f} ms" if rtt else
            f"- 전방 ToF 질의 {len(ext)}번")
    found = json.loads((d / "discovered.json").read_text(encoding="utf-8")).get("obstacles", []) \
        if (d / "discovered.json").exists() else []
    add(f"- 찾은 장애물 {len(found)}개")

    # ---- 영상·bag
    add("")
    add("## 영상·기록")
    vf = d / "video" / "video_frames.csv"
    if vf.exists():
        fr = list(csv.DictReader(vf.open(encoding="utf-8")))
        if len(fr) > 1:
            dur = float(fr[-1]["wall_time"]) - float(fr[0]["wall_time"])
            add(f"- 영상 {len(fr)}프레임, {dur:.0f} s ({len(fr) / max(dur, 1e-6):.1f} fps), "
                f"video.h264 {(d / 'video' / 'video.h264').stat().st_size / 1e6:.1f} MB")
        mp4 = d / "video" / "video.mp4"
        if not args.no_video and not mp4.exists():
            try:
                remux(d / "video" / "video.h264", mp4, fps=len(fr) / max(dur, 1e-6) if len(fr) > 1 else 25)
                add(f"- video.mp4 만듦 ({mp4.stat().st_size / 1e6:.1f} MB)")
            except ImportError:
                add("- video.mp4는 PyAV가 있는 파이썬으로: `~/venvs/tt/bin/python tools/flight_report.py <폴더>` (h264는 VLC로 바로 봄)")
    else:
        add("- 영상 저장 없음 (video:=false였거나 2026-10-28 이전 기록)")
    meta = d / "bag" / "metadata.yaml"
    if meta.exists():
        counts = bag_counts(meta)
        add(f"- bag: 토픽 {len(counts)}개 — " + ", ".join(f"{k} {v}" for k, v in sorted(counts.items(), key=lambda kv: -kv[1])[:10]))
        add("  - 요약 `ros2 bag info bag`, 다시 틀기 `ros2 bag play bag` (/rosout에 모든 노드 로그 줄이 있다)")
    else:
        add("- bag 없음")

    # ---- 경로 그림
    if fl:
        plot(d, fl, found, run)
        add("")
        add("![경로](path.png)")
    (d / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines))
    print(f"\n→ {d / 'report.md'}")
    return 0


def bag_counts(meta: Path) -> dict[str, int]:
    """metadata.yaml에서 토픽별 메시지 수 (yaml 모듈 없이 줄로 읽는다)."""
    counts, name = {}, None
    for line in meta.read_text(encoding="utf-8").splitlines():
        s = line.strip()
        if s.startswith("name:") and "/" in s:
            name = s.split(":", 1)[1].strip()
        elif s.startswith("message_count:") and name:
            counts[name] = int(s.split(":", 1)[1])
            name = None
    return counts


def remux(src: Path, dst: Path, fps: float) -> None:
    """받은 H.264를 다시 인코딩하지 않고 mp4에 담는다."""
    import av
    from fractions import Fraction

    with av.open(str(src), format="h264") as inp, av.open(str(dst), "w") as out:
        vin = inp.streams.video[0]
        vout = out.add_stream_from_template(vin) if hasattr(out, "add_stream_from_template") else out.add_stream(template=vin)
        tb = Fraction(1, 90000)
        vout.time_base = tb
        step = int(round(90000 / max(fps, 1.0)))
        for i, pkt in enumerate(inp.demux(vin)):
            if pkt.size == 0:
                continue
            pkt.pts = pkt.dts = i * step
            pkt.time_base = tb
            pkt.stream = vout
            out.mux(pkt)


def plot(d: Path, fl: list[dict], found: list[dict], run: dict) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    mp = d / "map.json"
    if not mp.exists() and run.get("map") and Path(run["map"]).exists():
        mp = Path(run["map"])
    fig, ax = plt.subplots(figsize=(7, 7))
    # 그림 글자는 영어로 둔다 (WSL에 한글 글꼴이 없으면 네모로 나온다)
    if mp.exists():
        m = json.loads(mp.read_text(encoding="utf-8"))
        b = m["bounds"]
        ax.add_patch(Rectangle((b["xmin"], b["ymin"]), b["xmax"] - b["xmin"], b["ymax"] - b["ymin"], fill=False, color="0.4"))
        for o in m.get("walls", []) + m.get("obstacles", []):
            ax.add_patch(Rectangle((o["x"] - o["sx"] / 2, o["y"] - o["sy"] / 2), o["sx"], o["sy"], color="0.8"))
        s = m["spawn"]
        ax.plot(s["x"], s["y"], "k^", ms=10, label="start")
        ax.arrow(s["x"], s["y"], 0.4 * math.cos(s["yaw"]), 0.4 * math.sin(s["yaw"]), head_width=0.1, color="k")
    for o in found:
        ax.add_patch(Rectangle((o["x"] - o["sx"] / 2, o["y"] - o["sy"] / 2), o["sx"], o["sy"], color="tab:red", alpha=0.5))
    colors = {"WANDER": "tab:blue", "FOLLOW": "tab:blue", "TURN": "tab:orange", "LOOK": "tab:orange",
              "BACKOFF": "tab:purple", "HOLD": "tab:red", "UNSTICK": "tab:red", "EXIT": "tab:pink", "LAND": "tab:green", "TAKEOFF": "tab:green"}
    seg, state = [], None
    for r in fl:
        if r["state"] in ("WAIT_POSE", "READY", "DONE", "ABORT"):
            continue
        p = (float(r["x"]), float(r["y"]))
        if r["state"] != state and seg:
            ax.plot(*zip(*seg), color=colors.get(state, "0.3"), lw=2)
            seg = [seg[-1]]
        seg.append(p)
        state = r["state"]
    if seg:
        ax.plot(*zip(*seg), color=colors.get(state, "0.3"), lw=2)
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title(f"{d.name}: estimated path (blue move, orange turn, green takeoff/land, red found obstacle)",
                 fontsize=9)
    fig.tight_layout()
    fig.savefig(d / "path.png", dpi=110)
    plt.close(fig)


if __name__ == "__main__":
    sys.exit(main())
