"""Tello 회차 기록 요약 (표준 라이브러리만 사용).

tello_link가 남긴 회차 폴더(data/outputs/tello/<run_id>/) 하나를 읽어 비행 요약을 낸다.

    python modules/autonomy/tello_link/flight_report.py data/outputs/tello/<run_id>
    python modules/autonomy/tello_link/flight_report.py data/outputs/tello/<run_id> --json

읽는 파일: manifest.json, sensor.jsonl, commands.jsonl, events.jsonl, metrics.json.
어느 파일이 없어도 동작한다(강제 종료한 회차에는 metrics.json이 없다). 깨진 줄은 건너뛰고 개수만 센다.

판단 기준
- 구간: 하방 tof가 유효한 동안을 비행, 나머지를 지상으로 본다. 손으로 들어 올려도 비행으로 잡힌다.
  비행 중 tof가 MERGE_GAP_S보다 짧게 끊긴 것은 같은 비행으로 합친다.
- 배터리 %/분: 구간 안 처음·끝 값의 차이를 구간 길이로 나눈다(음수 = 감소). 배터리는 정수 %라 짧은 구간 값은 거칠다.
- 높이: 하방 tof 최고와 같은 구간의 상태 `h` 최고를 나란히 보인다(h는 믿기 어렵다).
- rc 전송 간격: RC_GROUP_GAP_S보다 길게 끊기면 다른 묶음으로 보고, 간격은 묶음 안에서만 잰다.
- 조회 명령(battery?, EXT tof? 등)은 하나씩 나열하지 않고 명령별 개수·실패·응답 시간만 낸다.
- 모르는 값은 "없음"(JSON은 null)으로 둔다. 0이나 지어낸 값으로 채우지 않는다.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from collections import Counter
from datetime import datetime

MERGE_GAP_S = 1.0
RC_GROUP_GAP_S = 2.0
RC_AXES = ("lr", "fb", "ud", "yaw")
NONE = "없음"


# ---------- 읽기 ----------
def parse_time(s) -> float | None:
    if not isinstance(s, str):
        return None
    try:
        return datetime.fromisoformat(s.replace("Z", "+00:00")).timestamp()
    except ValueError:
        return None


def read_json(path: str):
    try:
        with open(path, encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def read_jsonl(path: str) -> tuple[list[dict], int, bool]:
    """(행 목록, 깨진 줄 수, 파일 있음)을 돌려준다."""
    if not os.path.exists(path):
        return [], 0, False
    rows, bad = [], 0
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                row = json.loads(line)
            except ValueError:
                bad += 1
                continue
            if isinstance(row, dict):
                rows.append(row)
            else:
                bad += 1
    return rows, bad, True


def _num(v) -> float | None:
    return v if isinstance(v, (int, float)) and not isinstance(v, bool) else None


def _round(v, nd=2):
    return None if v is None else round(v, nd)


# ---------- 구간 ----------
def split_phases(samples: list[tuple[float, dict]], merge_gap_s: float = MERGE_GAP_S) -> list[dict]:
    """(시각, si) 목록을 하방 tof 유효 여부로 지상·비행 구간으로 나눈다. 각 구간은 samples의 [i0, i1] 범위."""
    runs: list[list] = []  # [flying, i0, i1]
    for i, (_, si) in enumerate(samples):
        flying = si.get("tof_valid") is True
        if runs and runs[-1][0] == flying:
            runs[-1][2] = i
        else:
            runs.append([flying, i, i])
    # 비행 사이의 짧은 끊김은 비행으로 합친다(양 끝 지상 구간은 그대로 둔다).
    merged: list[list] = []
    for k, run in enumerate(runs):
        flying, i0, i1 = run
        if (not flying and merged and merged[-1][0] and k + 1 < len(runs)
                and samples[i1 + 1][0] - samples[i0 - 1][0] < merge_gap_s):
            merged[-1][2] = i1
            continue
        if merged and merged[-1][0] == flying:
            merged[-1][2] = i1
        else:
            merged.append([flying, i0, i1])

    n_flights = sum(1 for r in merged if r[0])
    phases, flight_no, seen_flight = [], 0, False
    for flying, i0, i1 in merged:
        if flying:
            flight_no += 1
            seen_flight = True
            label = "비행" if n_flights == 1 else f"비행 {flight_no}"
        elif n_flights == 0:
            label = "지상"
        elif not seen_flight:
            label = "지상 대기"
        elif flight_no == n_flights:
            label = "착륙 후"
        else:
            label = "지상(비행 사이)"
        phases.append({"kind": "flight" if flying else "ground", "label": label, "i0": i0, "i1": i1})
    return phases


def phase_stats(phase: dict, samples: list[tuple[float, dict]], t0: float | None) -> dict:
    part = samples[phase["i0"]:phase["i1"] + 1]
    ts = [t for t, _ in part]
    bats = [_num(si.get("battery_pct")) for _, si in part]
    bats = [b for b in bats if b is not None]
    temps = [_num(si.get("temp_high_c")) for _, si in part]
    temps = [x for x in temps if x is not None]
    duration = ts[-1] - ts[0]
    rate = None
    if bats and duration > 0:
        rate = (bats[-1] - bats[0]) / duration * 60.0
    out = {
        "kind": phase["kind"],
        "label": phase["label"],
        "start_rel_s": _round(ts[0] - t0, 1) if t0 is not None else None,
        "end_rel_s": _round(ts[-1] - t0, 1) if t0 is not None else None,
        "duration_s": round(duration, 1),
        "samples": len(part),
        "battery_start_pct": bats[0] if bats else None,
        "battery_end_pct": bats[-1] if bats else None,
        "battery_rate_pct_per_min": _round(rate),
        "temp_start_c": temps[0] if temps else None,
        "temp_end_c": temps[-1] if temps else None,
    }
    if phase["kind"] == "flight":
        tofs = [_num(si.get("tof_m")) for _, si in part if si.get("tof_valid") is True]
        tofs = [x for x in tofs if x is not None]
        hs = [_num(si.get("height_m")) for _, si in part]
        hs = [x for x in hs if x is not None]
        tof_max = max(tofs) if tofs else None
        h_max = max(hs) if hs else None
        out["tof_max_m"] = tof_max
        out["h_max_m"] = h_max
        out["tof_minus_h_m"] = _round(tof_max - h_max) if tof_max is not None and h_max is not None else None
    return out


# ---------- 명령 ----------
def _cmd_time(row: dict) -> float | None:
    return parse_time(row.get("sent_at")) or parse_time(row.get("recorded_at"))


def parse_rc(cmd: str) -> tuple[int, int, int, int] | None:
    parts = cmd.split()
    if len(parts) != 5:
        return None
    try:
        return tuple(int(p) for p in parts[1:])  # type: ignore[return-value]
    except ValueError:
        return None


def rc_stats(rows: list[dict]) -> dict:
    by_source = Counter(str(r.get("source")) if r.get("source") is not None else NONE for r in rows)
    axes = {a: {"nonzero": 0, "max_abs": None} for a in RC_AXES}
    unparsed = 0
    for r in rows:
        vals = parse_rc(r.get("cmd", ""))
        if vals is None:
            unparsed += 1
            continue
        for a, v in zip(RC_AXES, vals):
            if v != 0:
                axes[a]["nonzero"] += 1
            m = axes[a]["max_abs"]
            axes[a]["max_abs"] = abs(v) if m is None else max(m, abs(v))

    times = sorted(t for t in (parse_time(r.get("sent_at")) for r in rows if not r.get("blocked")) if t is not None)
    intervals, groups = [], 0
    for k, t in enumerate(times):
        if k == 0 or t - times[k - 1] > RC_GROUP_GAP_S:
            groups += 1
        else:
            intervals.append(t - times[k - 1])
    return {
        "count": len(rows),
        "blocked": sum(1 for r in rows if r.get("blocked")),
        "unparsed": unparsed,
        "by_source": dict(by_source.most_common()),
        "axes": axes,
        "groups": groups,
        "interval_mean_ms": _round(sum(intervals) / len(intervals) * 1000, 1) if intervals else None,
        "interval_max_ms": _round(max(intervals) * 1000, 1) if intervals else None,
    }


def query_stats(rows: list[dict]) -> list[dict]:
    out: dict[str, dict] = {}
    for r in rows:
        q = out.setdefault(r.get("cmd", ""), {"cmd": r.get("cmd", ""), "count": 0, "failed": 0, "rtts": []})
        q["count"] += 1
        if not r.get("ok"):
            q["failed"] += 1
        if _num(r.get("rtt_ms")) is not None:
            q["rtts"].append(r["rtt_ms"])
    result = []
    for q in out.values():
        rtts = q.pop("rtts")
        q["rtt_mean_ms"] = _round(sum(rtts) / len(rtts), 1) if rtts else None
        q["rtt_max_ms"] = max(rtts) if rtts else None
        result.append(q)
    return result


# ---------- 요약 ----------
def build_report(run_dir: str) -> dict:
    manifest = read_json(os.path.join(run_dir, "manifest.json"))
    metrics_path = os.path.join(run_dir, "metrics.json")
    metrics = read_json(metrics_path)
    sensor_rows, sensor_bad, sensor_ok = read_jsonl(os.path.join(run_dir, "sensor.jsonl"))
    cmd_rows, cmd_bad, cmd_ok = read_jsonl(os.path.join(run_dir, "commands.jsonl"))
    event_rows, event_bad, event_ok = read_jsonl(os.path.join(run_dir, "events.jsonl"))
    manifest = manifest if isinstance(manifest, dict) else {}

    samples = []
    for r in sensor_rows:
        t = parse_time(r.get("received_at"))
        if t is not None:
            samples.append((t, r.get("si") or {}))
    samples.sort(key=lambda x: x[0])

    all_times = [t for t, _ in samples]
    all_times += [t for t in (_cmd_time(r) for r in cmd_rows) if t is not None]
    all_times += [t for t in (parse_time(e.get("t")) for e in event_rows) if t is not None]
    t0 = parse_time(manifest.get("created_at"))
    if t0 is None and all_times:
        t0 = min(all_times)
    t_end = max(all_times) if all_times else None

    def rel(t):
        return _round(t - t0, 1) if t is not None and t0 is not None else None

    # 상태 전체
    bats = [b for b in (_num(si.get("battery_pct")) for _, si in samples) if b is not None]
    temps = [x for x in (_num(si.get("temp_high_c")) for _, si in samples) if x is not None]
    span = samples[-1][0] - samples[0][0] if len(samples) > 1 else 0.0
    phases = [phase_stats(p, samples, t0) for p in split_phases(samples)] if samples else []

    # 명령
    rc_rows = [r for r in cmd_rows if str(r.get("cmd", "")).startswith("rc ")]
    other = [r for r in cmd_rows if not str(r.get("cmd", "")).startswith("rc ")]
    queries = [r for r in other if r.get("category") == "query"]
    listed = sorted((r for r in other if r.get("category") != "query"), key=lambda r: _cmd_time(r) or 0.0)
    commands = [{
        "rel_s": rel(_cmd_time(r)),
        "at": r.get("sent_at") or r.get("recorded_at"),
        "cmd": r.get("cmd"),
        "source": r.get("source"),
        "ok": r.get("ok"),
        "response": r.get("response"),
        "rtt_ms": r.get("rtt_ms"),
        "blocked": bool(r.get("blocked")),
        "reason": r.get("reason"),
    } for r in listed]

    # 이벤트
    def ev(e, **fields):
        return {"rel_s": rel(parse_time(e.get("t"))), "at": e.get("t"), **fields}

    types = Counter(str(e.get("type")) for e in event_rows)
    mode_changes, safety, refused, emergencies, errors = [], [], [], [], []
    for e in event_rows:
        kind, d = e.get("type"), e.get("detail") or {}
        if kind == "gateway_mode_changed":
            mode_changes.append(ev(e, **{k: d.get(k) for k in ("from", "to", "by", "reason")}))
        elif kind == "auto_land":
            safety.append(ev(e, reason=d.get("reason")))
        elif kind == "takeoff_refused":
            refused.append(ev(e, failed=d.get("failed"), flying=d.get("flying"), busy=d.get("busy")))
        elif kind == "emergency":
            emergencies.append(ev(e, source=d.get("source")))
        elif str(kind).endswith("_error"):
            errors.append(ev(e, type=kind, error=d.get("error")))
    shown = {"gateway_mode_changed", "auto_land", "takeoff_refused", "emergency"}
    other_events = {k: v for k, v in types.items() if k not in shown and not k.endswith("_error")}

    return {
        "run_dir": os.path.abspath(run_dir),
        "run": {
            "run_id": manifest.get("run_id") or os.path.basename(os.path.normpath(run_dir)),
            "source": manifest.get("source"),
            "operator_controlled": manifest.get("operator_controlled"),
            "created_at": manifest.get("created_at"),
            "duration_s": _round(t_end - t0, 1) if t_end is not None and t0 is not None else None,
            "metrics_present": metrics is not None,
            "metrics": metrics,
            "files": {"manifest": bool(manifest), "sensor": sensor_ok, "commands": cmd_ok, "events": event_ok},
            "bad_lines": {"sensor": sensor_bad, "commands": cmd_bad, "events": event_bad},
        },
        "state": {
            "samples": len(samples),
            "rate_hz": _round((len(samples) - 1) / span, 1) if span > 0 else None,
            "battery_first_pct": bats[0] if bats else None,
            "battery_last_pct": bats[-1] if bats else None,
            "temp_first_c": temps[0] if temps else None,
            "temp_last_c": temps[-1] if temps else None,
            "temp_max_c": max(temps) if temps else None,
        },
        "phases": phases,
        "commands": commands,
        "takeoff_rtt_ms": [c["rtt_ms"] for c in commands if c["cmd"] == "takeoff"],
        "land_rtt_ms": [c["rtt_ms"] for c in commands if c["cmd"] == "land"],
        "queries": query_stats(queries),
        "rc": rc_stats(rc_rows),
        "events": {
            "mode_changes": mode_changes,
            "auto_land": safety,
            "safety_land_commands": [c for c in commands if c["cmd"] == "land"
                                     and str(c["source"] or "").startswith("safety:")],
            "takeoff_refused": refused,
            "emergency": emergencies,
            "errors": errors,
            "other_counts": other_events,
        },
    }


# ---------- 텍스트 ----------
def f(v, unit: str = "", nd: int | None = None) -> str:
    if v is None:
        return NONE
    if isinstance(v, bool):
        return "예" if v else "아니오"
    if nd is not None and isinstance(v, (int, float)):
        v = f"{v:.{nd}f}"
    return f"{v}{unit}"


def _at(item: dict) -> str:
    t = parse_time(item.get("at"))
    clock = datetime.fromtimestamp(t).strftime("%H:%M:%S.%f")[:-3] if t is not None else NONE
    return f"+{f(item.get('rel_s'), 's', 1):>8} ({clock})"


def format_text(rep: dict) -> str:
    run, st, rc, evs = rep["run"], rep["state"], rep["rc"], rep["events"]
    L = [f"# 회차 {run['run_id']}", f"폴더: {rep['run_dir']}"]
    missing = [k for k, ok in run["files"].items() if not ok]
    L.append(f"소스 {f(run['source'])} · 사람 조종 {f(run['operator_controlled'])} · "
             f"기록 길이 {f(run['duration_s'], '초', 1)}")
    L.append("metrics.json " + ("있음" if run["metrics_present"] else "없음(강제 종료 의심)")
             + (f" · 없는 파일: {', '.join(missing)}" if missing else ""))
    bad = {k: v for k, v in run["bad_lines"].items() if v}
    if bad:
        L.append("깨진 줄: " + ", ".join(f"{k} {v}" for k, v in bad.items()))

    L += ["", "## 상태",
          f"상태 패킷 {st['samples']}개, 평균 {f(st['rate_hz'], 'Hz')}",
          f"배터리 {f(st['battery_first_pct'], '%')} → {f(st['battery_last_pct'], '%')}",
          f"온도(높은 쪽) {f(st['temp_first_c'], '°C')} → {f(st['temp_last_c'], '°C')}, 최고 {f(st['temp_max_c'], '°C')}"]

    L += ["", "## 구간 (하방 tof 유효 = 비행, 손으로 들어도 비행으로 잡힌다)"]
    if not rep["phases"]:
        L.append(NONE)
    for p in rep["phases"]:
        L.append(f"- {p['label']}: +{f(p['start_rel_s'], 's', 1)} ~ +{f(p['end_rel_s'], 's', 1)} "
                 f"({f(p['duration_s'], '초', 1)}) · 배터리 {f(p['battery_start_pct'], '%')} → "
                 f"{f(p['battery_end_pct'], '%')} ({f(p['battery_rate_pct_per_min'], ' %/분', 2)}) · "
                 f"온도 {f(p['temp_start_c'], '°C')} → {f(p['temp_end_c'], '°C')}")
        if p["kind"] == "flight":
            L.append(f"    높이 최고: 하방 tof {f(p['tof_max_m'], 'm', 2)}, h {f(p['h_max_m'], 'm', 2)} "
                     f"(tof - h = {f(p['tof_minus_h_m'], 'm', 2)})")

    L += ["", "## 명령 (rc·조회 제외)"]
    if not rep["commands"]:
        L.append(NONE)
    for c in rep["commands"]:
        extra = " 막힘" if c["blocked"] else ""
        if c["reason"] and not c["ok"]:
            extra += f" ({c['reason']})"
        L.append(f"- {_at(c)} {c['cmd']:<10} source={f(c['source'])} ok={f(c['ok'])} "
                 f"응답={f(c['response'])} {f(c['rtt_ms'], 'ms')}{extra}")
    L.append(f"takeoff 응답: {', '.join(f(v, 'ms') for v in rep['takeoff_rtt_ms']) or NONE} · "
             f"land 응답: {', '.join(f(v, 'ms') for v in rep['land_rtt_ms']) or NONE}")
    if rep["queries"]:
        L.append("조회: " + ", ".join(f"{q['cmd']} {q['count']}회(실패 {q['failed']}, 평균 {f(q['rtt_mean_ms'], 'ms')}, "
                                      f"최대 {f(q['rtt_max_ms'], 'ms')})" for q in rep["queries"]))

    L += ["", "## rc",
          f"{rc['count']}개 (막힘 {rc['blocked']}" + (f", 해석 못 함 {rc['unparsed']}" if rc["unparsed"] else "") + ")",
          "출처: " + (", ".join(f"{k} {v}" for k, v in rc["by_source"].items()) or NONE),
          "축(0 아닌 횟수 / |최대|): " + ", ".join(
              f"{a} {rc['axes'][a]['nonzero']} / {f(rc['axes'][a]['max_abs'])}" for a in RC_AXES),
          f"전송 간격(묶음 {rc['groups']}개 안에서): 평균 {f(rc['interval_mean_ms'], 'ms')}, "
          f"최대 {f(rc['interval_max_ms'], 'ms')}"]

    L += ["", "## 이벤트"]
    L.append("모드 전환:" + ("" if evs["mode_changes"] else f" {NONE}"))
    for m in evs["mode_changes"]:
        L.append(f"- {_at(m)} {f(m['from'])} → {f(m['to'])} (by {f(m['by'])}, {f(m['reason'])})")
    L.append("안전 착륙:" + ("" if evs["auto_land"] or evs["safety_land_commands"] else f" {NONE}"))
    for a in evs["auto_land"]:
        L.append(f"- {_at(a)} auto_land {f(a['reason'])}")
    for c in evs["safety_land_commands"]:
        L.append(f"- {_at(c)} land source={c['source']} ok={f(c['ok'])}")
    L.append("이륙 거절:" + ("" if evs["takeoff_refused"] else f" {NONE}"))
    for r in evs["takeoff_refused"]:
        L.append(f"- {_at(r)} 실패 항목 {f(r['failed'])} (flying={f(r['flying'])}, busy={f(r['busy'])})")
    if evs["emergency"]:
        L.append("비상 정지:")
        L += [f"- {_at(e)} source={f(e['source'])}" for e in evs["emergency"]]
    if evs["errors"]:
        L.append("오류:")
        L += [f"- {_at(e)} {e['type']}: {f(e['error'])}" for e in evs["errors"]]
    if evs["other_counts"]:
        L.append("그 밖: " + ", ".join(f"{k} {v}" for k, v in evs["other_counts"].items()))
    return "\n".join(L)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Tello 회차 기록 폴더 하나를 요약한다")
    ap.add_argument("run_dir", help="data/outputs/tello/<run_id> 폴더")
    ap.add_argument("--json", action="store_true", help="JSON으로 출력")
    args = ap.parse_args(argv)
    if not os.path.isdir(args.run_dir):
        print(f"폴더가 없다: {args.run_dir}", file=sys.stderr)
        return 2
    rep = build_report(args.run_dir)
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(errors="backslashreplace")  # cp949 콘솔에서 못 찍는 글자 때문에 멈추지 않게
    print(json.dumps(rep, ensure_ascii=False, indent=2) if args.json else format_text(rep))
    return 0


if __name__ == "__main__":
    sys.exit(main())
