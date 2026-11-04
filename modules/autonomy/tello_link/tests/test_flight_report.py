"""flight_report 요약 계산을 작은 가짜 회차 폴더와 가짜 Tello 회차로 시험한다 (표준 라이브러리만).

    python -B -m unittest discover -s modules/autonomy/tello_link/tests -v
"""

import contextlib
import io
import json
import os
import sys
import tempfile
import time
import unittest
from datetime import datetime, timedelta, timezone

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import flight_report as fr  # noqa: E402
from fake_tello import FakeTello  # noqa: E402
from flight_control import FlightController  # noqa: E402
from tello_link import TelloLink  # noqa: E402

BASE = datetime(2026, 10, 28, 0, 0, 0, tzinfo=timezone.utc)


def at(s: float) -> str:
    return (BASE + timedelta(seconds=s)).isoformat(timespec="milliseconds")


def sample(t: float, bat: int, temp: int, tof_m: float | None, h_m: float = 0.0) -> dict:
    return {"received_at": at(t), "si": {"battery_pct": bat, "temp_high_c": temp, "height_m": h_m,
                                         "tof_m": tof_m, "tof_valid": tof_m is not None}}


def cmd(t: float | None, c: str, source: str, rtt=None, ok=True, category="flight", blocked=False) -> dict:
    return {"cmd": c, "category": category, "sent_at": at(t) if t is not None else None,
            "response": "ok" if rtt is not None else None, "ok": ok, "rtt_ms": rtt, "blocked": blocked,
            "source": source, "recorded_at": at(t if t is not None else 0)}


def write_run(root: str, sensor=(), commands=(), events=(), manifest=True, extra_lines=()) -> str:
    run = os.path.join(root, "tello_test_run")
    os.makedirs(run, exist_ok=True)
    if manifest:
        with open(os.path.join(run, "manifest.json"), "w", encoding="utf-8") as f:
            json.dump({"run_id": "tello_test_run", "created_at": at(0), "source": "fake",
                       "operator_controlled": True}, f)
    for name, rows in (("sensor", sensor), ("commands", commands), ("events", events)):
        with open(os.path.join(run, f"{name}.jsonl"), "w", encoding="utf-8") as f:
            for r in rows:
                f.write(json.dumps(r, ensure_ascii=False) + "\n")
            if name == "events":
                f.writelines(extra_lines)
    return run


def one_flight_sensor() -> list[dict]:
    """지상 0~59.9초, 비행 60~89.9초(70~70.4초 tof 끊김), 착륙 후 90~119.9초. 10Hz."""
    rows = []
    for i in range(1200):
        t = i / 10
        if t < 60:
            rows.append(sample(t, 60 if t < 30 else 58, 60 + int(t / 12), None))
        elif t < 90:
            dropout = 70 <= t < 70.5
            rows.append(sample(t, 58 if t < 75 else 50, 65, None if dropout else (1.2 if t == 80 else 1.0),
                               h_m=0.4 if t == 85 else 0.3))
        else:
            rows.append(sample(t, 50 if t < 100 else 48, 67, None))
    return rows


class SyntheticRunTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()

    def tearDown(self):
        self.tmp.cleanup()

    def test_phases_battery_height(self):
        run = write_run(self.tmp.name, sensor=one_flight_sensor())
        rep = fr.build_report(run)
        self.assertEqual([p["label"] for p in rep["phases"]], ["지상 대기", "비행", "착륙 후"])
        ground, flight, after = rep["phases"]
        self.assertAlmostEqual(ground["duration_s"], 59.9)
        self.assertAlmostEqual(ground["battery_rate_pct_per_min"], round(-2 / 59.9 * 60, 2))
        self.assertEqual((flight["start_rel_s"], flight["end_rel_s"]), (60.0, 89.9))  # 짧은 끊김은 합친다
        self.assertAlmostEqual(flight["battery_rate_pct_per_min"], round(-8 / 29.9 * 60, 2))
        self.assertEqual((after["battery_start_pct"], after["battery_end_pct"]), (50, 48))
        self.assertEqual((flight["tof_max_m"], flight["h_max_m"], flight["tof_minus_h_m"]), (1.2, 0.4, 0.8))
        self.assertNotIn("tof_max_m", ground)
        st = rep["state"]
        self.assertEqual((st["battery_first_pct"], st["battery_last_pct"]), (60, 48))
        self.assertEqual((st["temp_first_c"], st["temp_last_c"], st["temp_max_c"]), (60, 67, 67))
        self.assertEqual(st["rate_hz"], 10.0)

    def test_two_flights(self):
        rows = []
        for i in range(300):
            t = i / 10
            flying = 5 <= t < 10 or 15 <= t < 20
            rows.append(sample(t, 80, 60, 1.0 if flying else None))
        rep = fr.build_report(write_run(self.tmp.name, sensor=rows))
        self.assertEqual([p["label"] for p in rep["phases"]],
                         ["지상 대기", "비행 1", "지상(비행 사이)", "비행 2", "착륙 후"])

    def test_commands_rc_events(self):
        commands = [
            cmd(0.1, "command", "gateway", rtt=12.0, category="session"),
            cmd(1.0, "battery?", "gateway", rtt=10.0, category="query"),
            cmd(2.0, "battery?", "gateway", rtt=30.0, category="query"),
            cmd(50.0, "takeoff", "operator", rtt=5000.0),
            cmd(60.0, "rc 10 0 -20 0", "gateway:operator"),
            cmd(60.05, "rc 0 30 0 0", "gateway:operator"),
            cmd(60.10, "rc 0 0 0 0", "gateway:operator"),
            cmd(65.0, "rc 0 0 0 0", "gateway:deadman"),
            cmd(65.1, "rc 0 0 0 -25", "gateway:deadman"),
            cmd(None, "rc 0 0 0 0", "autonomy", ok=False, blocked=True),
            cmd(90.0, "land", "safety:low_battery", rtt=3000.0, category="stop"),
        ]
        events = [
            {"t": at(0.0), "type": "link_started", "detail": {}},
            {"t": at(0.5), "type": "gateway_mode_changed",
             "detail": {"from": "disconnected", "to": "grounded", "by": "gateway", "reason": "state_ok"}},
            {"t": at(40.0), "type": "takeoff_refused", "detail": {"failed": ["battery"], "flying": False, "busy": None}},
            {"t": at(89.9), "type": "auto_land", "detail": {"reason": "low_battery"}},
        ]
        run = write_run(self.tmp.name, sensor=one_flight_sensor(), commands=commands, events=events,
                        extra_lines=["{깨진 줄\n"])
        rep = fr.build_report(run)
        self.assertEqual([c["cmd"] for c in rep["commands"]], ["command", "takeoff", "land"])
        self.assertEqual((rep["takeoff_rtt_ms"], rep["land_rtt_ms"]), ([5000.0], [3000.0]))
        self.assertEqual(rep["queries"], [{"cmd": "battery?", "count": 2, "failed": 0,
                                           "rtt_mean_ms": 20.0, "rtt_max_ms": 30.0}])
        rc = rep["rc"]
        self.assertEqual((rc["count"], rc["blocked"]), (6, 1))
        self.assertEqual(rc["by_source"], {"gateway:operator": 3, "gateway:deadman": 2, "autonomy": 1})
        self.assertEqual(rc["axes"]["lr"], {"nonzero": 1, "max_abs": 10})
        self.assertEqual(rc["axes"]["ud"], {"nonzero": 1, "max_abs": 20})
        self.assertEqual(rc["axes"]["yaw"], {"nonzero": 1, "max_abs": 25})
        self.assertEqual(rc["groups"], 2)  # 5초 끊김은 다른 묶음
        self.assertAlmostEqual(rc["interval_mean_ms"], round((50 + 50 + 100) / 3, 1))
        self.assertAlmostEqual(rc["interval_max_ms"], 100.0)
        evs = rep["events"]
        self.assertEqual(evs["mode_changes"][0]["to"], "grounded")
        self.assertEqual(evs["auto_land"][0]["reason"], "low_battery")
        self.assertEqual(evs["safety_land_commands"][0]["source"], "safety:low_battery")
        self.assertEqual(evs["takeoff_refused"][0]["failed"], ["battery"])
        self.assertEqual(evs["other_counts"], {"link_started": 1})
        self.assertEqual(rep["run"]["bad_lines"]["events"], 1)
        self.assertAlmostEqual(rep["run"]["duration_s"], 119.9)
        self.assertFalse(rep["run"]["metrics_present"])
        text = fr.format_text(rep)
        self.assertIn("metrics.json 없음", text)
        self.assertIn("takeoff 응답: 5000.0ms", text)

    def test_missing_files_are_none(self):
        run = os.path.join(self.tmp.name, "empty_run")
        os.makedirs(run)
        rep = fr.build_report(run)
        self.assertIsNone(rep["run"]["duration_s"])
        self.assertIsNone(rep["state"]["battery_first_pct"])
        self.assertIsNone(rep["rc"]["interval_mean_ms"])
        self.assertEqual(rep["phases"], [])
        self.assertEqual(rep["run"]["files"], {"manifest": False, "sensor": False, "commands": False, "events": False})
        text = fr.format_text(rep)
        self.assertIn("기록 길이 없음", text)
        self.assertIn("lr 0 / 없음", text)

    def test_cli_json(self):
        run = write_run(self.tmp.name, sensor=one_flight_sensor())
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            self.assertEqual(fr.main([run, "--json"]), 0)
        self.assertEqual(json.loads(out.getvalue())["run"]["run_id"], "tello_test_run")
        with contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(fr.main([os.path.join(self.tmp.name, "없는_폴더")]), 2)


class FakeTelloRunTest(unittest.TestCase):
    """가짜 Tello로 이륙 → 잠깐 상승 → 착륙 회차를 끝까지 만들고 요약한다."""

    def test_takeoff_land_run(self):
        fake = FakeTello(cmd_port=58889, state_port=58890, video_port=51111, send_video=False)
        fake.motion_delay = 0.3
        fake.temp_override = 50
        fake.start()
        tmp = tempfile.TemporaryDirectory()
        try:
            link = TelloLink(host="127.0.0.1", cmd_port=58889, local_cmd_port=0, state_port=58890,
                             bind_ip="127.0.0.1", default_timeout=1.0, allow_flight=True,
                             log_root=tmp.name, run_id="report_test", source="fake").start()
            link.send("command")
            deadline = time.monotonic() + 3
            while link.latest_state() is None and time.monotonic() < deadline:
                time.sleep(0.05)
            ctrl = FlightController(link).start()
            time.sleep(0.5)
            ctrl.confirm_operator(True)
            self.assertTrue(ctrl.takeoff()["ok"])
            for _ in range(5):
                ctrl.set_input(0, 0, 0.2, 0, source="operator")
                time.sleep(0.1)
            time.sleep(0.5)
            self.assertTrue(ctrl.land()["ok"])
            time.sleep(0.5)
            ctrl._stop.set()
            ctrl._thread.join(timeout=20)
            link.close()

            rep = fr.build_report(os.path.join(tmp.name, "report_test"))
        finally:
            fake.close()
            tmp.cleanup()

        self.assertEqual(rep["run"]["source"], "fake")
        self.assertTrue(rep["run"]["operator_controlled"])
        self.assertTrue(rep["run"]["metrics_present"])
        flights = [p for p in rep["phases"] if p["kind"] == "flight"]
        self.assertEqual(len(flights), 1)
        self.assertGreater(flights[0]["tof_max_m"], 0.8)  # 가짜는 80cm까지 오르고 tof = h + 10cm
        self.assertEqual(len(rep["takeoff_rtt_ms"]), 1)
        self.assertEqual(len(rep["land_rtt_ms"]), 1)
        self.assertGreater(rep["takeoff_rtt_ms"][0], 250)  # motion_delay 0.3초 뒤 응답
        self.assertIn("operator", rep["rc"]["by_source"])
        self.assertGreater(rep["rc"]["axes"]["ud"]["nonzero"], 0)
        self.assertIsNotNone(rep["rc"]["interval_mean_ms"])


if __name__ == "__main__":
    unittest.main()
