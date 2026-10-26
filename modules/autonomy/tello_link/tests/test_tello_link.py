"""가짜 Tello로 tello_link를 시험한다 (표준 라이브러리만, 영상 제외).

    python -B -m unittest discover -s modules/autonomy/tello_link/tests -v
"""

import json
import math
import os
import sys
import tempfile
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from fake_tello import FakeTello  # noqa: E402
from tello_link import TelloLink, classify, parse_state  # noqa: E402

PACKET = ("mid:-1;x:-100;y:-100;z:-100;mpry:0,0,0;pitch:2;roll:-3;yaw:90;vgx:0;vgy:0;vgz:0;"
          "templ:60;temph:63;tof:120;h:35;bat:87;baro:12.34;time:5;agx:-2.00;agy:1.00;agz:-998.00;\r\n")


class ParseStateTest(unittest.TestCase):
    def test_raw_and_si(self):
        s = parse_state(PACKET)
        self.assertTrue(s.valid)
        self.assertEqual(s.raw["bat"], 87)
        self.assertEqual(s.raw["mpry"], [0, 0, 0])
        self.assertAlmostEqual(s.si["yaw_rad"], math.pi / 2)
        self.assertAlmostEqual(s.si["height_m"], 0.35)
        self.assertAlmostEqual(s.si["tof_m"], 1.2)
        self.assertEqual(s.time_source, "server_receive")
        self.assertIn("vgx", s.si["unverified_unit_fields"])

    def test_tof_out_of_range_is_invalid_not_zero(self):
        ground = parse_state(PACKET.replace("tof:120", "tof:10"))
        self.assertIsNone(ground.si["tof_m"])
        self.assertFalse(ground.si["tof_valid"])
        self.assertEqual(ground.si["tof_reason"], "below_min_or_on_ground")
        no_return = parse_state(PACKET.replace("tof:120", "tof:6553"))
        self.assertEqual(no_return.si["tof_reason"], "no_return")

    def test_broken_packet(self):
        s = parse_state("pitch:0;roll:x;")
        self.assertFalse(s.valid)
        self.assertTrue(s.reasons[0].startswith("missing_or_non_numeric"))


class ClassifyTest(unittest.TestCase):
    def test_categories(self):
        self.assertEqual(classify("battery?"), "query")
        self.assertEqual(classify("command"), "session")
        self.assertEqual(classify("emergency"), "stop")
        self.assertEqual(classify("takeoff"), "flight")
        self.assertEqual(classify("rc 0 0 0 0"), "flight")
        self.assertEqual(classify("forward 50"), "flight")
        self.assertEqual(classify("ap myssid mypass"), "config")
        self.assertEqual(classify("speed 50"), "config")
        self.assertEqual(classify("speed?"), "query")


class LinkWithFakeTest(unittest.TestCase):
    def setUp(self):
        self.fake = FakeTello(cmd_port=28889, state_port=28890, video_port=21111, send_video=False).start()
        self.tmp = tempfile.TemporaryDirectory()
        self.link = TelloLink(host="127.0.0.1", cmd_port=28889, local_cmd_port=0, state_port=28890,
                              bind_ip="127.0.0.1", default_timeout=1.0, log_root=self.tmp.name,
                              run_id="test_run", source="fake").start()

    def tearDown(self):
        self.link.close()
        self.fake.close()
        self.tmp.cleanup()

    def test_sdk_query_and_state(self):
        r = self.link.send("command")
        self.assertTrue(r.ok)
        self.assertTrue(self.link.sdk_mode)
        self.assertEqual(self.link.send("battery?").response, "87")
        deadline = time.monotonic() + 3
        while self.link.state_count < 5 and time.monotonic() < deadline:
            time.sleep(0.05)
        self.assertGreaterEqual(self.link.state_count, 5)
        state = self.link.latest_state()
        self.assertEqual(state["raw"]["bat"], 87)
        self.assertLess(state["age_s"], 1.0)

    def test_flight_blocked_and_not_sent(self):
        self.link.send("command")
        for cmd in ("takeoff", "forward 50", "ap ssid pw"):
            r = self.link.send(cmd)
            self.assertTrue(r.blocked)
            self.assertFalse(r.ok)
        self.assertTrue(self.link.rc(0, 0, 0, 0).blocked)
        time.sleep(0.2)
        self.assertEqual(self.fake.received, ["command"])

    def test_timeout_is_invalid(self):
        self.fake.close()
        r = self.link.send("battery?", timeout=0.3)
        self.assertFalse(r.valid)
        self.assertIn(r.reason, ("timeout", "connection_reset"))

    def test_run_folder(self):
        self.link.send("command")
        self.link.send("takeoff")
        time.sleep(0.4)
        self.link.close()
        run_dir = os.path.join(self.tmp.name, "test_run")
        for name in ("manifest.json", "commands.jsonl", "sensor.jsonl", "events.jsonl", "metrics.json"):
            self.assertTrue(os.path.exists(os.path.join(run_dir, name)), name)
        with open(os.path.join(run_dir, "metrics.json"), encoding="utf-8") as f:
            metrics = json.load(f)
        self.assertEqual(metrics["commands_blocked"], 1)
        self.link.close = lambda: None  # tearDown 중복 종료 방지


if __name__ == "__main__":
    unittest.main()
