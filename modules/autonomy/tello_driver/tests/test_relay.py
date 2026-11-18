"""Windows 중계(tello_relay.py)를 사이에 두고 TelloLink가 가짜 Tello와 명령·상태를 주고받는지 (표준 라이브러리만)."""

import os
import sys
import time
import unittest

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "tello_link"))

from fake_tello import FakeTello  # noqa: E402
from tello_link import TelloLink  # noqa: E402
from tello_relay import TelloRelay  # noqa: E402


def wait_until(pred, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if pred():
            return True
        time.sleep(0.02)
    return False


class RelayTest(unittest.TestCase):
    def setUp(self):
        # 가짜 Tello(명령 47889) ← 중계(드라이버 쪽 46889, 상태 받기 47890) ← TelloLink(상태 46890)
        self.fake = FakeTello(cmd_port=47889, state_port=47890, video_port=47111, send_video=False)
        self.fake.temp_override = 50
        self.fake.start()
        self.relay = TelloRelay(tello="127.0.0.1", tello_cmd_port=47889, driver_port=46889, state_port=47890,
                                video_port=47111, bind="127.0.0.1", driver_state_port=46890, driver_video_port=46111,
                                tello_local_port=47888, log=lambda *_: None).start()
        self.link = TelloLink(host="127.0.0.1", cmd_port=46889, local_cmd_port=0, state_port=46890,
                              bind_ip="127.0.0.1", default_timeout=1.0, allow_flight=True).start()

    def tearDown(self):
        self.link.close()
        self.relay.close()
        self.fake.close()

    def test_commands_replies_and_state_pass_through(self):
        r = self.link.send("command")
        self.assertTrue(r.ok, r)
        self.assertEqual(self.link.send("battery?").response, "87")
        self.assertTrue(wait_until(lambda: self.link.latest_state() is not None))
        self.link.rc(0, 20, 0, 0)
        self.assertTrue(wait_until(lambda: "rc 0 20 0 0" in self.fake.received))
        self.assertGreater(self.relay.counts["state"], 0)


if __name__ == "__main__":
    unittest.main()
