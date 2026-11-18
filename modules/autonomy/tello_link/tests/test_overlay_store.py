"""AI 합성 영상 받기 시험 (표준 라이브러리만).

    python -B -m unittest discover -s modules/autonomy/tello_link/tests -p "test_overlay_store.py" -v
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import overlay_store  # noqa: E402
from overlay_store import ALIVE_S, OverlayStore  # noqa: E402

JPEG = b"\xff\xd8\xff\xe0" + b"0" * 100


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


class OverlayStoreTest(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.store = OverlayStore(clock=self.clock)

    def test_nothing_before_first_put(self):
        self.assertIsNone(self.store.latest())
        st = self.store.status()
        self.assertEqual((st["alive"], st["fps"], st["age_s"], st["frame_id"]), (False, None, None, None))

    def test_keeps_latest_and_frame_ids(self):
        self.assertEqual(self.store.put(JPEG, "in-1", "f-1")[0], 200)
        self.clock.t += 0.1
        second = JPEG + b"1"
        self.store.put(second, "in-1", "f-2")
        self.assertIs(self.store.latest(), second)
        st = self.store.status()
        self.assertTrue(st["alive"])
        self.assertEqual((st["frame_id"], st["input_id"], st["received"]), ("f-2", "in-1", 2))
        self.assertEqual(st["fps"], 1.0)   # 2초 안에 2장

    def test_goes_stale_when_sender_stops(self):
        self.store.put(JPEG, "in-1", "f-1")
        self.clock.t += ALIVE_S + 0.1
        self.assertIsNone(self.store.latest())   # 멈춘 합성 영상을 계속 보여 주지 않는다
        st = self.store.status()
        self.assertFalse(st["alive"])
        self.assertIsNone(st["fps"])
        self.assertEqual(st["frame_id"], "f-1")  # 마지막으로 받은 장은 상태에 남긴다

    def test_rejects_non_jpeg_and_too_large(self):
        self.assertEqual(self.store.put(b"not a jpeg", None, None), (400, {"ok": False, "reason": "not_jpeg"}))
        big = b"\xff\xd8" + b"0" * overlay_store.MAX_IMAGE_BYTES
        self.assertEqual(self.store.put(big, None, None)[0], 413)
        self.assertEqual((self.store.status()["rejected"], self.store.latest()), (2, None))


if __name__ == "__main__":
    unittest.main()
