"""프레임 번호·보관 시험 (표준 라이브러리만).

    python -B -m unittest discover -s modules/autonomy/tello_link/tests -p "test_frame_store.py" -v
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from frame_store import FrameStore  # noqa: E402


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


class FrameStoreTest(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.store = FrameStore("run1", max_frames=3, max_age_s=5.0, clock=self.clock)

    def add(self, stamp, jpeg=b"x"):
        return self.store.add(jpeg, stamp, 1_700_000_000 + stamp, (960, 720))

    def test_no_session_no_frame(self):
        self.assertIsNone(self.store.input_id)
        self.assertIsNone(self.add(100.0))

    def test_ids_and_same_frame(self):
        self.assertEqual(self.store.new_session(), "run1-video1")
        a = self.add(100.0)
        self.assertEqual((a["input_id"], a["frame_id"], a["width"], a["height"]), ("run1-video1", "frame-000001", 960, 720))
        self.assertIs(self.add(100.0), a)
        self.assertEqual(self.add(100.1)["frame_id"], "frame-000002")
        self.assertIs(self.store.get("run1-video1", "frame-000001"), a)
        self.assertIsNone(self.store.get("run1-video0", "frame-000001"))
        self.assertIsNone(self.store.get("run1-video1", "frame-000009"))

    def test_keeps_recent_only(self):
        self.store.new_session()
        for i in range(5):
            self.add(100.0 + i * 0.1)
        self.assertIsNone(self.store.get("run1-video1", "frame-000002"))   # 개수 초과
        self.assertIsNotNone(self.store.get("run1-video1", "frame-000003"))
        self.clock.t = 106.0
        self.assertIsNone(self.store.get("run1-video1", "frame-000005"))   # 오래됨

    def test_new_session_resets(self):
        self.store.new_session()
        self.add(100.0)
        self.assertEqual(self.store.new_session(), "run1-video2")
        self.assertIsNone(self.store.get("run1-video1", "frame-000001"))
        self.assertEqual(self.add(100.0)["frame_id"], "frame-000001")   # 새 세션은 같은 시각이어도 새 번호


if __name__ == "__main__":
    unittest.main()
