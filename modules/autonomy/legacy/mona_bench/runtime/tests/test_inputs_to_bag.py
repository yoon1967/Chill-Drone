"""Saved live inputs -> replay bag (needs ROS: run in the container)."""
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from inputs_to_bag import RGB, TOF, airborne_frames, read_inputs, write_bag  # noqa: E402


class InputsToBagTests(unittest.TestCase):
    def make_run(self, root):
        import cv2
        import numpy as np
        frames = Path(root, "inputs", "frames")
        frames.mkdir(parents=True)
        image = np.zeros((72, 96, 3), np.uint8)
        image[:, :, 2] = 200                                   # red in BGR
        for stamp in (2_000_000_000, 1_000_000_000):          # written out of order on purpose
            frames.joinpath(f"{stamp}.jpg").write_bytes(cv2.imencode(".jpg", image)[1].tobytes())
        frames.joinpath("broken.jpg").write_bytes(b"x")        # not a stamp: ignored
        Path(root, "inputs", "tof.csv").write_text("1500000000,1.010\nbad line\n1000000000,1.000\n")

    def test_reads_sorted_and_skips_junk(self):
        with tempfile.TemporaryDirectory() as root:
            self.make_run(root)
            frames, tof = read_inputs(root)
            self.assertEqual([s for s, _ in frames], [1_000_000_000, 2_000_000_000])
            self.assertEqual(tof, [(1_000_000_000, 1.0), (1_500_000_000, 1.01)])

    def test_bag_has_replay_topics_with_original_stamps(self):
        import rosbag
        with tempfile.TemporaryDirectory() as root:
            self.make_run(root)
            out = Path(root, "input.bag")
            self.assertEqual(write_bag(root, out), (2, 2))
            with rosbag.Bag(str(out)) as bag:
                msgs = [(topic, msg) for topic, msg, _ in bag.read_messages()]
            images = [m for t, m in msgs if t == RGB]
            ranges = [m for t, m in msgs if t == TOF]
            self.assertEqual([m.header.stamp.to_nsec() for m in images], [1_000_000_000, 2_000_000_000])
            self.assertEqual((images[0].encoding, images[0].width, images[0].height), ("bgr8", 96, 72))
            self.assertAlmostEqual(ranges[1].range, 1.01)

    def test_frames_outside_the_tof_span_are_left_out(self):
        tof = [(10_000_000_000, 1.0), (20_000_000_000, 1.0)]
        frames = [(s, None) for s in (9_000_000_000, 9_600_000_000, 15_000_000_000, 20_400_000_000, 40_000_000_000)]
        self.assertEqual([s for s, _ in airborne_frames(frames, tof)], [9_600_000_000, 15_000_000_000, 20_400_000_000])
        self.assertEqual(airborne_frames(frames, []), [])

    def test_empty_run_is_an_error(self):
        with tempfile.TemporaryDirectory() as root:
            with self.assertRaises(SystemExit):
                write_bag(root, Path(root, "input.bag"))


if __name__ == "__main__":
    unittest.main()
