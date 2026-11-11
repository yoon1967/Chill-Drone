from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from floor_filter import HoldBuffer, remove_floor

FX = FY = 630.0
CX, CY = 320.0, 240.0


def scene(camera_height=1.2, pitch_deg=12.0, box=None):
    """Depth of a floor (and optionally a box face) seen by a camera pitched down by pitch_deg."""
    rows, cols = np.mgrid[0:480, 0:640].astype(float)
    rays = np.stack([(cols - CX) / FX, (rows - CY) / FY, np.ones_like(rows)], -1)
    p = np.radians(pitch_deg)
    # optical -> level frame (x right, y down, z forward), camera pitched down about x
    rot = np.array([[1, 0, 0], [0, np.cos(p), np.sin(p)], [0, -np.sin(p), np.cos(p)]])
    level = rays @ rot.T
    depth = np.full(rows.shape, np.inf)
    down = level[..., 1] > 1e-6
    depth[down] = camera_height / level[..., 1][down]  # ray scale where it meets the floor
    if box is not None:  # vertical face at forward distance box[0], |x| < box[1], above the floor
        distance, half_width = box
        t = distance / np.maximum(level[..., 2], 1e-6)
        x, y = level[..., 0] * t, level[..., 1] * t
        hit = (level[..., 2] > 0) & (np.abs(x) < half_width) & (y < camera_height) & (t < depth)
        depth[hit] = t[hit]
    depth[~np.isfinite(depth) | (depth > 20)] = 0
    return depth.astype(np.float32)


class FloorFilterTests(unittest.TestCase):
    def test_pitched_floor_is_removed_and_reported(self):
        out, stats = remove_floor(scene(), FX, FY, CX, CY, np.random.default_rng(1))
        self.assertTrue(stats["floor"])
        self.assertAlmostEqual(stats["height_m"], 1.2, delta=0.03)
        self.assertAlmostEqual(stats["tilt_deg"], 12.0, delta=1.0)
        self.assertEqual(int((out > 0).sum()), 0)

    def test_obstacle_in_front_survives(self):
        depth = scene(box=(2.0, 0.4))
        out, stats = remove_floor(depth, FX, FY, CX, CY, np.random.default_rng(2))
        self.assertTrue(stats["floor"])
        kept = out > 0
        self.assertGreater(int(kept.sum()), 5000)
        np.testing.assert_allclose(out[kept], depth[kept])
        # only the bottom 15 cm of the box face may go with the floor
        rows = np.nonzero(kept.any(1))[0]
        self.assertLess(rows.min(), 200)

    def test_no_floor_passes_depth_through(self):
        wall = np.full((480, 640), 2.0, np.float32)
        out, stats = remove_floor(wall, FX, FY, CX, CY, np.random.default_rng(3))
        self.assertFalse(stats["floor"])
        np.testing.assert_array_equal(out, wall)

    def test_invalid_pixels_become_zero(self):
        depth = scene(box=(2.0, 0.4))
        depth[0, 0] = np.nan
        out, _ = remove_floor(depth, FX, FY, CX, CY, np.random.default_rng(4))
        self.assertEqual(out[0, 0], 0)
        self.assertTrue(np.all(np.isfinite(out)))


class HoldBufferTests(unittest.TestCase):
    def test_frames_go_out_after_the_delay(self):
        buf = HoldBuffer(0.3)
        buf.push(0.0, 100.0, "a")
        self.assertEqual(buf.ready(0.2), [])
        self.assertEqual(buf.ready(0.31), ["a"])

    def test_jump_drops_buffered_and_new_frames_from_its_stamp(self):
        buf = HoldBuffer(0.3)
        for i in range(4):                         # stamps 100.0 .. 100.3, received 0.0 .. 0.3
            buf.push(0.1 * i, 100.0 + 0.1 * i, i)
        buf.set_hold(100.2)                         # bridge saw the jump on the frame stamped 100.2
        buf.push(0.4, 100.4, 4)
        self.assertEqual(buf.ready(1.0), [0, 1])
        self.assertEqual(buf.dropped, 3)
        buf.set_hold(0.0)                           # hold over: new frames go to the map again
        buf.push(2.0, 102.0, 5)
        self.assertEqual(buf.ready(2.4), [5])


if __name__ == "__main__":
    unittest.main()
