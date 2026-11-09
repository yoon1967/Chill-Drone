import math
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from sim_world import (CX, CY, FY, Drone, camera_rotation, clearance, cross_track, render_depth,  # noqa: E402
                       rotation_to_quaternion)


class GeometryTests(unittest.TestCase):
    def test_camera_looks_forward_and_down(self):
        r = camera_rotation(0.0, math.radians(10))
        forward, down, right = r[:, 2], r[:, 1], r[:, 0]
        self.assertAlmostEqual(forward[0], math.cos(math.radians(10)))
        self.assertAlmostEqual(forward[2], -math.sin(math.radians(10)))
        self.assertAlmostEqual(right[1], -1.0)                  # optical x = body right = map -y
        self.assertLess(down[2], 0)
        self.assertAlmostEqual(np.linalg.det(r), 1.0)
        q = rotation_to_quaternion(r)
        self.assertAlmostEqual(sum(v * v for v in q), 1.0)

    def test_floor_depth_matches_geometry(self):
        r = camera_rotation(0.0, 0.0)                           # level camera, floor 1 m below
        depth = render_depth((0.0, 0.0, 0.0), r, [], floor_z=-1.0)
        v = int(round(CY + FY * 0.3))                           # ray ~0.3 down per 1 forward -> floor ~3.33 m
        slope = (v - CY) / FY
        self.assertAlmostEqual(float(depth[v, int(CX)]), 1.0 / slope, places=3)
        self.assertEqual(float(depth[int(CY) - 50, int(CX)]), 0.0)   # above the horizon: nothing in range

    def test_box_in_front_is_seen_at_its_face(self):
        r = camera_rotation(0.0, 0.0)
        depth = render_depth((0.0, 0.0, 0.0), r, [[2.0, 2.4, -0.25, 0.25, -1.0, 0.5]], floor_z=-1.0)
        self.assertAlmostEqual(float(depth[int(CY), int(CX)]), 2.0, places=3)

    def test_clearance_only_counts_boxes_at_flight_height(self):
        boxes = [[2.0, 2.4, -0.25, 0.25, -1.0, 0.5], [1.0, 1.2, -0.1, 0.1, -1.0, -0.6]]
        self.assertAlmostEqual(clearance((2.2, 0.75), boxes, 0.0), 0.5)
        self.assertAlmostEqual(clearance((1.1, 0.0), boxes, 0.0), math.hypot(0.9, 0.0) - 0.0, places=6)

    def test_cross_track_distance_to_plan(self):
        path = [[0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [2.0, 2.0, 0.0]]
        self.assertAlmostEqual(cross_track((1.0, 0.3), path), 0.3)
        self.assertAlmostEqual(cross_track((2.5, 1.0), path), 0.5)
        self.assertAlmostEqual(cross_track((-1.0, 0.0), path), 1.0)

    def test_drone_lags_and_clamps(self):
        d = Drone(tau=0.3, limit=0.2)
        for _ in range(100):
            d.step((1.0, 0.0), 0.05)
        self.assertAlmostEqual(d.v[0], 0.2, places=3)
        self.assertGreater(d.p[0], 0.8)


if __name__ == "__main__":
    unittest.main()
