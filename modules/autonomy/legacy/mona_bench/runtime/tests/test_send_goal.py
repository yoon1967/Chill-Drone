import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from send_goal import forward_goal


class ForwardGoalTests(unittest.TestCase):
    def test_identity_pose_goes_along_map_x(self):
        goal, heading = forward_goal((0.5, -0.2, 0.3), (0, 0, 0, 1), 1.0)
        self.assertAlmostEqual(goal[0], 1.5)
        self.assertAlmostEqual(goal[1], -0.2)
        self.assertAlmostEqual(goal[2], 0.3)  # keeps the start height
        self.assertAlmostEqual(heading, 0.0)

    def test_yaw_rotates_goal(self):
        half = math.radians(90) / 2
        goal, heading = forward_goal((0, 0, 0), (0, 0, math.sin(half), math.cos(half)), 1.0)
        self.assertAlmostEqual(goal[0], 0.0, places=6)
        self.assertAlmostEqual(goal[1], 1.0, places=6)
        self.assertAlmostEqual(math.degrees(heading), 90.0, places=6)

    def test_pitch_keeps_goal_horizontal_distance(self):
        half = math.radians(20) / 2  # nose down 20 degrees about y
        goal, _ = forward_goal((0, 0, 0), (0, math.sin(half), 0, math.cos(half)), 1.0)
        self.assertAlmostEqual(math.hypot(goal[0], goal[1]), 1.0, places=6)

    def test_invalid_inputs_are_rejected(self):
        for pose, quat, dist in [((0, 0, 0), (0, 0, 0, 1), 0), ((0, 0, 0), (0, 0, 0, 1), 6),
                                 ((math.nan, 0, 0), (0, 0, 0, 1), 1), ((0, 0, 0), (0, 0, 0, 0), 1),
                                 ((0, 0, 0), (0, math.sin(math.pi/4), 0, math.cos(math.pi/4)), 1),
                                 ((0, 0, -0.2), (0, 0, 0, 1), 1)]:
            with self.assertRaises(ValueError):
                forward_goal(pose, quat, dist)


if __name__ == "__main__":
    unittest.main()
