"""비행 중 장애물 생성기(tools/spawn_obstacles.py)의 도우미 함수 검사. 실행: python -m unittest discover -s tests"""
import math
import random
import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

from spawn_obstacles import (SIZES, along_path, box_stack_sdf, choose_spot, collision_sdf, footprint,  # noqa: E402
                             gap2d, person_sdf)


class AlongPathTest(unittest.TestCase):
    def setUp(self):
        # ㄱ자 경로: (0,0) → (4,0) → (4,3)
        self.path = [np.array([0.0, 0.0, 1.0]), np.array([4.0, 0.0, 1.0]), np.array([4.0, 3.0, 1.0])]

    def test_points_ahead_of_drone(self):
        pts = along_path(self.path, (1.0, 0.3), [1.0, 2.5, 4.0])
        self.assertAlmostEqual(pts[0][0], 2.0)
        self.assertAlmostEqual(pts[0][1], 0.0)
        self.assertAlmostEqual(pts[0][2], 0.0)                     # 진행 방향 동쪽
        self.assertAlmostEqual(pts[1][0], 3.5)
        self.assertAlmostEqual(pts[2][0], 4.0)                     # 모퉁이를 돌아
        self.assertAlmostEqual(pts[2][1], 1.0)
        self.assertAlmostEqual(pts[2][2], math.pi / 2)

    def test_stops_at_path_end(self):
        self.assertEqual(len(along_path(self.path, (3.9, 2.5), [0.3, 2.0])), 1)


class ChooseSpotTest(unittest.TestCase):
    """8×4 m 방, 왼쪽에서 오른쪽으로 가는 경로."""

    def setUp(self):
        t, h = 0.1, 2.5
        self.walls = [footprint(4, t / 2, (8, t, h)), footprint(4, 4 - t / 2, (8, t, h)),
                      footprint(t / 2, 2, (t, 4, h)), footprint(8 - t / 2, 2, (t, 4, h))]
        self.bounds = {"xmin": 0, "ymin": 0, "xmax": 8, "ymax": 4, "zmax": h}
        self.path = [np.array([0.8, 2.0, 1.0]), np.array([7.2, 2.0, 1.0])]

    def test_spot_on_path_ahead(self):
        spot, _ = choose_spot(self.path, (1.0, 2.0, 1.0), SIZES["box_stack"], self.walls, [], self.bounds,
                              random.Random(0))
        x, y, heading, ahead = spot
        self.assertAlmostEqual(y, 2.0)
        self.assertTrue(2.7 <= x <= 4.1)                 # 1.8–3.0 m 앞
        self.assertAlmostEqual(heading, 0.0)

    def test_rejects_door_goal_and_blocked(self):
        spot, why = choose_spot(self.path, (1.0, 2.0, 1.0), SIZES["box_stack"], self.walls, [(3.4, 2.0)],
                                self.bounds, random.Random(0))
        self.assertIsNone(spot)                          # 앞 구간 전체가 문 0.9 m 안
        self.assertIn("문 앞", why)
        spot, why = choose_spot(self.path, (4.6, 2.0, 1.0), SIZES["box_stack"], self.walls, [], self.bounds,
                                random.Random(0))
        self.assertIsNone(spot)
        self.assertIn("목적지 앞", why)
        narrow = self.walls + [footprint(3.5, 3.1, (0.4, 1.8, 2.5)), footprint(3.5, 0.9, (0.4, 1.8, 2.5))]
        spot, why = choose_spot(self.path, (1.0, 2.0, 1.0), SIZES["box_stack"], narrow, [], self.bounds,
                                random.Random(0), ahead=(2.4, 2.6))
        self.assertIsNone(spot)                          # 0.4 m 틈에 놓으면 길이 막히거나 겹친다
        self.assertTrue({"길이 막힘", "벽·가구와 겹침"} & set(why))

    def test_estimate_slack_keeps_roomy_route(self):
        # 폭 2 m 통로 가운데에 0.6 m 상자: 실제 크기로는 양옆 0.7 m로 지나가지만,
        # 드론이 크게 어림(각 변 +0.35 m)하면 0.35 m만 남아 막히므로 고르지 않는다
        corridor = self.walls + [footprint(4.0, 3.5, (8.0, 1.0, 2.5)), footprint(4.0, 0.5, (8.0, 1.0, 2.5))]
        args = (self.path, (1.0, 2.0, 1.0), SIZES["box_stack"], corridor, [], self.bounds)
        spot, why = choose_spot(*args, random.Random(0))
        self.assertIsNone(spot)
        self.assertIn("길이 막힘", why)
        import spawn_obstacles
        saved, spawn_obstacles.ESTIMATE_SLACK = spawn_obstacles.ESTIMATE_SLACK, 0.0
        try:
            spot, _ = choose_spot(*args, random.Random(0))
        finally:
            spawn_obstacles.ESTIMATE_SLACK = saved
        self.assertIsNotNone(spot)


class ShapesTest(unittest.TestCase):
    def test_box_stack_is_one_static_model(self):
        root = ET.fromstring(box_stack_sdf("b", 2.0, 3.0, 0.5, random.Random(1)))
        model = root.find("model")
        self.assertEqual(model.findtext("static"), "true")
        self.assertEqual(len(model.findall("link/collision")), 1)
        self.assertGreaterEqual(len(model.findall("link/visual")), 2)
        size = [float(v) for v in model.find("link/collision/geometry/box/size").text.split()]
        self.assertEqual(size, list(SIZES["box_stack"]))

    def test_person_has_no_actor_pose(self):
        root = ET.fromstring(person_sdf("p", 2.0, 3.0, 1.0))
        actor = root.find("actor")
        self.assertIsNone(actor.find("pose"), "actor에 pose를 주면 좌표가 두 번 더해진다")
        self.assertTrue(actor.find("script/trajectory/waypoint/pose").text.startswith("2.000 3.000"))
        col = ET.fromstring(collision_sdf("p_c", 2.0, 3.0, SIZES["person"])).find("model")
        self.assertIsNone(col.find("link/visual"))
        self.assertIsNotNone(col.find("link/collision"))

    def test_gap(self):
        a = footprint(0.0, 0.0, (0.6, 0.6, 1.0))
        self.assertEqual(gap2d(a, footprint(0.5, 0.0, (0.6, 0.6, 1.0))), 0.0)
        self.assertAlmostEqual(gap2d(a, footprint(1.6, 0.0, (0.6, 0.6, 1.0))), 1.0)


if __name__ == "__main__":
    unittest.main()
