"""사람 동선(route_plan.py): 자율 비행 지도 위에서 대원이 걸을 길.

    python -B -m unittest discover -s modules/autonomy/tello_link/tests -p "test_route_plan.py" -v
"""

import base64
import math
import os
import random
import sys
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import route_plan  # noqa: E402
from route_plan import BLOCKED, SEEN, SUSPECT, UNKNOWN, CostMap, plan  # noqa: E402

RES = 0.1


class Room:
    """w×h칸(0.1 m) 지도. 칸 (i, j)는 x = i·0.1, y = j·0.1 m 둘레. 처음은 모두 본 곳이고 둘레는 벽이다."""

    def __init__(self, w, h, fill=SEEN, walls=True):
        self.w, self.h = w, h
        self.codes = bytearray([fill] * (w * h))
        if walls:
            self.box(0, 0, w - 1, 0, BLOCKED)
            self.box(0, h - 1, w - 1, h - 1, BLOCKED)
            self.box(0, 0, 0, h - 1, BLOCKED)
            self.box(w - 1, 0, w - 1, h - 1, BLOCKED)

    def box(self, i0, j0, i1, j1, code):
        for j in range(j0, j1 + 1):
            for i in range(i0, i1 + 1):
                self.codes[j * self.w + i] = code
        return self

    def grid(self, ver=1):
        return {"ver": ver, "res": RES, "w": self.w, "h": self.h, "x0": 0.0, "y0": 0.0,
                "cells": base64.b64encode(bytes(self.codes)).decode()}

    def code_at(self, x, y):
        return self.codes[math.floor(y / RES) * self.w + math.floor(x / RES)]


def two_rooms(door_cells):
    """6 m × 4 m 방 둘(가운데 x = 3 m 벽)과 벽 가운데 문 하나(폭 door_cells칸)."""
    room = Room(60, 40).box(30, 0, 30, 39, BLOCKED)
    d0 = (40 - door_cells) // 2
    room.box(30, d0, 30, d0 + door_cells - 1, SEEN)
    return room


def crossed_codes(room, pts, step=0.02):
    """동선 선분을 촘촘히 따라가며 지난 칸 코드들."""
    out = set()
    for (ax, ay), (bx, by) in zip(pts, pts[1:]):
        n = max(1, int(math.dist((ax, ay), (bx, by)) / step))
        for s in range(n + 1):
            out.add(room.code_at(ax + (bx - ax) * s / n, ay + (by - ay) * s / n))
    return out


class DoorTest(unittest.TestCase):
    def test_body_width_decides_which_doors_pass(self):   # 몸 폭 0.6 m: 0.6 m 문은 막히고 0.7 m 문부터 지나간다
        narrow = plan(CostMap(two_rooms(6).grid()), (1.0, 2.0), (5.0, 2.0))
        self.assertEqual((narrow["state"], narrow["reason"]), ("partial", "goal_unreachable"))
        self.assertLess(narrow["pts"][-1][0], 3.0)                 # 문 앞까지만
        wide = plan(CostMap(two_rooms(7).grid()), (1.0, 2.0), (5.0, 2.0))
        self.assertEqual(wide["state"], "ok")
        self.assertEqual(crossed_codes(two_rooms(7), wide["pts"]), {SEEN})
        self.assertAlmostEqual(wide["length_m"], 4.0, delta=0.15)
        thin = plan(CostMap(two_rooms(5).grid(), width_m=0.5), (1.0, 2.0), (5.0, 2.0))   # 몸 폭 0.5 m면 0.5 m 문도
        self.assertEqual(thin["state"], "ok")

    def test_open_room_is_one_straight_line(self):
        r = plan(CostMap(Room(60, 40).grid()), (1.0, 2.0), (5.0, 2.0))
        self.assertEqual((r["state"], len(r["pts"])), ("ok", 2))
        self.assertAlmostEqual(r["length_m"], 4.0, delta=0.15)
        self.assertEqual(r["walk_s"], round(r["length_m"] / route_plan.WALK_MPS, 1))   # 조심 걸음 0.5 m/s
        self.assertIsNone(r["gap_m"])


class MapRulesTest(unittest.TestCase):
    def test_unknown_cells_are_not_crossed(self):   # 못 본 곳 너머의 사람: 본 곳 끝까지만 가고 남은 거리를 알린다
        room = Room(60, 40).box(25, 1, 34, 38, UNKNOWN)
        r = plan(CostMap(room.grid()), (1.0, 2.0), (5.0, 2.0))
        self.assertEqual(r["state"], "partial")
        self.assertNotIn(UNKNOWN, crossed_codes(room, r["pts"]))
        self.assertGreater(r["gap_m"], 2.0)

    def test_standing_person_is_reached_beside(self):   # 서 있는 사람은 지도에서 막힘: 1 m 안 가장 가까운 칸으로
        room = Room(60, 40).box(48, 18, 51, 21, BLOCKED)        # 사람 몸(0.4 m)
        r = plan(CostMap(room.grid()), (1.0, 2.0), (4.95, 1.95))
        self.assertEqual(r["state"], "ok")
        self.assertGreater(r["snap_goal_m"], 0.3)
        self.assertLessEqual(r["snap_goal_m"], route_plan.SNAP_M)

    def test_start_deep_in_obstacles_or_off_the_map(self):
        room = Room(60, 40).box(1, 1, 30, 38, BLOCKED)
        self.assertEqual(plan(CostMap(room.grid()), (1.0, 2.0), (5.0, 2.0))["reason"], "start_blocked")
        self.assertEqual(plan(CostMap(Room(60, 40).grid()), (-5.0, 2.0), (5.0, 2.0))["reason"], "start_off_map")

    def test_found_obstacle_box_closes_a_corridor(self):   # 비행 노드가 찾은 장애물(지도에 없던 것)도 막힘
        hall = Room(60, 12)                                       # 폭 1.0 m 복도(벽 안쪽)
        self.assertEqual(plan(CostMap(hall.grid()), (0.5, 0.6), (5.5, 0.6))["state"], "ok")
        box = {"obstacles": [{"x": 3.0, "y": 0.6, "sx": 0.4, "sy": 0.4}]}
        self.assertEqual(plan(CostMap(hall.grid(), box), (0.5, 0.6), (5.5, 0.6))["state"], "partial")

    def test_suspect_cells_are_walked_around(self):   # 의심 칸(비행 노드도 피함): 줄 끝 틈으로 돈다
        room = Room(60, 40).box(30, 1, 30, 28, SUSPECT)
        r = plan(CostMap(room.grid()), (1.0, 1.0), (5.0, 1.0))
        self.assertEqual(r["state"], "ok")
        self.assertNotIn(SUSPECT, crossed_codes(room, r["pts"]))
        self.assertGreater(r["length_m"], 5.0)

    def test_boxed_in_start_has_no_route(self):
        room = Room(60, 40).box(5, 5, 20, 20, BLOCKED).box(8, 8, 14, 14, SEEN)   # 막힘에 둘러싸인 0.7 m 빈 곳: 설 자리 한 칸뿐
        r = plan(CostMap(room.grid()), (1.15, 1.15), (5.0, 2.0))
        self.assertEqual((r["state"], r["reason"]), ("no_path", "boxed_in"))
        room.box(8, 8, 17, 17, SEEN)                                               # 1 m면 조금은 움직인다
        self.assertEqual(plan(CostMap(room.grid()), (1.25, 1.25), (5.0, 2.0))["state"], "partial")


class GridTest(unittest.TestCase):
    def test_bad_grids_are_refused(self):
        good = Room(10, 10).grid()
        for bad in ({**good, "cells": "!!"}, {**good, "w": 11}, {**good, "res": 0}, {k: v for k, v in good.items() if k != "x0"}):
            with self.assertRaises(ValueError):
                CostMap(bad)
        big = {**good, "w": 600, "h": 600, "cells": base64.b64encode(bytes(360000)).decode()}
        with self.assertRaisesRegex(ValueError, "grid_too_big"):
            CostMap(big)

    def test_flight_size_map_is_quick(self):   # 비행 노드 지도 크기(200×200칸, 20 m)에 가구가 흩어진 방
        room = Room(200, 200)
        rnd = random.Random(7)
        for _ in range(120):
            i, j = rnd.randrange(5, 190), rnd.randrange(5, 190)
            room.box(i, j, i + rnd.randrange(1, 8), j + rnd.randrange(1, 8), BLOCKED)
        t0 = time.perf_counter()
        cmap = CostMap(room.grid())
        r = plan(cmap, (1.0, 1.0), (18.5, 18.5))
        self.assertIn(r["state"], ("ok", "partial"))
        self.assertLess(time.perf_counter() - t0, 2.0)   # 이 PC 약 0.1초. CI는 느릴 수 있어 넉넉히


if __name__ == "__main__":
    unittest.main()
