"""경로 위의 문(좁은 틈) 찾기 (autonomy/doorway.py, 11/05 10:51 실기체: 열린 문으로 가는 길이 좁아 포기했다)."""
import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from autonomy.doorway import Doorway, find_doorway, is_corridor, measure  # noqa: E402
from autonomy.explore import ViewpointBudget  # noqa: E402

RES = 0.1


def room(door=(2.6, 3.4), wall_x=3.0, thick=0.2, size=60):
    """x = wall_x 벽(두께 thick), y가 door 사이(칸 경계)인 문 — 기본은 폭 0.8 m, 가운데 y 3.0. 6×6 m, 칸 0.1 m."""
    b = np.zeros((size, size), dtype=bool)
    xs = (np.arange(size) + 0.5) * RES
    wall = (xs > wall_x - thick / 2) & (xs < wall_x + thick / 2)
    opening = (xs > door[0]) & (xs < door[1])
    b[np.ix_(wall, ~opening)] = True
    return b


def box(b, x0, x1, y0, y1):
    xs = (np.arange(b.shape[0]) + 0.5) * RES
    b[np.ix_((xs > x0) & (xs < x1), (xs > y0) & (xs < y1))] = True
    return b


class FindDoorwayTest(unittest.TestCase):
    def test_straight_through(self):
        d = find_doorway(room(), (0, 0), RES, [(1.0, 3.0), (5.0, 3.0)], narrow=0.4)
        self.assertIsNotNone(d)
        self.assertLess(np.linalg.norm(d.center - [3.0, 3.0]), 0.08)   # 칸 0.1 m
        self.assertAlmostEqual(d.width, 0.8, delta=0.1)
        self.assertGreater(d.normal @ [1, 0], 0.99)
        self.assertLess(d.along(2.2, 3.0), -0.7)
        self.assertAlmostEqual(d.across(3.0, 3.3), 0.3, delta=0.06)

    def test_goal_inside_the_gap_counts_as_reached(self):
        # 11/11 Gazebo ㄱ자 복도: 틈 가운데 (1.4, 5.7)을 81°로 지나 (1.60, 6.25)에 섰다. 볼 자리 (1.8, 5.7)은 틈 가로선 위(옆 0.4 m)라
        # 드론에서 0.59 m — 다시 계획하면 같은 틈을 되돌아 지났다. 방금 지나온 구간 옆에 있으면 닿은 것으로 본다
        from autonomy.doorway import Doorway
        a = math.radians(81)
        d = Doorway(np.array([1.4, 5.7]), np.array([math.cos(a), math.sin(a)]), 1.25)
        here = (1.60, 6.25)
        self.assertTrue(d.covers(here, (1.8, 5.7), pre=0.8, side=0.6))
        self.assertTrue(d.covers(here, (1.7, 6.6), pre=0.8, side=0.6), "바로 너머 0.5 m 안")
        self.assertFalse(d.covers(here, (2.9, 6.0), pre=0.8, side=0.6), "틈 옆으로 1.5 m: 지나온 구간이 아니다")
        self.assertFalse(d.covers(here, (1.8, 7.6), pre=0.8, side=0.6), "한참 너머는 계속 간다")
        self.assertFalse(d.covers(here, (1.2, 4.4), pre=0.8, side=0.6), "문 앞 자리보다 더 뒤")

    def test_oblique_path_gives_wall_normal(self):
        # 10:51처럼 비스듬히 다가가는 경로: 문의 가로선은 경로의 수직이 아니라 벽을 따라가야 폭이 맞다
        path = [(1.0, 1.2), (3.0, 3.0), (4.5, 4.4)]
        d = find_doorway(room(), (0, 0), RES, path, narrow=0.4)
        self.assertIsNotNone(d)
        self.assertLess(abs(math.degrees(math.atan2(d.normal[1], d.normal[0]))), 12)
        self.assertAlmostEqual(d.width, 0.8, delta=0.12)
        self.assertLess(abs(d.center[0] - 3.0), 0.15)

    def test_long_narrow_corridor_is_not_a_door(self):
        # 폭 0.6 m 복도 4 m: 좁은 구간이 2.5 m를 넘는다
        b = box(box(np.zeros((60, 60), dtype=bool), 1.0, 5.0, 2.5, 2.7), 1.0, 5.0, 3.3, 3.5)
        self.assertIsNone(find_doorway(b, (0, 0), RES, [(0.2, 3.0), (5.8, 3.0)], narrow=0.4))
        # 좁은 복도를 지나 그 너머 문이 있으면 그 문 (복도는 보통 경로로 간다)
        b = box(box(np.zeros((90, 60), dtype=bool), 0.8, 4.5, 2.5, 2.7), 0.8, 4.5, 3.3, 3.5)
        wall = np.zeros_like(b)
        xs = (np.arange(90) + 0.5) * RES
        wall[(xs > 6.4) & (xs < 6.6), :] = True
        wall[:, 26:34] = False
        d = find_doorway(b | wall, (0, 0), RES, [(0.3, 3.0), (8.5, 3.0)], narrow=0.4)
        self.assertIsNotNone(d)
        self.assertLess(np.linalg.norm(d.center - [6.5, 3.0]), 0.1)

    def test_passing_one_obstacle_is_not_a_door(self):
        b = box(np.zeros((60, 60), dtype=bool), 2.8, 3.2, 3.3, 3.7)   # 한쪽만 막힘: 옆으로 비킬 수 있다
        self.assertIsNone(find_doorway(b, (0, 0), RES, [(1.0, 3.0), (5.0, 3.0)], narrow=0.4))

    def test_too_narrow(self):
        self.assertIsNone(find_doorway(room(door=(2.8, 3.1)), (0, 0), RES, [(1.0, 3.0), (5.0, 3.0)], narrow=0.4))

    def test_wide_opening_has_no_narrow_part(self):
        self.assertIsNone(find_doorway(room(door=(1.5, 4.5)), (0, 0), RES, [(1.0, 3.0), (5.0, 3.0)], narrow=0.4))

    def test_skip_the_spot_the_drone_is_in(self):
        # 드론이 이미 문 안에 있으면 그 문은 아니다 (10:51 bag: 출발 자리가 늘 첫 좁은 구간이었다)
        self.assertIsNone(find_doorway(room(), (0, 0), RES, [(3.0, 3.0), (5.0, 3.0)], narrow=0.4))
        # 그 뒤에 다른 문이 있으면 그 문
        b = room()
        b = b | room(wall_x=4.5)
        d = find_doorway(b, (0, 0), RES, [(3.0, 3.0), (5.5, 3.0)], narrow=0.4)
        self.assertIsNotNone(d)
        self.assertLess(np.linalg.norm(d.center - [4.5, 3.0]), 0.1)

    def test_door_just_ahead_is_still_a_door(self):
        # 11/11 Gazebo 방 두 개: 문 0.5 m 앞에서 둘러본 뒤 다시 계획하자, 평소 여유(0.6 m)로는 그 자리부터 좁은 구간이라 문이 통째로
        # 건너뛰어져 보통 경로로 비스듬히 지났다(문틀에 −0.00 m). 가장 좁은 곳이 0.4 m 넘게 앞이면 문이다
        d = find_doorway(room(), (0, 0), RES, [(2.4, 3.0), (5.0, 3.0)], narrow=0.6)
        self.assertIsNotNone(d)
        self.assertLess(np.linalg.norm(d.center - [3.0, 3.0]), 0.1)
        self.assertLess(d.along(2.4, 3.0), -0.4)
        # 문틀 사이에 이미 들어섰으면(가운데까지 0.4 m 안) 전처럼 문이 아니다
        self.assertIsNone(find_doorway(room(), (0, 0), RES, [(2.8, 3.0), (5.0, 3.0)], narrow=0.6))

    def test_lab_door_between_tower_and_cabinet(self):
        # 10:51 실습실 모양: 문(0.9 m) 오른쪽 앞에 문 열린 사물함(깊이 0.5 m), 왼쪽 앞에 공기청정기 기둥
        b = room()
        box(b, 2.4, 2.9, 1.3, 2.45)       # 사물함 (벽 앞)
        box(b, 2.45, 2.8, 3.6, 3.95)      # 기둥
        d = find_doorway(b, (0, 0), RES, [(0.5, 1.0), (2.0, 3.0), (5.0, 3.0)], narrow=0.4)
        self.assertIsNotNone(d)
        self.assertLess(abs(d.center[1] - 3.0), 0.15)
        self.assertGreaterEqual(d.width, 0.7)
        self.assertLess(abs(math.degrees(math.atan2(d.normal[1], d.normal[0]))), 15)

    def test_measure_again_from_rough_guess(self):
        d = measure(room(), (0, 0), RES, (3.0, 2.85), (1.0, 0.3))
        self.assertLess(np.linalg.norm(d.center - [3.0, 3.0]), 0.08)
        self.assertAlmostEqual(d.width, 0.8, delta=0.1)


class CorridorTest(unittest.TestCase):
    """11/11 Gazebo ㄱ자 복도: 폭 1.4 m 복도가 지도에 군데군데 1.1–1.3 m로 잡혀, 복도 한가운데서 문 지나기를 한 판에 3–6번 했다."""

    @staticmethod
    def corridor():
        """x 0.5–5.5, 폭 1.4 m(y 2.3–3.7) 복도. x 2.8–3.2에서 위 벽이 0.2 m 튀어나와(지도 잡음) 그 자리만 1.2 m."""
        b = np.zeros((60, 60), dtype=bool)
        box(b, 0.5, 5.5, 2.1, 2.3)
        box(b, 0.5, 5.5, 3.7, 3.9)
        box(b, 2.8, 3.2, 3.5, 3.7)
        return b

    PATH = [(1.0, 3.0), (5.0, 3.0)]

    def test_stretch_of_corridor_is_not_a_door(self):
        b = self.corridor()
        d = find_doorway(b, (0, 0), RES, self.PATH, narrow=0.6)
        self.assertIsNotNone(d, "좁은 구간이 1 m뿐이라 문 후보로는 잡힌다")
        self.assertGreater(d.width, 1.1)
        self.assertTrue(is_corridor(b, (0, 0), RES, d), "앞뒤 1 m까지 양옆 벽이 같은 간격으로 이어진다")

    def test_real_doors_stay_doors(self):
        d = find_doorway(room(), (0, 0), RES, self.PATH, narrow=0.6)          # 방 사이 0.8 m 문
        self.assertFalse(is_corridor(room(), (0, 0), RES, d), "1.05 m보다 좁으면 앞뒤가 어떻든 문")
        wide = room(door=(2.4, 3.6))                                           # 지도가 넓게 본 문(1.2 m): 앞뒤가 방
        d = find_doorway(wide, (0, 0), RES, self.PATH, narrow=0.6)
        self.assertIsNotNone(d)
        self.assertFalse(is_corridor(wide, (0, 0), RES, d), "문 앞뒤가 트여 있다")
        end = np.zeros((60, 60), dtype=bool)                                   # 1.4 m 복도 끝 벽의 문(1.2 m로 잡힘) 너머가 방
        box(end, 0.5, 2.9, 2.1, 2.3)
        box(end, 0.5, 2.9, 3.7, 3.9)
        box(end, 2.9, 3.1, 0.0, 2.3)
        box(end, 2.9, 3.1, 3.5, 6.0)
        d = find_doorway(end, (0, 0), RES, self.PATH, narrow=0.6)
        self.assertIsNotNone(d)
        self.assertLess(abs(d.center[0] - 3.0), 0.2)
        self.assertFalse(is_corridor(end, (0, 0), RES, d), "문 너머가 트여 있다")

    def test_unseen_beyond_stays_a_door(self):
        b = self.corridor()
        d = find_doorway(b, (0, 0), RES, self.PATH, narrow=0.6)
        b[36:, :] = True                                                       # x 3.6 m 너머는 아직 못 봤다 (모르는 칸은 막힌 칸으로 온다)
        self.assertFalse(is_corridor(b, (0, 0), RES, d))


class ReopenTest(unittest.TestCase):
    """11/11 Gazebo 실습실: 0.75 m 틈으로 들어간 뒤 양쪽 문틀에 ToF가 찍혀 계획 지도에서 틈이 닫혔고, 출발점 3.4 m 앞에 내렸다."""

    def test_axis_of_passed_door_reconnects_both_sides(self):
        from autonomy.doorway import Doorway, open_axis
        from autonomy.planner import PlanError, plan
        from autonomy.voxel_map import VoxelMap
        occ = np.zeros((60, 60, 25), dtype=bool)
        occ[28:32, :, :] = True                                    # x 2.8–3.2 벽: 계획 지도에서는 틈이 통째로 닫혀 있다
        closed = VoxelMap(np.zeros(3), RES, occ)
        here, home = (1.5, 3.0, 1.0), (4.5, 3.0, 1.0)
        with self.assertRaises(PlanError):
            plan(closed, here, home, z_max=1.85)
        went_in = Doorway(np.array([3.0, 3.0]), np.array([-1.0, 0.0]), 0.6, 1.0)     # 들어올 때는 −x로 지났다
        back = Doorway(went_in.center.copy(), -went_in.normal, went_in.width, went_in.z)
        self.assertLess(back.along(here[0], here[1]), -0.4, "지금은 문 앞(안쪽)에 있다")
        opened = open_axis(closed, back, 0.8, 0.8, 1.0)
        self.assertTrue(closed.occupied_at((3.0, 3.0, 1.0)), "원래 지도는 그대로")
        self.assertFalse(opened.occupied_at((3.0, 3.0, 1.0)))
        self.assertTrue(opened.occupied_at((3.0, 3.4, 1.0)), "가운데 선 옆 ±0.1 m만 연다")
        out = back.point(0.8)
        self.assertEqual(len(plan(opened, (float(out[0]), float(out[1]), 1.0), home, z_max=1.85)) >= 1, True)
        path = plan(opened, here, home, z_max=1.85)                # 문 앞에서도 가운데 선으로 이어진다
        self.assertTrue(all(abs(p[1] - 3.0) < 0.2 for p in path if 2.8 < p[0] < 3.2))

    def test_blocked_stretch_of_own_trail_becomes_the_way_back(self):
        # 두 번째 실습실 판: 그 틈을 문 지나기가 아니라 보통 경로(줄인 여유)로 지나 '문 지나기로 지난 틈'에 없었다 → 자취에서 찾는다
        from autonomy.doorway import trail_gap
        trail = [(5.5 - 0.1 * i, 3.9, 1.0) for i in range(30)]            # (5.5 → 2.6): 서쪽으로 틈을 지나 들어옴
        trail += [(2.6, 3.9 + 0.1 * i, 1.0) for i in range(1, 6)]         # 안에서 북쪽으로 0.5 m
        d = trail_gap(trail, lambda p: 3.45 < p[0] < 4.15, 1.6)           # 지나온 x 3.5–4.1이 지금은 막혀 있다
        self.assertIsNotNone(d)
        self.assertAlmostEqual(float(d.center[0]), 3.8, delta=0.06)
        self.assertAlmostEqual(float(d.center[1]), 3.9, delta=0.01)
        self.assertGreater(float(d.normal[0]), 0.99, "되나가는 방향은 들어온 반대(동쪽)")
        self.assertLess(d.along(2.6, 4.4), -0.4, "드론은 그 틈 앞(안쪽)에 있다")
        self.assertIsNone(trail_gap(trail, lambda p: False, 1.6), "자취가 다 열려 있으면 틈 문제가 아니다")
        self.assertIsNone(trail_gap(trail, lambda p: p[0] > 3.45, 1.6), "자취 처음까지 막혀 있다")
        self.assertIsNone(trail_gap(trail, lambda p: 3.0 < p[0] < 5.0, 1.6), "2 m 넘게 막힌 것은 틈이 아니다")
        self.assertIsNone(trail_gap(trail, lambda p: p[0] < 3.0, 1.6), "드론이 선 자리부터 막혀 있다")


class DoorBudgetTest(unittest.TestCase):
    def test_door_target_uses_normal_limits(self):
        b = ViewpointBudget(0.0)
        b.tight = True
        for t in range(4):
            b.note_replan(float(t))
        self.assertIn("좁은 길", b.give_up(4.0), "문이 아닌 좁은 길은 재계획 3번 넘으면 포기 (11/04)")
        b.door = True
        self.assertIsNone(b.give_up(4.0), "문 너머는 문이 원래 좁으므로 평소 한도 (11/05)")
        for t in range(4, 7):
            b.note_replan(float(t))
        self.assertIn("문 너머", b.give_up(7.0))


class PassPeopleTest(unittest.TestCase):
    """곧게 지나는 길에 닿는 사람만 문을 막는다 (11/12 17:19 실기체: 문틀 뒤 사람 기록으로 문 지나기를 세 번 그만뒀다)."""

    @staticmethod
    def door(cx, cy, deg, width):
        return Doorway(np.array([cx, cy]), np.array([math.cos(math.radians(deg)), math.sin(math.radians(deg))]), width)

    def test_gap_is_measured_to_the_straight_segment(self):
        d = self.door(3.0, 3.0, 0.0, 0.8)
        self.assertAlmostEqual(d.pass_gap(3.5, 3.9, 0.8, 0.8), 0.9)    # 옆
        self.assertAlmostEqual(d.pass_gap(2.0, 3.0, 0.8, 0.8), 0.2)    # 문 앞 자리보다 뒤: 문 앞 자리까지
        self.assertAlmostEqual(d.pass_gap(5.0, 3.0, 0.8, 0.8), 1.2)    # 빠져나갈 자리보다 너머: 그 자리까지

    def test_person_behind_the_jamb_does_not_block(self):
        # 11/12 17:19: 다시 잰 문 (7.1–7.2, −2.6) 폭 1.18 m, 방향 −1°·−8°, 찾은 사람 (7.65, −3.50) — 옛 규칙(문 둘레 1.2 m)은 1.03 m라 막았다
        for deg in (-1.0, -8.0):
            d = self.door(7.15, -2.6, deg, 1.18)
            self.assertFalse(d.on_pass(7.65, -3.50, 0.5, 0.8, 0.5))

    def test_person_on_the_line_beyond_a_narrow_door_blocks(self):
        # 같은 비행의 첫 시도: 폭 0.78 m 문 (7.5, −2.9), 방향 −41° — 그 사람은 너머 가운데 선 0.36 m 옆이라 막는다
        self.assertTrue(self.door(7.5, -2.9, -41.0, 0.78).on_pass(7.65, -3.50, 0.8, 0.8, 0.5))

    def test_gap_between_two_people_still_blocks(self):
        # Gazebo people_pair (11/10): 1.2 m 간격 두 사람 사이 0.7 m를 문으로 봤다 → 두 사람 모두 몸 가장자리 0.35 m라 막는다
        d = self.door(0.0, 0.0, 0.0, 0.7)
        self.assertTrue(d.on_pass(0.0, 0.6, 0.8, 0.8, 0.5))
        self.assertTrue(d.on_pass(0.0, -0.6, 0.8, 0.8, 0.5))
        self.assertFalse(d.on_pass(0.0, 1.0, 0.8, 0.8, 0.5))      # 문 옆 1 m에 선 사람은 막지 않는다


if __name__ == "__main__":
    unittest.main()
