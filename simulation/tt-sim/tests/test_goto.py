"""운용자가 웹 지도에서 찍은 자리 (autonomy/goto.py): 명령 읽기와, 막힌·못 본 곳을 찍었을 때 갈 자리로 옮기기."""
import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from autonomy.goto import SNAP_MAX, parse, snap  # noqa: E402

RES = 0.1
ORIGIN = (-1.0, -2.0)


def cell(x, y):
    return int(math.floor((x - ORIGIN[0]) / RES)), int(math.floor((y - ORIGIN[1]) / RES))


def room(x0, y0, x1, y1, shape=(80, 60)):
    """shape 칸 지도에서 (x0, y0)–(x1, y1) 직사각형만 빈 곳(본 빈 곳)."""
    free = np.zeros(shape, dtype=bool)
    (i0, j0), (i1, j1) = cell(x0, y0), cell(x1, y1)
    free[i0:i1, j0:j1] = True
    return free


class ParseTest(unittest.TestCase):
    def test_goto_and_cancel(self):
        self.assertEqual(parse("goto 1.25 -3.5"), ("goto", 1.25, -3.5))
        self.assertEqual(parse("goto cancel"), ("cancel",))

    def test_other_commands_are_not_goto(self):
        for text in ("start", "land", "", "gotox 1 2"):
            self.assertIsNone(parse(text), text)

    def test_malformed_goto_is_an_error(self):
        for text in ("goto", "goto 1", "goto 1 2 3", "goto a b", "goto nan 1", "goto 1 inf", "goto 5000 0"):
            with self.assertRaises(ValueError, msg=text):
                parse(text)


class SnapTest(unittest.TestCase):
    def test_free_click_is_kept_exactly(self):
        free = room(0.0, 0.0, 4.0, 3.0)
        self.assertEqual(snap(free, ORIGIN, RES, (0.5, 0.5), (3.13, 2.27)), (3.13, 2.27, 0.0))

    def test_free_click_in_another_piece_is_kept(self):
        # 문 너머 방처럼 이 단면에서는 이어지지 않아도, 찍은 칸이 비었으면 그대로 간다 (길은 경로 계획이 본다: 문 지나기·가구 위)
        free = room(0.0, 0.0, 2.0, 3.0) | room(2.5, 0.0, 4.0, 3.0)
        self.assertEqual(snap(free, ORIGIN, RES, (0.5, 0.5), (3.0, 1.0)), (3.0, 1.0, 0.0))

    def test_click_just_past_the_seen_edge_moves_inside_with_clearance(self):
        # 본 곳 가장자리(x = 4.0) 바로 밖을 찍었다: 이어진 빈 곳 가운데 가장 가깝되, 둘레 0.2 m가 빈 칸으로
        free = room(0.0, 0.0, 4.0, 3.0)
        x, y, moved = snap(free, ORIGIN, RES, (0.5, 0.5), (4.3, 1.53))
        self.assertTrue(free[cell(x, y)])
        self.assertAlmostEqual(y, 1.55, places=2)
        self.assertAlmostEqual(x, 3.75, places=2)        # 가장자리에서 두 칸 안쪽 칸의 가운데
        self.assertAlmostEqual(moved, math.hypot(4.3 - x, 1.53 - y), places=6)

    def test_narrow_strip_has_no_roomy_cell_so_nearest_free_cell(self):
        free = room(0.0, 1.0, 4.0, 1.2)                  # 폭 0.2 m 띠: 둘레가 빈 칸이 없다
        x, y, moved = snap(free, ORIGIN, RES, (0.5, 1.1), (2.03, 1.6))
        self.assertTrue(free[cell(x, y)])
        self.assertAlmostEqual(x, 2.05, places=2)
        self.assertAlmostEqual(y, 1.15, places=2)

    def test_blocked_click_moves_to_the_drone_side_not_the_other_room(self):
        # 두 방 사이 벽(x 2.0–2.5)을 찍었다. 건넛방이 더 가까워도 드론과 이어진 쪽으로 옮긴다
        free = room(0.0, 0.0, 2.0, 3.0) | room(2.5, 0.0, 4.0, 3.0)
        x, y, moved = snap(free, ORIGIN, RES, (0.5, 0.5), (2.4, 1.5))
        self.assertLess(x, 2.0)
        self.assertTrue(free[cell(x, y)])

    def test_far_click_goes_as_near_as_the_drone_can_get(self):
        # 본 곳(x < 2) 너머 2.1 m를 찍었다. 드론이 방 안쪽에 있으면 그쪽 가장자리까지 가는 것이 뜻에 맞다 (11/11 Gazebo 방 두 개)
        free = room(0.0, 0.0, 2.0, 3.0)
        x, y, moved = snap(free, ORIGIN, RES, (0.5, 0.5), (4.1, 1.53))
        self.assertAlmostEqual(x, 1.75, places=2)
        self.assertAlmostEqual(y, 1.55, places=2)
        self.assertGreater(moved, SNAP_MAX)

    def test_refused_when_the_drone_is_already_as_near_as_it_can_get(self):
        # 이미 그 가장자리에 와 있다: 더 가까운 갈 곳이 없으면(0.5 m도 못 줄임) 받지 않는다. 찍은 곳 1.5 m 안이면 그래도 간다
        free = room(0.0, 0.0, 2.0, 3.0)
        self.assertIsNone(snap(free, ORIGIN, RES, (1.8, 1.5), (4.1, 1.53)))
        self.assertIsNotNone(snap(free, ORIGIN, RES, (1.8, 1.5), (3.2, 1.53)))

    def test_nearest_free_cell_when_the_roomy_one_is_much_farther(self):
        # 폭 0.3 m 통로(둘레가 빈 칸 없음) 끝 너머를 찍었다. 둘레가 빈 칸은 2 m 뒤 방에만 있다: 거기로 물러나지 않고 통로 끝으로 간다
        free = room(0.0, 0.0, 2.0, 3.0) | room(2.0, 1.4, 4.0, 1.7)
        x, y, moved = snap(free, ORIGIN, RES, (0.5, 1.5), (4.6, 1.55))
        self.assertAlmostEqual(x, 3.95, places=2)
        self.assertTrue(free[cell(x, y)])

    def test_not_exact_moves_even_when_the_clicked_cell_is_free(self):
        # 찍은 칸은 비었지만 거기까지 길이 없었다(다른 덩어리): 드론과 이어진 쪽의 가장 가까운 자리로
        free = room(0.0, 0.0, 2.0, 3.0) | room(2.5, 0.0, 4.0, 3.0)
        x, y, moved = snap(free, ORIGIN, RES, (0.5, 0.5), (3.5, 1.53), exact=False)
        self.assertLess(x, 2.0)
        self.assertAlmostEqual(moved, math.hypot(3.5 - x, 1.53 - y), places=6)

    def test_click_outside_the_map_is_handled(self):
        free = room(0.0, 0.0, 2.0, 3.0)
        far = snap(free, ORIGIN, RES, (1.0, 1.5), (-50.0, 80.0))     # 터무니없이 먼 곳: 그 방향 가장자리까지
        self.assertTrue(free[cell(far[0], far[1])])
        self.assertGreater(far[1], 2.5)
        near = snap(free, ORIGIN, RES, (0.5, 0.5), (-1.3, 1.0))   # 지도 밖이지만 빈 곳에서 1.5 m 안
        self.assertIsNotNone(near)
        self.assertTrue(free[cell(near[0], near[1])])

    def test_drone_cell_blocked_by_inflation_uses_nearby_free_piece(self):
        # 드론이 부풀린 벽 칸 위에 있다(벽에서 0.2 m): 0.4 m 안의 빈 칸과 이어진 곳에서 찾는다
        free = room(0.0, 0.0, 4.0, 3.0)
        got = snap(free, ORIGIN, RES, (-0.25, 1.0), (4.3, 1.5))
        self.assertIsNotNone(got)
        # 빈 칸이 0.4 m 안에 없으면(지도가 아직 비었다) 받지 않는다
        self.assertIsNone(snap(free, ORIGIN, RES, (-0.9, 1.0), (4.3, 1.5)))

    def test_empty_map_is_refused(self):
        self.assertIsNone(snap(np.zeros((10, 10), dtype=bool), ORIGIN, RES, (0.0, 0.0), (0.2, 0.2)))


try:   # 비행 노드의 받기·잡기 흐름: ROS(rclpy)가 있을 때만 (노드를 띄우지 않고 메서드만 빌려 쓴다)
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "nodes"))
    import mission_node
    from std_msgs.msg import String
except ImportError:   # pragma: no cover
    mission_node = None


class _Log:
    def __init__(self):
        self.lines = []

    def info(self, text):
        self.lines.append(text)

    warn = info


class _Pub:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg.data)


def _stub(state="FOLLOW", free=None):
    """Mission의 찍은 자리 메서드를 빌린 가짜 비행 노드. 지도는 높이 한 층짜리."""
    import json

    from autonomy import ObstacleMemory, VoxelMap
    from autonomy.explore import PeopleLog

    M = mission_node.Mission

    class Stub:
        goto_note, goto_end, search_elapsed, goto_refusal = M.goto_note, M.goto_end, M.search_elapsed, M.goto_refusal
        goto_request, goto_cancel, goto_take, goto_choose = M.goto_request, M.goto_cancel, M.goto_take, M.goto_choose
        goto_spot, goto_fallback, on_mission_cmd = M.goto_spot, M.goto_fallback, M.on_mission_cmd
        goto_relax, goto_relaxed_text = M.goto_relax, staticmethod(M.goto_relaxed_text)
        rebuild_maps, blocking_people, restore_after_people = M.rebuild_maps, M.blocking_people, M.restore_after_people
        goto_after, search_over = M.goto_after, M.search_over

        def __init__(self):
            self.state, self.search, self.search_time, self.search_start, self.t = state, True, 300.0, 100.0, 150.0
            self.returning, self.return_t0, self.return_best, self.return_retargeted = False, None, math.inf, False
            self.return_home = True
            self.goto_pending, self.goto_active, self.goto_drop, self.goto_seq, self.goto_last = None, None, False, 0, None
            self.goto_count, self.goto_pub, self.log = [0, 0], _Pub(), _Log()
            self.height, self.mapping, self.look_at, self.goal, self.goal_area = 1.0, True, None, None, 5.0
            self.vp_budget, self.leg_from, self.stopped = "old", None, 0
            self.approached, self.approach_pending = [], None
            self.map_radius = self.map_radius_tight = 0.2   # 지도 하나로 본다 (여유를 줄여 다시 만드는 것은 Gazebo에서 확인)
            occ = ~(free if free is not None else room(0.0, 0.0, 4.0, 3.0))
            # 찾은 장애물·사람을 얹는 바탕 지도 (rebuild_maps가 여기에 상자를 더해 inflated를 만든다)
            self.known = self.known_inflated = VoxelMap(np.array([ORIGIN[0], ORIGIN[1], 0.9]), RES, occ[:, :, None].repeat(2, axis=2))
            self.avoid, self.memory, self.people = True, ObstacleMemory(), PeopleLog(merge=1.0)
            self.found_radius, self.person_margin = mission_node.FOUND_RADIUS, mission_node.PERSON_MARGIN
            self.people_soft, self.drop_weak_people, self.people_relaxed_at, self.saved = False, False, None, 0
            self.goto_weak_dropped, self.base_height = False, 1.0
            self.cmap = type("Flown", (), {"flown": np.zeros(self.known.occ.shape, dtype=bool)})()
            self.rebuild_maps()

        def save_discovered(self):
            self.saved += 1

        def now(self):
            return self.t

        def get_logger(self):
            return self.log

        def send(self):
            self.stopped += 1

        def set_state(self, s):
            self.state = s

        def last(self):
            return json.loads(self.goto_pub.sent[-1])

    return Stub()


@unittest.skipIf(mission_node is None, "rclpy가 없어 비행 노드를 불러올 수 없다")
class MissionGotoTest(unittest.TestCase):
    def test_click_while_following_becomes_the_next_viewpoint(self):
        s = _stub("FOLLOW")
        s.on_mission_cmd(String(data="goto 3.13 2.27"))
        self.assertEqual(s.goto_pending, (3.13, 2.27))
        self.assertEqual(s.last()["state"], "pending")
        self.assertTrue(s.goto_take())                      # 가던 길을 멈추고 볼 자리를 다시 고르러
        self.assertEqual((s.state, s.stopped), ("PLAN", 1))
        self.assertTrue(s.goto_choose(0.5, 0.5, 50.0))
        self.assertEqual([float(v) for v in s.goal], [3.13, 2.27, 1.0])
        self.assertIsNone(s.goto_pending)
        self.assertIsNone(s.vp_budget)                      # 볼 자리 한도는 새로 잡는다
        self.assertIsNone(s.look_at)                        # 찍은 칸으로 그대로 가면 보는 쪽은 평소대로
        msg = s.last()
        self.assertEqual((msg["state"], msg["goal"], msg["click"]), ("going", [3.13, 2.27], [3.13, 2.27]))
        self.assertFalse(s.goto_take())                     # 한 번 잡은 뒤에는 가던 길을 끊지 않는다

    def test_unseen_click_goes_to_the_nearest_reachable_spot_and_looks_there(self):
        s = _stub("PLAN")
        s.goto_pending = (4.6, 1.53)                        # 본 곳 가장자리(x 4.0) 밖
        self.assertTrue(s.goto_choose(0.5, 0.5, 50.0))
        self.assertAlmostEqual(float(s.goal[0]), 3.75, places=2)
        self.assertEqual(s.look_at, (4.6, 1.53))            # 도착해서 찍은 쪽부터 본다
        self.assertEqual(s.last()["click"], [4.6, 1.53])
        self.assertIn("막혔거나 아직 못 본 곳", s.last()["text"])

    def test_click_the_drone_cannot_get_nearer_to_is_dropped_and_exploration_goes_on(self):
        s = _stub("PLAN")
        s.goto_pending = (7.5, 1.5)                         # 본 곳(x < 4) 너머 3.5 m, 드론은 이미 그 가장자리(3.7)에 있다
        self.assertFalse(s.goto_choose(3.7, 1.5, 50.0))
        self.assertIsNone(s.goto_active)
        self.assertIsNone(s.goal)
        self.assertEqual(s.last()["state"], "none")
        self.assertIn("더 다가갈 수 있는 본 빈 곳이 없습니다", s.last()["text"])

    def test_far_click_from_the_other_side_goes_to_the_edge_toward_it(self):
        s = _stub("PLAN")
        s.goto_pending = (7.5, 1.53)
        self.assertTrue(s.goto_choose(0.5, 0.5, 50.0))      # 방 반대쪽에 있으면 그쪽 가장자리까지 간다
        self.assertAlmostEqual(float(s.goal[0]), 3.75, places=2)
        self.assertEqual(s.look_at, (7.5, 1.53))

    def test_no_path_to_a_free_clicked_cell_falls_back_once_to_the_nearest_reachable_spot(self):
        # 두 방이 지도에서 아직 이어지지 않았다. 건넛방의 빈 칸을 찍으면 일단 그리로 잡고, 길이 없으면 이쪽 방의 가장 가까운 자리로 옮긴다
        s = _stub("PLAN", free=room(0.0, 0.0, 2.0, 3.0) | room(2.5, 0.0, 4.0, 3.0))
        s.goto_pending = (3.5, 1.53)
        self.assertTrue(s.goto_choose(0.5, 0.5, 50.0))
        self.assertEqual([float(v) for v in s.goal[:2]], [3.5, 1.53])
        self.assertTrue(s.goto_fallback(0.5, 0.5, "길 없음"))
        self.assertAlmostEqual(float(s.goal[0]), 1.75, places=2)
        self.assertEqual((s.state, s.look_at), ("REPLAN", (3.5, 1.53)))
        msg = s.last()
        self.assertEqual((msg["state"], msg["click"]), ("going", [3.5, 1.53]))
        self.assertAlmostEqual(msg["goal"][0], 1.75, places=2)
        self.assertIn("아직 길이 이어지지 않아", msg["text"])
        self.assertFalse(s.goto_fallback(0.5, 0.5, "길 없음"))   # 한 번만: 옮긴 자리로도 길이 없으면 그만둔다

    def test_no_fallback_after_the_spot_was_already_moved(self):
        s = _stub("PLAN")
        s.goto_pending = (4.6, 1.53)
        s.goto_choose(0.5, 0.5, 50.0)                       # 이미 갈 수 있는 가장 가까운 자리로 옮겼다
        self.assertFalse(s.goto_fallback(0.5, 0.5, "길 없음"))

    def test_interrupted_blocked_frontier_approach_can_be_chosen_again(self):
        s = _stub("FOLLOW")
        s.approached, s.approach_pending = [(1.0, 1.0), (5.1, 3.1)], (5.1, 3.1)   # 막힌 경계 (5.1, 3.1)을 보러 가는 중
        s.on_mission_cmd(String(data="goto 2.0 2.0"))
        self.assertTrue(s.goto_take())
        self.assertEqual((s.approached, s.approach_pending), ([(1.0, 1.0)], None))

    def test_refused_outside_exploration(self):
        for change, word in ((dict(search=False), "탐색 중에만"), (dict(state="TAKEOFF"), "지금 단계(TAKEOFF)"),
                             (dict(state="HOMESPIN"), "지금 단계(HOMESPIN)"), (dict(state="LAND"), "지금 단계(LAND)")):
            s = _stub("FOLLOW")
            for k, v in change.items():
                setattr(s, k, v)
            s.on_mission_cmd(String(data="goto 1.0 1.0"))
            self.assertIsNone(s.goto_pending, change)
            self.assertIn(word, s.last()["text"])
            self.assertTrue(s.last()["warn"])               # 거절은 웹 화면 버튼 옆에 경고색으로 보인다
            self.assertFalse(s.goto_take())

    def test_click_after_the_search_time_is_taken_and_then_it_goes_home(self):
        # 11/16 15:33 실기체: 탐색 시간(120초)이 끝나 돌아가던 중(2초·16초·24초 뒤) 찍은 세 자리가 모두 '탐색 시간이 다 됐습니다'로
        # 거절됐고 기체는 그대로 돌아와 내렸다. 운용자가 찍은 것은 시간 한도보다 앞선다: 다녀온 뒤 다시 돌아간다
        s = _stub("FOLLOW")
        s.t = 100.0 + 300.0 + 2.3
        s.returning, s.return_t0, s.return_retargeted = True, 400.0, True
        s.on_mission_cmd(String(data="goto 3.0 2.0"))
        self.assertEqual(s.goto_pending, (3.0, 2.0))
        msg = s.last()
        self.assertEqual((msg["state"], msg["warn"]), ("pending", False))
        self.assertIn("탐색 시간(300초)은 다 됐지만", msg["text"])
        self.assertIn("돌아가던 것을 멈추고", msg["text"])
        self.assertIn("둘러본 뒤 출발점으로 돌아갑니다", msg["text"])
        self.assertTrue(s.goto_take())
        self.assertEqual((s.returning, s.return_t0, s.return_retargeted, s.state), (False, None, False, "PLAN"))
        self.assertTrue(s.goto_choose(0.5, 0.5, s.search_elapsed()))
        self.assertEqual([float(v) for v in s.goal[:2]], [3.0, 2.0])
        self.assertEqual(s.last()["state"], "going")
        self.assertIn("둘러본 뒤 출발점으로 돌아갑니다 (탐색 시간이 다 됨)", s.last()["text"])

    def test_time_running_out_before_it_is_taken_still_goes(self):
        s = _stub("LOOK")
        s.on_mission_cmd(String(data="goto 1.0 1.0"))
        self.assertNotIn("탐색 시간", s.last()["text"])     # 받을 때는 시간이 남아 있었다
        self.assertTrue(s.goto_choose(0.5, 0.5, 300.0))     # 둘러보는 사이 탐색 시간이 다 됐다: 그래도 다녀온다
        self.assertIsNone(s.goto_pending)
        self.assertIn("출발점으로 돌아갑니다 (탐색 시간이 다 됨)", s.last()["text"])

    def test_what_comes_after_the_clicked_spot(self):
        s = _stub("LOOK")
        self.assertEqual(s.goto_after(), "자율 탐색을 이어갑니다")
        s.t = 500.0
        self.assertEqual(s.goto_after(), "탐색 시간(300초)이 다 됐으므로 출발점으로 돌아갑니다")
        s.return_home = False
        self.assertEqual(s.goto_after(), "탐색 시간(300초)이 다 됐으므로 그 자리에 내립니다")
        s.goto_active = {"click": (3.0, 2.0), "goal": (3.0, 2.0), "arrived": False}
        s.on_mission_cmd(String(data="goto cancel"))
        self.assertIn("그 자리에 내립니다", s.last()["text"])

    def test_click_while_returning_with_time_left_resumes_exploration(self):
        s = _stub("FOLLOW")
        s.returning, s.return_t0, s.return_retargeted = True, 140.0, True
        s.on_mission_cmd(String(data="goto 2.0 2.0"))
        self.assertIn("탐색으로 돌아갑니다", s.last()["text"])
        self.assertTrue(s.goto_take())
        self.assertEqual((s.returning, s.return_t0, s.return_retargeted, s.state), (False, None, False, "PLAN"))

    def test_cancel_on_the_way_picks_a_viewpoint_again(self):
        s = _stub("FOLLOW")
        s.goto_pending = (3.0, 2.0)
        s.goto_take()
        s.goto_choose(0.5, 0.5, 50.0)
        s.state = "FOLLOW"
        s.on_mission_cmd(String(data="goto cancel"))
        self.assertIsNone(s.goto_active)
        self.assertEqual(s.last()["state"], "none")
        self.assertTrue(s.goto_take())                      # 가던 길을 멈추고 자율로 다시 고른다
        self.assertEqual(s.state, "PLAN")
        self.assertFalse(s.goto_drop)

    def test_cancel_after_arrival_lets_the_look_finish(self):
        s = _stub("LOOK")
        s.goto_active = {"click": (3.0, 2.0), "goal": (3.0, 2.0), "arrived": True}
        s.on_mission_cmd(String(data="goto cancel"))
        self.assertFalse(s.goto_drop)
        self.assertFalse(s.goto_take())

    # ---- 찾은 장애물·사람 둘레가 길을 막을 때 (11/12 10:07 실기체: 카메라 지도로는 비었는데 '더 다가갈 곳이 없습니다'로 거절)

    @staticmethod
    def _two_rooms_with_people(box_source):
        """아래 방(y 0–1.2)과 위 방(y 1.5–3)이 폭 0.8 m 입구(x 1.6–2.4)로 이어진다. 입구 왼쪽을 찾은 장애물 둘레가,
        오른쪽을 찾은 사람 둘레가 덮어 닫았다 (지도 자체는 비어 있다). 드론은 아래 방."""
        s = _stub("PLAN", free=room(0.0, 0.0, 4.0, 1.2) | room(0.0, 1.5, 4.0, 3.0) | room(1.6, 1.2, 2.4, 1.5))
        s.memory.add({"x": 1.6, "y": 1.0, "z": 0.9, "sx": 0.5, "sy": 0.5, "sz": 1.8}, box_source)
        s.people.add(2.6, 1.35, 0.9, 120.0).hits = 20
        s.rebuild_maps()
        return s

    def test_camera_only_obstacle_is_dropped_to_reach_the_click(self):
        s = self._two_rooms_with_people("yolo:person")
        self.assertIsNone(s.goto_spot(2.0, 0.5, 2.0, 2.6, exact=False))   # 그대로는 위 방에 못 간다 (이미 입구 앞)
        (gx, gy, moved), done = s.goto_relax(2.0, 0.5, 2.0, 2.6)
        self.assertLess(moved, 0.3)                                       # 찍은 곳 바로 옆까지
        self.assertEqual(len(done), 2)                                    # 장애물 여유 0.2 → 카메라로만 본 장애물 빼기에서 멈춤
        self.assertIn("카메라로만 본 장애물 1개", done[1])
        self.assertEqual((s.memory.items, len(s.memory.suspects), s.saved), ([], 1, 1))   # 뺀 자리는 기억한다
        self.assertEqual((s.person_margin, s.people_soft, s.people_relaxed_at), (mission_node.PERSON_MARGIN, False, None))   # 사람 둘레는 그대로

    def test_person_margin_is_tightened_when_the_obstacle_is_not_camera_only(self):
        s = self._two_rooms_with_people("tof")                            # ToF가 본 장애물은 빼지 않는다
        (gx, gy, moved), done = s.goto_relax(2.0, 0.5, 2.0, 2.6)
        self.assertLess(moved, 0.3)
        self.assertEqual((len(s.memory.items), s.found_radius), (1, mission_node.FOUND_RADIUS_TIGHT))
        self.assertEqual((s.person_margin, s.people_soft), (mission_node.PERSON_MARGIN_TIGHT, False))
        self.assertEqual(s.people_relaxed_at, (2.0, 0.5))                 # 이 자리에서 벗어나면 되돌린다 (restore_after_people)
        self.assertIn("찾은 사람 둘레 여유를 0.15 m", done[-1])

    def test_weak_person_dropped_for_the_click_blocks_again_after_leaving(self):
        # 몇 번만 잡힌 사람(3번)이 입구 한가운데를 막았다: 둘레를 0.15 m로 줄여도 안 열려 그 사람을 뺀다. 2 m 벗어나면 다시 막는다
        s = _stub("PLAN", free=room(0.0, 0.0, 4.0, 1.2) | room(0.0, 1.5, 4.0, 3.0) | room(1.6, 1.2, 2.4, 1.5))
        s.people.add(2.0, 1.35, 0.9, 120.0).hits = 3
        s.rebuild_maps()
        (gx, gy, moved), done = s.goto_relax(2.0, 0.2, 2.0, 2.6)
        self.assertIn("몇 번만 잡힌 사람 1명을 빼고", done[-1])
        self.assertEqual((s.drop_weak_people, s.goto_weak_dropped, s.people_relaxed_at), (True, True, (2.0, 0.2)))
        s.restore_after_people(2.0, 1.0)                                  # 아직 0.8 m: 그대로
        self.assertTrue(s.drop_weak_people)
        s.restore_after_people(2.0, 2.6)                                  # 2.4 m 벗어남: 되돌린다
        self.assertEqual((s.drop_weak_people, s.goto_weak_dropped, s.person_margin, s.people_relaxed_at),
                         (False, False, mission_node.PERSON_MARGIN, None))
        self.assertTrue(s.inflated.occupied_at((2.0, 1.35, 1.0)))         # 그 사람이 다시 지도에 있다

    def test_relaxing_is_undone_when_it_does_not_open_the_way(self):
        s = self._two_rooms_with_people("tof")
        s.memory.add({"x": 2.0, "y": 1.35, "z": 0.9, "sx": 1.0, "sy": 0.3, "sz": 1.8}, "tof")   # 입구를 ToF 장애물이 통째로 막았다
        s.rebuild_maps()
        before = s.inflated.occ.copy()
        self.assertIsNone(s.goto_relax(2.0, 0.5, 2.0, 2.6))
        self.assertEqual((s.found_radius, s.person_margin, s.people_soft, s.drop_weak_people, len(s.memory.items)),
                         (mission_node.FOUND_RADIUS, mission_node.PERSON_MARGIN, False, False, 2))
        self.assertTrue(np.array_equal(s.inflated.occ, before))

    def test_click_beyond_the_blocked_opening_is_reached_after_relaxing(self):
        # 위 방 가장자리 밖(못 본 곳)을 찍었다: 그대로면 아래 방 입구 앞(찍은 곳에서 2 m 넘게)까지만 — 여유를 줄이면 위 방 가장자리까지
        s = self._two_rooms_with_people("yolo:person")
        s.goto_pending = (2.03, 3.3)
        self.assertTrue(s.goto_choose(2.0, 0.2, 50.0))
        self.assertGreater(float(s.goal[1]), 2.5)
        self.assertEqual(s.look_at, (2.03, 3.3))
        self.assertIn("찾은 장애물·사람 둘레가 막고 있어", s.last()["text"])
        self.assertIn("카메라로만 본 장애물 1개를 빼고", s.last()["text"])
        self.assertEqual(s.last()["state"], "going")

    def test_no_path_to_the_click_relaxes_before_settling_for_a_nearer_spot(self):
        s = self._two_rooms_with_people("yolo:person")
        s.goto_pending = (2.0, 2.6)                                       # 위 방의 빈 칸: 일단 그 칸으로 잡는다
        self.assertTrue(s.goto_choose(2.0, 0.5, 50.0))
        self.assertEqual([float(v) for v in s.goal[:2]], [2.0, 2.6])
        self.assertTrue(s.goto_fallback(2.0, 0.5, "길 없음"))             # 길이 없다: 여유를 줄여 같은 곳으로 다시 계획
        self.assertLess(math.hypot(float(s.goal[0]) - 2.0, float(s.goal[1]) - 2.6), 0.3)
        self.assertEqual((s.state, s.memory.items), ("REPLAN", []))
        self.assertIn("카메라로만 본 장애물 1개를 빼고", s.last()["text"])
        self.assertFalse(s.goto_fallback(2.0, 0.5, "길 없음"))            # 더 줄일 것도, 더 가까운 자리도 없으면 그만둔다

    def test_bad_command_text_is_ignored(self):
        s = _stub("FOLLOW")
        s.on_mission_cmd(String(data="goto 1.0"))
        self.assertIsNone(s.goto_pending)
        self.assertEqual(s.goto_pub.sent, [])
        self.assertIn("읽지 못했습니다", s.log.lines[-1])


if __name__ == "__main__":
    unittest.main()
