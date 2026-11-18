"""물러나기는 들어온 길 쪽으로만 (11/16 Gazebo 방 두 개: 폭 0.9 m 문을 지난 직후 문틀 사이에서 둘러보다, 한쪽 문틀이 정면 0.36 m에
잡혀 '돌기 전에 물러나기'가 걸렸다. 기체 뒤쪽은 들어온 길에서 80° 벗어난 쪽 — 반대편 문틀 — 이었는데 지도에는 '본 빈 곳'이라 곧게
0.35 m 물러나 0.04 m까지 붙었다). 비행 노드의 retreat_check·near_turn·start_backoff를 빌려 본다 (ROS가 없으면 건너뜀)."""
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from autonomy.near import retreat_body  # noqa: E402

try:   # 노드를 띄우지 않고 메서드만 빌려 쓴다
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "nodes"))
    import mission_node
except ImportError:   # pragma: no cover
    mission_node = None


class _Log:
    def __init__(self):
        self.lines = []

    def info(self, text):
        self.lines.append(text)

    warn = error = info


class _Map:
    """카메라 지도 흉내: seen(p) = 카메라가 본 빈 칸, flown(p) = 실제로 지나온 칸."""

    def __init__(self, seen, flown):
        self.seen, self.flown = seen, flown

    def flown_at(self, p):
        return self.flown(p)

    def known_free_at(self, p):
        return self.flown(p) or self.seen(p)


def _lane(p):
    """문을 +x로 지나온 길: y 1.95 둘레 ±0.25 m, x 3.0–4.8 (기체가 지나며 비운 폭)."""
    return 3.0 <= p[0] <= 4.8 and abs(p[1] - 1.95) <= 0.25


def _stub(yaw_deg, seen=lambda p: True, flown=_lane, blocked=lambda p: False, trail=True, mapping=True, front=0.36):
    M = mission_node.Mission

    class Stub:
        retreat_check, start_backoff, near_turn, note_rear, rear_blocked = (M.retreat_check, M.start_backoff, M.near_turn,
                                                                           M.note_rear, M.rear_blocked)

        def __init__(self):
            self.pose = (4.57, 1.97, 1.0, math.radians(yaw_deg))
            self.mapping, self.cmap, self.front = mapping, _Map(seen, flown), front
            # 지나온 자취 (0.1 m마다, 오래된 것 → 최근): 문을 +x로 곧게 지나 여기까지
            self.trail = [(3.2 + 0.1 * k, 1.95) for k in range(15)] if trail else []
            self.correction = type("Same", (), {"apply": staticmethod(lambda p: p)})()
            self.near_spot, self.near_tries, self.slow_turn_until = None, 0, -math.inf
            self.rear_logged = self.rear_last = -math.inf
            self.rear_refused = self.off_trail_refused = 0
            self.t, self.log, self.state, self.backoff, self.stopped = 100.0, _Log(), "LOOK", None, 0
            self.occupied_near = blocked

        def past_xy(self):
            return []

        def moving_fast(self):
            return False

        def now(self):
            return self.t

        def get_logger(self):
            return self.log

        def send(self, *a, **k):
            self.stopped += 1

        def set_state(self, s):
            self.state = s

        def turn(self, then="LOOK"):
            x, y, _, yaw = self.pose
            return self.near_turn(x, y, yaw, then)

    return Stub()


@unittest.skipIf(mission_node is None, "rclpy가 없어 비행 노드를 불러올 수 없다")
class RetreatTest(unittest.TestCase):
    def test_in_the_doorway_after_turning_it_does_not_back_into_the_other_jamb(self):
        # 11/16 Gazebo: 기수 −80°(한쪽 문틀을 봄), 뒤는 +y(반대편 문틀). 지도에는 뒤가 다 '본 빈 곳'이었다
        s = _stub(-80.0)
        self.assertFalse(s.turn())
        self.assertEqual((s.state, s.backoff, s.near_tries), ("LOOK", None, 0))       # 물러나지 않는다
        self.assertGreater(s.slow_turn_until, s.t)                                    # 대신 천천히 돈다
        self.assertEqual((s.rear_refused, s.off_trail_refused), (1, 1))
        self.assertRegex(s.log.lines[-1], r"들어온 길에서 8\d° 벗어나")                 # 자취가 조금 비껴 있어 80°가 아니라 83°
        self.assertIn("물러나지 않고 천천히 돕니다", s.log.lines[-1])
        rear, off, direction, room = s.retreat_check(4.57, 1.97, math.radians(-80.0))
        self.assertIsNone(rear)                                                       # 예전 뒤 확인(본 빈 곳)만으로는 통과했다
        self.assertAlmostEqual(math.degrees(off), 80.0, delta=5.0)
        self.assertAlmostEqual(direction[0], -1.0, places=2)                          # 들어온 길은 −x 쪽

    def test_same_check_guards_every_backoff(self):
        # 문 지나기를 그만둘 때·비상 정지 뒤 물러나기도 같은 start_backoff를 쓴다: 못 물러나면 멈춘 채 다음 단계로
        s = _stub(-80.0)
        self.assertFalse(s.start_backoff(4.57, 1.97, math.radians(-80.0), then="REPLAN"))
        self.assertEqual((s.state, s.backoff), ("REPLAN", None))
        self.assertAlmostEqual(s.slow_turn_until, s.t + mission_node.SLOW_TURN_AFTER)

    def test_straight_in_it_backs_out_the_way_it_came(self):
        s = _stub(0.0)                                                                # 들어온 기수 그대로: 뒤가 들어온 길
        self.assertTrue(s.turn())
        self.assertEqual((s.state, s.near_tries), ("BACKOFF", 1))
        self.assertAlmostEqual(s.backoff["dir"][0], -1.0, places=2)
        self.assertEqual(s.backoff["then"], "LOOK")
        self.assertEqual(s.rear_refused, 0)
        s = _stub(35.0)                                                               # 45° 안으로 비껴 있어도 들어온 길 쪽이다
        self.assertTrue(s.turn())

    def test_turned_around_the_rear_is_the_way_ahead(self):
        s = _stub(180.0)                                                              # 뒤가 +x: 아직 가 보지 않은 쪽
        self.assertFalse(s.turn())
        self.assertRegex(s.log.lines[-1], r"들어온 길에서 1[78]\d° 벗어나")

    def test_rear_it_actually_flew_through_is_fine_even_after_turning(self):
        # 같은 자리에서 뒤쪽 띠를 전에 실제로 지나갔다면(지나온 칸) 돈 뒤라도 물러난다
        s = _stub(-90.0, flown=lambda p: _lane(p) or (abs(p[0] - 4.57) <= 0.25 and 1.7 <= p[1] <= 2.8))
        self.assertTrue(s.turn())
        self.assertEqual(s.state, "BACKOFF")

    def test_no_way_in_yet_means_no_backing(self):
        # 이륙한 자리에서 둘러보는 중: 들어온 길이 없다. 뒤가 '본 빈 곳'이어도 물러나지 않는다
        s = _stub(-80.0, trail=False, flown=lambda p: math.hypot(p[0] - 4.57, p[1] - 1.97) <= 0.2)
        self.assertFalse(s.turn())
        self.assertIn("들어온 길이 없어", s.log.lines[-1])
        self.assertEqual(s.off_trail_refused, 1)

    def test_no_room_to_turn_keeps_following_the_trail(self):
        # 둘레 0.25 m 안이 막혀 돌 자리가 없으면 전처럼 자취 방향(지나온 선)으로 간다 — 기체 뒤쪽으로 곧게가 아니다
        s = _stub(-80.0, blocked=lambda p: math.hypot(p[0] - 4.57, p[1] - 1.97) >= 0.15 and abs(p[1] - 1.97) > 0.1)
        self.assertTrue(s.turn())
        self.assertLess(s.backoff["room"], mission_node.TURN_ROOM)
        vx, vy = retreat_body(s.backoff["dir"], math.radians(-80.0), 0.18, s.backoff["room"], mission_node.TURN_ROOM, 0.0,
                              mission_node.TRAIL_SIDEWAYS)
        wx = vx * math.cos(math.radians(-80.0)) - vy * math.sin(math.radians(-80.0))   # 세계 좌표 x 성분
        self.assertAlmostEqual(wx, -0.18, places=2)                                    # 들어온 −x 쪽으로

    def test_too_close_to_turn_in_place_still_backs_off(self):
        # 정면이 0.25 m 안으로 바짝 붙었으면 제자리에서 돌 때 모서리가 닿는다: 뒤가 들어온 길이 아니어도 전처럼 (본 빈 곳으로) 물러난다
        s = _stub(-80.0, front=0.20)
        self.assertTrue(s.turn())
        self.assertEqual((s.state, s.off_trail_refused), ("BACKOFF", 0))
        s = _stub(-80.0, front=0.26)                                                  # 돌 자리가 있으면 물러나지 않는다
        self.assertFalse(s.turn())
        s = _stub(-80.0, front=math.inf)                                              # 정면이 빈 채 물러나려 할 때(문 지나기를 그만둠)도
        self.assertFalse(s.start_backoff(4.57, 1.97, math.radians(-80.0)))
        self.assertEqual(s.off_trail_refused, 1)

    def test_unseen_rear_still_refuses_with_the_old_reason(self):
        s = _stub(0.0, seen=lambda p: False, flown=lambda p: p[0] >= 4.3 and _lane(p))   # 뒤 0.3 m부터 모르는 곳
        self.assertFalse(s.turn())
        self.assertIn("카메라가 본 적 없는 곳", s.log.lines[-1])
        self.assertEqual((s.rear_refused, s.off_trail_refused), (1, 0))

    def test_given_map_flights_are_unchanged(self):
        s = _stub(-80.0, mapping=False)                                               # 받은 지도로 나는 1–3단계: 카메라 지도가 없다
        self.assertTrue(s.turn())
        self.assertEqual(s.state, "BACKOFF")

    def test_one_event_is_counted_once(self):
        s = _stub(-80.0)
        for _ in range(20):                                                           # 막힌 동안 매 주기 다시 걸린다
            s.turn()
            s.t += 0.05
        self.assertEqual((s.rear_refused, s.off_trail_refused, len(s.log.lines)), (1, 1, 1))


if __name__ == "__main__":
    unittest.main()
