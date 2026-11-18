"""닿으면 반대쪽으로 떼어 내고 머문다 (11/13 13:51 실기체: 입력 0만 보내자 Tello가 왼쪽 벽 쪽으로 12초 동안 계속 밀었다.
17:44 실기체: 0.8초 밀어 5 cm만 떨어져 같은 기둥에 두 번 더 닿고 '같은 곳 세 번'으로 내렸다 → 거리로 밀고 한 사건은 한 번으로 센다).
비행 노드 메서드를 빌린 가짜 노드로 본다 (ROS가 없으면 건너뜀). 감지 자체는 test_contact."""
import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from autonomy.contact import ContactWatch, PushWatch  # noqa: E402
from autonomy.doorway import Doorway  # noqa: E402
from autonomy.obstacles import ObstacleMemory  # noqa: E402

try:   # 노드를 띄우지 않고 메서드만 빌려 쓴다
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "nodes"))
    import mission_node
except ImportError:   # pragma: no cover
    mission_node = None

DT = 0.05


class _Log:
    def __init__(self):
        self.lines = []

    def info(self, text):
        self.lines.append(text)

    warn = error = info


class _Map:
    """계획 지도 흉내: blocked(x, y)가 참인 곳이 막힌 칸."""

    def __init__(self):
        self.blocked = lambda x, y: False

    def occupied_at(self, p):
        return bool(self.blocked(p[0], p[1]))


def _stub():
    M = mission_node.Mission

    class Stub:
        contact_check, start_unstick, unstick_step = M.contact_check, M.start_unstick, M.unstick_step
        mark_contact, push_room, push_hint = M.mark_contact, M.push_room, M.push_hint

        def __init__(self):
            self.mapping, self.known_here, self.memory, self.map_dirty = False, True, ObstacleMemory(), False
            self.t, self.log, self.sent, self.boxes, self.states, self.fails = 10.0, _Log(), [], [], [], []
            self.state, self.watch, self.contact_hit, self.contact_armed = "DOOR", ContactWatch(), None, 0.0
            self.contacts, self.contact_count, self.contact_until, self.unstick, self.contact_last = [], 0, 0.0, None, None
            self.contact_episode, self.contact_pushes, self.believed = None, 0, _Map()
            self.push, self.stall_count = PushWatch(), 0
            self.slow_turn_until, self.tof_down, self.ceiling = -math.inf, 1.4, 2.5
            self.pose = self.raw_pose = (2.2, -0.26, 1.4, math.radians(80))
            self.last_cmd = (0.0, 0.0, 0.0, 0.0)
            self.params = type("P", (), {"vz_max": 0.5, "k_pos": 1.0})()
            self.door = {"d": Doorway(np.array([2.4, 0.5]), np.array([0.3, 0.95]), 0.6)}
            self.landed, self.moves = None, True

        def now(self):
            return self.t

        def get_logger(self):
            return self.log

        def send(self, vx=0.0, vy=0.0, vz=0.0, rate=0.0):
            self.sent.append((vx, vy, vz, rate))
            self.last_cmd = (vx, vy, vz, rate)

        def set_state(self, s):
            self.state = s
            self.states.append(s)

        def add_obstacle(self, box, source, why):
            self.boxes.append((box, source))

        def door_fail(self, center, add=False):
            self.fails.append(tuple(center))
            return len(self.fails)

        def start_landing(self, x, y):
            self.landed = (x, y)
            self.set_state("LAND")

        def camera_knows(self, p):
            return self.known_here

        def rebuild_maps(self):
            pass

        def save_discovered(self):
            pass

        def tick(self):
            """한 틱(DT): 닿음 검사 → 단계 처리 → 보낸 속도대로 자리를 옮긴다 (moves가 참일 때)."""
            self.t += DT
            x, y, z, yaw = self.pose
            if not self.contact_check(x, y, z, yaw):
                if self.state == "UNSTICK":
                    self.unstick_step(x, y, z, yaw)
                elif self.state == "HOLD" and self.t >= self.contact_until:
                    self.set_state("REPLAN")
            if self.moves:
                vx, vy = self.last_cmd[:2]
                self.pose = self.raw_pose = (x + (vx * math.cos(yaw) - vy * math.sin(yaw)) * DT,
                                             y + (vx * math.sin(yaw) + vy * math.cos(yaw)) * DT, z, yaw)

        def run_until(self, state, limit=5.0):
            t_end = self.t + limit
            while self.state != state and self.t < t_end:
                self.tick()

    return Stub()


def _along(s, start):
    """떼어 내는 방향으로 start에서 지금까지 옮겨 간 거리."""
    return (s.pose[0] - start[0]) * math.cos(s.unstick["dir"]) + (s.pose[1] - start[1]) * math.sin(s.unstick["dir"])


@unittest.skipIf(mission_node is None, "rclpy가 없어 비행 노드를 불러올 수 없다")
class UnstickTest(unittest.TestCase):
    def test_impact_pushes_away_until_distance(self):
        s = _stub()
        start = s.pose
        x, y, z, yaw = start
        # 13:51의 91.6초 그 값: 왼쪽(뒤)에 닿음 → 앞·오른쪽으로 뗀다
        s.contact_hit = (s.watch.state(s.t, 367.0, 531.0, 3.0, -2.0), s.watch.away(), s.t)
        s.tick()
        self.assertEqual(s.state, "UNSTICK")
        self.assertIsNone(s.door)                       # 문 지나기는 그만둔다 (한 번 센다)
        self.assertEqual(len(s.fails), 1)
        self.assertEqual(s.boxes[0][1], "contact")      # 닿은 쪽에 장애물
        bx, by = s.boxes[0][0]["x"], s.boxes[0][0]["y"]
        toward = math.atan2(by - y, bx - x) - yaw
        self.assertGreater(math.cos(toward - math.radians(125)), 0.95)   # 기체 왼쪽 뒤 (떼는 쪽 −55°의 반대)
        t_push = s.t
        s.run_until("HOLD")
        self.assertEqual(s.state, "HOLD")
        pushes = [c for c in s.sent if abs(c[0]) + abs(c[1]) > 0.01]
        vx, vy = pushes[0][:2]
        self.assertAlmostEqual(math.hypot(vx, vy), mission_node.CONTACT_PUSH_SPEED, places=3)
        self.assertLess(vy, 0.0)                        # 기체 오른쪽으로
        self.assertGreater(vx, 0.0)
        self.assertEqual(pushes[0][3], 0.0)             # 돌지 않는다
        # 시간이 아니라 거리로 끝난다: 0.3 m 떨어지면 바로 (0.3 m/s라 약 1초)
        self.assertGreaterEqual(_along(s, start), mission_node.CONTACT_PUSH_DIST - 1e-6)
        self.assertLess(_along(s, start), mission_node.CONTACT_PUSH_DIST + 0.05)
        self.assertLess(s.t - t_push, mission_node.CONTACT_PUSH_MAX - 0.5)
        self.assertEqual(s.sent[-1], (0.0, 0.0, 0.0, 0.0))
        self.assertIn("떼어 냈습니다", s.log.lines[-1])
        self.assertNotIn("못 미쳤습니다", s.log.lines[-1])
        self.assertEqual(s.contact_pushes, 1)
        s.run_until("REPLAN")
        self.assertEqual(s.state, "REPLAN")

    def test_push_gives_up_at_time_cap(self):
        # 17:44 실기체: 밀어도 추정 위치가 거의 안 옮겨졌다 → CONTACT_PUSH_MAX까지만 밀고 머문다
        s = _stub()
        s.moves = False
        s.start_unstick(*s.pose, "부딪힘", -math.pi / 2)
        t0 = s.t
        s.run_until("HOLD")
        self.assertEqual(s.state, "HOLD")
        self.assertAlmostEqual(s.t - t0, mission_node.CONTACT_PUSH_MAX, delta=0.1)
        self.assertIn("못 미쳤습니다", s.log.lines[-1])

    def test_one_episode_counts_once_and_pushes_farther(self):
        # 17:44 실기체: 4.7초 사이 세 번 닿음 — 한 사건이다. 내리지 않고 0.15 m씩 더 민다. 세 번 떼어 내도 또 닿으면 내린다
        s = _stub()
        s.moves = False
        dists = []
        for k in range(mission_node.CONTACT_PUSHES):
            s.state = "HOLD"
            s.start_unstick(*s.pose, "부딪힘", -math.pi / 2)
            self.assertEqual(s.state, "UNSTICK")
            dists.append(s.unstick["dist"])
            s.t += 2.3
        self.assertEqual(s.contact_count, 1)
        self.assertEqual(len(s.contacts), 1)
        self.assertEqual([round(d, 2) for d in dists], [0.3, 0.45, 0.6])
        self.assertIn("이어서 3번째", s.log.lines[-1])
        s.state = "HOLD"
        s.start_unstick(*s.pose, "부딪힘", -math.pi / 2)
        self.assertEqual(s.state, "LAND")
        self.assertIn("떼어 내도 또 닿아", s.log.lines[-1])

    def test_separate_episodes_in_one_place_land(self):
        s = _stub()
        for k in range(mission_node.CONTACT_LAND_N):
            s.state = "FOLLOW"
            s.start_unstick(*s.pose, "부딪힘", -math.pi / 2)
            s.t += mission_node.CONTACT_EPISODE + 2.0
        self.assertEqual(s.state, "LAND")
        self.assertEqual(s.contact_count, mission_node.CONTACT_LAND_N)
        self.assertIsNotNone(s.landed)

    def test_far_apart_contacts_do_not_land(self):
        s = _stub()
        for k in range(mission_node.CONTACT_LAND_N):
            s.state = "FOLLOW"
            s.pose = (2.2 + 2.0 * k, -0.26, 1.4, 0.0)
            s.start_unstick(*s.pose, "부딪힘", -math.pi / 2)
            s.t += mission_node.CONTACT_EPISODE + 2.0
        self.assertEqual(s.state, "UNSTICK")

    def test_push_shortened_when_map_blocked_on_that_side(self):
        # 떼어 낼 쪽(기체 오른쪽) 0.33 m부터 지도가 막혔으면(0.05 m 걸음으로 0.35 m에서 걸림) 가장자리·여유 0.2 m를 남기고 0.15 m만 민다
        s = _stub()
        x, y, z, yaw = s.pose
        right = yaw - math.pi / 2
        s.believed.blocked = lambda px, py: (px - x) * math.cos(right) + (py - y) * math.sin(right) >= 0.33
        s.start_unstick(x, y, z, yaw, "부딪힘", -math.pi / 2)
        self.assertAlmostEqual(s.unstick["dist"], 0.15, places=2)
        self.assertIn("막혀", s.log.lines[-1])
        s.believed.blocked = lambda px, py: (px - x) * math.cos(right) + (py - y) * math.sin(right) >= 0.12
        self.assertAlmostEqual(s.push_room(x, y, z, right, 0.3), 0.1, places=2)   # 바로 막혀 있어도 0.1 m는 민다

    def test_hit_on_push_side_stops_push(self):
        # 오른쪽으로 떼어 내다 오른쪽에서 부딪힘(반력은 왼쪽) → 더 밀지 않고 그쪽도 막는다. 왼쪽 벽의 튐(반력 오른쪽)은 흘려보낸다
        s = _stub()
        s.door = None
        s.start_unstick(*s.pose, "부딪힘", -math.pi / 2)
        n_boxes = len(s.boxes)
        s.tick()
        s.contact_hit = ("부딪힘 (수평 가속 0.57 g)", -math.pi / 2, s.t)   # 같은 쪽(왼쪽 벽) 튐: 떼어 낼 쪽이 그대로 오른쪽
        s.tick()
        self.assertEqual(s.state, "UNSTICK")
        s.contact_hit = ("부딪힘 (수평 가속 0.60 g)", math.pi / 2, s.t)    # 오른쪽에서 부딪힘: 떼어 낼 쪽이 왼쪽
        s.tick()
        self.assertEqual(s.state, "HOLD")
        self.assertEqual(len(s.boxes), n_boxes + 1)
        bx, by = s.boxes[-1][0]["x"], s.boxes[-1][0]["y"]
        side = math.atan2(by - s.pose[1], bx - s.pose[0]) - s.pose[3]
        self.assertLess(math.sin(side), -0.9)            # 기체 오른쪽에 세움
        self.assertIn("떼어 내는 쪽", s.log.lines[-1])

    def test_unseen_surface_blocked_wide(self):
        # 카메라 지도가 모르는 면(유리·흰 벽)에 정면으로 닿음: 닿은 방향과 직각으로 양옆 0.6 m까지 0.3 m 상자를 줄지어 넣는다
        # (Gazebo 안 보이는 4.5 m 벽: 0.3 m만 막아 상자 사이 틈을 문으로 보고 여섯 번 닿았다)
        s = _stub()
        s.mapping, s.known_here = True, False
        x, y, z, yaw = s.pose
        s.start_unstick(x, y, z, yaw, "부딪힘", math.pi)      # 떼어 낼 쪽은 뒤 → 닿은 쪽은 앞(지도 80°)
        boxes = s.memory.boxes()
        self.assertEqual(len(boxes), 5)
        toward = yaw
        along = [(b["x"] - x) * math.cos(toward) + (b["y"] - y) * math.sin(toward) for b in boxes]
        lat = sorted((b["x"] - x) * -math.sin(toward) + (b["y"] - y) * math.cos(toward) for b in boxes)
        for a in along:   # 상자 앞면이 기체 중심 0.12 m 앞
            self.assertAlmostEqual(a - mission_node.CONTACT_BOX / 2, mission_node.CONTACT_EDGE, places=2)
        self.assertAlmostEqual(lat[0], -mission_node.GLASS_HALF, places=2)
        self.assertAlmostEqual(lat[-1], mission_node.GLASS_HALF, places=2)
        self.assertEqual(s.boxes, [])                    # 카메라 지도에는 안 찍는다
        self.assertIn("양옆", s.log.lines[-2])
        self.assertEqual(s.state, "UNSTICK")

    def test_known_thing_blocked_small(self):
        # 카메라가 아는 것(문틀·가구)에 닿으면 그 자리만 — 옆의 문을 닫지 않게
        s = _stub()
        s.mapping, s.known_here = True, True
        s.start_unstick(*s.pose, "부딪힘", math.pi)
        self.assertEqual(len(s.boxes), 1)
        self.assertEqual(s.memory.boxes(), [])

    def test_blocked_turn_found_in_step(self):
        s = _stub()
        s.state, s.door = "LOOK", None
        s.watch.state(s.t, 40.0, 110.0, 1.0, -4.0)   # 왼쪽으로 기대어 있음 (방향만 쓴다)
        for k in range(30):   # 왼쪽으로 0.5 rad/s 돌라는데 기수가 그대로
            s.t += DT
            s.last_cmd = (0.0, 0.0, 0.0, 0.5)
            if s.contact_check(*s.pose):
                break
        self.assertEqual(s.state, "UNSTICK")
        self.assertIn("안 돎", s.log.lines[-1])
        self.assertLess(math.sin(s.unstick["dir"] - s.pose[3]), -0.9)   # 오른쪽으로

    def test_unknown_side_reuses_recent_contact(self):
        # 앞(지도 80°)에 닿아 떼어 낸 뒤, 같은 자리에서 '돌라는데 안 돎'(쪽 모름) → 그때 닿은 지도 방향의 반대로 다시 뗀다
        s = _stub()
        x, y, z, yaw = s.pose
        s.start_unstick(x, y, z, yaw, "부딪힘", math.pi)
        n_boxes = len(s.boxes)
        s.t += 5.0
        s.state = "FOLLOW"
        s.pose = (x, y - 0.1, z, yaw + 0.5)   # 조금 물러나 기수를 돌린 자리
        s.start_unstick(*s.pose, "돌라는데 1.2초째 기수가 1°밖에 안 돎", None)
        self.assertEqual(s.state, "UNSTICK")
        self.assertAlmostEqual(math.cos(s.unstick["dir"] - (yaw + math.pi)), 1.0, places=6)   # 지도에서 처음 닿은 쪽의 반대
        self.assertIn("전에 닿은 쪽", s.log.lines[-1])
        self.assertEqual(len(s.boxes), n_boxes)                                              # 장애물을 다시 세우지 않음

    def test_blocked_turn_unknown_side_uses_hint(self):
        # 11/14 Gazebo 반력 0 벽 판: 벽에 닿은 채 돌면서(0.45 rad/s) 0.12 m/s로 나아가려는데 기수도 자리도 그대로, 가속·기울기는 작다
        s = _stub()
        s.state, s.door, s.moves = "FOLLOW", None, False
        s.watch.state(s.t, 10.0, -5.0, -1.0, 0.0)
        for k in range(40):
            s.t += DT
            if s.contact_check(*s.pose):
                break
            s.last_cmd = (0.12, 0.0, 0.0, 0.45)
        self.assertEqual(s.state, "UNSTICK")
        self.assertIn("안 돎", s.log.lines[-1])
        self.assertIn("가라 했는데 안 간 쪽", s.log.lines[-1])
        self.assertAlmostEqual(math.cos(s.unstick["dir"] - (s.pose[3] + math.pi)), 1.0, places=6)   # 앞에 닿음 → 뒤로
        self.assertEqual(len(s.boxes), 1)                                                            # 앞에 장애물

    def test_unknown_side_only_holds(self):
        s = _stub()
        s.state = "BACKOFF"
        s.start_unstick(*s.pose, "물러나는 중 기수가 저절로 20° 돎", None)
        self.assertEqual(s.state, "HOLD")
        self.assertEqual(s.boxes, [])
        self.assertIn("떼어 내지는 않음", s.log.lines[-1])

    def _press(self, s, react, limit=6.0):
        """문 지나기 속도로 앞으로 가라는데 제자리 (Gazebo 11/14 안 보이는 벽). 걸리면 그때까지 걸린 시간."""
        s.moves = False
        s.watch.state(s.t, *react)
        t0 = s.t
        while s.t - t0 < limit:
            s.t += DT
            if s.contact_check(*s.pose):
                return s.t - t0
            s.last_cmd = (0.15, 0.0, 0.0, 0.0)
        return None

    def test_stall_found_in_step(self):
        s = _stub()
        took = self._press(s, (-100.0, 0.0, -2.0, 0.0))   # 반력 뒤쪽 0.1 g (앞에 닿아 밀고 있음, 0.6초 이어 밀림 규칙엔 못 미침)
        self.assertIsNotNone(took)
        self.assertLess(took, 2.6)
        self.assertEqual(s.state, "UNSTICK")
        self.assertEqual(s.stall_count, 1)
        self.assertIsNone(s.door)                                                   # 문 지나기는 그만둔다
        self.assertAlmostEqual(math.cos(s.unstick["dir"] - (s.pose[3] + math.pi)), 1.0, places=6)   # 가라 한 쪽(앞)의 반대로
        self.assertIn("가라는데 안 감", s.log.lines[-1])
        self.assertIn("닿은 쪽 기체 앞", s.log.lines[-1])
        bx, by = s.boxes[0][0]["x"], s.boxes[0][0]["y"]
        self.assertGreater(math.cos(math.atan2(by - s.pose[1], bx - s.pose[0]) - s.pose[3]), 0.95)   # 앞에 장애물

    def test_stall_without_reaction_takes_longer(self):
        s = _stub()
        took = self._press(s, (-10.0, 5.0, -1.0, 0.0))
        self.assertIsNotNone(took)
        self.assertGreater(took, 4.3)
        self.assertEqual(s.state, "UNSTICK")

    def test_ground_and_settle_are_ignored(self):
        s = _stub()
        s.tof_down = 0.10   # 바닥에 내려앉음 (10/30 기록)
        s.contact_hit = ("부딪힘", 0.0, s.t)
        self.assertFalse(s.contact_check(*s.pose))
        s.tof_down, s.contact_armed = 1.4, s.t + 0.2   # 떼어 낸 직후
        s.contact_hit = ("부딪힘", 0.0, s.t)
        self.assertFalse(s.contact_check(*s.pose))
        s.t += 0.3
        s.contact_hit = ("부딪힘", 0.0, s.t - 1.0)   # 오래된 것은 버린다
        self.assertFalse(s.contact_check(*s.pose))


if __name__ == "__main__":
    unittest.main()
