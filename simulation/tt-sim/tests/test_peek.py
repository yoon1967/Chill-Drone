"""옆 겨눠 보기 (11/13 17:44 실기체: 지도에는 빈 곳이던 자리에서 흰 기둥 모서리에 왼쪽 앞이 부딪혔다. 깊이는 그쪽을 0.7 m로
읽었지만 단안 깊이는 1.3 m 안의 흰 면을 다 그렇게 읽는다 → 나아가기 전에 그쪽으로 기수를 돌려 정면 ToF로 잰다).
비행 노드 메서드를 빌린 가짜 노드로 본다 (ROS가 없으면 건너뜀). 볼 쪽 고르기는 test_near.SideNearTest."""
import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

try:   # 노드를 띄우지 않고 메서드만 빌려 쓴다
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "nodes"))
    import mission_node
except ImportError:   # pragma: no cover
    mission_node = None

HFOV, DT = 0.955, 0.05
W = HFOV / 27
CENTERS = np.array([-HFOV / 2 + W * (k + 0.5) for k in range(27)])   # 오른쪽 끝 → 왼쪽 끝


def _ranges(left=None, right=None, far=3.6):
    """열 거리(배율 곱한 값): 왼쪽 7열·오른쪽 7열을 주어진 값으로."""
    r = np.full(27, far)
    if left is not None:
        r[-7:] = left
    if right is not None:
        r[:7] = right
    return r


class _Log:
    def __init__(self):
        self.lines = []

    def info(self, text):
        self.lines.append(text)

    warn = error = info


def _stub():
    M = mission_node.Mission

    class Stub:
        peek_check, peek_known, peek_step = M.peek_check, M.peek_known, M.peek_step

        def __init__(self):
            self.t, self.log, self.sent, self.states, self.hard = 50.0, _Log(), [], [], []
            self.pose = (5.8, -0.8, 1.0, 0.0)
            self.scene, self.scene_scale = None, 1.18
            self.peek, self.peek_memo, self.peek_count, self.peek_found = None, [], 0, 0
            self.progress, self.front, self.slow_turn_until = (0, 0, 0, 0), math.inf, -math.inf
            self.blocked = lambda x, y: False
            self.believed = type("B", (), {"occupied_at": lambda _, p: bool(self.blocked(p[0], p[1]))})()
            self.params = type("P", (), {"vz_max": 0.5, "k_pos": 1.0})()
            self.cmap = type("C", (), {"p": type("Q", (), {"res": 0.1})(), "mark_hard": lambda _, pts, w: self.hard.append(np.asarray(pts))})()
            self.maps_built = 0

        def now(self):
            return self.t

        def get_logger(self):
            return self.log

        def send(self, vx=0.0, vy=0.0, vz=0.0, rate=0.0):
            self.sent.append((vx, vy, vz, rate))

        def set_state(self, s):
            self.states.append(s)

        def sensor_origin(self, pose):
            return (pose[0] + 0.05 * math.cos(pose[3]), pose[1] + 0.05 * math.sin(pose[3]), pose[2])

        def update_camera_map(self, x, y, z):
            self.maps_built += 1

        def see(self, ranges, yaw=None, age=0.1):
            """깊이 장면 하나: age초 전에 받았고 yaw(없으면 지금 기수)로 찍었다."""
            self.scene = (self.t - age, self.pose[3] if yaw is None else yaw, CENTERS, np.asarray(ranges) * self.scene_scale)

        def run_peek(self, limit=6.0):
            """겨눠 보기가 끝날 때까지: 보낸 회전 속도대로 기수를 돌린다."""
            t_end = self.t + limit
            while self.peek is not None and self.t < t_end:
                self.t += DT
                x, y, z, yaw = self.pose
                self.peek_step(x, y, z, yaw)
                self.pose = (x, y, z, yaw + self.sent[-1][3] * DT)

    return Stub()


def _cmd():
    return type("Cmd", (), {"vx": 0.2, "vy": 0.0, "vz": 0.0, "yaw_rate": 0.1})()


@unittest.skipIf(mission_node is None, "rclpy가 없어 비행 노드를 불러올 수 없다")
class PeekTest(unittest.TestCase):
    def test_unknown_near_side_starts_peek_and_finds_corner(self):
        # 17:44의 84.6초: 왼쪽 7열이 0.7 m(배율 전), 지도에는 그쪽에 아무것도 없다 → 서서 왼쪽 약 20°를 겨눈다
        s = _stub()
        s.see(_ranges(left=0.7))
        self.assertTrue(s.peek_check(*s.pose, _cmd()))
        self.assertEqual(s.sent[-1], (0.0, 0.0, 0.0, 0.0))
        self.assertAlmostEqual(math.degrees(s.peek["yaw"]), 20.3, delta=1.5)
        self.assertIn("왼쪽 앞이 카메라에 가깝게", s.log.lines[-1])
        s.front = 0.24            # 겨눠 보니 0.24 m에 모서리
        s.run_peek()
        self.assertIsNone(s.peek)
        self.assertAlmostEqual(math.degrees(s.pose[3]), 20.3, delta=5.0)          # 그쪽으로 돌았다
        self.assertTrue(all(abs(c[3]) <= mission_node.SLOW_TURN_RATE + 1e-9 for c in s.sent))   # 천천히
        self.assertTrue(all(c[0] == 0.0 and c[1] == 0.0 for c in s.sent))         # 그동안 나아가지 않는다
        self.assertEqual((s.peek_count, s.peek_found), (1, 1))
        self.assertEqual(len(s.hard), 1)                                          # ToF가 잰 자리를 단단한 칸으로
        hx, hy = s.hard[0][:, 0].mean(), s.hard[0][:, 1].mean()
        self.assertAlmostEqual(math.hypot(hx - s.pose[0], hy - s.pose[1]), 0.29, delta=0.03)
        self.assertEqual(s.states, ["REPLAN"])
        self.assertIn("0.24 m에 무엇이 있습니다", s.log.lines[-1])

    def test_far_wall_misread_as_near_goes_on(self):
        # 1.2 m 밖의 흰 벽도 0.7로 읽힌다: 겨눠 보니 ToF가 비어 있으면 그대로 가고, 같은 곳·같은 쪽은 다시 보지 않는다
        s = _stub()
        s.see(_ranges(right=0.7))
        self.assertTrue(s.peek_check(*s.pose, _cmd()))
        self.assertLess(s.peek["yaw"], 0.0)
        s.front = math.inf
        s.run_peek()
        self.assertEqual((s.peek_count, s.peek_found, s.states, s.hard), (1, 0, [], []))
        self.assertIn("비어 있습니다", s.log.lines[-1])
        s.pose = (5.8, -0.8, 1.0, 0.0)
        s.see(_ranges(right=0.7))
        self.assertFalse(s.peek_check(*s.pose, _cmd()))        # 방금 본 쪽
        s.pose = (6.6, -0.8, 1.0, 0.0)                      # 0.8 m 더 간 자리에서는 다시 본다
        s.see(_ranges(right=0.7))
        self.assertTrue(s.peek_check(*s.pose, _cmd()))

    def test_side_the_map_already_knows_is_not_peeked(self):
        # 지도에 그쪽 1.3 m 안으로 벽이 있으면(이미 아는 벽 옆을 지나는 중) 겨눠 보지 않는다
        s = _stub()
        x, y, _, yaw = s.pose
        s.blocked = lambda px, py: px - x > 0.8 and py - y > 0.1   # 왼쪽 앞 0.8 m부터 막힌 덩어리 (왼쪽 20° 방향으로 0.85 m쯤)
        s.see(_ranges(left=0.7))
        self.assertFalse(s.peek_check(*s.pose, _cmd()))
        self.assertIsNone(s.peek)
        self.assertEqual(s.sent, [])

    def test_waits_for_scene_from_this_heading(self):
        # 막 28° 돌았는데 장면은 돌기 전 기수로 찍은 것: 나아가지 않고(회전·높이 명령은 그대로) 기다린다 — '돌면서 가기'도 마찬가지
        s = _stub()
        s.see(_ranges(), yaw=math.radians(-28))
        self.assertTrue(s.peek_check(*s.pose, _cmd()))
        self.assertEqual(s.sent[-1], (0.0, 0.0, 0.0, 0.1))
        self.assertIsNone(s.progress)                       # 일부러 선 것: '제자리에 걸림'으로 세지 않는다
        s.see(_ranges(), yaw=math.radians(-13))             # 12° 넘게 어긋나면 아직 안 간다
        self.assertTrue(s.peek_check(*s.pose, _cmd()))
        cmd = _cmd()
        s.see(_ranges(), yaw=math.radians(-9))              # 6–12° 사이는 전진을 줄여서 (딱 끊으면 가다 서다 한다): 9°면 절반
        self.assertFalse(s.peek_check(*s.pose, cmd))
        self.assertAlmostEqual(cmd.vx, 0.1, places=3)
        cmd = _cmd()
        s.see(_ranges(), yaw=math.radians(-5))              # 6° 안이면 그대로
        self.assertFalse(s.peek_check(*s.pose, cmd))
        self.assertAlmostEqual(cmd.vx, 0.2, places=6)
        s.see(_ranges(), age=0.9)                           # 장면이 0.6초 넘게 묵었다: 기다린다
        self.assertTrue(s.peek_check(*s.pose, _cmd()))

    def test_no_depth_means_rule_off(self):
        # 깊이 추정이 1.5초 넘게 안 오면 이 규칙 없이 간다 (정면 ToF는 그대로)
        s = _stub()
        self.assertFalse(s.peek_check(*s.pose, _cmd()))
        s.see(_ranges(left=0.7), age=2.0)
        self.assertFalse(s.peek_check(*s.pose, _cmd()))
        self.assertEqual(s.sent, [])

    def test_peek_gives_up_turning_after_timeout(self):
        # 기수가 안 돌아도(무엇에 걸림 등) PEEK_TIMEOUT 뒤에는 그 자리 값으로 끝낸다
        s = _stub()
        s.see(_ranges(left=0.7))
        s.peek_check(*s.pose, _cmd())
        t0 = s.t
        while s.peek is not None and s.t < t0 + 10:
            s.t += DT
            s.peek_step(*s.pose)                            # 기수를 돌려 주지 않는다
        self.assertIsNone(s.peek)
        self.assertAlmostEqual(s.t - t0, mission_node.PEEK_TIMEOUT + mission_node.PEEK_DWELL, delta=0.2)


if __name__ == "__main__":
    unittest.main()
