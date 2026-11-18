"""문 앞 사람 (11/12 17:19 실기체): 곧게 지나는 길에 닿는 사람만 세고, 그 자리가 화면 밖이면 돌아보고 떠났는지 본다.
비행 노드 메서드를 빌린 가짜 노드로 본다 (ROS가 없으면 건너뜀). 순수 계산은 test_doorway.PassPeopleTest."""
import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from autonomy.doorway import Doorway  # noqa: E402
from autonomy.explore import PeopleLog  # noqa: E402

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

    warn = info


def _stub():
    M = mission_node.Mission

    class Stub:
        door_people, door_unseen, door_look_step, sensor_origin = M.door_people, M.door_unseen, M.door_look_step, M.sensor_origin

        def __init__(self):
            self.people, self.hfov, self.t, self.log, self.sent = PeopleLog(merge=1.0), 0.955, 100.0, _Log(), []
            self.front, self.slow_turn_until = math.inf, -math.inf
            self.params = type("P", (), {"yaw_rate_max": 0.8})()
            # 문 가운데 (3, 0), x로 지나는 폭 0.9 m 문. 드론은 문 앞 0.8 m (2.2, 0)에서 정면을 본다
            self.d = Doorway(np.array([3.0, 0.0]), np.array([1.0, 0.0]), 0.9)
            self.door = {"d": self.d, "phase": "turn", "pre": 0.8, "t0": 90.0}

        def now(self):
            return self.t

        def get_logger(self):
            return self.log

        def send(self, vx=0.0, vy=0.0, vz=0.0, rate=0.0):
            self.sent.append(rate)

        def brake_before_turn(self, vz):
            return False

    return Stub()


@unittest.skipIf(mission_node is None, "rclpy가 없어 비행 노드를 불러올 수 없다")
class DoorPeopleTest(unittest.TestCase):
    def test_only_people_on_the_pass_count(self):
        s = _stub()
        on = s.people.add(3.5, 0.55, 0.9, 1.0)       # 너머 가운데 선 옆 0.38 m(몸 가장자리): 막는다
        off = s.people.add(3.0, -1.3, 0.9, 1.0)      # 문틀 뒤 옆 1.3 m: 막지 않는다 (사람끼리 1 m 넘게 떨어져야 따로 남는다)
        self.assertIsNotNone(off)
        self.assertEqual(s.door_people(s.d, 0.8), [on])

    def test_unseen_person_is_picked_once(self):
        s = _stub()
        side = s.people.add(3.5, 0.55, 0.9, 1.0)     # 정면에서 약 +23°: '떠났나' 확인 범위(±16°) 밖
        ahead = s.people.add(4.2, -0.4, 0.9, 1.0)    # 너머 정면 약 −12°: 범위 안 — 그대로 기다리며 본다
        self.assertIsNotNone(ahead)
        people = s.door_people(s.d, 0.8)
        self.assertEqual(set(map(id, people)), {id(side), id(ahead)})
        pose = (2.2, 0.0, 1.0, 0.0)
        self.assertIs(s.door_unseen(people, pose, set()), side)
        self.assertIsNone(s.door_unseen(people, pose, {id(side)}))    # 한 번 돌아본 사람은 다시 안 본다
        # 이미 그쪽을 보고 있으면 돌 일 없다 (그때는 다른 사람이 화면 밖이 된다)
        self.assertIsNone(s.door_unseen(people, (2.2, 0.0, 1.0, math.radians(23)), {id(ahead)}))

    def test_look_turns_toward_the_person_then_back(self):
        s = _stub()
        p = s.people.add(3.5, 0.55, 0.9, 1.0)
        s.door.update(phase="look", look=p, look_t=None, look_start=100.0)
        s.door_look_step(2.2, 0.0, 0.0, 0.0)
        self.assertGreater(s.sent[-1], 0.0)                         # 왼쪽(+)으로 돈다
        yaw = math.atan2(0.55, 1.3)
        s.t = 101.0
        s.door_look_step(2.2, 0.0, yaw, 0.0)                        # 그쪽을 봄: 지금부터 DOOR_LOOK초
        self.assertEqual(s.door["phase"], "look")
        s.t = 101.0 + mission_node.DOOR_LOOK
        s.door_look_step(2.2, 0.0, yaw, 0.0)
        self.assertEqual(s.door["phase"], "turn")
        self.assertIsNone(s.door["aligned"])
        self.assertAlmostEqual(s.door["t0"], 90.0 + 1.0 + mission_node.DOOR_LOOK)   # 돌아본 시간은 문 시간 한도에 넣지 않는다
        self.assertIn("지울 수 없어", s.log.lines[-1])

    def test_look_ends_as_soon_as_the_person_is_gone(self):
        s = _stub()
        p = s.people.add(3.5, 0.55, 0.9, 1.0)
        s.door.update(phase="look", look=p, look_t=100.5, look_start=100.0)
        p.gone = True                                               # 탐지 콜백이 '지나간 사람'으로 지웠다
        s.t = 100.8
        s.door_look_step(2.2, 0.0, math.atan2(0.55, 1.3), 0.0)
        self.assertEqual(s.door["phase"], "turn")
        self.assertEqual(s.log.lines, [])                           # 지운 것은 탐지 콜백이 이미 알렸다
        self.assertEqual(s.door_people(s.d, 0.8), [])               # 이제 문을 막는 사람이 없다


if __name__ == "__main__":
    unittest.main()
