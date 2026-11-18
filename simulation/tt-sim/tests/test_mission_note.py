"""지금 하는 일과 이유 (11/16 15:33 실기체: 탐색 시간 120초가 다 돼 출발점으로 돌아와 내렸는데, 웹 화면에는 '경로 비행 → 착륙 → 완료'만
나와 운용자가 왜 내렸는지 몰랐다). 비행 노드는 내리거나 중단하기 바로 앞에 남긴 로그를 이유로 잡아(Said·end_reason) 남은 탐색 시간·
돌아가는 이유와 함께 /tt/mission_note로 내고, 웹 화면이 머리글 밑 띠와 기록에 보인다 (ROS가 없으면 건너뜀)."""
import json
import math
import sys
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

try:   # 노드를 띄우지 않고 메서드만 빌려 쓴다
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "nodes"))
    import mission_node
    import numpy as np
except ImportError:   # pragma: no cover
    mission_node = None

try:   # 웹 화면 서버 (rclpy·cv2가 있어야 불러온다)
    sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "modules" / "autonomy" / "autonomy_web"))
    import autonomy_web
except ImportError:   # pragma: no cover
    autonomy_web = None


class _Log:
    def __init__(self):
        self.lines = []

    def info(self, text):
        self.lines.append(("info", text))

    def warn(self, text):
        self.lines.append(("warn", text))

    def warning(self, text):
        self.lines.append(("warn", text))

    def error(self, text):
        self.lines.append(("error", text))

    def fatal(self, text):
        self.lines.append(("fatal", text))

    def debug(self, text):
        self.lines.append(("debug", text))


class _Pub:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg.data)


def _stub(state="FOLLOW"):
    M = mission_node.Mission

    class Stub:
        set_state, publish_note, end_reason, search_elapsed = M.set_state, M.publish_note, M.end_reason, M.search_elapsed
        land_at_goal, start_landing = M.land_at_goal, M.start_landing

        def __init__(self):
            self.state, self.t, self.peek = state, 300.0, None
            self.log = mission_node.Said(_Log())
            self.state_pub, self.note_pub = _Pub(), _Pub()
            self.search, self.search_start, self.search_time, self.return_home = True, 100.0, 120.0, True
            self.returning, self.why_return, self.why_end = False, None, None
            self.goto_active = self.goto_pending = None
            self.goal, self.hold, self.land_t, self.saved, self.stopped = np.array([0.0, 0.0, 1.0]), None, None, 0, 0

        def now(self):
            return self.t

        def get_logger(self):
            return self.log

        def send(self):
            self.stopped += 1

        def save_discovered(self):
            self.saved += 1

        def note(self):
            return json.loads(self.note_pub.sent[-1])

    return Stub()


@unittest.skipIf(mission_node is None, "rclpy가 없어 비행 노드를 불러올 수 없다")
class SaidTest(unittest.TestCase):
    def test_logs_go_through_and_the_last_one_is_remembered(self):
        log = _Log()
        said = mission_node.Said(log)
        self.assertIsNone(said.last)
        said.info("가")
        said.warn("나")
        said.error("다")
        self.assertEqual(log.lines, [("info", "가"), ("warn", "나"), ("error", "다")])
        self.assertEqual(said.last[0], "다")
        self.assertLess(time.monotonic() - said.last[1], 1.0)
        said.debug("라")                                   # 나머지는 그대로 넘기고 기억하지 않는다
        self.assertEqual((log.lines[-1], said.last[0]), (("debug", "라"), "다"))


@unittest.skipIf(mission_node is None, "rclpy가 없어 비행 노드를 불러올 수 없다")
class EndReasonTest(unittest.TestCase):
    def test_the_log_right_before_landing_is_the_reason(self):
        s = _stub("FOLLOW")
        s.get_logger().info("요청에 따라 그 자리에 착륙합니다.")
        s.start_landing(1.0, 2.0)
        self.assertEqual((s.state, s.hold), ("LAND", (1.0, 2.0)))
        self.assertEqual(s.why_end, "요청에 따라 그 자리에 착륙합니다.")
        self.assertEqual(s.note()["why_end"], s.why_end)    # 단계가 바뀔 때 바로 알린다
        self.assertEqual(s.note()["state"], "LAND")
        s.set_state("DONE")                                 # 내린 뒤에도 이유는 그대로 남는다
        self.assertEqual((s.note()["state"], s.note()["why_end"]), ("DONE", "요청에 따라 그 자리에 착륙합니다."))

    def test_on_the_way_home_the_reason_says_so(self):
        s = _stub("FOLLOW")
        s.returning, s.why_return = True, "탐색 시간(120초)이 다 돼"
        s.get_logger().warn("출발점 1.2 m 앞에서 30초 넘게 더 가까워지지 못해 여기에 내립니다.")
        s.start_landing(1.0, 2.0)
        self.assertEqual(s.why_end, "탐색 시간(120초)이 다 돼 출발점으로 돌아가던 중 — 출발점 1.2 m 앞에서 30초 넘게 더 가까워지지 못해 여기에 내립니다.")

    def test_normal_return_landing_names_why_it_came_back(self):
        # 11/16 15:33: 이 경우였다. 바로 앞 로그가 없으므로(1초 전의 '사람 발견' 따위) land_at_goal이 이유를 정한다
        s = _stub("FOLLOW")
        s.returning, s.why_return = True, "탐색 시간(120초)이 다 돼"
        s.get_logger().info("사람 발견 #8: (-2.59, 0.57), 신뢰도 0.85, 약 2.6 m 앞")
        s.land_at_goal(0.05, 0.02)
        self.assertEqual((s.state, s.why_end), ("LAND", "탐색 시간(120초)이 다 돼 출발점으로 돌아와 내립니다."))

    def test_goal_flight_and_blocked_goal(self):
        s = _stub("FOLLOW")
        s.goal = np.array([3.0, 0.0, 1.0])
        s.land_at_goal(2.95, 0.0)
        self.assertEqual(s.why_end, "목적지에 도착해 내립니다.")
        s = _stub("FOLLOW")
        s.goal = np.array([3.0, 0.0, 1.0])
        s.land_at_goal(3.0 - mission_node.LAND_NEAR - 0.2, 0.0)   # 목적지 칸이 막혀 경로가 앞에서 끝났다
        self.assertIn("둘레가 막혀", s.why_end)
        self.assertAlmostEqual(s.hold[0], 3.0 - mission_node.LAND_NEAR - 0.2)

    def test_abort_takes_a_new_reason_and_stale_logs_are_not_reasons(self):
        s = _stub("LAND")
        s.why_end = "요청에 따라 그 자리에 착륙합니다."
        s.get_logger().error("드라이버가 기체를 내렸습니다 (배터리 10% 이하). 중단합니다.")
        s.set_state("ABORT")
        self.assertEqual(s.why_end, "드라이버가 기체를 내렸습니다 (배터리 10% 이하). 중단합니다.")
        self.assertEqual(s.saved, 1)
        s = _stub("FOLLOW")
        s.get_logger().info("앞 1.8 m가 비어 0.20 m/s로 갑니다")
        s.log.last = (s.log.last[0], time.monotonic() - 5.0)      # 5초 전에 한 말은 이유가 아니다
        s.start_landing(0.0, 0.0)
        self.assertIn("이유를 남기지 않고", s.why_end)


@unittest.skipIf(mission_node is None, "rclpy가 없어 비행 노드를 불러올 수 없다")
class NoteTest(unittest.TestCase):
    def test_search_clock_and_return_reason(self):
        s = _stub("READY")
        s.search_start = None
        s.publish_note()
        self.assertEqual((s.note()["search"], s.note()["returning"], s.note()["why_end"]), (None, False, None))
        s.search_start, s.t, s.state = 100.0, 180.0, "FOLLOW"
        s.publish_note()
        self.assertEqual(s.note()["search"], {"elapsed": 80.0, "limit": 120.0})
        self.assertTrue(s.note()["return_home"])
        s.t, s.returning, s.why_return = 238.0, True, "탐색 시간(120초)이 다 돼"
        s.publish_note()
        n = s.note()
        self.assertEqual((n["returning"], n["why_return"], n["search"]["elapsed"]), (True, "탐색 시간(120초)이 다 돼", 138.0))
        s.returning = False                                 # 찍은 자리로 가느라 돌아가기를 멈췄다: 지난 이유는 내지 않는다
        s.publish_note()
        self.assertIsNone(s.note()["why_return"])
        self.assertTrue(math.isfinite(s.note()["t"]))

    def test_goal_flight_has_no_search_clock(self):
        s = _stub("FOLLOW")
        s.search = False
        s.publish_note()
        self.assertIsNone(s.note()["search"])


@unittest.skipIf(autonomy_web is None, "rclpy·cv2가 없어 웹 화면 서버를 불러올 수 없다")
class WebNoteTest(unittest.TestCase):
    def test_only_changes_of_reason_go_to_the_record(self):
        ev = autonomy_web.note_events
        flying = {"state": "FOLLOW", "returning": False, "why_return": None, "why_end": None}
        back = {"state": "FOLLOW", "returning": True, "why_return": "탐색 시간(120초)이 다 돼", "why_end": None}
        self.assertEqual(ev(None, flying), [])
        self.assertEqual(ev(flying, flying), [])
        out = ev(flying, back)
        self.assertEqual(len(out), 1)
        self.assertIn("탐색 시간(120초)이 다 돼 출발점으로 돌아갑니다", out[0])
        self.assertIn("여기로 보내기", out[0])
        self.assertEqual(ev(back, back), [])                # 1초마다 같은 것이 와도 한 번만 적는다
        land = dict(back, state="LAND", why_end="탐색 시간(120초)이 다 돼 출발점으로 돌아와 내립니다.")
        self.assertEqual(ev(back, land), ["비행 노드: 내리는 이유 — 탐색 시간(120초)이 다 돼 출발점으로 돌아와 내립니다."])
        self.assertEqual(ev(land, dict(land, state="DONE")), [])
        abort = dict(land, state="ABORT", why_end="드라이버가 기체를 내렸습니다 (배터리 10% 이하). 중단합니다.")
        self.assertIn("중단한 이유", ev(land, abort)[0])
        again = dict(flying)                                # 찍은 자리를 다녀온 뒤 다시 돌아갈 때는 다시 적는다
        self.assertEqual(len(ev(again, back)), 1)


if __name__ == "__main__":
    unittest.main()
