"""이륙 전 기수 값 확인 (11/16 10:18 실기체: 땅에서 Tello yaw 값이 흐르는 동안 takeoff를 두 번 거절당했다).
지난 실기체 이륙 요청 52번: 받아들인 29번은 직전 10초 폭 0°, 거절한 23번은 모두 1° 이상(22번이 2° 이상).
판정은 autonomy/tello_state.py yaw_span. 비행 노드는 흐르는 동안 시작을 받지 않고(start_blocked), Tello가 거절하면 저절로 다시
보내지 않는다(takeoff_refused) — 그 비행은 20초 뒤 다시 보낸 순간에 운용자가 기체를 들고 있었다 (ROS가 없으면 그 부분은 건너뜀)."""
import sys
import unittest
from collections import deque
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from autonomy.tello_state import YAW_SETTLE_DEG, yaw_span  # noqa: E402

try:   # 노드를 띄우지 않고 메서드만 빌려 쓴다
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "nodes"))
    import mission_node
except ImportError:   # pragma: no cover
    mission_node = None


def _samples(t0, t1, yaw_at, dt=0.1):
    """t0–t1 사이 dt마다 (시각, 1° 단위로 자른 기수 값) — Tello 상태값처럼."""
    n = int(round((t1 - t0) / dt))
    return [(t0 + k * dt, float(int(yaw_at(t0 + k * dt)))) for k in range(n + 1)]


class YawSpanTest(unittest.TestCase):
    def test_steady_is_zero(self):
        self.assertEqual(yaw_span(_samples(0.0, 30.0, lambda t: 29.0), 30.0), 0.0)

    def test_drift_on_the_ground(self):
        # 11/16: 연결 25초 뒤부터 초당 1°쯤 (25초 +1° → 31초 +7°). 거절 가운데 느린 축은 초당 0.2–0.35°
        fast = yaw_span(_samples(0.0, 31.0, lambda t: max(0.0, t - 24.0)), 31.0)
        self.assertGreaterEqual(fast, 6.0)
        slow = yaw_span(_samples(0.0, 31.0, lambda t: 0.25 * t), 31.0)
        self.assertGreaterEqual(slow, YAW_SETTLE_DEG)

    def test_one_degree_flicker_is_not_drift(self):
        # 상태값은 1° 단위라 반올림 경계에서 깜빡일 수 있다 (Gazebo lab_f1 기록: 땅에서 폭 1°). 흐름으로 보지 않는다
        s = [(0.1 * k, 90.0 + (k // 7) % 2) for k in range(300)]
        self.assertEqual(yaw_span(s, 29.9), 1.0)
        self.assertLess(yaw_span(s, 29.9), YAW_SETTLE_DEG)

    def test_needs_some_history(self):
        self.assertIsNone(yaw_span([], 5.0))
        self.assertIsNone(yaw_span(_samples(3.0, 5.0, lambda t: 0.0), 5.0))        # 2초뿐
        self.assertEqual(yaw_span(_samples(1.5, 5.0, lambda t: 0.0), 5.0), 0.0)    # 3.5초면 판단한다 (11/03 14:32는 연결 3.8초 뒤 이륙)

    def test_only_recent_window_counts(self):
        # 20초 전에 기체를 돌려 놓았고 그 뒤로는 그대로
        s = _samples(0.0, 30.0, lambda t: 50.0 if t < 10.0 else 6.0)
        self.assertEqual(yaw_span(s, 30.0), 0.0)
        self.assertGreaterEqual(yaw_span(s, 15.0), 40.0)

    def test_wraps_at_180(self):
        s = [(0.0, 178.0), (1.0, 179.0), (2.0, 180.0), (3.0, -179.0), (4.0, -178.0)]
        self.assertAlmostEqual(yaw_span(s, 4.0), 4.0)


class _Log:
    def __init__(self):
        self.lines = []

    def info(self, text):
        self.lines.append(text)

    warn = error = info


def _refusal(sent_at):
    """드라이버 상태의 last_result: Tello가 takeoff에 error로 답함."""
    return {"cmd": "takeoff", "source": "autonomy", "ok": False, "flying": False,
            "result": {"cmd": "takeoff", "sent_at": sent_at, "response": "error", "ok": False, "reason": "error_response"}}


def _stub():
    M = mission_node.Mission

    class Stub:
        start_blocked, takeoff_result, takeoff_refused = M.start_blocked, M.takeoff_result, M.takeoff_refused

        def __init__(self):
            self.t, self.log, self.states = 40.0, _Log(), []
            self.ground_yaw: deque = deque()
            self.start_refused_t, self.takeoff_seen, self.takeoff_holds, self.takeoff_refusals = None, None, 0, 0
            self.flight_info, self.flight_req = {}, None

        def now(self):
            return self.t

        def get_logger(self):
            return self.log

        def set_state(self, s):
            self.states.append(s)

    return Stub()


@unittest.skipIf(mission_node is None, "rclpy가 없어 비행 노드를 불러올 수 없다")
class StartGateTest(unittest.TestCase):
    def test_steady_yaw_starts(self):
        s = _stub()
        s.ground_yaw.extend(_samples(10.0, 40.0, lambda t: 0.0))
        self.assertFalse(s.start_blocked())
        self.assertEqual((s.takeoff_holds, s.log.lines), (0, []))

    def test_no_state_yet_starts(self):
        # 상태값이 덜 쌓였으면 모른다: 전처럼 보낸다 (Gazebo는 위치가 잡히자마자 이륙한다)
        self.assertFalse(_stub().start_blocked())

    def test_drifting_yaw_refuses_start_and_tells_why(self):
        s = _stub()
        s.ground_yaw.extend(_samples(10.0, 40.0, lambda t: max(0.0, t - 33.0)))   # 11/16처럼 초당 1°
        self.assertTrue(s.start_blocked())
        self.assertEqual(s.takeoff_holds, 1)
        self.assertIn("흐르고 있어", s.log.lines[-1])
        self.assertIn("들었다", s.log.lines[-1])
        self.assertIn("한 번 더", s.log.lines[-1])

    def test_pressing_again_soon_goes_anyway(self):
        # 이 관계는 기록에서 본 것일 뿐이라 운용자가 넘길 수 있다. 한참 뒤(TAKEOFF_AGAIN 넘어)에 누른 것은 처음처럼 다시 막는다
        s = _stub()
        s.ground_yaw.extend(_samples(10.0, 100.0, lambda t: t - 10.0))
        self.assertTrue(s.start_blocked())
        s.t += 5.0
        self.assertFalse(s.start_blocked())
        self.assertIn("요청해 봅니다", s.log.lines[-1])
        s.t += 1.0
        self.assertTrue(s.start_blocked())                 # 넘긴 뒤 다시 누르면 새로 센다
        s.t += mission_node.TAKEOFF_AGAIN + 5.0
        self.assertTrue(s.start_blocked())
        self.assertEqual(s.takeoff_holds, 3)

    def test_stops_drifting_then_starts_without_pressing_twice(self):
        s = _stub()
        s.ground_yaw.extend(_samples(10.0, 40.0, lambda t: max(0.0, t - 33.0)))
        self.assertTrue(s.start_blocked())
        s.ground_yaw.extend(_samples(40.1, 55.0, lambda t: 7.0))   # 들었다 놓아 멈춤
        s.t = 55.0
        self.assertFalse(s.start_blocked())
        self.assertNotIn("요청해 봅니다", s.log.lines[-1])


@unittest.skipIf(mission_node is None, "rclpy가 없어 비행 노드를 불러올 수 없다")
class RefusalTest(unittest.TestCase):
    def test_refusal_goes_back_to_ready_without_retry(self):
        # 11/16: takeoff → 14 ms 만에 error. 전에는 20초 뒤 저절로 다시 보냈다 (그 순간 기체가 손에 들려 있었다)
        s = _stub()
        s.ground_yaw.extend(_samples(10.0, 40.0, lambda t: max(0.0, t - 33.0)))
        s.takeoff_seen = s.takeoff_result().get("sent_at")          # 시작할 때: 옛 결과 없음
        s.flight_req = ("takeoff", s.t, 1)
        self.assertFalse(s.takeoff_refused())                       # 아직 결과가 안 왔다
        s.flight_info = {"flying": False, "last_result": _refusal("2026-11-16T01:19:39.987+00:00")}
        self.assertTrue(s.takeoff_refused())
        self.assertEqual(s.states, ["READY"])
        self.assertIsNone(s.flight_req)                             # 다시 보낼 요청이 남지 않는다
        self.assertEqual(s.takeoff_refusals, 1)
        self.assertIn("거절했습니다", s.log.lines[-1])
        self.assertIn("저절로 다시 보내지 않습니다", s.log.lines[-1])
        self.assertIn("흐르는 중", s.log.lines[-1])

    def test_old_refusal_is_not_counted_for_the_next_start(self):
        # 시작 대기로 돌아온 뒤 다시 시작: 드라이버 상태에는 지난 거절이 그대로 남아 있다. 새 결과가 올 때까지는 거절로 보지 않는다
        s = _stub()
        s.flight_info = {"flying": False, "last_result": _refusal("2026-11-16T01:19:39.987+00:00")}
        s.takeoff_seen = s.takeoff_result().get("sent_at")          # 다시 시작할 때 본 것
        s.flight_req = ("takeoff", s.t, 1)
        self.assertFalse(s.takeoff_refused())
        s.flight_info = {"flying": False, "last_result": _refusal("2026-11-16T01:21:10.000+00:00")}
        self.assertTrue(s.takeoff_refused())

    def test_other_failures_keep_the_old_retry(self):
        s = _stub()
        s.flight_req = ("takeoff", s.t, 1)
        s.flight_info = {"last_result": {"cmd": "takeoff", "ok": False, "reason": "preflight_failed", "failed": ["operator"]}}
        self.assertFalse(s.takeoff_refused())                       # 이륙 전 점검 실패
        s.flight_info = {"last_result": {"cmd": "takeoff", "ok": False, "flying": False,
                                         "result": {"sent_at": "x", "response": None, "reason": "timeout"}}}
        self.assertFalse(s.takeoff_refused())                       # 응답 없음
        s.flight_info = {"last_result": {"cmd": "land", "ok": False, "result": {"sent_at": "y", "reason": "error_response"}}}
        self.assertFalse(s.takeoff_refused())                       # 착륙 결과
        self.assertEqual(s.states, [])


if __name__ == "__main__":
    unittest.main()
