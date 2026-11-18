"""닿음 감지와 떼어 낼 방향 (autonomy/contact.py, 11/13 13:51 실기체). 값은 그 비행과 정상 비행 기록에서 땄다."""
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from autonomy.contact import ContactWatch, PushWatch, react_world, side_name  # noqa: E402


def _deg(a):
    return math.degrees(a)


class StateTest(unittest.TestCase):
    def test_normal_flight_is_quiet(self):
        # 정상 비행: 수평 가속 93 mg 안팎까지, 기울기 3°까지 (rc 80으로 앞으로 갈 때)
        w = ContactWatch()
        for i in range(200):
            t = i * 0.1
            self.assertIsNone(w.state(t, 70.0 * math.sin(i), 60.0 * math.cos(i), -3.0, 1.0))

    def test_impact_and_away(self):
        # 91.6초: 문 앞에서 돌다 왼쪽 벽에 부딪힘 (agx 367, agy 531 mg) → 반력은 앞·오른쪽
        w = ContactWatch()
        why = w.state(0.0, 367.0, 531.0, 3.0, -2.0)
        self.assertIn("부딪힘", why)
        away = w.away()
        self.assertLess(away, 0.0)                        # 오른쪽으로 뗀다
        self.assertAlmostEqual(_deg(away), -55.3, delta=0.5)
        self.assertEqual(side_name(away + math.pi), "왼쪽 뒤")

    def test_press_needs_time(self):
        # 벽에 기대어 밀 때: 0.12 g 넘는 값이 0.6초 이어져야 한다 (한두 번은 정상 흔들림일 수 있다)
        w = ContactWatch()
        self.assertIsNone(w.state(0.0, 40.0, 160.0, 1.0, -5.0))
        self.assertIsNone(w.state(0.3, 40.0, 150.0, 1.0, -5.0))
        self.assertIsNone(w.state(0.4, 30.0, 60.0, 1.0, -5.0))     # 끊김: 처음부터 다시
        for k in range(6):
            self.assertIsNone(w.state(0.5 + 0.1 * k, 40.0, 200.0, 1.0, -8.0))
        self.assertIn("기대어", w.state(1.1, 40.0, 210.0, 1.0, -9.0))

    def test_tilt(self):
        w = ContactWatch()
        self.assertIsNone(w.state(0.0, 0.0, 0.0, 0.0, -14.0))
        self.assertIn("기울기", w.state(0.1, 0.0, 0.0, 0.0, -16.0))


class AwayTest(unittest.TestCase):
    def test_from_lean_when_accel_small(self):
        # 가속이 작으면 기운 쪽(미는 쪽)의 반대: 롤 −(왼쪽으로 밂) → 오른쪽으로, 피치 +(뒤로 밂) → 앞으로
        w = ContactWatch()
        w.state(0.0, 10.0, 20.0, 0.0, -10.0)
        self.assertAlmostEqual(_deg(w.away()), -90.0, delta=0.1)
        w.state(0.1, 10.0, 20.0, 5.0, 0.0)
        self.assertAlmostEqual(_deg(w.away()), 0.0, delta=0.1)
        w.state(0.2, 10.0, 20.0, 1.0, 1.0)
        self.assertIsNone(w.away())                      # 둘 다 작으면 모른다

    def test_side_names(self):
        self.assertEqual(side_name(0.0), "앞")
        self.assertEqual(side_name(math.pi / 2), "왼쪽")
        self.assertEqual(side_name(-math.pi / 2), "오른쪽")
        self.assertEqual(side_name(math.pi), "뒤")
        self.assertEqual(side_name(math.radians(-140)), "오른쪽 뒤")


class TurnTest(unittest.TestCase):
    def test_blocked_turn(self):
        # 92.9–98.6초: 왼쪽으로 0.3–0.8 rad/s 돌라는데 기수가 그대로 → 1.2초 뒤 잡는다
        w = ContactWatch()
        yaw = math.radians(80)
        hits = [w.turning(0.05 * k, 0.5, yaw) for k in range(30)]
        first = next(k for k, h in enumerate(hits) if h)
        self.assertAlmostEqual(first * 0.05, 1.2, delta=0.06)
        self.assertIn("안 돎", hits[first])

    def test_turning_well_is_quiet(self):
        w = ContactWatch()
        for k in range(100):   # 0.3 rad/s 명령에 0.25 rad/s로 돈다 (시작 0.2초 늦음)
            t = 0.05 * k
            self.assertIsNone(w.turning(t, 0.3, 0.25 * max(0.0, t - 0.2)))

    def test_small_or_changing_command_resets(self):
        w = ContactWatch()
        for k in range(40):   # 작은 회전(제자리 맞추기)은 보지 않는다
            self.assertIsNone(w.turning(0.05 * k, 0.2, 0.0))
        for k in range(40):   # 도는 쪽이 0.5초마다 바뀌면 다시 잰다
            self.assertIsNone(w.turning(2.0 + 0.05 * k, 0.5 if (k // 10) % 2 else -0.5, 0.0))

    def test_reset(self):
        w = ContactWatch()
        for k in range(20):
            w.turning(0.05 * k, 0.5, 0.0)
        w.reset()
        self.assertIsNone(w.turning(1.0, 0.5, 0.0))
        self.assertIsNone(w.turning(1.5, 0.5, 0.0))


def _feed(w, t0, t1, cmd, pos_at, react=(0.0, 0.0), dt=0.05):
    """t0부터 t1까지 dt마다 넣고, 처음 걸린 (시각, 결과)를 돌려준다. cmd·react는 (x, y) 또는 시각을 받는 함수."""
    k = 0
    while True:
        t = t0 + k * dt
        if t > t1 + 1e-9:
            return None
        c = cmd(t) if callable(cmd) else cmd
        r = react(t) if callable(react) else react
        w.add(t, c, pos_at(t), r)
        out = w.check(t)
        if out:
            return t, out
        k += 1


class PushTest(unittest.TestCase):
    """가라는데 안 감 (11/14 Gazebo 안 보이는 벽: 문을 지나다 벽에 10.8초 붙어 있었다). 값은 실기체 기록 56분에서 땄다."""

    def test_pressing_with_reaction_found_fast(self):
        # 문 지나기 속도 0.15 m/s로 앞(지도 90°)에 밀어붙이는데 제자리, 반력은 뒤쪽 0.1 g → 창 2초 + 앞당김 0.4초 뒤에 잡는다
        w = PushWatch()
        hit = _feed(w, 0.0, 6.0, (0.0, 0.15), lambda t: (1.0, 2.0), react=(0.0, -100.0))
        self.assertIsNotNone(hit)
        t, (why, toward) = hit
        self.assertAlmostEqual(t, 2.4, delta=0.11)
        self.assertAlmostEqual(math.degrees(toward), 90.0, delta=0.1)
        self.assertIn("가라는데 안 감", why)
        self.assertIn("밀림", why)

    def test_pressing_intermittently(self):
        # Gazebo 그 판처럼 앞으로 가기와 제자리 돌기를 0.5초씩 번갈아 해도 (앞으로는 반만) 잡는다
        w = PushWatch()
        hit = _feed(w, 0.0, 8.0, lambda t: (0.15, 0.0) if int(t / 0.5) % 2 == 0 else (0.0, 0.0), lambda t: (0.0, 0.0),
                    react=lambda t: (-150.0, 0.0) if int(t / 0.5) % 2 == 0 else (-20.0, 0.0))
        self.assertIsNotNone(hit)
        self.assertLess(hit[0], 3.6)                      # 3초 창 + 앞당김 0.4초
        self.assertIn("3초", hit[1][0])

    def test_stuck_without_reaction_needs_longer(self):
        # 반력이 없으면 2초 규칙은 안 걸리고, 4초 동안 0.5 m 넘게 가라 했는데 10%도 못 갔을 때 잡는다
        w = PushWatch()
        hit = _feed(w, 0.0, 8.0, (0.2, 0.0), lambda t: (0.0, 0.0))
        self.assertIsNotNone(hit)
        self.assertAlmostEqual(hit[0], 4.4, delta=0.11)
        self.assertNotIn("밀림", hit[1][0])

    def test_tello_start_lag_is_quiet(self):
        # 실기체 11/03 16:12·11/05 13:04: 돈 직후 앞으로 0.2–0.25 m/s를 보내도 2초 넘게 안 움직이다 간다 (반력은 정상 흔들림)
        w = PushWatch()
        pos = lambda t: (max(0.0, t - 2.4) * 0.2, 0.0)   # noqa: E731
        self.assertIsNone(_feed(w, 0.0, 12.0, (0.2, 0.0), pos, react=lambda t: (-25.0 * math.cos(7 * t), 20.0 * math.sin(5 * t))))

    def test_slow_progress_is_quiet(self):
        # 가라 한 것의 15%씩은 간다 (느린 바닥 인식·작은 rc): 반력이 없으면 닿음이 아니다
        w = PushWatch()
        self.assertIsNone(_feed(w, 0.0, 12.0, (0.2, 0.0), lambda t: (0.03 * t, 0.0)))

    def test_normal_flight_is_quiet(self):
        w = PushWatch()
        self.assertIsNone(_feed(w, 0.0, 12.0, (0.25, 0.0), lambda t: (max(0.0, t - 0.4) * 0.25, 0.0), react=(-40.0, 30.0)))

    def test_brake_restarts_window(self):
        # 앞으로 0.25 m/s로 미끄러지는 동안 뒤로 0.4 m/s 명령(제동)을 1초: 가라는 쪽과 반대로 움직이지만 닿음이 아니다.
        # 제동이 끝나면 처음부터 다시 잰다
        w = PushWatch()
        pos = lambda t: (0.25 * min(t, 1.0), 0.0)   # noqa: E731  1초까지 앞으로 미끄러지고 그 뒤 멈춤
        self.assertIsNone(_feed(w, 0.0, 1.0, (-0.4, 0.0), pos, react=(0.0, 0.0)))
        self.assertTrue(w.hist and w.hist[0][0] > 0.5)    # 제동 중에는 계속 비운다
        # 멈춘 뒤 뒤로 0.15 m/s 명령에 안 움직인다(반력 없음): 4초 규칙은 제동이 끝난 1초부터 센다
        hit = _feed(w, 1.05, 8.0, (-0.15, 0.0), lambda t: (0.25, 0.0))
        self.assertIsNotNone(hit)
        self.assertGreater(hit[0], 1.0 + 4.4 - 0.11)

    def test_zigzag_and_turning_are_quiet(self):
        w = PushWatch()
        # 지그재그 (모은 명령이 한쪽으로 안 모임)
        self.assertIsNone(_feed(w, 0.0, 8.0, lambda t: (0.0, 0.2) if int(t) % 2 else (0.0, -0.2), lambda t: (0.0, 0.0),
                                react=(0.0, 0.0)))
        w.reset()
        # 제자리 회전 (수평 명령 0)
        self.assertIsNone(_feed(w, 0.0, 8.0, (0.0, 0.0), lambda t: (0.0, 0.0), react=(-200.0, 0.0)))

    def test_gap_restarts(self):
        w = PushWatch()
        _feed(w, 0.0, 3.0, (0.2, 0.0), lambda t: (0.0, 0.0))
        # 1초 동안 안 넣었다(다른 단계) → 이어 보지 않는다: 3.0초 전 것까지 합쳤다면 4.4초에 걸렸을 것
        self.assertIsNone(_feed(w, 4.0, 6.0, (0.2, 0.0), lambda t: (0.0, 0.0)))

    def test_hint_after_hold(self):
        # 11/14 Gazebo 반력 0 벽 판: 머문 뒤(명령 0) 다시 돌면서 앞(지도 90°)으로 0.12 m/s 나아가려는데 제자리 → 그쪽이 닿은 쪽
        w = PushWatch()
        _feed(w, 0.0, 1.0, (0.0, 0.0), lambda t: (2.3, 2.38))
        _feed(w, 1.05, 2.2, (0.0, 0.12), lambda t: (2.3, 2.38))
        self.assertIsNone(w.check(2.2))                       # 판정에는 아직 안 걸린다 (창이 덜 참·반력 없음)
        self.assertAlmostEqual(math.degrees(w.hint(2.2)), 90.0, delta=0.1)

    def test_no_hint_when_moving_or_idle(self):
        w = PushWatch()
        _feed(w, 0.0, 2.0, (0.0, 0.12), lambda t: (0.0, 0.12 * t))   # 가라는 대로 간다
        self.assertIsNone(w.hint(2.0))
        w.reset()
        _feed(w, 0.0, 2.0, (0.0, 0.0), lambda t: (0.0, 0.0))         # 제자리에서 돌기만
        self.assertIsNone(w.hint(2.0))
        w.reset()
        _feed(w, 0.0, 2.0, (0.0, 0.03), lambda t: (0.0, 0.0))        # 너무 조금 가라 함 (0.05 m)
        self.assertIsNone(w.hint(2.0))
        self.assertIsNone(PushWatch().hint(1.0))

    def test_react_world(self):
        # 앞에 닿아 반력이 뒤(agx −)일 때 기수 90°면 지도 −y 쪽, 오른쪽에 닿아(agy −, 반력 왼쪽) 기수 0이면 지도 +y 쪽
        rx, ry = react_world((-100.0, 0.0, 0.0, 0.0), math.pi / 2)
        self.assertAlmostEqual(rx, 0.0, places=6)
        self.assertAlmostEqual(ry, -100.0, places=6)
        rx, ry = react_world((0.0, -80.0, 0.0, 0.0), 0.0)
        self.assertAlmostEqual(ry, 80.0, places=6)
        self.assertEqual(react_world(None, 1.0), (0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
