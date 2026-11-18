"""가까운 장애물 앞 규칙 검사 (2026-11-02 17:33 유리 사이 추락). 실행: python -m unittest discover -s tests"""
import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from autonomy.near import (EXIT_WIDTH, GAP_MIN_HEIGHT, GAP_MIN_WIDTH, contact, forward_speed, free_width, late_hit, off_tail,  # noqa: E402
                           limit_turn, pocket, rear_unknown, retreat_body, retreat_direction, room_around, sector_clear, side_near,
                           trail_cursor, trail_fold, trail_target, trail_trim, turn_brake, world_to_body)

INF = math.inf


class SideNearTest(unittest.TestCase):
    """옆 겨눠 보기에서 볼 쪽 고르기 (11/13 17:44 실기체의 깊이 장면 값)."""
    HFOV, CONE = 0.955, math.radians(12.5)

    def _centers(self):
        w = self.HFOV / 27
        return [-self.HFOV / 2 + w * (k + 0.5) for k in range(27)]   # 오른쪽 끝 → 왼쪽 끝 (왼쪽 +)

    def test_white_corner_on_the_left(self):
        # 84.57초: 왼쪽 7열이 0.7 m, 나머지는 3.7–4.0 m → 왼쪽 한쪽, 가운데 각은 ToF 원뿔 밖(약 +20°)
        left_to_right = [0.7] * 7 + [3.7, 3.8, 3.7, 3.7, 3.7, 3.7, 3.8, 3.8, 3.8, 3.8, 3.9, 3.9, 3.9, 3.9, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0]
        sides = side_near(self._centers(), left_to_right[::-1], self.CONE, 0.95, 3)
        self.assertEqual(len(sides), 1)
        self.assertAlmostEqual(math.degrees(sides[0]), 20.3, delta=1.5)

    def test_open_scene_and_short_runs(self):
        c = self._centers()
        self.assertEqual(side_near(c, [3.6] * 27, self.CONE, 0.95, 3), [])
        # 84.46초: 왼쪽 끝 두 열만 0.8·0.9 (셋째는 1.1) → 아직 아님
        rng = [3.6] * 27
        rng[-1], rng[-2], rng[-3] = 0.8, 0.9, 1.1
        self.assertEqual(side_near(c, rng, self.CONE, 0.95, 3), [])

    def test_center_only_is_tof_job(self):
        # 정면(ToF 원뿔 안)만 가까우면 ToF가 재는 곳이라 옆으로 치지 않는다. 양쪽이 다 가까우면 둘 다
        c = self._centers()
        rng = [0.7 if abs(a) < self.CONE else 3.5 for a in c]
        self.assertEqual(side_near(c, rng, self.CONE, 0.95, 3), [])
        both = side_near(c, [0.7] * 27, self.CONE, 0.95, 3)
        self.assertEqual(len(both), 2)
        self.assertGreater(both[0], 0.0)
        self.assertLess(both[1], 0.0)


class RearTest(unittest.TestCase):
    def test_rear_must_be_known_free(self):
        # 11/09 13:03 실기체: 이륙 직후 제자리에서 둘러보다(기수 −79°) 앞 0.44 m에 사람이 잡혀 본 적 없는 뒤로 후진하다 떨어졌다.
        # 뒤 띠(중심에서 0.1–0.55 m, 옆 ±0.15 m)가 모두 아는 빈 곳일 때만 물러난다
        self.assertIsNone(rear_unknown(lambda p: True, (0.0, 0.0, 1.0), 0.0, 0.55, 0.15))
        # 앞(+x)만 봤다: 뒤(−x)는 모름 → 바로 뒤 0.1 m에서 걸림
        self.assertAlmostEqual(rear_unknown(lambda p: p[0] > 0.0, (0.0, 0.0, 1.0), 0.0, 0.55, 0.15), 0.1)
        # 기수 −79°: 뒤는 +101° 쪽(+y). 그쪽 0.3 m 너머가 모름 → 0.3 m 근처에서 걸림
        d = rear_unknown(lambda p: p[1] < 0.3, (0.0, 0.0, 1.0), math.radians(-79), 0.55, 0.15)
        self.assertIsNotNone(d)
        self.assertLess(d, 0.35)
        # 옆으로만 막힌 곳(뒤 띠 옆 0.15 m 밖)은 상관없다
        self.assertIsNone(rear_unknown(lambda p: abs(p[1]) < 0.2, (0.0, 0.0, 1.0), 0.0, 0.55, 0.15))


class TurnBrakeTest(unittest.TestCase):
    def test_forward_speed_from_recent_poses(self):
        # 기수 90°(+y)로 0.2 m/s 가는 중: 앞 속도 0.2. 제자리에서 돌기만 하면 0
        h = [(t, (0.0, 0.2 * t, 1.0, math.pi / 2)) for t in [i * 0.05 for i in range(20)]]
        self.assertAlmostEqual(forward_speed(h), 0.2, places=3)
        spin = [(t, (1.0, 1.0, 1.0, 0.5 * t)) for t in [i * 0.05 for i in range(20)]]
        self.assertAlmostEqual(forward_speed(spin), 0.0, places=6)
        back = [(t, (-0.1 * t, 0.0, 1.0, 0.0)) for t in [i * 0.05 for i in range(20)]]
        self.assertLess(forward_speed(back), 0.0)
        self.assertEqual(forward_speed(h[:1]), 0.0)

    def test_brake_only_when_still_sliding_forward(self):
        # 11/05 13:04: 0.2 m/s로 가다 제자리 회전으로 바뀌면 도는 동안 0.10–0.27 m/s로 밀려갔다
        self.assertIsNone(turn_brake(0.03, 0.06, 1.5, 0.12, 0.3), "멈춘 채(0.00)는 그냥 돈다")
        self.assertIsNone(turn_brake(-0.1, 0.06, 1.5, 0.12, 0.3))
        self.assertAlmostEqual(turn_brake(0.07, 0.06, 1.5, 0.12, 0.3), -0.12, msg="rc 무시 구간 위로")
        self.assertAlmostEqual(turn_brake(0.15, 0.06, 1.5, 0.12, 0.3), -0.225)
        self.assertAlmostEqual(turn_brake(0.27, 0.06, 1.5, 0.12, 0.3), -0.3, msg="세게 밀지는 않는다")


class NearTest(unittest.TestCase):
    def test_glass_seen_late_by_tof(self):
        # 추락 기록: 정면 ToF가 비어 있다가 0.18 m에서 처음 잡혔다 → 유리로 본다
        seq = [INF, INF, INF, 0.18, INF, 0.19, 0.17, 0.22]
        hits = [late_hit(p, f, 0.6, 1.2) for p, f in zip(seq, seq[1:])]
        self.assertEqual(hits, [False, False, True, False, True, False, False])
        # 다가오는 벽은 1.2 m 근처에서 먼저 잡히고 줄어든다 → 유리 아님
        wall = [INF, 1.18, 1.0, 0.8, 0.6, 0.45]
        self.assertFalse(any(late_hit(p, f, 0.6, 1.2) for p, f in zip(wall, wall[1:])))
        self.assertTrue(late_hit(1.19, 0.5, 0.6, 1.2))    # 범위 끝(1.15 m 넘게) 값은 빈 것으로 본다
        self.assertFalse(late_hit(0.9, 0.5, 0.6, 1.2))    # 이미 보던 것이 가까워진 것

    def test_retreat_goes_back_along_incoming_path_not_opposite_heading(self):
        # 추락 기록: (6.4, -4.65)에서 서쪽으로 와 (5.73, -4.53)에서 멈추고 제자리에서 돌았다
        past = [(6.40, -4.65), (6.24, -4.72), (6.04, -4.76), (5.97, -4.77), (5.74, -4.79), (5.69, -4.67), (5.73, -4.55)]
        d = retreat_direction(past, (5.73, -4.53), 0.3)
        self.assertGreater(d[0], 0.6)   # 들어온 쪽(남동쪽, 방금 지나온 (5.97, -4.77))으로 물러난다
        self.assertLess(d[1], 0.0)
        # 그때 기수는 −135°라 예전 규칙(기수 반대쪽)은 북동쪽(+45°)으로 물러났다: 들어온 길과 60° 넘게 다르다
        back = (math.cos(math.radians(45)), math.sin(math.radians(45)))
        self.assertGreater(math.degrees(math.acos(d[0] * back[0] + d[1] * back[1])), 60)
        self.assertIsNone(retreat_direction([(1.0, 1.0), (1.05, 1.0)], (1.0, 1.02), 0.3))   # 제자리에만 있었으면 없다
        # 기수가 남쪽(−90°)을 볼 때 동쪽으로 가려면 기체 기준 왼쪽
        vx, vy = world_to_body(0.12, 0.0, -math.pi / 2)
        self.assertAlmostEqual(vx, 0.0, places=6)
        self.assertAlmostEqual(vy, 0.12, places=6)

    def test_slow_turn_near_obstacle_and_contact(self):
        self.assertEqual(limit_turn(0.8, 0.5, 0.8, 0.3), 0.3)
        self.assertEqual(limit_turn(-0.8, INF, 0.8, 0.3), -0.8)
        self.assertEqual(limit_turn(-0.8, INF, 0.8, 0.3, force=True), -0.3)
        self.assertIsNone(contact(math.radians(5), 3.0, math.radians(20), 15.0))
        self.assertIn("저절로", contact(math.radians(-25), 3.0, math.radians(20), 15.0))   # 142° → 117°
        self.assertIn("기울기", contact(0.0, 34.0, math.radians(20), 15.0))

    def test_exit_retraces_entry_trail_at_exact_angle(self):
        # 들어온 자취(0.1 m 간격)를 거꾸로 되짚는다: 옆으로 밀렸어도 선 위 뒤쪽 점을 겨눠 선으로 돌아온다
        trail = [(x / 10, 0.0, 1.0) for x in range(0, 21)]        # (0, 0) → (2, 0)
        cur = trail_cursor(trail, (2.05, 0.05), 0.3)
        self.assertEqual(cur, 20)
        target, cur, done = trail_target(trail, cur, (2.05, 0.05), 0.3)
        self.assertEqual((target[0], done), (1.7, False))
        target, cur, done = trail_target(trail, cur, (1.0, 0.2), 0.3)   # 옆으로 0.2 m 밀림
        self.assertEqual(cur, 10)
        self.assertLess(target[0], 1.0)
        self.assertEqual(target[1], 0.0)
        target, cur, done = trail_target(trail, cur, (0.05, 0.0), 0.3)
        self.assertEqual((cur, done), (0, True))
        # 추락 기록: 자취 끝 (5.73, −4.55) 근처에서 되짚으면 방금 지나온 (5.97, −4.77)쪽(남동)으로 — 기수와 무관
        past = [(6.40, -4.65, 1.0), (6.24, -4.72, 1.0), (6.04, -4.76, 1.0), (5.97, -4.77, 1.0), (5.74, -4.79, 1.0),
                (5.69, -4.67, 1.0), (5.73, -4.55, 1.0)]
        cur = trail_cursor(past, (5.73, -4.53), 0.3)
        target, cur, done = trail_target(past, cur, (5.73, -4.53), 0.3)
        self.assertGreater(target[0] - 5.73, 0.15)
        self.assertLess(target[1] + 4.53, -0.15)
        self.assertFalse(done)
        self.assertIsNone(trail_cursor(past, (9.0, 0.0), 0.3))   # 근처에 자취가 없으면 되짚지 못한다

    def test_exit_stops_at_trail_fold_and_trail_is_trimmed_after_retreat(self):
        # 실습실 Gazebo: 자취가 (3.26) → (2.61)에서 U턴해 (3.50)까지 돌아왔다. 되짚기가 접힌 점을 지나면 목표가 (2.61) ↔ (3.26)로 뒤집혔다
        pts = [(3.26, 8.29, 1.0), (2.61, 8.27, 1.0), (2.68, 8.34, 1.0), (2.80, 8.33, 1.0), (2.92, 8.34, 1.0), (3.05, 8.35, 1.0),
               (3.17, 8.35, 1.0), (3.50, 8.35, 1.0)]
        fold = trail_fold(pts, 7, math.radians(120))
        self.assertEqual(fold, 1)                       # (2.61, 8.27)이 접힌 점
        cut = pts[fold:]
        cur = trail_cursor(cut, (3.50, 8.35), 0.3)
        targets = []
        for here in [(3.50, 8.35), (3.2, 8.34), (2.95, 8.33), (2.75, 8.3), (2.64, 8.28)]:
            target, cur, done = trail_target(cut, cur, here, 0.3)
            targets.append(target[0])
        self.assertTrue(all(t <= 2.92 + 1e-9 for t in targets[1:]))   # 접힌 점 너머(3.26)를 겨누지 않는다
        self.assertTrue(done)                           # 접힌 점에서 끝
        corner = [(0.0, 0.0, 1.0), (0.5, 0.0, 1.0), (1.0, 0.0, 1.0), (1.0, 0.5, 1.0), (1.0, 1.0, 1.0)]   # 90° 모퉁이는 접힘이 아니다
        self.assertEqual(trail_fold(corner, 4, math.radians(120)), 0)
        # 0.3 m 물러난 뒤 자취를 지금 자리까지 자른다 (뒤에 남는 점이 접힘을 만들지 않게)
        trail = [(x / 10, 0.0, 1.0) for x in range(0, 11)]
        self.assertEqual(trail_trim(trail, (0.72, 0.03), 40, 0.5), 3)   # (0.7, 0)까지 남기고 3점 지움
        self.assertEqual(trail_trim(trail, (5.0, 5.0), 40, 0.5), 0)     # 근처에 자취가 없으면 그대로

    def test_trim_cuts_on_latest_leg_when_legs_overlap(self):
        # 11/11 Gazebo 주머니(폭 1 m 막다른 통로): 동쪽 안에서 서쪽으로 나오다(y 1.98) x 6.5에서 U턴해 x 7.2까지 되들어갔다(y 2.0→2.19).
        # 0.3 m 물러난 자리 (6.94, 1.95)에서는 옛 다리(나오던 길)의 점이 0.04 m로 더 가깝다 — 거기서 자르면 U턴과 방금 다리가 사라져
        # 되짚기가 통로 안쪽(동쪽)을 겨눈다. 가장 최근 다리의 가장 가까운 점에서 잘라야 한다
        out = [(round(7.6 - 0.1 * i, 2), 1.98, 1.0) for i in range(12)]                          # (7.6 → 6.5), 서쪽으로
        back = [(round(6.5 + 0.1 * i, 2), round(1.98 + 0.03 * i, 2), 1.0) for i in range(1, 8)]  # (6.6, 2.01) → (7.2, 2.19), 동쪽으로
        trail = out + back
        here = (6.94, 1.95)
        cut = trail_trim(trail, here, 40, 0.5)
        kept = trail[:len(trail) - cut]
        self.assertGreater(kept[-1][1], 2.05, "방금 들어온 다리 위에서 잘랐다")
        self.assertIn((6.5, 1.98, 1.0), kept, "U턴 점은 남는다")
        cur = trail_cursor(kept, here, 0.3)
        fold = trail_fold(kept, cur, math.radians(120))
        target, _, _ = trail_target(kept[fold:cur + 1], cur - fold, here, 0.3)
        self.assertLess(target[0], here[0] - 0.15, "되짚는 쪽은 서쪽(나가는 쪽)")

    def test_off_tail_tells_behind_from_ahead(self):
        self.assertAlmostEqual(off_tail((-1.0, 0.0), 0.0), 0.0, places=6)               # 똑바로 뒤
        self.assertAlmostEqual(off_tail((0.0, 1.0), 0.0), math.pi / 2, places=6)        # 옆
        self.assertAlmostEqual(off_tail((1.0, 0.0), 0.0), math.pi, places=6)            # 정면
        # 주머니 판: 기수 41°, 겨눈 곳이 동쪽(0°) → 뒤에서 139° 벗어남 = 기수 오른쪽 앞
        self.assertAlmostEqual(math.degrees(off_tail((1.0, 0.0), math.radians(41))), 139.0, delta=0.5)

    def test_free_width_tells_narrow_pocket_from_open_space(self):
        def walls(p):   # y = ±0.35 벽 → 폭 0.7 m 복도 (추락 자리 0.3–0.8 m)
            return abs(p[1]) >= 0.35
        self.assertAlmostEqual(free_width(walls, (0.0, 0.0, 1.0), 1.0), 0.7, delta=0.1)
        self.assertLess(free_width(walls, (0.0, 0.0, 1.0), 1.0), EXIT_WIDTH)
        self.assertGreaterEqual(free_width(lambda p: False, (0.0, 0.0, 1.0), 1.0), 2.0)   # 트인 곳
        self.assertGreaterEqual(free_width(lambda p: p[1] >= 0.2, (0.0, 0.0, 1.0), 1.0), 1.0)   # 한쪽 벽 옆은 좁은 곳이 아니다

    def test_room_around_sees_shelf_beside_the_drone(self):
        # 14:32: 정면 ToF는 선반 칸 안쪽(0.8–1.1 m)을 봤지만 옆 0.2 m에 선반 판이 있었다. 돌 자리는 사방을 본다
        def shelf(p):
            return p[1] <= -0.2 or abs(p[0] - 0.25) < 0.02
        self.assertLess(room_around(shelf, (0.0, 0.0, 1.0), 1.0), 0.25)
        self.assertGreaterEqual(room_around(lambda p: False, (0.0, 0.0, 1.0), 1.0), 1.0)

    def test_pocket_only_when_no_room_to_turn_and_both_front_sides_blocked(self):
        # 사용자(11/03): 정면 좌·우 1 m 안에 장애물이 모두 있을 때만 후진. 폭 0.4 m 막다른 곳(기수 +x, 끝 벽 x 0.5)
        def dead_end(p):
            return abs(p[1]) >= 0.2 or p[0] >= 0.5
        blocked, room, left, right = pocket(dead_end, (0.0, 0.0, 1.0), 0.0, 0.25)
        self.assertTrue(blocked)
        self.assertLess(room, 0.25)
        self.assertLess(max(left, right), 1.0)
        # 1.6 m 복도 한가운데: 70° 광선은 0.85 m에서 벽을 보지만 돌 자리(0.8 m)가 있으니 주머니가 아니다
        corridor = lambda p: abs(p[1]) >= 0.8   # noqa: E731
        self.assertFalse(pocket(corridor, (0.0, 0.0, 1.0), 0.0, 0.25)[0])
        self.assertGreaterEqual(sector_clear(corridor, (0.0, 0.0, 1.0), 0.0, 1), 0.8)
        # 오른쪽 벽에 0.2 m로 붙었지만 왼쪽 앞이 트임: 돌 자리는 없어도 왼쪽으로 돌아 나갈 수 있다 → 주머니 아님
        right_wall = lambda p: p[1] <= -0.2   # noqa: E731
        blocked, room, left, right = pocket(right_wall, (0.0, 0.0, 1.0), 0.0, 0.25)
        self.assertFalse(blocked)
        self.assertLess(right, 1.0)
        self.assertGreaterEqual(left, 1.0)

    def test_retreat_goes_straight_back_with_small_side_correction(self):
        # 자취가 바로 뒤: 곧게 뒤로. 뒤에서 30° 벗어남: 옆 보정은 0.05까지. 90° 옆인데 돌 자리가 있으면 그래도 곧게 뒤로
        self.assertEqual(retreat_body((-1.0, 0.0), 0.0, 0.12, 0.5, 0.25, 0.05, math.radians(45)), (-0.12, 0.0))
        vx, vy = retreat_body((-math.cos(math.radians(30)), math.sin(math.radians(30))), 0.0, 0.12, 0.5, 0.25, 0.05, math.radians(45))
        self.assertEqual(vx, -0.12)
        self.assertAlmostEqual(vy, 0.05)
        vx, vy = retreat_body((0.0, 1.0), 0.0, 0.12, 0.5, 0.25, 0.05, math.radians(45))
        self.assertEqual((vx, vy), (-0.12, 0.05))
        # 90° 옆이고 돌 자리가 없으면(주머니에서 돌고 난 뒤) 그 자취 방향으로만 — 지나온 선이 유일하게 아는 빈 곳
        vx, vy = retreat_body((0.0, 1.0), 0.0, 0.12, 0.1, 0.25, 0.05, math.radians(45))
        self.assertAlmostEqual(vx, 0.0)
        self.assertAlmostEqual(vy, 0.12)
        self.assertEqual(retreat_body(None, 1.0, 0.12, 0.0, 0.25, 0.05, math.radians(45)), (-0.12, 0.0))

    def test_gap_rule_matches_drone_size(self):
        # 기체 20×20×10 cm(가드 포함): 양옆 5 cm·위아래 5 cm 남는 틈이 하한
        self.assertEqual((GAP_MIN_WIDTH, GAP_MIN_HEIGHT), (0.30, 0.20))


if __name__ == "__main__":
    unittest.main()
