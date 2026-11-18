"""3단계 사람 찾기 탐색(본 곳 지도, 다음 볼 자리, 찾은 사람 목록) 검사. 실행: python -m unittest discover -s tests"""
import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from autonomy import VoxelMap  # noqa: E402
from autonomy.explore import (CoverageMap, FoundPerson, PeopleLog, ViewpointBudget, people_risk,  # noqa: E402
                               person_in_view, person_seen_toward, person_top)
from autonomy.state_estimator import occupancy_slice  # noqa: E402
from worldgen import generate, ground_truth, known_map  # noqa: E402


def _box(x, y, z, sx, sy, sz):
    return {"x": x, "y": y, "z": z, "sx": sx, "sy": sy, "sz": sz}


def _two_rooms():
    """8×4 m, x = 4에 칸막이 벽(가운데 y 1.5–2.5에 문)."""
    t, h = 0.1, 2.5
    walls = [_box(4, t / 2, h / 2, 8, t, h), _box(4, 4 - t / 2, h / 2, 8, t, h),
             _box(t / 2, 2, h / 2, t, 4, h), _box(8 - t / 2, 2, h / 2, t, 4, h),
             _box(4, 0.75, h / 2, t, 1.5, h), _box(4, 3.25, h / 2, t, 1.5, h)]
    return {"bounds": {"xmin": 0, "ymin": 0, "xmax": 8, "ymax": 4, "zmax": h}, "walls": walls, "lintels": [],
            "obstacles": []}


class CoverageTest(unittest.TestCase):
    def setUp(self):
        self.data = _two_rooms()
        occ, origin, res = occupancy_slice(self.data, 1.0, 0.1)
        self.cov = CoverageMap(occ, origin, res)

    def test_camera_sees_only_ahead_and_not_through_walls(self):
        seen = self.cov.visible(2.0, 0.8, 0.0)          # 서쪽 방 남쪽에서 동쪽을 봄
        i_far = int(6.0 / self.cov.p.cell)              # 칸막이 너머 동쪽 방
        j = int(0.8 / self.cov.p.cell)
        self.assertTrue(seen[int(3.0 / self.cov.p.cell), j])
        self.assertFalse(seen[i_far, j], "칸막이 벽 너머는 안 보여야 한다")
        self.assertFalse(seen[int(1.0 / self.cov.p.cell), j], "등 뒤는 안 보여야 한다")

    def test_next_viewpoint_goes_to_unseen_room_and_finishes(self):
        inf = VoxelMap.from_known_map(self.data, 0.1).inflate(0.25)
        free2d = ~inf.occ[:, :, int((1.0 - inf.origin[2]) / inf.res)]
        x, y, visited = 1.5, 2.0, []
        for _ in range(4):                               # 서쪽 방은 둘러보기로 다 본다
            yaw, gain = self.cov.best_look(x, y)
            if gain < self.cov.p.min_gain_cells:
                break
            self.cov.observe(x, y, yaw)
        vp = self.cov.next_viewpoint(x, y, free2d, inf.origin[:2], inf.res)
        self.assertIsNotNone(vp)
        self.assertGreater(vp[0], 4.0, "다음 볼 자리는 아직 못 본 동쪽 방이어야 한다")
        for _ in range(20):                              # 계속 가서 보면 끝난다
            if vp is None:
                break
            x, y = vp[0], vp[1]
            for _ in range(6):
                yaw, gain = self.cov.best_look(x, y)
                if gain < self.cov.p.min_gain_cells:
                    break
                self.cov.observe(x, y, yaw)
            visited.append((x, y))
            vp = self.cov.next_viewpoint(x, y, free2d, inf.origin[:2], inf.res, exclude=visited)
        self.assertIsNone(vp)
        self.assertGreater(self.cov.coverage(), 0.9)


class GeneratedMapTest(unittest.TestCase):
    def test_humans_become_visible_on_generated_maps(self):
        for seed in (7, 3):
            with self.subTest(seed=seed):
                w = generate(seed, 3)
                km, gt = known_map(w), ground_truth(w)
                occ, origin, res = occupancy_slice(km, 1.0, 0.1)
                cov = CoverageMap(occ, origin, res)
                inf = VoxelMap.from_known_map(km, 0.1).inflate(0.25)
                free2d = ~inf.occ[:, :, int((1.0 - inf.origin[2]) / inf.res)]
                x, y, visited = w.spawn["x"], w.spawn["y"], []
                for _ in range(25):
                    vp = cov.next_viewpoint(x, y, free2d, inf.origin[:2], inf.res, exclude=visited)
                    if vp is None:
                        break
                    x, y = vp[0], vp[1]
                    for _ in range(6):
                        yaw, gain = cov.best_look(x, y)
                        if gain < cov.p.min_gain_cells:
                            break
                        cov.observe(x, y, yaw)
                    visited.append((x, y))
                self.assertGreater(cov.coverage(), 0.9)
                for h in gt["humans"]:
                    i, j = int((h["x"] - cov.origin[0]) / cov.p.cell), int((h["y"] - cov.origin[1]) / cov.p.cell)
                    self.assertTrue(cov.seen[i, j], f"{h['name']} 자리를 못 봄")


class PeopleLogTest(unittest.TestCase):
    def test_merge_same_person(self):
        log = PeopleLog(merge=1.0)
        self.assertIsNotNone(log.add(2.0, 2.0, 0.8, 10.0))
        self.assertIsNone(log.add(2.4, 2.1, 0.9, 11.0))   # 같은 사람 (1 m 안)
        self.assertIsNotNone(log.add(4.0, 2.0, 0.7, 12.0))
        self.assertEqual(len(log.people), 2)
        self.assertAlmostEqual(log.people[0].x, 2.2)
        self.assertAlmostEqual(log.people[0].conf, 0.9)
        self.assertEqual(log.people[0].t_first, 10.0)
        log.add(6.0, 2.0, 0.44, 13.0)                       # 한 번 흐릿하게: 확정 목록에는 없다
        self.assertEqual(len(log.confirmed()), 2)            # 두 번 잡힌 사람 + 신뢰도 0.7로 한 번 잡힌 사람

    def test_entries_that_drift_together_merge(self):
        log = PeopleLog(merge=1.0)
        log.add(9.9, 1.4, 0.9, 1.0)           # 첫 추정이 0.75 m 빗나감 (실제 9.15)
        log.add(8.9, 1.5, 0.9, 1.2)           # 첫 항목과 1 m 넘게 떨어져 따로 생긴다
        self.assertEqual(len(log.people), 2)
        for k in range(5):                     # 첫 항목이 제자리로 옮겨 오면 둘이 합쳐져야 한다
            log.add(9.15, 1.45, 0.9, 2.0 + k)
        self.assertEqual(len(log.people), 1)
        self.assertEqual(log.people[0].hits, 7)
        self.assertEqual(log.people[0].t_first, 1.0)

    def test_person_seen_empty_stops_blocking_until_seen_again(self):
        # 2026-10-30 복도: 지나간 사람이 지우지 않는 장애물로 남아 통로·출발점을 막았다
        log = PeopleLog(merge=1.0)
        p = log.add(2.0, 0.0, 0.9, 1.0)
        self.assertEqual(log.blocking(), [p])
        for k in range(4):
            self.assertFalse(log.note_empty(p, 5))
        self.assertTrue(log.note_empty(p, 5))           # 다섯 번째에 막기를 푼다
        self.assertFalse(log.note_empty(p, 5))          # 이미 풀렸으면 다시 알리지 않는다
        self.assertEqual(log.blocking(), [])
        self.assertEqual(log.confirmed(), [p])          # 찾은 사람 목록에는 남는다
        self.assertIsNone(log.add(2.2, 0.1, 0.8, 9.0))  # 같은 자리에서 다시 잡히면 다시 막는다
        self.assertEqual(log.blocking(), [p])
        self.assertEqual((p.misses, p.gone, p.t_last), (0, False, 9.0))

    def test_person_top_from_head_row_and_kept_as_tallest(self):
        hfov, w, h = 1.4416, 960, 720
        f = (w / 2) / math.tan(hfov / 2)
        # 카메라 1.0 m, 2 m 앞 사람 머리가 화면 가운데보다 0.7 m 위 → 1.7 m
        y1 = h / 2 - 0.7 * f / 2.0
        self.assertAlmostEqual(person_top((400, y1, 560, 700), 1.0, 2.0, w, h, hfov), 1.7, places=3)
        self.assertIsNone(person_top((400, 2, 560, 700), 1.0, 2.0, w, h, hfov))   # 머리가 잘림
        log = PeopleLog(merge=1.0)
        p = log.add(2.0, 0.0, 0.9, 1.0)                  # 처음엔 머리가 잘려 못 쟀다
        self.assertIsNone(p.top)
        log.add(2.1, 0.0, 0.9, 2.0, top=1.1)
        log.add(2.0, 0.1, 0.9, 3.0, top=0.95)            # 작게 잰 값은 버리고 큰 값을 남긴다
        log.add(2.0, 0.0, 0.9, 4.0)
        self.assertEqual((p.top, p.top_n), (1.1, 2))

    def test_people_risk_high_only_when_close_or_surrounded(self):
        # 2026-11-02: 사람이 막아도 위험이 낮으면 탐색을 이어 가고, 높을 때만 그 자리에 내린다
        def at(*xy):
            return [FoundPerson(x, y, 0.9, 0.0) for x, y in xy]
        self.assertIsNone(people_risk([], 0.0, 0.0))
        self.assertIsNone(people_risk(at((0.9, 0.0), (0.0, 0.8)), 0.0, 0.0))               # 앞·왼쪽: 뒤가 비었다
        self.assertIn("0.5 m", people_risk(at((0.5, 0.0)), 0.0, 0.0))                     # 바로 옆
        self.assertIn("둘러", people_risk(at((0.8, 0.0), (-0.4, 0.7), (-0.4, -0.7)), 0.0, 0.0))   # 세 방향
        self.assertIsNone(people_risk(at((0.8, 0.0), (0.4, 0.7), (0.4, -0.7)), 0.0, 0.0))  # 셋 다 앞쪽: 뒤로 빠진다
        self.assertIsNone(people_risk(at((1.5, 0.0), (-1.5, 0.3), (0.0, -1.5)), 0.0, 0.0))  # 1 m 밖

    def test_person_in_view_only_straight_ahead_and_in_range(self):
        hfov = 1.4416
        self.assertIsNotNone(person_in_view(2.0, 0.0, (0.0, 0.0, 1.0), 0.0, hfov))
        d, b = person_in_view(2.0, 1.0, (0.0, 0.0, 1.0), 0.0, hfov)
        self.assertAlmostEqual(d, 5 ** 0.5)
        self.assertAlmostEqual(b, 0.4636, places=3)
        self.assertIsNone(person_in_view(0.0, 2.0, (0.0, 0.0, 1.0), 0.0, hfov))   # 옆 (화면 밖)
        self.assertIsNone(person_in_view(5.0, 0.0, (0.0, 0.0, 1.0), 0.0, hfov))   # 3.5 m 넘게 멂
        self.assertIsNone(person_in_view(0.5, 0.0, (0.0, 0.0, 1.0), 0.0, hfov))   # 너무 가까움 (몸이 잘림)
        self.assertIsNotNone(person_in_view(-2.0, 0.0, (0.0, 0.0, 1.0), 3.1, hfov))   # 뒤를 보면 뒤의 사람

    def test_person_seen_toward_allows_position_error(self):
        self.assertTrue(person_seen_toward(0.0, 3.0, [0.2]))       # 3 m에서 0.25 rad(약 0.7 m) 안
        self.assertFalse(person_seen_toward(0.0, 3.0, [0.4]))
        self.assertTrue(person_seen_toward(0.0, 1.0, [0.55]))      # 가까우면 0.7 m가 더 큰 각도
        self.assertFalse(person_seen_toward(0.0, 3.0, []))


class PersonGoneTimeTest(unittest.TestCase):
    def test_gone_needs_count_and_one_second(self):
        # 탐지가 초당 10번이 되면 '5번 못 봄'이 0.5초다. 시간도 1초 넘어야 지운다 (11/04)
        log = PeopleLog()
        p = log.add(1.0, 0.0, 0.9, 0.0)
        for k in range(5):
            self.assertFalse(log.note_empty(p, 5, 1.0 + 0.1 * k, 1.0), f"{k + 1}번째 (0.{k}초)")
        self.assertFalse(log.note_empty(p, 5, 1.9, 1.0))
        self.assertTrue(log.note_empty(p, 5, 2.0, 1.0))
        log.add(1.0, 0.0, 0.9, 3.0)               # 다시 잡히면 처음부터
        self.assertFalse(p.gone)
        self.assertEqual(p.misses, 0)
        q = log.add(5.0, 0.0, 0.9, 0.0)
        for k in range(4):
            log.note_empty(q, 5)
        self.assertTrue(log.note_empty(q, 5), "시각을 안 주면 예전처럼 횟수만 센다")


class ViewpointBudgetTest(unittest.TestCase):
    """11/04 10:17 실기체 로그의 시각·경로 길이를 그대로 넣는다 (시각은 볼 자리를 고른 때부터)."""

    def test_far_viewpoint_E_given_up_on_seventh_replan(self):
        # E (2.6, 6.4): 처음 경로 3.19 m, 그 뒤 다시 계획할 때마다 4.63·3.73·4.13·3.50·3.59·2.02·4.88 m … 56초 뒤에야 '길이 없음'
        b = ViewpointBudget(0.0)
        b.remaining(3.19, 0.0)
        for t, length in [(2.8, 4.63), (3.9, 3.73), (4.9, 4.13), (5.9, 3.50), (7.0, 3.59), (8.1, 2.02)]:
            b.note_replan(t)
            self.assertIsNone(b.give_up(t), f"{t} s")
            b.remaining(length, t)
        b.note_replan(11.1)
        why = b.give_up(11.1)
        self.assertIsNotNone(why)
        self.assertIn("재계획 7번", why)

    def test_replans_count_once_per_second(self):
        # 계획용 지도를 0.5초마다 만들면 같은 막힘에 재계획이 잦아진다: 1초 안의 것은 한 번만 센다
        b = ViewpointBudget(0.0)
        for i in range(20):
            b.note_replan(0.3 * i)
        self.assertEqual(b.replans, 5)          # 0, 1.2, 2.4, 3.6, 4.8초 (20번 중)
        self.assertIsNone(b.give_up(5.7))

    @staticmethod
    def _drive(b, t0, t1, moving=True, length=None):
        """t0–t1 초를 0.05 s 주기(20 Hz)로 경로를 따라간다. length를 주면 그 길이를 알린다."""
        t = t0
        while t <= t1 + 1e-9:
            if length is not None:
                b.remaining(length, t)
            b.tick(t, moving)
            t += 0.05

    def test_viewpoint_C_given_up_when_path_stops_shrinking(self):
        # C (5.4, 0.5): 경로 4.48 m → 8초 뒤 3.73 m, 그 뒤로는 다시 계획할수록 5.45·6.27·6.59·6.77·6.92 m로 길어졌다
        b = ViewpointBudget(0.0)
        b.remaining(4.48, 0.0)
        self._drive(b, 0.0, 8.0)
        b.remaining(3.73, 8.0)
        self._drive(b, 8.0, 13.3, length=5.45)
        b.replans = 5
        self._drive(b, 13.3, 19.9, length=6.92)
        self.assertIsNone(b.give_up(19.9))
        self._drive(b, 19.9, 20.2, length=6.92)
        self.assertIn("줄지 않음", b.give_up(20.2))   # 3.73 m가 된 8초부터 12초 움직임 (실제로는 28초 뒤에야 다른 자리를 골랐다)

    def test_turning_in_place_is_not_counted(self):
        # 장애물 가까이에서 17°/s로 180° 돌면 10초가 넘는다: 출발도 하기 전에 포기하면 안 된다 (Gazebo 복도·문 앞, 11/04)
        b = ViewpointBudget(0.0)
        b.remaining(3.0, 0.0)
        self._drive(b, 0.0, 15.0, moving=False)
        self.assertIsNone(b.give_up(15.0))
        self._drive(b, 15.0, 27.5, moving=True)   # 그 뒤 12초 넘게 움직였는데도 줄지 않으면 포기
        self.assertIsNotNone(b.give_up(27.5))

    def test_steady_progress_is_never_cut_and_tight_paths_get_less(self):
        b = ViewpointBudget(0.0)
        for i in range(40):                     # 0.2 m/s로 꾸준히 다가가면 몇 초가 걸려도 포기하지 않는다
            t = i * 2.0
            self._drive(b, t, t + 1.95, length=10.0 - 0.2 * t)
            self.assertIsNone(b.give_up(t + 1.95))
        tight = ViewpointBudget(0.0)
        tight.tight = True
        tight.remaining(3.0, 0.0)
        tight.replans = 4
        self.assertIn("좁은 길", tight.give_up(1.0))
        loose = ViewpointBudget(0.0)
        loose.remaining(3.0, 0.0)
        self._drive(loose, 0.0, 8.5)
        self.assertIsNone(loose.give_up(8.5))
        tight2 = ViewpointBudget(0.0)
        tight2.tight = True
        tight2.remaining(3.0, 0.0)
        self._drive(tight2, 0.0, 8.5)
        self.assertIsNotNone(tight2.give_up(8.5))


if __name__ == "__main__":
    unittest.main()
