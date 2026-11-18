"""지도에 없는 장애물 찾기·재계획 검사. 실행: python -m unittest discover -s tests"""
import math
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from autonomy import (ObstacleMemory, PlanError, TofRays, VoxelMap, expected_range, explained_by_map,  # noqa: E402
                      near_known, ground_box,
                      in_sight, inflate_box, is_known, is_unexpected, path_clear, plan, tof_box, with_boxes)
from autonomy.obstacles import ground_point, near_person_box  # noqa: E402
from worldgen import LayoutParams, generate, known_map  # noqa: E402

W, H, HFOV = 960, 720, 1.4416
F = (W / 2) / math.tan(HFOV / 2)
TOF_HALF = math.radians(12.5)
TOF_ROWS = (0.0, TOF_HALF, -TOF_HALF)   # nodes/mission_node.py와 같게
DRONE_RADIUS = 0.25
FOUND_RADIUS = 0.3    # nodes/mission_node.py와 같게
DRONE_HALF = 0.09


def _box(x, y, z, sx, sy, sz):
    return {"x": x, "y": y, "z": z, "sx": sx, "sy": sy, "sz": sz}


def _walled(width, depth, *obstacles):
    """사방이 벽으로 막힌 방 하나짜리 지도."""
    t, h = 0.1, 2.5
    walls = [_box(width / 2, t / 2, h / 2, width, t, h), _box(width / 2, depth - t / 2, h / 2, width, t, h),
             _box(t / 2, depth / 2, h / 2, t, depth, h), _box(width - t / 2, depth / 2, h / 2, t, depth, h)]
    return {"bounds": {"xmin": 0, "ymin": 0, "xmax": width, "ymax": depth, "zmax": h},
            "walls": walls, "lintels": [], "obstacles": list(obstacles)}


def _bbox(cam, yaw, fwd, left, width, top):
    """카메라 앞 fwd m, 왼쪽 left m에 서 있는 폭 width, 높이 top인 물체의 YOLO 상자 (앞면 기준)."""
    u = W / 2 - F * left / fwd
    return (u - F * width / 2 / fwd, H / 2 - F * (top - cam[2]) / fwd, u + F * width / 2 / fwd, H / 2 + F * cam[2] / fwd)


class TofTest(unittest.TestCase):
    def setUp(self):
        self.vmap = VoxelMap.from_known_map(_walled(4, 3), res=0.1)
        self.rays = TofRays.fan(7, TOF_HALF)

    def test_expected_range_hits_wall(self):
        # 오른쪽 벽 안쪽 면은 x = 3.9. 복셀로 바꾸면 최대 한 칸(0.1 m) 두꺼워진다
        r = expected_range(self.vmap, (2.5, 1.5, 1.0), self.rays.directions(0.0), max_range=2.0)
        self.assertGreater(r, 1.25)
        self.assertLessEqual(r, 1.42)

    def test_expected_range_nothing_in_reach(self):
        self.assertEqual(expected_range(self.vmap, (1.5, 1.5, 1.0), self.rays.directions(0.0), 1.2), math.inf)

    def test_cone_sees_off_axis_object(self):
        vmap = with_boxes(self.vmap, [_box(2.5, 1.75, 1.0, 0.1, 0.1, 2.0)])   # 정면에서 약 10° 옆
        r = expected_range(vmap, (1.2, 1.5, 1.0), self.rays.directions(0.0), 1.2)
        self.assertLess(r, 1.3)
        narrow = expected_range(vmap, (1.2, 1.5, 1.0), TofRays.fan(1, 0).directions(0.0), 1.2)
        self.assertEqual(narrow, math.inf)

    def test_row_geometry_matches_pitched_lidar(self):
        rays = TofRays.fan(3, TOF_HALF, rows=TOF_ROWS)
        dirs = rays.directions(math.pi / 2)        # 북쪽(+y)을 볼 때
        self.assertEqual(len(dirs), 9)
        for d in dirs:
            self.assertAlmostEqual(float(np.linalg.norm(d)), 1.0, places=6)
            self.assertGreater(d[1], 0.9)
        self.assertAlmostEqual(dirs[1][2], 0.0)                         # 가운데 줄 가운데
        self.assertAlmostEqual(dirs[4][2], math.sin(TOF_HALF), places=6)   # 위 줄
        self.assertAlmostEqual(dirs[7][2], -math.sin(TOF_HALF), places=6)  # 아래 줄
        self.assertAlmostEqual(dirs[0][0], math.sin(TOF_HALF), places=6)   # 첫 광선은 오른쪽: 북쪽을 보면 동쪽(+x)

    def test_is_unexpected(self):
        self.assertTrue(is_unexpected(0.6, math.inf, 1.2))
        self.assertTrue(is_unexpected(0.6, 1.0, 1.2))
        self.assertFalse(is_unexpected(0.9, 1.0, 1.2))       # 지도와 비슷하면 아는 물체
        self.assertFalse(is_unexpected(math.inf, 1.0, 1.2))  # 측정 범위 밖
        self.assertFalse(is_unexpected(-math.inf, 1.0, 1.2))

    def test_tof_box_front_face_at_hit(self):
        b = tof_box((1.0, 1.0, 1.0), 0.0, 0.8, TOF_HALF, 2.5)
        self.assertAlmostEqual(b["x"] - b["sx"] / 2, 1.8, places=2)
        self.assertAlmostEqual(b["y"], 1.0, places=3)
        self.assertAlmostEqual(b["sy"], 2 * 0.8 * math.tan(TOF_HALF), places=2)   # 폭 = 그 거리의 원뿔 폭
        self.assertAlmostEqual(b["sx"], 0.6, places=3)                            # 깊이는 모르므로 0.6 m
        self.assertEqual((b["z"], b["sz"]), (1.25, 2.5))           # 바닥부터 천장까지
        b = tof_box((1.0, 1.0, 1.0), math.pi / 2, 0.2, TOF_HALF, 2.5)
        self.assertAlmostEqual(b["y"] - b["sy"] / 2, 1.2, places=2)
        self.assertEqual(b["sx"], 0.3)                             # 가까우면 최소 0.3 m


class YoloGroundTest(unittest.TestCase):
    def test_ground_point_and_box(self):
        cam = (0.0, 0.0, 1.0)
        bbox = _bbox(cam, 0.0, fwd=3.0, left=0.5, width=0.6, top=1.0)
        gx, gy, dist = ground_point(bbox, cam, 0.0, W, H, HFOV)
        self.assertAlmostEqual(gx, 3.0, places=2)
        self.assertAlmostEqual(gy, 0.5, places=2)
        self.assertAlmostEqual(dist, 3.0, places=2)
        b = ground_box(bbox, cam, 0.0, W, H, HFOV, top=2.5)
        self.assertAlmostEqual(b["x"], 3.3, places=2)    # 앞면에서 깊이 절반 뒤
        self.assertAlmostEqual(b["y"], 0.5, places=2)
        self.assertAlmostEqual(b["sy"], 0.6, places=2)

    def test_close_person_with_feet_cut(self):
        cam = (0.0, 0.0, 1.0)
        for fwd, left in ((0.8, 0.3), (1.2, -0.2)):
            x1, y1, x2, y2 = _bbox(cam, 0.0, fwd=fwd, left=left, width=0.45, top=1.7)
            bbox = (max(x1, 0), max(y1, 0), min(x2, W), min(y2, H))      # YOLO 상자는 화면 안으로 잘린다
            self.assertIsNone(ground_point(bbox, cam, 0.0, W, H, HFOV))  # 발이 잘려 바닥 위치는 못 쓴다
            box, dist = near_person_box(bbox, cam, 0.0, W, H, HFOV, top=2.5)
            self.assertAlmostEqual(dist, fwd, delta=0.1)
            self.assertAlmostEqual(box["y"], left, delta=0.1)
            self.assertLess(abs(box["x"] - (fwd + 0.2)), 0.15)
        # 발이 잘렸는데 머리로 재면 2 m: 가려졌거나 일부만 잡힌 상자라 버린다
        x1, y1, x2, y2 = _bbox(cam, 0.0, fwd=2.0, left=0.0, width=0.45, top=1.7)
        self.assertIsNone(near_person_box((x1, y1, x2, H), cam, 0.0, W, H, HFOV, top=2.5))
        # 발이 보이면 None (ground_box가 맡는다)
        self.assertIsNone(near_person_box(_bbox(cam, 0.0, fwd=3.0, left=0.0, width=0.45, top=1.7),
                                          cam, 0.0, W, H, HFOV, top=2.5))

    def test_person_box_ignores_arms(self):
        cam = (0.0, 0.0, 1.0)
        bbox = _bbox(cam, 0.0, fwd=2.0, left=0.0, width=1.0, top=1.75)   # 팔을 벌려 상자 폭 1 m
        b = ground_box(bbox, cam, 0.0, W, H, HFOV, top=2.5, name="person")
        self.assertAlmostEqual(b["sx"], 0.4, places=2)    # 깊이 0.4 m
        self.assertAlmostEqual(b["sy"], 0.7, places=2)    # 폭은 0.7 m까지만
        self.assertAlmostEqual(b["x"], 2.2, places=2)
        chair = ground_box(bbox, cam, 0.0, W, H, HFOV, top=2.5, name="chair")
        self.assertAlmostEqual(chair["sy"], 1.0, places=2)

    def test_rotated_camera(self):
        cam = (2.0, 2.0, 1.0)
        bbox = _bbox(cam, math.pi / 2, fwd=2.0, left=0.5, width=0.6, top=1.0)
        gx, gy, _ = ground_point(bbox, cam, math.pi / 2, W, H, HFOV)
        self.assertAlmostEqual(gx, 1.5, places=2)        # 북쪽을 보면 왼쪽은 서쪽(-x)
        self.assertAlmostEqual(gy, 4.0, places=2)

    def test_rejects_unusable_boxes(self):
        cam = (0.0, 0.0, 1.0)
        self.assertIsNone(ground_box((400, 300, 500, H - 1), cam, 0.0, W, H, HFOV, 2.5))   # 아래가 잘림
        self.assertIsNone(ground_box((400, 100, 500, 300), cam, 0.0, W, H, HFOV, 2.5))     # 수평선 위 (벽 그림 등)
        far = _bbox(cam, 0.0, fwd=6.0, left=0.0, width=0.6, top=1.0)
        self.assertIsNone(ground_box(far, cam, 0.0, W, H, HFOV, 2.5, max_distance=4.0))

    def test_known_and_sight(self):
        data = _walled(6, 3, _box(2.0, 1.5, 0.5, 0.6, 0.6, 1.0))
        data["walls"].append(_box(4.0, 1.5, 1.25, 0.1, 3.0, 2.5))   # 가운데 칸막이
        vmap = VoxelMap.from_known_map(data, 0.1)
        self.assertTrue(is_known(vmap, _box(2.2, 1.6, 1.25, 0.5, 0.5, 2.5)))
        self.assertFalse(is_known(vmap, _box(3.0, 0.6, 1.25, 0.4, 0.4, 2.5)))
        self.assertTrue(in_sight(vmap, (1.0, 0.5, 1.0), (3.0, 0.6)))
        self.assertFalse(in_sight(vmap, (1.0, 0.5, 1.0), (5.0, 0.6)))   # 칸막이 너머


class KnownStructureTest(unittest.TestCase):
    def setUp(self):
        self.known = VoxelMap.from_known_map(_walled(6, 4), 0.1)
        self.fan = list(np.linspace(-TOF_HALF, TOF_HALF, 7))

    def test_wall_seen_with_pose_error_is_explained(self):
        # 실제로는 동쪽 벽(x = 5.9)을 0.5 m 앞에서 봤는데, 추정 위치가 0.3 m 뒤(서쪽)로 틀어져 있음
        origin = (5.1, 2.0, 1.0)
        self.assertTrue(explained_by_map(self.known, origin, 0.0, 0.5, self.fan, TOF_HALF, 0.35))

    def test_obstacle_in_open_space_is_not_explained(self):
        origin = (1.0, 2.0, 1.0)
        self.assertFalse(explained_by_map(self.known, origin, 0.0, 0.8, self.fan, TOF_HALF, 0.3))

    def test_near_known(self):
        self.assertTrue(near_known(self.known, _box(5.6, 2.0, 1.25, 0.3, 0.3, 2.5), 0.3, 1.0))
        self.assertFalse(near_known(self.known, _box(3.0, 2.0, 1.25, 0.3, 0.3, 2.5), 0.3, 1.0))


class MemoryTest(unittest.TestCase):
    def test_keeps_boxes_separate(self):
        mem = ObstacleMemory()
        self.assertTrue(mem.add(_box(2.0, 1.0, 1.25, 0.4, 0.4, 2.5), "tof"))
        self.assertTrue(mem.add(_box(2.2, 1.3, 1.25, 0.4, 0.4, 2.5), "tof"))     # 겹쳐도 따로 둔다
        self.assertEqual(len(mem.items), 2)
        self.assertEqual(mem.items[0].box["sx"], 0.4)                                 # 감싸서 키우지 않는다
        self.assertFalse(mem.add(_box(2.05, 1.05, 1.25, 0.2, 0.2, 2.5), "yolo:chair"))   # 안에 다 들어감
        self.assertEqual(mem.items[0].hits, 2)
        self.assertEqual(mem.items[0].sources, {"tof", "yolo:chair"})

    def test_covering_and_drop(self):
        mem = ObstacleMemory()
        mem.add(_box(2.0, 1.0, 1.25, 0.4, 0.4, 2.5), "tof")
        mem.add(_box(5.0, 1.0, 1.25, 0.4, 0.4, 2.5), "yolo:person")
        mem.add(_box(7.0, 1.0, 1.25, 0.4, 0.4, 2.5), "contact")
        self.assertIs(mem.covering(2.25, 1.0), mem.items[0])     # 가장자리 0.1 m 여유
        self.assertIsNone(mem.covering(3.0, 1.0))
        self.assertEqual(mem.drop_camera_only(), 1)               # 카메라로만 본 것만 지운다
        self.assertEqual([i.sources for i in mem.items], [{"tof"}, {"contact"}])
        self.assertTrue(mem.suspected(5.3, 1.1))                  # 지운 자리는 카메라로 다시 넣지 않는다
        self.assertFalse(mem.suspected(6.0, 1.0))


def fly(truth: dict, known: dict, start, goal, step=0.05, max_ticks=3000):
    """작은 운동학 시뮬레이션: 진짜 지도에 ToF를 쏘고, 모르는 물체를 찾으면 지도에 더해 다시 계획한다.

    돌려주는 값: (도착 여부, 진짜 장애물과의 최소 간격(드론 외곽 기준), 재계획 횟수, 찾은 장애물 수)
    """
    true_map = VoxelMap.from_known_map(truth, 0.1)
    known_map_ = VoxelMap.from_known_map(known, 0.1)
    known_inf = known_map_.inflate(DRONE_RADIUS)
    rays = TofRays.fan(7, TOF_HALF, rows=TOF_ROWS)
    mem = ObstacleMemory()
    believed, inflated = known_map_, known_inf
    hidden = [o for o in truth["obstacles"] if o not in known["obstacles"]]
    pos = np.array(start, dtype=float)
    yaw = 0.0
    waypoints = plan(inflated, pos, goal)[1:]
    replans, streak, clearance = 0, 0, math.inf
    for _ in range(max_ticks):
        if not waypoints:
            return True, clearance, replans, len(mem.items)
        d = waypoints[0] - pos
        dist = float(np.linalg.norm(d[:2]))
        if dist < 0.1:
            waypoints.pop(0)
            continue
        yaw = math.atan2(d[1], d[0])
        origin = pos + 0.05 * np.array([math.cos(yaw), math.sin(yaw), 0.0])
        dirs = rays.directions(yaw)
        measured = expected_range(true_map, origin, dirs, 1.2)
        if is_unexpected(measured, expected_range(believed, origin, dirs, 1.2), 1.2):
            streak += 1
            if streak >= 2 and mem.add(tof_box(origin, yaw, measured, TOF_HALF, 2.5), "tof"):
                believed = with_boxes(known_map_, mem.boxes())
                inflated = with_boxes(known_inf, [inflate_box(b, FOUND_RADIUS) for b in mem.boxes()])
                if not path_clear(inflated, [pos] + waypoints):
                    waypoints = plan(inflated, pos, goal)[1:]
                    replans += 1
                    continue
        else:
            streak = 0
        move = min(step, dist)
        pos = pos + np.array([d[0], d[1], d[2]]) / max(float(np.linalg.norm(d)), 1e-9) * move
        for o in hidden:   # 드론 외곽 상자(수평 반폭 0.09, 높이 반폭 0.025)와 숨은 물건 사이 간격
            gap = [abs(pos[k] - o[c]) - o[s] / 2 - h for k, c, s, h in
                   ((0, "x", "sx", DRONE_HALF), (1, "y", "sy", DRONE_HALF), (2, "z", "sz", 0.025))]
            clear = max(gap) if all(g < 0 for g in gap) else math.sqrt(sum(max(g, 0.0) ** 2 for g in gap))
            clearance = min(clearance, clear)
    return False, clearance, replans, len(mem.items)


class ReplanLoopTest(unittest.TestCase):
    def check(self, hidden, start=(0.8, 2.0, 1.0), goal=(7.2, 2.0, 1.0)):
        known = _walled(8, 4)
        truth = _walled(8, 4, *hidden)
        reached, clearance, replans, found = fly(truth, known, start, goal)
        self.assertTrue(reached, "목적지에 닿지 못함")
        self.assertGreater(clearance, 0.0, "숨은 장애물에 부딪힘")
        self.assertGreaterEqual(replans, 1)
        self.assertGreaterEqual(found, 1)
        return clearance

    def test_box_on_straight_line(self):
        self.check([_box(4.0, 2.0, 0.6, 0.6, 0.6, 1.2)])

    def test_off_center_and_wide(self):
        self.check([_box(4.0, 2.3, 0.6, 0.6, 1.2, 1.2)])

    def test_two_boxes(self):
        self.check([_box(3.0, 2.0, 0.6, 0.6, 0.6, 1.2), _box(5.2, 1.6, 0.6, 0.6, 0.9, 1.2)])

    def test_object_just_below_flight_height(self):
        # 의자 등받이처럼 꼭대기가 비행 높이 바로 아래(0.96 m)인 물건: 가운데 줄은 못 보고 아래 줄이 본다
        self.check([_box(4.0, 2.0, 0.74, 0.44, 0.44, 0.44)])

    def test_low_object_is_flown_over(self):
        # 비행 높이(1 m)보다 낮은 물건은 ToF가 못 보지만 부딪히지도 않는다
        known = _walled(8, 4)
        truth = _walled(8, 4, _box(4.0, 2.0, 0.35, 0.4, 0.4, 0.7))
        reached, clearance, replans, _ = fly(truth, known, (0.8, 2.0, 1.0), (7.2, 2.0, 1.0))
        self.assertTrue(reached)
        self.assertEqual(replans, 0)
        self.assertGreater(clearance, 0.2)

    def test_blocked_corridor_raises(self):
        inflated = with_boxes(VoxelMap.from_known_map(_walled(8, 4), 0.1).inflate(DRONE_RADIUS),
                              [_box(4.0, 2.0, 1.25, 0.4, 4.0, 2.5)])
        with self.assertRaises(PlanError):
            plan(inflated, (0.8, 2.0, 1.0), (7.2, 2.0, 1.0))

    def test_short_person_is_passed_over_below_height_cap_but_tall_is_not(self):
        # 2026-11-02 사용자: 사람이 막으면 높이를 올려 지나갈 수 있는지 본다. 경로 높이 상한은 드라이버 1.5 m − 0.15
        def corridor(person_top):
            # 폭 1.2 m 복도를 가로막고 선 사람: 몸 상자(잰 키 + 0.1)에 사람 둘레 여유 0.15 m를 더한다
            person = inflate_box(_box(4.0, 2.0, (person_top + 0.1) / 2, 0.5, 1.2, person_top + 0.1), 0.15)
            base = VoxelMap.from_known_map(_walled(8, 4, _box(4.0, 0.7, 1.25, 1.0, 1.4, 2.5),
                                                   _box(4.0, 3.3, 1.25, 1.0, 1.4, 2.5)), 0.1).inflate(DRONE_RADIUS)
            return with_boxes(base, [person])
        path = plan(corridor(0.9), (0.8, 2.0, 1.25), (7.2, 2.0, 1.25), z_max=1.35)   # 앉은 키쯤: 위로 넘는다
        self.assertLessEqual(max(p[2] for p in path), 1.35 + 1e-6)
        a, b = next((a, b) for a, b in zip(path, path[1:]) if a[0] <= 4.0 <= b[0])   # 사람 위를 지나는 구간
        self.assertGreater(a[2] + (b[2] - a[2]) * (4.0 - a[0]) / max(b[0] - a[0], 1e-9), 1.15 - 1e-6)
        with self.assertRaises(PlanError):   # 선 사람(1.7 m)은 1.35 m 아래로 못 넘는다
            plan(corridor(1.7), (0.8, 2.0, 1.25), (7.2, 2.0, 1.25), z_max=1.35)


class BlockerWorldTest(unittest.TestCase):
    """--blockers로 만든 2단계 맵: 받은 지도로 짠 경로가 실제로 숨은 물건에 막혀야 시험이 된다."""

    def test_known_plan_is_blocked(self):
        blocked = total = 0
        seeds = range(6)
        for seed in seeds:
            w = generate(seed, 2, LayoutParams(blockers=2))
            data = known_map(w)
            inflated = VoxelMap.from_known_map(data, 0.1).inflate(DRONE_RADIUS)
            path = plan(inflated, (w.spawn["x"], w.spawn["y"], 1.0), (w.goal["x"], w.goal["y"], 1.0))
            for o in w.obstacles:
                if o.blocker:   # 경로 선이 물건 상자 자체를 지나간다
                    alone = VoxelMap.from_known_map({"bounds": data["bounds"], "obstacles": [_as_dict(o.box)]}, 0.1)
                    blocked += not path_clear(alone, path)
                    total += 1
        self.assertEqual(blocked, total, f"{total}개 중 {blocked}개만 경로 위에 있음")


def _as_dict(b):
    return {"x": b.x, "y": b.y, "z": b.z, "sx": b.sx, "sy": b.sy, "sz": b.sz}


if __name__ == "__main__":
    unittest.main()
