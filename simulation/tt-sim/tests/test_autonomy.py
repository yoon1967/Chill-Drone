"""복셀 지도·경로 계획·경로 추종 검사. 실행: python -m unittest discover -s tests"""
import math
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from autonomy import (FollowerParams, PlanError, VoxelMap, astar, farthest_point, follow,  # noqa: E402
                      path_length, plan)
from worldgen import generate, ground_truth, known_map  # noqa: E402

SEEDS = range(15)
DRONE_RADIUS = 0.25


def _box(x, y, z, sx, sy, sz):
    return {"x": x, "y": y, "z": z, "sx": sx, "sy": sy, "sz": sz}


def _room(*boxes):
    return {"bounds": {"xmin": 0, "ymin": 0, "xmax": 4, "ymax": 3, "zmax": 2.5},
            "walls": [], "lintels": [], "obstacles": list(boxes)}


class VoxelMapTest(unittest.TestCase):
    def test_box_becomes_voxels(self):
        vmap = VoxelMap.from_known_map(_room(_box(1.0, 1.0, 0.5, 0.4, 0.2, 1.0)), res=0.1)
        self.assertEqual(vmap.occ.shape, (40, 30, 25))
        self.assertEqual(int(vmap.occ.sum()), 4 * 2 * 10)
        self.assertTrue(vmap.occupied_at((1.0, 1.0, 0.5)))
        self.assertFalse(vmap.occupied_at((1.0, 1.0, 1.05)))   # 상자 위는 비어 있다
        self.assertTrue(vmap.occupied_at((1.0, 1.0, -0.05)))   # 바닥 아래는 막힘
        self.assertTrue(vmap.occupied_at((1.0, 1.0, 2.6)))     # 벽 높이 위도 막힘

    def test_inflate_grows_round(self):
        vmap = VoxelMap.empty((0, 0, 0), (1, 1, 1), 0.1)
        vmap.occ[5, 5, 5] = True
        big = vmap.inflate(0.2)
        self.assertTrue(big.occ[7, 5, 5] and big.occ[5, 3, 5] and big.occ[5, 5, 7])
        self.assertFalse(big.occ[7, 7, 7])   # 모서리 방향은 덜 부푼다(공 모양)
        self.assertEqual(int(vmap.occ.sum()), 1)   # 원본은 그대로

    def test_unreachable_goal_fails_fast(self):
        # 11/06 14:35 실기체: 길이 없을 때 A*가 닿는 칸을 다 뒤져 0.8–1.4초 걸렸다. 평면 연결 검사로 몇 ms 안에 끝나야 한다
        import time
        wall = VoxelMap.from_known_map(_room(_box(2.0, 1.5, 1.25, 0.2, 3.0, 2.5)), 0.1).inflate(DRONE_RADIUS)
        t0 = time.perf_counter()
        with self.assertRaises(PlanError) as ctx:
            plan(wall, (0.5, 1.5, 1.0), (3.5, 1.5, 1.0))
        self.assertIn("길이 없습니다", str(ctx.exception))
        self.assertLess(time.perf_counter() - t0, 0.25)
        low = VoxelMap.from_known_map(_room(_box(2.0, 1.5, 0.3, 0.2, 3.0, 0.6)), 0.1).inflate(DRONE_RADIUS)
        self.assertGreater(len(plan(low, (0.5, 1.5, 1.0), (3.5, 1.5, 1.0))), 1)   # 낮은 벽은 위로 넘는다 (평면 검사가 막지 않는다)

    def test_layers_that_do_not_connect_fail_fast(self):
        # 11/11 09:32 실기체: 층끼리 안 이어진 지도(가짜 바닥 위 1.35 m 층만 열림)에서 '길 없음'에 0.6–2.2초 걸렸다. 평면에 모으면
        # 이어져 보이지만(왼쪽 벽은 1.0 m 아래로, 오른쪽 벽은 1.4 m 위로만 열림) 3D로는 못 지난다 → A* 없이 바로 실패해야 한다
        import time
        vm = VoxelMap.empty((0, 0, 0), (20, 20, 2.5), 0.1)
        vm.occ[99, :, 10:] = True      # x 9.9–10.0: 1.0 m 위가 막힘
        vm.occ[100, :, :14] = True     # x 10.0–10.1: 1.4 m 아래가 막힘
        t0 = time.perf_counter()
        with self.assertRaises(PlanError):
            plan(vm, (5.0, 10.0, 1.0), (15.0, 10.0, 1.0))
        self.assertLess(time.perf_counter() - t0, 0.5)

    def test_high_start_comes_down_to_flight_height_early(self):
        # 11/11 09:32 실기체: 가구를 넘느라 1.4 m에 뜬 뒤 경로가 끝까지 1.35–1.45 m에 머물렀다. 밑이 비어 있으면 곧 비행 높이로 내려온다
        vm = VoxelMap.empty((0, 0, 0), (8, 4, 2.5), 0.1)
        path = plan(vm, (1.0, 2.0, 1.4), (6.0, 2.0, 1.0))
        self.assertGreaterEqual(len(path), 3)
        self.assertAlmostEqual(float(path[1][2]), 1.0, places=6)
        self.assertLess(float(np.linalg.norm(path[1][:2] - path[0][:2])), 1.2)
        self.assertTrue(all(abs(float(p[2]) - 1.0) < 1e-6 for p in path[1:]))
        # 바로 밑이 가구(1.2 m까지 막힘)면 한 층 계획을 쓰지 않고 넘어간 뒤 내려온다
        vm.occ[5:20, 15:25, :12] = True
        over = plan(vm, (1.0, 2.0, 1.4), (6.0, 2.0, 1.0))
        self.assertTrue(all(vm.segment_free(a, b) for a, b in zip(over, over[1:])))
        self.assertAlmostEqual(float(over[-1][2]), 1.0, places=6)

    def test_climbs_only_near_what_it_must_cross(self):
        # Gazebo 상자 넘기(11/11): 돌아올 때 0.7 m 상자 3.5 m 앞에서부터 1.35 m로 날았다. 넘어야 하는 곳 앞뒤로만 오르내린다
        vm = VoxelMap.empty((0, 0, 0), (9, 3, 2.5), 0.1)
        vm.occ[35:46, :, :13] = True                           # x 3.5–4.6 m, 방 폭 전체, 1.3 m 아래가 막힘 (부풀린 지도)
        path = plan(vm, (1.0, 1.5, 1.0), (8.0, 1.5, 1.0))
        self.assertTrue(all(vm.segment_free(a, b) for a, b in zip(path, path[1:])))
        self.assertGreaterEqual(max(float(p[2]) for p in path), 1.3)
        for p in path:
            if p[0] < 2.3 or p[0] > 5.8:                       # 상자 가장자리에서 1.2 m 넘게 떨어진 곳은 비행 높이
                self.assertAlmostEqual(float(p[2]), 1.0, places=6, msg=str([tuple(np.round(q, 2)) for q in path]))
        # 가구 사이 0.6 m 틈에서는 내려갔다 다시 오르지 않는다
        vm.occ[52:60, :, :13] = True                           # x 5.2–6.0 m에 하나 더
        path = plan(vm, (1.0, 1.5, 1.0), (8.0, 1.5, 1.0))
        self.assertTrue(all(vm.segment_free(a, b) for a, b in zip(path, path[1:])))
        between = [float(p[2]) for p in path if 4.6 <= p[0] <= 5.2]
        self.assertTrue(all(zz >= 1.3 for zz in between), between)

    def test_segment_free(self):
        vmap = VoxelMap.from_known_map(_room(_box(2.0, 1.5, 0.5, 0.2, 3.0, 1.0)), res=0.1)
        self.assertFalse(vmap.segment_free((0.5, 1.5, 0.5), (3.5, 1.5, 0.5)))   # 낮은 벽에 막힘
        self.assertTrue(vmap.segment_free((0.5, 1.5, 1.5), (3.5, 1.5, 1.5)))    # 위로는 지나감

    def test_save_load_and_ply(self):
        vmap = VoxelMap.from_known_map(_room(_box(1.0, 1.0, 0.5, 0.4, 0.4, 1.0)), res=0.1)
        with tempfile.TemporaryDirectory() as tmp:
            vmap.save(Path(tmp) / "m.npz")
            again = VoxelMap.load(Path(tmp) / "m.npz")
            self.assertTrue(np.array_equal(again.occ, vmap.occ))
            n = vmap.to_ply(Path(tmp) / "m.ply")
            self.assertGreater(n, 0)
            self.assertLess(n, int(vmap.occ.sum()))   # 겉면만 쓴다
            self.assertIn(f"element vertex {n}", (Path(tmp) / "m.ply").read_text())


class PlannerTest(unittest.TestCase):
    def test_goes_over_low_obstacle_and_around_tall_one(self):
        low = VoxelMap.from_known_map(_room(_box(2.0, 1.5, 0.3, 0.4, 3.0, 0.6)), 0.1).inflate(DRONE_RADIUS)
        path = plan(low, (0.5, 1.5, 1.0), (3.5, 1.5, 1.0))
        self.assertTrue(all(low.segment_free(a, b) for a, b in zip(path, path[1:])))
        tall = VoxelMap.from_known_map(_room(_box(2.0, 1.0, 1.25, 0.2, 2.0, 2.5)), 0.1).inflate(DRONE_RADIUS)
        path = plan(tall, (0.5, 1.0, 1.0), (3.5, 1.0, 1.0))
        self.assertGreater(max(p[1] for p in path), 2.0)   # 벽 끝(y=2.0)을 돌아간다

    def test_passes_door_at_center(self):
        # x = 2 칸막이에 폭 1.0 m 문(y 0.5–1.5). 문 위쪽 모서리를 스치는 지름길 대신 문 가운데로 지나야 한다
        room = _room(_box(2.0, 2.25, 1.25, 0.1, 1.5, 2.5), _box(2.0, 0.25, 1.25, 0.1, 0.5, 2.5))
        inflated = VoxelMap.from_known_map(room, 0.1).inflate(DRONE_RADIUS)
        path = plan(inflated, (0.5, 2.5, 1.0), (3.5, 2.5, 1.0))
        for a, b in zip(path, path[1:]):
            if (a[0] - 2.0) * (b[0] - 2.0) <= 0 and a[0] != b[0]:
                y = a[1] + (b[1] - a[1]) * (2.0 - a[0]) / (b[0] - a[0])
                self.assertLess(abs(y - 1.0), 0.1, f"문 가운데(y 1.0)가 아니라 y {y:.2f}로 지남")
                break
        else:
            self.fail("칸막이를 지나지 않음")
        self.assertTrue(all(abs(p[2] - 1.0) < 1e-9 for p in path))   # 칸 중심(0.95·1.05)이 아니라 비행 높이 그대로

    def test_start_by_wall_keeps_detour_through_middle(self):
        # 11/14 '가운데로': 벽(y < 0.2) 옆 0.15 m에서 출발해 벽 옆 자리로 간다. A*는 벽에서 떨어져 돌아가는데, 전에는 지름길 규칙이
        # '가장 좁은 곳'(출발점)만 비교해 벽을 따라가는 직선으로 되돌렸다. 이제 지름길이 더 비싸면 쓰지 않는다
        vmap = VoxelMap.empty((0, 0, 0), (8, 3, 2.5), 0.1)
        vmap.occ[:, :2, :] = True
        s, g = (0.55, 0.35, 1.0), (7.45, 0.35, 1.0)
        old = plan(vmap, s, g, z_min=0.9, z_max=1.1, by_cost=False)
        self.assertLess(max(p[1] for p in old), 0.45, "전 규칙: 벽을 따라가는 직선")
        path = plan(vmap, s, g, z_min=0.9, z_max=1.1)
        self.assertGreaterEqual(max(p[1] for p in path), 0.65, "벽에서 0.5 m쯤 떨어져 지난다")
        self.assertTrue(all(vmap.segment_free(a, b) for a, b in zip(path, path[1:])))
        self.assertLess(path_length(path), 1.15 * math.dist(s, g))   # 돌아가도 조금만
        self.assertLessEqual(len(path), 6)                           # 경유점이 잘게 쪼개지지 않는다

    def test_open_space_is_still_one_straight_line(self):
        # 벽에서 0.6 m 넘게 떨어진 곳끼리는 전처럼 곧게 잇는다 (비스듬해도)
        vmap = VoxelMap.empty((0, 0, 0), (8, 3, 2.5), 0.1)
        vmap.occ[:, :2, :] = True
        for s, g in (((0.55, 1.55, 1.0), (7.45, 1.55, 1.0)), ((0.55, 1.05, 1.0), (7.45, 2.55, 1.0))):
            self.assertEqual(len(plan(vmap, s, g, z_min=0.9, z_max=1.1)), 2)

    def test_keeps_flight_height_unless_blocked(self):
        # 가운데 1.0 m 높이 탁자(부풀리면 1.25 m)가 길을 거의 막는다. 위로 넘지 않고 옆으로 돌아가야 한다
        room = _room(_box(2.0, 1.2, 0.5, 1.0, 2.4, 1.0))
        inflated = VoxelMap.from_known_map(room, 0.1).inflate(DRONE_RADIUS)
        path = plan(inflated, (0.5, 1.5, 1.0), (3.5, 1.5, 1.0))
        self.assertTrue(all(p[2] < 1.25 for p in path))
        # 옆길도 없으면 그때는 넘어간다
        wall = _room(_box(2.0, 1.5, 0.5, 1.0, 3.0, 1.0))
        inflated = VoxelMap.from_known_map(wall, 0.1).inflate(DRONE_RADIUS)
        self.assertGreater(max(p[2] for p in plan(inflated, (0.5, 1.5, 1.0), (3.5, 1.5, 1.0))), 1.25)

    def test_farthest_point_is_reachable_and_far(self):
        world = generate(7, 3)
        km = known_map(world)
        inflated = VoxelMap.from_known_map(km, 0.1).inflate(DRONE_RADIUS)
        start = (km["spawn"]["x"], km["spawn"]["y"], 1.0)
        goal = farthest_point(inflated, start, 1.0)
        self.assertFalse(inflated.occupied_at(goal))
        self.assertGreater(math.dist(start[:2], goal[:2]), 3.0)
        self.assertTrue(plan(inflated, start, goal))   # 실제로 경로가 있다

    def test_diagonal_does_not_cut_corners(self):
        # 11/04 외부 리뷰: A*가 막힌 두 칸이 꼭짓점만 맞댄 사이를 대각으로 건너, 같은 코드의 직선 검사가 지나갈 수 없다고 보는 경로를
        # 돌려주었다. 칸막이 x = 2.0–2.1 m에 칸 하나씩 엇갈린 '바둑판 틈'만 있으면 지나갈 길이 없다
        vmap = VoxelMap.empty((0, 0, 0), (4, 3, 2.5), 0.1)
        vmap.occ[20, :, :] = True
        vmap.occ[21, :, :] = True
        vmap.occ[20, 15, :] = False      # (20, 15)와 (21, 16)만 비었다: 꼭짓점만 닿는다
        vmap.occ[21, 16, :] = False
        with self.assertRaises(PlanError):
            astar(vmap, (0.5, 1.5, 1.0), (3.5, 1.5, 1.0), z_min=0.9, z_max=1.1)
        vmap.occ[21, 15, :] = False      # 한 칸이 이어지면 지나간다
        path = plan(vmap, (0.5, 1.55, 1.0), (3.5, 1.55, 1.0), prefer=0.0, z_min=0.9, z_max=1.1)
        self.assertTrue(all(vmap.segment_free(a, b) for a, b in zip(path, path[1:])))

    def test_returned_path_always_passes_segment_check(self):
        # 출발·도착점이 칸 구석에 있어도 돌려준 모든 선분이 직선 검사를 지난다 (전에는 첫 칸 중심을 실제 출발점으로 바꿔 검사 없이 이었다)
        rng = np.random.default_rng(3)
        for trial in range(30):
            vmap = VoxelMap.empty((0, 0, 0), (4, 3, 2.5), 0.1)
            vmap.occ[:, :, :] = rng.random((40, 30, 1)) < 0.18
            vmap.occ[:3, :, :] = vmap.occ[-3:, :, :] = False
            start = (0.1 + rng.random() * 0.2, 0.2 + rng.random() * 2.6, 1.0)
            goal = (3.7 + rng.random() * 0.2, 0.2 + rng.random() * 2.6, 1.0)
            try:
                path = plan(vmap, start, goal, z_min=0.9, z_max=1.1)
            except PlanError:
                continue
            for a, b in zip(path, path[1:]):
                self.assertTrue(vmap.segment_free(a, b), f"시험 {trial}: {a} → {b}")

    def test_no_path_raises(self):
        wall = VoxelMap.from_known_map(_room(_box(2.0, 1.5, 1.25, 0.2, 3.0, 2.5)), 0.1).inflate(DRONE_RADIUS)
        with self.assertRaises(PlanError):
            astar(wall, (0.5, 1.5, 1.0), (3.5, 1.5, 1.0))

    def test_generated_worlds(self):
        """생성한 실내 맵에서 출발점→목적지 경로가 있고, 정답 장애물 전부와 부딪히지 않는다."""
        for seed in SEEDS:
            with self.subTest(seed=seed):
                world = generate(seed, 1)
                km, gt = known_map(world), ground_truth(world)
                inflated = VoxelMap.from_known_map(km, 0.1).inflate(DRONE_RADIUS)
                start = (km["spawn"]["x"], km["spawn"]["y"], km["flight_height"])
                goal = (km["goal"]["x"], km["goal"]["y"], km["goal"]["z"])
                path = plan(inflated, start, goal)
                self.assertLess(np.linalg.norm(path[-1][:2] - np.array(goal[:2])), 0.31)
                truth = VoxelMap.from_known_map({**km, "obstacles": gt["obstacles"]}, 0.1).inflate(0.1)
                for a, b in zip(path, path[1:]):
                    self.assertTrue(truth.segment_free(a, b), f"{a} → {b} 충돌")
                self.assertLess(path_length(path), 4 * math.dist(start, goal) + 6)
                self.assertTrue(all(0.4 - 1e-6 <= p[2] <= 1.85 for p in path[1:-1]))


class FollowerTest(unittest.TestCase):
    P = FollowerParams()

    def test_straight_ahead_moves_forward(self):
        cmd, done = follow((0, 0, 1, 0.0), (2, 0, 1), self.P, final=False)
        self.assertFalse(done)
        self.assertAlmostEqual(cmd.vx, self.P.speed)
        self.assertAlmostEqual(cmd.vy, 0.0)
        self.assertAlmostEqual(cmd.yaw_rate, 0.0)

    def test_turns_before_moving_when_target_is_behind(self):
        cmd, _ = follow((0, 0, 1, 0.0), (-2, 0.1, 1), self.P, final=False)
        self.assertEqual((cmd.vx, cmd.vy), (0.0, 0.0))
        self.assertAlmostEqual(abs(cmd.yaw_rate), self.P.yaw_rate_max)

    def test_body_frame_conversion(self):
        cmd, _ = follow((0, 0, 1, math.pi / 2), (0, 2, 1), self.P, final=False)   # 북쪽을 보고 북쪽 목표
        self.assertAlmostEqual(cmd.vx, self.P.speed, places=6)
        self.assertAlmostEqual(cmd.vy, 0.0, places=6)

    def test_slows_down_and_arrives(self):
        near, _ = follow((0, 0, 1, 0.0), (0.25, 0, 1), self.P, final=True)
        self.assertLess(near.vx, self.P.speed)
        _, done = follow((0, 0, 1, 0.0), (0.1, 0, 1), self.P, final=True)
        self.assertTrue(done)

    def test_climbs_with_limit(self):
        cmd, _ = follow((0, 0, 0.1, 0.0), (0, 0, 1.0), self.P, final=True)
        self.assertAlmostEqual(cmd.vz, self.P.vz_max)
        self.assertEqual((cmd.vx, cmd.yaw_rate), (0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
