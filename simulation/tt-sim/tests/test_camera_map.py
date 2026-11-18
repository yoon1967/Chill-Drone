"""카메라 깊이로 만드는 지도(4단계) 검사. 실행: python -m unittest discover -s tests"""
import dataclasses
import math
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from autonomy import VoxelMap  # noqa: E402
from autonomy.camera_map import CameraMap, CameraMapParams, _dilate, _near_cluster  # noqa: E402

HFOV = 1.4416
# 시험 영상은 96×72라, 실제(240×180, 6화소마다)와 광선 각도 간격을 맞춘다. 시험은 정답 깊이를 방향마다 한 장면만 넣으므로
# 2 m 밖도 한 장면에 막힘으로 친다(hit_far=hit). 실제 비행의 '두 장면 확인' 규칙은 test_far_wall_needs_two_frames가 본다
PARAMS = CameraMapParams(stride=2, hit_far=0.85, hit_max_range=4.0)   # 6×4 m 방 가운데서 벽이 3 m 밖
BOUNDS = {"xmin": 0, "ymin": 0, "xmax": 6, "ymax": 4, "zmax": 2.5}


def _box(x, y, z, sx, sy, sz):
    return {"x": x, "y": y, "z": z, "sx": sx, "sy": sy, "sz": sz}


def _world(*boxes):
    t, h = 0.1, 2.5
    walls = [_box(3, t / 2, h / 2, 6, t, h), _box(3, 4 - t / 2, h / 2, 6, t, h),
             _box(t / 2, 2, h / 2, t, 4, h), _box(6 - t / 2, 2, h / 2, t, 4, h)]
    return VoxelMap.from_known_map({"bounds": BOUNDS, "walls": walls, "lintels": [], "obstacles": list(boxes)}, 0.02)


def render_depth(world: VoxelMap, cam, yaw, w=96, h=72, max_t=8.0):
    """광선을 0.02 m씩 밟아 처음 막힌 곳(또는 바닥)까지의 광축 방향 거리 영상을 만든다."""
    f = (w / 2) / math.tan(HFOV / 2)
    vs, us = np.mgrid[0:h, 0:w] + 0.5
    left, up = -(us - w / 2) / f, -(vs - h / 2) / f
    c, s = math.cos(yaw), math.sin(yaw)
    dirs = np.stack([c - s * left, s + c * left, up], axis=-1).reshape(-1, 3)
    norm = np.linalg.norm(dirs, axis=1)
    unit = dirs / norm[:, None]
    depth = np.full(len(unit), np.inf)
    alive = np.ones(len(unit), dtype=bool)
    for t in np.arange(0.02, max_t, 0.02):
        p = np.asarray(cam) + unit * t
        idx = np.floor((p - world.origin) / world.res).astype(int)
        inside = np.all((idx >= 0) & (idx < world.shape), axis=1)
        occ = np.zeros(len(p), dtype=bool)
        occ[inside] = world.occ[idx[inside, 0], idx[inside, 1], idx[inside, 2]]
        stop = alive & (occ | (p[:, 2] <= 0.0))
        depth[stop] = t / norm[stop]
        alive &= ~stop
    return depth.reshape(h, w)


class IntegrateTest(unittest.TestCase):
    def test_wall_occupied_front_free_behind_unknown(self):
        world = _world(_box(3.0, 2.0, 1.25, 0.1, 4.0, 2.5))      # x = 3 칸막이 (2.95–3.05)
        cmap = CameraMap(BOUNDS, PARAMS)
        cmap.integrate_depth(render_depth(world, (1.0, 2.0, 1.0), 0.0), (1.0, 2.0, 1.0), 0.0, HFOV)
        k = cmap.slice_index(1.0)
        occ, unk = cmap.occupied()[:, :, 5:15].any(axis=2), cmap.unknown()[:, :, k]   # 높이 0.5–1.5 m
        self.assertTrue(occ[29:31, 20].any(), "칸막이가 막힘으로 찍혀야 한다")
        self.assertFalse(occ[12:28, 20].any(), "칸막이 앞은 빈 곳")
        self.assertTrue(cmap.known_free()[15:28, 20, k].all())
        self.assertTrue(unk[33:58, 20].all(), "칸막이 뒤는 모름")

    def test_far_wall_needs_two_frames_near_wall_needs_one(self):
        # 단안 깊이는 멀수록 틀려 한 장면만으로 가짜 벽이 섰다 (10/30 실기체, 막힌 칸의 25%). 2 m 밖은 두 장면이 맞아야 막힘
        world = _world(_box(3.0, 2.0, 1.25, 0.1, 4.0, 2.5))      # x = 3 칸막이
        far = CameraMap(BOUNDS, CameraMapParams(stride=2))                                          # 실제 규칙 (hit_far 0.4)
        far.integrate_depth(render_depth(world, (0.5, 2.0, 1.0), 0.0), (0.5, 2.0, 1.0), 0.0, HFOV)   # 2.45 m 앞
        self.assertFalse(far.occupied()[29:31, 20, 5:15].any(), "2 m 밖 한 장면은 아직 막힘이 아니다")
        self.assertTrue(far.seen[29:31, 20, 5:15].any(), "그래도 본 칸이다")
        self.assertTrue(far.planning_map(0.0).occupied_at((2.95, 2.0, 1.0)), "계획은 한 번 본 것도 피한다")
        far.integrate_depth(render_depth(world, (0.5, 2.0, 1.0), 0.0), (0.5, 2.0, 1.0), 0.0, HFOV)
        self.assertTrue(far.occupied()[29:31, 20, 5:15].any(), "두 장면이 맞으면 막힘")
        near = CameraMap(BOUNDS, CameraMapParams(stride=2))
        near.integrate_depth(render_depth(world, (1.5, 2.0, 1.0), 0.0), (1.5, 2.0, 1.0), 0.0, HFOV)   # 1.45 m 앞
        self.assertTrue(near.occupied()[29:31, 20, 5:15].any(), "2 m 안은 한 장면에 막힘")

    def test_nan_depth_is_a_failed_measurement_not_free(self):
        # 11/04 외부 리뷰: NaN뿐인 영상 한 장이 앞 4 m의 복셀 13,698개를 빈 곳으로 만들었다. NaN은 잴 수 없었던 화소다 (REP 117)
        cmap = CameraMap(BOUNDS, PARAMS)
        self.assertEqual(cmap.integrate_depth(np.full((72, 96), np.nan), (1.0, 2.0, 1.0), 0.0, HFOV), 0)
        self.assertFalse(cmap.seen.any(), "NaN 영상은 아무것도 비우지 않는다")
        world = _world(_box(3.0, 2.0, 1.25, 0.1, 4.0, 2.5))
        depth = render_depth(world, (1.0, 2.0, 1.0), 0.0)
        depth[:, :48] = np.nan                                     # 왼쪽 절반 (+y 쪽) 실패
        cmap.integrate_depth(depth, (1.0, 2.0, 1.0), 0.0, HFOV)
        k = cmap.slice_index(1.0)
        self.assertFalse(cmap.seen[20:28, 22:26, k].any(), "실패한 쪽(+y, 앞 1–1.8 m)은 그대로 모름")
        self.assertGreater(cmap.known_free()[20:28, 14:18, k].mean(), 0.9, "잰 쪽(−y)은 빈 곳")
        sky = CameraMap(BOUNDS, PARAMS)                            # +inf는 '범위 안에 없음' — max_range까지 비운다
        sky.integrate_depth(np.full((72, 96), np.inf), (1.0, 2.0, 1.0), 0.0, HFOV)
        self.assertTrue(sky.known_free()[15:40, 20, k].all())

    def test_frontier_prefers_travel_direction_and_skips_visited(self):
        # 복도: x 2–8 m를 봤고 양 끝(x<2, x>8)은 모름 → 경계 무리가 앞뒤에 하나씩 (옆은 지도 끝이라 경계가 아니다).
        # 가던 쪽을 고르고, 둘러본 자리 1.5 m 안은 피한다
        cmap = CameraMap({"xmin": 0.0, "ymin": 0.0, "xmax": 10.0, "ymax": 4.0, "zmax": 2.5}, CameraMapParams())
        i0, i1 = int(2.0 / cmap.p.res), int(8.0 / cmap.p.res)
        k = cmap.slice_index(1.0)
        cmap.seen[i0:i1, :, k - 1:k + 2] = True                # 본 빈 곳 (logodds 0 → 막힘 아님), 폭은 지도 끝까지
        plan = cmap.planning_map(0.0)
        ahead = cmap.frontier_viewpoint(5.0, 2.0, plan, 1.0, heading=0.0)
        back = cmap.frontier_viewpoint(5.0, 2.0, plan, 1.0, heading=math.pi)
        self.assertIsNotNone(ahead)
        self.assertGreater(ahead[0], 5.0)                       # +x 쪽 경계
        self.assertLess(back[0], 5.0)                           # 뒤를 보고 있으면 −x 쪽 경계
        again = cmap.frontier_viewpoint(5.0, 2.0, plan, 1.0, heading=0.0, visited=[(ahead[0], ahead[1])])
        self.assertIsNotNone(again)
        self.assertGreaterEqual(math.hypot(again[0] - ahead[0], again[1] - ahead[1]), 1.5 - 1e-6)   # 둘러본 자리 1.5 m 안은 다시 안 고름

    def test_frontier_skips_shelf_cubbies_already_seen_from_front(self):
        # 11/03 14:32: 노트북 보관함 앞은 다 봤는데 선반 칸 안쪽만 모르는 칸이라, 그걸 보러 선반 앞까지 들어가 뒤집혔다
        cmap = CameraMap({"xmin": 0.0, "ymin": 0.0, "xmax": 6.0, "ymax": 4.0, "zmax": 2.5}, CameraMapParams())
        r, k = cmap.p.res, cmap.slice_index(1.0)
        band = slice(k - 3, k + 4)
        cmap.seen[:, :, band] = True                              # 방은 다 봤다
        j0, j1 = int(3.6 / r), int(3.9 / r)
        cmap.seen[int(2.0 / r):int(4.0 / r), j0:j1, band] = False  # 선반 칸 안쪽만 모른다
        for xs in np.arange(2.0, 4.05, 0.4):                       # 칸막이 판과 뒤판
            cmap.logodds[int(xs / r), int(3.5 / r):int(4.0 / r), band] = 2.0
        cmap.logodds[int(2.0 / r):int(4.0 / r) + 1, int(3.9 / r), band] = 2.0
        plan = cmap.planning_map(0.0)
        self.assertIsNotNone(cmap.frontier_viewpoint(3.0, 1.5, plan, 1.0), "예전 규칙은 선반 앞을 볼 자리로 고른다")
        self.assertIsNone(cmap.frontier_viewpoint(3.0, 1.5, plan, 1.0, shadow=0.3, clearance=0.45, standoff=1.2),
                          "가구에 붙은 모르는 칸은 이미 다 본 곳")

    def test_door_frontier_is_viewed_from_roomy_spot_in_front(self):
        # 문(폭 0.8 m) 너머를 모른다: 경계 칸은 문틀 옆이라 좁다 → 문 앞 1.2 m 안, 막힌 곳에서 0.45 m 넘게 떨어진 자리에서 본다
        cmap = CameraMap({"xmin": 0.0, "ymin": 0.0, "xmax": 6.0, "ymax": 4.0, "zmax": 2.5}, CameraMapParams())
        r, k = cmap.p.res, cmap.slice_index(1.0)
        band = slice(k - 3, k + 4)
        iw = int(3.0 / r)
        cmap.seen[:iw + 1, :, band] = True                         # 문 이쪽 방과 벽(문 자리 포함)은 봤다
        cmap.logodds[iw, :, band] = 2.0
        cmap.logodds[iw, int(1.6 / r):int(2.4 / r), band] = -1.0    # 문 (y 1.6–2.4)
        plan = cmap.planning_map(0.0)
        vp = cmap.frontier_viewpoint(1.0, 2.0, plan, 1.0, shadow=0.3, clearance=0.45, standoff=1.2)
        self.assertIsNotNone(vp, "문 너머는 트인 모르는 곳이라 경계다")
        self.assertLess(vp[0], 3.0)                                 # 문 이쪽에서
        for fy in (1.55, 2.45):                                     # 문틀 모서리에서 0.45 m 넘게 떨어진 자리
            self.assertGreaterEqual(math.hypot(vp[0] - 3.05, vp[1] - fy), 0.45 - 1e-6)
        self.assertLess(abs(vp[1] - 2.0), 0.6)                     # 문 앞
        self.assertLessEqual(math.hypot(vp[0] - 3.0, vp[1] - 2.0), 1.0)   # 가는 길에서 문에 가장 가까운 넓은 자리

    def test_viewpoint_prefers_roomy_spot(self):
        # 11/14 '가운데로': 아래 벽(y < 0.2)을 따라가던 드론. x < 4 m는 봤고 그 너머를 모른다. 전에는 가는 길에서 가장 가까운 후보라
        # 벽 옆(둘레 0.45 m만 겨우 넘는 자리)을 골랐다. room_prefer를 주면 둘레가 1 m 빈 자리를, 조금 더 가서 고른다
        cmap = CameraMap({"xmin": 0.0, "ymin": 0.0, "xmax": 6.0, "ymax": 4.0, "zmax": 2.5}, CameraMapParams())
        r, k = cmap.p.res, cmap.slice_index(1.0)
        band = slice(k - 3, k + 4)
        cmap.seen[:int(4.0 / r), :, band] = True
        cmap.logodds[:int(4.0 / r), :2, band] = 2.0
        plan = cmap.planning_map(0.0)
        kw = dict(shadow=0.3, clearance=0.45, standoff=1.2)
        old = cmap.frontier_viewpoint(1.0, 0.7, plan, 1.0, **kw)
        new = cmap.frontier_viewpoint(1.0, 0.7, plan, 1.0, room_prefer=1.0, room_weight=2.0, **kw)
        self.assertIsNotNone(old)
        self.assertIsNotNone(new)
        self.assertLess(old[1] - 0.2, 0.8, "전 규칙은 벽 가까운 자리")
        self.assertGreaterEqual(new[1] - 0.2, 0.95 - 1e-6, "둘레 1 m가 빈 자리")
        self.assertLess(new[3] - old[3], 0.7)                         # 가는 거리는 조금만 더
        self.assertLess(math.hypot(new[0] - old[0], new[1] - old[1]), 1.0)

    VIEW = dict(shadow=0.3, clearance=0.45, standoff=1.2)          # 비행 노드와 같은 볼 자리 인자
    GAIN = dict(view_range=3.0, min_gain=0.3, cost_distance=6.0)   # 11/04 새 점수

    def test_view_gain_prefers_open_space_over_closed_nook(self):
        # 11/04 10:17 실기체: 출발점 옆 막힌 구석의 경계를 85초 동안 오가며, 7–9 m 떨어진 트인 라운지 쪽은 끝내 고르지 않았다.
        # 8×8 m 방(동쪽 벽 x = 8에 폭 1.2 m 트인 곳, 그 너머는 모름), 출발점 옆 북쪽 벽에 폭 2 m·깊이 1.2 m 구석(안은 모름)
        cmap = CameraMap({"xmin": 0.0, "ymin": 0.0, "xmax": 12.0, "ymax": 8.0, "zmax": 2.5}, CameraMapParams())
        r, k = cmap.p.res, cmap.slice_index(1.0)
        band = slice(k - 3, k + 4)
        iw = int(8.0 / r)
        cmap.seen[:iw + 1, :, band] = True
        cmap.logodds[iw, :, band] = 2.0
        cmap.logodds[iw, int(3.4 / r):int(4.6 / r), band] = -1.0
        cmap.seen[int(3.0 / r):int(5.0 / r), int(6.8 / r):, band] = False
        cmap.logodds[int(3.0 / r) - 1, int(6.7 / r):, band] = 2.0
        cmap.logodds[int(5.0 / r), int(6.7 / r):, band] = 2.0
        plan = cmap.planning_map(0.0)
        old = cmap.frontier_viewpoint(2.0, 4.0, plan, 1.0, **self.VIEW)
        self.assertGreater(old[1], 6.0, "예전 점수(경계 칸 수, 거리 3 m 할인)는 가까운 구석을 고른다")
        new = cmap.frontier_viewpoint(2.0, 4.0, plan, 1.0, **self.VIEW, **self.GAIN)
        self.assertIsNotNone(new)
        self.assertGreater(new[0], 7.0, "새로 보일 넓이로 세면 트인 곳 앞을 고른다")
        self.assertLess(abs(new[1] - 4.0), 0.7)
        self.assertGreater(new[4], 3.0, "트인 곳 너머 3 m 안이 새로 보인다")

    def test_view_gain_skips_spot_with_nothing_new_to_see(self):
        # 다 본 방 가운데 0.5 m 얼룩만 못 봤다: 경계 칸은 90개가 넘지만 보러 가도 새로 보일 것이 0.3 m²도 안 된다
        cmap = CameraMap({"xmin": 0.0, "ymin": 0.0, "xmax": 6.0, "ymax": 4.0, "zmax": 2.5}, CameraMapParams())
        r, k = cmap.p.res, cmap.slice_index(1.0)
        band = slice(k - 3, k + 4)
        cmap.seen[:, :, band] = True
        cmap.seen[int(4.0 / r):int(4.5 / r), int(2.0 / r):int(2.5 / r), band] = False
        plan = cmap.planning_map(0.0)
        self.assertIsNotNone(cmap.frontier_viewpoint(1.0, 2.0, plan, 1.0, **self.VIEW), "예전 점수는 그 얼룩을 보러 간다")
        self.assertIsNone(cmap.frontier_viewpoint(1.0, 2.0, plan, 1.0, **self.VIEW, **self.GAIN))

    def test_exclude_radius_per_entry(self):
        # 포기한 볼 자리는 둘레 1.5 m를 다시 고르지 않는다 (가는 길이 없던 자리는 예전처럼 0.6 m)
        cmap = CameraMap({"xmin": 0.0, "ymin": 0.0, "xmax": 10.0, "ymax": 4.0, "zmax": 2.5}, CameraMapParams())
        i0, i1 = int(2.0 / cmap.p.res), int(8.0 / cmap.p.res)
        k = cmap.slice_index(1.0)
        cmap.seen[i0:i1, :, k - 1:k + 2] = True
        plan = cmap.planning_map(0.0)
        first = cmap.frontier_viewpoint(5.0, 2.0, plan, 1.0, heading=0.0)
        near = cmap.frontier_viewpoint(5.0, 2.0, plan, 1.0, heading=0.0, exclude=[(first[0], first[1])])
        self.assertGreater(near[0], 5.0, "0.6 m만 빼면 같은 경계의 옆 칸을 다시 고른다")
        far = cmap.frontier_viewpoint(5.0, 2.0, plan, 1.0, heading=0.0, exclude=[(first[0], first[1], 1.5)])
        self.assertGreaterEqual(math.hypot(far[0] - first[0], far[1] - first[1]), 1.5 - 1e-6)

    PASS = dict(shadow=0.3, passage=0.3, passage_area=1.0, reach_frontier=True, clearance=0.45, standoff=1.2, z_top=1.85)

    @staticmethod
    def _wall_room(gap=None, window=False, close_windows=True):
        """8×6 m: x < 3 본 빈 곳, x = 3 벽(전체 높이). gap: y = 3 가운데 그 폭의 틈(전체 높이).
        window: 벽 y 2–4가 창턱(0–0.9 m)·위틀(2.1 m부터)이고 그 사이는 유리라 안 보임 — 깊이가 유리 너머 x 3–5를 빈 곳으로 칠했다."""
        r = 0.1
        c = CameraMap({"xmin": 0.0, "ymin": 0.0, "xmax": 8.0, "ymax": 6.0, "zmax": 2.5}, CameraMapParams(close_windows=close_windows))
        iw = int(3.0 / r)
        c.seen[:iw + 1, :, :] = True
        c.logodds[:iw, :, :] = -1.0
        c.logodds[iw, :, :] = 2.0
        if gap:
            c.logodds[iw, int(round((3.0 - gap / 2) / r)):int(round((3.0 + gap / 2) / r)), :] = -1.0
        if window:
            j0, j1, k0, k1 = int(2.0 / r), int(4.0 / r), c.slice_index(0.9), c.slice_index(2.1)
            c.logodds[iw, j0:j1, k0:k1] = -1.0
            c.seen[iw + 1:int(5.0 / r), j0:j1, k0:k1] = True
            c.logodds[iw + 1:int(5.0 / r), j0:j1, k0:k1] = -1.0
        return c

    def test_passage_over_30cm_is_a_frontier_smaller_hole_is_not(self):
        # 11/04 사용자: 30×30 cm 넘는 통로가 있으면 그 너머를 본다. 그늘(0.3 m)만 쓰면 0.6 m보다 좁은 통로는 경계가 아니었다
        wide = self._wall_room(gap=0.5)
        plan = wide.planning_map(0.0)
        self.assertIsNone(wide.frontier_viewpoint(1.5, 3.0, plan, 1.0, shadow=0.3, clearance=0.45, standoff=1.2),
                          "예전 규칙은 0.5 m 틈을 놓친다")
        vp = wide.frontier_viewpoint(1.5, 3.0, plan, 1.0, **self.PASS)
        self.assertIsNotNone(vp)
        self.assertLess(vp[0], 3.0)
        self.assertLess(abs(vp[1] - 3.0), 0.5, "틈 앞에서 본다")
        self.assertEqual(wide.frontier_census(1.5, 3.0, plan, 1.0)["passages"], 1)
        hole = self._wall_room(gap=0.2)
        plan = hole.planning_map(0.0)
        self.assertIsNone(hole.frontier_viewpoint(1.5, 3.0, plan, 1.0, **self.PASS), "0.2 m 구멍 너머는 경계가 아니다")
        census = hole.frontier_census(1.5, 3.0, plan, 1.0)
        self.assertEqual(census["passages"], 0)
        self.assertGreaterEqual(census["small"], 1)

    def test_window_beyond_sill_is_closed(self):
        # 11/04 10:17: 창턱 위 유리 너머를 깊이가 빈 곳으로 칠했고, 창턱을 넘으면 닿는다고 보고 그 끝을 보러 갔다
        before = self._wall_room(window=True, close_windows=False)
        old = before.frontier_viewpoint(1.5, 3.0, before.planning_map(0.0), 1.0, shadow=0.3, clearance=0.45, standoff=1.2, z_top=1.85)
        self.assertIsNotNone(old, "예전 규칙(가구 위로 넘어 거리 재기)은 창 너머 끝을 고른다")
        self.assertGreater(old[0], 3.0)
        c = self._wall_room(window=True)
        sill = c.window_sills()
        self.assertTrue(sill[30, 25] and not sill[30, 10], "창 아래 벽만 창턱, 창 옆 벽(가운데도 막힘)은 아니다")
        plan = c.planning_map(0.0)
        self.assertTrue(plan.occ[30, 30, :].all(), "계획 지도는 창턱 기둥을 위아래 전체 막는다 (창턱 위 1.5–1.85 m로 지나지 않게)")
        self.assertIsNone(c.frontier_viewpoint(1.5, 3.0, plan, 1.0, **self.PASS), "닫힌 창 너머는 막힌 것")
        census = c.frontier_census(1.5, 3.0, plan, 1.0, z_top=1.85)
        self.assertEqual((census["passages"], census["beyond"]), (0, 1))
        self.assertIsNone(c.blocked_frontier_approach(1.5, 3.0, plan, 1.0, shadow=0.3, z_top=1.85),
                          "막힌 경계 다가가 보기도 창 너머는 하지 않는다")
        self.assertIsNone(c.blocked_frontier_approach(1.5, 3.0, plan, 1.0, shadow=0.3))
        door = self._wall_room(gap=0.6, window=True)                # 창 옆에 문이 있으면 그 너머는 닿는 곳
        self.assertIsNotNone(door.frontier_viewpoint(1.5, 3.0, door.planning_map(0.0), 1.0, **self.PASS))

    @staticmethod
    def _desk_row(duct=False):
        """8×6 m: x < 5 본 빈 곳, x ≥ 5 모름. x 2.5–3.0에 벽에서 벽까지 책상 줄(0–0.8 m) — 비행 높이로는 못 지나고 위로 넘는다.
        duct: 책상 줄 바로 위 1.9–2.3 m에 덕트(높은 막힌 칸)가 있어 창턱처럼 보인다."""
        r = 0.1
        c = CameraMap({"xmin": 0.0, "ymin": 0.0, "xmax": 8.0, "ymax": 6.0, "zmax": 2.5}, CameraMapParams())
        c.seen[:int(5.0 / r), :, :] = True
        c.logodds[:int(5.0 / r), :, :] = -1.0
        c.logodds[int(2.5 / r):int(3.0 / r), :, :c.slice_index(0.8)] = 2.0
        if duct:
            c.logodds[26:29, :, 19:23] = 2.0   # x 2.6–2.9 m, 1.9–2.3 m
        return c

    def test_furniture_is_still_flown_over(self):
        # 11/04 10:17 실기체는 떠 있던 시간의 77%를 1.2 m 위에서(가구 위로) 날았다. 비행 높이로만 닿는 경계를 고르게 했더니
        # 기록 재생의 판단 4번 중 3번에서 볼 자리가 없었다 → 가구 위로는 그대로 넘는다
        c = self._desk_row()
        plan = c.planning_map(0.0)
        flat = dict(self.PASS, z_top=None)
        self.assertIsNone(c.frontier_viewpoint(1.5, 3.0, plan, 1.0, **flat), "비행 높이로만 재면 책상 줄 너머를 못 고른다")
        vp = c.frontier_viewpoint(1.5, 3.0, plan, 1.0, **self.PASS)
        self.assertIsNotNone(vp)
        self.assertGreater(vp[0], 3.0)
        self.assertEqual(c.frontier_census(1.5, 3.0, plan, 1.0, z_top=1.85)["passages"], 1)

    def test_eroded_tof_band_is_not_flown_over(self):
        # 11/11 Gazebo 복도: ToF가 찍은 띠(비행 높이 ±0.35 m)의 위 칸을 카메라 광선이 먼저 지워 0.8–1.3 m만 남자 '낮은 가구'로 보고
        # 1.85 m로 넘었다(실제로는 잘못 찍힌 띠였지만, 유리·흰 벽이면 들이받는다). ToF는 꼭대기를 본 것이 아니다
        c = CameraMap({"xmin": 0.0, "ymin": 0.0, "xmax": 8.0, "ymax": 6.0, "zmax": 2.5}, CameraMapParams())
        c.seen[:] = True
        c.logodds[:] = -1.0
        c.mark_hard(np.array([(2.55, y, z) for y in np.arange(2.05, 4.0, 0.1) for z in np.arange(0.65, 1.36, 0.1)]), 1.5)
        self.assertTrue(c.planning_map(0.0).occupied_at((2.55, 3.0, 1.85)), "띠 꼭대기 1.4 m + 0.6 m")
        c.logodds[:, :, c.slice_index(1.05):] = -1.0   # 위 칸부터 지워져 0.6–1.0 m만 남았다
        plan = c.planning_map(0.0)
        self.assertTrue(plan.occupied_at((2.55, 3.0, 1.0)))
        self.assertTrue(plan.occupied_at((2.55, 3.0, 1.85)), "찍힌 칸이 남아 있으면 그 위로 넘지 않는다")
        c.logodds[:] = -1.0                            # 다 지워지면 비행 높이도 위도 같이 열린다
        plan = c.planning_map(0.0)
        self.assertFalse(plan.occupied_at((2.55, 3.0, 1.0)))
        self.assertFalse(plan.occupied_at((2.55, 3.0, 1.85)))

    def test_flown_over_sill_is_not_a_window(self):
        # 높은 덕트 아래 책상 줄은 창턱과 모양이 같다. 이미 그 위로 지나왔다면 닫힌 창일 수 없다
        c = self._desk_row(duct=True)
        plan = c.planning_map(0.0)
        self.assertIsNone(c.frontier_viewpoint(1.5, 3.0, plan, 1.0, **self.PASS), "덕트 아래 책상 줄은 창턱으로 본다")
        c.mark_free_sphere((2.75, 3.0, 1.6))
        plan = c.planning_map(0.0)
        vp = c.frontier_viewpoint(1.5, 3.0, plan, 1.0, **self.PASS)
        self.assertIsNotNone(vp, "지나온 자리 0.5 m 안은 창턱이 아니다")
        self.assertGreater(vp[0], 3.0)

    def test_max_dist_keeps_only_reachable_in_time(self):
        # 11/04 12:57: 탐색 98초째(남은 22초)에 10.4 m 밖을 골랐다. 남은 시간 안에 닿을 거리 안에서만 고른다
        cmap = CameraMap({"xmin": 0.0, "ymin": 0.0, "xmax": 12.0, "ymax": 4.0, "zmax": 2.5}, CameraMapParams())
        k = cmap.slice_index(1.0)
        cmap.seen[int(1.0 / cmap.p.res):int(11.0 / cmap.p.res), :, k - 1:k + 2] = True   # x 1–11 m를 봤다, 양 끝 모름
        plan = cmap.planning_map(0.0)
        far = cmap.frontier_viewpoint(7.0, 2.0, plan, 1.0, heading=math.pi)   # −x로 가던 중: 5–6 m 밖 −x 경계를 고른다
        self.assertLess(far[0], 3.0)
        self.assertGreater(far[3], 4.0)
        near = cmap.frontier_viewpoint(7.0, 2.0, plan, 1.0, heading=math.pi, max_dist=4.0)
        self.assertIsNotNone(near)
        self.assertGreater(near[0], 9.0, "4 m 안의 +x 경계만 남는다")
        self.assertLessEqual(near[3], 4.0)
        self.assertIsNone(cmap.frontier_viewpoint(5.0, 2.0, plan, 1.0, max_dist=1.0), "1 m 안에는 볼 곳이 없다")

    def test_forget_behind_clears_only_seen_free_cells_in_wedge(self):
        # 단안 깊이가 흰 벽을 꿰뚫어 칠한 '본 빈 칸'만 모르는 칸으로 되돌린다. 막힌 칸·ToF 칸·지나간 칸·부채꼴 밖은 그대로
        cmap = CameraMap({"xmin": 0.0, "ymin": 0.0, "xmax": 6.0, "ymax": 4.0, "zmax": 2.5}, CameraMapParams())
        r, k = cmap.p.res, cmap.slice_index(1.0)
        cmap.seen[:, :, :] = True
        cmap.logodds[:, :, :] = -1.0
        cmap.logodds[int(3.5 / r), int(2.0 / r), k] = 2.0                       # 막힌 칸
        cmap.mark_hard([(3.0, 2.05, 1.0)])                                       # ToF로 찍은 칸
        cmap.flown[int(4.0 / r), int(2.0 / r), k] = True                         # 지나간 칸
        n = cmap.forget_behind((1.0, 2.0, 1.0), 0.0, 1.2, 4.0, math.radians(17.5), 0.5, 1.5)
        self.assertGreater(n, 0)
        self.assertFalse(cmap.seen[int(2.5 / r), int(2.05 / r), k], "ToF 면 뒤 정면은 모르는 칸으로")
        self.assertTrue(cmap.seen[int(1.8 / r), int(2.05 / r), k], "ToF 면 앞은 그대로")
        self.assertTrue(cmap.seen[int(2.5 / r), int(3.5 / r), k], "부채꼴 밖은 그대로")
        self.assertTrue(cmap.seen[int(2.5 / r), int(2.05 / r), cmap.slice_index(2.0)], "높이 띠 밖은 그대로")
        self.assertGreater(cmap.logodds[int(3.5 / r), int(2.0 / r), k], cmap.p.occupied, "막힌 칸은 그대로")
        self.assertTrue(cmap.hard[int(3.0 / r), int(2.05 / r), k])
        self.assertTrue(cmap.seen[int(4.0 / r), int(2.0 / r), k], "지나간 칸은 그대로")

    def test_flown_path_stays_open_unless_tof_sees_something_there_again(self):
        # 지나온 길은 열어 둔다(카메라가 아래 탁자를 봐도 돌아갈 길이 남게). 지나간 뒤 그 자리에 ToF가 막힘을 찍으면(사람·의자가
        # 들어옴) 그 칸과 둘레 여유는 막는다 (11/04 외부 리뷰: 지나온 길을 늘 비우면 새 장애물을 못 본다)
        cmap = CameraMap({"xmin": 0.0, "ymin": 0.0, "xmax": 6.0, "ymax": 4.0, "zmax": 2.5}, CameraMapParams())
        cmap.seen[:, :, :] = True
        cmap.logodds[:, :, :] = -1.0
        for x in np.arange(1.0, 5.0, 0.2):
            cmap.mark_free_sphere((x, 2.0, 1.0))
        cmap.logodds[int(2.0 / 0.1), int(2.0 / 0.1), cmap.slice_index(1.0)] = 2.0   # 지나간 뒤 단안 깊이가 찍은 막힘 (잡음일 수 있다)
        plan = cmap.planning_map(0.4)
        self.assertFalse(plan.occupied_at((2.0, 2.0, 1.0)), "단안 깊이 막힘은 지나온 칸을 못 덮는다")
        self.assertFalse(plan.occupied_at((3.0, 2.0, 1.0)), "지나온 칸은 열림")
        cmap.mark_hard([(3.0, 2.0, 1.0)])                                      # 지나간 뒤 ToF가 그 자리에 무엇을 잼
        plan = cmap.planning_map(0.4)
        self.assertTrue(plan.occupied_at((3.0, 2.0, 1.0)))
        self.assertTrue(plan.occupied_at((3.3, 2.0, 1.0)), "둘레 여유(0.4 m)도 막는다")
        self.assertFalse(plan.occupied_at((4.5, 2.0, 1.0)), "먼 지나온 칸은 그대로 열림")

    def test_tof_surface_is_not_flown_anymore(self):
        # 11/05 13:04 실기체: 위치가 틀어져 예전에 지나온 칸이 지금의 문짝·칸막이 자리와 겹쳤다. ToF가 그 면을 잰 자리는 '지나온 칸'에서
        # 빼야 막힌 칸(ToF 한 점의 단안 막힘 포함)과 찾은 장애물 상자가 경로를 막는다
        cmap = CameraMap({"xmin": 0.0, "ymin": 0.0, "xmax": 6.0, "ymax": 4.0, "zmax": 2.5}, CameraMapParams())
        cmap.seen[:, :, :] = True
        cmap.logodds[:, :, :] = -1.0
        for x in np.arange(1.0, 5.0, 0.2):
            cmap.mark_free_sphere((x, 2.0, 1.0))
        for _ in range(3):                                           # ToF가 앞 0.7 m(3.0, 2.0)에 면을 잼 (초당 5번, 0.6초)
            cmap.integrate_ray((2.3, 2.0, 1.0), 0.0, 0.7, 1.2)
        self.assertFalse(cmap.planning_map(0.2).occupied_at((3.0, 2.0, 1.0)), "그대로면 지나온 칸이 덮는다")
        n = cmap.unfly((3.0, 2.0), 0.3, 0.65, 1.35)
        self.assertGreater(n, 0)
        plan = cmap.planning_map(0.2)
        self.assertTrue(plan.occupied_at((3.0, 2.0, 1.0)))
        self.assertTrue(plan.occupied_at((2.85, 2.0, 1.0)), "둘레 여유(0.2 m)도")
        self.assertFalse(plan.occupied_at((4.5, 2.0, 1.0)), "먼 지나온 칸은 그대로")
        self.assertTrue(cmap.flown[int(4.5 / 0.1), int(2.0 / 0.1), cmap.slice_index(1.0)])

    def test_known_free_at(self):
        # 물러나기 전 뒤 확인(11/09 13:03): 본 빈 칸·지나온 칸만 '아는 빈 곳'. 모르는 칸·막힌 칸·지도 밖은 아니다
        cmap = CameraMap({"xmin": 0.0, "ymin": 0.0, "xmax": 6.0, "ymax": 4.0, "zmax": 2.5}, CameraMapParams())
        self.assertFalse(cmap.known_free_at((2.0, 2.0, 1.0)), "아무것도 안 본 지도")
        cmap.mark_free_sphere((2.0, 2.0, 1.0))
        self.assertTrue(cmap.known_free_at((2.0, 2.0, 1.0)), "지나온 칸")
        cmap.integrate_ray((3.0, 1.0, 1.0), 0.0, 1.0, 1.2)            # (3.0–4.0, 1.0)을 비우고 (4.0, 1.0)에 면
        self.assertTrue(cmap.known_free_at((3.5, 1.0, 1.0)), "광선이 지나간 빈 칸")
        self.assertFalse(cmap.known_free_at((4.0, 1.0, 1.0)), "면을 잰 칸")
        self.assertFalse(cmap.known_free_at((3.5, 3.0, 1.0)), "본 적 없는 칸")
        self.assertFalse(cmap.known_free_at((-0.5, 1.0, 1.0)), "지도 밖")

    def test_flown_at_is_stricter_than_seen(self):
        # 돈 뒤의 물러나기(11/16 Gazebo 방 두 개): 카메라가 '본 빈 칸'은 문틀을 놓칠 수 있다. 실제로 지나온 칸만 따로 묻는다
        cmap = CameraMap({"xmin": 0.0, "ymin": 0.0, "xmax": 6.0, "ymax": 4.0, "zmax": 2.5}, CameraMapParams())
        cmap.mark_free_sphere((2.0, 2.0, 1.0))
        cmap.integrate_ray((3.0, 1.0, 1.0), 0.0, 1.0, 1.2)
        self.assertTrue(cmap.flown_at((2.0, 2.0, 1.0)), "지나온 칸")
        self.assertTrue(cmap.flown_at((2.15, 2.15, 1.0)), "지나온 자리 둘레 (기체 크기만큼)")
        self.assertFalse(cmap.flown_at((2.45, 2.0, 1.0)), "지나온 자리에서 0.4 m 넘게 옆")
        self.assertTrue(cmap.known_free_at((3.5, 1.0, 1.0)))
        self.assertFalse(cmap.flown_at((3.5, 1.0, 1.0)), "광선이 지나갔을 뿐인 칸")
        self.assertFalse(cmap.flown_at((-0.5, 1.0, 1.0)), "지도 밖")
        cmap.mark_hard([(2.0, 2.0, 1.0)])                             # 지나온 뒤 ToF가 그 칸에 면을 쟀다
        self.assertFalse(cmap.flown_at((2.0, 2.0, 1.0)), "지나온 뒤 막힘으로 확정된 칸")

    def test_planning_inflates_horizontal_disk_plus_one_cell_up_down(self):
        # 11/03 속도: 9×9×9 공으로 100만 칸을 부풀리면 121–139 ms라, 수평 원반으로 부풀린 뒤 위아래 한 칸(0.1 m, 기체 높이)만
        # 더한다(9 ms). 수직 여유 0.4→0.1 m는 의도한 것: 가구 위는 기둥 막기·over_clear 규칙이 따로 막는다
        from scipy import ndimage
        cmap = CameraMap({"xmin": 0.0, "ymin": 0.0, "xmax": 6.0, "ymax": 4.0, "zmax": 2.5}, CameraMapParams())
        cmap.seen[:] = True                                        # 모르는 칸 없음
        i, j, k = int(3.0 / cmap.p.res), int(2.0 / cmap.p.res), cmap.slice_index(1.8)   # 기둥 높이(0.3–1.3 m) 밖의 막힌 칸 하나
        cmap.logodds[i, j, k] = 2.0
        occ = cmap.planning_map(0.4).occ                           # 반지름 4칸
        self.assertTrue(occ[i + 4, j, k] and occ[i, j - 4, k] and occ[i + 3, j + 2, k])   # 수평 원반: 3² + 2² ≤ 4²
        self.assertFalse(occ[i + 3, j + 3, k])                     # 3² + 3² > 4²: 원반 밖
        self.assertTrue(occ[i, j, k + 1] and occ[i + 4, j, k - 1])  # 위아래 한 칸은 원반 그대로
        self.assertFalse(occ[i, j, k + 2] or occ[i, j, k - 2])     # 두 칸 위아래는 막지 않는다 (공이었을 때는 막았다)
        self.assertEqual(int(occ.sum()), 3 * 49)                   # 반지름 4 원반 49칸 × 3단
        # 2D로 쓰는 호출(mask[:, :, None]·2D 배열)은 예전처럼 원반으로만 부풀린다
        rng = np.random.default_rng(1)
        mask = rng.random((40, 30)) < 0.03
        yy, xx = np.mgrid[-3:4, -3:4]
        ref = ndimage.binary_dilation(mask, structure=xx * xx + yy * yy <= 9)
        np.testing.assert_array_equal(_dilate(mask[:, :, None], 3)[:, :, 0], ref)
        np.testing.assert_array_equal(_dilate(mask, 3), ref)

    def test_standoff_from_cropped_edt_picks_same_spot_as_disk_dilation(self):
        # 11/03 속도: 경계 무리마다 25×25로 부풀리는(10 ms) 대신 무리를 ±standoff 상자로 잘라 EDT 거리로 'standoff 안'을 본다.
        # 거리 ≤ 반지름인 칸은 원반으로 부풀린 칸과 같은 집합이라 고르는 자리도 같다
        rng = np.random.default_rng(5)
        for _ in range(3):
            mask = np.zeros((50, 40), dtype=bool)
            i0, j0 = rng.integers(5, 45), rng.integers(5, 35)
            mask[i0:i0 + rng.integers(1, 6), j0:j0 + rng.integers(1, 6)] = True
            mask[rng.integers(0, 50, 4), rng.integers(0, 40, 4)] = True
            box, gap, (ti, tj) = _near_cluster(mask, 12)
            near = np.zeros_like(mask)
            near[box] = gap <= 12
            np.testing.assert_array_equal(near, _dilate(mask[:, :, None], 12)[:, :, 0])
            ii, jj = np.mgrid[box]
            self.assertTrue(mask[ti + box[0].start, tj + box[1].start].all())          # 가리키는 칸은 무리 칸
            np.testing.assert_allclose(gap, np.hypot(ii - (ti + box[0].start), jj - (tj + box[1].start)))   # 그 칸까지 거리
        # 문 장면(test_door_frontier_is_viewed_from_roomy_spot_in_front)과 문 둘·기둥 장면: 옛 부풀리기 코드가 고른 자리 그대로
        def scene(doors, seen_y=4.0, pillar=False):
            cmap = CameraMap({"xmin": 0.0, "ymin": 0.0, "xmax": 6.0, "ymax": 4.0, "zmax": 2.5}, CameraMapParams())
            r, k = cmap.p.res, cmap.slice_index(1.0)
            band, iw = slice(k - 3, k + 4), int(3.0 / r)
            cmap.seen[:iw + 1, :int(seen_y / r), band] = True
            cmap.logodds[iw, :, band] = 2.0
            for y0, y1 in doors:
                cmap.logodds[iw, int(y0 / r):int(y1 / r), band] = -1.0
            if pillar:
                cmap.logodds[int(1.5 / r):int(1.7 / r), int(1.9 / r):int(2.3 / r), band] = 2.0
            return cmap
        kw = dict(shadow=0.3, clearance=0.45, standoff=1.2)
        for cmap, pose, expect in ((scene([(1.6, 2.4)]), (1.0, 2.0), (2.75, 1.95, 19, 1.7414)),
                                   (scene([(0.8, 1.4), (2.6, 3.4)], seen_y=3.6, pillar=True), (0.8, 2.0), (0.85, 3.25, 106, 1.2))):
            vp = cmap.frontier_viewpoint(pose[0], pose[1], cmap.planning_map(0.0), 1.0, **kw)
            self.assertEqual(vp[2], expect[2])
            np.testing.assert_allclose((vp[0], vp[1], vp[3]), (expect[0], expect[1], expect[3]), atol=1e-9)

    def test_hard_cells_from_tof_survive_camera_misses(self):
        # ToF로 확인한 벽(단단한 칸)은 카메라 광선이 꿰뚫어도 보통 칸의 1/4로만 지워진다
        cmap = CameraMap({"xmin": 0.0, "ymin": 0.0, "xmax": 6.0, "ymax": 4.0, "zmax": 2.5}, CameraMapParams())
        cam = np.array([0.5, 2.0, 1.0])
        pts = np.array([[2.0, y, 1.0] for y in np.arange(1.6, 2.41, 0.05)])
        n = cmap.mark_hard(pts, 1.5)
        self.assertGreaterEqual(n, 8)
        self.assertTrue(cmap.planning_map(0.0).occupied_at((2.0, 2.0, 1.0)))
        cmap.logodds[int(2.0 / cmap.p.res), int(3.0 / cmap.p.res), cmap.slice_index(1.0)] = cmap.p.occupied + 0.3   # 보통 막힘 칸
        unit = np.array([[1.0, 0.0, 0.0], [math.cos(math.atan2(1.0, 1.5)), math.sin(math.atan2(1.0, 1.5)), 0.0]])
        for _ in range(4):   # 두 칸 다 꿰뚫는 먼 광선 네 장면
            cmap._integrate_rays(cam, unit, np.array([5.0, 5.0]), max_range=5.5)
        hard_cell = cmap.logodds[int(2.0 / cmap.p.res), int(2.0 / cmap.p.res), cmap.slice_index(1.0)]
        soft_cell = cmap.logodds[int(2.0 / cmap.p.res), int(3.0 / cmap.p.res), cmap.slice_index(1.0)]
        self.assertGreater(hard_cell, cmap.p.suspect)
        self.assertLess(soft_cell, cmap.p.suspect)

    def test_sight_line_sees_body_and_seen_ratio(self):
        # 찾은 사람이 아직 있는지: 그 방향 구간에 막힌 칸(사람 몸)이 있나, 그 자리를 본 적 있나
        cmap = CameraMap({"xmin": 0.0, "ymin": 0.0, "xmax": 6.0, "ymax": 4.0, "zmax": 2.5}, CameraMapParams())
        cam = (0.5, 2.0, 1.0)
        self.assertEqual(cmap.sight_line(cam, (3.5, 2.0), 2.4, 4.0, 1.0), (False, 0.0))   # 아직 아무것도 못 봄
        i, j = int(3.6 / cmap.p.res), int(2.0 / cmap.p.res)
        k0, k1 = cmap.slice_index(0.7), cmap.slice_index(1.3)
        cmap.seen[:, :, :] = True
        cmap.logodds[i, j, k0:k1 + 1] = 0.9      # 3.6 m 자리에 사람 몸
        blocked, seen = cmap.sight_line(cam, (3.5, 2.0), 2.4, 4.0, 1.0)
        self.assertTrue(blocked)
        self.assertEqual(seen, 1.0)
        self.assertFalse(cmap.sight_line(cam, (3.5, 2.0), 0.3, 2.4, 1.0)[0])      # 앞쪽은 비어 있다
        self.assertFalse(cmap.sight_line(cam, (3.5, 3.5), 2.4, 4.0, 1.0)[0])      # 다른 방향은 비어 있다

    def test_best_look_prefers_small_turn(self):
        cmap = CameraMap(BOUNDS, PARAMS)                          # 아무것도 안 봄: 모든 방향이 똑같이 모름
        yaw, _ = cmap.best_look(3.0, 2.0, 1.0, HFOV, now_yaw=0.5)
        self.assertLess(abs(yaw - 0.5), 0.8, "모르는 칸 수가 같으면 지금 보는 쪽에 가까운 방향")

    def test_blurred_door_edge_leaves_no_phantom(self):
        # x = 3 칸막이에 폭 1 m 문(y 1.5–2.5). 단안 깊이처럼 가로로 번지게 하면 문틀과 먼 벽(4.9 m, 믿는 거리
        # 밖) 사이 화소가 중간 거리를 내서, 문틀 바로 뒤 빈 곳에 막힘이 찍힌다. 경계 거르기를 켜면 찍히지 않아야 한다
        from scipy import ndimage
        world = _world(_box(3.0, 0.75, 1.25, 0.1, 1.5, 2.5), _box(3.0, 3.25, 1.25, 0.1, 1.5, 2.5))
        cam = (1.0, 2.0, 1.0)
        depth = render_depth(world, cam, 0.0)
        blurred = ndimage.uniform_filter1d(np.where(np.isfinite(depth), depth, 8.0), 5, axis=1)
        phantom = (slice(32, 56), slice(1, 39), slice(5, 15))    # 칸막이 너머 x 3.2–5.6, 높이 0.5–1.5 (실제로 빈 곳)
        for ratio, expect in ((0.0, True), (PARAMS.edge_ratio, False)):
            cmap = CameraMap(BOUNDS, CameraMapParams(stride=2, hit_far=0.85, edge_ratio=ratio))
            cmap.integrate_depth(blurred, cam, 0.0, HFOV)
            self.assertEqual(bool(cmap.occupied()[phantom].any()), expect, f"edge_ratio {ratio}")
            self.assertTrue(cmap.occupied()[29:31, 5:14, 5:15].any(), "문틀 옆 칸막이는 찍힌다")
        # 문틀(칸막이 끝, y 1.5·2.5)은 늘 경계 화소다. 경계의 가까운 쪽 화소를 찍어야 문틀이 지도에 남는다
        # (시험 영상은 2화소마다 광선을 쏴서 끝 화소가 빠질 수 있으므로 끝에서 한 칸 안까지 본다)
        for hit in (False, True):
            cm = CameraMap(BOUNDS, CameraMapParams(stride=2, hit_far=0.85, edge_hit=hit))
            cm.integrate_depth(blurred, cam, 0.0, HFOV)
            occ = cm.occupied()[29:31, :, 5:15].any(axis=(0, 2))
            if hit:
                self.assertTrue(occ[13:15].any() and occ[25:27].any(), "문틀 양쪽 끝이 찍힌다")
                self.assertFalse(cm.occupied()[phantom].any(), "가까운 쪽 화소만 찍으므로 헛 장애물은 없다")

    def test_top_below_reports_mapped_furniture_under_drone(self):
        # 11/09 17:07: 하방 ToF는 바로 밑 물건까지 거리다. 지도가 아는 기체 밑 가구 꼭대기를 위치 추정에 알려 높이 = ToF + 꼭대기로
        world = _world(_box(3.0, 2.0, 0.35, 0.8, 0.8, 0.7))     # 꼭대기 0.7 m 조형물, 방 가운데
        cm = CameraMap(BOUNDS, PARAMS)
        for cam, yaw in (((1.0, 2.0, 1.0), 0.0), ((5.0, 2.0, 1.0), math.pi), ((3.0, 0.5, 1.0), math.pi / 2)):
            cm.integrate_depth(render_depth(world, cam, yaw), cam, yaw, HFOV)   # 세 방향에서 보고 지도에 넣는다
        top = cm.top_below(2.65, 2.0, 1.3 - 0.2)             # 상자 옆면 위 (카메라는 면만 찍으므로 속은 비어 있다)
        self.assertIsNotNone(top)
        self.assertAlmostEqual(top, 0.7, delta=0.15)
        self.assertIsNone(cm.top_below(1.5, 2.0, 0.8), "빈 바닥 위에는 없다")
        self.assertIsNone(cm.top_below(2.65, 2.0, 0.45), "주어진 높이보다 위의 칸은 세지 않는다 (0.3–0.45 m는 한 칸)")

    def test_floor_read_short_from_high_camera_is_not_obstacle(self):
        # 11/11 09:32 실기체: 1.4 m로 날 때 단안 깊이가 바닥을 35%쯤 가깝게 읽어 0.45–0.55 m에 가짜 바닥이 깔렸고, 기둥 막기가 그 위
        # 1.2 m까지 막아 복도를 1.0 m 높이로 못 지났다. 바닥 높이를 카메라 높이의 0.45배(0.6 m까지)로 보면 깔리지 않는다
        world = _world()
        cam = (1.0, 2.0, 1.4)
        short = render_depth(world, cam, 0.0) * 0.65              # 모두 35% 가깝게 읽음: 바닥이 0.49 m에 찍힌다
        k10 = int(1.0 / 0.1)
        seen = {}
        for frac in (0.0, 0.45):
            cm = CameraMap(BOUNDS, dataclasses.replace(PARAMS, floor_frac=frac))
            cm.integrate_depth(short, cam, 0.0, HFOV)
            ghost = int((cm.logodds[15:35, 15:25, 3:6] > cm.p.suspect).sum())      # x 1.5–3.5, y 1.5–2.5, 높이 0.3–0.6 m
            free = int((~cm.planning_map(0.2).occ[15:35, 15:25, k10]).sum())       # 그 위 1.0 m 높이로 갈 수 있는 칸
            seen[frac] = (ghost, free)
        self.assertGreater(seen[0.0][0], 20, "예전 규칙: 가짜 바닥이 깔린다")
        self.assertEqual(seen[0.45][0], 0, "새 규칙: 깔리지 않는다")
        self.assertGreater(seen[0.45][1], 2 * seen[0.0][1] + 20, f"새 규칙이 1.0 m 높이를 연다 {seen}")

    def test_scaled_down_depth_raises_the_floor_cutoff(self):
        # 실기체 11/06 14:35: 정면 ToF로 맞춘 배율 0.85–0.9를 곱하자 이미 35% 가깝게 읽힌 바닥이 더 가깝게 들어와, 1.0 m로 날아도
        # 카메라 높이의 0.45배 위에 가짜 바닥이 남았다. 곱한 배율을 알려 주면 곱하기 전 깊이 기준으로 바닥을 가른다
        world = _world()
        cam = (1.0, 2.0, 1.0)
        short = render_depth(world, cam, 0.0) * 0.65 * 0.82       # 바닥이 1 − 0.65 × 0.82 = 0.47 m에 찍힌다

        def ghost(scale):
            cm = CameraMap(BOUNDS, PARAMS)
            cm.integrate_depth(short, cam, 0.0, HFOV, **({} if scale is None else {"scale": scale}))
            return int((cm.logodds[15:35, 15:25, 3:6] > cm.p.suspect).sum())   # x 1.5–3.5, y 1.5–2.5, 높이 0.3–0.6 m

        self.assertGreater(ghost(None), 10, "배율을 모르면 0.45 m 위라 막힘으로 남는다")
        self.assertEqual(ghost(0.82), 0, "배율 0.82: 문턱이 카메라 높이의 1 − 0.55 × 0.82 = 0.55배")
        self.assertEqual(ghost(1.2), ghost(None), "1보다 큰 배율은 문턱을 낮추지 않는다")

    def test_desk_is_still_seen_from_high_camera(self):
        world = _world(_box(3.0, 2.0, 0.375, 1.0, 1.0, 0.75))    # 0.75 m 책상
        cam = (1.0, 2.0, 1.6)
        cm = CameraMap(BOUNDS, PARAMS)
        cm.integrate_depth(render_depth(world, cam, 0.0), cam, 0.0, HFOV)
        self.assertTrue(cm.occupied()[25:35, 15:25, 6:8].any(), "바닥 상한 0.6 m 위의 책상은 높이 떠서 봐도 막힘이다")

    def test_floor_scale_recovers_depth_scale(self):
        from perception.depth import floor_scale
        world = _world(_box(4.0, 1.0, 0.5, 0.6, 0.6, 1.0))        # 앞에 상자 하나 (바닥 일부를 가린다)
        cam = (1.0, 2.0, 1.0)
        depth = render_depth(world, cam, 0.0)
        self.assertAlmostEqual(floor_scale(depth, HFOV, 1.0), 1.0, delta=0.02)
        self.assertAlmostEqual(floor_scale(depth * 0.8, HFOV, 1.0), 1.25, delta=0.03, msg="짧게 추정하면 늘린다")
        self.assertAlmostEqual(floor_scale(depth * 1.2, HFOV, 1.0), 1 / 1.2, delta=0.03)
        wall = render_depth(world, (5.5, 2.0, 1.0), 0.0)           # 벽 0.4 m 앞: 바닥이 안 보인다
        self.assertIsNone(floor_scale(wall, HFOV, 1.0))

    def test_floor_is_not_obstacle(self):
        cmap = CameraMap(BOUNDS, PARAMS)
        world = _world()
        cmap.integrate_depth(render_depth(world, (1.0, 2.0, 1.0), 0.0), (1.0, 2.0, 1.0), 0.0, HFOV)
        occ = cmap.occupied()
        self.assertFalse(occ[:, :, :int(0.15 / 0.1)].any(), "바닥은 막힘으로 찍히지 않는다")
        self.assertTrue(occ[:, :, 5:15].any(), "앞 벽(x = 5.9)은 찍힌다")

    def test_planning_blocks_unknown_and_frontier_finishes(self):
        world = _world()
        cmap = CameraMap(BOUNDS, PARAMS)
        cam = (3.0, 2.0, 1.0)
        cmap.mark_free_sphere(cam)
        plan_map = cmap.planning_map(0.25)
        self.assertTrue(plan_map.occupied_at((5.0, 2.0, 1.0)), "아직 못 본 곳은 막힌 것으로 본다")
        cmap.integrate_depth(render_depth(world, cam, 0.0), cam, 0.0, HFOV)
        vp = cmap.frontier_viewpoint(3.0, 2.0, cmap.planning_map(0.25), 1.0)
        self.assertIsNotNone(vp, "옆과 뒤는 아직 모르므로 볼 자리가 있다")
        for k in range(1, 8):                                    # 제자리에서 둘러보면 방 전체가 보인다
            yaw = k * math.pi / 4
            cmap.integrate_depth(render_depth(world, cam, yaw), cam, yaw, HFOV)
        self.assertGreater(cmap.explored_area(1.0), 0.85 * 5.8 * 3.8)
        self.assertIsNone(cmap.frontier_viewpoint(3.0, 2.0, cmap.planning_map(0.25), 1.0))

    def test_best_look_points_to_unknown(self):
        world = _world()
        cmap = CameraMap(BOUNDS, PARAMS)
        cam = (3.0, 2.0, 1.0)
        for yaw in (0.0, math.pi / 2, -math.pi / 2):              # 서쪽만 아직 안 봤다
            cmap.integrate_depth(render_depth(world, cam, yaw), cam, yaw, HFOV)
        yaw, gain = cmap.best_look(3.0, 2.0, 1.0, HFOV)
        self.assertGreater(gain, 0)
        self.assertLess(abs(abs(yaw) - math.pi), 0.5, "서쪽을 봐야 한다")


if __name__ == "__main__":
    unittest.main()
