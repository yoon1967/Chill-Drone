"""가구 위로 넘기(11/03, 수직 공간 쓰기) 검사: 계획 지도의 기둥 막기, 위로 넘는 경로, 가구 위로 닿는 경계, ToF가 가구를 볼 때 높이.
실행: python -m unittest discover -s tests"""
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from autonomy import PlanError, plan, tello_state  # noqa: E402
from autonomy.camera_map import CameraMap, CameraMapParams  # noqa: E402
from autonomy.state_estimator import EstimatorParams, PoseEstimator  # noqa: E402

BOUNDS = {"xmin": 0, "ymin": 0, "xmax": 6, "ymax": 4, "zmax": 2.5}


def room(seen_top: float = 2.3, box_top: float = 0.7, seen_x: float = 6.0, params: CameraMapParams | None = None) -> CameraMap:
    """6×4 m 방: x < seen_x, 높이 0.3–seen_top m를 본 빈 칸으로. x 2.8–3.2에 폭 전체를 막는 높이 box_top 상자(본 것)."""
    c = CameraMap(BOUNDS, params)
    r = c.p.res
    k_lo, k_hi, i_seen = int(0.3 / r), int(seen_top / r), int(seen_x / r)
    c.seen[:i_seen, :, k_lo:k_hi] = True
    c.logodds[:i_seen, :, k_lo:k_hi] = -1.0
    i0, i1, kb = int(2.8 / r), int(3.2 / r), int(box_top / r)
    c.logodds[i0:i1, :, 0:kb] = 2.0
    c.seen[i0:i1, :, 0:kb] = True
    return c


class OverflyTest(unittest.TestCase):
    def test_low_furniture_blocks_only_up_to_seen_top_plus_clearance(self):
        pm = room().planning_map(0.2)
        self.assertTrue(pm.occupied_at((3.0, 2.0, 1.0)))     # 비행 높이에서는 막힘
        # 최소 1.2 m까지 막고 위로 한 칸(0.1 m)을 더한다. 11/03 속도 개선 전에는 부풀리기가 3D 공이라 반지름 0.2 m만큼(1.4 m까지)
        # 더 막았다; 이제 수직 여유는 기체 높이 한 칸이고 가구 위는 over_clear 규칙이 막는다
        self.assertTrue(pm.occupied_at((3.0, 2.0, 1.25)))
        self.assertFalse(pm.occupied_at((3.0, 2.0, 1.35)))
        self.assertFalse(pm.occupied_at((3.0, 2.0, 1.6)))    # 카메라가 빈 것으로 본 그 위는 지난다
        self.assertTrue(room(seen_top=1.1).planning_map(0.2).occupied_at((3.0, 2.0, 1.6)))   # 위를 못 봤으면 못 지난다
        self.assertTrue(room(box_top=1.7).planning_map(0.2).occupied_at((3.0, 2.0, 2.0)))    # 선 사람 키(1.7)면 2.2 m까지 막힘
        old = room(params=CameraMapParams(over_clear=0.0)).planning_map(0.2)                 # 예전 규칙: 기둥 전체
        self.assertTrue(old.occupied_at((3.0, 2.0, 1.6)))

    def test_planner_climbs_over_low_box_below_height_cap(self):
        path = plan(room().planning_map(0.2), (1.0, 2.0, 1.0), (5.0, 2.0, 1.0), z_max=1.85)
        top = max(p[2] for p in path)
        self.assertGreaterEqual(top, 1.3)                     # 막힌 1.3 m 바로 위 칸(1.35 m)으로 넘는다 (11/03: 부풀리기 수직 여유 한 칸)
        self.assertLessEqual(top, 1.85 + 1e-6)
        self.assertAlmostEqual(path[-1][2], 1.0)              # 넘은 뒤 비행 높이로 돌아온다
        with self.assertRaises(PlanError):                     # 상한이 막힌 높이(1.3 m 칸 아래)면 못 넘는다 (11/03 전에는 1.3이 막힘)
            plan(room().planning_map(0.2), (1.0, 2.0, 1.0), (5.0, 2.0, 1.0), z_max=1.25)

    def test_frontier_beyond_furniture_is_reachable_only_with_z_top(self):
        c = room(seen_x=4.5)                                   # x > 4.5는 모르는 곳: 상자 너머에만 경계가 있다
        pm = c.planning_map(0.2)
        self.assertIsNone(c.frontier_viewpoint(1.0, 2.0, pm, 1.0))
        vp = c.frontier_viewpoint(1.0, 2.0, pm, 1.0, z_top=1.85)
        self.assertIsNotNone(vp)
        self.assertGreater(vp[0], 3.2)

    def test_tof_over_furniture_is_not_trusted_unless_h_drops_too(self):
        est = PoseEstimator((1.0, 1.0), 0.0, 0.0, params=EstimatorParams(height_source="tof"))
        t = [0.0]

        def step(tof, h=40):
            t[0] += 0.1
            est.on_state(tello_state.to_si(tello_state.parse(f"yaw:0;vgz:0;h:{h};tof:{tof};")), t[0])
        for _ in range(5):
            step(100)
        for _ in range(80):                                    # 8초 동안 0.7 m 가구 위, h는 그대로: 높이 1.0 m 유지
            step(30)
        self.assertAlmostEqual(est.z, 1.0, places=2)
        for _ in range(12):                                    # 멀어지는 쪽(가구를 벗어남)은 1초면 믿는다
            step(100)
        self.assertAlmostEqual(est.z, 1.0, places=2)
        for _ in range(35):                                    # 정말 내려감(h도 0.7 m 내려감): 3.5초는 버티고
            step(30, h=-30)
        self.assertAlmostEqual(est.z, 1.0, places=2)
        for _ in range(8):                                     # 4.3초면 믿는다
            step(30, h=-30)
        self.assertAlmostEqual(est.z, 0.3, places=2)


if __name__ == "__main__":
    unittest.main()
