"""정답 위치 없이 위치 추정(Tello 상태값 + 지도 보정) 검사. 실행: python -m unittest discover -s tests"""
import math
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from autonomy import tello_state  # noqa: E402
from autonomy.state_estimator import EstimatorParams, PoseEstimator, occupancy_slice, tof_likelihood  # noqa: E402
from simdrone.tello_emu import StateSynth  # noqa: E402

FAN = np.linspace(-math.radians(12.5), math.radians(12.5), 7)


def _box(x, y, z, sx, sy, sz):
    return {"x": x, "y": y, "z": z, "sx": sx, "sy": sy, "sz": sz}


def _room(width, depth, *obstacles):
    t, h = 0.1, 2.5
    walls = [_box(width / 2, t / 2, h / 2, width, t, h), _box(width / 2, depth - t / 2, h / 2, width, t, h),
             _box(t / 2, depth / 2, h / 2, t, depth, h), _box(width - t / 2, depth / 2, h / 2, t, depth, h)]
    return {"bounds": {"xmin": 0, "ymin": 0, "xmax": width, "ymax": depth, "zmax": h},
            "walls": walls, "lintels": [], "obstacles": list(obstacles)}


def _state(yaw, vx, vy=0.0, h=1.0):
    """우리 좌표계 값 → Tello 문자열 → 다시 읽은 값 (실제처럼 정수로 반올림된다)."""
    raw = tello_state.encode(tello_state.from_si(yaw, vx, vy, 0.0, h, h, 90))
    return tello_state.to_si(tello_state.parse(raw))


def _true_range(occ, origin, res, x, y, yaw, max_range=1.2):
    est = PoseEstimator((x, y), yaw, occ2d=occ, origin=origin, res=res,
                        params=EstimatorParams(n_particles=2, init_xy=0.0, init_yaw=0.0, scale_sigma=0.0))
    est.x[:] = x
    est.y[:] = y
    est.b[:] = 0.0
    return float(est.expected_ranges(FAN)[0])


class TelloStateTest(unittest.TestCase):
    def test_parse_real_string(self):
        text = ("mid:-1;x:0;y:0;z:0;mpry:0,0,0;pitch:1;roll:-2;yaw:-12;vgx:3;vgy:-1;vgz:0;templ:60;temph:63;"
                "tof:98;h:90;bat:87;baro:120.31;time:12;agx:-3.00;agy:1.00;agz:-999.00;\r\n")
        raw = tello_state.parse(text)
        self.assertEqual(raw["yaw"], -12)
        self.assertEqual(raw["mpry"], "0,0,0")
        si = tello_state.to_si(raw)
        self.assertAlmostEqual(si["vx"], -0.3)           # 실기체: 앞으로 날 때 vgx가 음수 (vx_sign -1)
        self.assertAlmostEqual(si["vy"], -0.1)           # vy_sign +1 (2026-10-29 실기체)
        self.assertEqual(si["frame"], "takeoff")
        self.assertAlmostEqual(si["h"], 0.9)
        self.assertAlmostEqual(si["tof"], 0.98)
        self.assertAlmostEqual(si["yaw"], math.radians(12))

    def test_round_trip_quantizes_like_tello(self):
        si = _state(math.radians(30.4), 0.437, -0.12, 1.003)
        self.assertAlmostEqual(si["yaw"], math.radians(30), places=6)   # 1° 단위
        self.assertAlmostEqual(si["vx"], 0.4)                            # 0.1 m/s 단위
        self.assertAlmostEqual(si["vy"], -0.1)
        self.assertAlmostEqual(si["h"], 1.0)


class DeadReckoningTest(unittest.TestCase):
    def test_straight_line(self):
        est = PoseEstimator((1.0, 1.0), 0.0, params=EstimatorParams(use_map=False))
        for k in range(51):                                   # 5초, 10 Hz, 0.4 m/s
            est.on_state(_state(0.0, 0.4), k * 0.1)
        x, y, z, yaw, _ = est.estimate()
        self.assertAlmostEqual(x, 3.0, places=6)
        self.assertAlmostEqual(y, 1.0, places=6)
        self.assertAlmostEqual(z, 1.0, places=6)

    def test_tello_yaw_zero_is_not_map_zero(self):
        # 기체는 지도에서 북쪽(+y)을 보고 출발, Tello yaw는 전원 켤 때 기준이라 임의 값(+40°).
        # 속도값도 그 Tello yaw 0 방향 기준이라, 앞으로 0.5 m/s는 (0.5·cos 40°, 0.5·sin 40°)로 온다 (0.1 m/s로 반올림)
        est = PoseEstimator((1.0, 1.0), math.pi / 2, params=EstimatorParams(use_map=False))
        a = math.radians(40)
        for k in range(21):
            est.on_state(_state(a, 0.5 * math.cos(a), 0.5 * math.sin(a)), k * 0.1)
        x, y, _, yaw, _ = est.estimate()
        self.assertAlmostEqual(x, 1.0, delta=0.08)
        self.assertAlmostEqual(y, 2.0, delta=0.08)
        self.assertAlmostEqual(yaw, math.pi / 2, places=6)

    def test_turned_around_still_moves_forward(self):
        # 2026-10-29 실기체 충돌: 창문(−x)을 보고 출발해 돌아선(+x) 뒤 앞으로 갔는데, 속도값을 기체 기준으로 읽어
        # 추정이 거꾸로(−x) 갔다. Tello 흉내(실기체와 같은 규칙)로 같은 비행을 만들어 추정이 +x로 가는지 본다
        synth = StateSynth(seed=3, vel_noise=0.0, scale_sigma=0.0, yaw_noise_deg=0.0, yaw_drift_deg_per_min=0.0,
                           h_noise=0.0)
        est = PoseEstimator((5.5, 4.45), math.pi, params=EstimatorParams(use_map=False))
        t, x = 0.0, 5.5

        def feed(yaw):
            est.on_state(tello_state.to_si(tello_state.parse(synth.sample(t, x, 4.45, 1.0, yaw, 0.0, 1.0))), t)

        for _ in range(5):                       # 출발: −x를 보고 제자리
            feed(math.pi)
            t += 0.1
        for k in range(1, 19):                   # 제자리에서 +x로 돌아섬
            feed(math.pi - k * math.pi / 18)
            t += 0.1
        for _ in range(21):                      # +x로 0.5 m/s, 2초
            x += 0.05
            feed(0.0)
            t += 0.1
        ex, ey, _, eyaw, _ = est.estimate()
        # 속도값이 0.1 m/s 단위라 (Tello yaw 0이 임의 방향이면) 축마다 최대 0.05 m/s씩 반올림된다 → 2초에 옆으로 0.1 m 남짓
        self.assertAlmostEqual(ex, x, delta=0.15)
        self.assertAlmostEqual(ey, 4.45, delta=0.15)
        self.assertAlmostEqual(math.cos(eyaw), 1.0, delta=0.01)


def _raw(yaw_deg, vgx, vgy):
    """Tello가 실제로 보낸 정수 값 그대로 (yaw °, vgx·vgy dm/s)."""
    return tello_state.to_si({"yaw": yaw_deg, "vgx": vgx, "vgy": vgy, "vgz": 0, "h": 100, "tof": 100, "bat": 90})


class NoSlipTest(unittest.TestCase):
    def test_slow_flight_off_axis_follows_heading(self):
        # 2026-11-05 17:17 실기체 복도: Tello yaw 16°로 0.2 m/s 직진하는 7초 내내 vgx −2, vgy 0 (옆 성분 0.055가 잘림).
        # 예전에는 위치가 기수에서 16° 비켜(Tello yaw 0 방향으로) 곧게 갔다. 이제는 기수 쪽으로 간다
        est = PoseEstimator((0.0, 0.0), 0.0, params=EstimatorParams(use_map=False))
        for k in range(71):
            est.on_state(_raw(16, -2, 0), k * 0.1)
        x, y, _, yaw, _ = est.estimate()
        self.assertAlmostEqual(yaw, 0.0, places=6)
        self.assertAlmostEqual(x, 1.4 * math.cos(math.radians(16)), delta=0.01)   # 기수 쪽 성분만
        self.assertAlmostEqual(y, 0.0, delta=0.01)
        old = PoseEstimator((0.0, 0.0), 0.0, params=EstimatorParams(use_map=False, no_slip=False))
        for k in range(71):
            old.on_state(_raw(16, -2, 0), k * 0.1)
        self.assertAlmostEqual(old.estimate()[1], 1.4 * math.sin(math.radians(16)), delta=0.01)

    def test_backward_also_follows_heading(self):
        est = PoseEstimator((0.0, 0.0), 0.0, params=EstimatorParams(use_map=False))
        for k in range(21):
            est.on_state(_raw(16, 2, 0), k * 0.1)                # 뒤로 0.2 m/s
        x, y, *_ = est.estimate()
        self.assertAlmostEqual(x, -0.4 * math.cos(math.radians(16)), delta=0.01)
        self.assertAlmostEqual(y, 0.0, delta=0.01)

    def test_clear_side_slide_is_kept(self):
        # 돌고 난 뒤 남은 속도로 옆으로 0.3 m/s 미끄러짐: 기수에 수직인 성분이 커서 잰 값 그대로 쓴다
        est = PoseEstimator((0.0, 0.0), 0.0, params=EstimatorParams(use_map=False))
        for k in range(11):
            est.on_state(_raw(90, -3, 0), k * 0.1)
        x, y, *_ = est.estimate()
        self.assertAlmostEqual(x, 0.0, delta=0.01)
        self.assertAlmostEqual(y, 0.3, delta=0.01)

    def test_fast_turn_keeps_measured_velocity(self):
        # 빨리 도는 동안(90°/s)은 속도가 기수를 늦게 따라가므로 바로잡지 않는다
        est = PoseEstimator((0.0, 0.0), 0.0, params=EstimatorParams(use_map=False))
        for k in range(11):
            est.on_state(_raw(9 * k, -2, 0), k * 0.1)
        x, y, *_ = est.estimate()
        self.assertAlmostEqual(x, 0.2, delta=0.01)
        self.assertAlmostEqual(y, 0.0, delta=0.02)    # 회전 속도를 0.25초 넘게 모은 뒤부터 안 바로잡는다


class HeightTest(unittest.TestCase):
    """실기체 높이: 2026-10-27 제자리 비행 기록(0.5초 간격 발췌). h는 바닥 기준이 아니었고 하방 ToF가 실제 높이였다."""
    # (경과 s, h cm, 하방 tof cm, vgz)
    FLIGHT = [(0.0, 0, 10, 0), (1.0, -10, 18, 6), (1.6, 0, 73, 2), (3.0, 30, 120, -2), (4.5, 20, 88, -1),
              (5.5, 0, 76, 0), (8.5, 10, 79, 0), (12.0, 0, 77, 0), (15.4, 0, 56, -4), (16.2, -40, 17, -4),
              (17.5, 0, 10, -1)]

    def run_flight(self, source):
        """기록 발췌를 상태값처럼 초당 10번으로 이어 넣는다. 발췌 시각마다의 높이를 돌려준다."""
        est = PoseEstimator((1.0, 1.0), 0.0, 0.0, params=EstimatorParams(height_source=source))
        zs = []
        for (t0, h0, f0, v0), (t1, h1, f1, v1) in zip(self.FLIGHT, self.FLIGHT[1:] + [self.FLIGHT[-1]]):
            n = max(1, round((t1 - t0) * 10))
            for k in range(n):
                a = k / n
                raw = tello_state.parse(f"yaw:0;vgx:0;vgy:0;vgz:{round(v0 + a * (v1 - v0))};"
                                        f"h:{round(h0 + a * (h1 - h0))};tof:{round(f0 + a * (f1 - f0))};bat:90;")
                est.on_state(tello_state.to_si(raw), t0 + a * (t1 - t0))
                if k == 0:
                    zs.append(est.z)
        return zs

    def test_vgz_up_is_positive(self):
        si = tello_state.to_si(tello_state.parse("vgz:6;"))
        self.assertAlmostEqual(si["vz"], 0.6)   # 이륙 중에 +6이었다
        self.assertEqual(tello_state.from_si(0, 0, 0, 0.6, 1, 1, 90)["vgz"], 6)

    def test_tof_height_follows_floor_distance(self):
        zs = self.run_flight("tof")
        self.assertEqual(zs[0], 0.0)                              # 바닥(ToF 10 cm = 무효) → 출발 높이
        self.assertAlmostEqual(zs[6], 0.79, places=2)             # 떠 있을 때 ToF 0.79 m
        self.assertLess(zs[-1], 0.2)                              # 착륙 뒤 ToF 무효 → vz로 내려온 값
        self.assertLess(self.run_flight("h")[6], 0.2)             # h로는 떠 있는데도 0.1 m로 보였다

    def test_tof_jump_from_table_keeps_height(self):
        est = PoseEstimator((1.0, 1.0), 0.0, 0.0, params=EstimatorParams(height_source="tof"))
        for k, tof in enumerate([100, 100, 100, 25, 25, 100]):   # 1 m로 날다가 0.75 m 탁자 위를 지남
            est.on_state(tello_state.to_si(tello_state.parse(f"yaw:0;vgz:0;h:0;tof:{tof};")), k * 0.1)
            if tof == 25:
                self.assertAlmostEqual(est.z, 1.0, places=2)
        self.assertAlmostEqual(est.z, 1.0, places=2)

    def test_tof_trusted_again_when_change_persists(self):
        est = PoseEstimator((1.0, 1.0), 0.0, 0.0, params=EstimatorParams(height_source="tof"))
        for k in range(30):   # 1.0 m로 보다가 계속 1.4 m를 잰다(예측이 틀린 경우): 1초 뒤 믿는다
            est.on_state(tello_state.to_si(tello_state.parse(f"yaw:0;vgz:0;h:0;tof:{100 if k < 5 else 140};")), k * 0.1)
        self.assertAlmostEqual(est.z, 1.4, places=2)

    @staticmethod
    def feed(est, seq, t0=0.0, vgx=0):
        """(하방 tof cm, h cm) 차례를 0.1초마다 넣는다. 끝 시각을 돌려준다."""
        t = t0
        for tof, h in seq:
            est.on_state(tello_state.to_si(tello_state.parse(f"yaw:0;vgx:{vgx};vgy:0;vgz:0;h:{h};tof:{tof};")), t)
            t += 0.1
        return t

    # 2026-11-09 17:07 270–272 s: 조형물 위로 지나며 ToF가 0.1초에 0.04–0.12 m씩 줄었다(h는 1.6–1.7 그대로)
    EDGE = [(123, 160), (107, 160), (97, 160), (93, 160), (92, 160), (91, 160), (91, 160), (92, 160),
            (80, 170), (70, 170), (67, 170), (66, 170), (67, 170)]

    def test_slow_furniture_edge_keeps_height(self):
        for gap, low in ((0.0, True), (0.4, False)):
            est = PoseEstimator((1.0, 1.0), 0.0, 0.0, params=EstimatorParams(height_source="tof", h_guard_gap=gap))
            t = self.feed(est, [(136, 160)] * 30)            # 바닥 위 1.36 m (h 1.6: 평소 차이 −0.24)
            self.feed(est, self.EDGE, t)
            if low:
                self.assertLess(est.z, 0.75)                 # 예전: 조금씩 줄어 tof_jump에 안 걸려 높이가 조형물 위 0.67 m가 됐다
            else:
                self.assertGreater(est.z, 1.3)               # h로 이어 간다
                self.assertTrue(est.h_guarding)

    def test_climb_over_furniture_follows_h(self):
        est = PoseEstimator((1.0, 1.0), 0.0, 0.0, params=EstimatorParams(height_source="tof"))
        t = self.feed(est, [(136, 160)] * 30)
        t = self.feed(est, self.EDGE, t)
        # 그 위에서 오른다: h 1.7→2.0, ToF는 조형물까지 0.7→0.96. 예전 높이(0.96)로는 아직 1 m 아래라 계속 올렸다
        self.feed(est, [(70 + 2 * k, 170 + (30 * k) // 13) for k in range(14)], t)
        self.assertGreater(est.z, 1.6)               # h 2.0 + 평소 차이 −0.24 쪽으로 (ToF 0.96이 아니라)

    def test_real_descent_follows_tof(self):
        est = PoseEstimator((1.0, 1.0), 0.0, 0.0, params=EstimatorParams(height_source="tof"))
        t = self.feed(est, [(140, 140)] * 30)
        self.feed(est, [(140 - 3 * k, 140 - 3 * k) for k in range(18)], t)   # ToF와 h가 같이 0.3 m/s로 내려간다
        self.assertAlmostEqual(est.z, 0.89, delta=0.03)
        self.assertEqual(est.h_guard_count, 0)

    def test_h_offset_shift_accepted_after_moving_away(self):
        # 17:07 202–207 s: h−ToF가 +0.3에서 −0.3으로 바뀐 뒤 60초 동안 그대로였다(가구가 아니라 h가 어긋남).
        # 가구로 본 자리에서 1.5 m 넘게 옮겨 가도 어긋남이 그대로면 ToF로 돌아간다
        est = PoseEstimator((1.0, 1.0), 0.0, 0.0, params=EstimatorParams(height_source="tof", use_map=False))
        t = self.feed(est, [(150, 150)] * 30)
        t = self.feed(est, [(100, 150)] * 10, t)             # 제자리: 가구로 본다
        self.assertTrue(est.h_guarding)
        self.assertGreater(est.z, 1.4)
        self.feed(est, [(100, 150)] * 40, t, vgx=5)          # 0.5 m/s로 4초(2 m) 옮겨 가도 그대로
        self.assertFalse(est.h_guarding)
        self.assertAlmostEqual(est.z, 1.0, delta=0.02)

    def test_mapped_furniture_below_adds_its_top_to_tof(self):
        # 사용자 지시(11/09): 하방 ToF는 바로 밑 물건까지 거리라 고도 기준이 못 된다 — 밑에 무엇이 있는지는 카메라 지도(비전)가 안다.
        # 지도가 '밑에 꼭대기 0.7 m 조형물'이라 하면 높이는 ToF + 0.7. h가 어긋나 h_guard가 못 잡는 경우에도, 조금씩 줄어도
        est = PoseEstimator((1.0, 1.0), 0.0, 0.0, params=EstimatorParams(height_source="tof", h_guard_gap=0.0))
        t = self.feed(est, [(136, 160)] * 30)
        for k, (tof, h) in enumerate(self.EDGE):
            est.on_under(0.70, t + 0.1 * k - 0.02)           # 비행 노드가 10 Hz로 알려 준다
            self.feed(est, [(tof, h)], t + 0.1 * k)
        self.assertAlmostEqual(est.z, 0.67 + 0.70, delta=0.03)
        self.assertTrue(est.under_guarding)
        self.assertEqual(est.under_guard_count, 1)
        self.feed(est, [(136, 160)] * 5, t + 1.3)            # 조형물을 벗어나 ToF가 다시 바닥: 그대로 ToF
        self.assertAlmostEqual(est.z, 1.36, delta=0.02)
        self.assertFalse(est.under_guarding)

    def test_mapped_top_that_does_not_fit_is_ignored(self):
        # 가구 가장자리: 지도는 밑에 꼭대기 0.7이라 하는데 ToF는 옆 바닥 1.36을 잰다 → ToF + 0.7 = 2.06은 높이와 안 맞으니 ToF 그대로
        est = PoseEstimator((1.0, 1.0), 0.0, 0.0, params=EstimatorParams(height_source="tof"))
        t = self.feed(est, [(136, 160)] * 10)
        for k in range(10):
            est.on_under(0.70, t + 0.1 * k - 0.02)
            self.feed(est, [(136, 160)], t + 0.1 * k)
        self.assertAlmostEqual(est.z, 1.36, delta=0.02)
        self.assertFalse(est.under_guarding)

    def test_stale_under_map_is_ignored(self):
        est = PoseEstimator((1.0, 1.0), 0.0, 0.0, params=EstimatorParams(height_source="tof", h_guard_gap=0.0))
        t = self.feed(est, [(136, 160)] * 30)
        est.on_under(0.70, t - 3.0)                          # 3초 전 값
        self.feed(est, [(66, 160)] * 5, t)                   # 한 번에 튄 ToF: tof_jump가 막는다 (h 그대로라 안 믿음)
        self.assertFalse(est.under_guarding)
        self.assertGreater(est.z, 1.3)


class MapCorrectionTest(unittest.TestCase):
    def setUp(self):
        self.occ, self.origin, self.res = occupancy_slice(_room(4.0, 3.0), 1.0, 0.05)

    def fly(self, scale, obstacle_close=False, use_map=True):
        """(1, 1.5)에서 동쪽 벽(안쪽 면 x = 3.9)을 향해 0.4 m/s로 약 6초. Tello 속도는 scale배로 틀리게 보고."""
        est = PoseEstimator((1.0, 1.5), 0.0, occ2d=self.occ, origin=self.origin, res=self.res,
                            params=EstimatorParams(use_map=use_map), seed=1)
        x_true = 1.0
        for k in range(1, 60):
            x_true += 0.4 * 0.1
            est.on_state(_state(0.0, 0.4 * scale), k * 0.1)
            measured = _true_range(self.occ, self.origin, self.res, x_true, 1.5, 0.0)
            if obstacle_close:
                measured = min(measured, 0.6)       # 지도에 없는 물체가 늘 0.6 m 앞에 있다
            est.on_tof(measured, FAN)
        return est.estimate(), x_true

    def test_wall_in_view_fixes_scale_error(self):
        (x_dr, *_), x_true = self.fly(1.15, use_map=False)
        (x_pf, _, _, _, sigma), _ = self.fly(1.15)
        self.assertGreater(abs(x_dr - x_true), 0.3)            # 속도 15% 과대 → 적분만 하면 크게 어긋남
        self.assertLess(abs(x_pf - x_true), 0.15)             # 앞 벽이 보이면 바로잡힌다
        self.assertLess(sigma, 0.3)

    def test_unmapped_obstacle_does_not_break_estimate(self):
        (x_pf, *_), x_true = self.fly(1.0, obstacle_close=True)
        self.assertLess(abs(x_pf - x_true), 0.2)

    def test_likelihood_is_asymmetric(self):
        expected = np.array([1.0, 1.0, math.inf])
        closer = tof_likelihood(0.5, expected, 1.2, 0.08)       # 지도보다 가까움: 약한 감점
        farther = tof_likelihood(1.15, np.array([0.6]), 1.2, 0.08)   # 지도보다 멂: 강한 감점
        unseen = tof_likelihood(math.inf, np.array([0.5]), 1.2, 0.08)
        self.assertAlmostEqual(closer[0], 0.3)
        self.assertAlmostEqual(farther[0], 0.05)
        self.assertAlmostEqual(unseen[0], 0.05)

    def test_standing_before_unmapped_obstacle_keeps_estimate(self):
        # (2, 1.5)에서 동쪽을 보고 멈춰 있다. 지도에 없는 물체가 0.33 m 앞에 있고, 왼쪽 앞(y 1.6–1.8)에는 지도에
        # 있는 상자가 있어 '왼쪽으로 0.1 m 비켜난 후보'라면 그 상자로 0.33 m가 설명된다. 같은 측정을 50번 받아도 그쪽으로
        # 쏠리면 안 된다 (2단계 seed 1에서 비상 정지 뒤 0.7 m 틀어진 문제)
        occ, origin, res = occupancy_slice(_room(4.0, 3.0, _box(2.44, 1.7, 1.0, 0.12, 0.2, 2.0)), 1.0, 0.05)
        est = PoseEstimator((2.0, 1.5), 0.0, occ2d=occ, origin=origin, res=res,
                            params=EstimatorParams(init_xy=0.15), seed=3)
        for k in range(1, 51):
            est.on_state(_state(0.0, 0.0), k * 0.1)
            est.on_tof(0.33, FAN)
        x, y, *_ = est.estimate()
        self.assertLess(math.hypot(x - 2.0, y - 1.5), 0.05)

    def test_relocalize_spread_and_turn_fixes_stuck_estimate(self):
        # 실제는 (1.0, 1.0)인데 추정은 (1.4, 1.25)에 σ 0.05로 굳어 있다. 서쪽·남쪽 벽이 0.9 m 안에 있다
        def turn(est):
            for k in range(1, 601):                              # 6°/s로 한 바퀴 (60초)
                yaw = math.radians(6.0) * k * 0.1
                est.on_state(_state(yaw, 0.0, 0.0), k * 0.1)
                est.on_tof(_true_range(self.occ, self.origin, self.res, 1.0, 1.0, yaw), FAN, k * 0.1)
            x, y, *_ = est.estimate()
            return math.hypot(x - 1.0, y - 1.0)

        def stuck():
            return PoseEstimator((1.4, 1.25), 0.0, occ2d=self.occ, origin=self.origin, res=self.res,
                                 params=EstimatorParams(init_xy=0.05), seed=4)

        self.assertGreater(turn(stuck()), 0.3)    # 그냥 돌면 굳은 후보끼리만 겨뤄 못 바로잡는다
        est = stuck()
        est.spread(0.4, math.radians(3.0))
        self.assertLess(turn(est), 0.12)          # 흩은 뒤 돌면 맞는 자리로 돌아온다

    def test_some_particles_stop_at_wall(self):
        # 남쪽 벽(안쪽 면 y = 0.1) 가까이에서 남쪽으로 조금 움직인다. 벽에 닿는 일부 후보만 벽 앞에 멈춘다
        est = PoseEstimator((3.0, 0.3), 0.0, occ2d=self.occ, origin=self.origin, res=self.res,
                            params=EstimatorParams(init_xy=0.04), seed=2)
        for k in range(1, 11):
            est.on_state(_state(0.0, 0.0, -0.1), k * 0.1)       # 오른쪽(남쪽)으로 0.1 m/s, 1초
        self.assertGreater(float(est.y.min()), 0.09)            # 어느 후보도 벽 속(y < 0.1)에 있지 않다

    def test_estimate_keeps_moving_when_most_particles_blocked(self):
        # x 0–2.0에 y 1.5 칸막이. 실제로는 칸막이 끝 오른쪽(x 2.05)으로 북쪽으로 지나가는데, 추정은 0.45 m 왼쪽
        # (x 1.6, 칸막이 줄 위)에 σ 2 cm로 굳어 있다. 모든 후보가 칸막이에 막혀도 추정이 멈추면 안 된다
        occ, origin, res = occupancy_slice(_room(4.0, 3.0, _box(1.0, 1.5, 1.25, 2.0, 0.1, 2.5)), 1.0, 0.05)

        def fly(params):
            est = PoseEstimator((1.6, 1.0), math.pi / 2, occ2d=occ, origin=origin, res=res, params=params, seed=1)
            for k in range(1, 34):                               # 북쪽으로 0.3 m/s, 약 1 m
                est.on_state(_state(math.pi / 2, 0.0, 0.3), k * 0.1)   # 속도값은 Tello yaw 0 기준 → 북쪽 = (0, 0.3)
            return est.estimate()[1]

        self.assertLess(fly(EstimatorParams(init_xy=0.02, block_stop_frac=1.1)), 1.5)   # 예전: 칸막이 앞에 멈춤
        self.assertGreater(fly(EstimatorParams(init_xy=0.02)), 1.8)                     # 지금: 계속 따라간다

if __name__ == "__main__":
    unittest.main()
