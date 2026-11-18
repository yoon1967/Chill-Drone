"""Gazebo용 Tello 흉내(simdrone/tello_emu.py): SDK 명령 응답, 이륙·착륙 단계, rc → 속도."""
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from autonomy import tello_state  # noqa: E402
from simdrone.tello_emu import FLYING, GROUND, LANDING, EmuParams, StateSynth, TelloEmu  # noqa: E402


def fly(emu: TelloEmu, t: float, h: float, until: float, dt: float = 0.02):
    """간단한 높이 적분으로 emu.update를 돌린다. (마지막 시각, 높이, 받은 늦은 응답들)."""
    replies = []
    while t < until:
        vel, r = emu.update(t, h)
        replies += r
        h = max(0.0, h + vel[2] * dt)
        t += dt
    return t, h, replies


class TelloEmuTest(unittest.TestCase):
    def test_silent_before_sdk_mode(self):
        emu = TelloEmu()
        self.assertIsNone(emu.command(0.0, "takeoff"))
        self.assertEqual(emu.phase, GROUND)
        self.assertEqual(emu.command(0.0, "command"), "ok")
        self.assertEqual(emu.command(0.0, "battery?"), "87")

    def test_ext_tof_like_expansion_kit(self):
        emu = TelloEmu()
        emu.command(0.0, "command")
        self.assertIsNone(emu.command(0.0, "EXT tof?"))         # Gazebo ToF를 안 받으면 키트가 없는 것처럼
        emu.front_tof_m = 0.4815
        self.assertEqual(emu.command(0.0, "EXT tof?"), "tof 482")   # mm
        emu.front_tof_m = math.inf
        self.assertEqual(emu.command(0.0, "EXT tof?"), "tof 8190")  # 범위 밖 (실측값)
        emu.front_tof_m = 1.3
        self.assertEqual(emu.command(0.0, "EXT tof?"), "tof 8190")

    def test_takeoff_replies_after_reaching_height(self):
        emu = TelloEmu()
        emu.command(0.0, "command")
        self.assertIsNone(emu.command(0.0, "takeoff"))      # 'ok'는 다 오른 뒤에
        t, h, replies = fly(emu, 0.0, 0.0, 1.9)
        self.assertEqual((h, replies), (0.0, []))           # 모터가 도는 동안은 그대로
        t, h, replies = fly(emu, t, h, 12.0)
        self.assertEqual(replies, ["ok"])
        self.assertEqual(emu.phase, FLYING)
        self.assertAlmostEqual(h, 0.8, delta=0.02)
        self.assertEqual(emu.command(t, "takeoff"), "error")

    def test_rc_moves_with_lag_and_only_when_flying(self):
        p = EmuParams()
        emu = TelloEmu(p)
        emu.command(0.0, "command")
        emu.command(0.0, "rc 0 50 0 0")
        self.assertEqual(emu.update(0.0, 0.0)[0], (0.0, 0.0, 0.0, 0.0))   # 바닥에서는 rc를 듣지 않는다
        emu.command(0.0, "takeoff")
        t, h, _ = fly(emu, 0.0, 0.0, 12.0)
        emu.command(t, "rc -20 30 0 50")
        vel, _ = emu.update(t, h)
        vel, _ = emu.update(t + p.tau, h)
        self.assertAlmostEqual(vel[0], 0.3 * (1 - math.exp(-1)), delta=0.01)   # 앞 0.3 m/s를 시상수로 따라감
        vel, _ = emu.update(t + 3.0, h)
        self.assertAlmostEqual(vel[0], 0.3, delta=0.005)
        self.assertAlmostEqual(vel[1], 0.2, delta=0.005)                        # rc 왼쪽 - → y +
        self.assertAlmostEqual(vel[3], -0.5 * p.yaw_full, delta=0.01)          # rc 시계 + → 반시계 -

    def test_land_and_idle_landing(self):
        emu = TelloEmu()
        emu.command(0.0, "command")
        emu.command(0.0, "takeoff")
        t, h, _ = fly(emu, 0.0, 0.0, 12.0)
        self.assertEqual(emu.command(t, "land"), None)
        t, h, replies = fly(emu, t, h, t + 5.0)
        self.assertEqual((emu.phase, replies), (GROUND, ["ok"]))
        self.assertLess(h, 0.04)
        self.assertEqual(emu.command(t, "land"), "error")
        emu.command(t, "takeoff")
        t, h, _ = fly(emu, t, h, t + 12.0)
        flying_since = emu.log[-1][0]
        t, h, _ = fly(emu, t, h, flying_since + 14.9)
        self.assertEqual(emu.phase, FLYING)
        t, h, _ = fly(emu, t, h, flying_since + 15.2)      # 15초 넘게 명령이 없으면 스스로 내린다
        self.assertEqual(emu.phase, LANDING)

    def test_last_rc_stays_while_landing(self):
        # 2026-11-12 13:58 실기체: 뒤쪽 제동(rc 0 -36 0 0) 직후 land → 내려오는 내내 그 속도로 뒤로 갔다.
        # 흉내도 land에서 rc를 지우지 않는다. 지우는 것은 드라이버가 land 앞에 보내는 rc 0 0 0 0이다
        p = EmuParams()
        emu = TelloEmu(p)
        emu.command(0.0, "command")
        emu.command(0.0, "takeoff")
        t, h, _ = fly(emu, 0.0, 0.0, 12.0)
        emu.command(t, "rc 0 -36 0 0")
        t, h, _ = fly(emu, t, h, t + 1.0)
        emu.command(t, "land")
        vel, _ = emu.update(t + 0.02, h)
        self.assertAlmostEqual(vel[0], -0.36 * p.v_full, delta=0.005)   # 뒤로 가며
        self.assertLess(vel[2], 0.0)                                    # 내려온다
        emu.command(t + 0.04, "rc 0 50 0 0")                            # 내려오는 중의 rc는 듣지 않는다
        vel, _ = emu.update(t + 0.06, h)
        self.assertAlmostEqual(vel[0], -0.36 * p.v_full, delta=0.005)
        t, h, _ = fly(emu, t + 0.08, h, t + 6.0)
        self.assertEqual((emu.phase, emu.rc), (GROUND, (0, 0, 0, 0)))

    def test_zero_rc_before_land_descends_straight(self):
        emu = TelloEmu()
        emu.command(0.0, "command")
        emu.command(0.0, "takeoff")
        t, h, _ = fly(emu, 0.0, 0.0, 12.0)
        emu.command(t, "rc 0 -36 0 0")
        t, h, _ = fly(emu, t, h, t + 1.0)
        emu.command(t, "rc 0 0 0 0")
        emu.command(t, "land")
        vel, _ = emu.update(t + 0.02, h)
        self.assertEqual(vel[:2], (0.0, 0.0))
        self.assertLess(vel[2], 0.0)

    def test_emergency_drops(self):
        emu = TelloEmu()
        emu.command(0.0, "command")
        emu.command(0.0, "takeoff")
        t, h, _ = fly(emu, 0.0, 0.0, 12.0)
        self.assertEqual(emu.command(t, "emergency"), "ok")
        vel, replies = emu.update(t + 0.02, h)
        self.assertLess(vel[2], -1.0)
        t, h, _ = fly(emu, t, h, t + 2.0)
        self.assertEqual(emu.phase, GROUND)


class StateSynthTest(unittest.TestCase):
    def test_same_seed_same_packets(self):
        a, b = StateSynth(seed=4), StateSynth(seed=4)
        for k in range(5):
            args = (k * 0.1, 0.03 * k, 0.0, 1.0, 0.2, 0.0, 1.0)
            self.assertEqual(a.sample(*args), b.sample(*args))

    def test_velocity_in_tello_yaw_zero_frame_and_height(self):
        # 실기체처럼 속도값은 Tello yaw 0 방향(전원 켤 때) 기준이다 (2026-10-29). 지도 +y로 0.3 m/s를
        # 그 기준으로 돌려 읽으면 다시 지도 +y가 나와야 한다 (0.1 m/s 단위)
        synth = StateSynth(seed=1, vel_noise=0.0, scale_sigma=0.0, h_noise=0.0, yaw_drift_deg_per_min=0.0)
        yaw = math.radians(90)
        synth.sample(0.0, 0.0, 0.0, 1.0, yaw, 0.0, 1.0)
        raw = tello_state.parse(synth.sample(1.0, 0.0, 0.3, 1.0, yaw, 0.0, 1.0))
        si = tello_state.to_si(raw)
        self.assertEqual(si["frame"], "takeoff")
        c, s = math.cos(synth.yaw_zero), math.sin(synth.yaw_zero)
        self.assertAlmostEqual(c * si["vx"] - s * si["vy"], 0.0, delta=0.08)
        self.assertAlmostEqual(s * si["vx"] + c * si["vy"], 0.3, delta=0.08)
        # 실기체에서 본 꼴: 앞으로 날 때 (vgx, vgy) ≈ −속도·(cos ψ, sin ψ), ψ = Tello yaw
        psi = math.radians(raw["yaw"])
        self.assertAlmostEqual(-(raw["vgx"] * math.cos(psi) + raw["vgy"] * math.sin(psi)) / 10.0, 0.3, delta=0.08)
        self.assertAlmostEqual(si["h"], 1.0)

    def test_contact_accel_when_blocked(self):
        # 기수 90°(지도 +y)로 0.2 m/s 앞으로 가라는데 제자리 → 반력은 뒤쪽: agx ≈ −2.5 × (0.2 − 0.05) g
        synth = StateSynth(seed=2)
        yaw = math.radians(90)
        synth.sample(0.0, 1.0, 1.0, 1.0, yaw, 0.0, 1.0, cmd=(0.2, 0.0))
        raw = tello_state.parse(synth.sample(0.1, 1.0, 1.0, 1.0, yaw, 0.0, 1.0, cmd=(0.2, 0.0)))
        self.assertAlmostEqual(raw["agx"], -375.0, delta=50.0)
        self.assertAlmostEqual(raw["agy"], 0.0, delta=50.0)
        # 왼쪽으로 밀다 막히면 반력은 오른쪽(agy +)
        raw = tello_state.parse(synth.sample(0.2, 1.0, 1.0, 1.0, yaw, 0.0, 1.0, cmd=(0.0, 0.2)))
        self.assertGreater(raw["agy"], 300.0)

    def test_contact_accel_quiet_when_following(self):
        # 명령대로 가면(지도 +y로 0.2 m/s) 잡음뿐, 명령을 안 주면 전처럼 0
        synth = StateSynth(seed=3)
        yaw = math.radians(90)
        for k in range(10):
            raw = tello_state.parse(synth.sample(0.1 * k, 1.0, 1.0 + 0.02 * k, 1.0, yaw, 0.0, 1.0, cmd=(0.2, 0.0)))
            if k:
                self.assertLess(math.hypot(raw["agx"], raw["agy"]), 60.0)
        raw = tello_state.parse(StateSynth(seed=3).sample(0.0, 1.0, 1.0, 1.0, yaw, 0.0, 1.0))
        self.assertEqual((raw["agx"], raw["agy"]), (0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
