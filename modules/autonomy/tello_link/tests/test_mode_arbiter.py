"""모드 판정기 시험: 전이표, 우선순위, 자율 끊김, 넘겨받기 (표준 라이브러리만).

    python -B -m unittest discover -s modules/autonomy/tello_link/tests -v
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mode_arbiter import ModeArbiter  # noqa: E402


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


def vel(vx=0.0, vy=0.0, vz=0.0, yaw=0.0):
    return {"vx_mps": vx, "vy_mps": vy, "vz_mps": vz, "yaw_rate_rps": yaw, "frame": "body"}


class ArbiterTestBase(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.arb = ModeArbiter(source="fake", clock=self.clock, slew_rc_per_s=0)  # 가감속은 test_rc_mixer에서

    def actions(self, kind=None):
        return [a for a in self.arb.drain() if kind is None or a.kind == kind]

    def controls(self, actions):
        return [a.payload["cmd"] for a in actions if a.kind == "autonomy"]

    def flights(self, actions):
        return [a.payload["name"] for a in actions if a.kind == "flight"]

    def to_grounded(self, autonomy=True):
        self.arb.on_link(True)
        if autonomy:
            self.arb.on_autonomy_connected()
        self.arb.drain()

    def to_manual(self, autonomy=True):
        self.to_grounded(autonomy)
        self.assertTrue(self.arb.handle_cmd("takeoff")["ok"])
        self.arb.on_takeoff_done(True)
        self.arb.drain()

    def to_auto(self):
        self.to_manual()
        r = self.arb.handle_cmd("auto_start", {"goal": {"x_m": 1.0, "y_m": 0.0, "frame": "map"}}, client="monitor")
        self.assertTrue(r["ok"], r)
        acts = self.arb.drain()
        start = [a.payload for a in acts if a.kind == "autonomy"][0]
        self.arb.on_ack({"id": start["id"], "ok": True})
        return start


class TransitionTest(ArbiterTestBase):
    def test_starts_disconnected_then_grounded(self):
        self.assertEqual(self.arb.mode, "disconnected")
        self.arb.on_link(True)
        self.assertEqual(self.arb.mode, "grounded")
        self.arb.on_link(False)
        self.assertEqual(self.arb.mode, "disconnected")

    def test_takeoff_goes_manual_only_after_done(self):
        self.to_grounded()
        r = self.arb.handle_cmd("takeoff")
        self.assertTrue(r["ok"])
        self.assertEqual(self.flights(self.arb.drain()), ["takeoff"])
        self.assertEqual(self.arb.mode, "grounded")
        self.assertNotIn("takeoff", self.arb.allowed())  # 응답 기다리는 중 두 번 못 누름
        self.arb.on_takeoff_done(True)
        self.assertEqual(self.arb.mode, "manual")
        self.assertEqual(self.arb.authority, "operator")

    def test_takeoff_failure_stays_grounded(self):
        self.to_grounded()
        self.arb.handle_cmd("takeoff")
        self.arb.on_takeoff_done(False, "preflight_failed")
        self.assertEqual(self.arb.mode, "grounded")
        self.assertIn("takeoff", self.arb.allowed())

    def test_full_cycle(self):
        self.to_auto()
        self.assertEqual((self.arb.mode, self.arb.authority), ("auto", "autonomy"))
        self.assertTrue(self.arb.handle_cmd("pause")["ok"])
        self.assertEqual(self.arb.mode, "paused")
        self.assertEqual(self.controls(self.arb.drain()), ["pause"])
        self.assertTrue(self.arb.handle_cmd("resume")["ok"])
        self.assertEqual(self.arb.mode, "auto")
        self.assertEqual(self.controls(self.arb.drain()), ["resume"])
        self.assertTrue(self.arb.handle_cmd("land")["ok"])
        acts = self.arb.drain()
        self.assertEqual(self.arb.mode, "landing")
        self.assertEqual(self.controls(acts), ["stop"])
        self.assertEqual(self.flights(acts), ["land"])
        self.arb.on_landed()
        self.assertEqual(self.arb.mode, "grounded")

    def test_not_allowed_in_mode(self):
        self.to_grounded()
        for name in ("land", "pause", "resume", "takeover", "auto_start", "auto_stop"):
            r = self.arb.handle_cmd(name)
            self.assertEqual((r["ok"], r["reason"]), (False, "not_allowed_in_mode"), name)

    def test_mode_changes_emit_events(self):
        self.to_grounded()
        self.arb.handle_cmd("takeoff")
        self.arb.on_takeoff_done(True)
        ev = [a.payload for a in self.arb.drain() if a.kind == "event"]
        self.assertEqual(ev[-1]["kind"], "mode_changed")
        self.assertEqual((ev[-1]["detail"]["from"], ev[-1]["detail"]["to"]), ("grounded", "manual"))

    def test_mission_arrived_pauses(self):
        self.to_auto()
        self.arb.on_mission_state({"state": "LAND", "phase": "arrived", "progress": 1.0})
        self.assertEqual(self.arb.mode, "paused")
        self.assertFalse(self.arb.mission_active)
        self.assertNotIn("resume", self.arb.allowed())

    def test_safety_landing_stops_autonomy(self):
        self.to_auto()
        self.arb.on_safety_landing("battery_low")
        self.assertEqual(self.arb.mode, "landing")
        self.assertEqual(self.arb.snapshot()["landing_reason"], "battery_low")
        self.assertEqual(self.controls(self.arb.drain()), ["stop"])

    def test_auto_stop(self):
        self.to_auto()
        self.arb.handle_cmd("auto_stop")
        self.assertEqual(self.arb.mode, "paused")
        self.assertEqual(self.controls(self.arb.drain()), ["stop"])
        self.assertNotIn("resume", self.arb.allowed())


class PriorityTest(ArbiterTestBase):
    def test_emergency_always_wins(self):
        self.to_auto()
        self.arb.on_cmd_vel(vel(0.2))
        r = self.arb.handle_cmd("emergency", client="control")
        self.assertTrue(r["ok"])
        acts = self.arb.drain()
        self.assertEqual(self.flights(acts), ["emergency"])
        self.assertEqual(self.controls(acts), ["stop"])
        self.assertEqual(self.arb.mode, "emergency")
        self.assertIsNone(self.arb.tick().rc)  # 비상 정지 뒤 rc 없음
        self.assertFalse(self.arb.on_cmd_vel(vel(0.2)))

    def test_emergency_even_when_disconnected(self):
        self.assertTrue(self.arb.handle_cmd("emergency")["ok"])
        self.assertEqual(self.flights(self.arb.drain()), ["emergency"])

    def test_autonomy_land_ignored_when_operator_controls(self):
        self.to_manual()
        self.assertFalse(self.arb.on_autonomy_request("land")["ok"])
        self.assertEqual(self.arb.mode, "manual")

    def test_autonomy_land_accepted_in_auto(self):
        self.to_auto()
        self.assertTrue(self.arb.on_autonomy_request("land")["ok"])
        self.assertEqual(self.arb.mode, "landing")
        self.assertEqual(self.flights(self.arb.drain()), ["land"])

    def test_autonomy_cannot_takeoff_or_emergency(self):
        self.to_grounded()
        self.assertEqual(self.arb.on_autonomy_request("takeoff")["reason"], "takeoff_by_operator_only")
        self.assertEqual(self.arb.mode, "grounded")
        self.to_auto()
        r = self.arb.on_autonomy_request("emergency")
        self.assertEqual(r["reason"], "emergency_by_operator_only")
        self.assertEqual(self.arb.mode, "paused")
        self.assertNotIn("emergency", self.flights(self.arb.drain()))

    def test_cmd_vel_ignored_outside_auto(self):
        self.to_manual()
        self.assertFalse(self.arb.on_cmd_vel(vel(0.2)))
        self.assertEqual([a.payload["kind"] for a in self.arb.drain()], ["cmd_ignored"])
        self.assertFalse(self.arb.on_cmd_vel(vel(0.2)))
        self.assertEqual(self.arb.drain(), [])  # 1초에 한 번만 알림
        self.assertEqual(self.arb.tick().source, "deadman")

    def test_replay_rejects_everything(self):
        arb = ModeArbiter(source="replay", clock=self.clock)
        arb.on_link(True)
        self.assertEqual(arb.allowed(), [])
        self.assertEqual(arb.handle_cmd("emergency")["reason"], "replay_source")
        self.assertEqual(arb.drain()[-1].kind, "event")  # mode_changed만, flight 없음


class AutonomyLinkTest(ArbiterTestBase):
    def test_stale_holds_then_resumes(self):
        self.to_auto()
        self.arb.on_cmd_vel(vel(0.2))
        self.assertEqual(self.arb.tick().rc, (0, 20, 0, 0))
        self.clock.advance(0.6)
        out = self.arb.tick()
        self.assertEqual((out.rc, out.source, self.arb.mode), ((0, 0, 0, 0), "autonomy_stale", "auto"))
        self.assertIn("autonomy_stale", self.arb.warnings())
        self.arb.on_cmd_vel(vel(0.1))
        self.assertEqual(self.arb.tick().rc, (0, 10, 0, 0))
        self.assertNotIn("autonomy_stale", self.arb.warnings())

    def test_timeout_pauses(self):
        self.to_auto()
        self.arb.on_cmd_vel(vel(0.2))
        self.clock.advance(2.1)
        out = self.arb.tick()
        self.assertEqual((out.rc, self.arb.mode), ((0, 0, 0, 0), "paused"))
        self.assertIn("autonomy_timeout", self.arb.warnings())
        self.assertEqual(self.controls(self.arb.drain()), ["pause"])
        self.assertIn("resume", self.arb.allowed())

    def test_start_grace_counts_from_auto_start(self):
        self.to_auto()
        self.clock.advance(0.4)
        self.assertEqual(self.arb.tick().source, "autonomy")

    def test_disconnect_pauses_immediately(self):
        self.to_auto()
        self.arb.on_autonomy_disconnected()
        self.assertEqual(self.arb.mode, "paused")
        self.assertIn("autonomy_disconnected", self.arb.warnings())
        self.assertNotIn("resume", self.arb.allowed())

    def test_rejected_start_returns_to_manual(self):
        self.to_manual()
        self.arb.handle_cmd("auto_start", client="monitor")
        start = [a.payload for a in self.arb.drain() if a.kind == "autonomy"][0]
        self.arb.on_ack({"id": start["id"], "ok": False, "reason": "pose_not_ready"})
        self.assertEqual(self.arb.mode, "manual")
        self.assertFalse(self.arb.mission_active)


class AutoStartTest(ArbiterTestBase):
    def test_requires_autonomy(self):
        self.to_manual(autonomy=False)
        r = self.arb.handle_cmd("auto_start", client="monitor")
        self.assertEqual(r["reason"], "autonomy_not_connected")

    def test_rejects_autonomy_joined_in_air(self):
        self.to_manual(autonomy=False)
        self.arb.on_autonomy_connected()
        self.assertFalse(self.arb.autonomy_ground_origin)
        self.assertNotIn("auto_start", self.arb.allowed())
        self.assertEqual(self.arb.handle_cmd("auto_start", client="monitor")["reason"], "autonomy_joined_in_air")

    def test_phone_cannot_start(self):
        self.to_manual()
        self.assertEqual(self.arb.handle_cmd("auto_start", client="control")["reason"], "client_not_allowed")
        self.assertEqual(self.arb.mode, "manual")

    def test_not_ready_warning_on_ground(self):
        self.to_grounded(autonomy=False)
        self.assertIn("autonomy_not_ready", self.arb.warnings())
        self.arb.on_autonomy_connected()
        self.assertNotIn("autonomy_not_ready", self.arb.warnings())

    def test_start_sends_goal(self):
        start = self.to_auto()
        self.assertEqual(start["goal"]["x_m"], 1.0)
        self.assertTrue(self.arb.autonomy_used)


class OperatorTest(ArbiterTestBase):
    def test_joystick_takes_over(self):
        self.to_auto()
        self.arb.on_operator_rc([0.1, 0.1, 0, 0])  # 데드존 안: 무시
        self.assertEqual(self.arb.mode, "auto")
        self.arb.on_operator_rc([0, -1, 0, 0])
        self.assertEqual(self.arb.mode, "manual")
        self.assertEqual(self.arb.last_transition["reason"], "joystick_takeover")
        self.assertEqual(self.controls(self.arb.drain()), ["stop"])
        self.assertEqual(self.arb.operator_interventions, 1)
        self.assertEqual(self.arb.tick().rc, (0, -30, 0, 0))
        self.assertFalse(self.arb.on_cmd_vel(vel(0.2)))  # 넘겨받은 뒤 자율 명령은 기체로 안 감

    def test_takeover_from_paused(self):
        self.to_auto()
        self.arb.handle_cmd("pause")
        self.arb.handle_cmd("takeover")
        self.assertEqual(self.arb.mode, "manual")
        self.assertEqual(self.controls(self.arb.drain()), ["pause", "stop"])
        self.assertEqual(self.arb.operator_interventions, 2)

    def test_deadman(self):
        self.to_manual()
        self.arb.on_operator_rc([0, 1, 0, 0])
        self.assertEqual(self.arb.tick().rc, (0, 30, 0, 0))
        self.clock.advance(0.35)
        self.assertEqual(self.arb.tick().source, "deadman")

    def test_screen_disconnect_zeroes_rc(self):
        self.to_manual()
        self.arb.on_operator_rc([0, 1, 0, 0])
        self.arb.on_operator_disconnected()
        self.assertEqual(self.arb.tick().rc, (0, 0, 0, 0))

    def test_manual_flight_has_no_intervention(self):
        self.to_manual()
        self.arb.handle_cmd("land")
        self.assertEqual(self.arb.operator_interventions, 0)

    def test_snapshot_fields(self):
        self.to_auto()
        snap = self.arb.snapshot()
        for key in ("mode", "authority", "allowed", "warnings", "flying", "mission", "last_transition",
                    "operator_interventions", "autonomy_used"):
            self.assertIn(key, snap)
        self.assertTrue(snap["flying"])


if __name__ == "__main__":
    unittest.main()
