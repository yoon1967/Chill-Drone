import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from gateway_bridge import (DriftMonitor, Mission, RcCompensation, Scan, route_forward_m, scan_yaw,
                            tello_yaw_ccw, body_command, bottom_tof_m, clamp_xy, classify_gateway, map_to_body,
                            parse_tello_raw, screen_url, status_line, video_action, yaw_of)

RAW = ("mid:-1;x:-100;y:-100;z:-100;mpry:0,0,0;pitch:-8;roll:1;yaw:54;vgx:0;vgy:0;vgz:0;templ:62;temph:65;"
       "tof:125;h:0;bat:100;baro:34.09;time:0;agx:-99.00;agy:-28.00;agz:-1004.00;")


class TelloStateTests(unittest.TestCase):
    def test_raw_state_is_parsed(self):
        fields = parse_tello_raw(RAW)
        self.assertEqual(fields["tof"], 125.0)
        self.assertEqual(fields["mpry"], "0,0,0")
        self.assertAlmostEqual(fields["baro"], 34.09)

    def test_bottom_tof_uses_recorder_rule(self):
        self.assertAlmostEqual(bottom_tof_m(parse_tello_raw(RAW)), 1.25)
        for tof in ("10", "0", "600", "6553", "x"):
            self.assertIsNone(bottom_tof_m(parse_tello_raw(f"tof:{tof};")))
        self.assertIsNone(bottom_tof_m({}))


class FrameTests(unittest.TestCase):
    def test_map_velocity_is_rotated_into_body_frame(self):
        # body facing +90 deg (map +y): map +y is straight ahead, map +x is to the right
        bx, by = map_to_body(0.0, 0.2, math.pi / 2)
        self.assertAlmostEqual(bx, 0.2)
        self.assertAlmostEqual(by, 0.0)
        bx, by = map_to_body(0.2, 0.0, math.pi / 2)
        self.assertAlmostEqual(bx, 0.0)
        self.assertAlmostEqual(by, -0.2)

    def test_yaw_from_quaternion(self):
        half = math.radians(30) / 2
        self.assertAlmostEqual(yaw_of(0, 0, math.sin(half), math.cos(half)), math.radians(30))

    def test_horizontal_speed_is_clamped_keeping_direction(self):
        vx, vy = clamp_xy(0.3, 0.4, 0.2)
        self.assertAlmostEqual(math.hypot(vx, vy), 0.2)
        self.assertAlmostEqual(vy / vx, 0.4 / 0.3)
        self.assertEqual(clamp_xy(0.05, 0.0, 0.2), (0.05, 0.0))
        self.assertEqual(clamp_xy(0.1, 0.1, 0.0), (0.0, 0.0))


class CommandTests(unittest.TestCase):
    def test_fresh_command_is_sent_in_body_frame(self):
        cmd, hold = body_command((0.0, 0.15), 0.05, 0.05, math.pi / 2, 0.2)
        self.assertIsNone(hold)
        self.assertAlmostEqual(cmd["vx_mps"], 0.15)
        self.assertAlmostEqual(cmd["vy_mps"], 0.0)
        self.assertEqual((cmd["vz_mps"], cmd["yaw_rate_rps"]), (0.0, 0.0))

    def test_stale_inputs_hover(self):
        for args, reason in (((None, 0.0, 0.0, 0.0), "no_fresh_command"),
                             (((0.1, 0.0), 1.0, 0.0, 0.0), "no_fresh_command"),
                             (((0.1, 0.0), 0.0, 2.0, 0.0), "pose_stale"),
                             (((0.1, 0.0), 0.0, 0.0, None), "pose_stale")):
            cmd, hold = body_command(*args, 0.2)
            self.assertEqual(hold, reason)
            self.assertEqual(cmd, {"vx_mps": 0.0, "vy_mps": 0.0, "vz_mps": 0.0, "yaw_rate_rps": 0.0})


class DriftMonitorTests(unittest.TestCase):
    def feed(self, monitor, seconds, slam_v, vps_dms, t0=0.0):
        """10 Hz poses moving at slam_v (m/s, x) and VPS readings vps_dms (dm/s); returns drifting() at the end."""
        out = False
        for i in range(int(seconds * 10)):
            t = t0 + i * 0.1
            monitor.add_pose(t, slam_v * t, 0.0)
            monitor.add_vps(t, vps_dms, 0.0)
            out = monitor.drifting(t)
        return out

    def test_slam_moving_while_the_drone_does_not(self):
        m = DriftMonitor()                                   # 11/06: SLAM 0.14 m/s forward, vgx 0
        self.assertTrue(self.feed(m, 4.0, 0.14, 0))
        self.assertAlmostEqual(m.mismatch(), 0.14, places=2)

    def test_drone_moving_while_slam_holds(self):
        m = DriftMonitor()                                   # controller holding a drifting estimate
        self.assertTrue(self.feed(m, 4.0, 0.0, 2))           # vgx 2 = at least 0.15 m/s
        self.assertAlmostEqual(m.mismatch(), 0.15, places=2)

    def test_whole_dms_rounding_at_flight_speed_passes(self):
        # 0.3 m/s diagonal: vgx/vgy read (3, 1) while the true speed is 0.287 m/s (sim 11/09 false slam_drift)
        m = DriftMonitor()
        for i in range(60):
            t = i * 0.1
            m.add_pose(t, 0.25 * t, 0.14 * t)
            m.add_vps(t, 3, 1)
            self.assertFalse(m.drifting(t))
        self.assertEqual(m.mismatch(), 0.0)

    def test_curve_keeps_slam_speed(self):
        m = DriftMonitor()                                   # 0.3 m/s around a 0.5 m radius
        for i in range(60):
            t = i * 0.1
            a = 0.6 * t
            m.add_pose(t, 0.5 * math.sin(a), 0.5 * (1 - math.cos(a)))
            m.add_vps(t, round(3 * math.cos(a)), round(3 * math.sin(a)))
            self.assertFalse(m.drifting(t))
        self.assertAlmostEqual(m.slam_speed(), 0.3, delta=0.01)

    def test_agreement_and_short_spikes_pass(self):
        self.assertFalse(self.feed(DriftMonitor(), 6.0, 0.2, 2))   # flying 0.2 m/s, VPS 2 dm/s
        self.assertFalse(self.feed(DriftMonitor(), 6.0, 0.0, 0))   # hovering
        self.assertFalse(self.feed(DriftMonitor(), 6.0, 0.05, 0))  # small SLAM jitter below the limit
        m = DriftMonitor()
        self.assertFalse(self.feed(m, 2.0, 0.14, 0))               # not enough window yet

    def fly(self, monitor, seconds, slam_v, vps_v, slam_yaw, tello_yaw_deg):
        """10 Hz: SLAM moving at slam_v (map x, y) with yaw slam_yaw; Tello reading vps_v (m/s, its own frame)."""
        out = False
        for i in range(int(seconds * 10)):
            t = i * 0.1
            monitor.add_pose(t, slam_v[0] * t, slam_v[1] * t, slam_yaw)
            monitor.add_vps(t, round(vps_v[0] * 10), round(vps_v[1] * 10), tello_yaw_deg)
            out = monitor.drifting(t)
        return out

    @staticmethod
    def tello_vel(forward, side, yaw_deg, offset_deg=180.0):
        """Tello vgx/vgy (m/s) of a body velocity: forward points at yaw + offset in its frame."""
        h = math.radians(yaw_deg + offset_deg)
        return (forward * math.cos(h) - side * math.sin(h), forward * math.sin(h) + side * math.cos(h))

    def test_side_drift_while_flying_is_caught_with_yaw(self):
        # 0.3 m/s forward, SLAM also slides 0.15 m/s sideways (11/06 live: 0.14 m/s): speeds nearly agree (0.34 vs 0.3,
        # inside the rounding band), the sideways part does not. Below about 0.13 m/s the whole-dm/s rounding hides it.
        m = DriftMonitor()
        self.assertTrue(self.fly(m, 5.0, (0.3, 0.15), self.tello_vel(0.3, 0.0, -49.0), 0.0, -49.0))
        self.assertFalse(self.fly(DriftMonitor(), 5.0, (0.3, 0.15), self.tello_vel(0.3, 0.0, -49.0), 0.0, None))

    def test_flying_with_yaw_passes(self):
        for yaw_deg in (-87.0, -49.0, 6.0, 120.0):
            for offset in (172.0, 180.0, 186.0):   # logs: velocity angle - yaw 165-177 deg
                m = DriftMonitor()
                self.assertFalse(self.fly(m, 5.0, (0.3, 0.0), self.tello_vel(0.3, 0.0, yaw_deg, offset), 0.0, yaw_deg),
                                 (yaw_deg, offset, m.mismatch()))
                m = DriftMonitor()   # turned in the map too: SLAM heading 0.7 rad, flying along it
                self.assertFalse(self.fly(m, 5.0, (0.3 * math.cos(0.7), 0.3 * math.sin(0.7)),
                                          self.tello_vel(0.3, 0.0, yaw_deg, offset), 0.7, yaw_deg), (yaw_deg, offset))

    def test_no_vps_means_no_decision(self):
        m = DriftMonitor()
        for i in range(40):
            m.add_pose(i * 0.1, 0.2 * i * 0.1, 0.0)
        self.assertIsNone(m.mismatch())
        self.assertFalse(m.drifting(4.0))


class RcCompensationTests(unittest.TestCase):
    """develop gateway hello: k_lin 100, k_yaw 57, rc_limit 30 (rc = gain x cmd)."""

    def rc(self, comp, cmd):
        out = comp.apply(cmd)
        return (round(out["vx_mps"] * comp.k_lin), round(out["vy_mps"] * comp.k_lin), round(out["yaw_rate_rps"] * comp.k_yaw))

    def test_sends_the_rc_the_tello_needs(self):
        comp = RcCompensation(100.0, 57.0, 30)
        # 11/06: 0.1 m/s went out as rc 10 and the Tello did not move; now rc 11 + 75 x 0.12 = 20 (11/09 step flight)
        self.assertEqual(self.rc(comp, {"vx_mps": 0.12, "vy_mps": -0.04, "vz_mps": 0.0, "yaw_rate_rps": 0.0}), (20, -14, 0))
        self.assertEqual(self.rc(comp, {"vx_mps": 0.0, "vy_mps": 0.0, "vz_mps": 0.0, "yaw_rate_rps": 0.2}), (0, 0, 22))
        self.assertEqual(self.rc(comp, {"vx_mps": 1.0, "vy_mps": 0.0, "vz_mps": 0.0, "yaw_rate_rps": -1.0}), (30, 0, -30))

    def test_tiny_commands_and_limits(self):
        comp = RcCompensation(100.0, 57.0, 30)
        self.assertEqual(self.rc(comp, {"vx_mps": 0.005, "vy_mps": 0.0, "vz_mps": 0.0, "yaw_rate_rps": 0.01}), (0, 0, 0))
        self.assertAlmostEqual(comp.max_speed(), 19 / 75)
        self.assertAlmostEqual(comp.max_yaw_rate(), 24 / 82)

    def test_no_gains_means_unchanged(self):
        cmd = {"vx_mps": 0.1, "vy_mps": 0.0, "vz_mps": 0.0, "yaw_rate_rps": 0.0}
        comp = RcCompensation(None, 57.0, 30)
        self.assertFalse(comp.active)
        self.assertEqual(comp.apply(cmd), cmd)
        self.assertIsNone(comp.max_speed())


class RouteForwardTests(unittest.TestCase):
    def test_start_goal_picks_the_distance(self):
        self.assertEqual(route_forward_m({"forward_m": 2}, 5.0), 2.0)
        self.assertEqual(route_forward_m({"forward_m": 4.0}, 5.0), 4.0)

    def test_anything_else_keeps_the_default(self):
        for goal in (None, {}, {"forward_m": 0.2}, {"forward_m": 9}, {"forward_m": "4"}, {"forward_m": True},
                     {"x_m": 1.0}, [4]):
            self.assertEqual(route_forward_m(goal, 5.0), 5.0, goal)


class ScanYawTests(unittest.TestCase):
    def test_tello_yaw_is_clockwise_positive(self):
        self.assertAlmostEqual(tello_yaw_ccw(90.0), -math.pi / 2)    # turned right 90 deg = -90 deg CCW
        self.assertAlmostEqual(tello_yaw_ccw(-30.0), math.radians(30))

    def test_source_chosen_at_start_is_kept(self):
        # SLAM lost on a plain wall: the Tello yaw still steers
        self.assertEqual(scan_yaw("tello", 0.7, 0.1, None, 5.0), 0.7)
        self.assertIsNone(scan_yaw("tello", 0.7, 2.0, 0.3, 0.1))      # Tello state stale: stop, do not switch frames
        self.assertEqual(scan_yaw("slam", 0.7, 0.1, 0.3, 0.1), 0.3)
        self.assertIsNone(scan_yaw("slam", 0.7, 0.1, 0.3, 2.0))


class ScanTests(unittest.TestCase):
    def run_scan(self, start, dt=0.1, steps=600):
        scan, yaw, seen = Scan(start, 60.0), start, []
        for _ in range(steps):
            rate = scan.yaw_rate(yaw)
            self.assertLessEqual(abs(rate), 0.35 + 1e-9)
            if scan.done:
                break
            yaw = math.atan2(math.sin(yaw + rate * dt), math.cos(yaw + rate * dt))
            seen.append(yaw)
        return scan, yaw, seen

    def test_turns_left_right_and_back(self):
        scan, yaw, seen = self.run_scan(0.0)
        self.assertTrue(scan.done)
        self.assertAlmostEqual(max(seen), math.radians(60), delta=math.radians(4))
        self.assertAlmostEqual(min(seen), math.radians(-60), delta=math.radians(4))
        self.assertLess(abs(yaw), math.radians(3.5))
        self.assertLess(len(seen) * 0.1, 15.0)                  # about 11 s

    def test_wraps_around_pi(self):
        scan, yaw, seen = self.run_scan(math.radians(170))   # left target is -130 deg after wrapping
        self.assertTrue(scan.done)
        self.assertLess(abs(math.atan2(math.sin(yaw - math.radians(170)), math.cos(yaw - math.radians(170)))),
                        math.radians(3.5))

    def test_zero_angle_is_off(self):
        scan = Scan(0.3, 0.0)
        self.assertTrue(scan.done)
        self.assertEqual(scan.yaw_rate(1.0), 0.0)


class MissionTests(unittest.TestCase):
    def test_start_needs_a_pose_and_runs_once(self):
        mission = Mission()
        self.assertEqual(mission.control("start", False), (False, "pose_not_ready"))
        self.assertEqual(mission.control("start", True), (True, None))
        # a repeated start (gateway mission_start after -AutoStart) is accepted, not a refusal the gateway retries
        self.assertEqual(mission.control("start", True), (True, None))
        self.assertEqual(mission.message()["phase"], "running")
        mission.control("pause", True)
        self.assertEqual(mission.control("start", True), (False, "already_running"))

    def test_start_refused_when_slam_origin_is_off_flight_height(self):
        mission = Mission()
        self.assertEqual(mission.control("start", True, 0.45), (False, "origin_off_flight_height"))
        self.assertEqual(mission.control("start", True, -0.3), (False, "origin_off_flight_height"))
        self.assertEqual(mission.control("start", True, 0.1), (True, None))
        self.assertEqual(mission.control("start", True, 0.45), (True, None))   # already running: repeat accepted

    def test_pause_resume_stop(self):
        mission = Mission()
        self.assertEqual(mission.control("pause", True), (False, "not_running"))
        mission.control("start", True)
        self.assertEqual(mission.control("pause", True), (True, None))
        self.assertEqual(mission.control("resume", True), (True, None))
        self.assertEqual(mission.control("stop", True), (True, None))
        self.assertEqual(mission.message()["phase"], "stopped")
        self.assertEqual(mission.control("dance", True), (False, "unknown_cmd"))

    def test_path_leaving_flight_height_is_not_flown(self):
        mission = Mission()
        mission.control("start", True)
        mission.goal = [4.0, 0.0, 0.1]
        # around the side with the smoothing bulge seen in replay (up to ~0.46 m): flown
        self.assertTrue(mission.path_ok([[0.0, 0.0, 0.1], [2.0, -2.4, 0.56], [4.0, 0.0, 0.1]]))
        self.assertFalse(mission.path_ok([[0.0, 0.0, 0.1], [2.0, 0.0, 1.9], [4.0, 0.0, 0.1]]))    # over the top
        self.assertFalse(mission.path_ok([[0.0, 0.0, 0.1], [2.0, 0.0, -0.6], [4.0, 0.0, 0.1]]))   # under
        self.assertEqual(mission.phase, "running")             # a bad plan alone does not end the route
        mission.fail("path_leaves_flight_height")
        msg = mission.message()
        self.assertEqual((msg["phase"], msg["reason"]), ("failed", "path_leaves_flight_height"))
        self.assertTrue(Mission().path_ok([[0.0, 0.0, 9.0]]))   # no goal yet

    def test_slam_height_drift_does_not_block_level_plans(self):
        # hand-held 11/09: 4 m level walk, ToF constant, SLAM z +0.6 m by the end; plans start at the drifted height
        mission = Mission()
        mission.control("start", True)
        mission.goal = [4.0, 0.0, 0.0]
        self.assertTrue(mission.path_ok([[3.0, 0.0, 0.62], [3.5, 0.0, 0.62], [4.0, 0.0, 0.62]]))
        self.assertFalse(mission.path_ok([[3.0, 0.0, 0.62], [3.5, 0.0, 1.3], [4.0, 0.0, 0.62]]))   # over, from there

    def test_plan_swinging_far_off_the_route_line_is_not_flown(self):
        mission = Mission()
        mission.control("start", True)
        mission.start, mission.goal = [0.0, 0.0, 0.0], [4.0, 0.0, 0.0]
        self.assertTrue(mission.path_ok([[0.0, 0.0, 0.0], [2.0, -2.9, 0.0], [4.0, 0.0, 0.0]]))   # replay: 1.8-2.9 m
        self.assertFalse(mission.path_ok([[0.0, 0.0, 0.0], [2.0, -6.0, 0.0], [4.0, 0.0, 0.0]]))  # replay: 6 m
        narrow = Mission(side_limit=1.5)                                                       # -MaxSideM 1.5
        narrow.control("start", True)
        narrow.start, narrow.goal = [0.0, 0.0, 0.0], [4.0, 0.0, 0.0]
        self.assertFalse(narrow.path_ok([[0.0, 0.0, 0.0], [2.0, -1.9, 0.0], [4.0, 0.0, 0.0]]))
        mission.goal = [0.0, 4.0, 0.0]                                                         # route along +y
        self.assertAlmostEqual(mission.side_offset([1.5, 2.0, 0.0]), 1.5)
        self.assertTrue(Mission().path_ok([[0.0, 9.0, 0.0]]))                                  # no route yet

    def test_plan_to_a_goal_moved_far_is_not_flown(self):
        mission = Mission()
        mission.control("start", True)
        mission.start, mission.goal = [0.0, 0.0, 0.0], [3.0, 0.0, 0.0]
        self.assertTrue(mission.path_ok([[0.0, 0.0, 0.0], [2.0, -1.0, 0.0], [3.4, -0.6, 0.0]]))   # moved 0.7 m
        self.assertTrue(mission.path_ok([[0.0, 0.0, 0.0], [1.0, -0.9, 0.0], [2.04, -1.23, 0.0]]))  # short of goal
        self.assertFalse(mission.path_ok([[0.0, 0.0, 0.0], [2.0, 1.5, 0.0], [3.0, 2.5, 0.0]]))    # sim: moved 2.5 m
        self.assertFalse(mission.path_ok([[0.0, 0.0, 0.0], [3.0, 0.2, 0.0], [4.8, 0.2, 0.0]]))    # 1.8 m past goal
        self.assertAlmostEqual(mission.along([1.5, 9.0, 0.0]), 1.5)

    def test_plan_going_behind_the_start_is_not_flown(self):
        mission = Mission()
        mission.control("start", True)
        mission.start, mission.goal = [0.0, 0.0, 0.0], [2.0, 0.0, 0.0]
        # usual avoidance plans start with a small dip back (sim: -0.07 m; after the look-around -0.37 m near the line)
        self.assertTrue(mission.path_ok([[0.0, 0.0, 0.0], [-0.07, -0.3, 0.0], [1.0, -1.2, 0.0], [2.4, -0.3, 0.0]]))
        self.assertTrue(mission.path_ok([[0.0, 0.0, 0.0], [-0.37, -0.26, 0.0], [-0.33, -0.45, 0.0], [1.1, -0.85, 0.0]]))
        # stepping aside just behind the start (sim person 1 m: -0.14 m back, 0.85 m sideways)
        self.assertTrue(mission.path_ok([[0.0, 0.0, 0.0], [-0.14, -0.63, 0.0], [-0.05, -0.85, 0.0], [1.0, -1.15, 0.0]]))
        self.assertTrue(mission.path_ok([[0.0, 0.0, 0.0], [-0.44, -0.3, 0.0], [-0.35, -1.3, 0.0], [1.0, -1.2, 0.0]]))
        # sim 3 m corridor replan: back 0.96 m (directly behind, never seen)
        self.assertFalse(mission.path_ok([[0.0, 0.0, 0.0], [-0.96, -0.43, 0.0], [2.4, -0.25, 0.0]]))
        self.assertFalse(mission.path_ok([[0.0, 0.0, 0.0], [-0.8, 0.0, 0.0], [2.0, 0.0, 0.0]]))      # 0.8 m back
        # sim 2.5 m corridor: back 0.7 m and round the outside of a side wall never seen
        self.assertFalse(mission.path_ok([[0.0, 0.0, 0.0], [-0.66, -1.1, 0.0], [-0.2, -2.3, 0.0], [2.4, -0.2, 0.0]]))

    def test_no_progress_fails_the_route(self):
        mission = Mission()
        mission.control("start", True)
        mission.goal = [4.0, 0.0, 0.0]
        mission.check_progress((1.0, 1.0, 0.0), 0.0)
        mission.check_progress((1.05, 1.0, 0.0), 10.0)       # 5 cm: not progress
        self.assertEqual(mission.phase, "running")
        mission.check_progress((1.05, 1.02, 0.0), 15.5)
        self.assertEqual((mission.phase, mission.reason), ("failed", "no_progress"))
        moving = Mission()
        moving.control("start", True)
        for i in range(40):                                    # 0.2 m per second for 40 s
            moving.check_progress((0.2 * i, 0.0, 0.0), float(i))
        self.assertEqual(moving.phase, "running")

    def test_arrival_at_a_goal_the_planner_moved(self):
        mission = Mission()
        mission.control("start", True)
        mission.goal = [4.0, 0.1, 0.0]
        mission.update((5.1, -0.8, 0.0), path_end=(5.15, -0.85, 0.0))       # replay: goal moved 1.46 m
        self.assertEqual((mission.phase, mission.reason), ("arrived", "moved_goal_reached"))
        far = Mission()
        far.control("start", True)
        far.goal = [4.0, 0.1, 0.0]
        far.update((5.6, -1.7, 0.0), path_end=(5.65, -1.71, 0.0))           # moved 2.4 m: not this route's goal
        self.assertEqual(far.phase, "running")
        far.update((3.0, 0.0, 0.0), path_end=(5.15, -0.85, 0.0))            # not at the end yet
        self.assertEqual(far.phase, "running")

    def test_arrival_within_radius(self):
        mission = Mission()
        mission.control("start", True)
        mission.goal = [5.0, 0.0, 0.0]
        mission.update((4.0, 0.0, 0.0))
        self.assertEqual(mission.phase, "running")
        mission.update((4.8, 0.1, 0.0))
        self.assertEqual(mission.message()["phase"], "arrived")
        self.assertEqual(mission.message()["goal"], {"x_m": 5.0, "y_m": 0.0, "z_m": 0.0, "frame": "map"})


SCREEN = "http://127.0.0.1:8010"


def drone_state(connected=True, video=None, mode="grounded"):
    return {"type": "drone_state", "source": "tello", "connected": connected, "mode": mode,
            "video": video or {"enabled": True, "on": False, "ground_s": None}}


class GatewayCheckTests(unittest.TestCase):
    def test_screen_url_is_what_the_operator_opens(self):
        self.assertEqual(screen_url("http://host.docker.internal:8010/"), SCREEN)

    def test_verdicts(self):
        cases = (
            ((None, None, None), "unreachable"),
            # 11/05: an old check_server.py tab/server on the same port answers /api/status, not /api/drone/state
            ((404, None, 200), "check_server"),
            ((404, None, 404), "not_gateway"),
            ((200, {"type": "status"}, None), "not_gateway"),
            ((200, drone_state(connected=False), None), "drone_disconnected"),
            ((200, drone_state(), None), "ready"),
        )
        for (status, state, probe), verdict in cases:
            got, message = classify_gateway(status, state, probe, SCREEN)
            self.assertEqual(got, verdict, message)
            self.assertTrue(message)
        self.assertIn("check_server.py", classify_gateway(404, None, 200, SCREEN)[1])

    def test_video_action_names_the_missing_step(self):
        self.assertIsNone(video_action(drone_state(), 0.5, SCREEN))
        self.assertIn("waiting for the drone", video_action(drone_state(connected=False), None, SCREEN))
        self.assertIn("--no-video", video_action(drone_state(video={"enabled": False}), None, SCREEN))
        # on the ground video off is normal before a mission (11/06: pressing video-on early broke two routes)
        off = video_action(drone_state(), None, SCREEN)
        self.assertIn("normal before a mission", off)
        self.assertIn("only for a hand-held check", off)
        self.assertIn(SCREEN, off)
        self.assertIn("60 s", video_action(drone_state(), 8.0, SCREEN))  # had frames, gateway turned video off
        climbing = dict(drone_state(mode="manual"), sequence={"stage": "climb", "land_on_finish": True})
        self.assertIn("do not press video-on", video_action(climbing, None, SCREEN))
        flying = video_action(drone_state(mode="manual"), None, SCREEN)
        self.assertIn("video is off", flying)
        self.assertIn(SCREEN, flying)
        self.assertIn("no frames yet", video_action(drone_state(video={"enabled": True, "on": True}), None, SCREEN))

    def test_status_line(self):
        line = status_line(12.3, drone_state(video={"on": True, "ground_s": 41.0}), {"video_published": 80,
                           "tof_published": 70}, 0.1, True, "running", SCREEN)
        self.assertIn("video on 41/60s on ground", line)
        self.assertIn("frames 80", line)
        self.assertNotIn("->", line)
        line = status_line(3, None, {}, None, False, "idle", SCREEN)
        self.assertIn("gateway unreachable", line)
        self.assertIn("-> waiting for the drone", line)


if __name__ == "__main__":
    unittest.main()
