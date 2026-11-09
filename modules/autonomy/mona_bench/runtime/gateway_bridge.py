#!/usr/bin/env python3
"""Live link between MoNA (ROS 1 in this container) and the drone gateway's /ws/autonomy (AUTONOMY_BRIDGE 0.1.0).

In:  Tello state from the WebSocket (bottom ToF) and frames from GET /snapshot.jpg -> /tello/tof_btm, /tello/image_raw.
Out: MoNA's candidate velocity (map frame, PID output) rotated into the body frame -> `cmd_vel`, plus pose, path and
     mission_state for the gateway map.

The gateway alone talks to the Tello and forwards `cmd_vel` only in `auto` mode, which the operator starts after an
operator take-off. This node never takes off, lands or asks for an emergency stop. While the drone is held by hand
(grounded), commands are computed, sent and logged by the gateway as ignored; nothing flies. The gateway's start is only
available while flying, so `~autostart` starts the route on the first pose for that hand-held bench check.
"""
import json
import math
import threading
import time

SCHEMA = "0.1.0"
TOF_MIN_CM, TOF_MAX_CM = 10, 600       # same validity rule as the recorder (grounded 0-10 cm is not a height)
ARRIVE_M = 0.3
PATH_HEIGHT_LIMIT_M = 0.55             # plan leaving flight height by more is not flown (PID flies level only);
                                       # planner ceiling/floor are +-0.5 m, the B-spline smoothing can bulge past them
PATH_BAD_LIMIT_S = 5.0                 # only such plans for this long -> route failed (gateway hovers, then lands)
PATH_SIDE_LIMIT_M = 3.0                # plan farther than this from the straight route line is not flown either
                                       # (~max_side_m; replays: going round a 0.5 m obstacle took 1.8-2.9 m)
GOAL_MOVED_MAX_M = 1.5                 # planner may move the goal this far; reaching that end counts as arrived,
                                       # and a plan ending farther from the goal is not flown
NO_PROGRESS_M, NO_PROGRESS_S = 0.1, 15.0   # running but not moving this far for this long -> route failed
BACK_LIMIT_M = 0.5                     # plan going farther behind the route start is not flown: the camera never saw
                                       # behind (the +-90 deg look-around maps the sides). Sim 11/08, 2.5 m corridor:
                                       # back 0.7-1.2 m round a wall it had not seen -> collision before this rule;
                                       # normal avoidance dips back 0.14-0.44 m.
MAX_SPEED_MPS = 0.3                    # horizontal cap; the gateway hello lowers it to the real reachable speed
SCAN_LIMIT_S = 45.0                    # look-around must finish in this time (+-90 deg at 0.24 rad/s takes about 30 s)
# The develop gateway turns cmd_vel into rc with assumed gains (hello k_lin 100, k_yaw 57, rc_limit 30), but the Tello
# does not move below about rc 12. rc step flight 11/09 (indoor, mission pads under the drone, Tello VPS, mean of each
# 2.5 s step after 1 s): forward rc 10/12 -> 0.01 m/s, 15 -> 0.07, 20 -> 0.10, 25 -> 0.19, 30 -> 0.26 (fit
# rc = 10.9 + 74.5 v); yaw rc 10 -> 3.7 deg/s, 15 -> 8.2, 20 -> 8.7, 25 -> 13.8, 30 -> 15.3 (fit 6.2 + 81.8 rad/s).
# 11/06 the route's 0.1 m/s commands became rc 10 and
# the drone never left after takeoff. Until the gateway is calibrated (S15P31B205-274) the bridge sends the speed
# that the gateway's gains turn into the rc the Tello needs: rc = start rc + slope x speed.
TELLO_RC_MIN_LIN, TELLO_K_LIN = 11, 75.0     # horizontal: rc at which it starts to move, rc per m/s above it
TELLO_RC_MIN_YAW, TELLO_K_YAW = 6, 82.0      # yaw: rc per rad/s above the start rc
MIN_CMD_MPS, MIN_CMD_RPS = 0.01, 0.02        # smaller commands are sent as 0 (no rc jumps at hover)
ORIGIN_HEIGHT_TOL_M = 0.25           # route start z (map) must be near 0: the planner ceiling/floor (+-0.5 m)
                                       # are relative to where SLAM started, which must be the flight height
CMD_FRESH_S = 0.3                      # PID output older than this -> hover (0) instead of an old command
POSE_FRESH_S = 0.5


def parse_tello_raw(raw):
    """'mid:-1;x:-100;...;tof:62;h:0;' -> {'mid': -1, ..., 'tof': 62, 'h': 0}; non-numeric values stay text."""
    out = {}
    for part in (raw or "").strip().strip(";").split(";"):
        if ":" not in part:
            continue
        key, value = part.split(":", 1)
        try:
            out[key] = float(value) if "," not in value else value
        except ValueError:
            out[key] = value
    return out


def bottom_tof_m(fields):
    tof = fields.get("tof")
    if not isinstance(tof, float) or not math.isfinite(tof) or not TOF_MIN_CM < tof < TOF_MAX_CM:
        return None
    return tof / 100.0


def yaw_of(qx, qy, qz, qw):
    return math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


def map_to_body(vx, vy, yaw):
    """Rotate a horizontal map-frame velocity into the body frame (x forward, y left)."""
    c, s = math.cos(yaw), math.sin(yaw)
    return c * vx + s * vy, -s * vx + c * vy


def clamp_xy(vx, vy, limit):
    speed = math.hypot(vx, vy)
    if limit <= 0:
        return 0.0, 0.0
    if speed > limit:
        return vx * limit / speed, vy * limit / speed
    return vx, vy


class Mission:
    """Phases follow the gateway's mission_state: idle, running, paused, arrived, stopped, failed."""

    def __init__(self, side_limit=PATH_SIDE_LIMIT_M):
        self.phase, self.state, self.reason, self.goal = "idle", "WAIT_POSE", None, None
        self.start = None   # route start position (map), for the sideways limit
        self.side_limit = side_limit
        self._progress = None   # ((x, y), time) of the last NO_PROGRESS_M step

    def control(self, cmd, pose_ready, start_z=None):
        if cmd == "start":
            if not pose_ready:
                return False, "pose_not_ready"
            if self.phase != "running" and start_z is not None and abs(start_z) > ORIGIN_HEIGHT_TOL_M:
                # 11/06: video was turned on during take-off, SLAM started 0.45 m below flight height and every
                # plan hit the ceiling. Refuse; the gateway retries, then lands.
                return False, "origin_off_flight_height"
            if self.phase == "running":
                # Same route already running (e.g. -AutoStart before the gateway's mission_start): accept, so the
                # gateway does not count it as a refusal, retry and finally land.
                return True, None
            if self.phase == "paused":
                return False, "already_running"
            self.phase, self.state, self.reason = "running", "FOLLOW", None
            return True, None
        if cmd == "pause":
            if self.phase != "running":
                return False, "not_running"
            self.phase, self.state = "paused", "PAUSED"
            return True, None
        if cmd == "resume":
            if self.phase != "paused":
                return False, "not_paused"
            self.phase, self.state = "running", "FOLLOW"
            return True, None
        if cmd == "stop":
            self.phase, self.state, self.reason = "stopped", "STOPPED", "operator_stop"
            return True, None
        return False, "unknown_cmd"

    def update(self, position, path_end=None):
        """Arrived near the goal, or near the end of the current plan when the planner moved the goal a little
        (its goal relocation when the goal is close to an obstacle; replay 11/06: (4.0, 0.1) -> (5.2, -0.9))."""
        if self.phase != "running" or self.goal is None or position is None:
            return
        if math.hypot(self.goal[0] - position[0], self.goal[1] - position[1]) < ARRIVE_M:
            self.phase, self.state, self.reason = "arrived", "ARRIVED", "goal_reached"
        elif (path_end is not None
              and math.hypot(path_end[0] - self.goal[0], path_end[1] - self.goal[1]) <= GOAL_MOVED_MAX_M
              and math.hypot(path_end[0] - position[0], path_end[1] - position[1]) < ARRIVE_M):
            self.phase, self.state, self.reason = "arrived", "ARRIVED", "moved_goal_reached"

    def side_offset(self, point):
        """Distance of a point from the straight route line (start -> goal), horizontal."""
        if self.start is None or self.goal is None:
            return 0.0
        sx, sy = self.start[0], self.start[1]
        dx, dy = self.goal[0] - sx, self.goal[1] - sy
        length = math.hypot(dx, dy)
        if length < 1e-6:
            return math.hypot(point[0] - sx, point[1] - sy)
        return abs((point[0] - sx) * dy - (point[1] - sy) * dx) / length

    def path_ok(self, points):
        """Plans that must not be flown; the caller hovers until a replan that is fine.
        - leaving the flight height (over or under an obstacle): the PID has no vertical speed, so it would fly
          the plan level and hit what the plan went around;
        - swinging more than side_limit off the route line: unseen space counts as free for the planner,
          and replays swung up to 6 m sideways, through where side walls would be indoors;
        - ending more than GOAL_MOVED_MAX_M to the side of the route line or past the goal: the planner moved the
          goal far (sim: through a side wall into unseen space), so it is no longer this route. A plan that stops
          short of a blocked goal (sim: (2.0, -1.2) for (3, 0)) is fine; the planner moves the goal and replans."""
        if self.goal is None or not points:
            return True
        if max(abs(p[2] - self.goal[2]) for p in points) > PATH_HEIGHT_LIMIT_M:
            return False
        end = points[-1]
        if self.side_offset(end) > GOAL_MOVED_MAX_M or self.along(end) > self.along(self.goal) + GOAL_MOVED_MAX_M:
            return False
        if self.start is not None and min(self.along(p) for p in points) < -BACK_LIMIT_M:
            return False   # behind the start: never seen by the forward camera
        return max(self.side_offset(p) for p in points) <= self.side_limit

    def along(self, point):
        """Distance of a point along the route line from the start (negative behind it)."""
        if self.start is None or self.goal is None:
            return 0.0
        dx, dy = self.goal[0] - self.start[0], self.goal[1] - self.start[1]
        length = math.hypot(dx, dy)
        if length < 1e-6:
            return 0.0
        return ((point[0] - self.start[0]) * dx + (point[1] - self.start[1]) * dy) / length

    def check_progress(self, position, now):
        """Fail a running route that has not moved NO_PROGRESS_M for NO_PROGRESS_S (stuck hovering, e.g. facing a
        wall: sim 11/06 hovered 39 s 0.15 m from one). The gateway then hovers and lands."""
        if self.phase != "running" or position is None:
            self._progress = None
            return
        if self._progress is None or math.hypot(position[0] - self._progress[0][0],
                                                position[1] - self._progress[0][1]) >= NO_PROGRESS_M:
            self._progress = ((position[0], position[1]), now)
        elif now - self._progress[1] > NO_PROGRESS_S:
            self.fail("no_progress")

    def fail(self, reason):
        if self.phase == "running":
            self.phase, self.state, self.reason = "failed", "FAILED", reason

    def message(self):
        goal = None if self.goal is None else {"x_m": self.goal[0], "y_m": self.goal[1], "z_m": self.goal[2], "frame": "map"}
        return {"type": "mission_state", "state": self.state, "phase": self.phase, "goal": goal,
                "progress": None, "reason": self.reason}


class DriftMonitor:
    """SLAM speed vs the Tello's own speed (downward-camera VPS, vgx/vgy in dm/s, so 0.1 m/s steps).

    11/06 flight: SLAM moved 0.55 m forward in 4 s while vgx stayed 0 and the drone visibly did not move; the
    controller then flies to correct a motion that did not happen (sim: 0.06-0.08 m/s drift pushed the drone 0.8-1 m
    the other way, to 0.2 m from a wall). Frames differ, so only speeds are compared, averaged over WINDOW_S.

    The Tello reports each axis in whole dm/s, so a moving reading only says the speed lies in a band (vgx 3 = 0.25 to
    0.35 m/s). At 0.3 m/s that rounding alone reached the old 0.08 m/s limit (sim 11/09: 3 m corridor, no drift ->
    slam_drift). The mismatch is the distance from the SLAM speed to that band; an axis reading 0 is taken as 0, so a
    hovering drone keeps the full check. SLAM speed is the mean over 1 s chords, so curves and slowdowns in the window
    do not shorten it.

    Speed alone misses a drift across the flight direction (0.3 m/s forward + 0.08 m/s sideways is 0.31 m/s; sim 11/09
    hit a wall). The Tello's vgx/vgy turn with its yaw (flight logs 11/03: flying forward, velocity angle - yaw stayed
    165-177 deg), so with both yaws the sideways part (relative to the heading) is compared too. Only its size is used,
    which does not depend on that 180 deg offset or on the two yaw zeros."""

    # 0.06: with the rounding band a hovering 0.08 m/s side drift sat exactly on 0.08 and was missed (sim 11/09).
    # Stationary SLAM jitter in replays was about 0.006 m/s.
    WINDOW_S, LIMIT_MPS, HOLD_S, CHORD_S = 2.5, 0.06, 1.0, 1.0
    VEC_BAND_MPS = 0.07          # whole dm/s on both axes: up to 0.05 each, about 0.07 on a rotated component
    TELLO_HEADING_DEG = 180.0    # Tello: velocity angle (atan2(vgy, vgx)) of flying forward = yaw + about 180

    def __init__(self):
        self.poses, self.vps, self._over_since = [], [], None

    def add_pose(self, t, x, y, yaw=None):
        self.poses.append((t, x, y, yaw))
        self.poses = [p for p in self.poses if t - p[0] <= self.WINDOW_S]

    def add_vps(self, t, vgx_dms, vgy_dms, yaw_deg=None):
        lo = [max(abs(v) - 0.5, 0.0) if v else 0.0 for v in (vgx_dms, vgy_dms)]
        hi = [abs(v) + 0.5 if v else 0.0 for v in (vgx_dms, vgy_dms)]
        side = None
        if yaw_deg is not None:
            h = math.radians(yaw_deg + self.TELLO_HEADING_DEG)
            side = 0.1 * abs(vgx_dms * math.sin(h) - vgy_dms * math.cos(h))
            band = self.VEC_BAND_MPS if (vgx_dms or vgy_dms) else 0.0
            side = (max(side - band, 0.0), side + band)
        self.vps.append((t, 0.1 * math.hypot(*lo), 0.1 * math.hypot(*hi), side))
        self.vps = [v for v in self.vps if t - v[0] <= self.WINDOW_S]

    def slam_speeds(self):
        """Mean speed and mean sideways speed (None without yaw) over CHORD_S chords, or (None, None)."""
        speeds, sides, j = [], [], 0
        for t, x, y, yaw in self.poses:
            while j + 1 < len(self.poses) and self.poses[j + 1][0] <= t - self.CHORD_S:
                j += 1
            t0, x0, y0, _ = self.poses[j]
            if t - t0 >= 0.8 * self.CHORD_S:
                vx, vy = (x - x0) / (t - t0), (y - y0) / (t - t0)
                speeds.append(math.hypot(vx, vy))
                if yaw is not None:
                    sides.append(abs(vx * math.sin(yaw) - vy * math.cos(yaw)))
        if not speeds:
            return None, None
        return sum(speeds) / len(speeds), (sum(sides) / len(sides) if len(sides) == len(speeds) else None)

    def slam_speed(self):
        return self.slam_speeds()[0]

    def mismatch(self):
        """Distance (m/s) from SLAM to the VPS band (speed, and sideways when both yaws are known), or None."""
        if len(self.poses) < 5 or len(self.vps) < 10 or self.poses[-1][0] - self.poses[0][0] < 0.8 * self.WINDOW_S:
            return None
        slam, slam_side = self.slam_speeds()
        if slam is None:
            return None
        n = len(self.vps)
        lo, hi = sum(v[1] for v in self.vps) / n, sum(v[2] for v in self.vps) / n
        out = max(0.0, slam - hi, lo - slam)
        if slam_side is not None and all(v[3] is not None for v in self.vps):
            side_lo, side_hi = sum(v[3][0] for v in self.vps) / n, sum(v[3][1] for v in self.vps) / n
            out = max(out, slam_side - side_hi, side_lo - slam_side)
        return out

    def drifting(self, now):
        """True once the mismatch has stayed above LIMIT_MPS for HOLD_S."""
        m = self.mismatch()
        if m is None or m <= self.LIMIT_MPS:
            self._over_since = None
            return False
        if self._over_since is None:
            self._over_since = now
        return now - self._over_since >= self.HOLD_S

    def reset(self):
        self.poses, self.vps, self._over_since = [], [], None


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class Scan:
    """Look around before the route: yaw to start+angle, start-angle, back to start, hovering (no horizontal speed).

    The camera sees about +-27 deg; the planner treats unseen space as free, so indoors it planned the way round a
    person into a side wall it had never seen (sim 11/08, 3 m corridor: 0.25 m from the wall). Turning +-60 deg first
    puts walls up to about +-87 deg in the map before the first plan. Yaw is counter-clockwise positive (map frame)."""

    TOL = math.radians(3.0)

    def __init__(self, start_yaw, angle_deg=90.0, rate=0.35, gain=1.5):
        a = math.radians(angle_deg)
        self.targets = [wrap_angle(start_yaw + a), wrap_angle(start_yaw - a), wrap_angle(start_yaw)] if a > 0 else []
        self.rate, self.gain = rate, gain
        self.done = not self.targets

    def yaw_rate(self, yaw):
        """Yaw rate (rad/s) toward the current target; advances through the targets; 0 when done."""
        while self.targets:
            err = wrap_angle(self.targets[0] - yaw)
            if abs(err) > self.TOL:
                return math.copysign(max(0.1, min(self.rate, self.gain * abs(err))), err)
            self.targets.pop(0)
        self.done = True
        return 0.0


class RcCompensation:
    """Real Tello speed -> the cmd_vel the gateway's linear rc mapping (hello k_lin, k_yaw, rc_limit) turns into the rc
    the Tello needs. Without gains in the hello (another gateway) commands go out unchanged."""

    def __init__(self, k_lin=None, k_yaw=None, rc_limit=None):
        ok = all(isinstance(v, (int, float)) and math.isfinite(v) and v > 0 for v in (k_lin, k_yaw, rc_limit))
        self.active = ok
        self.k_lin, self.k_yaw, self.rc_limit = (float(k_lin), float(k_yaw), float(rc_limit)) if ok else (None, None, None)

    def max_speed(self):
        """Highest real horizontal speed per axis (m/s) the gateway's rc limit allows, or None."""
        return (self.rc_limit - TELLO_RC_MIN_LIN) / TELLO_K_LIN if self.active else None

    def max_yaw_rate(self):
        return (self.rc_limit - TELLO_RC_MIN_YAW) / TELLO_K_YAW if self.active else None

    def _axis(self, v, rc_min, slope, eps, gain):
        if abs(v) < eps:
            return 0.0
        rc = min(rc_min + slope * abs(v), self.rc_limit)
        return round(math.copysign(rc / gain, v), 3)

    def apply(self, cmd):
        if not self.active:
            return cmd
        return dict(cmd,
                    vx_mps=self._axis(cmd["vx_mps"], TELLO_RC_MIN_LIN, TELLO_K_LIN, MIN_CMD_MPS, self.k_lin),
                    vy_mps=self._axis(cmd["vy_mps"], TELLO_RC_MIN_LIN, TELLO_K_LIN, MIN_CMD_MPS, self.k_lin),
                    yaw_rate_rps=self._axis(cmd["yaw_rate_rps"], TELLO_RC_MIN_YAW, TELLO_K_YAW, MIN_CMD_RPS, self.k_yaw))


def body_command(candidate, candidate_age_s, pose_age_s, yaw, limit):
    """cmd_vel fields to send while running: the rotated, clamped PID output, or hover with the reason."""
    if candidate is None or candidate_age_s > CMD_FRESH_S:
        return {"vx_mps": 0.0, "vy_mps": 0.0, "vz_mps": 0.0, "yaw_rate_rps": 0.0}, "no_fresh_command"
    if yaw is None or pose_age_s > POSE_FRESH_S:
        return {"vx_mps": 0.0, "vy_mps": 0.0, "vz_mps": 0.0, "yaw_rate_rps": 0.0}, "pose_stale"
    bx, by = clamp_xy(*map_to_body(candidate[0], candidate[1], yaw), limit)
    return {"vx_mps": round(bx, 3), "vy_mps": round(by, 3), "vz_mps": 0.0, "yaw_rate_rps": 0.0}, None


def stamp_lag(header_stamp_s, now_s):
    return now_s - header_stamp_s


class Stats:
    def __init__(self):
        self.lock = threading.Lock()
        self.counts = {}
        self.lags = {}

    def count(self, key, n=1):
        with self.lock:
            self.counts[key] = self.counts.get(key, 0) + n

    def lag(self, key, value):
        with self.lock:
            self.lags.setdefault(key, []).append(value)

    def summary(self):
        with self.lock:
            lags = {}
            for key, values in self.lags.items():
                ordered = sorted(values)
                pick = lambda q: round(ordered[min(len(ordered) - 1, int(q * len(ordered)))], 3)  # noqa: E731
                lags[key] = dict(n=len(ordered), median_s=pick(0.5), p90_s=pick(0.9), max_s=round(ordered[-1], 3))
            return dict(counts=dict(self.counts), lags=lags)


def screen_url(base):
    """The gateway screen as the operator opens it on this PC (the container reaches it via host.docker.internal)."""
    return base.rstrip("/").replace("host.docker.internal", "127.0.0.1")


def classify_gateway(state_status, state, status_probe, screen):
    """Is `base` the drone gateway with a drone attached? -> (verdict, message).

    state_status: HTTP status of GET /api/drone/state (None if unreachable); state: its JSON or None;
    status_probe: HTTP status of GET /api/status (check_server.py's API) or None.
    The 11/05 live try lost its video because an old check_server.py tab on the same port was used: its buttons
    hit the gateway as 404s and nothing reached the drone. That case is named here so it is not a silent no-op.
    """
    if state_status is None:
        return "unreachable", f"no server at {screen}; start gateway_server.py first"
    if state_status == 404 and status_probe == 200:
        return "check_server", (f"{screen} is check_server.py, not the gateway; stop it, start gateway_server.py "
                                "and reopen the tab")
    if state_status != 200 or not isinstance(state, dict) or state.get("type") != "drone_state":
        return "not_gateway", f"{screen} does not answer /api/drone/state (HTTP {state_status}); is it the gateway?"
    if not state.get("connected"):
        return "drone_disconnected", (f"gateway up (source {state.get('source')}) but no drone state; "
                                      "power the drone and join its Wi-Fi")
    return "ready", f"gateway ready: source {state.get('source')}, mode {state.get('mode')}"


def video_action(state, frames_s, screen):
    """What the operator must do for video, or None when frames are arriving.

    frames_s: seconds since the bridge last published a frame (None if never)."""
    if frames_s is not None and frames_s < 2.0:
        return None
    if not isinstance(state, dict) or not state.get("connected"):
        return "waiting for the drone (gateway has no drone state)"
    video = state.get("video") or {}
    if video.get("enabled") is False:
        return "gateway runs with --no-video; restart it without that flag"
    seq = state.get("sequence") or {}
    if not video.get("on") and seq.get("stage") in ("takeoff", "climb"):
        return "mission climbing; video turns on by itself at flight height (do not press video-on)"
    if not video.get("on") and state.get("mode") in ("grounded", "disconnected"):
        # Before take-off this is normal: the gateway's mission start turns video on at flight height, and video
        # turned on earlier moves this node's origin below flight height (11/06: two routes failed that way).
        if frames_s is not None:
            return (f"video turned off (60 s on the ground); for a hand-held check press video-on in {screen} again")
        return ("video off (normal before a mission: mission start turns it on at flight height; "
                f"press video-on in {screen} only for a hand-held check)")
    if not video.get("on"):
        hint = " (it turns off by itself after 60 s on the ground)" if frames_s is not None else ""
        return f"video is off{hint}: reload {screen} in the browser, then press its video-on button"
    return "video is on but no frames yet; wait a few seconds or toggle video off/on"


def status_line(elapsed_s, state, counts, frames_s, pose_ok, mission_phase, screen):
    video = (state or {}).get("video") or {}
    ground = video.get("ground_s")
    parts = [f"gateway {(state or {}).get('mode', 'unreachable')}",
             f"video {'on' if video.get('on') else 'off'}" + (f" {ground:.0f}/60s on ground" if ground else ""),
             f"frames {counts.get('video_published', 0)}",
             f"tof {counts.get('tof_published', 0)}",
             f"pose {'ok' if pose_ok else 'none'}",
             f"route {mission_phase}"]
    action = video_action(state, frames_s, screen)
    return f"[{elapsed_s:4.0f}s] " + " | ".join(parts) + (f"\n        -> {action}" if action else "")


def fetch_json(url, timeout=2.0):
    """GET -> (status, json or None); status None when nothing answers."""
    from urllib.error import HTTPError, URLError
    from urllib.request import urlopen
    try:
        with urlopen(url, timeout=timeout) as response:
            body = response.read()
            status = response.status
    except HTTPError as error:
        return error.code, None
    except (URLError, OSError, ValueError):
        return None, None
    try:
        return status, json.loads(body)
    except ValueError:
        return status, None


def check_gateway(base):
    base = base.rstrip("/")
    status, state = fetch_json(base + "/api/drone/state")
    probe = fetch_json(base + "/api/status")[0] if status == 404 else None
    verdict, message = classify_gateway(status, state, probe, screen_url(base))
    return verdict, message, state


def check_main(argv):
    """`gateway_bridge.py --check URL [--wait S] [--out FILE]`: wait until the gateway has a drone (exit 0) or say why not
    (exit 4). Runs before roscore so a wrong tab or server costs seconds, not a whole live run."""
    import argparse
    parser = argparse.ArgumentParser(prog="gateway_bridge.py --check")
    parser.add_argument("--check", required=True, metavar="URL")
    parser.add_argument("--wait", type=float, default=60.0)
    parser.add_argument("--out")
    args = parser.parse_args(argv)
    deadline, last = time.monotonic() + max(0.0, args.wait), None
    while True:
        verdict, message, state = check_gateway(args.check)
        if message != last:
            print(("OK   " if verdict == "ready" else "WAIT ") + message, flush=True)
            last = message
        if verdict == "ready" or time.monotonic() >= deadline:
            break
        time.sleep(1.0)
    if verdict == "ready":
        action = video_action(state, None, screen_url(args.check))
        if action:
            print("NEXT " + action, flush=True)
    if args.out:
        with open(args.out, "w") as handle:
            json.dump({"gateway": args.check, "verdict": verdict, "message": message,
                       "source": (state or {}).get("source"), "mode": (state or {}).get("mode"),
                       "video": (state or {}).get("video")}, handle, indent=2)
    return 0 if verdict == "ready" else 4


def ros_main():
    from pathlib import Path
    from urllib.request import urlopen

    import cv2
    import numpy as np
    import rospy
    import websocket
    from cv_bridge import CvBridge
    from geometry_msgs.msg import PoseStamped, Twist
    from nav_msgs.msg import Odometry, Path as PathMsg
    from sensor_msgs.msg import Image, Range

    from send_goal import forward_goal

    rospy.init_node("gateway_bridge")
    base = rospy.get_param("~gateway", "http://host.docker.internal:8010").rstrip("/")
    video_hz = float(rospy.get_param("~video_hz", 10.0))
    cmd_hz = float(rospy.get_param("~cmd_hz", 10.0))
    forward_m = float(rospy.get_param("~goal_forward_m", 5.0))
    summary_path = rospy.get_param("~summary", "")
    status_path = rospy.get_param("~status", "")    # one line per second for live.sh to show on the console
    # Keep the inputs exactly as published (gateway JPEG bytes + bottom ToF, same ROS stamps) so a live flight can be
    # turned into a replay bag later (inputs_to_bag.py) and planner changes checked on the real flight's images.
    inputs_dir = rospy.get_param("~inputs_dir", "")
    if inputs_dir:
        Path(inputs_dir, "frames").mkdir(parents=True, exist_ok=True)
        tof_log = open(Path(inputs_dir, "tof.csv"), "a", buffering=1)
    else:
        tof_log = None
    screen = screen_url(base)
    # Bench option: start the route as soon as SLAM gives a pose, without the gateway's start (which needs a
    # flying, manual-mode drone). Safe because the gateway flies cmd_vel only in auto mode.
    autostart = bool(rospy.get_param("~autostart", False))
    scan_deg = float(rospy.get_param("~scan_deg", 90.0))   # look-around before the route; 0 turns it off
    bridge = CvBridge()
    stats = Stats()
    mission = Mission(side_limit=float(rospy.get_param("~max_side_m", PATH_SIDE_LIMIT_M)))
    drift = DriftMonitor()
    drift_guard = bool(rospy.get_param("~drift_guard", True))
    rc_comp_on = bool(rospy.get_param("~rc_compensation", True))   # see TELLO_RC_MIN_LIN
    # develop gateway: an arrived/failed route leaves the drone hovering (paused); ask it to land (autonomy request)
    land_on_finish = bool(rospy.get_param("~land_on_finish", True))
    lock = threading.Lock()
    max_speed = float(rospy.get_param("~max_speed", MAX_SPEED_MPS))
    shared = dict(ws=None, limit=max_speed, mode=None, odom=None, odom_t=None, cmd=None, cmd_t=None, path=None, path_sent=None,
                  frame_t=None, gateway_state=None, video_seen=False, path_bad_since=None, scan=None,
                  route_start=None, scan_t=0.0, rc=RcCompensation(), scan_rate=0.35, land_sent=False)
    image_pub = rospy.Publisher("/tello/image_raw", Image, queue_size=1)
    tof_pub = rospy.Publisher("/tello/tof_btm", Range, queue_size=5)
    goal_pub = rospy.Publisher("/waypoint_generator/waypoints", PathMsg, queue_size=1, latch=True)

    def send(msg):
        with lock:
            ws = shared["ws"]
            if ws is None:
                return False
            try:
                ws.send(json.dumps({"schema_version": SCHEMA, "at": time.strftime("%Y-%m-%dT%H:%M:%S"), **msg}))
                return True
            except Exception as error:  # reconnect loop picks it up
                rospy.logwarn_throttle(5, "gateway send failed: %s", error)
                shared["ws"] = None
                return False

    def pose_ready():
        return shared["odom"] is not None and time.monotonic() - shared["odom_t"] < POSE_FRESH_S

    def begin_route():
        """Route accepted: remember the start pose, look around (Scan), then publish the goal from the start pose."""
        odom = shared["odom"]
        q = odom.pose.pose.orientation
        shared["route_start"], shared["scan_t"] = odom, time.monotonic()
        shared["land_sent"] = False
        shared["scan"] = Scan(yaw_of(q.x, q.y, q.z, q.w), scan_deg, rate=shared["scan_rate"])
        mission.state = "SCAN" if not shared["scan"].done else "FOLLOW"
        if shared["scan"].done:
            publish_goal(odom)
        else:
            rospy.loginfo("looking around +-%.0f deg before the route", scan_deg)

    def publish_goal(odom):
        p, q = odom.pose.pose.position, odom.pose.pose.orientation
        goal, _ = forward_goal((p.x, p.y, p.z), (q.x, q.y, q.z, q.w), forward_m)
        message = PathMsg()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "map"
        pose = PoseStamped()
        pose.header = message.header
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = goal
        pose.pose.orientation.w = 1.0
        message.poses.append(pose)
        goal_pub.publish(message)
        mission.goal = [round(v, 3) for v in goal]
        mission.start = [round(p.x, 3), round(p.y, 3), round(p.z, 3)]
        rospy.loginfo("route goal %.2f m ahead at %s (map)", forward_m, mission.goal)

    def on_gateway(msg):
        kind = msg.get("type")
        if kind == "hello":
            limit = msg.get("max_speed_mps")
            if isinstance(limit, (int, float)) and math.isfinite(limit) and limit > 0:
                shared["limit"] = min(max_speed, float(limit))  # never above the planner speed
            rc = RcCompensation(msg.get("k_lin"), msg.get("k_yaw"), msg.get("rc_limit")) if rc_comp_on else RcCompensation()
            shared["rc"] = rc
            if rc.active:   # the real speed the rc limit allows, not the gateway's assumed max_speed_mps
                shared["limit"] = min(max_speed, rc.max_speed())
                shared["scan_rate"] = min(0.35, rc.max_yaw_rate())
            rospy.loginfo("gateway hello: source=%s max_speed=%s k_lin=%s rc_limit=%s -> real limit %.2f m/s, "
                          "look-around %.2f rad/s, rc compensation %s", msg.get("source"), msg.get("max_speed_mps"),
                          msg.get("k_lin"), msg.get("rc_limit"), shared["limit"], shared["scan_rate"],
                          "on" if rc.active else "off")
        elif kind == "tello_state":
            stats.count("tello_state")
            fields = parse_tello_raw(msg.get("raw"))
            if isinstance(fields.get("vgx"), float) and isinstance(fields.get("vgy"), float):
                yaw = fields.get("yaw")
                drift.add_vps(time.monotonic(), fields["vgx"], fields["vgy"], yaw if isinstance(yaw, float) else None)
            height = bottom_tof_m(fields)
            if height is None:
                stats.count("tof_invalid")
                return
            out = Range()
            out.header.stamp = rospy.Time.now()
            out.header.frame_id = "base_link"
            out.radiation_type = Range.INFRARED
            out.min_range, out.max_range, out.range = 0.1, 6.0, height
            tof_pub.publish(out)
            stats.count("tof_published")
            if tof_log is not None:
                tof_log.write("%d,%.3f\n" % (out.header.stamp.to_nsec(), height))
        elif kind == "flight_status":
            if msg.get("mode") != shared["mode"]:
                rospy.loginfo("gateway mode: %s -> %s", shared["mode"], msg.get("mode"))
            shared["mode"] = msg.get("mode")
        elif kind == "control":
            was_running = mission.phase == "running"
            odom = shared["odom"]
            ok, reason = mission.control(msg.get("cmd"), pose_ready(),
                                         None if odom is None else odom.pose.pose.position.z)
            if ok and msg.get("cmd") == "start" and not was_running:
                begin_route()
            rospy.loginfo("control %s -> %s", msg.get("cmd"), "ok" if ok else reason)
            send({"type": "ack", "id": msg.get("id"), "ok": ok, "reason": reason})
        elif kind == "request_result":
            rospy.loginfo("gateway request %s -> %s %s", msg.get("name"), "ok" if msg.get("ok") else "refused",
                          msg.get("reason") or "")
        elif kind == "error":
            rospy.logwarn("gateway error: %s", msg)

    def ws_loop():
        url = base.replace("http://", "ws://").replace("https://", "wss://") + "/ws/autonomy"
        while not rospy.is_shutdown():
            try:
                ws = websocket.create_connection(url, timeout=5)
                with lock:
                    shared["ws"] = ws
                send({"type": "hello", "node": "mona_bench_gateway_bridge", "version": "0.1"})
                rospy.loginfo("connected to %s", url)
                stats.count("ws_connect")
                ws.settimeout(2)
                while not rospy.is_shutdown() and shared["ws"] is ws:
                    try:
                        raw = ws.recv()
                    except websocket.WebSocketTimeoutException:
                        continue
                    if not raw:
                        break
                    on_gateway(json.loads(raw))
            except Exception as error:
                rospy.logwarn_throttle(5, "gateway link down (%s); retrying", error)
            with lock:
                shared["ws"] = None
            time.sleep(1.0)

    def video_loop():
        period, last_id = 1.0 / video_hz, None
        while not rospy.is_shutdown():
            tick = time.monotonic()
            try:
                with urlopen(base + "/snapshot.jpg", timeout=1.0) as response:
                    frame_id = response.headers.get("X-Frame-Id")
                    payload = response.read()
                if frame_id is not None and frame_id == last_id:
                    stats.count("video_duplicate")
                else:
                    image = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_COLOR)
                    if image is None:
                        stats.count("video_bad_jpeg")
                    else:
                        out = bridge.cv2_to_imgmsg(image, "bgr8")
                        out.header.stamp = rospy.Time.now()
                        out.header.frame_id = "camera_link"
                        image_pub.publish(out)
                        stats.count("video_published")
                        if inputs_dir:
                            Path(inputs_dir, "frames", "%d.jpg" % out.header.stamp.to_nsec()).write_bytes(payload)
                        last_id = frame_id
                        shared["frame_t"], shared["video_seen"] = time.monotonic(), True
                stats.lag("video_http_s", time.monotonic() - tick)
            except Exception as error:
                stats.count("video_unavailable")
                rospy.logwarn_throttle(10, "no video from gateway (%s); turn video on in %s", error, screen)
            time.sleep(max(0.0, period - (time.monotonic() - tick)))

    def gateway_state_loop():
        # The drone_state the operator screen sees (mode, video on/off, ground seconds) for the console status line.
        while not rospy.is_shutdown():
            status, state = fetch_json(base + "/api/drone/state", timeout=1.0)
            shared["gateway_state"] = state if status == 200 else None
            shared["gateway_polled"] = True
            time.sleep(1.0)

    def on_odom(msg):
        shared["odom"], shared["odom_t"] = msg, time.monotonic()
        q = msg.pose.pose.orientation
        drift.add_pose(shared["odom_t"], msg.pose.pose.position.x, msg.pose.pose.position.y, yaw_of(q.x, q.y, q.z, q.w))
        stats.lag("image_to_odom_s", stamp_lag(msg.header.stamp.to_sec(), rospy.Time.now().to_sec()))

    def on_candidate(msg):
        shared["cmd"], shared["cmd_t"] = (msg.linear.x, msg.linear.y), time.monotonic()

    def on_bspline(msg):
        q = [(p.x, p.y, p.z) for p in msg.pos_pts]
        shared["path"] = [[round((a[i] + 4 * b[i] + c[i]) / 6, 3) for i in range(3)] for a, b, c in zip(q, q[1:], q[2:])]
        stats.count("plans")
        ok = mission.path_ok(shared["path"])
        if not ok:
            stats.count("plans_off_height")
            rospy.logwarn("plan leaves flight height (> %.2f m) or the route line (> %.1f m sideways); hovering "
                          "until a replan", PATH_HEIGHT_LIMIT_M, mission.side_limit)
        if ok:
            shared["path_bad_since"] = None
        elif shared["path_bad_since"] is None:
            shared["path_bad_since"] = time.monotonic()

    rospy.Subscriber("/sync/tf_odom", Odometry, on_odom, queue_size=5)
    rospy.Subscriber("/mona/candidate_cmd_vel", Twist, on_candidate, queue_size=5)
    try:
        from plan_manage.msg import Bspline
        rospy.Subscriber("/planning/bspline", Bspline, on_bspline, queue_size=2)
    except ImportError:
        rospy.logwarn("bspline message not available; path is not shown on the gateway map")

    threading.Thread(target=ws_loop, daemon=True).start()
    threading.Thread(target=video_loop, daemon=True).start()
    threading.Thread(target=gateway_state_loop, daemon=True).start()

    def write_status(started):
        if not status_path or not shared.get("gateway_polled"):   # no line until the gateway was asked once
            return
        frame_t = shared["frame_t"]
        line = status_line(time.monotonic() - started, shared["gateway_state"], stats.summary()["counts"],
                           None if frame_t is None else time.monotonic() - frame_t, pose_ready(), mission.phase, screen)
        try:
            Path(status_path + ".tmp").write_text(line + "\n")
            Path(status_path + ".tmp").replace(status_path)
        except OSError as error:
            rospy.logwarn_throttle(30, "status file: %s", error)

    seq, last_state, state_sent = 0, None, 0.0
    started, status_t = time.monotonic(), 0.0
    rate = rospy.Rate(cmd_hz)
    while not rospy.is_shutdown():
        now = time.monotonic()
        odom = shared["odom"]
        position = yaw = None
        if odom is not None:
            p, q = odom.pose.pose.position, odom.pose.pose.orientation
            position, yaw = (p.x, p.y, p.z), yaw_of(q.x, q.y, q.z, q.w)
            send({"type": "pose", "x_m": round(p.x, 3), "y_m": round(p.y, 3), "z_m": round(p.z, 3),
                  "yaw_rad": round(yaw, 3), "sigma_m": None, "frame": "map"})
        if autostart and mission.phase == "idle" and pose_ready():
            mission.control("start", True)
            begin_route()
            rospy.loginfo("autostart: route started without a gateway start (bench mode)")
        scan = shared["scan"]
        scanning = mission.phase == "running" and scan is not None and not scan.done
        if not scanning:
            mission.update(position, shared["path"][-1] if shared["path"] else None)
            mission.check_progress(position, now)
        bad_since = shared["path_bad_since"]
        if mission.phase == "running" and bad_since is not None and now - bad_since > PATH_BAD_LIMIT_S:
            mission.fail("no_flyable_plan")
            rospy.logwarn("only off-height or off-route plans for %.0f s; route failed", PATH_BAD_LIMIT_S)
        # also while looking around: the drone hovers there, which is where the guard sees drift best
        if mission.phase == "running" and drift_guard and drift.drifting(now):
            mission.fail("slam_drift")
            rospy.logwarn("SLAM speed and the Tello's own speed disagree by %.2f m/s; route failed", drift.mismatch())
            stats.count("slam_drift_stop")
        if scanning and now - shared["scan_t"] > SCAN_LIMIT_S:   # e.g. pose lost while turning
            mission.fail("scan_timeout")
            rospy.logwarn("look-around did not finish in %.0f s; route failed", SCAN_LIMIT_S)
            scanning = False
        if scanning:
            fresh = yaw is not None and now - (shared["odom_t"] or -1e9) <= POSE_FRESH_S
            rate_cmd = scan.yaw_rate(yaw) if fresh else 0.0
            if scan.done:   # back facing the start direction: the map has the sides now, plan the route
                mission.state = "FOLLOW"
                publish_goal(shared["route_start"])
                rospy.loginfo("look-around done")
            stats.count("scan_ticks")
        if mission.phase == "running" and scanning:
            cmd = {"vx_mps": 0.0, "vy_mps": 0.0, "vz_mps": 0.0, "yaw_rate_rps": round(rate_cmd, 3)}
            hold = None if fresh else "pose_stale"
            seq += 1
            if send({"type": "cmd_vel", **shared["rc"].apply(cmd), "frame": "body", "seq": seq}):
                stats.count("cmd_sent")
                stats.count("cmd_scan" if fresh else "cmd_hold_pose_stale")
        elif mission.phase == "running":
            cmd, hold = body_command(shared["cmd"], now - (shared["cmd_t"] or -1e9),
                                     now - (shared["odom_t"] or -1e9), yaw, shared["limit"])
            if bad_since is not None:   # current plan goes over/under something: hover until a level replan
                cmd, hold = {"vx_mps": 0.0, "vy_mps": 0.0, "vz_mps": 0.0, "yaw_rate_rps": 0.0}, "path_off_height"
            seq += 1
            if send({"type": "cmd_vel", **shared["rc"].apply(cmd), "frame": "body", "seq": seq}):
                stats.count("cmd_sent")
                stats.count("cmd_hold_" + hold if hold else "cmd_moving")
        if shared["path"] is not None and shared["path"] is not shared["path_sent"]:
            if send({"type": "path", "points": shared["path"], "frame": "map"}):
                shared["path_sent"] = shared["path"]
        state = mission.message()
        if state != last_state or now - state_sent >= 1.0:
            send(state)
            last_state, state_sent = state, now
        if land_on_finish and mission.phase in ("arrived", "failed") and not shared["land_sent"]:
            if send({"type": "request", "name": "land", "id": "mona-land-%d" % seq}):   # after the mission_state
                shared["land_sent"] = True
                rospy.loginfo("route %s (%s): asked the gateway to land", mission.phase, mission.reason)
        if now - status_t >= 1.0:
            write_status(started)
            status_t = now
        rate.sleep()

    if summary_path:
        final = shared["gateway_state"]
        Path(summary_path).write_text(json.dumps(dict(stats.summary(), mission=mission.message(),
                                                      last_gateway_mode=shared["mode"],
                                                      video_ever=shared["video_seen"],
                                                      last_video_action=video_action(
                                                          final, None if shared["frame_t"] is None
                                                          else time.monotonic() - shared["frame_t"], screen),
                                                      note="cmd_vel goes to the gateway, which flies it only in auto mode"),
                                                 indent=2))


if __name__ == "__main__":
    import sys
    if "--check" in sys.argv[1:]:
        sys.exit(check_main(sys.argv[1:]))
    ros_main()
