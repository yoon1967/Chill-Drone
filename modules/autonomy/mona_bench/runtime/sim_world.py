#!/usr/bin/env python3
"""Closed-loop avoidance check: the real planner, trajectory server and PID (planner.launch) fly a simulated drone
through a box world. No camera, SLAM or depth network: the depth they would produce is rendered exactly, so this
checks planning, control and the bridge's plan guards, not perception.

World (map frame = MoNA's: x forward, y left, z up, origin at the start at flight height, floor at -height_m):
boxes are axis-aligned [x0, x1, y0, y1, z0, z1] in metres. The camera sits on the drone, looks along +x pitched
down by camera_pitch_deg (MoNA assumes 10 deg), 640x480 with the planner's intrinsics, and publishes metric depth
(0 = nothing within range) on /sync/depth_metric for the floor filter, like the live pipeline.

The drone follows /mona/candidate_cmd_vel (map frame) with a first-order lag, clamped like the bridge (~max_speed = plan_max_vel);
plans the bridge would not fly (Mission.path_ok) make it hover, and only such plans for 5 s fail the route.
Pose reaches the planner with a delay like SLAM (~0.1 s). Result JSON: arrival, time, closest approach to a box
(drone centre to box surface, horizontal and at flight height), collision (< drone_radius_m), path.
"""
import json
import math
import time

import numpy as np

FX, FY, CX, CY = 631.304382, 643.417053, 320.982631, 243.577051   # planner.launch map intrinsics
W, H = 640, 480


def camera_rotation(yaw, pitch_down):
    """Rotation map <- camera optical frame (x right, y down, z forward) for a body at `yaw` looking pitch_down."""
    cy, sy, cp, sp = math.cos(yaw), math.sin(yaw), math.cos(pitch_down), math.sin(pitch_down)
    forward = np.array([cy * cp, sy * cp, -sp])
    left = np.array([-sy, cy, 0.0])
    up = np.cross(forward, left)
    return np.column_stack([-left, -up, forward])   # columns: optical x (right), y (down), z (forward)


def render_depth(position, rotation, boxes, floor_z, max_range=6.0, stride=1):
    """Optical-axis depth (m) per pixel from the floor plane and boxes; 0 where nothing is within max_range."""
    us, vs = np.meshgrid(np.arange(0, W, stride), np.arange(0, H, stride))
    rays_c = np.stack([(us - CX) / FX, (vs - CY) / FY, np.ones_like(us, dtype=float)], axis=-1)
    rays = rays_c @ rotation.T                                   # map-frame direction with unit optical z
    o = np.asarray(position, dtype=float)
    t_best = np.full(rays.shape[:2], np.inf)
    dz = rays[..., 2]
    with np.errstate(divide="ignore", invalid="ignore"):
        t_floor = (floor_z - o[2]) / dz
    t_best = np.where((dz < -1e-9) & (t_floor > 0), np.minimum(t_best, t_floor), t_best)
    for x0, x1, y0, y1, z0, z1 in boxes:
        lo, hi = np.array([x0, y0, z0]), np.array([x1, y1, z1])
        with np.errstate(divide="ignore", invalid="ignore"):
            inv = 1.0 / rays
            t1, t2 = (lo - o) * inv, (hi - o) * inv
        tmin = np.nanmax(np.minimum(t1, t2), axis=-1)
        tmax = np.nanmin(np.maximum(t1, t2), axis=-1)
        hit = (tmax >= np.maximum(tmin, 0.0)) & np.isfinite(tmin)
        t_best = np.where(hit & (tmin > 0), np.minimum(t_best, tmin), t_best)
    depth = np.where(np.isfinite(t_best), t_best, 0.0)          # unit optical z: t is the depth
    depth[(depth > max_range) | (depth < 0)] = 0.0
    if stride > 1:   # computed on a coarser grid for speed, handed over at full size like a camera frame
        depth = np.repeat(np.repeat(depth, stride, axis=0), stride, axis=1)[:H, :W]
    return depth.astype(np.float32)


def clearance(point_xy, boxes, z, radius=0.0):
    """Horizontal distance from the drone centre to the nearest box that spans height z (inf if none)."""
    best = math.inf
    for x0, x1, y0, y1, z0, z1 in boxes:
        if not (z0 - radius <= z <= z1 + radius):
            continue
        dx = max(x0 - point_xy[0], 0.0, point_xy[0] - x1)
        dy = max(y0 - point_xy[1], 0.0, point_xy[1] - y1)
        best = min(best, math.hypot(dx, dy))
    return best


def cross_track(point_xy, path):
    """Horizontal distance from the drone to the current plan (polyline of its sampled points)."""
    best = math.inf
    px, py = point_xy
    for (ax, ay, _), (bx, by, _) in zip(path, path[1:]):
        dx, dy = bx - ax, by - ay
        length2 = dx * dx + dy * dy
        t = 0.0 if length2 < 1e-12 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length2))
        best = min(best, math.hypot(px - (ax + t * dx), py - (ay + t * dy)))
    if len(path) == 1:
        best = math.hypot(px - path[0][0], py - path[0][1])
    return best


class Drone:
    """Horizontal first-order velocity response; height and yaw stay (the PID sends neither)."""

    def __init__(self, tau=0.3, limit=0.2, gain=1.0):
        self.p = np.zeros(3)
        self.gain = gain   # real speed / commanded (rc calibration error: Tello slower or faster than asked)
        self.v = np.zeros(3)
        self.yaw, self.yaw_rate = 0.0, 0.0
        self.tau, self.limit = tau, limit

    def step(self, cmd_xy, dt, yaw_rate_cmd=0.0):
        a = min(1.0, dt / self.tau)
        self.yaw_rate += (yaw_rate_cmd - self.yaw_rate) * a
        self.yaw = math.atan2(math.sin(self.yaw + self.yaw_rate * dt), math.cos(self.yaw + self.yaw_rate * dt))
        self.step_xy(cmd_xy, dt)

    def step_xy(self, cmd_xy, dt):
        cx, cy = cmd_xy
        speed = math.hypot(cx, cy)
        if speed > self.limit:
            cx, cy = cx * self.limit / speed, cy * self.limit / speed
        cx, cy = cx * self.gain, cy * self.gain
        a = min(1.0, dt / self.tau)
        self.v[0] += (cx - self.v[0]) * a
        self.v[1] += (cy - self.v[1]) * a
        self.p[:2] += self.v[:2] * dt


def ros_main():
    from collections import deque
    from pathlib import Path

    import rospy
    from cv_bridge import CvBridge
    from geometry_msgs.msg import PoseStamped, Twist
    from nav_msgs.msg import Odometry, Path as PathMsg
    from sensor_msgs.msg import Image

    from gateway_bridge import DriftMonitor, Mission, PATH_BAD_LIMIT_S, Scan

    rospy.init_node("sim_world")
    scenario = json.loads(Path(rospy.get_param("~scenario_file")).read_text())
    name = scenario["name"]
    height = float(scenario.get("height_m", 1.0))
    boxes = [list(map(float, b)) for b in scenario.get("boxes", [])]
    goal_forward = float(scenario.get("goal_forward_m", 4.0))
    duration = float(scenario.get("duration_s", 70.0))
    pitch = math.radians(float(scenario.get("camera_pitch_deg", 10.0)))
    delay = float(scenario.get("pose_delay_s", 0.1))
    radius = float(scenario.get("drone_radius_m", 0.15))
    side = float(scenario.get("max_side_m", 3.0))
    scan_deg = float(scenario.get("scan_deg", 0.0))   # look-around before the route (bridge ~scan_deg)
    depth_scale = float(scenario.get("depth_scale", 1.0))   # perception bias: >1 sees things farther than they are
    # SLAM error: the planner, PID and guards see the estimate; collisions are judged on the true position.
    drift = np.array(list(scenario.get("pose_drift_mps", [0.0, 0.0])) + [0.0], dtype=float)  # steady drift (m/s)
    drift_from_start = bool(scenario.get("drift_from_start", False))   # drift already during the look-around
    pose_noise = float(scenario.get("pose_noise_m", 0.0))     # per-pose jitter (m, 1 sigma)
    depth_noise = float(scenario.get("depth_noise", 0.0))     # per-pixel relative depth noise (1 sigma)
    rng = np.random.default_rng(int(scenario.get("seed", 7)))
    est_offset = np.zeros(3)
    drift_mon = DriftMonitor() if scenario.get("drift_guard", True) else None
    out_path = rospy.get_param("~result")
    floor_z = -height

    bridge = CvBridge()
    depth_pub = rospy.Publisher("/sync/depth_metric", Image, queue_size=2)
    cam_pub = rospy.Publisher("/sync/camera_pose", PoseStamped, queue_size=5)
    odom_pub = rospy.Publisher("/sync/tf_odom", Odometry, queue_size=5)
    goal_pub = rospy.Publisher("/waypoint_generator/waypoints", PathMsg, queue_size=1, latch=True)
    state = dict(cmd=(0.0, 0.0), cmd_t=-1e9, plans=0, plans_bad=0, path=None, bad_since=None, bad_plans=[],
                 plan_shapes=[], t0=time.monotonic(), plan_min_clear=math.inf, xtrack=[])
    mission = Mission(side_limit=side)

    def on_cmd(msg):
        state["cmd"], state["cmd_t"] = (msg.linear.x, msg.linear.y), time.monotonic()

    def on_bspline(msg):
        q = [(p.x, p.y, p.z) for p in msg.pos_pts]
        pts = [[(a[i] + 4 * b[i] + c[i]) / 6 for i in range(3)] for a, b, c in zip(q, q[1:], q[2:])]
        state["path"], state["plans"] = pts, state["plans"] + 1
        plan_clear = min((clearance(p[:2], boxes, 0.0) for p in pts), default=math.inf)
        state["plan_min_clear"] = min(state["plan_min_clear"], plan_clear)
        if len(state["plan_shapes"]) < 12:
            state["plan_shapes"].append(dict(t=round(time.monotonic() - state["t0"], 1), ctrl=len(q),
                                             pts=[[round(p[0], 2), round(p[1], 2)] for p in pts[::2]]))
        if mission.path_ok(pts):
            state["bad_since"] = None
        else:
            state["plans_bad"] += 1
            state["bad_plans"].append(dict(
                max_dz=round(max(abs(p[2] - (mission.goal or [0, 0, 0])[2]) for p in pts), 3),
                max_side=round(max(mission.side_offset(p) for p in pts), 3),
                end=[round(v, 2) for v in pts[-1]]))
            if state["bad_since"] is None:
                state["bad_since"] = time.monotonic()

    rospy.Subscriber("/mona/candidate_cmd_vel", Twist, on_cmd, queue_size=5)
    try:
        from plan_manage.msg import Bspline
        rospy.Subscriber("/planning/bspline", Bspline, on_bspline, queue_size=5)
    except ImportError:
        rospy.logwarn("no Bspline message; plan guards not applied")

    drone = Drone(limit=float(rospy.get_param("~max_speed", 0.2)), gain=float(scenario.get("speed_gain", 1.0)))
    pending = deque()   # (release time, stamp, position, yaw, depth) to delay pose like SLAM

    def publish_pose(stamp, pos, yaw):
        cam = PoseStamped()
        cam.header.stamp, cam.header.frame_id = stamp, "map"
        cam.pose.position.x, cam.pose.position.y, cam.pose.position.z = pos
        q_cam = rotation_to_quaternion(camera_rotation(yaw, pitch))
        cam.pose.orientation.x, cam.pose.orientation.y, cam.pose.orientation.z, cam.pose.orientation.w = q_cam
        cam_pub.publish(cam)
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id, odom.child_frame_id = stamp, "map", "body_link"
        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = pos
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        odom.twist.twist.linear.x, odom.twist.twist.linear.y = drone.v[0], drone.v[1]
        odom_pub.publish(odom)

    rate_hz, depth_every = 20.0, 2           # physics 20 Hz, depth + pose 10 Hz
    rate = rospy.Rate(rate_hz)
    rospy.sleep(3.0)                         # let the planner subscribe
    t0 = time.monotonic()
    tick = 0
    goal_sent = False
    scan, scan_s = None, None
    track, min_clear, closest = [], math.inf, None
    hover_ticks = moving_ticks = 0
    result = None
    last_step, loop_dts = None, []
    while not rospy.is_shutdown():
        now = time.monotonic()
        elapsed = now - t0
        stamp = rospy.Time.now()
        if tick % depth_every == 0:
            # 4-pixel blocks: full 640x480 took 0.42 s per frame for a room (loop fell to ~2 Hz)
            depth = render_depth(drone.p.copy(), camera_rotation(drone.yaw, pitch), boxes, floor_z,
                                 stride=4) * np.float32(depth_scale)
            if depth_noise > 0:
                depth = (depth * (1.0 + depth_noise * rng.standard_normal(depth.shape))).astype(np.float32)
                depth[depth < 0] = 0
            img = bridge.cv2_to_imgmsg(depth, "32FC1")
            img.header.stamp, img.header.frame_id = stamp, "camera_link"
            est = drone.p + est_offset + (pose_noise * rng.standard_normal(3) * np.array([1, 1, 0])
                                          if pose_noise > 0 else 0.0)
            pending.append((now + delay, stamp, tuple(est), drone.yaw, img))
        while pending and pending[0][0] <= now:
            _, s, pos, yaw_seen, img = pending.popleft()
            depth_pub.publish(img)
            publish_pose(s, pos, yaw_seen)
            if drift_mon is not None:
                drift_mon.add_pose(now, pos[0], pos[1], yaw_seen)
        if drift_mon is not None and tick % depth_every == 0:   # Tello VPS: true velocity in 0.1 m/s steps
            # Tello convention: velocity angle of flying forward = yaw + 180 deg (yaw given so that it holds)
            drift_mon.add_vps(now, round(drone.v[0] * 10), round(drone.v[1] * 10),
                              math.degrees(drone.yaw) - DriftMonitor.TELLO_HEADING_DEG)
        if scan is None and elapsed > 2.0:   # route start: look around first (same Scan as the bridge)
            mission.control("start", True, 0.0)
            scan = Scan(0.0, scan_deg, rate=0.29)   # develop gateway rc limit 30: about 0.29 rad/s (bridge rc compensation)
        scanning = scan is not None and not scan.done and mission.phase == "running"
        yaw_cmd = scan.yaw_rate(drone.yaw) if scanning else 0.0
        if scan is not None and scan.done and not goal_sent:
            goal = PathMsg()
            goal.header.stamp, goal.header.frame_id = stamp, "map"
            g = PoseStamped()
            g.header = goal.header
            g.pose.position.x, g.pose.position.y, g.pose.position.z = goal_forward, 0.0, 0.0
            g.pose.orientation.w = 1.0
            goal.poses.append(g)
            goal_pub.publish(goal)
            mission.goal, mission.start = [goal_forward, 0.0, 0.0], [0.0, 0.0, 0.0]
            goal_sent = True
            scan_s = round(elapsed - 2.0, 1)
        if goal_sent:
            est_now = tuple(drone.p + est_offset)   # the bridge only knows the SLAM estimate
            mission.update(est_now, state["path"][-1] if state["path"] else None)
            mission.check_progress(est_now, now)
        if drift_mon is not None and mission.phase == "running" and drift_mon.drifting(now):
            mission.fail("slam_drift")   # also while looking around, like the bridge
        if (mission.phase == "running" and state["bad_since"] is not None
                and now - state["bad_since"] > PATH_BAD_LIMIT_S):
            mission.fail("no_flyable_plan")
        cmd = (0.0, 0.0)
        if scanning:
            pass                                 # hovering while looking around, like the bridge
        elif mission.phase == "running" and now - state["cmd_t"] < 0.3 and state["bad_since"] is None:
            cmd = state["cmd"]
            moving_ticks += 1
        elif mission.phase == "running":
            hover_ticks += 1
        # Real elapsed time: the planner's trajectory runs on the clock, so a slow loop must not slow the drone
        # (11/06 sim: rendering four boxes made the loop slow, the drone lagged and cut a corner).
        step = min(0.2, now - last_step) if last_step is not None else 1.0 / rate_hz
        last_step = now
        loop_dts.append(step)
        drone.step(cmd, step, yaw_cmd)
        if goal_sent or (drift_from_start and mission.phase == "running"):
            est_offset += drift * step          # SLAM drift accumulates while the route runs (or from the look-around)
        if mission.phase == "running" and state["path"]:
            state["xtrack"].append(cross_track((drone.p + est_offset)[:2], state["path"]))
        c = clearance(drone.p[:2], boxes, 0.0)
        if c < min_clear:
            min_clear, closest = c, [round(v, 3) for v in drone.p[:2]]
        if tick % 10 == 0:
            track.append([round(elapsed, 1), round(drone.p[0], 3), round(drone.p[1], 3)])
        if mission.phase in ("arrived", "failed") or elapsed > duration:
            result = dict(
                scenario=name, phase=mission.phase, reason=mission.reason or ("timeout" if elapsed > duration else None),
                time_s=round(elapsed - 2.0, 1), final=[round(v, 3) for v in drone.p[:2]],
                min_clearance_m=None if math.isinf(min_clear) else round(min_clear, 3), closest_at=closest,
                collision=bool(min_clear < radius), max_side_m=round(max(abs(p[2]) for p in track), 3) if track else 0,
                plans=state["plans"], plans_not_flown=state["plans_bad"], not_flown=state["bad_plans"][:20],
                plan_shapes=state["plan_shapes"], scan_s=scan_s,
                slam_offset_end_m=[round(v, 3) for v in est_offset[:2]],
                plan_min_clearance_m=None if math.isinf(state["plan_min_clear"]) else round(state["plan_min_clear"], 3),
                cross_track_max_m=round(max(state["xtrack"]), 3) if state["xtrack"] else None,
                cross_track_mean_m=round(sum(state["xtrack"]) / len(state["xtrack"]), 3) if state["xtrack"] else None,
                loop_hz=round(len(loop_dts) / max(1e-6, sum(loop_dts)), 1) if loop_dts else None,
                loop_dt_max_s=round(max(loop_dts), 3) if loop_dts else None,
                hover_share=round(hover_ticks / max(1, hover_ticks + moving_ticks), 3), boxes=boxes, track=track)
            break
        tick += 1
        rate.sleep()
    if result is not None:
        Path(out_path).write_text(json.dumps(result, indent=2))
        rospy.loginfo("%s: %s %s in %.1f s, min clearance %s m, collision %s", name, result["phase"], result["reason"],
                      result["time_s"], result["min_clearance_m"], result["collision"])
    rospy.signal_shutdown("done")


def rotation_to_quaternion(r):
    """3x3 rotation -> (x, y, z, w)."""
    t = np.trace(r)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return ((r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s, 0.25 * s)
    i = int(np.argmax(np.diag(r)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1.0 + r[i, i] - r[j, j] - r[k, k]) * 2
    q = [0.0, 0.0, 0.0]
    q[i] = 0.25 * s
    q[j] = (r[j, i] + r[i, j]) / s
    q[k] = (r[k, i] + r[i, k]) / s
    return (q[0], q[1], q[2], (r[k, j] - r[j, k]) / s)


if __name__ == "__main__":
    ros_main()
