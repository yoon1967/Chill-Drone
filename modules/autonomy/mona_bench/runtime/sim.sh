#!/usr/bin/env bash
# Closed-loop avoidance sim over a set of box worlds (sim_world.py + planner.launch). One fresh planner per
# scenario so maps do not carry over. Results: /output/sim/<run>/<scenario>.json and summary.json.
set -eo pipefail
run_dir="/output/sim/$(date -u +%Y%m%d_%H%M%S)_${RANDOM}"
mkdir -p "$run_dir"
echo "Sim output: $run_dir"
height="${MONA_SIM_HEIGHT_M:-1.0}"
forward="${MONA_GOAL_FORWARD_M:-4}"
side="${MONA_MAX_SIDE_M:-3.0}"
python3 - "$run_dir" "$height" "$forward" "$side" <<'PY'
import json, os, sys
run, h, fwd, side = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
floor = -h
def box(x, w, y=0.0, d=0.4, top=0.5, bottom=None):   # centre y, width w (along y), depth d (along x), top over flight z
    return [x, x + d, y - w / 2, y + w / 2, floor if bottom is None else bottom, top]
def person(x, y=0.0):   # standing person: shoulders 0.5 m, body 0.3 m deep, 1.7 m tall from the floor
    return [x, x + 0.3, y - 0.25, y + 0.25, floor, floor + 1.7]
people = [   # MONA_SIM_SET=person: person 1 m ahead (e.g. with MONA_GOAL_FORWARD_M=2)
    dict(name="person_1m_center", boxes=[person(1.0)]),
    dict(name="person_1m_offset_0.3m", boxes=[person(1.0, y=0.3)]),
    dict(name="person_1m_corridor_3m", boxes=[person(1.0), [-1.0, 6.0, 1.5, 1.7, floor, 1.5],
                                              [-1.0, 6.0, -1.7, -1.5, floor, 1.5]]),
    dict(name="person_1m_corridor_2.5m", boxes=[person(1.0), [-1.0, 6.0, 1.25, 1.45, floor, 1.5],
                                                [-1.0, 6.0, -1.45, -1.25, floor, 1.5]]),
    # live DA-V2 saw a person 1 m away at 1.3-1.65 m (README "장애물 녹화"); also check the opposite bias
    dict(name="person_1m_depth_far_x1.3", boxes=[person(1.0)], depth_scale=1.3),
    dict(name="person_1m_depth_near_x0.8", boxes=[person(1.0)], depth_scale=0.8),
]
def wall(x0, x1, y0, y1):
    return [x0, x1, y0, y1, floor, 1.8]
def room(back=-1.0, front=4.0, half=2.0):   # closed room around the start (all indoor tests)
    return [wall(back - 0.2, back, -half - 0.2, half + 0.2), wall(front, front + 0.2, -half - 0.2, half + 0.2),
            wall(back, front, half, half + 0.2), wall(back, front, -half - 0.2, -half)]
indoor = [   # MONA_SIM_SET=indoor: rooms, a doorway, clutter, a goal near a wall, SLAM error
    dict(name="room5x4_person_1m", boxes=room() + [person(1.0)]),
    dict(name="room5x4_two_people", boxes=room() + [person(1.0, y=0.5), person(1.8, y=-0.6)], goal_forward_m=3.0),
    dict(name="doorway_1m_offset", goal_forward_m=3.0,   # wall at 2 m with a 1 m door centred 0.6 m left
         boxes=room(front=5.0) + [wall(2.0, 2.2, -2.0, 0.1), wall(2.0, 2.2, 1.1, 2.0)]),
    dict(name="goal_0.6m_before_wall", boxes=room(front=2.6)),
    dict(name="person_1m_slam_drift_side", boxes=room() + [person(1.0)], pose_drift_mps=[0.0, 0.08]),
    dict(name="person_1m_slam_drift_side_0.15", boxes=room() + [person(1.0)], pose_drift_mps=[0.0, 0.15]),
    dict(name="person_1m_slam_drift_fwd", boxes=room() + [person(1.0)], pose_drift_mps=[0.06, 0.0]),
    dict(name="person_1m_slam_drift_during_scan", boxes=room() + [person(1.0)], pose_drift_mps=[0.0, 0.08],
         drift_from_start=True),
    dict(name="person_1m_noisy", boxes=room() + [person(1.0)], depth_noise=0.1, pose_noise_m=0.03),
]
indoor4m = [   # MONA_SIM_SET=indoor4m: 4 m forward in a 6 x 4 m room (goal 1 m before the far wall)
    dict(name="room6x4_no_obstacle", boxes=room(front=5.0)),
    dict(name="room6x4_person_1m", boxes=room(front=5.0) + [person(1.0)]),
    dict(name="room6x4_person_2m_offset", boxes=room(front=5.0) + [person(2.0, y=0.3)]),
    dict(name="corridor3m_person_1m", boxes=room(front=5.0, half=1.5) + [person(1.0)]),
    # rc calibration error (gateway rc fit from few flights): the Tello moves slower / faster than asked
    dict(name="room6x4_person_1m_slow_x0.6", boxes=room(front=5.0) + [person(1.0)], speed_gain=0.6),
    dict(name="room6x4_person_1m_fast_x1.3", boxes=room(front=5.0) + [person(1.0)], speed_gain=1.3),
]
sets = {"person": people, "indoor": indoor, "indoor4m": indoor4m}
scenarios = sets[os.environ["MONA_SIM_SET"]] if os.environ.get("MONA_SIM_SET") in sets else [
    dict(name="no_obstacle", boxes=[]),
    dict(name="box_0.5m_at_2m", boxes=[box(2.0, 0.5)]),
    dict(name="box_1.0m_at_2m", boxes=[box(2.0, 1.0)]),
    dict(name="box_0.5m_offset_left", boxes=[box(2.0, 0.5, y=0.3)]),
    dict(name="box_0.5m_room_4m_wide", boxes=[box(2.0, 0.5), [-1.0, 7.0, 2.0, 2.2, floor, 1.5],
                                              [-1.0, 7.0, -2.2, -2.0, floor, 1.5], [6.5, 6.7, -2.2, 2.2, floor, 1.5]]),
    dict(name="low_box_below_flight", boxes=[box(2.0, 0.5, top=-0.6)]),
]
only = [n for n in os.environ.get("MONA_SIM_ONLY", "").split(",") if n]
scenarios = [s for s in scenarios if not only or s["name"] in only]
for s in scenarios:
    s.setdefault("goal_forward_m", fwd)   # a scenario may need its own goal (doorway)
    s.setdefault("duration_s", 100.0)     # look-around about 30 s (0.24 rad/s) + 4 m at 0.2 m/s and detours
    s.update(height_m=h, max_side_m=side, scan_deg=float(os.environ.get("MONA_SCAN_DEG", "90")))
    with open(f"{run}/{s['name']}.scenario.json", "w") as f:
        json.dump(s, f)
print(" ".join(s["name"] for s in scenarios))
PY
export OPENBLAS_NUM_THREADS=1
for scenario in "$run_dir"/*.scenario.json; do
    name="$(basename "$scenario" .scenario.json)"
    roslaunch mona_integration sim.launch scenario_file:="$scenario" result:="$run_dir/$name.json" \
        ctrl_pt_dist:="${MONA_CTRL_PT_DIST:-0.1}" plan_max_vel:="${MONA_PLAN_MAX_VEL:-0.2}" \
        > "$run_dir/$name.log" 2>&1 || true
    if [ -s "$run_dir/$name.json" ]; then
        python3 -c "import json,sys; r=json.load(open(sys.argv[1])); print('%-24s %-8s %-18s %5ss  clearance drone %-6s plan %-6s collision %-5s off-plan max %-5s mean %-5s side %-5s plans %s (not flown %s) loop %sHz' % (r['scenario'], r['phase'], r['reason'], r['time_s'], r['min_clearance_m'], r['plan_min_clearance_m'], r['collision'], r['cross_track_max_m'], r['cross_track_mean_m'], r['max_side_m'], r['plans'], r['plans_not_flown'], r['loop_hz']))" "$run_dir/$name.json"
    else
        echo "$name: no result; see $run_dir/$name.log"
    fi
done
python3 - "$run_dir" <<'PY'
import glob, json, os, sys
rows = [json.load(open(p)) for p in sorted(glob.glob(os.path.join(sys.argv[1], "*.json")))
        if not p.endswith(".scenario.json") and not p.endswith("summary.json")]
summary = [{k: r.get(k) for k in ("scenario", "phase", "reason", "time_s", "min_clearance_m", "plan_min_clearance_m",
                                  "collision", "cross_track_max_m", "cross_track_mean_m", "max_side_m", "plans",
                                  "plans_not_flown", "hover_share", "loop_hz", "final")} for r in rows]
json.dump(summary, open(os.path.join(sys.argv[1], "summary.json"), "w"), indent=2)
PY
