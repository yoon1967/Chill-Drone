#!/usr/bin/env bash
# Live MoNA run against the drone gateway (/ws/autonomy). The gateway flies cmd_vel only in auto mode; nothing here
# takes off or lands. Stop with Enter or Ctrl+C; poses, plans and commands are recorded and summarized.
set -eo pipefail
run_dir="/output/live/$(date -u +%Y%m%d_%H%M%S)_${RANDOM}"
mkdir -p "$run_dir"
echo "Live output: $run_dir"
# MONA_GATEWAY_WAIT_S: seconds the pre-check waits for the gateway and drone (default 60).
gateway="${MONA_GATEWAY:-http://host.docker.internal:8010}"
forward_m="${MONA_GOAL_FORWARD_M:-5}"
seconds="${MONA_LIVE_SECONDS:-300}"
autostart="${MONA_AUTOSTART:-false}"
depth_model="${MONA_DEPTH_MODEL:-da_v2_metric_indoor_small}"
depth_device="${MONA_DEPTH_DEVICE:-cuda}"
case "$depth_device" in cuda|cpu) ;; *) echo "MONA_DEPTH_DEVICE must be cuda or cpu" >&2; exit 2 ;; esac
case "$autostart" in true|false) ;; *) echo "MONA_AUTOSTART must be true or false" >&2; exit 2 ;; esac
# Same record modes as replay.sh. full adds metric depth, point clouds and the planner map (heavier; may drop frames).
record_mode="${MONA_RECORD:-light}"
record_topics=(/orb_slam2_rgbd/pose /sync/tf_odom /planning/bspline /planning/pos_cmd /mona/candidate_cmd_vel
               /waypoint_generator/waypoints /tello/tof_btm)
case "$record_mode" in
    light) ;;
    full) record_topics+=(/sync/depth_metric /sync/depth /sync/pcl_cam /sync/pcl_ground /sync/camera_info
                          /sdf_map/occupancy /sdf_map/occupancy_inflate) ;;
    *) echo "MONA_RECORD must be light or full" >&2; exit 2 ;;
esac
python3 - "$forward_m" "$seconds" <<'PY'
import math, sys
forward, seconds = float(sys.argv[1]), float(sys.argv[2])
if not (math.isfinite(forward) and 0 < forward <= 5):
    sys.exit("MONA_GOAL_FORWARD_M must be in (0, 5]")
if not (math.isfinite(seconds) and 1 <= seconds <= 3600):
    sys.exit("MONA_LIVE_SECONDS must be in [1, 3600]")
PY
python3 - "$run_dir/live-options.json" "$gateway" "$forward_m" "$autostart" "$depth_model" "$depth_device" \
    "$record_mode" <<'PY'
import json, sys
json.dump(dict(gateway=sys.argv[2], goal_forward_m=float(sys.argv[3]), autostart=sys.argv[4] == "true",
               depth_model=sys.argv[5], depth_device=sys.argv[6], record=sys.argv[7],
               note="the gateway flies cmd_vel only in auto mode"), open(sys.argv[1], "w"), indent=2)
PY
# Before any ROS process: the URL must be the gateway (not check_server.py on the same port) with a drone attached.
if ! python3 /opt/mona_runtime/gateway_bridge.py --check "$gateway" --wait "${MONA_GATEWAY_WAIT_S:-60}" \
        --out "$run_dir/gateway-check.json"; then
    echo "Gateway not ready; nothing was started. Details: $run_dir/gateway-check.json" >&2
    exit 4
fi
export OPENBLAS_NUM_THREADS=1
roscore > "$run_dir/roscore.log" 2>&1 &
master_pid=$!
cleanup() {
    if [ -n "${bridge_pid:-}" ]; then kill -INT "$bridge_pid" 2>/dev/null || true; wait "$bridge_pid" 2>/dev/null || true; fi
    if [ -n "${record_pid:-}" ]; then kill -INT "$record_pid" 2>/dev/null || true; wait "$record_pid" 2>/dev/null || true; fi
    for pid in "${stack_pid:-}" "$master_pid"; do
        [ -z "$pid" ] || kill "$pid" 2>/dev/null || true
    done
}
trap cleanup EXIT
trap 'stop=1' INT TERM
for i in {1..40}; do rosparam list >/dev/null 2>&1 && break; sleep .25; done
roslaunch mona_integration full.launch hardware:=false depth_device:="$depth_device" depth_model:="$depth_model" ${MONA_LAUNCH_ARGS:-} \
    > "$run_dir/stack.log" 2>&1 &
stack_pid=$!
for i in {1..120}; do
    kill -0 "$stack_pid" 2>/dev/null || { echo "Stack exited; see $run_dir/stack.log" >&2; exit 3; }
    rostopic info /tello/image_raw 2>/dev/null | grep -q '/sc_depth_pl' && break
    sleep 1
done
rosbag record --lz4 -O "$run_dir/outputs.bag" "${record_topics[@]}" > "$run_dir/record.log" 2>&1 &
record_pid=$!
python3 -u /opt/mona_runtime/gateway_bridge.py _gateway:="$gateway" _goal_forward_m:="$forward_m" \
    _autostart:="$autostart" _summary:="$run_dir/bridge-summary.json" _status:="$run_dir/status.txt" \
    _inputs_dir:="$run_dir/inputs" _max_side_m:="${MONA_MAX_SIDE_M:-3.0}" _scan_deg:="${MONA_SCAN_DEG:-90}" \
    _land_on_finish:="${MONA_LAND_ON_FINISH:-true}" > "$run_dir/bridge.log" 2>&1 &
bridge_pid=$!
echo "Running up to ${seconds}s. Gateway: $gateway. Recording: $record_mode. Press Enter to stop. Status every 5 s:"
stop=0
start=$(date +%s)
shown=0
while [ "$stop" = 0 ] && [ $(( $(date +%s) - start )) -lt "${seconds%.*}" ]; do
    kill -0 "$stack_pid" 2>/dev/null || { echo 'Stack exited during the live run' >&2; break; }
    kill -0 "$bridge_pid" 2>/dev/null || { echo "Bridge exited; see $run_dir/bridge.log" >&2; break; }
    if [ $(( $(date +%s) - shown )) -ge 5 ] && [ -s "$run_dir/status.txt" ]; then
        cat "$run_dir/status.txt"
        shown=$(date +%s)
    fi
    # Enter stops; without a terminal (EOF) read fails at once, so wait the second instead.
    if read -r -t 1 _ 2>/dev/null; then stop=1; else rc=$?; [ "$rc" -gt 128 ] || sleep 1; fi
done
trap - INT TERM
kill -INT "$bridge_pid" 2>/dev/null || true
wait "$bridge_pid" 2>/dev/null || true
bridge_pid=''
kill -INT "$record_pid" 2>/dev/null || true
wait "$record_pid" 2>/dev/null || true
record_pid=''
python3 /opt/mona_runtime/summarize_output.py "$run_dir/outputs.bag" "$run_dir/result.json" - "$record_mode" > /dev/null
python3 - "$run_dir/result.json" <<'PY'
import json, sys
result = json.load(open(sys.argv[1]))
result.pop("flight_commands_sent", None)  # cmd_vel did go to the gateway; whether it flew depends on the gateway mode
result["scope"] = "live_outputs; the gateway flies cmd_vel only in auto mode; see bridge-summary.json and the gateway log"
json.dump(result, open(sys.argv[1], "w"), indent=2)
print(json.dumps(result["stages"], indent=2))
PY
cat "$run_dir/bridge-summary.json" 2>/dev/null || echo "bridge summary missing; see $run_dir/bridge.log"
echo "Inputs kept in $run_dir/inputs; replay them with run.ps1 -Action InputBag -Bag <this live folder>, then -Action Replay."
