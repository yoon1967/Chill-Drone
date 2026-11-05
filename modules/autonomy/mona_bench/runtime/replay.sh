#!/usr/bin/env bash
set -eo pipefail
run_dir="/output/replays/$(date -u +%Y%m%d_%H%M%S)_${RANDOM}"
mkdir -p "$run_dir"
echo "Replay output: $run_dir"
python3 /opt/mona_runtime/inspect_bag.py /input.bag --output "$run_dir/input-quality.json"
depth_model="${MONA_DEPTH_MODEL:-da_v2_metric_indoor_small}"
case "$depth_model" in
    da_v2_metric_indoor_small) model_file=/models/da_v2_metric_indoor_small_518x686_fp16.onnx ;;
    da_v2_metric_indoor_small_fp32) model_file=/models/da_v2_metric_indoor_small_518x686.onnx ;;
    sc_depth_v2_nyu) model_file=/models/sc_depth_v2_nyu_480x640.onnx ;;
    *) echo "MONA_DEPTH_MODEL must be da_v2_metric_indoor_small, da_v2_metric_indoor_small_fp32 or sc_depth_v2_nyu" >&2; exit 2 ;;
esac
test -s "$model_file" || { echo "Missing $depth_model ONNX model: $model_file" >&2; exit 2; }
test -s /input.bag || { echo 'Missing ROS1 input bag' >&2; exit 2; }
play_rate="${MONA_PLAY_RATE:-1}"
# light: poses, plans and commands only. Recording 1.2 MB depth images, point clouds and the planner map
# at 8-10 Hz loaded the 1x replay enough to drop frames (and the recorder dropped messages itself).
record_mode="${MONA_RECORD:-light}"
record_topics=(/orb_slam2_rgbd/pose /sync/tf_odom /planning/bspline /planning/pos_cmd /mona/candidate_cmd_vel
               /waypoint_generator/waypoints /tello/tof_btm)
case "$record_mode" in
    light) ;;
    full) record_topics+=(/sync/depth_metric /sync/depth /sync/pcl_cam /sync/pcl_ground /sync/camera_info
                          /sdf_map/occupancy /sdf_map/occupancy_inflate) ;;
    *) echo "MONA_RECORD must be light or full" >&2; exit 2 ;;
esac
depth_device="${MONA_DEPTH_DEVICE:-cuda}"
case "$depth_device" in cuda|cpu) ;; *) echo "MONA_DEPTH_DEVICE must be cuda or cpu" >&2; exit 2 ;; esac
python3 - "$play_rate" "${MONA_GOAL_FORWARD_M:-}" "$run_dir/replay-options.json" "$depth_device" "$depth_model" "$record_mode" <<'PY'
import json, math, sys
rate = float(sys.argv[1])
if not (math.isfinite(rate) and 0.05 <= rate <= 1):
    sys.exit("MONA_PLAY_RATE must be in [0.05, 1]")
json.dump(dict(play_rate=rate, goal_forward_m=float(sys.argv[2]) if sys.argv[2] else None,
               depth_device=sys.argv[4], depth_model=sys.argv[5], record=sys.argv[6], flight_commands_sent=False), open(sys.argv[3], "w"), indent=2)
PY
# One BLAS thread per Python process (see full.launch); the goal sender and recorders inherit it.
export OPENBLAS_NUM_THREADS=1
roscore > "$run_dir/roscore.log" 2>&1 &
master_pid=$!
cleanup() {
    if [ -n "${record_pid:-}" ]; then kill -INT "$record_pid" 2>/dev/null || true; wait "$record_pid" || true; fi
    for pid in "${play_pid:-}" "${goal_pid:-}" "${stack_pid:-}" "$master_pid"; do
        [ -z "$pid" ] || kill "$pid" 2>/dev/null || true
    done
}
trap cleanup EXIT
for i in {1..40}; do rosparam list >/dev/null 2>&1 && break; sleep .25; done
rosparam set /use_sim_time true
roslaunch mona_integration full.launch hardware:=false depth_device:="$depth_device" depth_model:="$depth_model" > "$run_dir/stack.log" 2>&1 &
stack_pid=$!
ready=false
for i in {1..120}; do
    kill -0 "$stack_pid" 2>/dev/null || { echo "Stack exited; see $run_dir/stack.log" >&2; exit 3; }
    if rostopic info /tello/image_raw 2>/dev/null | grep -q '/sc_depth_pl'; then ready=true; break; fi
    sleep 1
done
$ready || { echo "Depth input subscription not ready; see $run_dir/stack.log" >&2; exit 3; }
goal_record=''
if [ -n "${MONA_GOAL_FORWARD_M:-}" ]; then
    # Offline goal only: planner/PID outputs stay in this replay; no drone command path exists.
    goal_record="$run_dir/goal.json"
    python3 -u /opt/mona_runtime/send_goal.py --forward-m "$MONA_GOAL_FORWARD_M" \
        --record "$goal_record" > "$run_dir/goal.log" 2>&1 &
    goal_pid=$!
fi
rosbag record --lz4 -O "$run_dir/outputs.bag" "${record_topics[@]}" > "$run_dir/record.log" 2>&1 &
record_pid=$!
rosbag play --clock --delay=3 --rate="$play_rate" /input.bag > "$run_dir/playback.log" 2>&1 &
play_pid=$!
while kill -0 "$play_pid" 2>/dev/null; do
    kill -0 "$stack_pid" 2>/dev/null || { echo 'Stack exited during playback' >&2; exit 3; }
    sleep 0.25
done
wait "$play_pid"
play_pid=''
sleep 1
kill -INT "$record_pid"
wait "$record_pid" || true
record_pid=''
python3 /opt/mona_runtime/summarize_output.py "$run_dir/outputs.bag" "$run_dir/result.json" "${goal_record:--}" "$record_mode"
