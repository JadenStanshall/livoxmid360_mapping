#!/bin/bash
# Run SuperLoc localization against a prior map.
#
# Usage:
#   ./run_localization.sh <bag_path> <map_path> [x] [y] [z] [yaw]
#
# Arguments:
#   bag_path   ROS 2 bag directory to replay
#   map_path   Absolute path to final_map.pcd (output of postprocess_map.py)
#   x y z      Initial position in map frame (metres, default 0.0)
#   yaw        Initial heading in map frame (radians, default 0.0)
#
# Output:
#   <bag_path_basename>_loc_<timestamp>/        — output directory
#   <output>/launch.log                         — full node output log
#
# Example:
#   ./run_localization.sh \
#       /home/jaden/fluff/bags/custom_msg_map_2_2026-05-11-10-34-34 \
#       /home/jaden/spark_lio/maps/final_maps/map_1_so_v2.pcd

BAG_PATH="$1"
MAP_PATH="$2"
INIT_X="${3:-0.0}"
INIT_Y="${4:-0.0}"
INIT_Z="${5:-0.0}"
INIT_YAW="${6:-0.0}"

if [ -z "$BAG_PATH" ] || [ -z "$MAP_PATH" ]; then
    echo "Usage: $0 <bag_path> <map_path> [x] [y] [z] [yaw]"
    exit 1
fi

if [ ! -f "$MAP_PATH" ]; then
    echo "ERROR: map not found: $MAP_PATH"
    exit 1
fi

if [ ! -d "$BAG_PATH" ]; then
    echo "ERROR: bag not found: $BAG_PATH"
    exit 1
fi

set -e

source ~/ws_livox/install/setup.bash
source ~/spark_lio/install/setup.bash

pkill -SIGINT -f "feature_extraction_node" 2>/dev/null || true
pkill -SIGINT -f "laser_mapping_node"      2>/dev/null || true
pkill -SIGINT -f "imu_preintegration_node" 2>/dev/null || true
pkill -SIGINT -f "ros2 bag play"           2>/dev/null || true
sleep 2

BAG_NAME=$(basename "$BAG_PATH")
TIMESTAMP=$(date +%Y-%m-%d-%H-%M-%S)
RECORD_DIR="$(dirname "$BAG_PATH")/${BAG_NAME}_loc_${TIMESTAMP}"
mkdir -p "$RECORD_DIR"

LOG_FILE="$RECORD_DIR/launch.log"

echo "[localization] Map:     $MAP_PATH"
echo "[localization] Bag:     $BAG_PATH"
echo "[localization] Init:    x=$INIT_X y=$INIT_Y z=$INIT_Z yaw=$INIT_YAW"
echo "[localization] Output:  $RECORD_DIR"
echo "[localization] Log:     $LOG_FILE"

# Launch SuperLoc nodes + RViz — all output goes to log file, not the terminal.
setsid ros2 launch spark_lio_bringup localization.launch.py \
    map_path:="$MAP_PATH" \
    x:="$INIT_X" y:="$INIT_Y" z:="$INIT_Z" yaw:="$INIT_YAW" \
    use_rviz:=true > "$LOG_FILE" 2>&1 &

LAUNCH_PID=$!
LAUNCH_PGID=$(ps -o pgid= -p $LAUNCH_PID | tr -d ' ')

MONITOR_PID=""
PLAY_PID=""

# Kill all background processes and the launch process group (which includes RViz).
cleanup() {
    echo ""
    echo "[localization] Stopping..."
    [ -n "$PLAY_PID"    ] && kill -SIGINT "$PLAY_PID"    2>/dev/null || true
    [ -n "$MONITOR_PID" ] && kill -SIGINT "$MONITOR_PID" 2>/dev/null || true
    [ -n "$MONITOR_PID" ] && wait "$MONITOR_PID" 2>/dev/null || true
    kill -SIGINT -"$LAUNCH_PGID" 2>/dev/null || kill -SIGINT "$LAUNCH_PID" 2>/dev/null || true
    wait "$LAUNCH_PID" 2>/dev/null || true
    echo ""
    echo "[localization] Done."
    echo "  Full node log: $LOG_FILE"
}
trap 'cleanup; exit 130' INT
trap 'cleanup; exit 143' TERM

echo "[localization] Waiting for nodes to initialise (10 s)..."
sleep 10

# Live localization monitor — sole owner of the terminal from here on.
python3 "$(dirname "$0")/scripts/localization_monitor.py" --log-file "$LOG_FILE" &
MONITOR_PID=$!

# Play the bag in background so Ctrl+C can interrupt the wait and trigger cleanup.
ros2 bag play "$BAG_PATH" --clock --rate 1.0 > "$RECORD_DIR/bag_play.log" 2>&1 &
PLAY_PID=$!
wait "$PLAY_PID" || true
PLAY_PID=""

# Bag finished naturally — run cleanup normally.
cleanup
