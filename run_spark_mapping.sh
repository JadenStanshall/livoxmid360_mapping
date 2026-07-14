#!/bin/bash
# Run mapping pipeline for one bag, save map, then shut down cleanly.
# Usage: ./run_mapping.sh <bag_path> <output_dir>

BAG_PATH="$1"
OUTPUT_DIR="$2"

set -e

source ~/ws_livox/install/setup.bash
source ~/spark_lio/install/setup.bash

# Kill any stale ROS processes from previous runs before starting fresh.
pkill -SIGINT -f "spark_lio_mapping" 2>/dev/null || true
pkill -SIGINT -f "kiss_matcher_sam"  2>/dev/null || true
pkill -SIGINT -f "ros2 bag play"     2>/dev/null || true
pkill -SIGINT -f "ros2 launch spark_lio_bringup" 2>/dev/null || true
sleep 2

mkdir -p "$OUTPUT_DIR"

echo "[mapping] Starting pipeline. Output: $OUTPUT_DIR"
echo "[mapping] Bag: $BAG_PATH"

# Launch in a new process group so we can SIGINT the whole group cleanly.
setsid ros2 launch spark_lio_bringup mapping.launch.py \
    bag_path:="$BAG_PATH" \
    output_dir:="$OUTPUT_DIR" \
    use_rviz:=false &

LAUNCH_PID=$!
LAUNCH_PGID=$(ps -o pgid= -p $LAUNCH_PID | tr -d ' ')
echo "[mapping] Launch PID=$LAUNCH_PID PGID=$LAUNCH_PGID"

# Wait for the bag play process to actually start (launch delays 3 s)
echo "[mapping] Waiting for bag replay to start..."
sleep 15

echo "[mapping] Bag is playing (~8 min at 0.5x rate). Waiting for it to finish..."
while pgrep -f "ros2 bag play" > /dev/null 2>&1; do
    sleep 5
done

# Trigger KISS-Matcher-SAM to save the map before shutting down.
# The node subscribes to /save_dir (std_msgs/String) and saves on receipt.
echo "[mapping] Triggering map save to $OUTPUT_DIR ..."
ros2 topic pub --once /save_dir std_msgs/msg/String "{data: '$OUTPUT_DIR'}" 2>/dev/null || true

# Give KISS-Matcher-SAM time to write keyframe scans + map PCD.
echo "[mapping] Waiting for map save to complete (15 s)..."
sleep 15

echo "[mapping] Sending SIGINT to shut down nodes..."
kill -SIGINT -"$LAUNCH_PGID" 2>/dev/null || kill -SIGINT "$LAUNCH_PID" 2>/dev/null

echo "[mapping] Waiting for shutdown..."
wait "$LAUNCH_PID" 2>/dev/null || true
sleep 3

# Expose the map PCD at a predictable top-level path.
SESSION_MAP="$OUTPUT_DIR/session/session_map.pcd"
if [ -f "$SESSION_MAP" ]; then
    cp "$SESSION_MAP" "$OUTPUT_DIR/globalMap.pcd"
    echo "[mapping] Global map saved: $OUTPUT_DIR/globalMap.pcd"
else
    echo "[mapping] WARNING: expected map not found at $SESSION_MAP"
fi

echo "[mapping] Done. Map files:"
ls -lh "$OUTPUT_DIR"
