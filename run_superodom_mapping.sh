#!/bin/bash
# Run SuperOdom LiDAR-only mapping pipeline on one bag.
# Usage: ./run_superodom_mapping.sh <bag_path> <output_dir>
#
# Output:
#   <output_dir>/globalMap.ply   — full accumulated point cloud (PLY format)
#
# Evaluate with:
#   python3 scripts/evaluate_map.py <output_dir>
#   (trajectory metrics unavailable — SuperOdom does not write poses_kitti.txt)

BAG_PATH="$1"
OUTPUT_DIR="$2"

if [ -z "$BAG_PATH" ] || [ -z "$OUTPUT_DIR" ]; then
    echo "Usage: $0 <bag_path> <output_dir>"
    exit 1
fi

source ~/ws_livox/install/setup.bash
source ~/spark_lio/install/setup.bash

# SuperOdom saves its PLY to this hardcoded path (ROOT_DIR set at compile time).
SUPERODOM_SRC="$(ros2 pkg prefix super_odometry 2>/dev/null)"
# ROOT_DIR is the source directory, not the install directory — resolve via source tree.
SUPERODOM_PLY="/home/jaden/spark_lio/src/SuperOdom/super_odometry/PLY/saved_scans.ply"

# Kill any stale processes from previous runs.
pkill -SIGINT -f "feature_extraction_node" 2>/dev/null || true
pkill -SIGINT -f "laser_mapping_node"      2>/dev/null || true
pkill -SIGINT -f "imu_preintegration_node" 2>/dev/null || true
pkill -SIGINT -f "ros2 bag play"           2>/dev/null || true
pkill -SIGINT -f "superodom_mapping"       2>/dev/null || true
sleep 2

mkdir -p "$OUTPUT_DIR"

echo "[superodom] Starting LiDAR-only pipeline. Output: $OUTPUT_DIR"
echo "[superodom] Bag: $BAG_PATH"

setsid ros2 launch spark_lio_bringup superodom_mapping.launch.py \
    bag_path:="$BAG_PATH" \
    output_dir:="$OUTPUT_DIR" &

LAUNCH_PID=$!
LAUNCH_PGID=$(ps -o pgid= -p $LAUNCH_PID | tr -d ' ')
echo "[superodom] Launch PID=$LAUNCH_PID PGID=$LAUNCH_PGID"

echo "[superodom] Waiting for bag replay to start..."
sleep 20

echo "[superodom] Bag is playing (~8 min at 0.5x rate). Waiting for it to finish..."
while pgrep -f "ros2 bag play" > /dev/null 2>&1; do
    sleep 5
done

# Give laserMapping time to flush the final PLY write.
echo "[superodom] Bag finished. Waiting 10 s for final PLY flush..."
sleep 10

echo "[superodom] Sending SIGINT to shut down nodes..."
kill -SIGINT -"$LAUNCH_PGID" 2>/dev/null || kill -SIGINT "$LAUNCH_PID" 2>/dev/null || true

echo "[superodom] Waiting for shutdown..."
wait "$LAUNCH_PID" 2>/dev/null || true
sleep 3

# Copy the map to the output directory.
if [ -f "$SUPERODOM_PLY" ]; then
    cp "$SUPERODOM_PLY" "$OUTPUT_DIR/globalMap.ply"
    echo "[superodom] Map saved: $OUTPUT_DIR/globalMap.ply"
else
    echo "[superodom] WARNING: map not found at $SUPERODOM_PLY"
    echo "[superodom] Check that save_ply: true is set in superodom_lidar_only.yaml"
fi

echo "[superodom] Done. Map files:"
ls -lh "$OUTPUT_DIR"
