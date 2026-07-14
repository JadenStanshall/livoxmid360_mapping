#!/bin/bash
# Visualize a point cloud (PCD or PLY) with pcl_viewer.
#
# Usage:
#   ./pcl_viz.sh [path/to/map.pcd|ply]
#   ./pcl_viz.sh --ply path/to/map.ply
#   ./pcl_viz.sh path/to/map.pcd --voxel 0.05
#   ./pcl_viz.sh --ply path/to/map.ply --voxel 0.1
#
# Flags:
#   --ply            Input is a PLY file (also auto-detected from .ply extension)
#   --voxel <res>    Downsample to voxels of <res> metres before display (default 0.05)
#
# Defaults to maps/map_1/globalMap.pcd if no file argument is given.

IS_PLY=false
VOXEL_RES=""
FILE=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --ply)        IS_PLY=true; shift ;;
        --voxel)      VOXEL_RES="${2:-0.05}"; shift 2 ;;
        --voxel=*)    VOXEL_RES="${1#*=}"; shift ;;
        -*)           echo "Unknown flag: $1"; exit 1 ;;
        *)            FILE="$1"; shift ;;
    esac
done

FILE="${FILE:-/home/jaden/spark_lio/maps/map_1/globalMap.pcd}"

if [ ! -f "$FILE" ]; then
    echo "ERROR: file not found: $FILE"
    exit 1
fi

# Auto-detect PLY from extension even if --ply wasn't passed.
if [[ "${FILE,,}" == *.ply ]]; then
    IS_PLY=true
fi

TMPFILE=""
DISPLAY_FILE="$FILE"

cleanup() {
    [ -n "$TMPFILE" ] && rm -f "$TMPFILE"
}
trap cleanup EXIT

# If PLY or voxel downsampling is needed, preprocess to a temp PCD.
if [ "$IS_PLY" = true ] || [ -n "$VOXEL_RES" ]; then
    TMPFILE="/tmp/pcl_viz_$$.pcd"

    if [ -n "$VOXEL_RES" ]; then
        echo "Voxel downsampling at ${VOXEL_RES} m → $TMPFILE"
        python3 - "$FILE" "$VOXEL_RES" "$TMPFILE" <<'EOF'
import sys, open3d as o3d
pcd = o3d.io.read_point_cloud(sys.argv[1])
print(f"  Before: {len(pcd.points):,} points")
pcd = pcd.voxel_down_sample(float(sys.argv[2]))
print(f"  After : {len(pcd.points):,} points")
o3d.io.write_point_cloud(sys.argv[3], pcd)
EOF
    else
        # PLY → PCD conversion only (no downsampling)
        echo "Converting PLY → temp PCD: $TMPFILE"
        python3 - "$FILE" "$TMPFILE" <<'EOF'
import sys, open3d as o3d
pcd = o3d.io.read_point_cloud(sys.argv[1])
print(f"  Points: {len(pcd.points):,}")
o3d.io.write_point_cloud(sys.argv[2], pcd)
EOF
    fi

    DISPLAY_FILE="$TMPFILE"
fi

echo "Opening: $FILE"
[ -n "$VOXEL_RES" ] && echo "Voxel  : ${VOXEL_RES} m"
echo "Controls: left-click+drag=rotate, scroll=zoom, r=reset view, q=quit"

pcl_viewer "$DISPLAY_FILE"
