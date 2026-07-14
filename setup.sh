#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WORKSPACE_DIR"

ALL=false
for arg in "$@"; do
  case "$arg" in
    --all) ALL=true ;;
    *) echo "Unknown argument: $arg"; exit 1 ;;
  esac
done

apply_patch() {
  local dir="$1"
  local patch="$2"
  if git -C "$dir" apply --reverse --check "$patch" 2>/dev/null; then
    echo "  (already applied, skipping)"
  else
    git -C "$dir" apply "$patch"
  fi
}

echo "==> Initializing submodules..."
if [ "$ALL" = true ]; then
  git submodule update --init --recursive
else
  git submodule update --init src/SuperOdom
fi

echo ""
echo "==> Applying patches..."
apply_patch src/SuperOdom "$WORKSPACE_DIR/patches/SuperOdom.patch"
echo "  [done] SuperOdom"

if [ "$ALL" = true ]; then
  apply_patch src/spark_fast_lio "$WORKSPACE_DIR/patches/spark_fast_lio.patch"
  echo "  [done] spark_fast_lio"
  apply_patch src/kiss_matcher "$WORKSPACE_DIR/patches/kiss_matcher.patch"
  echo "  [done] kiss_matcher"
fi

echo ""
echo "==> Sourcing ROS 2 Humble..."
set +u
source /opt/ros/humble/setup.bash
set -u

echo ""
echo "==> Installing ROS dependencies..."
rosdep install --from-paths src --ignore-src -r -y

echo ""
echo "==> Building workspace..."
if [ "$ALL" = true ]; then
  colcon build --symlink-install
else
  colcon build --symlink-install --packages-ignore spark_fast_lio kiss_matcher_ros
fi

echo ""
echo "Setup complete. Each new terminal:"
