#!/bin/bash
# Kill all processes from a running localization demo.

echo "[kill] Stopping localization..."

pkill -SIGINT -f "feature_extraction_node"   2>/dev/null || true
pkill -SIGINT -f "imu_preintegration_node"   2>/dev/null || true
pkill -SIGINT -f "laser_mapping_node"        2>/dev/null || true
pkill -SIGINT -f "localization_monitor.py"   2>/dev/null || true
pkill -SIGINT -f "ros2 bag play"             2>/dev/null || true
pkill -SIGINT -f "rviz2"                     2>/dev/null || true

sleep 2

# Force-kill anything that ignored SIGINT.
pkill -SIGKILL -f "feature_extraction_node"  2>/dev/null || true
pkill -SIGKILL -f "imu_preintegration_node"  2>/dev/null || true
pkill -SIGKILL -f "laser_mapping_node"       2>/dev/null || true
pkill -SIGKILL -f "localization_monitor.py"  2>/dev/null || true

echo "[kill] Done."
