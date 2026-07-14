#!/usr/bin/env python3
"""
Interactive initial pose estimator for SuperLoc localization.

Loads a prior map PCD and the first LiDAR frame from a bag, shows both in a
top-down 2D view, and lets you keyboard-align the scan to the map. Press Space
to run ICP refinement from the current placement, Enter to confirm and print
the ready-to-paste run_localization.sh command.

Usage:
    source ~/ws_livox/install/setup.bash && source ~/spark_lio/install/setup.bash
    python3 scripts/initial_pose_estimator.py <map.pcd> <bag_path>

Controls:
    W/S         translate Y ±0.5 m      Shift+W/S   fine ±0.05 m
    A/D         translate X ±0.5 m      Shift+A/D   fine ±0.05 m
    Q/E         rotate yaw ±5°          Shift+Q/E   fine ±0.5°
    Space       run ICP refinement from current pose
    R           reset pose to origin
    Enter       confirm and print run_localization.sh command
"""

import argparse
import sys

import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d
import rosbag2_py
from rclpy.serialization import deserialize_message


ARROW_LEN = 1.5
MAX_DISPLAY_PTS = 30_000


def load_map(map_path: str):
    print(f"Loading map: {map_path} ...")
    pcd = o3d.io.read_point_cloud(map_path)
    pts = np.asarray(pcd.points, dtype=np.float64)
    print(f"  {len(pts):,} points total")
    step = max(1, len(pts) // MAX_DISPLAY_PTS)
    display_pts = pts[::step]
    print(f"  Displaying {len(display_pts):,} points (1 in {step})")
    return pts, display_pts


def load_first_scan(bag_path: str) -> np.ndarray:
    from livox_ros_driver2.msg import CustomMsg
    print("Reading first /livox/lidar frame from bag ...")
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''),
    )
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == '/livox/lidar':
            msg = deserialize_message(data, CustomMsg)
            pts = np.array([[p.x, p.y, p.z] for p in msg.points], dtype=np.float64)
            print(f"  {len(pts):,} points in first frame")
            return pts
    print("ERROR: no /livox/lidar messages found in bag", file=sys.stderr)
    sys.exit(1)


def apply_pose(pts: np.ndarray, x: float, y: float, yaw: float) -> np.ndarray:
    c, s = np.cos(yaw), np.sin(yaw)
    R = np.array([[c, -s], [s, c]])
    return (R @ pts[:, :2].T).T + np.array([x, y])


def pose_to_matrix(x: float, y: float, yaw: float) -> np.ndarray:
    c, s = np.cos(yaw), np.sin(yaw)
    T = np.eye(4)
    T[:2, :2] = [[c, -s], [s, c]]
    T[0, 3], T[1, 3] = x, y
    return T


def matrix_to_pose(T: np.ndarray):
    return T[0, 3], T[1, 3], np.arctan2(T[1, 0], T[0, 0])


def run_icp(scan_pts: np.ndarray, map_pts: np.ndarray,
            x: float, y: float, yaw: float):
    print("Running ICP refinement ...")

    scan_o3d = o3d.geometry.PointCloud()
    scan_o3d.points = o3d.utility.Vector3dVector(scan_pts)
    scan_o3d.estimate_normals(o3d.geometry.KDTreeSearchParamKNN(20))

    map_o3d = o3d.geometry.PointCloud()
    map_o3d.points = o3d.utility.Vector3dVector(map_pts)
    map_o3d.estimate_normals(o3d.geometry.KDTreeSearchParamKNN(20))

    result = o3d.pipelines.registration.registration_icp(
        scan_o3d, map_o3d,
        max_correspondence_distance=1.0,
        init=pose_to_matrix(x, y, yaw),
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50),
    )

    nx, ny, nyaw = matrix_to_pose(result.transformation)
    print(f"  fitness={result.fitness:.4f}  inlier_rmse={result.inlier_rmse:.4f}")
    print(f"  Refined pose: x={nx:.3f}  y={ny:.3f}  yaw={np.degrees(nyaw):.1f}°")
    if result.fitness < 0.1:
        print("  WARNING: low fitness — try a better initial placement before running ICP.")
    return nx, ny, nyaw


def main():
    parser = argparse.ArgumentParser(
        description="Interactive initial pose estimator for SuperLoc."
    )
    parser.add_argument("map_path", help="Prior map PCD file")
    parser.add_argument("bag_path", help="ROS 2 bag directory")
    args = parser.parse_args()

    map_pts_full, map_pts_display = load_map(args.map_path)
    scan_pts = load_first_scan(args.bag_path)

    state = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}

    # ── Figure ──────────────────────────────────────────────────────────────────
    plt.style.use('dark_background')
    # Disable matplotlib default shortcuts that clash with our controls.
    for key in ('s', 'q', 'r', 'a', 'd', 'w', 'e'):
        plt.rcParams.pop(f'keymap.{key}', None)
    for keymap in list(plt.rcParams.keys()):
        if keymap.startswith('keymap.'):
            plt.rcParams[keymap] = [
                k for k in plt.rcParams[keymap]
                if k not in ('s', 'q', 'r', 'a', 'd', 'w', 'e',
                             'S', 'Q', 'R', 'A', 'D', 'W', 'E')
            ]
    fig, ax = plt.subplots(figsize=(13, 10))
    ax.set_aspect('equal')
    ax.grid(True, color='#333333', linewidth=0.5)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title(f"Initial Pose Estimator — {args.map_path.split('/')[-1]}")

    map_sc = ax.scatter(
        map_pts_display[:, 0], map_pts_display[:, 1],
        c='#777777', s=0.5, linewidths=0, label='Prior map', rasterized=True,
    )
    scan_sc = ax.scatter(
        *apply_pose(scan_pts, 0, 0, 0).T,
        c='#00ccff', s=1.5, linewidths=0, label='First scan', rasterized=True,
    )

    arrow_handle = [None]

    def draw_arrow(x, y, yaw):
        if arrow_handle[0] is not None:
            arrow_handle[0].remove()
        dx, dy = ARROW_LEN * np.cos(yaw), ARROW_LEN * np.sin(yaw)
        arrow_handle[0] = ax.annotate(
            '', xy=(x + dx, y + dy), xytext=(x, y),
            arrowprops=dict(arrowstyle='->', color='#ff6600', lw=2.5),
        )

    pose_text = ax.text(
        0.02, 0.98, '', transform=ax.transAxes,
        fontsize=10, va='top', fontfamily='monospace',
        bbox=dict(boxstyle='round', facecolor='#222222', alpha=0.85),
    )
    ax.text(
        0.02, 0.02,
        "W/S: Y±   A/D: X±   Q/E: yaw±   (Shift = fine)\n"
        "Space: ICP refine    R: reset    Enter: confirm",
        transform=ax.transAxes, fontsize=8, va='bottom',
        color='#aaaaaa', fontfamily='monospace',
        bbox=dict(boxstyle='round', facecolor='#1a1a1a', alpha=0.85),
    )
    ax.legend(loc='upper right', markerscale=6, fontsize=9)

    # ── Helpers ─────────────────────────────────────────────────────────────────
    def refresh():
        x, y, yaw = state['x'], state['y'], state['yaw']
        scan_sc.set_offsets(apply_pose(scan_pts, x, y, yaw))
        draw_arrow(x, y, yaw)
        pose_text.set_text(
            f"x   = {x:+.3f} m\n"
            f"y   = {y:+.3f} m\n"
            f"yaw = {np.degrees(yaw):+.1f}°"
        )
        fig.canvas.draw_idle()

    def parse_key(raw: str):
        """Return (fine: bool, base_key: str) handling both 'shift+w' and 'W' forms."""
        if raw.startswith('shift+'):
            return True, raw[6:]
        if len(raw) == 1 and raw.isupper():
            return True, raw.lower()
        return False, raw

    # ── Key handler ─────────────────────────────────────────────────────────────
    def on_key(event):
        k = event.key or ''
        fine, base = parse_key(k)

        dt = 0.05 if fine else 0.5
        dr = np.radians(0.5 if fine else 5.0)

        x, y, yaw = state['x'], state['y'], state['yaw']

        if   base == 'w': y += dt
        elif base == 's': y -= dt
        elif base == 'd': x += dt
        elif base == 'a': x -= dt
        elif base == 'e': yaw += dr
        elif base == 'q': yaw -= dr
        elif base == 'r': x, y, yaw = 0.0, 0.0, 0.0
        elif base == ' ':
            nx, ny, nyaw = run_icp(scan_pts, map_pts_full, x, y, yaw)
            state['x'], state['y'], state['yaw'] = nx, ny, nyaw
            refresh()
            return
        elif k in ('enter', 'return'):
            sep = '=' * 55
            print(f"\n{sep}")
            print("Initial pose confirmed:")
            print(f"  x   = {x:.4f} m")
            print(f"  y   = {y:.4f} m")
            print(f"  yaw = {yaw:.4f} rad  ({np.degrees(yaw):.1f}°)")
            print("\nRun localization with:")
            print(f"  ./run_localization.sh \\")
            print(f"      {args.bag_path} \\")
            print(f"      {args.map_path} \\")
            print(f"      {x:.4f} {y:.4f} 0.0 {yaw:.4f}")
            print(sep)
            plt.close(fig)
            return
        else:
            return

        state['x'], state['y'], state['yaw'] = x, y, yaw
        refresh()

    fig.canvas.mpl_connect('key_press_event', on_key)
    refresh()
    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    main()
