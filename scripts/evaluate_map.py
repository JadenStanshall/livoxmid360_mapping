#!/usr/bin/env python3
"""
evaluate_map.py — Quantitative quality metrics for a spark-lio mapping output.

Usage:
    python3 scripts/evaluate_map.py <map_dir>
    python3 scripts/evaluate_map.py maps/map_1_v8
"""

import argparse
import os
import sys
import numpy as np


# ── Thresholds for pass / warn / fail ─────────────────────────────────────────
TRAJ_Z_SPAN_PASS  = 0.05   # m
TRAJ_Z_SPAN_WARN  = 0.15
FLOOR_STD_PASS    = 0.020  # m
FLOOR_STD_WARN    = 0.050
ROT_PURITY_PASS   = 0.010  # max |non-yaw| rotation component
ROT_PURITY_WARN   = 0.030


def grade(val, pass_thr, warn_thr, low_is_good=True):
    if low_is_good:
        if val <= pass_thr: return "PASS"
        if val <= warn_thr: return "WARN"
        return "FAIL"
    else:
        if val >= pass_thr: return "PASS"
        if val >= warn_thr: return "WARN"
        return "FAIL"


def load_poses(path):
    poses = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line:
                poses.append(list(map(float, line.split())))
    return poses


def fit_plane_rms(points):
    """Fit plane via PCA, return RMS point-to-plane distance."""
    centroid = points.mean(axis=0)
    _, _, Vt = np.linalg.svd(points - centroid, full_matrices=False)
    normal = Vt[-1]
    dists = (points - centroid) @ normal
    return float(np.sqrt(np.mean(dists ** 2)))


def analyse_layer(pts, z_center, half_t, name):
    mask  = (pts[:, 2] >= z_center - half_t) & (pts[:, 2] <= z_center + half_t)
    layer = pts[mask]
    if len(layer) < 20:
        print(f"    {name}: too few points ({len(layer)}) — widen band or check map")
        return None, None
    z_std   = float(np.std(layer[:, 2]))
    z_range = float(layer[:, 2].max() - layer[:, 2].min())
    rms     = fit_plane_rms(layer)
    g = grade(z_std, FLOOR_STD_PASS, FLOOR_STD_WARN)
    print(f"    {name} [{len(layer):,} pts within ±{half_t*100:.0f}cm of Z={z_center:.3f}m]")
    print(f"      Z std dev  : {z_std*100:5.1f} cm   [{g}]")
    print(f"      Z range    : {z_range*100:5.1f} cm")
    print(f"      Plane RMS  : {rms*100:5.1f} cm")
    return z_std, rms


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("map_dir", help="Map output directory (contains globalMap.pcd and session/)")
    args = ap.parse_args()

    map_dir    = args.map_dir
    # Accept either .pcd (spark pipeline) or .ply (SuperOdom pipeline).
    pcd_path = os.path.join(map_dir, "globalMap.pcd")
    if not os.path.exists(pcd_path):
        pcd_path = os.path.join(map_dir, "globalMap.ply")
    poses_path = os.path.join(map_dir, "session", "poses_kitti.txt")

    print(f"\nEvaluating: {map_dir}")
    print("=" * 62)

    # ── 1. Trajectory metrics ──────────────────────────────────────────────────
    if not os.path.exists(poses_path):
        print(f"WARNING: poses not found: {poses_path}")
    else:
        poses = load_poses(poses_path)
        n = len(poses)

        # Extract components
        # KITTI row = [r11 r12 r13 tx  r21 r22 r23 ty  r31 r32 r33 tz]
        tx  = np.array([p[3]  for p in poses])
        ty  = np.array([p[7]  for p in poses])
        tz  = np.array([p[11] for p in poses])
        r11 = np.array([p[0]  for p in poses])
        r21 = np.array([p[4]  for p in poses])
        r31 = np.array([p[8]  for p in poses])
        r32 = np.array([p[9]  for p in poses])
        r33 = np.array([p[10] for p in poses])

        z_span = float(tz.max() - tz.min())
        z_std  = float(np.std(tz))

        print(f"\n[Trajectory]  {n} keyframes")
        g = grade(z_span, TRAJ_Z_SPAN_PASS, TRAJ_Z_SPAN_WARN)
        print(f"  Z span     : {z_span*100:6.1f} cm   [{g}]  (target < {TRAJ_Z_SPAN_PASS*100:.0f} cm)")
        print(f"  Z std dev  : {z_std*100:6.1f} cm")
        print(f"  Z range    : {tz.min():.3f} → {tz.max():.3f} m")
        print(f"  XY extent  : {tx.max()-tx.min():.1f} m × {ty.max()-ty.min():.1f} m")

        # Rotation purity — with ground-vehicle constraint, roll/pitch should be ~0
        # Pure yaw: r31=0, r32=0, r33=1; equivalently sin(pitch)=r31 ≈ 0
        max_r31 = float(np.max(np.abs(r31)))
        max_r32 = float(np.max(np.abs(r32)))
        rot_err = max(max_r31, max_r32)
        g = grade(rot_err, ROT_PURITY_PASS, ROT_PURITY_WARN)
        print(f"\n[Rotation purity]  (ground-vehicle: expect r31≈0, r32≈0)")
        print(f"  Max |r31|  : {max_r31:.4f}   [{g}]")
        print(f"  Max |r32|  : {max_r32:.4f}")

        # Yaw trajectory — look for discontinuities (the "twist")
        yaw = np.arctan2(r21, r11) * 180.0 / np.pi
        dy  = np.diff(yaw)
        # Unwrap for continuous yaw
        dy_unwrap = ((dy + 180) % 360) - 180
        max_jump_idx = int(np.argmax(np.abs(dy_unwrap)))
        max_jump     = float(np.abs(dy_unwrap[max_jump_idx]))

        print(f"\n[Yaw trajectory]")
        print(f"  Total yaw change : {np.sum(np.abs(dy_unwrap)):.1f}°")
        print(f"  Max inter-KF jump: {max_jump:.2f}° at keyframe {max_jump_idx}–{max_jump_idx+1}")
        if max_jump > 10:
            print(f"  WARNING: {max_jump:.1f}° yaw jump at KF {max_jump_idx} may be the 'twist'")

        # Position smoothness
        dpos = np.sqrt(np.diff(tx)**2 + np.diff(ty)**2 + np.diff(tz)**2)
        max_jump_pos_idx = int(np.argmax(dpos))
        max_jump_pos     = float(dpos[max_jump_pos_idx])
        print(f"\n[Position smoothness]")
        print(f"  Max inter-KF dist: {max_jump_pos*100:.1f} cm at keyframe {max_jump_pos_idx}–{max_jump_pos_idx+1}")
        print(f"  Median inter-KF  : {np.median(dpos)*100:.1f} cm  (keyframe threshold ≈ 30 cm)")

    # ── 2. Point cloud metrics ─────────────────────────────────────────────────
    if not os.path.exists(pcd_path):
        print(f"\nWARNING: globalMap.pcd not found: {pcd_path}")
        return

    try:
        import open3d as o3d
    except ImportError:
        print("\n(Install open3d for point-cloud analysis:  pip3 install open3d)")
        return

    pcd = o3d.io.read_point_cloud(pcd_path)
    pts = np.asarray(pcd.points)

    print(f"\n[Point cloud]  {len(pts):,} points")
    print(f"  X : {pts[:,0].min():.2f} → {pts[:,0].max():.2f} m")
    print(f"  Y : {pts[:,1].min():.2f} → {pts[:,1].max():.2f} m")
    print(f"  Z : {pts[:,2].min():.3f} → {pts[:,2].max():.3f} m  "
          f"(span {(pts[:,2].max()-pts[:,2].min()):.2f} m)")

    # Auto-detect floor and ceiling from Z histogram peaks
    hist, edges = np.histogram(pts[:, 2], bins=300)
    centers     = (edges[:-1] + edges[1:]) / 2
    smoothed    = np.convolve(hist, np.ones(7) / 7, mode='same')
    thr         = smoothed.max() * 0.12
    dense       = centers[smoothed > thr]

    if len(dense) >= 2:
        floor_z   = float(dense[0])
        ceiling_z = float(dense[-1])
        print(f"\n[Room geometry]  (auto-detected from Z histogram)")
        print(f"  Floor Z    : {floor_z:.3f} m")
        print(f"  Ceiling Z  : {ceiling_z:.3f} m")
        print(f"  Room height: {ceiling_z - floor_z:.2f} m")

        print(f"\n[Horizontal surface planarity]")
        analyse_layer(pts, floor_z,   0.15, "Floor  ")
        analyse_layer(pts, ceiling_z, 0.15, "Ceiling")
    else:
        print("\n  Could not auto-detect floor/ceiling — check Z range")

    print("\n" + "=" * 62)


if __name__ == "__main__":
    main()
