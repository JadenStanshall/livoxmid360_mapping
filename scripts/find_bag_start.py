#!/usr/bin/env python3
"""
find_bag_start.py — Find where manual-handling noise ends in a ROS2 bag.

Reads /livox/imu directly from the bag's SQLite database and plots:
  - wx, wy (pitch/roll rates) — spike during manual handling, near-zero during normal nav
  - wz (yaw rate) — spikes during turns (expected, not a problem)
  - acc norm deviation from 1g — should stay low during normal operation

Usage:
    python3 scripts/find_bag_start.py <bag_dir>
    python3 scripts/find_bag_start.py <bag_dir> --poses maps/map_1_v2/session/poses_kitti.txt
    python3 scripts/find_bag_start.py <bag_dir> --no-plot    # text output only

The suggested --start-offset is based on pitch/roll RMS settling below threshold,
NOT total gyro — so normal turns don't get flagged as noise.
"""

import argparse
import glob
import os
import struct
import sys

import numpy as np


# ── CDR offsets for sensor_msgs/Imu from livox_ros_driver2 ───────────────────
# frame_id = "livox_frame" (12 bytes incl null)
# Verified against bags from this robot.
_OFF_WX, _OFF_WY, _OFF_WZ = 132, 140, 148   # angular_velocity
_OFF_AX, _OFF_AY, _OFF_AZ = 228, 236, 244   # linear_acceleration
_MIN_MSG_LEN = 252


def _find_db(bag_dir: str) -> str:
    dbs = glob.glob(os.path.join(bag_dir, "*.db3"))
    if not dbs:
        raise FileNotFoundError(f"No .db3 file found in {bag_dir}")
    return sorted(dbs)[0]


def read_imu(bag_dir: str):
    """Return (times_s, wx, wy, wz, acc_norm) as numpy arrays."""
    import sqlite3
    db = _find_db(bag_dir)
    conn = sqlite3.connect(db)
    row = conn.execute("SELECT id FROM topics WHERE name='/livox/imu'").fetchone()
    if row is None:
        conn.close()
        raise RuntimeError("/livox/imu not found in bag")
    tid = row[0]
    rows = conn.execute(
        "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp", (tid,)
    ).fetchall()
    conn.close()

    if not rows:
        raise RuntimeError("No /livox/imu messages")

    t0 = rows[0][0] / 1e9
    times, wx_a, wy_a, wz_a, anorm_a = [], [], [], [], []

    for ts, data in rows:
        buf = bytes(data)
        if len(buf) < _MIN_MSG_LEN:
            continue
        try:
            wx = struct.unpack_from("<d", buf, _OFF_WX)[0]
            wy = struct.unpack_from("<d", buf, _OFF_WY)[0]
            wz = struct.unpack_from("<d", buf, _OFF_WZ)[0]
            ax = struct.unpack_from("<d", buf, _OFF_AX)[0]
            ay = struct.unpack_from("<d", buf, _OFF_AY)[0]
            az = struct.unpack_from("<d", buf, _OFF_AZ)[0]
        except struct.error:
            continue
        times.append(ts / 1e9 - t0)
        wx_a.append(wx); wy_a.append(wy); wz_a.append(wz)
        anorm_a.append((ax*ax + ay*ay + az*az) ** 0.5)

    return (np.asarray(times), np.asarray(wx_a), np.asarray(wy_a),
            np.asarray(wz_a), np.asarray(anorm_a))


def rolling_rms(times: np.ndarray, values: np.ndarray, window_s: float, step_s: float = 0.5):
    """Sliding-window RMS sampled every step_s seconds."""
    t_out = np.arange(times[0] + window_s, times[-1], step_s)
    rms_out = np.empty(len(t_out))
    for i, t in enumerate(t_out):
        mask = (times >= t - window_s) & (times <= t)
        rms_out[i] = float(np.sqrt(np.mean(values[mask] ** 2))) if mask.any() else 0.0
    return t_out, rms_out


def find_clean_start(t_win, pitch_roll_rms, acc_dev_rms, pr_thr, acc_thr, hold_s) -> float:
    """
    First time where pitch/roll RMS AND acc dev RMS both stay below threshold
    for hold_s seconds.  Uses pitch/roll (wx, wy) NOT total gyro so that
    normal yaw turns don't get flagged.
    """
    clean = (pitch_roll_rms < pr_thr) & (acc_dev_rms < acc_thr)
    for i, t in enumerate(t_win):
        future = (t_win >= t) & (t_win <= t + hold_s)
        if future.any() and np.all(clean[future]):
            return max(0.0, t - 1.0)
    return 0.0


def analyse_poses(path: str) -> None:
    poses = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line:
                poses.append(list(map(float, line.split())))
    if not poses:
        print("  (empty)")
        return
    x = [p[3] for p in poses]
    y = [p[7] for p in poses]
    z = [p[11] for p in poses]
    suspect = sum(1 for v in z if abs(v) > 0.3)
    print(f"  Keyframes : {len(poses)}")
    print(f"  Z span    : {max(z)-min(z):.3f} m  ({min(z):.3f} → {max(z):.3f})")
    print(f"  XY span   : {max(x)-min(x):.1f} × {max(y)-min(y):.1f} m")
    print(f"  Suspect Z : {suspect}/{len(poses)} keyframes with |Z| > 0.3 m")
    print(f"\n  {'Frame':>5}  {'X':>7}  {'Y':>7}  {'Z':>7}")
    for i, p in enumerate(poses):
        flag = "  <-- drift" if abs(p[11]) > 0.3 else ""
        print(f"  {i:>5}  {p[3]:>7.2f}  {p[7]:>7.2f}  {p[11]:>7.3f}{flag}")
    return z


def make_plot(times, wx, wy, wz, acc_norm,
              t_win, pr_rms, wz_rms, acc_dev_rms,
              offset, pr_thr, acc_thr,
              poses_z=None, poses_path=None):
    try:
        import matplotlib.pyplot as plt
        import matplotlib.gridspec as gridspec
    except ImportError:
        print("\n(Install matplotlib to see the plot:  pip3 install matplotlib)")
        return

    n_rows = 4 if poses_z is None else 5
    fig = plt.figure(figsize=(14, 3.0 * n_rows))
    gs = gridspec.GridSpec(n_rows, 1, hspace=0.45)
    axs = [fig.add_subplot(gs[i]) for i in range(n_rows)]

    kw_raw = dict(lw=0.4, alpha=0.35)
    kw_rms = dict(lw=1.8)

    # ── 1. Pitch & roll angular velocity ────────────────────────────────────
    axs[0].plot(times, np.abs(wx), color="tab:blue",  label="|wx| pitch", **kw_raw)
    axs[0].plot(times, np.abs(wy), color="tab:cyan",  label="|wy| roll",  **kw_raw)
    axs[0].plot(t_win, pr_rms,     color="tab:blue",  label="pitch/roll RMS", **kw_rms)
    axs[0].axhline(pr_thr, color="red", ls="--", lw=1, label=f"threshold {pr_thr}")
    axs[0].axvline(offset, color="green", lw=2, label=f"start offset = {offset:.1f}s")
    axs[0].set_ylabel("rad/s")
    axs[0].set_title("Pitch & roll rate  (manual handling → high;  flat-floor driving → near zero)")
    axs[0].legend(fontsize=7, loc="upper right", ncol=3)
    axs[0].grid(True, alpha=0.25)

    # ── 2. Yaw angular velocity ───────────────────────────────────────────────
    axs[1].plot(times, np.abs(wz), color="tab:purple", label="|wz| yaw", **kw_raw)
    axs[1].plot(t_win, wz_rms,    color="tab:purple", label="yaw RMS",  **kw_rms)
    axs[1].axvline(offset, color="green", lw=2)
    axs[1].set_ylabel("rad/s")
    axs[1].set_title("Yaw rate  (spikes = normal turns, not a problem)")
    axs[1].legend(fontsize=7, loc="upper right")
    axs[1].grid(True, alpha=0.25)

    # ── 3. Accelerometer norm deviation from 1g ──────────────────────────────
    acc_dev = np.abs(acc_norm - 1.0)
    axs[2].plot(times, acc_dev,     color="darkorange", label="|‖acc‖ − 1g|", **kw_raw)
    axs[2].plot(t_win, acc_dev_rms, color="darkorange", label="acc dev RMS",  **kw_rms)
    axs[2].axhline(acc_thr, color="red", ls="--", lw=1, label=f"threshold {acc_thr}")
    axs[2].axvline(offset, color="green", lw=2)
    axs[2].set_ylabel("g-units")
    axs[2].set_title("Acceleration deviation from 1g  (stable ≈ flat floor; spikes = bumps / lifting)")
    axs[2].legend(fontsize=7, loc="upper right")
    axs[2].grid(True, alpha=0.25)

    # ── 4. Accelerometer Z component (gravity axis) ──────────────────────────
    axs[3].plot(times, acc_norm, color="tab:brown", label="‖acc‖ (g-units)", **kw_raw)
    axs[3].axhline(1.0, color="gray", ls=":", lw=1, label="1g")
    axs[3].axvline(offset, color="green", lw=2, label=f"start offset = {offset:.1f}s")
    axs[3].set_ylabel("g-units")
    axs[3].set_title("Total acceleration norm  (should stay ≈ 1g on flat floor)")
    axs[3].legend(fontsize=7, loc="upper right")
    axs[3].grid(True, alpha=0.25)
    if poses_z is None:
        axs[3].set_xlabel("Time into bag (s)")

    # ── 5. Keyframe Z trajectory (if poses provided) ──────────────────────────
    if poses_z is not None:
        axs[4].plot(range(len(poses_z)), poses_z, "o-", color="tab:red", lw=1.5, ms=4)
        axs[4].axhline(0, color="gray", ls=":", lw=1)
        axs[4].axhspan(-0.3, 0.3, alpha=0.15, color="green", label="±0.3 m target zone")
        axs[4].set_xlabel("Keyframe index")
        axs[4].set_ylabel("Z (m)")
        title = f"Keyframe Z trajectory  —  {poses_path}" if poses_path else "Keyframe Z trajectory"
        axs[4].set_title(title)
        axs[4].legend(fontsize=7)
        axs[4].grid(True, alpha=0.25)

    fig.suptitle("IMU quality analysis  —  find_bag_start.py", fontsize=11, y=1.01)
    plt.tight_layout()
    plt.show()


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("bag_dir", help="ROS2 bag directory containing *.db3")
    ap.add_argument("--poses", metavar="PATH",
                    help="poses_kitti.txt to analyse and include in plot")
    ap.add_argument("--no-plot", action="store_true",
                    help="suppress the matplotlib chart")
    ap.add_argument("--pr-threshold", type=float, default=0.10,
                    help="pitch/roll RMS threshold for 'clean' (rad/s) [default: 0.10]")
    ap.add_argument("--acc-threshold", type=float, default=0.08,
                    help="acc dev RMS threshold (g-units) [default: 0.08]")
    ap.add_argument("--hold", type=float, default=5.0,
                    help="seconds both metrics must stay clean [default: 5]")
    ap.add_argument("--window", type=float, default=2.0,
                    help="rolling-window size (seconds) [default: 2]")
    args = ap.parse_args()

    print(f"Reading /livox/imu from {args.bag_dir} …")
    times, wx, wy, wz, acc_norm = read_imu(args.bag_dir)
    print(f"  {len(times)} messages  ·  {times[-1]:.1f} s total\n")

    # Pitch/roll magnitude (what we actually care about for handling detection)
    pr_mag = np.sqrt(wx**2 + wy**2)
    acc_dev = np.abs(acc_norm - 1.0)

    t_win, pr_rms  = rolling_rms(times, pr_mag,  args.window)
    _,     wz_rms  = rolling_rms(times, np.abs(wz), args.window)
    _,     acc_rms = rolling_rms(times, acc_dev, args.window)

    offset = find_clean_start(t_win, pr_rms, acc_rms,
                              args.pr_threshold, args.acc_threshold, args.hold)

    print("=" * 58)
    print(f"  Suggested --start-offset : {offset:.1f} s")
    print("=" * 58)
    print(f"\n  ros2 bag play <bag> --clock --rate 0.5 --start-offset {offset:.1f}")

    print(f"\n  Peak pitch/roll RMS : {pr_rms.max():.4f} rad/s  at t={t_win[pr_rms.argmax()]:.1f}s")
    print(f"  Peak yaw RMS        : {wz_rms.max():.4f} rad/s  at t={t_win[wz_rms.argmax()]:.1f}s")
    print(f"  Peak acc dev RMS    : {acc_rms.max():.4f} g      at t={t_win[acc_rms.argmax()]:.1f}s")

    # Per-10s table
    print(f"\n  {'t(s)':>6}  {'pitchRollRMS':>13}  {'yawRMS':>8}  {'accDev':>8}  status")
    print("  " + "-" * 55)
    prev = -10.0
    for t, pr, wz_r, ad in zip(t_win, pr_rms, wz_rms, acc_rms):
        if t - prev < 10.0:
            continue
        ok = pr < args.pr_threshold and ad < args.acc_threshold
        marker = "CLEAN" if ok else "noisy"
        star = "  <-- suggested start" if abs(t - offset) < 1.5 else ""
        print(f"  {t:>6.1f}  {pr:>13.4f}  {wz_r:>8.4f}  {ad:>8.4f}  {marker}{star}")
        prev = t

    # Poses analysis
    poses_z = None
    if args.poses:
        if not os.path.exists(args.poses):
            print(f"\nWARNING: poses file not found: {args.poses}")
        else:
            print(f"\nPoses: {args.poses}")
            poses_z = analyse_poses(args.poses)

    # Plot
    if not args.no_plot:
        make_plot(times, wx, wy, wz, acc_norm,
                  t_win, pr_rms, wz_rms, acc_rms,
                  offset, args.pr_threshold, args.acc_threshold,
                  poses_z=poses_z, poses_path=args.poses)


if __name__ == "__main__":
    main()
