#!/usr/bin/env python3
"""
Live localization dashboard + end-of-run summary.

Live display (refreshes every second):
  Pose, uncertainty (covariance σ), motion, smoothness, scan matching quality.

End-of-run summary (printed on shutdown):
  Trajectory stats, smoothness over full run, instability events,
  frequency consistency, start→end drift, covariance stats.

Run standalone:
    source ~/ws_livox/install/setup.bash && source ~/spark_lio/install/setup.bash
    python3 scripts/localization_monitor.py
"""

import time
import math
import collections
import argparse
import threading

import numpy as np
from scipy.spatial import KDTree
from rich.console import Console
from rich.table import Table
from rich import box as rbox
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker


LIVE_WINDOW      = 20     # frames for live smoothness display
JUMP_POS_THRESH  = 0.20   # m   — flag as instability event
JUMP_YAW_THRESH  = 5.0    # °   — flag as instability event
MATCH_WINDOW_S   = 2.0    # seconds per scan-match reporting window
MATCH_SAMPLES    = 400    # scan points sampled per match evaluation


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def _yaw_from_quat(q) -> float:
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))


def _count_log_levels(log_file: str):
    if not log_file:
        return 0, 0
    try:
        lines = open(log_file, errors='replace').readlines()
    except OSError:
        return 0, 0
    warns  = sum(1 for l in lines if 'WARN'  in l)
    errors = sum(1 for l in lines if 'ERROR' in l and 'process has died' not in l)
    return warns, errors


class LocalizationMonitor(Node):
    def __init__(self, log_file: str, console: Console):
        super().__init__('localization_monitor')
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.FATAL)
        self._log_file = log_file
        self._console  = console

        self._t0        = time.time()
        self._last_msg  = None
        self._prev_pose = None
        self._prev_wall = None

        self._live_dpos = collections.deque(maxlen=LIVE_WINDOW)
        self._live_dyaw = collections.deque(maxlen=LIVE_WINDOW)

        self._all_poses    = []
        self._all_dpos     = []
        self._all_dyaw_deg = []
        self._all_dt       = []
        self._all_cov_x    = []
        self._all_cov_y    = []
        self._all_cov_yaw  = []

        self._msg_count  = 0
        self._last_count = 0
        self._last_hz_t  = time.time()
        self._hz         = 0.0

        self._map_kdtree     = None
        self._map_building   = False
        self._window_scores  = []
        self._match_best     = None
        self._match_worst    = None
        self._match_n_frames = 0
        self._window_t0      = time.time()

        self._marker_pub = self.create_publisher(Marker, '/robot_marker', 10)
        self._path_pub   = self.create_publisher(Path,   '/robot_path',   10)
        self._path_msg   = Path()
        self._path_msg.header.frame_id = 'map'
        self.create_subscription(Odometry,    '/laser_odometry',  self._on_odom, 10)
        self.create_subscription(PointCloud2, '/overall_map',     self._on_map,   1)
        self.create_subscription(PointCloud2, '/registered_scan', self._on_scan, 10)
        self.create_timer(1.0, self._refresh)

    # ── Subscribers ───────────────────────────────────────────────────────────

    def _on_odom(self, msg: Odometry):
        self._msg_count += 1
        self._last_msg   = msg
        now = time.time()

        x   = msg.pose.pose.position.x
        y   = msg.pose.pose.position.y
        yaw = _yaw_from_quat(msg.pose.pose.orientation)

        self._all_poses.append((now, x, y, yaw))
        self._publish_marker(msg)
        self._publish_path(msg)

        cov = msg.pose.covariance
        self._all_cov_x.append(cov[0])
        self._all_cov_y.append(cov[7])
        self._all_cov_yaw.append(cov[35])

        if self._prev_pose is not None:
            px, py, pyaw = self._prev_pose
            dp   = math.hypot(x - px, y - py)
            dyaw = abs(math.degrees(_wrap(yaw - pyaw)))
            dt   = now - (self._prev_wall or now)
            self._live_dpos.append(dp)
            self._live_dyaw.append(dyaw)
            self._all_dpos.append(dp)
            self._all_dyaw_deg.append(dyaw)
            self._all_dt.append(dt)

        self._prev_pose = (x, y, yaw)
        self._prev_wall = now

    def _publish_path(self, odom_msg: Odometry):
        ps = PoseStamped()
        ps.header = odom_msg.header
        ps.pose   = odom_msg.pose.pose
        self._path_msg.header.stamp = odom_msg.header.stamp
        self._path_msg.poses.append(ps)
        self._path_pub.publish(self._path_msg)

    def _publish_marker(self, odom_msg: Odometry):
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp    = odom_msg.header.stamp
        m.ns, m.id        = 'robot', 0
        m.type            = Marker.CUBE
        m.action          = Marker.ADD
        m.pose            = odom_msg.pose.pose
        m.scale.x, m.scale.y, m.scale.z = 0.55, 0.45, 0.25
        m.color.r = m.color.g = m.color.b = 0.65
        m.color.a = 1.0
        self._marker_pub.publish(m)

    # ── Scan matching ─────────────────────────────────────────────────────────

    @staticmethod
    def _parse_cloud_xy(msg: PointCloud2) -> np.ndarray:
        raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        raw = raw.reshape(msg.width * msg.height, msg.point_step)
        x   = raw[:, 0:4].view(np.float32).reshape(-1)
        y   = raw[:, 4:8].view(np.float32).reshape(-1)
        ok  = ~(np.isnan(x) | np.isnan(y) | np.isinf(x) | np.isinf(y))
        return np.column_stack([x[ok], y[ok]])

    def _on_map(self, msg: PointCloud2):
        if self._map_kdtree is not None or self._map_building:
            return
        self._map_building = True
        def _build():
            xy = self._parse_cloud_xy(msg)
            if len(xy) > 100:
                self._map_kdtree = KDTree(xy)
        threading.Thread(target=_build, daemon=True).start()

    def _on_scan(self, msg: PointCloud2):
        if self._map_kdtree is None:
            return
        xy = self._parse_cloud_xy(msg)
        if len(xy) < 10:
            return
        idx      = np.random.choice(len(xy), min(MATCH_SAMPLES, len(xy)), replace=False)
        dists, _ = self._map_kdtree.query(xy[idx])
        self._window_scores.append(float(np.mean(dists)))
        now = time.time()
        if now - self._window_t0 >= MATCH_WINDOW_S and self._window_scores:
            self._match_best     = float(min(self._window_scores))
            self._match_worst    = float(max(self._window_scores))
            self._match_n_frames = len(self._window_scores)
            self._window_scores  = []
            self._window_t0      = now

    # ── Live dashboard ────────────────────────────────────────────────────────

    def _refresh(self):
        now = time.time()
        dt  = now - self._last_hz_t
        if dt >= 1.0:
            self._hz         = (self._msg_count - self._last_count) / dt
            self._last_count = self._msg_count
            self._last_hz_t  = now
        self._console.clear()
        self._console.print(self._build_table(now - self._t0))

    def _build_table(self, elapsed: float) -> Table:
        t = Table(box=rbox.SIMPLE_HEAVY, show_header=False, padding=(0, 2), expand=False)
        t.add_column("metric", style="white",    min_width=28, no_wrap=True)
        t.add_column("value",  style="cyan bold", min_width=10, justify="right", no_wrap=True)
        t.add_column("unit",   style="dim",       min_width=4,  no_wrap=True)

        warns, errors = _count_log_levels(self._log_file)

        t.add_row("[bold]Localization Monitor[/bold]", "", "")
        t.add_section()
        t.add_row("elapsed",       f"{elapsed:.1f}",     "s")
        t.add_row("rate",          f"{self._hz:.1f}",    "Hz")
        t.add_row("node warnings", f"{warns}",           "")
        t.add_row("node errors",   f"{errors}",
                  "[red]⚠[/red]" if errors else "")

        if self._last_msg is None:
            t.add_section()
            t.add_row("waiting for /laser_odometry ...", "", "")
            return t

        msg = self._last_msg
        x   = msg.pose.pose.position.x
        y   = msg.pose.pose.position.y
        yaw = _yaw_from_quat(msg.pose.pose.orientation)
        spd = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        oz  = msg.twist.twist.angular.z
        cov = msg.pose.covariance
        cov_valid = (cov[0] + cov[7] + cov[35]) > 1e-9

        n = len(self._live_dpos)
        if n >= 2:
            jpos = float(np.std(self._live_dpos))
            jyaw = float(np.std(self._live_dyaw))
            mjmp = float(max(self._live_dpos))
        else:
            jpos = jyaw = mjmp = float('nan')

        t.add_section()
        t.add_row("[bold]POSE[/bold]", "", "")
        t.add_row("  x",   f"{x:+.4f}",               "m")
        t.add_row("  y",   f"{y:+.4f}",               "m")
        t.add_row("  yaw", f"{math.degrees(yaw):+.2f}", "°")

        t.add_section()
        t.add_row("[bold]UNCERTAINTY[/bold]", "", "")
        if cov_valid:
            t.add_row("  σ_x",   f"{math.sqrt(cov[0]):.4f}",                "m")
            t.add_row("  σ_y",   f"{math.sqrt(cov[7]):.4f}",                "m")
            t.add_row("  σ_yaw", f"{math.degrees(math.sqrt(cov[35])):.2f}", "°")
        else:
            t.add_row("  not reported by this pipeline", "", "")

        t.add_section()
        t.add_row("[bold]MOTION[/bold]", "", "")
        t.add_row("  speed", f"{spd:.3f}",               "m/s")
        t.add_row("  ω",     f"{math.degrees(oz):+.2f}", "°/s")

        t.add_section()
        t.add_row(f"[bold]SMOOTHNESS[/bold]  (last {n}/{LIVE_WINDOW} frames)", "", "")
        if not math.isnan(jpos):
            t.add_row("  position jitter",  f"{jpos*100:.2f}",  "cm")
            t.add_row("  yaw jitter",        f"{jyaw:.2f}",      "°")
            t.add_row("  max position jump", f"{mjmp*100:.2f}",  "cm")
        else:
            t.add_row("  collecting...", "", "")

        t.add_section()
        t.add_row(f"[bold]SCAN MATCHING[/bold]  (best/worst over {MATCH_WINDOW_S:.0f} s)", "", "")
        if self._map_kdtree is None:
            t.add_row("  building map KD-tree...", "", "")
        elif self._match_best is None:
            t.add_row("  collecting...", "", "")
        else:
            t.add_row("  best  (mean NN dist)",  f"{self._match_best*100:.2f}",  "cm")
            t.add_row("  worst (mean NN dist)",  f"{self._match_worst*100:.2f}", "cm")

        return t

    # ── End-of-run summary ────────────────────────────────────────────────────

    def print_summary(self):
        n = len(self._all_poses)
        if n < 2:
            print("\n[monitor] Not enough data for end-of-run summary.")
            return

        poses = np.array([(x, y, yaw) for (_, x, y, yaw) in self._all_poses])
        times = np.array([t for (t, _, _, _) in self._all_poses])
        dpos  = np.array(self._all_dpos)
        dyaw  = np.array(self._all_dyaw_deg)
        dts   = np.array(self._all_dt)

        duration   = times[-1] - times[0]
        total_dist = float(dpos.sum())
        total_rot  = float(dyaw.sum())
        avg_speed  = total_dist / duration if duration > 0 else 0.0
        mean_hz    = float(1.0 / dts.mean())  if len(dts) else 0.0
        std_hz     = float(np.std(1.0 / dts)) if len(dts) else 0.0
        pos_jitter = float(dpos.std())
        yaw_jitter = float(dyaw.std())
        max_jump   = float(dpos.max())
        max_yjump  = float(dyaw.max())
        pos_events = int((dpos > JUMP_POS_THRESH).sum())
        yaw_events = int((dyaw > JUMP_YAW_THRESH).sum())

        x0, y0     = poses[0, 0], poses[0, 1]
        xf, yf     = poses[-1, 0], poses[-1, 1]
        drift      = math.hypot(xf - x0, yf - y0)
        yaw_drift  = abs(math.degrees(_wrap(poses[-1, 2] - poses[0, 2])))

        cov_x   = np.array(self._all_cov_x)
        cov_y   = np.array(self._all_cov_y)
        cov_yaw = np.array(self._all_cov_yaw)
        cov_valid = (cov_x.sum() + cov_y.sum() + cov_yaw.sum()) > 1e-9

        console = Console()
        t = Table(title="Localization Summary", box=rbox.SIMPLE_HEAVY,
                  show_header=False, padding=(0, 2), expand=False)
        t.add_column("metric", style="white",     min_width=30, no_wrap=True)
        t.add_column("value",  style="cyan bold",  min_width=10, justify="right", no_wrap=True)
        t.add_column("unit",   style="dim",        min_width=4,  no_wrap=True)

        def s(label, value, unit=''):
            t.add_row(f"  {label}", value, unit)

        t.add_row("[bold]TRAJECTORY[/bold]", "", "")
        s("duration",         f"{duration:.1f}",    "s")
        s("total distance",   f"{total_dist:.2f}",  "m")
        s("total rotation",   f"{total_rot:.1f}",   "°")
        s("average speed",    f"{avg_speed:.3f}",   "m/s")
        s("frames received",  f"{n}",               "")

        t.add_section()
        t.add_row("[bold]FREQUENCY[/bold]", "", "")
        s("mean rate",           f"{mean_hz:.2f}",          "Hz")
        s("rate std dev",        f"{std_hz:.2f}",           "Hz")
        s("min inter-frame gap", f"{dts.min()*1000:.1f}",   "ms")
        s("max inter-frame gap", f"{dts.max()*1000:.1f}",   "ms")

        t.add_section()
        t.add_row("[bold]SMOOTHNESS[/bold]  (full run)", "", "")
        s("mean pos jump / frame", f"{dpos.mean()*100:.2f}", "cm")
        s("pos jitter (σ)",        f"{pos_jitter*100:.2f}",  "cm")
        s("max pos jump",          f"{max_jump*100:.2f}",    "cm")
        s("mean yaw jump / frame", f"{dyaw.mean():.2f}",     "°")
        s("yaw jitter (σ)",        f"{yaw_jitter:.2f}",      "°")
        s("max yaw jump",          f"{max_yjump:.2f}",       "°")

        t.add_section()
        t.add_row("[bold]INSTABILITY EVENTS[/bold]", "", "")
        s(f"pos jumps  > {JUMP_POS_THRESH*100:.0f} cm",
          f"{pos_events}",  f"{'⚠' if pos_events else '✓'}  {pos_events/n*100:.1f}% of frames")
        s(f"yaw jumps  > {JUMP_YAW_THRESH:.0f} °",
          f"{yaw_events}", f"{'⚠' if yaw_events else '✓'}  {yaw_events/n*100:.1f}% of frames")

        t.add_section()
        t.add_row("[bold]START → END DRIFT[/bold]", "", "")
        s("position drift", f"{drift:.3f}",     "m")
        s("yaw drift",      f"{yaw_drift:.2f}", "°")

        t.add_section()
        t.add_row("[bold]UNCERTAINTY[/bold]  (covariance σ, full run)", "", "")
        if cov_valid:
            s("mean σ_x",   f"{np.sqrt(cov_x.mean()):.4f}",                         "m")
            s("mean σ_y",   f"{np.sqrt(cov_y.mean()):.4f}",                         "m")
            s("mean σ_yaw", f"{math.degrees(np.sqrt(cov_yaw.mean())):.2f}",          "°")
            s("max σ_x",    f"{np.sqrt(cov_x.max()):.4f}",                          "m")
            s("max σ_y",    f"{np.sqrt(cov_y.max()):.4f}",                          "m")
        else:
            t.add_row("  not reported by this pipeline", "", "")

        console.print(t)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--log-file', default='',
                        help='Path to launch.log for warning/error counts')
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    console = Console(force_terminal=True)
    node = LocalizationMonitor(log_file=args.log_file, console=console)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    node.print_summary()


if __name__ == '__main__':
    main()
