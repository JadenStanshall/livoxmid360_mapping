"""
Mapping launch file.

Brings up the full offline mapping pipeline:
  1. spark-fast-lio  — LiDAR-inertial odometry
  2. KISS-Matcher-SAM — keyframe selection, loop closure, pose graph optimisation
  3. ros2 bag play   — replays the recorded bag (starts 3 s after nodes are ready)
  4. RViz2           — optional live visualisation

Usage:
    ros2 launch spark_lio_bringup mapping.launch.py bag_path:=/path/to/bag

Arguments:
    bag_path    (required) Path to the ROS 2 bag directory to replay.
    output_dir  (optional) Directory where globalMap.pcd and keyframe PCDs are saved.
                           Defaults to <workspace>/maps/.
    use_rviz    (optional) Launch RViz2 for live visualisation. Default: true.

When the bag finishes, CTRL-C the launch to trigger KISS-Matcher-SAM's shutdown
handler, which saves globalMap.pcd and keyframe PCDs to output_dir.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _default_output_dir() -> str:
    """Resolve <workspace>/maps/ relative to the installed package share path."""
    pkg_share = get_package_share_directory("spark_lio_bringup")
    # install/<pkg>/share/<pkg>  →  up 4 levels = workspace root
    ws_root = os.path.normpath(os.path.join(pkg_share, "..", "..", "..", ".."))
    candidate = os.path.join(ws_root, "maps")
    return candidate if os.path.isdir(candidate) else os.path.expanduser("~/spark_lio/maps")


def generate_launch_description():
    pkg_share = FindPackageShare("spark_lio_bringup")

    # ── Launch arguments ────────────────────────────────────────────────────────
    bag_path_arg = DeclareLaunchArgument(
        "bag_path",
        description="Path to the ROS 2 bag directory to replay.",
    )

    output_dir_arg = DeclareLaunchArgument(
        "output_dir",
        default_value=_default_output_dir(),
        description="Directory where KISS-Matcher-SAM saves globalMap.pcd and keyframe PCDs.",
    )

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Launch RViz2 for live visualisation.",
    )

    # ── spark-fast-lio ──────────────────────────────────────────────────────────
    fast_lio_node = Node(
        package="spark_fast_lio",
        executable="spark_lio_mapping",
        name="spark_fast_lio",
        parameters=[
            PathJoinSubstitution([pkg_share, "config", "mid360_fast_lio.yaml"]),
            {"use_sim_time": True},  # required when replaying a bag with --clock
        ],
        remappings=[
            ("lidar", "/livox/lidar"),
            ("imu",   "/livox/imu"),
        ],
        output="screen",
        emulate_tty=True,
    )

    # ── KISS-Matcher-SAM ────────────────────────────────────────────────────────
    kiss_matcher_node = Node(
        package="kiss_matcher_ros",
        executable="kiss_matcher_sam",
        name="kiss_matcher_sam",
        parameters=[
            PathJoinSubstitution([pkg_share, "config", "kiss_matcher_sam.yaml"]),
            {"use_sim_time": True},
        ],
        # The node subscribes to hardcoded /odom and /cloud; remap to spark-fast-lio outputs.
        remappings=[
            ("/odom",  "/odometry"),
            ("/cloud", "/cloud_registered"),
        ],
        output="screen",
        emulate_tty=True,
    )

    # ── RViz2 ───────────────────────────────────────────────────────────────────
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", PathJoinSubstitution([pkg_share, "rviz", "mapping.rviz"])],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        output="screen",
    )

    # ── Bag replay ──────────────────────────────────────────────────────────────
    # Delayed by 3 s to give nodes time to initialise before the first messages arrive.
    # --rate 0.5: half-speed replay so the IEKF (max_iteration=10) has enough wall-clock
    # time per scan to converge before Z residuals grow past the acceptance threshold.
    bag_play = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=["ros2", "bag", "play", LaunchConfiguration("bag_path"), "--clock", "--rate", "0.5"],
                output="screen",
            )
        ],
    )

    return LaunchDescription([
        bag_path_arg,
        output_dir_arg,
        use_rviz_arg,
        fast_lio_node,
        kiss_matcher_node,
        rviz_node,
        bag_play,
    ])
