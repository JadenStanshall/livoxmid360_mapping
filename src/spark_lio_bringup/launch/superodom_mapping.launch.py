"""
superodom_mapping.launch.py — LiDAR-only mapping pipeline using SuperOdom.

Brings up:
  1. feature_extraction_node  — Livox CustomMsg → edge/planar features
  2. laser_mapping_node       — LiDAR-only SLAM (no IMU in pose loop)
  3. imu_preintegration_node  — launches but stays idle (imu_topic is empty)
  4. ros2 bag play            — replays the recorded bag (starts 5 s after nodes)

With imu_topic set to "" in superodom_lidar_only.yaml, featureExtraction detects
an empty IMU buffer and runs the lidar-only fallback path: no point deskewing,
identity quaternion for scan initialisation.

Map output: saved incrementally to
  src/SuperOdom/super_odometry/PLY/saved_scans.ply
The run_superodom_mapping.sh script copies this to <output_dir>/globalMap.ply
after the bag finishes.

Usage:
    ros2 launch spark_lio_bringup superodom_mapping.launch.py bag_path:=/path/to/bag
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_ros


def _default_output_dir() -> str:
    pkg_share = get_package_share_directory("spark_lio_bringup")
    ws_root = os.path.normpath(os.path.join(pkg_share, "..", "..", "..", ".."))
    candidate = os.path.join(ws_root, "maps")
    return candidate if os.path.isdir(candidate) else os.path.expanduser("~/spark_lio/maps")


def generate_launch_description():
    pkg_share = FindPackageShare("spark_lio_bringup")
    superodom_share = FindPackageShare("super_odometry")

    bag_path_arg = DeclareLaunchArgument(
        "bag_path",
        description="Path to the ROS 2 bag directory to replay.",
    )
    output_dir_arg = DeclareLaunchArgument(
        "output_dir",
        default_value=_default_output_dir(),
        description="Directory where the final map PLY is copied after the run.",
    )

    config = PathJoinSubstitution([pkg_share, "config", "superodom_lidar_only.yaml"])
    calib  = PathJoinSubstitution([superodom_share, "config", "livox", "livox_mid360_calibration.yaml"])

    feature_extraction_node = Node(
        package="super_odometry",
        executable="feature_extraction_node",
        parameters=[config, {"calibration_file": calib, "use_sim_time": True}],
        output="screen",
        emulate_tty=True,
    )

    laser_mapping_node = Node(
        package="super_odometry",
        executable="laser_mapping_node",
        parameters=[config, {"calibration_file": calib, "use_sim_time": True}],
        output="screen",
        emulate_tty=True,
    )

    # Idle when imu_topic is empty — launched for completeness in case laserMapping
    # publishes to a topic imuPreintegration is expected to relay.
    imu_preintegration_node = Node(
        package="super_odometry",
        executable="imu_preintegration_node",
        parameters=[config, {"calibration_file": calib, "use_sim_time": True}],
        output="screen",
        emulate_tty=True,
    )

    # 5 s delay — SuperOdom needs slightly longer to initialise than spark-fast-lio.
    bag_play = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2", "bag", "play",
                    LaunchConfiguration("bag_path"),
                    "--clock",
                    "--rate", "0.5",
                ],
                output="screen",
            )
        ],
    )

    return LaunchDescription([
        launch_ros.actions.SetParameter(name="use_sim_time", value="true"),
        bag_path_arg,
        output_dir_arg,
        feature_extraction_node,
        laser_mapping_node,
        imu_preintegration_node,
        bag_play,
    ])
