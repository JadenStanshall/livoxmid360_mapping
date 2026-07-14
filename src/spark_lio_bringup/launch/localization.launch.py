"""
Localization launch file.

Brings up SuperOdom in localization_mode against a pre-built prior map.
Launches all three required SuperOdom nodes:
  - feature_extraction_node
  - imu_preintegration_node
  - laser_mapping_node (localization_mode: true)

Usage:
    ros2 launch spark_lio_bringup localization.launch.py map_path:=/path/to/final_map.pcd

Arguments:
    map_path    (required) Absolute path to final_map.pcd (output of postprocess_map.py).
    x, y, z     Initial position of the robot in the map frame (metres). Default: 0.0.
    roll, pitch, yaw  Initial orientation (radians). Default: 0.0.
    use_rviz    Launch RViz2 for visualisation. Default: true.

The robot must start from a known position in the map. If the initial pose is not
the origin, supply x/y/yaw at minimum.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _default_map_path() -> str:
    """Resolve <workspace>/maps/final_map.pcd relative to the installed package."""
    pkg_share = get_package_share_directory("spark_lio_bringup")
    ws_root = os.path.normpath(os.path.join(pkg_share, "..", "..", "..", ".."))
    return os.path.join(ws_root, "maps", "final_map.pcd")


def generate_launch_description():
    bringup_share = FindPackageShare("spark_lio_bringup")
    superodom_share = get_package_share_directory("super_odometry")

    superodom_config   = os.path.join(superodom_share, "config", "livox_mid360.yaml")
    superodom_calib    = os.path.join(superodom_share, "config", "livox", "livox_mid360_calibration.yaml")

    # ── Launch arguments ────────────────────────────────────────────────────────
    map_path_arg = DeclareLaunchArgument(
        "map_path",
        default_value=_default_map_path(),
        description="Absolute path to final_map.pcd (prior map for localization).",
    )

    x_arg   = DeclareLaunchArgument("x",     default_value="0.0", description="Initial X (m)")
    y_arg   = DeclareLaunchArgument("y",     default_value="0.0", description="Initial Y (m)")
    z_arg   = DeclareLaunchArgument("z",     default_value="0.0", description="Initial Z (m)")
    ro_arg  = DeclareLaunchArgument("roll",  default_value="0.0", description="Initial roll (rad)")
    pi_arg  = DeclareLaunchArgument("pitch", default_value="0.0", description="Initial pitch (rad)")
    ya_arg  = DeclareLaunchArgument("yaw",   default_value="0.0", description="Initial yaw (rad)")

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Launch RViz2 for visualisation.",
    )

    # ── SuperOdom nodes ─────────────────────────────────────────────────────────
    feature_extraction_node = Node(
        package="super_odometry",
        executable="feature_extraction_node",
        parameters=[
            superodom_config,
            {"calibration_file": superodom_calib},
        ],
        output="screen",
        emulate_tty=True,
    )

    imu_preintegration_node = Node(
        package="super_odometry",
        executable="imu_preintegration_node",
        parameters=[
            superodom_config,
            {"calibration_file": superodom_calib},
        ],
        output="screen",
        emulate_tty=True,
    )

    laser_mapping_node = Node(
        package="super_odometry",
        executable="laser_mapping_node",
        parameters=[
            superodom_config,
            {
                "calibration_file": superodom_calib,
                "laser_mapping_node.localization_mode": True,
                "map_dir": LaunchConfiguration("map_path"),
                "laser_mapping_node.init_x":     LaunchConfiguration("x"),
                "laser_mapping_node.init_y":     LaunchConfiguration("y"),
                "laser_mapping_node.init_z":     LaunchConfiguration("z"),
                "laser_mapping_node.init_roll":  LaunchConfiguration("roll"),
                "laser_mapping_node.init_pitch": LaunchConfiguration("pitch"),
                "laser_mapping_node.init_yaw":   LaunchConfiguration("yaw"),
            },
        ],
        output="screen",
        emulate_tty=True,
    )

    # ── RViz2 ───────────────────────────────────────────────────────────────────
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", PathJoinSubstitution([bringup_share, "rviz", "localization.rviz"])],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        output="screen",
    )

    return LaunchDescription([
        map_path_arg,
        x_arg, y_arg, z_arg, ro_arg, pi_arg, ya_arg,
        use_rviz_arg,
        feature_extraction_node,
        imu_preintegration_node,
        laser_mapping_node,
        rviz_node,
    ])
