"""Convert all Ouster .pcap.zst sessions in a dataset to pointcloud rosbags.

Usage:
    ros2 launch ouster_ros convert_pcap.launch.py \
        robot_namespace:=<robot> data_dir:=<dataset_dir>

Discovers every ouster/<robot>_ouster_<timestamp>/ session under data_dir,
decompresses its pcap files (recovering a truncated trailing file), and writes
one rosbag2/<robot>_lidar_pointcloud_<timestamp>/ bag per session.

Requires ouster_ros built with -DBUILD_PCAP=ON (the convert script reports a
clear error otherwise).
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    args = [
        DeclareLaunchArgument("data_dir", description="Dataset directory containing ouster/ captures"),
        DeclareLaunchArgument(
            "robot_namespace",
            default_value=os.environ.get("ROBOT_NAME", "robot1"),
            description="Robot name for topic namespacing and frame ids",
        ),
        # pcap_to_mcap pass-through params (defaults match the pcap_to_mcap C++ defaults).
        DeclareLaunchArgument("point_type", default_value="original"),
        DeclareLaunchArgument("organized", default_value="1"),
        DeclareLaunchArgument("destagger", default_value="1"),
        DeclareLaunchArgument("min_range", default_value="0.0"),
        DeclareLaunchArgument("max_range", default_value="1000.0"),
        DeclareLaunchArgument("v_reduction", default_value="1"),
        DeclareLaunchArgument("mask_path", default_value=""),
        DeclareLaunchArgument("timestamp_mode", default_value="TIME_FROM_PTP_1588"),
        DeclareLaunchArgument("ptp_utc_tai_offset", default_value="0"),
    ]

    script = Path(get_package_prefix("ouster_ros")) / "lib" / "ouster_ros" / "convert_pcap.py"

    convert = ExecuteProcess(
        cmd=[
            "python3", str(script),
            "--data-dir", LaunchConfiguration("data_dir"),
            "--robot-namespace", LaunchConfiguration("robot_namespace"),
            "--point-type", LaunchConfiguration("point_type"),
            "--organized", LaunchConfiguration("organized"),
            "--destagger", LaunchConfiguration("destagger"),
            "--min-range", LaunchConfiguration("min_range"),
            "--max-range", LaunchConfiguration("max_range"),
            "--v-reduction", LaunchConfiguration("v_reduction"),
            "--mask-path", LaunchConfiguration("mask_path"),
            "--timestamp-mode", LaunchConfiguration("timestamp_mode"),
            "--ptp-utc-tai-offset", LaunchConfiguration("ptp_utc_tai_offset"),
        ],
        output="screen",
    )

    # Shut the launch down as soon as the conversion process exits so `ros2 launch`
    # returns instead of hanging. NOTE: `ros2 launch` does not propagate a child's
    # non-zero exit code (it always returns 0); the failing process is logged as
    # "process has died ... exit code N". Programmatic/automated callers that need
    # a reliable exit status should invoke `ros2 run ouster_ros convert_pcap.py`
    # directly (as the odometry-rerun automation does).
    shutdown_on_exit = RegisterEventHandler(
        OnProcessExit(target_action=convert, on_exit=[EmitEvent(event=Shutdown())])
    )

    return LaunchDescription([*args, convert, shutdown_on_exit])
