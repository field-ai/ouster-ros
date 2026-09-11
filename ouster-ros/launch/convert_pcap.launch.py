"""Convert all Ouster .pcap.zst sessions in a dataset to pointcloud rosbags.

Usage:
    ros2 launch ouster_ros convert_pcap.launch.py \
        robot_namespace:=<robot> data_dir:=<dataset_dir>

Discovers every ouster/<robot>_ouster_<timestamp>/ session under data_dir,
decompresses its pcap files (recovering a truncated trailing file), and writes
one rosbag2/<robot>_lidar_pointcloud_<timestamp>/ bag per session.

Older datasets recorded the lidar packets into a rosbag2/<robot>_lidar_*/ bag of
ouster_sensor_msgs/msg/PacketMsg and have no ouster/ sessions. Convert those with
offline_packet_converter.launch.py instead; it takes the same arguments.

The pcap_to_mcap conversion parameters (point_type, organized, ranges, etc.) are
read from a single source of truth -- the ouster_ros config/fieldai_params.yaml --
rather than being duplicated here. Override with params_file:=<path>.

Requires ouster_ros built with -DBUILD_PCAP=ON (the convert script reports a clear
error otherwise).

NOTE: `ros2 launch` does not propagate a child's non-zero exit code (it always
returns 0); a failed conversion is logged as "process has died ... exit code N".
Programmatic/automated callers that need a reliable exit status should invoke
`ros2 run ouster_ros convert_pcap_to_rosbag2.py` directly (as the odometry-rerun
automation does).
"""

import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

_SCRIPT = Path(get_package_prefix("ouster_ros")) / "lib" / "ouster_ros" / "convert_pcap_to_rosbag2.py"

# pcap_to_mcap pass-through params, sourced from fieldai_params.yaml. Fallbacks
# match the pcap_to_mcap C++ defaults in case a key is absent from the yaml.
_PARAM_DEFAULTS = {
    "point_type": "original",
    "organized": True,
    "destagger": True,
    "min_range": 0.0,
    "max_range": 1000.0,
    "v_reduction": 1,
    "mask_path": "",
    "timestamp_mode": "TIME_FROM_PTP_1588",
    "ptp_utc_tai_offset": 0,
}


def _load_params(params_file):
    """Read the ros__parameters block from a fieldai_params.yaml-style file."""
    try:
        with open(params_file) as f:
            data = yaml.safe_load(f) or {}
    except (OSError, yaml.YAMLError):
        return {}
    # Support both the wildcard node block and a flat ros__parameters block.
    for node in ("/**", *data.keys()):
        block = data.get(node)
        if isinstance(block, dict) and "ros__parameters" in block:
            return block["ros__parameters"] or {}
    return data.get("ros__parameters", {}) or {}


def launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration("params_file").perform(context)
    data_dir = LaunchConfiguration("data_dir").perform(context)
    robot_namespace = LaunchConfiguration("robot_namespace").perform(context)

    params = _load_params(params_file)

    def p(key):
        return params.get(key, _PARAM_DEFAULTS[key])

    cmd = [
        "python3", str(_SCRIPT),
        "--data-dir", data_dir,
        "--robot-namespace", robot_namespace,
        "--point-type", str(p("point_type")),
        "--organized", str(p("organized")).lower(),
        "--destagger", str(p("destagger")).lower(),
        "--min-range", str(p("min_range")),
        "--max-range", str(p("max_range")),
        "--v-reduction", str(int(p("v_reduction"))),
        "--timestamp-mode", str(p("timestamp_mode")),
        # yaml stores the offset as a float (seconds); pcap_to_mcap wants an int.
        "--ptp-utc-tai-offset", str(int(float(p("ptp_utc_tai_offset")))),
    ]
    mask_path = str(p("mask_path") or "")
    if mask_path:
        cmd += ["--mask-path", mask_path]

    convert = ExecuteProcess(cmd=cmd, output="screen")

    # Shut the launch down as soon as the conversion process exits so `ros2 launch`
    # returns instead of hanging (see exit-code note in the module docstring).
    shutdown_on_exit = RegisterEventHandler(
        OnProcessExit(target_action=convert, on_exit=[EmitEvent(event=Shutdown())])
    )
    return [convert, shutdown_on_exit]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("data_dir", description="Dataset directory containing ouster/ captures"),
        DeclareLaunchArgument(
            "robot_namespace",
            default_value=os.environ.get("ROBOT_NAME", "robot1"),
            description="Robot name for topic namespacing and frame ids",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([FindPackageShare("ouster_ros"), "config", "fieldai_params.yaml"]),
            description="Params file supplying pcap_to_mcap conversion parameters",
        ),
        OpaqueFunction(function=launch_setup),
    ])
