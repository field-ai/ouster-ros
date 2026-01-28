from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.substitution import Substitution
from typing import List
import os


def generate_launch_description():
    input_bag_dir_arg = DeclareLaunchArgument(
        'input_bag_dir',
        description='Path to input bag directory or file'
    )
    
    ouster_metadata_fp = DeclareLaunchArgument(
        'ouster_metadata_filepath',
        description='Path to Ouster metadata JSON file'
    )
    
    robot_namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        description='Robot name for topic namespacing'
    )
    
    timestamp_mode_arg = DeclareLaunchArgument(
        'timestamp_mode',
        default_value='TIME_FROM_PTP_1588',
        description='Timestamp mode for lidar packets'
    )
    
    class ExpandPath(Substitution):
        def __init__(self, path_sub):
            super().__init__()
            self.path_sub = path_sub
            
        def describe(self):
            return f'ExpandPath({self.path_sub.describe()})'
            
        def perform(self, context):
            path = self.path_sub.perform(context)
            return os.path.expanduser(path)
    
    converter_exe = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ouster_ros', 'offline_packet_converter',
            ExpandPath(LaunchConfiguration('input_bag_dir')),
            ExpandPath(LaunchConfiguration('ouster_metadata_filepath')),
            LaunchConfiguration('robot_namespace'),
            LaunchConfiguration('timestamp_mode')
        ],
        output='screen',
        shell=False
    )
    
    return LaunchDescription([
        input_bag_dir_arg,
        ouster_metadata_fp,
        robot_namespace_arg,
        timestamp_mode_arg,
        converter_exe
    ])