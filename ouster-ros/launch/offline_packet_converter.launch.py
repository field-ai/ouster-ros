from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.substitution import Substitution
from typing import List
import os


def generate_launch_description():
    # Declare launch arguments
    input_bag_arg = DeclareLaunchArgument(
        'input_bag',
        description='Path to input bag directory or file'
    )
    
    output_bag_arg = DeclareLaunchArgument(
        'output_bag',
        description='Path to output bag directory'
    )
    
    metadata_file_arg = DeclareLaunchArgument(
        'metadata_file',
        description='Path to Ouster metadata JSON file'
    )
    
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        description='Robot name for topic namespacing'
    )
    
    timestamp_mode_arg = DeclareLaunchArgument(
        'timestamp_mode',
        default_value='TIME_FROM_INTERNAL_OSC',
        description='Timestamp mode for lidar packets'
    )
    
    # Custom substitution to expand ~ to home directory
    class ExpandPath(Substitution):
        def __init__(self, path_sub):
            super().__init__()
            self.path_sub = path_sub
            
        def describe(self):
            return f'ExpandPath({self.path_sub.describe()})'
            
        def perform(self, context):
            path = self.path_sub.perform(context)
            return os.path.expanduser(path)
    
    # Use ros2 run to find the executable automatically
    converter_exe = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ouster_ros', 'offline_packet_converter',
            ExpandPath(LaunchConfiguration('input_bag')),
            ExpandPath(LaunchConfiguration('output_bag')),
            ExpandPath(LaunchConfiguration('metadata_file')),
            LaunchConfiguration('robot_name'),
            LaunchConfiguration('timestamp_mode')
        ],
        output='screen',
        shell=False
    )
    
    return LaunchDescription([
        input_bag_arg,
        output_bag_arg,
        metadata_file_arg,
        robot_name_arg,
        timestamp_mode_arg,
        converter_exe
    ])