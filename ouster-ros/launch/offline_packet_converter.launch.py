from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    input_bag_dir_arg = DeclareLaunchArgument(
        'data_dir',
        description='Path to input bag directory'
    )
    
    robot_namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        description='Robot name for topic namespacing',
        default_value=os.environ.get('ROBOT_NAME', 'robot1')
    )
    
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('ouster_ros'),
            'config',
            'fieldai_params.yaml'
        ]),
        description='Path to parameter YAML file'
    )
    
    converter_node = Node(
        package='ouster_ros',
        executable='offline_packet_converter_node',
        name='offline_packet_converter_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'data_dir': LaunchConfiguration('data_dir'),
                'robot_namespace': LaunchConfiguration('robot_namespace'),
            }
        ]
    )
    
    return LaunchDescription([
        input_bag_dir_arg,
        robot_namespace_arg,
        params_file_arg,
        converter_node
    ])