from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    input_bag_dir_arg = DeclareLaunchArgument(
        'input_bag_dir',
        description='Path to input bag directory'
    )
    
    ouster_metadata_fp = DeclareLaunchArgument(
        'ouster_metadata_filepath',
        description='Path to Ouster metadata JSON file'
    )
    
    robot_namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        description='Robot name for topic namespacing'
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
        executable='offline_packet_converter',
        name='offline_packet_converter',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'input_bag_dir': LaunchConfiguration('input_bag_dir'),
                'ouster_metadata_filepath': LaunchConfiguration('ouster_metadata_filepath'),
                'robot_namespace': LaunchConfiguration('robot_namespace'),
            }
        ]
    )
    
    return LaunchDescription([
        input_bag_dir_arg,
        ouster_metadata_fp,
        robot_namespace_arg,
        params_file_arg,
        converter_node
    ])