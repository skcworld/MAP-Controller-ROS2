from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Launch arguments
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='True',
        description='Whether to start rviz'
    )
    
    map_name_arg = DeclareLaunchArgument(
        'map_name', default_value='f',
        description='Name of the map to use'
    )
    
    base_map_dir_arg = DeclareLaunchArgument(
        'base_map_dir', 
        default_value=PathJoinSubstitution([FindPackageShare('f1tenth_simulator')]),
        description='Directory containing maps'
    )
    
    # Get launch configurations
    rviz = LaunchConfiguration('rviz')
    map_name = LaunchConfiguration('map_name')
    base_map_dir = LaunchConfiguration('base_map_dir')
    
    # Define controllers and nodes
    controller_node = Node(
        package='map_controller',
        executable='map_controller',
        name='control_node',
        parameters=[
            os.path.join(get_package_share_directory('map_controller'), 'config', 'map_params.yaml')
        ],
        output='screen'
    )
    
    tracker_node = Node(
        package='map_controller',
        executable='time_error_tracker',
        name='tracker_node',
        output='screen'
    )
    
    # Return the launch description
    return LaunchDescription([
        rviz_arg,
        map_name_arg,
        base_map_dir_arg,
        controller_node,
        tracker_node,
    ])