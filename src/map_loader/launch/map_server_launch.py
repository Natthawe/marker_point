from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # path แบบ relative ใน package
    default_map_yaml_path = os.path.join(
        get_package_share_directory('map_loader'),
        'maps',
        'aisin',
        'origin',
        'demo.yaml'
    )

    # ใช้ LaunchConfiguration
    map_yaml_file = LaunchConfiguration('map')

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml_file}],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server']
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=default_map_yaml_path,
            description='Path to the map yaml file inside the package'
        ),
        map_server_node,
        TimerAction(
            period=2.0,
            actions=[lifecycle_manager]
        )
    ])
