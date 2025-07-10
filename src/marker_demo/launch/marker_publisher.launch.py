""" Launch file for the marker publisher node. """
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
import os
import yaml
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('marker_demo'),
        'config',
        'marker_config.yaml'
    )

    with open(config_file, 'r') as file:
        full_config = yaml.safe_load(file)
        params = full_config.get('publish_markers6', {}).get('ros__parameters', {})

    marker_node = Node(
        package='marker_demo',
        executable='publish_markers6',
        name='publish_markers6',
        output='screen',
        parameters=[params]
    )

    return LaunchDescription([
        TimerAction(
            period=4.0,
            actions=[marker_node]
        )
    ])
