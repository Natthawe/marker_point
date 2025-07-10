import os
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import RegisterEventHandler, EmitEvent, IncludeLaunchDescription, DeclareLaunchArgument


def generate_launch_description():

    # rviz2
    rviz2_dir = get_package_share_directory('map_loader')
    launch_rviz2_pkg = os.path.join(rviz2_dir, 'launch')

    # map_loader
    map_loader_dir = get_package_share_directory('map_loader')
    launch_map_loader_pkg = os.path.join(map_loader_dir, 'launch')

    # marker_demo
    marker_demo_dir = get_package_share_directory('marker_demo')
    launch_marker_demo_pkg = os.path.join(marker_demo_dir, 'launch')

    return LaunchDescription([

        # rviz2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_rviz2_pkg, 'rviz2_launch.py'))
        ),

        # map_loader
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_map_loader_pkg, 'map_server_launch.py'))
        ),

        # marker_demo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_marker_demo_pkg, 'marker_publisher.launch.py'))
        ),

    ])
