#!/usr/bin/env python3

# Nov/6 doesn't work. 

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Use simulation time
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Get launch file paths
    turtlebot3_gazebo_launch = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'launch',
        'turtlebot3_test.launch.py'
    )

    slam_toolbox_launch = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch',
        'online_async_launch.py'
    )

    lidar_detection_launch = os.path.join(
        get_package_share_directory('lidar_detection'),
        'launch',
        'lidar_navigation.launch.py'
    )

    nav2_launch = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'navigation_launch.py'
    )

    # Include launch files
    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(turtlebot3_gazebo_launch)
    )

    launch_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_toolbox_launch),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    launch_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_detection_launch)
    )

    launch_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Combine all
    return LaunchDescription([
        launch_gazebo,
        launch_slam,
        launch_lidar,
        launch_nav2,
    ])
