import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # CHANGE THIS: Path to your map file
    # Ideally, put map.yaml in a 'maps' folder inside your package
    map_file_path = '/home/ros2_env/taki/otslam/2d_map/map_check_nov30.yaml'

    return LaunchDescription([
        
        # 1. Start the Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_file_path}],
            remappings=[
                ('/map', '/virtual_map'),
                ('/map_metadata', '/virtual_map_metadata')
            ]
        ),

        # 2. Start the Lifecycle Manager (Crucial! This activates the map_server)
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{'use_sim_time': False},
                        {'autostart': True},
                        {'node_names': ['map_server']}]
        )
    ])