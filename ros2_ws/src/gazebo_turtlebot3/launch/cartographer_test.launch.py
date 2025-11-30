import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get the directory for your configurations (e.g., where you'd put the .lua file)
    pkg_dir = get_package_share_directory('turtlebot3_cartographer') 
    
    # Cartographer configuration file (You will create this LUA file)
    configuration_basename = LaunchConfiguration('configuration_basename', 
                                                 default='turtlebot3_lds_2d.lua')

    # Path to the .lua config file
    cartographer_config_dir = os.path.join(
        pkg_dir, 'config'
    )

    # Launch Cartographer node
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', configuration_basename,
            # Ensure the Lidar topic is correct for your simulation:
            '-load_state_filename', '' # Use this if you want to load a previous map state
        ],
        remappings=[
            ('echoes', 'scan'), # Standard Lidar topic remapping
            ('odom', 'odom'), # Standard odometry topic
        ]
    )

    # Launch map server to save the map after mapping is done
    # Note: Cartographer often requires its own map saving service (finish_trajectory)
    # This is a basic example for visualization/debugging.

    return LaunchDescription([
        # Declaration of launch arguments
        DeclareLaunchArgument(
            'configuration_basename',
            default_value='turtlebot3_lds_2d.lua',
            description='File name of Cartographer configuration lua file'
        ),
        
        # Start the Cartographer node
        cartographer_node,
    ])