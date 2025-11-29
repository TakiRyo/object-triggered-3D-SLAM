from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # Create the launch configuration variable
    use_sim_time = LaunchConfiguration('use_sim_time')

    # === Scanner Node (Camera / File Saver) ===
    scanner_node = Node(
        package='system_manager',
        executable='scanner_node', 
        name='scanner_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'wait_time': 5.0,  
        }]
    )

    # === System Manager Node (State Machine) ===
    manager_node = Node(
        package='system_manager',
        executable='manager_node',
        name='system_manager',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }]
    )

    return LaunchDescription([
        # Declare the argument to allow passing "use_sim_time:=true"
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        scanner_node,
        manager_node,
    ])