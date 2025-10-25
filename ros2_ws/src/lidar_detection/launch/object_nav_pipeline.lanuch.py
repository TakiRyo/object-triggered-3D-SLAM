from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # 1️⃣ Lidar cluster publisher (detects wall + object)
        Node(
            package='lidar_detection',
            executable='lidar_cluster_publisher',
            name='lidar_cluster_publisher',
            output='screen'
        ),

        # 2️⃣ Object goal selector (decides stable goal)
        Node(
            package='lidar_detection',
            executable='object_goal_selector',
            name='object_goal_selector',
            output='screen'
        ),

        # 3️⃣ Nav2 goal sender (sends confirmed goal to Nav2)
        Node(
            package='lidar_detection',
            executable='send_goal_node',
            name='send_goal_node',
            output='screen'
        ),
    ])
