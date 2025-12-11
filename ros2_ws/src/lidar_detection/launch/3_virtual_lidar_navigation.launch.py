from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():

    # 1. Virtual Scan Node (Generates reference scan from map)
    virtual_scan_node = Node(
        package='virtual_scan', # Check your package name!
        executable='virtual_scan_node',
        name='virtual_scan_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 2. Change Detector (The Core Logic)
    # Subscribes to: /scan, /virtual_scan
    # Publishes: /added_obstacles (Green), /removed_obstacles (Purple)
    # ★ REMAPPING: We merge both outputs into '/object_clusters' for the selector
    change_detector = Node(
        package='lidar_detection',
        executable='diff_node',
        name='diff_node',
        output='screen',
        remappings=[
            ('/added_objects', '/object_clusters'),      # New objects -> Goal Selector
            # ('/removed_objects', '/object_clusters')   # Disappeared walls -> Goal Selector
        ],
        parameters=[{
            'distance_threshold': 0.5, # 50cm gap required
            'time_threshold': 2.0,     # Must persist for 2s
            'grid_resolution': 0.1,
            'decay_rate': 0.5
        }]
    )

    # 3. Object Goal Selector (Filters & Stabilizes Goals)
    # Input: /object_clusters (Now contains BOTH new and removed objects)
    multi_goal_selector = Node(
        package='lidar_detection',
        executable='multi_object_goal_selector',
        name='multi_goal_selector',
        output='screen',
        parameters=[{
            'cluster_distance_threshold': 4.0,
            'min_cluster_points': 1,      # Lowered slightly for grid points
            'wall_thickness_threshold': 0.3,
            'stability_time': 1.0,        # Reduced since detector already handles time
            'lock_margin': 1.5,
            'smoothing_factor': 1.0,
            'visiting_point_buffer': 0.1,
            'scan_step_threshold': 2.0,
            'points_count_normal': 6,
            'points_count_big': 8,
            'degree_visiting_points': 20.0,
            'scan_point_interval': 1.0,
            'scan_point_interval_removed': 5.0,
            "min_scan_points": 4,  
        }]
    )

    # 4. Goal Sender (Navigation)
    multi_goal_sender = Node(
        package='lidar_detection',
        executable='multi_send_goal_node',
        name='multi_goal_sender',
        output='screen',
        parameters=[{
            'reach_threshold': 0.60,
        }]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        
        # virtual_scan_node,
        change_detector,
        multi_goal_selector,
        multi_goal_sender,
    ])