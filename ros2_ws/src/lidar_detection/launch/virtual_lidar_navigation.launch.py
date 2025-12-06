from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():

    # === LiDAR cluster publisher (detects wall + object) ===
    virtual_lidar_cluster = Node(
        package='lidar_detection',
        executable='virtual_lidar_cluster_publisher',
        name='virtual_lidar_cluster_publisher',
        output='screen',
        parameters=[{
            'gap_threshold': 0.2,
            'min_cluster_points': 1,
            'wall_length_threshold': 2.0,
            'wall_linearity_threshold': 0.001,
            'wall_min_points': 20,
            'object_length_threshold': 1.0,
            'object_max_points': 20,
            'max_range_ratio': 1.0,
            'obj_len_max': 1.0, #1.0
            'wal_len_min': 2.0, 
            'wal_lin_max': 0.001,
            'obj_nmp_min': 1,
            'wal_nmp_min': 150,

            'diff_threshold': 1.0
        }]
    )

    # === Object goal selector (filters + stabilizes clusters) ===
    multi_goal_selector = Node(
        package='lidar_detection',
        executable='multi_object_goal_selector',
        name='multi_goal_selector',
        output='screen',
        parameters=[{
            'cluster_distance_threshold': 4.0,
            'min_cluster_points': 8,
            'wall_thickness_threshold': 0.3,
            'stability_time': 3.0,
            'lock_margin': 1.5,
            'smoothing_factor': 1.0,
            'visiting_point_buffer': 0.1,
            'scan_step_threshold': 2.0,
            'points_count_normal': 6,
            'points_count_big': 8,
            'degree_visiting_points': 20.0,
            'scan_point_interval': 1.0, #distance between visiting points in meters
            "min_scan_points": 8,

        }]
    )


    # === Goal sender (publishes visiting points & Nav2 goals) ===
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

        virtual_lidar_cluster,
        multi_goal_selector,
        multi_goal_sender,
    ])
