from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # --- CONFIGURATION ---
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    db_path = '/home/ros2_env/taki/otslam/ros2_ws/src/rtab_3d_scan/database/rtabmap.db'

    # 1. RGBD Synchronization Node
    # This aligns the RGB image and Depth image timestamps
    sync_node = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        name='rgbd_sync',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'approx_sync': True,
            'queue_size': 20,
            'qos': 1,
            'qos_camera_info': 1,
        }],
        remappings=[
            # Input Topics (From your ros2 topic list)
            ('rgb/image',       '/intel_realsense_r200_rgb/image_raw'),
            ('rgb/camera_info', '/intel_realsense_r200_rgb/camera_info'),
            ('depth/image',     '/intel_realsense_r200_depth/depth/image_raw'),
            # Output Topic (To SLAM)
            ('rgbd_image',      '/rgbd_image_synced')
        ]
    )

    # 2. RTAB-Map SLAM Node
    # Configured for LiDAR + Wheel Odometry (No Visual Odometry)
    slam_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        arguments=['--delete_db_on_start'], # Resets map every run for clean baseline
        parameters=[{
            'use_sim_time': use_sim_time,
            'database_path': db_path,
            'frame_id': 'base_link',
            
            # Subscriptions
            'subscribe_depth': False, # We subscribe to rgbd_image instead
            'subscribe_rgbd': True,
            'subscribe_scan': True,   # ENABLE LIDAR
            'approx_sync': True,

            # --- KEY BASELINE STRATEGY ---
            # Reg/Strategy 1 = ICP (LiDAR correction). 
            # We do NOT use Visual Odometry (Strategy 0).
            'Reg/Strategy': '1', 
            
            # Force 3DoF (x, y, theta) since we are on a flat floor
            'Reg/Force3DoF': 'true',
            
            # Map Optimizations
            'RGBD/NeighborLinkRefining': 'true', 
            'RGBD/ProximityBySpace': 'true',
            
            # Memory / Map Management
            'Grid/FromDepth': 'false', # Use LiDAR for 2D Grid, not Depth Cam
            'Grid/RangeMax': '5.0',
        }],
        remappings=[
            ('rgbd_image', '/rgbd_image_synced'),
            ('scan',       '/scan'),
            ('odom',       '/odom'), # Wheel odometry
            ('imu',        '/imu')
        ]
    )

    # 3. Visualization Node
    viz_node = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'subscribe_rgbd': True,
            'subscribe_scan': True,
            'frame_id': 'base_link',
        }],
        remappings=[
            ('rgbd_image', '/rgbd_image_synced'),
            ('scan',       '/scan'),
            ('odom',       '/odom')
        ]
    )

    return LaunchDescription([
        sync_node,
        slam_node,
        viz_node
    ])