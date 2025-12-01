from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # --- TF SECTION: The Bridge ---
    # The Robot has "camera_rgb_optical_frame".
    # The Plugin publishes "realsense_rgb_frame".
    # We connect them with a 0,0,0 (Identity) transform.
    tf_optical_bridge = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='optical_bridge',
        arguments = ["0", "0", "0", "1.57", "-1.57", "0", "camera_rgb_optical_frame", "realsense_rgb_frame"]
    )

    # --- RTAB-MAP SECTION ---
    sync_node = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        name='rgbd_sync',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'approx_sync': True,
            'queue_size': 20,
        }],
        remappings=[
            ('rgb/image',       '/intel_realsense_r200_rgb/image_raw'),
            ('rgb/camera_info', '/intel_realsense_r200_rgb/camera_info'),
            ('depth/image',     '/intel_realsense_r200_depth/depth/image_raw'),
            ('rgbd_image',      '/rgbd_image_synced')
        ]
    )

    slam_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        arguments=['--delete_db_on_start'], 
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_link',
            'subscribe_depth': False,
            'subscribe_rgbd': True,
            'subscribe_scan': True,
            'approx_sync': True,
            # LiDAR Strategy
            'Reg/Strategy': '1',       
            'Reg/Force3DoF': 'true',
            'RGBD/NeighborLinkRefining': 'true', 
            'Grid/RangeMax': '5.0',
        }],
        remappings=[
            ('rgbd_image', '/rgbd_image_synced'),
            ('scan',       '/scan'),
            ('odom',       '/odom'),
        ]
    )

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
        tf_optical_bridge, # <--- The critical fix
        sync_node,
        slam_node,
        viz_node
    ])