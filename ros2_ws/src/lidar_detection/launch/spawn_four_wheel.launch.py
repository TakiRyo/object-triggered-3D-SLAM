import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    package_name = 'lidar_detection'

    # === 1. Load and process the Xacro file ===
    urdf_file = os.path.join(
        get_package_share_directory(package_name),
        'urdf',
        'four_wheel_2d.urdf.xacro'
    )
    robot_description = Command(['xacro ', urdf_file])

    # === 2. Load your Gazebo world ===
    world_file = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'test_room.world'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'world': world_file}.items()
    )

    # === 3. Publish robot_description ===
    use_sim_time = LaunchConfiguration('use_sim_time')
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(robot_description, value_type=str)
        }]
    )

    # === 4. Spawn robot into Gazebo ===
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'four_wheel',
            '-topic', 'robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.1'
        ],
        output='screen'
    )

    # # === 5. Optional: RViz2 for visualization ===
    # rviz_config = os.path.join(
    #     get_package_share_directory(package_name),
    #     'config',
    #     'four_wheel_config.rviz'
    # )
    # rviz = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', rviz_config],
    #     output='screen'
    # )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        gazebo,
        rsp,
        spawn_entity,
        # rviz
    ])
