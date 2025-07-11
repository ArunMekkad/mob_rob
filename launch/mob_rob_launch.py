import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Define package names for easy reference
    pkg_mob_rob = FindPackageShare('mob_rob')

    # Get paths to configuration files and world
    world_file_path = PathJoinSubstitution([
        pkg_mob_rob, 'worlds', 'outdoor.world'
    ])
    slam_params_file_path = PathJoinSubstitution([
        pkg_mob_rob, 'config', 'mapper_params_online_async.yaml'
    ])
    nav2_params_file_path = PathJoinSubstitution([
        pkg_mob_rob, 'config', 'nav2_params.yaml'
    ])
    rviz_config_file_path = PathJoinSubstitution([
        pkg_mob_rob, 'config', 'mob_rob_drive.rviz'
    ])

    # Declare a common launch argument for use_sim_time
    # This ensures all included launch files use the same setting
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # 1. Launch Gazebo and spawn mob_rob
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_mob_rob, 'launch', 'spawn.launch.py'])
        ]),
        launch_arguments={
            'world': world_file_path,
            'use_sim_time': LaunchConfiguration('use_sim_time') # Pass sim time to spawn.launch.py
        }.items()
    )

    # 2. Launch SLAM Toolbox for Online Async SLAM
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node', 
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file_path,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # 3. Launch Nav2 Stack
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_mob_rob, 'launch', 'navigation_launch.py']) 
        ]),
        launch_arguments={
            'params_file': nav2_params_file_path,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    # 4. Launch RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file_path],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}] # Essential for RViz in sim
    )

    return LaunchDescription([
        use_sim_time_arg,
        gazebo_launch,
        slam_toolbox_node,
        nav2_bringup_launch,
        rviz_node,
    ])