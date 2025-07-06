import os
from ament_index_python.packages  import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():

    # Replace with the package name
    package_name='mob_rob' 

    # Robot state publisher description 'ros2 launch mob_rob rsp.launch.py use_sim_time:=true'
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(package_name),
                         'launch','rsp.launch.py')]),
                         launch_arguments={'use_sim_time':'true','use_ros2_control':'true'}.items() # Toggle the use_ros2_control value here
    )
    
    #  Path to gazebo_params.yaml for adding as a launch argument 
    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml') 

    # Gazebo description 'ros2 launch gazebo_ros gazebo.launch.py'
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'),
                         'launch','gazebo.launch.py')]),
                         launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
    )

    # Spawner Node 'ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity mob_rob'

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'mob_rob'],
        output='screen')
    
    # Diff_cont node 'ros2 run controller_manager spawner diff_cont'

    diff_cont = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['diff_cont']
    )

    # Joint broad node 'ros2 run controller_manager spawner diff_cont'

    joint_broad = Node(
        package="controller_manager",
        executable="spawner",
        arguments=['joint_broad']
    )

    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        diff_cont,
        joint_broad
    ])