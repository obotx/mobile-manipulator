#!/usr/bin/env python3

import os
import launch
import launch_ros
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    # Arguments
    robot_description_file = launch.substitutions.LaunchConfiguration(
        'robot_description_file', default='summit_xls.urdf.xacro'
    )

    robot_description_path = launch.substitutions.PathJoinSubstitution([
        FindPackageShare('summit_xl_description'),
        'robots',
        robot_description_file
    ])

    # Process xacro
    robot_description_content = launch.substitutions.Command([
        launch.substitutions.PathJoinSubstitution([
            launch.substitutions.FindExecutable(name="xacro")
        ]),
        ' ',
        robot_description_path,
        ' prefix:=summit_xl_ hq:=true ros_planar_move_plugin:=false gpu:=false'
    ])

    robot_description = {
        'robot_description': launch_ros.descriptions.ParameterValue(
            robot_description_content, value_type=str
        )
    }

    # Paths
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    controller_config = PathJoinSubstitution([
        FindPackageShare('summit_xl_control'),
        'summit_xl_controllers.yaml'
    ])

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return launch.LaunchDescription([

        # Declare arguments
        DeclareLaunchArgument('robot_description_file', default_value='summit_xls.urdf.xacro'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        # 🔥 Robot State Publisher
        launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                robot_description,
                {'use_sim_time': use_sim_time}
            ],
        ),

        # 🔥 Launch Gazebo Harmonic
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([ros_gz_sim_share, 'launch', 'gz_sim.launch.py'])
            ]),
            launch_arguments={'gz_args': '-r -v 4 empty.sdf'}.items()
        ),

        # 🔥 Spawn Robot in Gazebo
        launch_ros.actions.Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'summit_xl',
                '-topic', '/robot_description',
                '-x', '0.0', '-y', '0.0', '-z', '0.5',
                '-Y', '0.0',
                '-allow_renaming', 'true'
            ],
            output='screen'
        ),

        # 🔥 ROS 2 ↔ Gazebo Clock Bridge
        launch_ros.actions.Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            output='screen'
        ),

        # 🔥 Load Mecanum + Joint State Controllers
        launch_ros.actions.Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'joint_state_broadcaster',
                'mecanum_controller',
                '-c', '/controller_manager',
                '--param-file', controller_config
            ],
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # 🔥 RViz2
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    ])