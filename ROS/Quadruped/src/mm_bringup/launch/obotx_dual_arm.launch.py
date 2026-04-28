#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, DeclareLaunchArgument, 
    TimerAction, RegisterEventHandler
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_desc = get_package_share_directory('mm_description')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_ctrl = get_package_share_directory('mm_control')
    
    prefix = LaunchConfiguration('prefix', default='obotx_')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    xacro_file = PathJoinSubstitution([pkg_desc, 'urdf', 'obotx_dual_arm.urdf.xacro'])
    worlds_path = PathJoinSubstitution([pkg_desc, 'worlds', 'basic.sdf'])
    controller_config = PathJoinSubstitution([pkg_ctrl, 'config', 'obotx_dual_arm_controllers.yaml'])

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ', xacro_file,
            ' prefix:=', prefix,
            ' use_gazebo:=true',
            ' hq:=true',
            ' left_arm_mount_x:=0.15', ' left_arm_mount_y:=0.2', ' left_arm_mount_z:=0.195',
            ' left_arm_mount_r:=0.0', ' left_arm_mount_p:=0.0', ' left_arm_mount_yaw:=3.14159',
            ' right_arm_mount_x:=0.15', ' right_arm_mount_y:=-0.2', ' right_arm_mount_z:=0.195',
            ' right_arm_mount_r:=0.0', ' right_arm_mount_p:=0.0', ' right_arm_mount_yaw:=3.14159' 
        ]),
        value_type=str
    )

    # 1. Gazebo Harmonic
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([ros_gz_sim, 'launch', 'gz_sim.launch.py'])),
        launch_arguments=[
            ('gz_args', f'-r -v 1 empty.sdf'),
            ('use_sim_time', 'true')
        ]
    )

    # 2. Robot State Publisher
    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}]
    )

    # 3. Spawn Robot
    spawn_robot = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-name', 'obotx_dual_arm', '-topic', '/robot_description',
                   '-x', '0.0', '-y', '0.0', '-z', '0.0', '-Y', '0.0', '-allow_renaming', 'true'],
        output='screen'
    )

    # 4. Controller Spawner & RViz
    spawn_controllers = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', 'mecanum_drive_controller', 
                   'left_arm_controller', 'right_arm_controller',
                   '-c', '/controller_manager', '--param-file', controller_config],
        output='screen', parameters=[{'use_sim_time': use_sim_time}]
    )

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 🔹 SEQUENCE: 
    # Wait for 'create' to exit -> Delay 2.0s for Gazebo physics/init -> Start controllers + RViz
    start_controllers_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                TimerAction(
                    period=2.0,  # Gazebo needs ~1-2s to parse URDF & register gz_ros2_control interfaces
                    actions=[spawn_controllers, rviz]
                )
            ]
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('prefix', default_value='obotx_'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        
        # Clock bridge (can start immediately)
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
             output='screen'),
             
        gazebo,
        rsp,
        spawn_robot,
        start_controllers_after_spawn  # 👈 Triggers only AFTER spawn completes + delay
    ])