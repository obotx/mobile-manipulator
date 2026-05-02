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
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
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
    bridge_config = PathJoinSubstitution([pkg_desc, 'config', 'ros_gz_bridge.yaml'])

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
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ros_gz_sim, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={
            'gz_args': ['-r -v 1 ', worlds_path],
            'use_sim_time': 'true'
        }.items()
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[{'config_file': bridge_config}], 
        output='screen',
    )
    
    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}]
    )

    spawn_robot = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-name', 'obotx_dual_arm', '-topic', '/robot_description',
                   '-x', '0.0', '-y', '0.0', '-z', '0.0', '-Y', '0.0', '-allow_renaming', 'true'],
        output='screen'
    )

    spawn_controllers = Node(
        package='controller_manager', executable='spawner',
        arguments=[
                   'left_arm_controller', 'right_arm_controller',
                   '-c', '/controller_manager', '--param-file', controller_config],
        output='screen', parameters=[{'use_sim_time': use_sim_time}]
    )

    start_mecanum_drive_controller_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'mecanum_drive_controller'],
        output='screen'
    )

    start_joint_state_broadcaster_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    delayed_start = TimerAction(
        period=20.0,
        actions=[start_joint_state_broadcaster_cmd]
    )

    load_joint_state_broadcaster_cmd = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_joint_state_broadcaster_cmd,
            on_exit=[start_mecanum_drive_controller_cmd]))

    start_controllers_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=start_mecanum_drive_controller_cmd,
            on_exit=[spawn_controllers, rviz]
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('prefix', default_value='obotx_'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        ros_gz_bridge,
        gazebo,
        rsp,
        spawn_robot,
        delayed_start,
        load_joint_state_broadcaster_cmd,
        start_controllers_after_spawn 
    ])