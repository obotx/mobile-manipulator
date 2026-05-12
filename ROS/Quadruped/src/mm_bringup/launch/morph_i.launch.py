#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, DeclareLaunchArgument,
    RegisterEventHandler, ExecuteProcess
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_desc = get_package_share_directory('mm_description')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_ctrl = get_package_share_directory('mm_control')
    pkg_gazebo = get_package_share_directory('mm_gazebo')

    prefix = LaunchConfiguration('prefix', default='obotx_')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    xacro_file = PathJoinSubstitution([pkg_desc, 'urdf', 'robot', 'morph_i.urdf.xacro'])
    worlds_path = PathJoinSubstitution([pkg_gazebo, 'worlds', 'basic.world'])

    controller_config = PathJoinSubstitution([pkg_ctrl, 'config', 'controller_manager', 'morph_i.yaml'])
    rqt_config = PathJoinSubstitution([pkg_ctrl, 'config', 'rqt', 'morph_i.perspective'])
    rviz_config = PathJoinSubstitution([pkg_desc, 'rviz', 'morph_i.rviz'])
    bridge_config = PathJoinSubstitution([pkg_gazebo, 'config', 'ros_gz_bridge.yaml'])

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ', xacro_file,
            ' prefix:=', prefix,
            ' use_gazebo:=true',
            ' left_arm_mount_x:=0.15', ' left_arm_mount_y:=0.15', ' left_arm_mount_z:=0.158566',
            ' left_arm_mount_r:=0.0', ' left_arm_mount_p:=0.0', ' left_arm_mount_yaw:=3.14159',
            ' right_arm_mount_x:=0.15', ' right_arm_mount_y:=-0.15', ' right_arm_mount_z:=0.158566',
            ' right_arm_mount_r:=0.0', ' right_arm_mount_p:=0.0', ' right_arm_mount_yaw:=3.14159'
        ]),
        value_type=str
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ros_gz_sim, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={
            'gz_args': ['-r -v 4 ', worlds_path],
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
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}]
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'obotx_dual_arm',
            '-topic', '/robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.0', '-Y', '0.0',
            '-allow_renaming', 'true'
        ],
        output='screen'
    )

    # Controllers
    start_joint_state_broadcaster_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    start_mecanum_drive_controller_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'mecanum_drive_controller'],
        output='screen'
    )

    spawn_controllers = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'left_arm_controller',
            'right_arm_controller',
            'gripper_left_controller',
            'gripper_right_controller',
            '-c', '/controller_manager',
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    rqt_node = Node(
        package='rqt_gui',
        executable='rqt_gui',
        name='rqt_console',
        arguments=['--perspective-file', rqt_config],
        output='screen'
    )


    start_joint_state_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[start_joint_state_broadcaster_cmd]
        )
    )

    start_mecanum_after_js = RegisterEventHandler(
        OnProcessExit(
            target_action=start_joint_state_broadcaster_cmd,
            on_exit=[start_mecanum_drive_controller_cmd]
        )
    )

    start_controllers_after_mecanum = RegisterEventHandler(
        OnProcessExit(
            target_action=start_mecanum_drive_controller_cmd,
            on_exit=[spawn_controllers, rviz]
        )
    )

    start_rqt_last = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_controllers,
            on_exit=[rqt_node]
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('prefix', default_value='obotx_'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        ros_gz_bridge,
        gazebo,
        rsp,
        spawn_robot,

        start_joint_state_after_spawn,
        start_mecanum_after_js,
        start_controllers_after_mecanum,
        start_rqt_last,
    ])