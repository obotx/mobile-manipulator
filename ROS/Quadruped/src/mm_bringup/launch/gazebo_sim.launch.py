#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import LogInfo

def generate_launch_description():
    use_gazebo = LaunchConfiguration('use_gazebo')
    declare_use_gazebo = DeclareLaunchArgument(
        'use_gazebo',
        default_value='true',
        description='Launch Gazebo simulation. Set to false for RViz-only mode.'
    )

    pkg_desc = FindPackageShare('mm_description')
    pkg_ctrl = FindPackageShare('mm_control')
    pkg_gz_sim = FindPackageShare('ros_gz_sim')
    
    urdf_xacro = PathJoinSubstitution([pkg_desc, 'urdf', 'arm', 'arm.urdf.xacro'])
    controllers_yaml = PathJoinSubstitution([pkg_ctrl, 'config', 'arm_controllers.yaml'])
    worlds_path = PathJoinSubstitution([pkg_desc, 'worlds', 'basic.sdf'])
    LogInfo(msg=['World path resolves to: ', worlds_path])

    robot_description = ParameterValue(
        Command(['xacro ', urdf_xacro, ' use_gazebo:=', use_gazebo]),
        value_type=str
    )

    # Explicit boolean conversion for consistent parameter typing
    use_sim_time_val = PythonExpression(['"true" == "', use_gazebo, '"'])

    # ================= GAZEBO MODE =================
    gz_args = PythonExpression([
        '"-r -v 4 ', worlds_path, ' --physics-engine gz-physics-bullet-featherstone-plugin"'
    ])
    gz_nodes = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_gz_sim, 'launch', 'gz_sim.launch.py'])
            ),
            launch_arguments=[
                ('gz_args', gz_args),
                ('use_sim_time', 'true')
            ]
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            output='screen'
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-topic', 'robot_description', '-name', 'arm', '-z', '0.0'],
            output='screen'
        ),

        TimerAction(period=2.5, actions=[
            Node(package='controller_manager', executable='spawner',
                 arguments=['joint_state_broadcaster', '--param-file', controllers_yaml],
                 parameters=[{'use_sim_time': True}], output='screen'),
            Node(package='controller_manager', executable='spawner',
                 arguments=['arm_controller', '--param-file', controllers_yaml],
                 parameters=[{'use_sim_time': True}], output='screen'),
            Node(package='controller_manager', executable='spawner',
                 arguments=['right_side_controller', '--param-file', controllers_yaml],
                 parameters=[{'use_sim_time': True}], output='screen'),
        ])
    ], condition=IfCondition(use_gazebo))

    # ================= RVIZ-ONLY MODE =================
    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': False}
        ],
        condition=UnlessCondition(use_gazebo),
        output='screen'
    )

    return LaunchDescription([
        declare_use_gazebo,

        gz_nodes,
        jsp_node,

        # robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[
                {'robot_description': robot_description},
                {'use_sim_time': use_sim_time_val},
                {'publish_frequency': 30.0}
            ],
            output='screen'
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', PathJoinSubstitution([pkg_desc, 'rviz', 'default.rviz'])],
            parameters=[{'use_sim_time': use_sim_time_val}],
            output='screen'
        ),
    ])