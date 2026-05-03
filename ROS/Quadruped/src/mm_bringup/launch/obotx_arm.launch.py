#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PythonExpression, Command, LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    pkg_desc = get_package_share_directory('mm_description')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_ctrl = get_package_share_directory('mm_control')
    
    prefix = LaunchConfiguration('prefix', default='summit_')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    xacro_file = PathJoinSubstitution([pkg_desc, 'urdf', 'summit_xl_arm.urdf.xacro'])
    worlds_path = PathJoinSubstitution([pkg_desc, 'worlds', 'basic.sdf'])

    
    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ', xacro_file,
            ' prefix:=', prefix,
            ' use_gazebo:=true',
            ' hq:=true',
            ' arm_mount_x:=0.1',
            ' arm_mount_y:=0.0',
            ' arm_mount_z:=0.38062',
            ' arm_mount_r:=0.0',
            ' arm_mount_p:=0.0',
            ' arm_mount_yaw:=3.14159'
        ]),
        value_type=str
    )
    
    controller_config = PathJoinSubstitution([pkg_ctrl, 'config', 'summit_xl_arm_controllers.yaml'])
    gz_args = PythonExpression([
        '"-r -v 4 ', worlds_path, ' --physics-engine gz-physics-bullet-featherstone-plugin"'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('prefix', default_value='summit_'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        
        # Robot State Publisher
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', output='screen',
             parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}]),
        
        # Gazebo Harmonic
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([ros_gz_sim, 'launch', 'gz_sim.launch.py'])
            ),
            launch_arguments=[
                ('gz_args', gz_args),
                ('use_sim_time', 'true')
            ]
        ),
        
        # Spawn Robot
        Node(package='ros_gz_sim', executable='create',
             arguments=['-name', 'summit_xl_arm', '-topic', '/robot_description',
                        '-x', '0.0', '-y', '0.0', '-z', '0.0', '-Y', '0.0', '-allow_renaming', 'true'],
             output='screen'),

        
        # ROS 2 ↔ Gazebo Bridge
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                        '/mecanum_controller/reference@geometry_msgs/msg/Twist]gz.msgs.Twist'],
             output='screen'),
        
        # Controller Spawner
        Node(package='controller_manager', executable='spawner',
             arguments=['joint_state_broadcaster', 'mecanum_controller', 'arm_controller',
                        '-c', '/controller_manager', '--param-file', controller_config],
             output='screen', parameters=[{'use_sim_time': use_sim_time}]),
        
        # RViz
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             parameters=[{'use_sim_time': use_sim_time}]),
    ])