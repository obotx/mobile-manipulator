#!/usr/bin/env python3
import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

ARGUMENTS = [
    DeclareLaunchArgument('prefix', default_value='obotx_', 
                            description='Prefix for robot joints and links'),
    DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false'],
                            description='Use simulation (Gazebo) clock if true'),
    DeclareLaunchArgument('use_gazebo', default_value='true', choices=['true', 'false'],
                            description='Use simulation (Gazebo) if true'),
    DeclareLaunchArgument('use_moveit', default_value='false', choices=['true', 'false'],
                            description='Use MoveIt virtual base joint ros2 control if true and use_gazebo is true'),
                          
    # Left arm mount arguments
    DeclareLaunchArgument('left_arm_mount_x', default_value='0.15', description='Left arm mount X'),
    DeclareLaunchArgument('left_arm_mount_y', default_value='0.15', description='Left arm mount Y'),
    DeclareLaunchArgument('left_arm_mount_z', default_value='0.158566', description='Left arm mount Z'),
    DeclareLaunchArgument('left_arm_mount_r', default_value='0.0', description='Left arm mount roll'),
    DeclareLaunchArgument('left_arm_mount_p', default_value='0.0', description='Left arm mount pitch'),
    DeclareLaunchArgument('left_arm_mount_yaw', default_value='3.14159', description='Left arm mount yaw'),
    
    # Right arm mount arguments
    DeclareLaunchArgument('right_arm_mount_x', default_value='0.15', description='Right arm mount X'),
    DeclareLaunchArgument('right_arm_mount_y', default_value='-0.15', description='Right arm mount Y'),
    DeclareLaunchArgument('right_arm_mount_z', default_value='0.158566', description='Right arm mount Z'),
    DeclareLaunchArgument('right_arm_mount_r', default_value='0.0', description='Right arm mount roll'),
    DeclareLaunchArgument('right_arm_mount_p', default_value='0.0', description='Right arm mount pitch'),
    DeclareLaunchArgument('right_arm_mount_yaw', default_value='3.14159', description='Right arm mount yaw'),
    
    # Visualization & JSP arguments (from reference)
    DeclareLaunchArgument('use_jsp', default_value='false', choices=['true', 'false'],
                          description='Enable the joint state publisher'),
    DeclareLaunchArgument('jsp_gui', default_value='true', choices=['true', 'false'],
                          description='Flag to enable joint_state_publisher_gui'),
    DeclareLaunchArgument('use_rviz', default_value='true', 
                          description='Whether to start RViz'),
]


def generate_launch_description():
    pkg_desc = FindPackageShare('mm_description')
    
    xacro_file = PathJoinSubstitution([pkg_desc, 'urdf', 'robot', 'morph_i.urdf.xacro'])
    rviz_config = PathJoinSubstitution([pkg_desc, 'rviz', 'morph_i.rviz'])
    
    prefix = LaunchConfiguration('prefix')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_jsp = LaunchConfiguration('use_jsp')
    jsp_gui = LaunchConfiguration('jsp_gui')
    use_rviz = LaunchConfiguration('use_rviz')
    
    robot_description = ParameterValue(Command([
        FindExecutable(name='xacro'), ' ', xacro_file,
        ' prefix:=', prefix,
        ' use_gazebo:=', LaunchConfiguration('use_gazebo'),
        ' use_moveit:=', LaunchConfiguration('use_moveit'),
        ' left_arm_mount_x:=', LaunchConfiguration('left_arm_mount_x'),
        ' left_arm_mount_y:=', LaunchConfiguration('left_arm_mount_y'),
        ' left_arm_mount_z:=', LaunchConfiguration('left_arm_mount_z'),
        ' left_arm_mount_r:=', LaunchConfiguration('left_arm_mount_r'),
        ' left_arm_mount_p:=', LaunchConfiguration('left_arm_mount_p'),
        ' left_arm_mount_yaw:=', LaunchConfiguration('left_arm_mount_yaw'),
        ' right_arm_mount_x:=', LaunchConfiguration('right_arm_mount_x'),
        ' right_arm_mount_y:=', LaunchConfiguration('right_arm_mount_y'),
        ' right_arm_mount_z:=', LaunchConfiguration('right_arm_mount_z'),
        ' right_arm_mount_r:=', LaunchConfiguration('right_arm_mount_r'),
        ' right_arm_mount_p:=', LaunchConfiguration('right_arm_mount_p'),
        ' right_arm_mount_yaw:=', LaunchConfiguration('right_arm_mount_yaw')
    ]), value_type=str)
    
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }]
    )
    
    start_joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_jsp)
    )
    
    start_joint_state_publisher_gui_cmd = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(jsp_gui)
    )
    
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz)
    )
    
    ld = LaunchDescription(ARGUMENTS)   
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_joint_state_publisher_cmd)
    ld.add_action(start_joint_state_publisher_gui_cmd)
    ld.add_action(start_rviz_cmd)
    
    return ld