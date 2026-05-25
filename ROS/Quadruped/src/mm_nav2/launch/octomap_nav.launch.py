#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node as LaunchNode
from launch_ros.substitutions import FindPackageShare
import rclpy
from rclpy.node import Node
from rclpy.action import get_action_names_and_types
from launch.substitutions import LaunchConfiguration
import time

def wait_for_system_ready(context):
    use_moveit = LaunchConfiguration('use_moveit').perform(context) == 'true'
    rclpy.init()
    node = Node("nav2_waiter")
    node.get_logger().info(f"Waiting MoveIt (use_moveit={use_moveit})")
    while rclpy.ok():
        try:
            actions = get_action_names_and_types(node)
            moveit_ready = any("/move_action" in action_name for action_name, _ in actions)
            if use_moveit and moveit_ready:
                node.get_logger().info("MoveIt READY → launching Nav2")
                break
            if not use_moveit:
                break
        except Exception as e:
            node.get_logger().warn(f"Error checking actions: {e}")
        time.sleep(0.5)
    node.destroy_node()
    rclpy.shutdown()
    return []

def generate_launch_description():
    nav2_pkg = FindPackageShare('nav2_bringup')
    mm_nav2_pkg = FindPackageShare('mm_nav2')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_moveit = LaunchConfiguration('use_moveit', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')

    params_file = PathJoinSubstitution([
        mm_nav2_pkg,
        'params',
        'nav2_octomap_params.yaml'
    ])

    rviz_config = PathJoinSubstitution([
        nav2_pkg,
        'rviz',
        'nav2_default_view.rviz'
    ])

    # NAV2 NODES
    controller_server = LaunchNode(
        package='nav2_controller',
        executable='controller_server',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    planner_server = LaunchNode(
        package='nav2_planner',
        executable='planner_server',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    behavior_server = LaunchNode(
        package='nav2_behaviors',
        executable='behavior_server',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    bt_navigator = LaunchNode(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    waypoint_follower = LaunchNode(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    velocity_smoother = LaunchNode(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    lifecycle_manager = LaunchNode(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother',
            ]
        }],
        output='screen',
    )

    rviz = LaunchNode(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
        output='screen',
    )

    wait_gate = OpaqueFunction(function=wait_for_system_ready)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true'
        ),
        DeclareLaunchArgument(
            'use_moveit',
            default_value='true',
            description='Wait for MoveIt before starting Nav2'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true'
        ),
        LogInfo(msg="[INFO] Nav2 bringup starting..."),
        wait_gate,
        LogInfo(msg="[INFO] Starting Nav2 stack"),
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        lifecycle_manager,
        rviz,
    ])