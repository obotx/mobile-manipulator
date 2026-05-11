#!/usr/bin/env python3

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, OpaqueFunction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import time
import rclpy
from rclpy.node import Node as RclPyNode
from controller_manager_msgs.srv import ListControllers

PKG_MOVEIT_CONFIG = 'mm_moveit_config'
PKG_MM_DESC = 'mm_description'

REQUIRED_CONTROLLERS = [
    "gripper_right_controller",
    "gripper_left_controller",
    "right_arm_controller",
    "left_arm_controller",
    "mecanum_drive_controller",
    "joint_state_broadcaster",
]

def wait_for_active_controllers(context):
    rclpy.init()
    node = RclPyNode("controller_waiter")
    node.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
    client = node.create_client(
        ListControllers,
        "/controller_manager/list_controllers"
    )
    node.get_logger().debug(
        "Waiting for controller_manager service..."
    )

    if not client.wait_for_service(timeout_sec=60.0):
        raise RuntimeError(
            "/controller_manager/list_controllers not available"
        )
    
    logged_active = set()
    
    while rclpy.ok():
        future = client.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(node, future)
        active_set = {c.name for c in future.result().controller if c.state == "active"}
        newly_active = active_set - logged_active

        if newly_active:
            node.get_logger().debug(f"Newly active: {newly_active}")
            logged_active.update(active_set)

        if all(ctrl in active_set for ctrl in REQUIRED_CONTROLLERS):
            node.get_logger().debug("All required controllers are active. Proceeding to MoveGroup...")
            break

        time.sleep(1.0)

    node.destroy_node()
    rclpy.shutdown()
    return []

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
    declare_robot_name_cmd = DeclareLaunchArgument(
        name='robot_name',
        default_value='morph_i',
        description='Name of the robot (matches config directory under config/)')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='Whether to start RViz')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value='moveit.rviz',
        description='RViz configuration file name')

    def launch_setup(context):
        robot_name = LaunchConfiguration('robot_name').perform(context)
        use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'
        use_rviz = LaunchConfiguration('use_rviz').perform(context) == 'true'
        rviz_config_file = LaunchConfiguration('rviz_config_file').perform(context)

        pkg_moveit_share = FindPackageShare(PKG_MOVEIT_CONFIG).find(PKG_MOVEIT_CONFIG)
        config_dir = os.path.join(pkg_moveit_share, 'config', robot_name)

        pkg_mm_share = FindPackageShare(PKG_MM_DESC).find(PKG_MM_DESC)
        urdf_path = os.path.join(pkg_mm_share, 'urdf', 'robot', f'{robot_name}.urdf.xacro')
        
        moveit_config = (
            MoveItConfigsBuilder(robot_name, package_name=PKG_MOVEIT_CONFIG)
            .robot_description(file_path=urdf_path) 
            .robot_description_semantic(file_path=os.path.join(config_dir, f'{robot_name}.srdf'))
            .joint_limits(file_path=os.path.join(config_dir, 'joint_limits.yaml'))
            .robot_description_kinematics(file_path=os.path.join(config_dir, 'kinematics.yaml'))
            .trajectory_execution(file_path=os.path.join(config_dir, 'moveit_controllers.yaml'))
            .planning_pipelines(
                pipelines=["ompl"],
                default_planning_pipeline="ompl"
            )
            .planning_scene_monitor(
                publish_robot_description=False,
                publish_robot_description_semantic=True,
                publish_planning_scene=True,
            )
            .to_moveit_configs()
        )

        wait_for_active_controllers(context)
        
        world_to_odom_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom'],
            output='screen'
        )

        base_bridge_node = Node(
            package='mm_moveit_config',
            executable='base_cmd_vel_bridge.py',
            name='base_cmd_vel_bridge',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'cmd_vel_topic': '/mecanum_drive_controller/cmd_vel',
                'odom_topic': '/mecanum_drive_controller/odom',
                'frame_id': 'obotx_base_footprint_platform',
                'max_linear_vel': 1.0,
                'max_angular_vel': 0.8,
                'verbose': False,
            }]
        )

        odom_republisher_node = Node(
            package='mm_moveit_config',
            executable='repub_odometry_mdof_joint_states.py',
            name='base_state_republisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'odom_topic': '/mecanum_drive_controller/odom',
                'mdof_topic': '/multi_dof_joint_states',
                'joint_name': 'position'
            }]
        )

        move_group_capabilities = {"capabilities": "move_group/ExecuteTaskSolutionCapability"}
        start_move_group_cmd = Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=[
                moveit_config.to_dict(),
                {'use_sim_time': use_sim_time},
                {'initial_positions_file_path': os.path.join(config_dir, 'initial_positions.yaml')},
                # move_group_capabilities,
            ],
        )

        rviz_config_path = PathJoinSubstitution([
            pkg_moveit_share, 'config', robot_name, rviz_config_file
        ])

        start_rviz_cmd = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.planning_pipelines,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
                {'use_sim_time': use_sim_time},
            ],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        )

        rviz_exit_handler = RegisterEventHandler(
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            event_handler=OnProcessExit(
                target_action=start_rviz_cmd,
                on_exit=EmitEvent(event=Shutdown(reason='RViz exited')),
            ),
        )

        return [
            world_to_odom_tf,
            start_move_group_cmd, 
            start_rviz_cmd, 
            base_bridge_node,  
            odom_republisher_node, 
            rviz_exit_handler
        ]

    ld = LaunchDescription()
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld