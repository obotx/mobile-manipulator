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
from launch.substitutions import LaunchConfiguration, PythonExpression
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

    # =========================
    # Target pose arguments
    # =========================
    declare_target_x_cmd = DeclareLaunchArgument(
        name='target_x',
        default_value='1.0',
        description='Target pose X')

    declare_target_y_cmd = DeclareLaunchArgument(
        name='target_y',
        default_value='1.6',
        description='Target pose Y')

    declare_target_z_cmd = DeclareLaunchArgument(
        name='target_z',
        default_value='0.8',
        description='Target pose Z')

    def launch_setup(context):

        robot_name = LaunchConfiguration('robot_name').perform(context)
        use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'
        use_rviz = LaunchConfiguration('use_rviz').perform(context) == 'true'
        rviz_config_file = LaunchConfiguration('rviz_config_file').perform(context)

        pkg_moveit_share = FindPackageShare(PKG_MOVEIT_CONFIG).find(PKG_MOVEIT_CONFIG)
        config_dir = os.path.join(pkg_moveit_share, 'config', robot_name)

        pkg_mm_share = FindPackageShare(PKG_MM_DESC).find(PKG_MM_DESC)

        urdf_path = os.path.join(
            pkg_mm_share,
            'urdf',
            'robot',
            f'{robot_name}.urdf.xacro'
        )

        moveit_config = (
            MoveItConfigsBuilder(robot_name, package_name=PKG_MOVEIT_CONFIG)
            .robot_description(file_path=urdf_path)
            .robot_description_semantic(
                file_path=os.path.join(config_dir, f'{robot_name}.srdf')
            )
            .joint_limits(
                file_path=os.path.join(config_dir, 'joint_limits.yaml')
            )
            .robot_description_kinematics(
                file_path=os.path.join(config_dir, 'kinematics.yaml')
            )
            .trajectory_execution(
                file_path=os.path.join(config_dir, 'moveit_controllers.yaml')
            )
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

        moveit_node = Node(
            package="mm_moveit_demos",
            executable="simple_pose_target",
            output="screen",
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,

                # =========================
                # Pose parameters
                # =========================
                {
                    "target_x": PythonExpression(["float(", LaunchConfiguration("target_x"), ")"]),
                    "target_y": PythonExpression(["float(", LaunchConfiguration("target_y"), ")"]),
                    "target_z": PythonExpression(["float(", LaunchConfiguration("target_z"), ")"]),
                    "use_sim_time": use_sim_time,
                }
            ],
        )

        return [
            moveit_node
        ]

    ld = LaunchDescription()

    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_rviz_config_file_cmd)

    # Add pose args
    ld.add_action(declare_target_x_cmd)
    ld.add_action(declare_target_y_cmd)
    ld.add_action(declare_target_z_cmd)

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld