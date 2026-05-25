#!/usr/bin/env python3

import os
import yaml
import rclpy
from launch import LaunchDescription
from geometry_msgs.msg import Pose
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

PKG_MOVEIT_CONFIG = 'mm_moveit_config'
PKG_MM_DESC = 'mm_description'


def wait_pose(topic):
    rclpy.init()
    node = rclpy.create_node(
        'wait_pose_node'
    )
    msg = None
    def cb(data):
        nonlocal msg
        msg = data
    sub = node.create_subscription(Pose, topic, cb, 10)
    while rclpy.ok() and msg is None:
        rclpy.spin_once(
            node,
            timeout_sec=0.1
        )
    node.destroy_node()
    rclpy.shutdown()
    return msg


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
    # Launch Arguments
    declare_robot_name_cmd = DeclareLaunchArgument(name='robot_name', default_value='morph_i')
    declare_use_sim_time_cmd = DeclareLaunchArgument(name='use_sim_time', default_value='true')
    declare_target_x_cmd = DeclareLaunchArgument(name='target_x', default_value='0.8')
    declare_target_y_cmd = DeclareLaunchArgument(name='target_y', default_value='0.3')
    declare_target_z_cmd = DeclareLaunchArgument(name='target_z', default_value='0.8')
    declare_target_arm_cmd = DeclareLaunchArgument(name='arm_side', default_value='auto', description='left or right')

    # Launch Setup
    def launch_setup(context):
        robot_name = LaunchConfiguration('robot_name').perform(context)
        use_sim_time = (LaunchConfiguration('use_sim_time').perform(context) == 'true')
        pkg_moveit_share = FindPackageShare(PKG_MOVEIT_CONFIG).find(PKG_MOVEIT_CONFIG)
        config_dir = os.path.join(pkg_moveit_share, 'config', robot_name)
        pkg_mm_share = FindPackageShare(PKG_MM_DESC).find(PKG_MM_DESC)
        urdf_path = os.path.join(pkg_mm_share,'urdf', 'robot', f'{robot_name}.urdf.xacro')

        # MoveIt Config
        moveit_config = (
            MoveItConfigsBuilder(robot_name,package_name=PKG_MOVEIT_CONFIG)
            .robot_description(file_path=urdf_path)
            .robot_description_semantic(file_path=os.path.join(config_dir, f'{robot_name}.srdf'))
            .joint_limits(file_path=os.path.join(config_dir, 'joint_limits.yaml'))
            .robot_description_kinematics(file_path=os.path.join(config_dir,'kinematics.yaml'))
            .trajectory_execution(file_path=os.path.join(config_dir, 'moveit_controllers.yaml'))
            .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
            .planning_scene_monitor(
                publish_robot_description=False,
                publish_robot_description_semantic=True,
                publish_planning_scene=True,
            )
            .to_moveit_configs()
        )

        # MoveIt Demo Node
        moveit_node = Node(
            package="mm_moveit_demos",
            executable="move_arm_pose",
            output="screen",
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                {
                    "use_sim_time": use_sim_time,
                    "target_x": PythonExpression(["float(", LaunchConfiguration("target_x"), ")"]),
                    "target_y": PythonExpression(["float(", LaunchConfiguration("target_y"), ")"]),
                    "target_z": PythonExpression(["float(", LaunchConfiguration("target_z"), ")"]),
                    "arm_side": LaunchConfiguration("arm_side"),
                }
            ],
        )
        return [moveit_node]

    # Launch Description
    ld = LaunchDescription()
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_target_x_cmd)
    ld.add_action(declare_target_y_cmd)
    ld.add_action(declare_target_z_cmd)
    ld.add_action(declare_target_arm_cmd)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld