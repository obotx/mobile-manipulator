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
from launch.substitutions import LaunchConfiguration
from launch_param_builder import ParameterBuilder
from moveit_configs_utils import MoveItConfigsBuilder
import launch_ros
from launch.conditions import IfCondition, UnlessCondition

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
        default_value='moveit_servo.rviz',
        description='RViz configuration file name')
    
    launch_as_standalone_node = LaunchConfiguration(
        "launch_as_standalone_node", default="false"
    )

    def launch_setup(context):
        robot_name = LaunchConfiguration('robot_name').perform(context)
        use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'

        pkg_moveit_share = FindPackageShare(PKG_MOVEIT_CONFIG).find(PKG_MOVEIT_CONFIG)
        config_dir = os.path.join(pkg_moveit_share, 'config', robot_name)
        rviz_config_file = LaunchConfiguration('rviz_config_file').perform(context)
        rviz_config_path = PathJoinSubstitution([
            pkg_moveit_share, 'rviz', rviz_config_file
        ])

        servo_config_left = os.path.join(config_dir, 'servo_left.yaml')
        servo_config_right = os.path.join(config_dir, 'servo_right.yaml')
        pkg_mm_share = FindPackageShare(PKG_MM_DESC).find(PKG_MM_DESC)
        urdf_path = os.path.join(pkg_mm_share, 'urdf', 'robot', f'{robot_name}.urdf.xacro')
        pilz_cartesian_limits_file_path = os.path.join(config_dir, 'pilz_cartesian_limits.yaml')
        
        moveit_config = (
            MoveItConfigsBuilder(robot_name, package_name=PKG_MOVEIT_CONFIG)
            .robot_description(file_path=urdf_path) 
            .robot_description_semantic(file_path=os.path.join(config_dir, f'{robot_name}.srdf'))
            .joint_limits(file_path=os.path.join(config_dir, 'joint_limits.yaml'))
            .robot_description_kinematics(file_path=os.path.join(config_dir, 'kinematics.yaml'))
            .trajectory_execution(file_path=os.path.join(config_dir, 'moveit_controllers.yaml'))
            .planning_pipelines(
                pipelines=["ompl", "pilz_industrial_motion_planner", "stomp"],
                default_planning_pipeline="pilz_industrial_motion_planner"
            )
            .planning_scene_monitor(
                publish_robot_description=False,
                publish_robot_description_semantic=True,
                publish_planning_scene=True,
            )
            .pilz_cartesian_limits(file_path=pilz_cartesian_limits_file_path)
            .to_moveit_configs()
        )

        servo_param_left = {
            "moveit_servo": ParameterBuilder(PKG_MOVEIT_CONFIG)
            .yaml(f'config/{robot_name}/servo_left.yaml')
            .to_dict()
        }   

        servo_param_right = {
            "moveit_servo": ParameterBuilder(PKG_MOVEIT_CONFIG)
            .yaml(f'config/{robot_name}/servo_right.yaml')
            .to_dict()
        }

        acceleration_filter_update_period = {"update_period": 0.01}
        planning_group_name_left = {"planning_group_name": "left_arm"}
        planning_group_name_right = {"planning_group_name": "right_arm"}

        wait_for_active_controllers(context)


        container = launch_ros.actions.ComposableNodeContainer(
            name="moveit_servo_demo_container",
            namespace="/",
            package="rclcpp_components",
            executable="component_container_mt",
            composable_node_descriptions=[
                launch_ros.descriptions.ComposableNode(
                    package="tf2_ros",
                    plugin="tf2_ros::StaticTransformBroadcasterNode",
                    name="static_tf_broadcaster_odom",
                    parameters=[{
                        "x": 0.0, "y": 0.0, "z": 0.0,
                        "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
                        "frame_id": "world",
                        "child_frame_id": "odom"
                    }],
                ),
                launch_ros.descriptions.ComposableNode(
                    package="tf2_ros",
                    plugin="tf2_ros::StaticTransformBroadcasterNode",
                    name="static_tf_broadcaster_odom_gt",
                    parameters=[{
                        "x": 0.0, "y": 0.0, "z": 0.0,
                        "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
                        "frame_id": "world",
                        "child_frame_id": "odom_gt"
                    }],
                ),
                launch_ros.descriptions.ComposableNode(
                    package="moveit_servo",
                    plugin="moveit_servo::ServoNode",
                    name="servo_node_left",
                    parameters=[
                        servo_param_left,
                        acceleration_filter_update_period,
                        planning_group_name_left,
                        moveit_config.robot_description,
                        moveit_config.robot_description_semantic,
                        moveit_config.robot_description_kinematics,
                        moveit_config.joint_limits,
                        {'use_sim_time': use_sim_time}
                    ],
                    condition=UnlessCondition(launch_as_standalone_node),
                ),
                launch_ros.descriptions.ComposableNode(
                    package="moveit_servo",
                    plugin="moveit_servo::ServoNode",
                    name="servo_node_right",
                    parameters=[
                        servo_param_right,
                        acceleration_filter_update_period,
                        planning_group_name_right,
                        moveit_config.robot_description,
                        moveit_config.robot_description_semantic,
                        moveit_config.robot_description_kinematics,
                        moveit_config.joint_limits,
                        {'use_sim_time': use_sim_time}
                    ],
                    condition=UnlessCondition(launch_as_standalone_node),
                ),
                launch_ros.descriptions.ComposableNode(
                    package="robot_state_publisher",
                    plugin="robot_state_publisher::RobotStatePublisher",
                    name="robot_state_publisher",
                    parameters=[moveit_config.robot_description, {'use_sim_time': use_sim_time}],
                ),
            ],
            output="screen",
        )

        base_bridge_node = Node(
            package='mm_moveit_config',
            executable='base_cmd_vel_bridge.py',
            name='base_cmd_vel_bridge',
            output='screen',
            arguments=['--ros-args', '--log-level', 'stretch_kinematics_plugin:=debug'],
            parameters=[{
                'duration_scaling': 1,
                'sync_with_arms': False,
                'cmd_vel_topic': '/cmd_vel',
                'odom_topic': '/odom',
                'frame_id': 'obotx_base_footprint_platform',
                'max_linear_vel': 0.5,
                'max_angular_vel': 0.8,
                'verbose': True,
            }]
        )

        odom_republisher_node = Node(
            package='mm_moveit_config',
            executable='repub_odometry_mdof_joint_states.py',
            name='base_state_republisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'odom_topic': '/odom',
                'mdof_topic': '/multi_dof_joint_states',
                'joint_name': 'position'
            }]
        )

        move_group_cmd = Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=[
                moveit_config.to_dict(),
                {'use_sim_time': use_sim_time},
                {'initial_positions_file_path': os.path.join(config_dir, 'initial_positions.yaml')},
            ],
        )

        servo_node_left = Node(
            package='moveit_servo',
            executable='servo_node',
            name='servo_node_left',
            output='screen',
            parameters=[
                moveit_config.to_dict(),
                servo_config_left,
                acceleration_filter_update_period,
                planning_group_name_left,
                {'use_sim_time': use_sim_time}
            ],
            condition=IfCondition(launch_as_standalone_node),
        )

        servo_node_right = Node(
            package='moveit_servo',
            executable='servo_node',
            name='servo_node_right',
            output='screen',
            parameters=[
                moveit_config.to_dict(),
                servo_config_right,
                acceleration_filter_update_period,
                planning_group_name_right,
                {'use_sim_time': use_sim_time}
            ],
            condition=IfCondition(launch_as_standalone_node),
        )

        rviz_cmd = Node(
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
                target_action=rviz_cmd,
                on_exit=EmitEvent(event=Shutdown(reason='RViz exited')),
            ),
        )

        return [
            # move_group_cmd,
            container,
            servo_node_left, 
            servo_node_right, 
            rviz_cmd, 
            base_bridge_node,  
            odom_republisher_node, 
            rviz_exit_handler,
        ]

    ld = LaunchDescription()
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld