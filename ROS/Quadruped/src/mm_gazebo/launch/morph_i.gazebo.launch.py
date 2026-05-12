#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    default_robot_name  = 'MORPH_I'
    gazebo_models_path  = 'models'
    default_world_file  = 'basic.world'
    gazebo_worlds_path  = 'worlds'

    pkg_desc        = get_package_share_directory('mm_description')
    pkg_ros_gz_sim  = get_package_share_directory('ros_gz_sim')
    pkg_ctrl        = get_package_share_directory('mm_control')
    pkg_moveit      = get_package_share_directory('mm_moveit_config')
    pkg_gazebo      = get_package_share_directory('mm_gazebo')

    gazebo_models_path = os.path.join(pkg_gazebo, gazebo_models_path)
    bridge_config = PathJoinSubstitution([pkg_gazebo, 'config', 'ros_gz_bridge.yaml'])

    jsp_gui = LaunchConfiguration('jsp_gui')
    load_controllers = LaunchConfiguration('load_controllers')
    robot_name = LaunchConfiguration('robot_name')
    use_rviz = LaunchConfiguration('use_rviz')
    use_gazebo = LaunchConfiguration('use_gazebo')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_file = LaunchConfiguration('world_file')

    world_path = PathJoinSubstitution([
        pkg_gazebo,
        gazebo_worlds_path,
        world_file
    ])

    declare_robot_name_cmd = DeclareLaunchArgument(
        name='robot_name',
        default_value=default_robot_name,
        description='The name for the robot')

    declare_load_controllers_cmd = DeclareLaunchArgument(
        name='load_controllers',
        default_value='true',
        description='Flag to enable loading of ROS 2 controllers')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        name='use_robot_state_pub',
        default_value='true',
        description='Flag to enable robot state publisher')

    declare_jsp_gui_cmd = DeclareLaunchArgument(
        name='jsp_gui',
        default_value='false',
        description='Flag to enable joint_state_publisher_gui')

    declare_use_gazebo_cmd = DeclareLaunchArgument(
        name='use_gazebo',
        default_value='true',
        description='Flag to enable Gazebo')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='Flag to enable RViz')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_world_cmd = DeclareLaunchArgument(
        name='world_file',
        default_value=default_world_file,
        description='World file name (e.g., empty.world, house.world, pick_and_place_demo.world)')

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_desc, 'launch', 'robot_state_publisher.launch.py')
        ]),
        launch_arguments={
            'jsp_gui': jsp_gui,
            'use_gazebo': use_gazebo,
            'use_rviz': use_rviz,
            'use_sim_time': use_sim_time
        }.items(),
        condition=IfCondition(use_robot_state_pub)
    )

    load_controllers_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_moveit, 'launch', 'load_ros2_controller.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items(),
        condition=IfCondition(load_controllers)
    )

    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        gazebo_models_path)

    start_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments=[('gz_args', [' -r -v 1 ', world_path])])

    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_config,
        }],
        output='screen'
    )

    start_gazebo_ros_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', '/robot_description',
            '-name', robot_name,
            '-allow_renaming', 'true',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0'
        ])

    ld = LaunchDescription()

    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_jsp_gui_cmd)
    ld.add_action(declare_load_controllers_cmd)
    ld.add_action(declare_use_gazebo_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_cmd)

    ld.add_action(set_env_vars_resources)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(load_controllers_cmd)
    ld.add_action(start_gazebo_cmd)
    ld.add_action(start_gazebo_ros_bridge_cmd)
    ld.add_action(start_gazebo_ros_spawner_cmd)

    return ld