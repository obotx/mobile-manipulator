import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get package paths
    pkg_delto_description = FindPackageShare("dg_description").find("dg_description")
    
    # For Gazebo Harmonic, we need to make sure the 'share' directory is in the resource path
    # so that package:// URIs work.
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[pkg_delto_description]
    )

    # Gazebo Harmonic Launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={
            # Use -v 4 to see debug info if it fails to load meshes
            "gz_args": " -r empty.sdf -v 4" 
        }.items(),
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("dg4f_gz"), "urdf", "dg4f_gz.xacro"]
            ),
        ]
    )
    
    robot_description = {"robot_description": robot_description_content}


    # Robot State Publisher
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # Spawn Entity in Gazebo Harmonic
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "/robot_description",
            "-name", "dg4f",
            "-allow_renaming", "true",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.5", # Lift it up slightly to avoid falling through floor if physics is weird
        ],
    )

    # Controllers
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("dg4f_gz"),
            "config",
            "dg4f_gz_controller.yaml",
        ]
    )
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            '--param-file',
            robot_controllers,
        ],
        output="screen",
    )

    nodes = [
        gz_resource_path, # Set the env var FIRST
        gazebo,
        node_robot_state_publisher,
        gz_spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[joint_trajectory_controller_spawner],
            )
        ),
    ]

    return LaunchDescription(nodes)