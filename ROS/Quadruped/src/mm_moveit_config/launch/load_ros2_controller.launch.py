#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    start_mecanum_drive_controller_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'mecanum_drive_controller'],
        output='screen'
    )
    start_left_arm_controller_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'left_arm_controller'],
        output='screen')
    
    start_right_arm_controller_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'right_arm_controller'],
        output='screen')
    
    start_gripper_left_controller_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'gripper_left_controller'],
        output='screen')
    
    start_gripper_right_controller_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'gripper_right_controller'],
        output='screen')

    start_joint_state_broadcaster_cmd = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen')

    delayed_start = TimerAction(
        period=10.0,
        actions=[start_joint_state_broadcaster_cmd]
    )

    load_mecanum_drive_controller_cmd = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_joint_state_broadcaster_cmd,
            on_exit=[start_mecanum_drive_controller_cmd]))

    load_left_arm_controller_cmd = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_mecanum_drive_controller_cmd,
            on_exit=[start_left_arm_controller_cmd]))
    
    load_right_arm_controller_cmd = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_left_arm_controller_cmd,
            on_exit=[start_right_arm_controller_cmd]))

    load_gripper_left_controller_cmd = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_right_arm_controller_cmd,
            on_exit=[start_gripper_left_controller_cmd]))
    
    load_gripper_right_controller_cmd = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_gripper_left_controller_cmd,
            on_exit=[start_gripper_right_controller_cmd]))

    ld = LaunchDescription()

    ld.add_action(delayed_start)
    ld.add_action(load_mecanum_drive_controller_cmd)
    ld.add_action(load_left_arm_controller_cmd)
    ld.add_action(load_right_arm_controller_cmd)
    ld.add_action(load_gripper_left_controller_cmd)
    ld.add_action(load_gripper_right_controller_cmd)

    return ld