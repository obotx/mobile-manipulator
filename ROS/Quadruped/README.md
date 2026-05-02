# ROS 2 Version: MORPH Series – Mobile Omni Robotic Platform with Hands

https://github.com/user-attachments/assets/9d1917a3-4d70-42b7-870c-debb7992be44

The **MORPH** series implements two distinct parallel manipulator configurations mounted on an omnidirectional mobile base. Both variants utilize chained mechanisms but differ fundamentally in their kinematic architecture.

> **Note:** This documentation covers the **ROS 2 Jazzy** implementation using **Gazebo Harmonic**.

## MORPH-I (Type I) : Dual Independent Parallel Manipulators
This configuration features two completely separate manipulators mounted side-by-side on the mobile base. There is no mechanical coupling between the left and right sides, allowing for independent operation.

**Kinematic Structure (Per Arm):**
- **Base Revolute Joint**: Rotates the entire column assembly
- **Vertical Prismatic Columns**: Left and right sliding columns
- **Horizontal Bar**: Connects the tops of both columns
- **End-Effector**: Mounted on the horizontal bar mechanism

---

## Prerequisites

Before proceeding, ensure you have the following installed:
1. **ROS 2 Jazzy Jalisco**
2. **Gazebo Harmonic**
3. Non-linear mimic gazebo plugin 

If you have not installed these yet, please refer to the official installation guides:
- [Install ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html)
- [Install Gazebo Harmonic](https://gazebosim.org/docs/harmonic/install_ubuntu/)
- [Install Non-linear Mimics Gazebo Plugin](https://github.com/drmwnrafi/gz_nonlinear_mimic)

---

## Quick Start Guide
### 1. Build the Workspace

Navigate to the root of the ROS Quadruped package directory and build the packages

```bash
# Navigate to the package directory
cd mobile-manipulator/ROS/Quadruped

# Build the packages with symlink install for faster development
colcon build --symlink-install

# Source the environment
source install/setup.bash

# Launch the main bringup file to start Gazebo and spawn the robot
ros2 launch mm_bringup obotx_dual_arm.launch.py
```

The visualization looks like this :
![launch_visualization](./assets/obotx_launch.png)
### 2. Control the Robot
Open a new terminal.

- Mobile Base Control
    Use the standard teleop twist keyboard to drive the omnidirectional base
    ```bash
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -r /cmd_vel:=/mecanum_drive_controller/cmd_vel
    ```
- Arm Control (Example)
    You can send joint trajectory goals directly via the command line to test arm movements. Below is an example sequence for the Right Arm.
    ```bash
    ros2 action send_goal /right_arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {
      joint_names: [
        obotx_right_arm_mount_joint,
        obotx_right_joint_slider_left_slide,
        obotx_right_joint_slider_left_hinge,
        obotx_right_joint_telescopic_slide,
        obotx_right_joint_hinge_telescopic_hand,
        obotx_right_finger_1_joint_1,
        obotx_right_finger_2_joint_1,
        obotx_right_finger_middle_joint_1
      ],
      points: [
        {
          positions: [0.0, 0.9, -0.2, 0.6, 0.8, 1.2, 1.2, 1.2],
          time_from_start: {sec: 3}
        },
        {
          positions: [0.5, 0.3, 0.2, 0.0, 0.0, 0.05, 0.05, 0.05],
          time_from_start: {sec: 6}
        },
        {
          positions: [-0.5, 0.7, 0.0, 0.6, -0.7, 1.2, 1.2, 1.2],
          time_from_start: {sec: 9}
        },
        {
          positions: [0.0, 0.2, 0.0, 0.0, 0.0, 0.05, 0.05, 0.05],
          time_from_start: {sec: 12}
        }
      ]
    }}"
    ```
    
## Joint Interface Reference

| Left Arm Joints | Right Arm Joints | Type | Description |
| :--- | :--- | :--- | :--- |
| `obotx_left_arm_mount_joint` | `obotx_right_arm_mount_joint` | Revolute | Base rotation of the arm  |
| `obotx_left_joint_slider_left_slide` | `obotx_right_joint_slider_left_slide` | Prismatic | Vertical slide (Left Column) |
| `obotx_left_joint_slider_left_hinge` | `obotx_right_joint_slider_left_hinge` | Prismatic | Vertical slide (Hinge/Column) |
| `obotx_left_joint_telescopic_slide` | `obotx_right_joint_telescopic_slide` | Prismatic | Telescopic extension |
| `obotx_left_joint_hinge_telescopic_hand` | `obotx_right_joint_hinge_telescopic_hand` | Revolute | Hand orientation hinge |
| `obotx_left_palm_wrist_roll_joint` | `obotx_right_palm_wrist_roll_joint` | Revolute | Wrist Roll |
| `obotx_left_palm_wrist_pitch_joint` | `obotx_right_palm_wrist_pitch_joint` | Revolute | Wrist Pitch |
| `obotx_left_palm_wrist_yaw_joint` | `obotx_right_palm_wrist_yaw_joint` | Revolute | Wrist Yaw |
| `obotx_left_palm_finger_1_joint` | `obotx_right_palm_finger_1_joint` | Revolute | Finger 1 Side Move Joint |
| `obotx_left_palm_finger_2_joint` | `obotx_right_palm_finger_2_joint` | Revolute | Finger 2 Side Move Joint |
| `obotx_left_finger_1_joint_1` | `obotx_right_finger_1_joint_1` | Revolute | Finger 1 Joint 1 |
| `obotx_left_finger_1_joint_2` | `obotx_right_finger_1_joint_2` | Revolute | Finger 1 Joint 2 |
| `obotx_left_finger_1_joint_3` | `obotx_right_finger_1_joint_3` | Revolute | Finger 1 Joint 3 |
| `obotx_left_finger_2_joint_1` | `obotx_right_finger_2_joint_1` | Revolute | Finger 2 Joint 1 |
| `obotx_left_finger_2_joint_2` | `obotx_right_finger_2_joint_2` | Revolute | Finger 2 Joint 2 |
| `obotx_left_finger_2_joint_3` | `obotx_right_finger_2_joint_3` | Revolute | Finger 2 Joint 3 |
| `obotx_left_finger_middle_joint_1` | `obotx_right_finger_middle_joint_1` | Revolute | Middle Finger Joint 1 |
| `obotx_left_finger_middle_joint_2` | `obotx_right_finger_middle_joint_2` | Revolute | Middle Finger Joint 2 |
| `obotx_left_finger_middle_joint_3` | `obotx_right_finger_middle_joint_3` | Revolute | Middle Finger Joint 3 |
