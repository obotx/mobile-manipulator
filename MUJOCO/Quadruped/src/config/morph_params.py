import numpy as np

BASE_NAME = "base_footprint"

EE_LEFT_NAME    = 'ee_site_left'
EE_RIGHT_NAME   = 'ee_site_right'

ACTUATOR_NAMES = [
    "ColumnLeftBearingJointMotor_1",
    "ColumnRightBearingJointMotor_1",
    "ArmLeftJointMotor_1",
    "act_BaseLeft",
    "HandBearing_1",
    "ColumnLeftBearingJointMotor_2",
    "ColumnRightBearingJointMotor_2",
    "ArmLeftJointMotor_2",
    "act_BaseRight",
    "HandBearing_2",
]

JOINT_NAMES_LEFT = [
    "ColumnLeftBearingJoint_1",
    "ColumnRightBearingJoint_1",
    "ArmLeftJoint_1",
    "BaseJoint_1",
    "HandBearingJoint_1"
    ]

JOINT_NAMES_RIGHT = [
    "ColumnLeftBearingJoint_2",
    "ColumnRightBearingJoint_2",
    "ArmLeftJoint_2",
    "BaseJoint_2",
    "HandBearingJoint_2"
]

ACTUATOR_NAMES_LEFT = [
    "act_ColumnLeft_1",
    "act_ColumnLeft_2",
    "act_TelescopicLeft",
    "act_BaseLeft",
    "HandBearing_1",
    ]

ACTUATOR_NAMES_RIGHT = [
    "act_ColumnRight_1",
    "act_ColumnRight_2",
    "act_TelescopicRight",
    "act_BaseRight",
    "HandBearing_2",
]

GRIPPER_JOINT_LEFT = [
    "finger_c_joint_1_1",
    "finger_c_joint_2_1",
    "finger_c_joint_3_1",
    "finger_b_joint_1_1",
    "finger_b_joint_2_1",
    "finger_b_joint_3_1",
    "finger_a_joint_1_1",
    "finger_a_joint_2_1",
    "finger_a_joint_3_1",
    "palm_finger_c_joint_1",
    "palm_finger_b_joint_1",
    "gripper_x_rotation_1",
    "gripper_y_rotation_1",
    "gripper_z_rotation_1",
]

GRIPPER_JOINT_RIGHT = [
    "finger_c_joint_1_2",
    "finger_c_joint_2_2",
    "finger_c_joint_3_2",
    "finger_b_joint_1_2",
    "finger_b_joint_2_2",
    "finger_b_joint_3_2",
    "finger_a_joint_1_2",
    "finger_a_joint_2_2",
    "finger_a_joint_3_2",
    "palm_finger_c_joint_2",
    "palm_finger_b_joint_2",
    "gripper_x_rotation_2",
    "gripper_y_rotation_2",
    "gripper_z_rotation_2",
]


GRIPPER_ACT_LEFT = [
    "finger_c_joint_1_1",
    "finger_c_joint_2_1",
    "finger_c_joint_3_1",
    "finger_b_joint_1_1",
    "finger_b_joint_2_1",
    "finger_b_joint_3_1",
    "finger_a_joint_1_1",
    "finger_a_joint_2_1",
    "finger_a_joint_3_1",
    "palm_finger_c_joint_1",
    "palm_finger_b_joint_1",
    "wrist_X_1",
    "wrist_Y_1",
    "wrist_Z_1",
]

GRIPPER_ACT_RIGHT = [
    "finger_c_joint_1_2",
    "finger_c_joint_2_2",
    "finger_c_joint_3_2",
    "finger_b_joint_1_2",
    "finger_b_joint_2_2",
    "finger_b_joint_3_2",
    "finger_a_joint_1_2",
    "finger_a_joint_2_2",
    "finger_a_joint_3_2",
    "palm_finger_c_joint_2",
    "palm_finger_b_joint_2",
    "wrist_X_2",
    "wrist_Y_2",
    "wrist_Z_2",
]

WHEEL_JOINT_NAMES = [
    "front_left_wheel_rolling_joint",   # FL
    "front_right_wheel_rolling_joint",  # FR
    "back_left_wheel_rolling_joint",    # RL (back = rear)
    "back_right_wheel_rolling_joint"    # RR
]

DT = 0.002
DAMPING = 8e-4
MOBILE_LX = 0.445  # front-rear distance (m)
MOBILE_LY = 0.409  # left-right distance (m)
R_WHEEL = 0.120  # from geom size
LX = 0.2225
LY = 0.2045
L_DIAGONAL = LX + LY  # 0.427

# Base joint PID
BASE_PID_KP = 10.0
BASE_PID_KI = 0.0
BASE_PID_KD = 7.0

# IK bounds
IK_BOUNDS_H = (0.0, 1.5)
IK_BOUNDS_A = (0.0, 0.7)
IK_ALPHA_MIN_DEG = 20.0
IK_L3_MAX = 0.7
IK_D2 = 0.1

# ARM OFFSET
ARM_L_OFF = np.array([0.15, 0.15, 0.158566])
ARM_L_OFF = np.array([0.15, -0.15, 0.158566])
