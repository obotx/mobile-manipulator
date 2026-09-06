from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Dict
import numpy as np

@dataclass
class MotionProfileConfig:
    profile_type: str = "trapezoidal"
    max_vel: float = 5.0
    max_accel: float = 15.0
    max_jerk: float = 80.0
    alpha: float = 0.1

@dataclass
class TaskConfig:
    """Configuration for trajectory-based tasks."""
    pick_position: tuple = (-0.33, -0.053, -0.01)  # [x, y, z]
    drop_position: tuple = (3.0, -6.7, -1.5708)     # [x, y, yaw]

@dataclass
class TeleopConfig:
    """Configuration for landmark-based teleoperation."""
    xml_path: Path = Path(__file__).parent / "xmls" / "combine" / "floor_morph_i.xml"
    
    robot_prefix: str = "robot"
    arm_prefixes: List[str] = field(default_factory=lambda: ["left", "right"])
    
    arm_motors: List[str] = field(default_factory=lambda: [
        "base_rotation", "primary_column_lift", "secondary_column_lift",
        "telescopic_extend", "wrist_pitch", "gripper_roll_joint",
        "gripper_pitch_joint", "gripper_yaw_joint"
    ])
    arm_motors_max_vel: Dict[str, float] = field(default_factory=lambda: {
        "base_rotation":          np.pi * 4,
        "primary_column_lift":    10.5,
        "secondary_column_lift":  10.5,
        "telescopic_extend":      10.5,
        "wrist_pitch":            np.pi,
        "gripper_roll_joint":     np.pi,
        "gripper_pitch_joint":    np.pi,
        "gripper_yaw_joint":      np.pi,
    })
    
    force_threshold_left: float = 15.0
    force_threshold_right: float = 15.0
    
    target_move_step: float = 0.005
    tracking_mode: str = "upper-only"
    max_iter: int = 20
    solver: str = "daqp"
    ik_damping: float = 1e-4
    control_freq: float = 200.0
    sim_steps_per_ctrl: int = 10
    
    human_x_min: List[float] = field(default_factory=lambda: [-0.40, -0.40])
    human_x_max: List[float] = field(default_factory=lambda: [ 0.40,  0.40])
    human_y_min: List[float] = field(default_factory=lambda: [-0.40, -0.40])
    human_y_max: List[float] = field(default_factory=lambda: [ 0.40,  0.40])
    human_z_min: List[float] = field(default_factory=lambda: [-0.275, -0.275])
    human_z_max: List[float] = field(default_factory=lambda: [ 0.275,  0.275])
    human_x_mid: List[float] = field(default_factory=lambda: [0.0, 0.0])
    human_y_mid: List[float] = field(default_factory=lambda: [0.0, 0.0])
    human_z_mid: List[float] = field(default_factory=lambda: [0.975, 0.975])
    
    robot_x_min: List[float] = field(default_factory=lambda: [-0.60, -0.60])
    robot_x_max: List[float] = field(default_factory=lambda: [ 0.60,  0.60])
    robot_y_min: List[float] = field(default_factory=lambda: [-0.60, -0.60])
    robot_y_max: List[float] = field(default_factory=lambda: [ 0.60,  0.60])
    robot_z_min: List[float] = field(default_factory=lambda: [-0.625, -0.625])
    robot_z_max: List[float] = field(default_factory=lambda: [ 0.625,  0.625])
    robot_x_mid: List[float] = field(default_factory=lambda: [0.15, 0.15])
    robot_y_mid: List[float] = field(default_factory=lambda: [0.0, 0.0])
    robot_z_mid: List[float] = field(default_factory=lambda: [0.675, 0.675])
    
    shoulder_to_y_mid: bool = True
    elbow_to_x_scale: bool = True
    arm_to_y_mid: bool = True
    
    robot_arm_left_name: str = "robot_arm_left"
    robot_arm_right_name: str = "robot_arm_right"
    
    smoothing_factor: float = 0.9
    smoothing_step: float = 0.05
    
    hand_wrist_max_dist: float = 0.30
    
    face_target_enabled: bool = True
    yaw_offset: float = 0.0
    
    seated_hip_height: float = 0.65
    
    viewer_width: int = 960
    viewer_height: int = 1000
    viewer_title: str = "MORPH I <-> FLOOR"
    
    max_points: int = 75
    
    def __post_init__(self):
        self.x_scale_neg = [self.robot_x_min[i] / self.human_x_min[i] if self.human_x_min[i] != 0 else 1.0 for i in range(2)]
        self.x_scale_pos = [self.robot_x_max[i] / self.human_x_max[i] if self.human_x_max[i] != 0 else 1.0 for i in range(2)]
        self.y_scale_neg = [self.robot_y_min[i] / self.human_y_min[i] if self.human_y_min[i] != 0 else 1.0 for i in range(2)]
        self.y_scale_pos = [self.robot_y_max[i] / self.human_y_max[i] if self.human_y_max[i] != 0 else 1.0 for i in range(2)]
        self.z_scale_neg = [self.robot_z_min[i] / self.human_z_min[i] if self.human_z_min[i] != 0 else 1.0 for i in range(2)]
        self.z_scale_pos = [self.robot_z_max[i] / self.human_z_max[i] if self.human_z_max[i] != 0 else 1.0 for i in range(2)]

@dataclass
class SimulationConfig:
    xml_path: str
    run_mode: str
    record: bool
    data_mode: str
    ws_url: str
    landmark_csv: str
    playback_rate: float
    target_mode: str
    control_mode: str                   # "landmark", "trajectory", "keyboard"
    motion_profile: MotionProfileConfig
    task_config: TaskConfig = None      # Only used when control_mode == "trajectory"
    world_name: str = "market"          # "plain" or "market"