from dataclasses import dataclass

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
    world_name: str = "market"  # "plain" or "market"