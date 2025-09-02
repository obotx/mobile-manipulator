from dataclasses import dataclass
from typing import Tuple
import json
from pathlib import Path
import logging

@dataclass
class MobileBaseConfig:
    kv: float = 50.0  # Reduced for smoother control
    max_torque: float = 100.0  # Increased for better movement
    v_forward: float = 0.3  # Increased default speeds
    v_strafe: float = 0.3
    v_rotate: float = 1.0
    max_wheel_velocity: float = 5.0  # Increased max velocity
    auto_stop_timeout: float = 0.1  # Faster response
    velocity_decay_rate: float = 0.95  # Slower decay
    min_velocity_threshold: float = 0.01
    enable_smoothing: bool = True
    smoothing_factor: float = 0.3

@dataclass
class GripperConfig:
    control_gain: float = 500.0  # Increased for better position control
    damping: float = 50.0  # Increased damping for stability
    open_positions: dict = None
    closed_positions: dict = None
    trajectory_duration: float = 2.0  # Duration for smooth motion
    max_force: float = 1000.0  # Maximum force limit

@dataclass
class InterfaceConfig:
    window_size: Tuple[int, int] = (900, 700)  # Larger window
    telemetry_update_rate: float = 30.0  # Higher refresh rate
    log_level: str = 'INFO'
    log_file: str = 'robot_control.log'

def load_config(config_path: str = "robot_config.json"):
    """Load configuration from JSON file"""
    defaults = {
        'mobile': MobileBaseConfig().__dict__,
        'gripper': GripperConfig(
            open_positions={
                # Default open positions for common gripper joints
                'finger_a_joint_1': 0.0,
                'finger_a_joint_2': 0.0,
                'finger_a_joint_3': 0.0,
                'finger_b_joint_1': 0.0,
                'finger_b_joint_2': 0.0,
                'finger_b_joint_3': 0.0,
                'finger_c_joint_1': 0.0,
                'finger_c_joint_2': 0.0,
                'finger_c_joint_3': 0.0,
                'palm_finger_b_joint': 0.0,
                'palm_finger_c_joint': 0.0,
            },
            closed_positions={
                # Default closed positions - adjust these based on your gripper
                'finger_a_joint_1': 1.2,
                'finger_a_joint_2': 1.0,
                'finger_a_joint_3': 1.0,
                'finger_b_joint_1': 1.2,
                'finger_b_joint_2': 1.0,
                'finger_b_joint_3': 1.0,
                'finger_c_joint_1': 1.2,
                'finger_c_joint_2': 1.0,
                'finger_c_joint_3': 1.0,
                'palm_finger_b_joint': 0.0,
                'palm_finger_c_joint': 0.0,
            }
        ).__dict__,
        'interface': InterfaceConfig().__dict__
    }
    
    if Path(config_path).exists():
        try:
            with open(config_path, 'r') as f:
                loaded = json.load(f)
            # Merge with defaults
            for section in defaults:
                if section in loaded:
                    defaults[section].update(loaded[section])
            logging.info(f"Loaded configuration from {config_path}")
        except Exception as e:
            logging.warning(f"Error loading config: {e}, using defaults")
    else:
        logging.info("No config file found, using defaults")
    
    return defaults

def save_config(config: dict, config_path: str = "robot_config.json"):
    """Save configuration to JSON file"""
    try:
        with open(config_path, 'w') as f:
            json.dump(config, f, indent=2)
        logging.info(f"Configuration saved to {config_path}")
    except Exception as e:
        logging.error(f"Error saving config: {e}")

def create_default_config():
    """Create a default configuration file"""
    config = load_config()
    save_config(config)
    print("Default configuration file created: robot_config.json")
    print("You can edit this file to customize robot parameters")

if __name__ == "__main__":
    create_default_config()