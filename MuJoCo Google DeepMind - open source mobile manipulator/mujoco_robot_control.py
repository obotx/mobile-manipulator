#!/usr/bin/env python3
"""
Unified MuJoCo Robot and Gripper Control System
Features:
- Wheeled robot control (mecanum wheels)
- 3-finger gripper control
- Pygame-based keyboard interface
- Real-time telemetry display
- Configurable parameters
- Emergency stop functionality
- Smooth velocity transitions
"""

import mujoco
import mujoco.viewer
import numpy as np
import time
import threading
import sys
import json
import logging
import pygame
from dataclasses import dataclass
from typing import Dict, Tuple, Optional
from pathlib import Path
import signal

# --- Configuration ---
@dataclass
class UnifiedRobotConfig:
    """Unified robot configuration parameters"""
    # Model file
    model_file: str = 'arm_obotx_with_mobile_base.xml'
    
    # Mobile base control parameters
    mobile_kv: float = 200.0
    max_torque: float = 10.0
    
    # Mobile base velocity parameters
    v_forward: float = 0.1
    v_strafe: float = 0.1
    v_rotate: float = 0.5
    max_wheel_velocity: float = 1.0 * 2 * np.pi / 60.0  # 1 RPM in rad/s
    
    # Gripper control parameters
    gripper_control_gain: float = 100.0
    gripper_damping: float = 10.0
    
    # Auto-stop parameters
    auto_stop_timeout: float = 0.3
    velocity_decay_rate: float = 0.85
    min_velocity_threshold: float = 0.005
    
    # Smoothing parameters
    enable_smoothing: bool = True
    smoothing_factor: float = 0.2
    
    # Display parameters
    pygame_window_size: Tuple[int, int] = (800, 600)
    telemetry_update_rate: float = 10.0
    
    # Logging
    log_level: str = 'INFO'
    log_file: str = 'unified_robot_control.log'

class UnifiedRobotController:
    def __init__(self, config: UnifiedRobotConfig):
        self.config = config
        self.setup_logging()

        # Gripper state
        self.gripper_state = 'open'
        self.gripper_target_positions = {}
        
        # Control mode
        self.control_mode = 'mobile'  # 'mobile' or 'gripper'
        
        # Speed multiplier
        self.speed_multiplier = 1.0
        
        # Statistics
        self.stats = {
            'total_commands': 0,
            'gripper_commands': 0,
            'mobile_commands': 0,
            'start_time': time.time(),
            'last_wheel_velocities': np.zeros(4),
            'max_velocity_reached': 0.0
        }
        
        # Threading
        self.lock = threading.Lock()
        
        # Load MuJoCo model
        try:
            self.model = mujoco.MjModel.from_xml_path(config.model_file)
            self.data = mujoco.MjData(self.model)
            self.logger.info(f"Loaded model: {config.model_file}")
        except Exception as e:
            self.logger.error(f"Failed to load model {config.model_file}: {e}")
            raise
        


        self.define_gripper_states()
        self.set_gripper_state('open')

        # Initialize pygame
        pygame.init()
        self.screen = pygame.display.set_mode(config.pygame_window_size)
        pygame.display.set_caption("Unified Robot & Gripper Control")
        self.font = pygame.font.Font(None, 24)
        self.small_font = pygame.font.Font(None, 18)
        
        # Robot state
        self.velocity = np.array([0.0, 0.0, 0.0])  # [vx, vy, omega]
        self.target_velocity = np.array([0.0, 0.0, 0.0])
        self.last_command_time = time.time()
        self.manual_stop_flag = False
        self.emergency_stop = False
        self.running = True
        
        # Wheel configuration (from your original code)
        self.wheel_qvel_ids = {"FL": 19, "FR": 6, "BL": 45, "BR": 32}
        self.actuator_ids = {"FR": 0, "FL": 1, "BR": 2, "BL": 3}
        
        # Gripper configuration
        self.gripper_joints = [
            'gripper_z_rotation', 'gripper_y_rotation', 'gripper_x_rotation',
            'finger_a_joint_1', 'finger_a_joint_2', 'finger_a_joint_3',
            'palm_finger_b_joint', 'finger_b_joint_1', 'finger_b_joint_2', 'finger_b_joint_3',
            'palm_finger_c_joint', 'finger_c_joint_1', 'finger_c_joint_2', 'finger_c_joint_3'
        ]
        
        # Get gripper joint indices
        self.gripper_joint_indices = {}
        for joint_name in self.gripper_joints:
            try:
                self.gripper_joint_indices[joint_name] = mujoco.mj_name2id(
                    self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name
                )
            except:
                self.logger.warning(f"Gripper joint '{joint_name}' not found in model")
        
        
        # Setup signal handlers
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        self.logger.info("Unified robot controller initialized")
    
    def setup_logging(self):
        """Setup logging configuration"""
        logging.basicConfig(
            level=getattr(logging, self.config.log_level),
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler(self.config.log_file),
                logging.StreamHandler()
            ]
        )
        self.logger = logging.getLogger('UnifiedRobotController')
    
    def signal_handler(self, signum, frame):
        """Handle shutdown signals gracefully"""
        self.logger.info(f"Received signal {signum}, shutting down...")
        self.emergency_stop = True
        self.running = False
    
    def define_gripper_states(self):
        """Define open and closed positions for all gripper joints"""
        self.gripper_states = {
            'open': {
                'gripper_z_rotation': 0.0,
                'gripper_y_rotation': 0.0,
                'gripper_x_rotation': 0.0,
                'finger_a_joint_1': 0.1,
                'finger_a_joint_2': 0.1,
                'finger_a_joint_3': -0.1,
                'palm_finger_b_joint': 0.1,
                'finger_b_joint_1': 0.1,
                'finger_b_joint_2': 0.1,
                'finger_b_joint_3': -0.1,
                'palm_finger_c_joint': -0.1,
                'finger_c_joint_1': 0.1,
                'finger_c_joint_2': 0.1,
                'finger_c_joint_3': -0.1,
            },
            'closed': {
                'gripper_z_rotation': 0.0,
                'gripper_y_rotation': 0.0,
                'gripper_x_rotation': 0.0,
                'finger_a_joint_1': 5.0,
                'finger_a_joint_2': 1.2,
                'finger_a_joint_3': -1.0,
                'palm_finger_b_joint': 0.0,
                'finger_b_joint_1': 5.0,
                'finger_b_joint_2': 1.2,
                'finger_b_joint_3': -1.0,
                'palm_finger_c_joint': 0.0,
                'finger_c_joint_1': 5.0,
                'finger_c_joint_2': 1.2,
                'finger_c_joint_3': -1.0,
            }
        }
    
    def set_gripper_state(self, state: str):
        """Set target gripper state"""
        if state in self.gripper_states:
            with self.lock:
                self.gripper_state = state
                self.gripper_target_positions = self.gripper_states[state].copy()
                self.stats['gripper_commands'] += 1
                self.stats['total_commands'] += 1
            self.logger.info(f"Gripper state set to: {state}")
        else:
            self.logger.warning(f"Unknown gripper state: {state}")
    
    def apply_gripper_control(self):
        """Apply PD control to gripper joints"""
        for joint_name, target_pos in self.gripper_target_positions.items():
            if joint_name in self.gripper_joint_indices:
                joint_id = self.gripper_joint_indices[joint_name]
                
                # Current position and velocity
                current_pos = self.data.qpos[joint_id]
                current_vel = self.data.qvel[joint_id]
                
                # PD control
                position_error = target_pos - current_pos
                velocity_error = 0.0 - current_vel
                
                # Control force
                control_force = (self.config.gripper_control_gain * position_error - 
                               self.config.gripper_damping * velocity_error)
                
                # Apply control (find actuator for this joint)
                for i in range(self.model.nu):
                    if self.model.actuator_trnid[i, 0] == joint_id:
                        self.data.ctrl[i] = control_force
                        break
    
    def set_mobile_velocity_command(self, vx: float, vy: float, omega: float, command_name: str):
        """Set target velocity for mobile base"""
        with self.lock:
            self.target_velocity = np.array([vx, vy, omega])
            self.last_command_time = time.time()
            self.manual_stop_flag = False
            self.stats['mobile_commands'] += 1
            self.stats['total_commands'] += 1
            
            # Track maximum velocity
            max_vel = np.max(np.abs(self.target_velocity))
            if max_vel > self.stats['max_velocity_reached']:
                self.stats['max_velocity_reached'] = max_vel
        
        self.logger.debug(f"Mobile {command_name}: vx={vx:.3f}, vy={vy:.3f}, omega={omega:.3f}")
    
    def emergency_stop_robot(self):
        """Emergency stop - immediately halt all motion"""
        with self.lock:
            self.velocity = np.zeros(3)
            self.target_velocity = np.zeros(3)
            self.emergency_stop = True
            self.manual_stop_flag = True
            self.last_command_time = time.time()
        
        self.logger.warning("EMERGENCY STOP ACTIVATED")
    
    def manual_stop(self):
        """Manual stop for mobile base"""
        with self.lock:
            self.target_velocity = np.zeros(3)
            self.manual_stop_flag = True
            self.last_command_time = time.time()
        
        self.logger.info("Manual stop executed")
    
    def get_wheel_velocities(self) -> np.ndarray:
        """Get current wheel velocities from simulation"""
        return np.array([
            self.data.qvel[self.wheel_qvel_ids["FL"]],
            self.data.qvel[self.wheel_qvel_ids["FR"]],
            self.data.qvel[self.wheel_qvel_ids["BL"]],
            self.data.qvel[self.wheel_qvel_ids["BR"]]
        ])
    
    def mecanum_inverse_kinematics(self, vx: float, vy: float, omega: float) -> np.ndarray:
        """Convert body velocities to wheel velocities"""
        return np.array([
            vx - vy - omega,  # FL
            vx + vy + omega,  # FR
            vx + vy - omega,  # BL
            vx - vy + omega   # BR
        ])
    
    def cap_wheel_velocities(self, target_velocities: np.ndarray) -> np.ndarray:
        """Cap wheel velocities while preserving ratios"""
        max_target = np.max(np.abs(target_velocities))
        if max_target > self.config.max_wheel_velocity:
            scale_factor = self.config.max_wheel_velocity / max_target
            return target_velocities * scale_factor
        return target_velocities
    
    def smooth_velocity_transition(self, target: np.ndarray) -> np.ndarray:
        """Apply velocity smoothing to reduce jerky movements"""
        if not self.config.enable_smoothing:
            return target
            
        with self.lock:
            self.velocity = (self.velocity * (1 - self.config.smoothing_factor) + 
                           target * self.config.smoothing_factor)
            return self.velocity.copy()
    
    def update_auto_stop(self):
        """Handle auto-stop logic for mobile base"""
        if self.emergency_stop:
            return
            
        current_time = time.time()
        time_since_last_command = current_time - self.last_command_time
        
        if not self.manual_stop_flag and time_since_last_command > self.config.auto_stop_timeout:
            with self.lock:
                self.target_velocity *= self.config.velocity_decay_rate
                
                if np.all(np.abs(self.target_velocity) < self.config.min_velocity_threshold):
                    self.target_velocity = np.zeros(3)
    
    def apply_mobile_control(self):
        """Apply control to mobile base"""
        if self.emergency_stop:
            return
            
        # Get current wheel velocities
        wheel_velocities = self.get_wheel_velocities()
        
        # Apply velocity smoothing
        current_velocity = self.smooth_velocity_transition(self.target_velocity)
        
        # Mecanum inverse kinematics
        target_wheel_vels = self.mecanum_inverse_kinematics(
            current_velocity[0], current_velocity[1], current_velocity[2]
        )
        
        # Cap velocities
        target_wheel_vels = self.cap_wheel_velocities(target_wheel_vels)
        
        # PD control
        velocity_error = target_wheel_vels - wheel_velocities
        command = velocity_error * self.config.mobile_kv
        
        # Apply torque limits
        command = np.clip(command, -self.config.max_torque, self.config.max_torque)
        
        # Apply commands to actuators
        self.data.ctrl[self.actuator_ids["FL"]] = command[0]
        self.data.ctrl[self.actuator_ids["FR"]] = command[1]
        self.data.ctrl[self.actuator_ids["BL"]] = command[2]
        self.data.ctrl[self.actuator_ids["BR"]] = command[3]
        
        # Update statistics
        self.stats['last_wheel_velocities'] = wheel_velocities
    
    def handle_keyboard_events(self):
        """Handle pygame keyboard events"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYDOWN:
                self.handle_keydown(event.key)
    
    def handle_keydown(self, key):
        """Handle individual key press events"""
        # Mode switching
        if key == pygame.K_TAB:
            self.control_mode = 'gripper' if self.control_mode == 'mobile' else 'mobile'
            self.logger.info(f"Switched to {self.control_mode} control mode")
        
        # Emergency controls
        elif key == pygame.K_ESCAPE:
            self.emergency_stop_robot()
        elif key == pygame.K_SPACE:
            self.manual_stop()
        
        # Speed controls
        elif key == pygame.K_PLUS or key == pygame.K_EQUALS:
            self.speed_multiplier = min(2.0, self.speed_multiplier + 0.1)
        elif key == pygame.K_MINUS:
            self.speed_multiplier = max(0.1, self.speed_multiplier - 0.1)
        
        # Control mode specific commands
        elif self.control_mode == 'mobile':
            self.handle_mobile_controls(key)
        elif self.control_mode == 'gripper':
            self.handle_gripper_controls(key)
    
    def handle_mobile_controls(self, key):
        """Handle mobile base control keys"""
        # Calculate current speeds
        v_fwd = self.config.v_forward * self.speed_multiplier
        v_str = self.config.v_strafe * self.speed_multiplier
        v_rot = self.config.v_rotate * self.speed_multiplier
        
        # Basic movement
        if key == pygame.K_w:
            self.set_mobile_velocity_command(v_fwd, 0.0, 0.0, "Forward")
        elif key == pygame.K_s:
            self.set_mobile_velocity_command(-v_fwd, 0.0, 0.0, "Backward")
        elif key == pygame.K_a:
            self.set_mobile_velocity_command(0.0, v_str, 0.0, "Strafe Left")
        elif key == pygame.K_d:
            self.set_mobile_velocity_command(0.0, -v_str, 0.0, "Strafe Right")
        elif key == pygame.K_q:
            self.set_mobile_velocity_command(0.0, 0.0, v_rot, "Rotate Left")
        elif key == pygame.K_e:
            self.set_mobile_velocity_command(0.0, 0.0, -v_rot, "Rotate Right")
        
        # Diagonal movements
        elif key == pygame.K_r:
            self.set_mobile_velocity_command(v_fwd, -v_str, 0.0, "Forward-Right")
        elif key == pygame.K_t:
            self.set_mobile_velocity_command(v_fwd, v_str, 0.0, "Forward-Left")
        elif key == pygame.K_f:
            self.set_mobile_velocity_command(-v_fwd, -v_str, 0.0, "Backward-Right")
        elif key == pygame.K_g:
            self.set_mobile_velocity_command(-v_fwd, v_str, 0.0, "Backward-Left")
        elif key == pygame.K_x:
            self.manual_stop()
    
    def handle_gripper_controls(self, key):
        """Handle gripper control keys"""
        if key == pygame.K_g:
            self.set_gripper_state('closed')
        elif key == pygame.K_r:
            self.set_gripper_state('open')
    
    def draw_interface(self):
        """Draw the pygame interface"""
        # Clear screen
        self.screen.fill((30, 30, 30))
        
        # Title
        title = self.font.render("Unified Robot & Gripper Control", True, (255, 255, 255))
        self.screen.blit(title, (10, 10))
        
        # Current mode indicator
        mode_color = (100, 255, 100) if self.control_mode == 'mobile' else (255, 100, 100)
        mode_text = self.font.render(f"Mode: {self.control_mode.upper()}", True, mode_color)
        self.screen.blit(mode_text, (10, 40))
        
        # Speed multiplier
        speed_text = self.small_font.render(f"Speed: {self.speed_multiplier:.1f}x", True, (200, 200, 200))
        self.screen.blit(speed_text, (10, 70))
        
        # Emergency stop indicator
        if self.emergency_stop:
            emergency_text = self.font.render(" EMERGENCY STOP ACTIVE ", True, (255, 0, 0))
            self.screen.blit(emergency_text, (10, 100))
        
        # Controls based on mode
        y_offset = 130
        if self.control_mode == 'mobile':
            mobile_controls = [
                "MOBILE BASE CONTROLS:",
                "WASD - Move Forward/Back/Left/Right",
                "QE - Rotate Left/Right",
                "RTFG - Diagonal Movement",
                "X/SPACE - Manual Stop"
            ]
            for i, control in enumerate(mobile_controls):
                color = (255, 255, 255) if i == 0 else (180, 180, 180)
                text = self.small_font.render(control, True, color)
                self.screen.blit(text, (10, y_offset + i * 20))
        else:
            gripper_controls = [
                "GRIPPER CONTROLS:",
                "G - Grasp (Close Gripper)",
                "R - Release (Open Gripper)"
            ]
            for i, control in enumerate(gripper_controls):
                color = (255, 255, 255) if i == 0 else (180, 180, 180)
                text = self.small_font.render(control, True, color)
                self.screen.blit(text, (10, y_offset + i * 20))
        
        # General controls
        general_y = y_offset + 120
        general_controls = [
            "GENERAL CONTROLS:",
            "TAB - Switch Control Mode",
            "ESC - Emergency Stop",
            "+/- - Speed Control"
        ]
        for i, control in enumerate(general_controls):
            color = (255, 255, 255) if i == 0 else (180, 180, 180)
            text = self.small_font.render(control, True, color)
            self.screen.blit(text, (10, general_y + i * 20))
        
        # Status information
        status_y = general_y + 100
        
        # Mobile base status
        wheel_vels = self.get_wheel_velocities()
        mobile_status = [
            f"Mobile Velocity: [{self.velocity[0]:.2f}, {self.velocity[1]:.2f}, {self.velocity[2]:.2f}]",
            f"Wheel Velocities: [{wheel_vels[0]:.2f}, {wheel_vels[1]:.2f}, {wheel_vels[2]:.2f}, {wheel_vels[3]:.2f}]"
        ]
        
        for i, status in enumerate(mobile_status):
            text = self.small_font.render(status, True, (100, 255, 100))
            self.screen.blit(text, (10, status_y + i * 20))
        
        # Gripper status
        gripper_status = f"Gripper State: {self.gripper_state.upper()}"
        gripper_text = self.small_font.render(gripper_status, True, (255, 100, 100))
        self.screen.blit(gripper_text, (10, status_y + 50))
        
        # Statistics
        stats_y = status_y + 80
        uptime = time.time() - self.stats['start_time']
        stats_text = [
            f"Uptime: {uptime:.1f}s",
            f"Total Commands: {self.stats['total_commands']}",
            f"Mobile Commands: {self.stats['mobile_commands']}",
            f"Gripper Commands: {self.stats['gripper_commands']}"
        ]
        
        for i, stat in enumerate(stats_text):
            text = self.small_font.render(stat, True, (150, 150, 150))
            self.screen.blit(text, (10, stats_y + i * 20))
        
        pygame.display.flip()
    
    def control_loop(self):
        """Main control loop"""
        self.logger.info("Starting unified control loop")
        
        # Telemetry timing
        last_telemetry_time = time.time()
        telemetry_interval = 1.0 / self.config.telemetry_update_rate
        
        try:
            with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
                self.logger.info("MuJoCo viewer launched")
                
                while viewer.is_running() and self.running:
                    step_start = time.time()
                    
                    # Handle pygame events
                    self.handle_keyboard_events()
                    
                    # Update auto-stop logic for mobile base
                    self.update_auto_stop()
                    
                    # Apply controls
                    self.apply_mobile_control()
                    self.apply_gripper_control()
                    
                    # Run physics step
                    if not self.emergency_stop:
                        mujoco.mj_step(self.model, self.data)
                    
                    # Sync viewer
                    viewer.sync()
                    
                    # Update interface
                    current_time = time.time()
                    if current_time - last_telemetry_time >= telemetry_interval:
                        self.draw_interface()
                        last_telemetry_time = current_time
                    
                    # Maintain timing
                    elapsed = time.time() - step_start
                    sleep_time = max(0, self.model.opt.timestep - elapsed)
                    if sleep_time > 0:
                        time.sleep(sleep_time)
                        
        except Exception as e:
            self.logger.error(f"Error in control loop: {e}")
            self.running = False
        finally:
            self.logger.info("Control loop finished")
    
    def run(self):
        """Run the unified controller"""
        try:
            self.control_loop()
        except KeyboardInterrupt:
            self.logger.info("Received keyboard interrupt")
        except Exception as e:
            self.logger.error(f"Unexpected error: {e}")
        finally:
            self.shutdown()
    
    def shutdown(self):
        """Clean shutdown"""
        self.running = False
        pygame.quit()
        self.logger.info("Shutting down unified robot controller")
        
        # Print final statistics
        uptime = time.time() - self.stats['start_time']
        print(f"\n Session Stats:")
        print(f"   Total Commands: {self.stats['total_commands']}")
        print(f"   Mobile Commands: {self.stats['mobile_commands']}")
        print(f"   Gripper Commands: {self.stats['gripper_commands']}")
        print(f"   Uptime: {uptime:.1f}s")

def load_config(config_path: str = "unified_robot_config.json") -> UnifiedRobotConfig:
    """Load configuration from JSON file"""
    if Path(config_path).exists():
        try:
            with open(config_path, 'r') as f:
                config_dict = json.load(f)
            print(f"Loaded configuration from {config_path}")
            return UnifiedRobotConfig(**config_dict)
        except Exception as e:
            print(f"Error loading config: {e}, using defaults")
            return UnifiedRobotConfig()
    else:
        # Create default config file
        config = UnifiedRobotConfig()
        try:
            with open(config_path, 'w') as f:
                json.dump(config.__dict__, f, indent=2)
            print(f"Created default configuration file: {config_path}")
        except Exception as e:
            print(f"Could not create config file: {e}")
        return config

def main():
    """Main entry point"""
    print(" Unified MuJoCo Robot & Gripper Controller")
    print("=" * 50)
    
    try:
        # Load configuration
        config = load_config()
        
        # Create and run controller
        controller = UnifiedRobotController(config)
        controller.run()
        
    except Exception as e:
        print(f" Failed to start unified controller: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()