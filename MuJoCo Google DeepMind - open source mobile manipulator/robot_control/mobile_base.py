import numpy as np
import mujoco
import time
from typing import Dict, Optional
from .trajectory import MobileBaseTrajectory

class MobileBaseController:
    def __init__(self, model, data, config):
        self.model = model
        self.data = data
        self.config = config
        self.trajectory = MobileBaseTrajectory()
        
        # Initialize wheel mappings - using the working script's configuration
        self.wheel_qvel_ids = {"FL": 19, "FR": 6, "BL": 45, "BR": 32}  # From working script
        self.motor_ids = {"FR": 0, "FL": 1, "BR": 2, "BL": 3}  # From working script
        
        # Verify the IDs exist in the model
        for wheel, joint_id in self.wheel_qvel_ids.items():
            if joint_id >= self.model.nq:
                print(f"Warning: Invalid wheel joint ID {joint_id} for {wheel}")
        
        for wheel, motor_id in self.motor_ids.items():
            if motor_id >= self.model.nu:
                print(f"Warning: Invalid motor ID {motor_id} for {wheel}")
        
        # State variables from working script
        self.velocity = np.array([0.0, 0.0, 0.0])  # Current smoothed velocity
        self.target_velocity = np.array([0.0, 0.0, 0.0])  # Target velocity
        self.last_command_time = time.time()
        self.manual_stop_flag = False
        self.emergency_stop = False
        
        # Mecanum wheel geometry - using standard values
        self.wheel_radius = 0.05  # 5cm radius
        self.lx = 0.2  # half of wheelbase length
        self.ly = 0.15  # half of wheelbase width
        
    def set_velocity_command(self, vx: float, vy: float, omega: float):
        """Set target velocity for mobile base"""
        self.target_velocity = np.array([vx, vy, omega])
        self.last_command_time = time.time()
        self.manual_stop_flag = False
        
    def emergency_stop(self):
        """Immediately halt all motion"""
        with self.lock:
            self.velocity = np.zeros(3)
            self.target_velocity = np.zeros(3)
            self.emergency_stop = True
            self.manual_stop_flag = True
            self.last_command_time = time.time()
        
    def manual_stop(self):
        """Manual stop for mobile base"""
        self.target_velocity = np.zeros(3)
        self.manual_stop_flag = True
        self.last_command_time = time.time()
        
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
        # Using the simpler version from working script that worked well
        return np.array([
            vx - vy - omega,  # FL
            vx + vy + omega,  # FR
            vx + vy - omega,  # BL
            vx - vy + omega   # BR
        ])
    
    def cap_wheel_velocities(self, target_velocities: np.ndarray) -> np.ndarray:
        """Cap wheel velocities while preserving ratios"""
        max_target = np.max(np.abs(target_velocities))
        if max_target > self.config['max_wheel_velocity']:
            scale_factor = self.config['max_wheel_velocity'] / max_target
            return target_velocities * scale_factor
        return target_velocities
    
    def smooth_velocity_transition(self, target: np.ndarray) -> np.ndarray:
        """Apply velocity smoothing to reduce jerky movements"""
        if not self.config.get('enable_smoothing', True):
            return target
            
        smoothing_factor = self.config.get('smoothing_factor', 0.2)
        self.velocity = (self.velocity * (1 - smoothing_factor) + 
                       target * smoothing_factor)
        return self.velocity.copy()
    
    def update_auto_stop(self):
        """Handle auto-stop logic for mobile base"""
        if self.emergency_stop:
            return
            
        current_time = time.time()
        time_since_last_command = current_time - self.last_command_time
        
        if not self.manual_stop_flag and time_since_last_command > self.config['auto_stop_timeout']:
            self.target_velocity *= self.config['velocity_decay_rate']
            if np.all(np.abs(self.target_velocity) < self.config['min_velocity_threshold']):
                self.target_velocity = np.zeros(3)
    
    def apply_control(self, dt: float):
        """Apply control to mobile base"""
        if self.emergency_stop:
            # Set all motors to zero torque
            for motor_id in self.motor_ids.values():
                self.data.ctrl[motor_id] = 0.0
            return
            
        # Update auto-stop
        self.update_auto_stop()
        
        # Get current wheel velocities
        wheel_velocities = self.get_wheel_velocities()
        
        # Apply velocity smoothing (either from trajectory or basic smoothing)
        if hasattr(self.trajectory, 'update_velocity'):
            current_velocity = self.trajectory.update_velocity(self.target_velocity, dt)
        else:
            current_velocity = self.smooth_velocity_transition(self.target_velocity)
        
        # Mecanum inverse kinematics
        target_wheel_vels = self.mecanum_inverse_kinematics(
            current_velocity[0], current_velocity[1], current_velocity[2]
        )
        
        # Cap velocities
        target_wheel_vels = self.cap_wheel_velocities(target_wheel_vels)
        
        # PD control for motor torques - using working script's approach
        velocity_error = target_wheel_vels - wheel_velocities
        motor_torques = velocity_error * self.config['kv']
        
        # Apply torque limits
        motor_torques = np.clip(motor_torques, -self.config['max_torque'], self.config['max_torque'])
        
        # Apply motor commands - using working script's motor mapping
        self.data.ctrl[self.motor_ids["FL"]] = motor_torques[0]
        self.data.ctrl[self.motor_ids["FR"]] = motor_torques[1]
        self.data.ctrl[self.motor_ids["BL"]] = motor_torques[2]
        self.data.ctrl[self.motor_ids["BR"]] = motor_torques[3]
    
    def get_status(self) -> Dict:
        """Get current mobile base status"""
        return {
            'target_velocity': self.target_velocity.copy(),
            'current_velocity': self.velocity.copy(),
            'wheel_velocities': self.get_wheel_velocities(),
            'emergency_stop': self.emergency_stop,
            'manual_stop': self.manual_stop_flag
        }