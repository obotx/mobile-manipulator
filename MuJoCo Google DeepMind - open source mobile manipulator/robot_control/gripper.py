import mujoco
import numpy as np
from typing import Dict
import time

class GripperController:
    def __init__(self, model, data, config):
        self.model = model
        self.data = data
        self.config = config
        self.state = 'open'
        
        # Motion planning variables
        self.current_positions = {}
        self.target_positions = config['open_positions'].copy()
        self.trajectory_start_time = time.time()
        self.trajectory_duration = 2.0  # 2 seconds for smooth motion
        self.is_moving = False
        
        # Get gripper joint indices and actuator indices
        self.joint_indices = {}
        self.actuator_indices = {}
        
        # Map joints to their actuators
        for joint_name in self.target_positions.keys():
            try:
                joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
                self.joint_indices[joint_name] = joint_id
                
                # Find actuator that controls this joint
                for i in range(self.model.nu):
                    if self.model.actuator_trntype[i] == mujoco.mjtTrn.mjTRN_JOINT:
                        if self.model.actuator_trnid[i, 0] == joint_id:
                            self.actuator_indices[joint_name] = i
                            break
            except:
                print(f"Warning: Could not find joint {joint_name}")
                pass
        
        # Initialize current positions
        self._update_current_positions()
    
    def _update_current_positions(self):
        """Update current joint positions from simulation"""
        for joint_name, joint_id in self.joint_indices.items():
            self.current_positions[joint_name] = self.data.qpos[joint_id]
    
    def set_state(self, state: str):
        """Set target gripper state with smooth transition"""
        if state == self.state:
            return  # Already in target state
        
        # Update current positions before starting new trajectory
        self._update_current_positions()
        
        # Set new target positions
        if state == 'open':
            self.target_positions = self.config['open_positions'].copy()
        elif state == 'closed':
            self.target_positions = self.config['closed_positions'].copy()
        
        # Start trajectory
        self.state = state
        self.is_moving = True
        self.trajectory_start_time = time.time()
    
    def _cubic_interpolation(self, t):
        """Cubic interpolation for smooth motion (0 to 1)"""
        # Cubic easing: smooth acceleration and deceleration
        if t <= 0:
            return 0
        elif t >= 1:
            return 1
        else:
            # Smooth cubic curve
            return 3 * t**2 - 2 * t**3
    
    def _get_interpolated_position(self, joint_name, current_time):
        """Get interpolated position for smooth trajectory"""
        if joint_name not in self.current_positions or joint_name not in self.target_positions:
            return 0.0
        
        # Calculate progress (0 to 1)
        elapsed = current_time - self.trajectory_start_time
        progress = min(elapsed / self.trajectory_duration, 1.0)
        
        # Apply cubic interpolation
        smooth_progress = self._cubic_interpolation(progress)
        
        # Interpolate between current and target
        start_pos = self.current_positions[joint_name]
        target_pos = self.target_positions[joint_name]
        
        return start_pos + (target_pos - start_pos) * smooth_progress
    
    def apply_control(self):
        """Apply smooth position control to gripper joints"""
        current_time = time.time()
        
        # Check if trajectory is complete
        if self.is_moving and (current_time - self.trajectory_start_time) >= self.trajectory_duration:
            self.is_moving = False
            # Update current positions to final targets
            self.current_positions = self.target_positions.copy()
        
        for joint_name in self.target_positions.keys():
            if joint_name in self.joint_indices and joint_name in self.actuator_indices:
                joint_id = self.joint_indices[joint_name]
                actuator_id = self.actuator_indices[joint_name]
                
                # Get current state
                current_pos = self.data.qpos[joint_id]
                current_vel = self.data.qvel[joint_id]
                
                # Get target position (interpolated if moving)
                if self.is_moving:
                    target_pos = self._get_interpolated_position(joint_name, current_time)
                else:
                    target_pos = self.target_positions[joint_name]
                
                # PD control for position
                position_error = target_pos - current_pos
                velocity_error = 0.0 - current_vel  # Target velocity is 0
                
                # Control force calculation
                kp = self.config['control_gain']
                kd = self.config['damping']
                control_force = kp * position_error + kd * velocity_error
                
                # Apply control to actuator
                self.data.ctrl[actuator_id] = control_force
    
    def get_status(self):
        """Get current gripper status"""
        return {
            'state': self.state,
            'is_moving': self.is_moving,
            'progress': min((time.time() - self.trajectory_start_time) / self.trajectory_duration, 1.0) if self.is_moving else 1.0
        }