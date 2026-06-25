import numpy as np
import mujoco
from typing import Tuple

class BaseJointPIDController:
    def __init__(self):
        self.integral_1 = 0.0; self.prev_error_1 = 0.0
        self.integral_2 = 0.0; self.prev_error_2 = 0.0

    def compute_torques(self, model: mujoco.MjModel, data: mujoco.MjData, target_angle_1: float, target_angle_2: float, kp=20, ki=0.1, kd=7) -> Tuple[float, float]:
        dt = model.opt.timestep
        def calc_torque(joint_name, target, integral, prev_error):
            jnt_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            qpos = data.qpos[model.jnt_qposadr[jnt_id]]
            error = (target - qpos + np.pi) % (2 * np.pi) - np.pi
            integral += error * dt
            derivative = (error - prev_error) / dt
            torque = kp * error + ki * integral + kd * derivative
            return torque, integral, error
        
        t1, self.integral_1, self.prev_error_1 = calc_torque("BaseJoint_1", target_angle_1, self.integral_1, self.prev_error_1)
        t2, self.integral_2, self.prev_error_2 = calc_torque("BaseJoint_2", target_angle_2, self.integral_2, self.prev_error_2)
        return t1, t2

class MobileBaseController:
    def __init__(self, r: float, D: float):
        self.r = r; self.D = D
        self.mobile_dot = np.zeros(4); self.target_vel = np.zeros(4); self.command = np.zeros(4)
        self.integral_x = 0.0; self.integral_y = 0.0; self.integral_yaw = 0.0
        self.prev_delta_x = 0.0; self.prev_delta_y = 0.0; self.prev_delta_yaw = 0.0
        self.deriv_x = 0.0; self.deriv_y = 0.0; self.deriv_yaw = 0.0

    def compute_wheel_commands(self, model: mujoco.MjModel, data: mujoco.MjData, base_id: int, target: np.ndarray, alpha=0.0) -> np.ndarray:
        k_p = 5.0; k_i = 0.1; k_d = 0.8; k_p_theta = 5.0; k_i_theta = 0.1; k_d_theta = 0.8
        dt = model.opt.timestep
        
        self.mobile_dot[:] = [data.qvel[19], data.qvel[6], data.qvel[45], data.qvel[32]]
        target_x, target_y, target_yaw = target 
        current_x, current_y = data.xpos[base_id, 0], data.xpos[base_id, 1]
        w, x, y, z = data.xquat[base_id]
        current_yaw = np.arctan2(2*(w*z + x*y), 1 - 2*(y**2 + z**2))
        
        delta_x = target_x - current_x; delta_y = target_y - current_y
        delta_yaw = np.arctan2(np.sin(target_yaw - current_yaw), np.cos(target_yaw - current_yaw))
        
        delta_x_local = np.cos(current_yaw) * delta_x + np.sin(current_yaw) * delta_y
        delta_y_local = -np.sin(current_yaw) * delta_x + np.cos(current_yaw) * delta_y
        
        self.integral_x += delta_x_local * dt; self.integral_y += delta_y_local * dt; self.integral_yaw += delta_yaw * dt
        
        deriv_x_local = (delta_x_local - self.prev_delta_x) / dt
        deriv_y_local = (delta_y_local - self.prev_delta_y) / dt
        deriv_yaw = (delta_yaw - self.prev_delta_yaw) / dt
        
        self.deriv_x = alpha * self.deriv_x + (1 - alpha) * deriv_x_local
        self.deriv_y = alpha * self.deriv_y + (1 - alpha) * deriv_y_local
        self.deriv_yaw = alpha * self.deriv_yaw + (1 - alpha) * deriv_yaw
        
        self.prev_delta_x = delta_x_local; self.prev_delta_y = delta_y_local; self.prev_delta_yaw = delta_yaw
        
        v_x_local = k_p * delta_x_local + k_i * self.integral_x + k_d * self.deriv_x
        v_y_local = k_p * delta_y_local + k_i * self.integral_y + k_d * self.deriv_y
        omega = k_p_theta * delta_yaw + k_i_theta * self.integral_yaw + k_d_theta * self.deriv_yaw
        
        self.target_vel[:] = [
            (v_x_local - v_y_local - omega * self.D) / self.r, 
            (v_x_local + v_y_local + omega * self.D) / self.r, 
            (v_x_local + v_y_local - omega * self.D) / self.r, 
            (v_x_local - v_y_local + omega * self.D) / self.r
        ]
        
        self.command = self.target_vel - self.mobile_dot
        self.command[np.abs(self.command) < 0.05] = 0.0
        return np.array([self.command[1], self.command[0], self.command[3], self.command[2]])

class GripperController:
    def __init__(self):
        self.open_pos = [0.0, 0.0, 0.0610865, 0.0, -0.0872665, 0.0610865, 0.0, -0.0872665, 0.0610865, 0.0, -0.0872665]
        self.close_pos = [0.0, 0.0, 0.8066, 0.174533, -0.610865, 0.8066, 0.174533, -0.610865, 0.8066, 0.174533, -0.610865]

    def apply_commands(self, data: mujoco.MjData, gripper_ids_left: list, gripper_ids_right: list, left_is_grab: bool, right_is_grab: bool):
        def apply_cmd(gripper_ids, is_grabbing):
            cmd = data.ctrl[gripper_ids].copy()
            target = self.close_pos if is_grabbing else self.open_pos
            cmd[9], cmd[10] = target[0], target[1]
            cmd[0], cmd[1], cmd[2] = target[2], target[3], target[4]
            cmd[3], cmd[4], cmd[5] = target[5], target[6], target[7]
            cmd[6], cmd[7], cmd[8] = target[8], target[9], target[10]
            data.ctrl[gripper_ids] = cmd
        apply_cmd(gripper_ids_left, left_is_grab)
        apply_cmd(gripper_ids_right, right_is_grab)