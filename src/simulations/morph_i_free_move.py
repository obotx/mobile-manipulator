import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import mujoco, os
import glfw, time
import numpy as np
import datetime, cv2
from scipy.optimize import minimize
import argparse
from modules.utils.pubsub import IPCPubSub
from scipy.spatial.transform import Rotation as R
from modules.controllers.morph_i import MorphIManipulatorV2
import threading

np.set_printoptions(suppress=True, precision=4)

class ParallelRobot:
    DAMPING = 8e-4
    DT = 0.002
        
    base_integral_1=0 
    base_prev_error_1=0
    base_integral_2=0 
    base_prev_error_2=0

    r = 0.1  
    D = 0.55   

    mobile_dot = np.zeros(4)
    target_vel = np.zeros(4)
    command = np.zeros(4)

    integral_x = 0.0
    integral_y = 0.0
    integral_yaw = 0.0
    prev_delta_x = 0.0
    prev_delta_y = 0.0
    prev_delta_yaw = 0.0
    
    ARM_JOINT_LEFT = [
        "ColumnLeftBearingJoint_1",
        "ColumnRightBearingJoint_1",
        "ArmLeftJoint_1",
        "BaseJoint_1",]
    
    ARM_JOINT_RIGHT = [
        "ColumnLeftBearingJoint_2",
        "ColumnRightBearingJoint_2",
        "ArmLeftJoint_2",
        "BaseJoint_2",
    ]
    
    ARM_ACTUATOR_LEFT = [
        "ColumnLeftBearingJointMotor_1",
        "ColumnRightBearingJointMotor_1",
        "ArmLeftJointMotor_1",
        "BaseJointMotor_1",
    ]
    
    ARM_ACTUATOR_RIGHT = [
        "ColumnLeftBearingJointMotor_2",
        "ColumnRightBearingJointMotor_2",
        "ArmLeftJointMotor_2",
        "BaseJointMotor_2",
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
        "HandBearing_1"
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
        "HandBearing_2"
    ]
        
    def __init__(self, path: str, run_mode: str, record: bool):
        self.model = mujoco.MjModel.from_xml_path(path)
        self.data = mujoco.MjData(self.model)
        self.manipulator_control_left = MorphIManipulatorV2()
        self.manipulator_control_right = MorphIManipulatorV2()
        self.reset("home")
        self._target_lock = threading.Lock()
        self._initialize_ids()
        self._initialize_arrays()
        
        self._terminate = False  
        self.paused = False 
        self.run_mode = run_mode.lower()
        self.record = record
        self.current_ctrl = np.zeros(len(self.ARM_ACTUATOR_LEFT) + len(self.ARM_ACTUATOR_RIGHT))
        
        self.camera = mujoco.MjvCamera()
        self.camera.distance = 5.0         
        self.camera.azimuth = 90            
        self.camera.elevation = -45         
        self.camera.lookat[:] = [0, 0, 0]
        
        self.use_ik = False 
        self.direct_arm_commands = np.concatenate([
            np.concatenate([
                self.data.ctrl[self.arm_ids_left[:3]] / 100.0,
                [0.0],
                self.data.ctrl[self.arm_ids_left[4:]]
            ]),
            np.concatenate([
                self.data.ctrl[self.arm_ids_right[:3]] / 100.0,
                [0.0],
                self.data.ctrl[self.arm_ids_right[4:]]
            ]),
            self.data.ctrl[self.gripper_ids_left],
            self.data.ctrl[self.gripper_ids_right]
        ])

        if self.run_mode == "glfw":
            if not glfw.init():
                raise RuntimeError("GLFW failed to initialize")
            self.window = glfw.create_window(1200, 900, "Gripper Simulation", None, None)
            if not self.window:
                glfw.terminate()
                raise RuntimeError("GLFW failed to create window")
            glfw.make_context_current(self.window)
            self.ctx = mujoco.MjrContext(self.model, mujoco.mjtFontScale.mjFONTSCALE_150)
            self.viewport = mujoco.MjrRect(0, 0, 1200, 900)
            self.scene = mujoco.MjvScene(self.model, maxgeom=500)
            self.opt = mujoco.MjvOption()

            glfw.set_key_callback(self.window, self.on_key)
            # glfw.set_cursor_pos_callback(self.window, self._cursor_pos_callback)
            # glfw.set_mouse_button_callback(self.window, self._mouse_button_callback)
            glfw.set_scroll_callback(self.window, self._scroll_callback)

            # self.scene.flags = mujoco.mjtRndFlag.mjRND_SHADOW #disable shadow
        
            self._last_mouse_x = 0
            self._last_mouse_y = 0
            self._mouse_left_pressed = False
            self._mouse_right_pressed = False
            self._mouse_middle_pressed = False
            
        if self.run_mode == "cv":
            self.renderer_top = mujoco.Renderer(self.model, height=640, width=1024)

            self.model.vis.global_.offheight = 640
            self.model.vis.global_.offwidth = 1024
        
        self.current_waypoint_idx = 0
        self.time_at_current_waypoint = 0.0
        self.speed = 3  
        self.angular_speed = 3  
        self.grab_time = 0.0
        self.grab_hold_duration = 0.5
        self.grab_move_speed = 0.2
        self.progress = 0
        
        self.h1, self.h2, self.l1, self.l2 = 0, 0, 0, 0
        self.top_video_writer = None
        self.pov_video_writer = None

        self.ipc = IPCPubSub()
        self.subscriber = self.ipc.create_subscriber()
        self.subscriber.subscribe("target_base", self._on_target_base)
        self.subscriber.subscribe("target_left", self._on_target_left)
        self.subscriber.subscribe("target_right", self._on_target_right)
        self.subscriber.subscribe("ik_mode", self._on_ik_mode)
        self.subscriber.subscribe("u_control", self._on_arm_control)
        self.subscriber.start()
        
    def _initialize_ids(self):
        self.arm_link_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "Arm")
        self.end_effector_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "Gripper_Link1")
        self.base_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "base_footprint")

        self.arm_ids_left = np.array([self.model.actuator(name).id for name in self.ARM_ACTUATOR_LEFT])
        self.arm_ids_right = np.array([self.model.actuator(name).id for name in self.ARM_ACTUATOR_RIGHT])
        
        self.gripper_ids_left = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name) for name in self.GRIPPER_ACT_LEFT]
        self.gripper_ids_right = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name) for name in self.GRIPPER_ACT_RIGHT]
        
        self.qpos_indices = []
        self.qvel_indices = []

        for name in self.ARM_ACTUATOR_LEFT + self.ARM_ACTUATOR_LEFT:
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            qpos_adr = self.model.jnt_qposadr[joint_id]
            dof_adr = self.model.jnt_dofadr[joint_id]
            joint_type = self.model.jnt_type[joint_id]

            if joint_type in [3, 2]:
                self.qpos_indices.append(qpos_adr)
                self.qvel_indices.append(dof_adr)
            elif joint_type == 1:
                self.qpos_indices.extend(range(qpos_adr, qpos_adr + 3))
                self.qvel_indices.extend(range(dof_adr, dof_adr + 3))
            elif joint_type == 0:
                self.qpos_indices.extend(range(qpos_adr, qpos_adr + 6))
                self.qvel_indices.extend(range(dof_adr, dof_adr + 6))

        self.qpos_indices = np.array(self.qpos_indices)
        self.qvel_indices = np.array(self.qvel_indices)

        l0, r0 = self.get_encoder()
        self.target_left = np.array(self.manipulator_control_left.fk(l0))
        self.target_right = np.array(self.manipulator_control_right.fk(r0))
        self.target_base = self.localization()

    def _initialize_arrays(self):
        self.jacp = np.zeros((3, self.model.nv))
        self.jacr = np.zeros((3, self.model.nv))
        self.error = np.zeros(6)
        self.error_pos = np.zeros(3)
        self.error_ori = np.zeros(3)
        self.site_quat = np.zeros(4)
        self.site_quat_conj = np.zeros(4)
        self.error_quat = np.zeros(4)
        self.target_rot_left = np.eye(3)
        self.target_rot_right = np.eye(3)
        
    def _on_ik_mode(self, msg):
        try:
            enabled = bool(msg)
            with self._target_lock:
                self.use_ik = enabled
                self.target_right = arr.copy()
        except Exception as e:
            print(f"[PubSub] Invalid target_right message: {msg}, error: {e}")

            print(f"[INFO] IK mode: {'ENABLED' if enabled else 'DISABLED'}")
        except Exception as e:
            print(f"[ERROR] Invalid ik_mode message: {msg}, error: {e}")

    def _on_arm_control(self, msg):
        try:
            raw = np.array(msg, dtype=float)
            if raw.shape != (8,):
                raise ValueError(f"Expected 8 values, got {raw.size}")

            h_min, h_max = -75.0, 75.0   
            h_out_min, h_out_max = 0.0, 1.5  

            a_min, a_max = -30.0, 30.0    
            a_out_min, a_out_max = 0.0, 0.6

            def remap(value, in_min, in_max, out_min, out_max):
                return out_min + (value - in_min) * (out_max - out_min) / (in_max - in_min)

            l_h1 = np.clip(remap(raw[0], h_min, h_max, h_out_min, h_out_max), h_out_min, h_out_max)
            l_h2 = np.clip(remap(raw[1], h_min, h_max, h_out_min, h_out_max), h_out_min, h_out_max)
            l_a1 = np.clip(remap(raw[2], a_min, a_max, a_out_min, a_out_max), a_out_min, a_out_max)
            l_theta = np.deg2rad(raw[3])  
            r_h1 = np.clip(remap(raw[4], h_min, h_max, h_out_min, h_out_max), h_out_min, h_out_max)
            r_h2 = np.clip(remap(raw[5], h_min, h_max, h_out_min, h_out_max), h_out_min, h_out_max)
            r_a1 = np.clip(remap(raw[6], a_min, a_max, a_out_min, a_out_max), a_out_min, a_out_max)
            r_theta = np.deg2rad(raw[7])  
            mapped = np.array([l_h1, l_h2, l_a1, l_theta, r_h1, r_h2, r_a1, r_theta])

            with self._target_lock:
                self.direct_arm_commands = mapped.copy()
        except Exception as e:
            print(f"[ERROR] Invalid u_control message: {msg}, error: {e}")

    def _on_target_base(self, msg):
        try:
            arr = np.array(msg, dtype=float)
            if arr.shape != (3,):
                raise ValueError(f"Expected shape (3,), got {arr.shape}")
            with self._target_lock:
                self.target_base = arr.copy()
        except Exception as e:
            print(f"[PubSub] Invalid target_base message: {msg}, error: {e}")

    def _on_target_left(self, msg):
        try:
            arr = np.array(msg, dtype=float)
            if arr.shape != (3,):
                raise ValueError(f"Expected shape (3,), got {arr.shape}")
            with self._target_lock:
                self.target_left = arr.copy()
        except Exception as e:
            print(f"[PubSub] Invalid target_left message: {msg}, error: {e}")

    def _on_target_right(self, msg):
        try:
            arr = np.array(msg, dtype=float)
            if arr.shape != (3,):
                raise ValueError(f"Expected shape (3,), got {arr.shape}")
            with self._target_lock:
                self.target_right = arr.copy()
        except Exception as e:
            print(f"[PubSub] Invalid target_right message: {msg}, error: {e}")

    def reset(self, keyframe_name:str):
        key_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, keyframe_name)
        mujoco.mj_resetDataKeyframe(self.model, self.data, key_id)
        
    def get_keyframe(self, keyframe_name:str):
        def format_array(arr):
            return " ".join(f"{x:.6f}" for x in arr)
        
        print(f'<key\n    name="{keyframe_name}"')
        print(f'    qpos="  {format_array(self.data.qpos)}"')
        print(f'    qvel="  {format_array(self.data.qvel)}"')
        print(f'    ctrl="  {format_array(self.data.ctrl)}"')
        print('/>')
        
    def configure_model(self):
        self.model.opt.timestep = self.DT
        self.model.body_gravcomp[:] = True
        
    def get_joint_qpos_addr(self, joint_name):
        jnt_id = self.model.joint(joint_name).id
        return self.model.jnt_qposadr[jnt_id]
    
    def obstacle_avoidance(self):
        pass
    
    def localization(self):
        x, y = self.data.xpos[self.base_id, 0], self.data.xpos[self.base_id, 1]
        w, xq, yq, zq = self.data.xquat[self.base_id]
        yaw = np.arctan2(2 * (w * zq + xq * yq), 1 - 2 * (yq**2 + zq**2))
        return np.array([x, y, yaw])
    
    def get_encoder(self):
        col_left_1 = self.data.qpos[self.get_joint_qpos_addr("ColumnLeftBearingJoint_1")]
        col_right_1 = self.data.qpos[self.get_joint_qpos_addr("ColumnRightBearingJoint_1")]
        horiz_left = self.data.qpos[self.get_joint_qpos_addr("ArmLeftJoint_1")]
        base_left = self.data.qpos[self.get_joint_qpos_addr("BaseJoint_1")]
        hand_bearing_left = self.data.qpos[self.get_joint_qpos_addr("HandBearingJoint_1")]
        
        roll_left  = self.data.qpos[self.get_joint_qpos_addr("gripper_x_rotation_1")]
        pitch_left = self.data.qpos[self.get_joint_qpos_addr("gripper_y_rotation_1")]
        yaw_left   = self.data.qpos[self.get_joint_qpos_addr("gripper_z_rotation_1")]

        col_left_2 = self.data.qpos[self.get_joint_qpos_addr("ColumnLeftBearingJoint_2")]
        col_right_2 = self.data.qpos[self.get_joint_qpos_addr("ColumnRightBearingJoint_2")]
        horiz_right = self.data.qpos[self.get_joint_qpos_addr("ArmLeftJoint_2")]
        base_right = self.data.qpos[self.get_joint_qpos_addr("BaseJoint_2")]
        hand_bearing_right = self.data.qpos[self.get_joint_qpos_addr("HandBearingJoint_2")]
        
        roll_right  = self.data.qpos[self.get_joint_qpos_addr("gripper_x_rotation_2")]
        pitch_right = self.data.qpos[self.get_joint_qpos_addr("gripper_y_rotation_2")]
        yaw_right   = self.data.qpos[self.get_joint_qpos_addr("gripper_z_rotation_2")]

        arm_encoder_left = np.array([
            col_left_1, col_right_1,
            horiz_left,
            base_left,
            hand_bearing_left,
            roll_left, pitch_left, yaw_left
        ])

        arm_encoder_right = np.array([
            col_left_2, col_right_2,
            horiz_right,
            base_right,
            hand_bearing_right,
            roll_right, pitch_right, yaw_right
        ])

        return arm_encoder_left, arm_encoder_right
        
    def send_command_arm(self, u_arm_control, side='both'):
        u_arm_control = np.asarray(u_arm_control)

        if side == 'left':
            actuator_ids = self.arm_ids_left
        elif side == 'right':
            actuator_ids = self.arm_ids_right
        elif side == 'both':
            actuator_ids = self.arm_ids_left + self.arm_ids_right
        else:
            raise ValueError("side must be 'left', 'right', or 'both'")

        if u_arm_control.shape != (len(actuator_ids),):
            raise ValueError(
                f"Arm control input shape {u_arm_control.shape} does not match "
                f"number of {side} arm actuators ({len(actuator_ids)})"
            )

        ctrl_ranges = self.model.actuator_ctrlrange[actuator_ids]
        lo, hi = ctrl_ranges[:, 0], ctrl_ranges[:, 1]
        u_clipped = np.clip(u_arm_control, lo, hi)
        self.data.ctrl[actuator_ids] = u_clipped
        
    def send_command_hand(self, u_hand_control, side='both'):
        u_hand_control = np.asarray(u_hand_control)

        if side == 'left':
            actuator_ids = self.gripper_ids_left
        elif side == 'right':
            actuator_ids = self.gripper_ids_right
        elif side == 'both':
            actuator_ids = self.gripper_ids_left + self.gripper_ids_right
        else:
            raise ValueError("side must be 'left', 'right', or 'both'")

        if u_hand_control.shape != (len(actuator_ids),):
            raise ValueError(
                f"Hand control input shape {u_hand_control.shape} does not match "
                f"number of {side} hand actuators ({len(actuator_ids)})"
            )

        ctrl_ranges = self.model.actuator_ctrlrange[actuator_ids]
        lo, hi = ctrl_ranges[:, 0], ctrl_ranges[:, 1]
        u_clipped = np.clip(u_hand_control, lo, hi)
        self.data.ctrl[actuator_ids] = u_clipped
    
    def ik(self, 
        target_pos, 
        target_rot=None,
        arm="left",
        pos_weight=1.0,
        ori_weight=0.3,
        cache_threshold_pos=0.001,
        cache_threshold_ori=0.001,  
        max_iter=50,
        tol=1e-6):

        target_pos = np.asarray(target_pos, dtype=float)
        
        if target_rot is None:
            curr_q_left, curr_q_right = self.get_encoder()
            curr_q = curr_q_left if arm == "left" else curr_q_right
            curr_pose = self.manipulator_control_left.fk(curr_q) if arm == "left" else self.manipulator_control_right.fk(curr_q)
            target_rot = R.from_quat(curr_pose[3:]).as_matrix()
        elif len(target_rot) == 4:
            target_rot = R.from_quat(target_rot).as_matrix()
        else:
            target_rot = np.asarray(target_rot, dtype=float)
            if target_rot.shape != (3, 3):
                raise ValueError("target_rot must be quat (4,) or rotmat (3,3)")

        target_key = (tuple(target_pos), tuple(target_rot.ravel()))

        if not hasattr(self, '_ik_cache'):
            self._ik_cache = {'left': {'target': None, 'result': None},
                            'right': {'target': None, 'result': None}}

        cache = self._ik_cache[arm]

        if cache['target'] is not None:
            prev_pos, prev_rot_flat = cache['target']
            prev_pos = np.array(prev_pos)
            prev_rot = np.array(prev_rot_flat).reshape(3, 3)
            
            pos_dist = np.linalg.norm(target_pos - prev_pos)
            rot_error = R.from_matrix(target_rot @ prev_rot.T)
            ori_dist = np.linalg.norm(rot_error.as_rotvec())
            
            if pos_dist <= cache_threshold_pos and ori_dist <= cache_threshold_ori:
                return cache['result'].copy()

        q_left, q_right = self.get_encoder()
        q0 = q_left if arm == "left" else q_right
        
        if arm == "left" :
            control = self.manipulator_control_left
        else :
            control = self.manipulator_control_right

        q_ik = control.ik(
            target_pos=target_pos,
            target_rot=target_rot,
            q0=q0,
            pos_weight=pos_weight,
            ori_weight=ori_weight,
            max_iter=max_iter,
            tol=tol,
        )

        self._ik_cache[arm]['target'] = (tuple(target_pos), tuple(target_rot.ravel()))
        self._ik_cache[arm]['result'] = q_ik.copy() if q_ik is not None else q0.copy()

        if q_ik is None:
            print(f"[IK] Warning: Failed for {arm} arm. Returning current or fallback configuration.")
        
        return q_ik
            
    def pid_base_joints(self, target_angle_1, target_angle_2, kp=10, ki=0.0, kd=7):
        dt = self.model.opt.timestep
        jnt1_id = self.model.joint("BaseJoint_1").id
        qpos1 = self.data.qpos[self.model.jnt_qposadr[jnt1_id]]
        error1 = (target_angle_1 - qpos1 + np.pi) % (2 * np.pi) - np.pi

        self.base_integral_1 += error1 * dt
        derivative1 = (error1 - self.base_prev_error_1) / dt 
        self.base_prev_error_1 = error1

        torque1 = kp * error1 + ki * self.base_integral_1 + kd * derivative1

        jnt2_id = self.model.joint("BaseJoint_2").id
        qpos2 = self.data.qpos[self.model.jnt_qposadr[jnt2_id]]
        error2 = (target_angle_2 - qpos2 + np.pi) % (2 * np.pi) - np.pi

        self.base_integral_2 += error2 * dt
        derivative2 = (error2 - self.base_prev_error_2) / dt
        self.base_prev_error_2 = error2

        torque2 = kp * error2 + ki * self.base_integral_2 + kd * derivative2

        return torque1, torque2
                
    def control_base(self, target, alpha=0):
        k_p = 5.0
        k_i = 0.1
        k_d = 0.8
        
        k_p_theta = 5.0
        k_i_theta = 0.1
        k_d_theta = 0.8

        self.mobile_dot[0] = self.data.qvel[19]
        self.mobile_dot[1] = self.data.qvel[6] 
        self.mobile_dot[2] = self.data.qvel[45]
        self.mobile_dot[3] = self.data.qvel[32]

        target_x, target_y, target_yaw = target 

        current_x, current_y = self.data.xpos[self.base_id, 0], self.data.xpos[self.base_id, 1]
        w, x, y, z = self.data.xquat[self.base_id]
        current_yaw = np.arctan2(2*(w*z + x*y), 1 - 2*(y**2 + z**2))

        delta_x = target_x - current_x
        delta_y = target_y - current_y
        delta_yaw = np.arctan2(np.sin(target_yaw - current_yaw),
                            np.cos(target_yaw - current_yaw))

        delta_x_local = np.cos(current_yaw) * delta_x + np.sin(current_yaw) * delta_y
        delta_y_local = -np.sin(current_yaw) * delta_x + np.cos(current_yaw) * delta_y

        self.integral_x += delta_x_local * self.model.opt.timestep
        self.integral_y += delta_y_local * self.model.opt.timestep
        self.integral_yaw += delta_yaw * self.model.opt.timestep
        deriv_x_local = (delta_x_local - self.prev_delta_x) / self.model.opt.timestep
        deriv_y_local = (delta_y_local - self.prev_delta_y) / self.model.opt.timestep
        deriv_yaw = (delta_yaw - self.prev_delta_yaw) / self.model.opt.timestep

        self.deriv_x = alpha * getattr(self, "deriv_x", 0.0) + (1 - alpha) * deriv_x_local
        self.deriv_y = alpha * getattr(self, "deriv_y", 0.0) + (1 - alpha) * deriv_y_local
        self.deriv_yaw = alpha * getattr(self, "deriv_yaw", 0.0) + (1 - alpha) * deriv_yaw

        self.prev_delta_x = delta_x_local
        self.prev_delta_y = delta_y_local
        self.prev_delta_yaw = delta_yaw

        v_x_local = k_p * delta_x_local + k_i * self.integral_x + k_d * self.deriv_x
        v_y_local = k_p * delta_y_local + k_i * self.integral_y + k_d * self.deriv_y
        omega = k_p_theta * delta_yaw + k_i_theta * self.integral_yaw + k_d_theta * self.deriv_yaw

        self.target_vel[0] = (v_x_local - v_y_local - omega * self.D) / self.r
        self.target_vel[1] = (v_x_local + v_y_local + omega * self.D) / self.r
        self.target_vel[2] = (v_x_local + v_y_local - omega * self.D) / self.r
        self.target_vel[3] = (v_x_local - v_y_local + omega * self.D) / self.r

        self.command = self.target_vel - self.mobile_dot
        eps = 0.05
        self.command[np.abs(self.command) < eps] = 0.0

        self.data.ctrl[0] = self.command[1]
        self.data.ctrl[1] = self.command[0]
        self.data.ctrl[2] = self.command[3]
        self.data.ctrl[3] = self.command[2]
        
    def control_arms_hands(self):
        q_left, q_right = self.get_encoder()

        if self.use_ik:
            u_left_desired = self.ik(
                target_pos=self.target_left,
                target_rot=self.target_rot_left,
            )
            u_right_desired = self.ik(
                target_pos=self.target_right,
                target_rot=self.target_rot_right,
            )

            if u_left_desired is None or u_right_desired is None:
                print("[Control] IK failed; skipping command update.")
                return

            arm_cmd_left = u_left_desired[:4]
            arm_cmd_right = u_right_desired[:4]
            phi_left, roll_left, pitch_left, yaw_left = u_left_desired[4], u_left_desired[5], u_left_desired[6], u_left_desired[7]
            phi_right, roll_right, pitch_right, yaw_right = u_right_desired[4], u_right_desired[5], u_right_desired[6], u_right_desired[7]

        else:
            if len(self.direct_arm_commands) != 38:
                print("[Control] direct_arm_commands has wrong length.")
                return

            arm_cmd_left = self.direct_arm_commands[0:4]
            arm_cmd_right = self.direct_arm_commands[4:8]

            grip_left = self.direct_arm_commands[8:23]
            grip_right = self.direct_arm_commands[23:38]

            roll_left, pitch_left, yaw_left = grip_left[11], grip_left[12], grip_left[13]
            phi_left = grip_left[14]
            roll_right, pitch_right, yaw_right = grip_right[11], grip_right[12], grip_right[13]
            phi_right = grip_right[14]

        theta_left = arm_cmd_left[3]
        theta_right = arm_cmd_right[3]
        u_base_left, u_base_right = self.pid_base_joints(theta_left, theta_right)

        offset = np.array([-0.0036, -0.0062, -0.0006])
        gain = 100.0
        raw_L = (arm_cmd_left[:3] + offset) * gain
        raw_R = (arm_cmd_right[:3] + offset) * gain

        alpha = 0.1
        if not hasattr(self, '_smooth_cmd_L'):
            self._smooth_cmd_L = raw_L.copy()
            self._smooth_cmd_R = raw_R.copy()
        self._smooth_cmd_L = (1 - alpha) * self._smooth_cmd_L + alpha * raw_L
        self._smooth_cmd_R = (1 - alpha) * self._smooth_cmd_R + alpha * raw_R

        final_arm_cmd_left = np.array([
            self._smooth_cmd_L[0],
            self._smooth_cmd_L[1],
            self._smooth_cmd_L[2],
            u_base_left
        ])
        final_arm_cmd_right = np.array([
            self._smooth_cmd_R[0],
            self._smooth_cmd_R[1],
            self._smooth_cmd_R[2],
            u_base_right
        ])

        self.send_command_arm(final_arm_cmd_left, side='left')
        self.send_command_arm(final_arm_cmd_right, side='right')

        current_hand_left = self.data.ctrl[self.gripper_ids_left].copy()
        current_hand_right = self.data.ctrl[self.gripper_ids_right].copy()

        current_hand_left[14] = phi_left   # HandBearing_1
        current_hand_left[11] = roll_left  # wrist_X_1
        current_hand_left[12] = pitch_left # wrist_Y_1
        current_hand_left[13] = yaw_left   # wrist_Z_1

        current_hand_right[14] = phi_right
        current_hand_right[11] = roll_right
        current_hand_right[12] = pitch_right
        current_hand_right[13] = yaw_right

        self.send_command_hand(current_hand_left, side='left')
        self.send_command_hand(current_hand_right, side='right')
            
    def step_simulation(self, render=True):
        # self.get_keyframe("home")
        self.control_base(target=self.target_base, alpha=0.1)
        self.control_arms_hands()
            
        q_left, q_right = self.get_encoder()
        fk_left = self.manipulator_control_left.fk(q_left)
        print("-----------")
        print(q_left)
        print(fk_left)

        # # Print joint angles
        # print(f"Joint Angles (q_left):")
        # joint_names = ['z_act_a', 'z_act_b', 'l_telesc', 'theta_base', 'phi_wrist', 
        #             'roll_grip', 'pitch_grip', 'yaw_grip']
        # for i, (name, val) in enumerate(zip(joint_names, q_left)):
        #     print(f"  {name:12}: {np.rad2deg(val):+.4f} {'rad' if i >= 3 else 'm'}")

        # # Print FK results
        # pos = fk_left[:3]
        # quat = fk_left[3:]
        # rpy = R.from_quat(quat).as_euler('xyz', degrees=True)

        # print(f"\nFK Result (Left Arm):")
        # print(f"  Position: [{pos[0]:+.3f}, {pos[1]:+.3f}, {pos[2]:+.3f}] m")
        # print(f"  Rotation: [{rpy[0]:+.1f}°, {rpy[1]:+.1f}°, {rpy[2]:+.1f}°] (roll, pitch, yaw)")
        # print(f"  Quaternion: [{quat[0]:+.3f}, {quat[1]:+.3f}, {quat[2]:+.3f}, {quat[3]:+.3f}]")
                
        mujoco.mj_step(self.model, self.data, nstep=5)
        if self.run_mode == "glfw" and render:
            mujoco.mjv_updateScene(
                self.model, self.data, self.opt, None, self.camera, 0xFFFF, self.scene
            )
            glfw.swap_buffers(self.window)
            glfw.poll_events()
            
    def camera_display(self):
        self.frame_count = getattr(self, 'frame_count', 0)
        self.renderer_top.update_scene(self.data, self.camera)
        rgb_top = self.renderer_top.render()
        if rgb_top is None or rgb_top.size == 0:
            print("Error: Top view rendering failed")
            return
        bgr_top = cv2.cvtColor(rgb_top, cv2.COLOR_RGB2BGR)

        if self.top_video_writer is not None and self.top_video_writer.isOpened():
            self.top_video_writer.write(bgr_top)

        self.frame_count += 1

        cv2.imshow("MuJoCo Top View", bgr_top)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self._terminate = True

    def run_cv(self):
        if self.record:
            output_dir = "output_videos"
            os.makedirs(output_dir, exist_ok=True)
            
            frame_width, frame_height = self.model.vis.global_.offwidth, self.model.vis.global_.offheight 
            fps = 30
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            top_video_file = os.path.join(output_dir, f"top_view_{timestamp}.mp4")
            pov_video_file = os.path.join(output_dir, f"pov_view_{timestamp}.mp4")
            fourcc = cv2.VideoWriter_fourcc(*'MJPG') 

            self.top_video_writer = cv2.VideoWriter(top_video_file, fourcc, fps, (frame_width, frame_height))
            self.pov_video_writer = cv2.VideoWriter(pov_video_file, fourcc, fps, (frame_width, frame_height))

            if not self.top_video_writer.isOpened():
                print(f"Error: Failed to open top video writer for {top_video_file}")
            if not self.pov_video_writer.isOpened():
                print(f"Error: Failed to open side video writer for {pov_video_file}")

        try:
            mujoco.mj_step(self.model, self.data, nstep=1)
            self._terminate = False
            while not self._terminate:
                self.step_simulation()
        except Exception as e:
            print(f"Simulation error: {e}")
        finally:
            if self.record:
                if self.top_video_writer is not None:
                    self.top_video_writer.release()
                    print(f"Released top video writer: {top_video_file}")
                if self.pov_video_writer is not None:
                    self.pov_video_writer.release()
                    print(f"Released side video writer: {pov_video_file}")
            cv2.destroyAllWindows()
            self.renderer_top.close()

    def run_glfw(self):
        mujoco.mj_step(self.model, self.data, nstep=1)
        while not glfw.window_should_close(self.window) and not self._terminate:
            self.step_simulation()
            mujoco.mjv_updateScene(self.model, self.data, self.opt, None, self.camera, mujoco.mjtCatBit.mjCAT_ALL, self.scene)
            mujoco.mjr_render(self.viewport, self.scene, self.ctx)
        glfw.terminate()
           
    def on_key(self, window, key, scancode, action, mods):
        if action not in (glfw.PRESS, glfw.REPEAT):
            return
        if key == glfw.KEY_ESCAPE:
            glfw.set_window_should_close(self.window, True)
            return
        
        if key == glfw.KEY_ENTER:
            self.reset()
            self.gripper_ctrl = 0.0
            return

    def _cursor_pos_callback(self, window, xpos, ypos):
        if self.camera.type != mujoco.mjtCamera.mjCAMERA_FREE:
            self._last_mouse_x, self._last_mouse_y = xpos, ypos
            return

        dx = xpos - self._last_mouse_x
        dy = ypos - self._last_mouse_y
        self._last_mouse_x, self._last_mouse_y = xpos, ypos
        factor = 0.001
        if self._mouse_left_pressed:
            mujoco.mjv_moveCamera(
                self.model, mujoco.mjtMouse.mjMOUSE_ROTATE_H,
                dx*factor, dy*factor, self.scene, self.camera
            )
        elif self._mouse_right_pressed:
            mujoco.mjv_moveCamera(
                self.model, mujoco.mjtMouse.mjMOUSE_MOVE_H,
                dx*factor, dy*factor, self.scene, self.camera
            )
        elif self._mouse_middle_pressed:
            mujoco.mjv_moveCamera(
                self.model, mujoco.mjtMouse.mjMOUSE_ZOOM,
                dx*factor, dy*factor, self.scene, self.camera
            )

    def _mouse_button_callback(self, window, button, action, mods):
        if self.camera.type != mujoco.mjtCamera.mjCAMERA_FREE:
            return
        pressed = (action == glfw.PRESS)
        if button == glfw.MOUSE_BUTTON_LEFT:
            self._mouse_left_pressed = pressed
        elif button == glfw.MOUSE_BUTTON_RIGHT:
            self._mouse_right_pressed = pressed
        elif button == glfw.MOUSE_BUTTON_MIDDLE:
            self._mouse_middle_pressed = pressed

    def _scroll_callback(self, window, xoffset, yoffset):
        if self.camera.type != mujoco.mjtCamera.mjCAMERA_FREE:
            return
        factor = 0.05
        mujoco.mjv_moveCamera(
            self.model, mujoco.mjtMouse.mjMOUSE_ZOOM,
            0, yoffset*factor, self.scene, self.camera
        )

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run MuJoCo Parallel Robot Simulation")
    parser.add_argument("--run", choices=["glfw", "cv"], default="glfw",
                        help="Run mode: 'glfw' or 'cv'")
    parser.add_argument("--record", action="store_true",
                        help="Record video output to MP4 (only applicable with --run cv)")
    args = parser.parse_args()

    if args.record and args.run != "cv":
        print("Warning: --record is only applicable with --run cv. Ignoring --record.")
        args.record = False
    xml_path = os.path.join(os.path.dirname(__file__), '..', 'env', 'market_world_plain.xml')
    xml_path = os.path.abspath(xml_path)
    sim = ParallelRobot(xml_path, args.run, args.record)
    if args.run == "glfw":
        sim.run_glfw()
    else:  # cv
        sim.run_cv()