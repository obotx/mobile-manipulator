from email.mime import base
import ompl.base as ob
import ompl.geometric as og
from pickle import FALSE
import cv2
import numpy as np
import mujoco
import threading
import glfw
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation as R
import pyvista as pv
import time

from config.morph_params import *
from core.manipulator.kinematics import MorphIManipulator
from core.base.mecanum import *
from core.basic.pid import SimplePID
from utils.quaternion import *
from utils.camera import VideoRecorder
from interface.glfw_interface import setup_glfw_window, GLFWInputHandler
from core.basic.pubsub import ZMQPubSub
from core.basic.mj_helpers import *
from core.wbc.hierarchical_wbc import *

def fmt(v):
    formatted_items = ", ".join(f"{x:7.3f}" for x in v)
    return f"[{formatted_items}]"

class ParallelRobot:
    def __init__(self, path: str, run_mode: str, record: bool):
        self.model = mujoco.MjModel.from_xml_path(path)
        self.data = mujoco.MjData(self.model)
        self.model.opt.timestep = DT
        self.model.body_gravcomp[:] = True

        # self.reset("home")
        self._target_lock = threading.Lock()
        self._initialize_ids()
        self._initialize_arrays()

        self._terminate = False
        self.paused = False
        self.run_mode = run_mode.lower()
        self.record = record and (self.run_mode == "cv")
        self.use_ik = False

        # Manipulators for left and right arms
        self.manipulator_left = MorphIManipulator(
            name="left",
            bounds_theta_deg=(-360, 360) 
        )
        self.manipulator_right = MorphIManipulator(
            name="right",
            bounds_theta_deg=(-360, 360) 
        )

        # Initialize targets from home pose
        l0, r0 = self.get_encoder()
        self.target_left = self.manipulator_left.fk(l0) 
        self.target_right = self.manipulator_right.fk(r0) 
        self.target_base = self.localization()

        # Initial direct commands
        self.direct_arm_commands = np.concatenate([
            [0.5, 0.58, 0.3],                               # h1, h2, a1 (left)
            [0],                                           # theta (left)
            [self.data.ctrl[self.actuator_ids[4]]],        # phi (left)
            [0.5, 0.58, 0.3],                               # h1, h2, a1 (right)
            [0],                                           # theta (right)
            [self.data.ctrl[self.actuator_ids[9]]]         # phi (right)
        ])

        # Camera setup
        self.camera = mujoco.MjvCamera()
        self.camera.distance = 5.0
        self.camera.azimuth = 90
        self.camera.elevation = -45
        self.camera.lookat[:] = [0, 0, 0]

        # Rendering context
        if self.run_mode == "glfw":
            self.window = setup_glfw_window()
            self.ctx = mujoco.MjrContext(self.model, mujoco.mjtFontScale.mjFONTSCALE_150)
            self.viewport = mujoco.MjrRect(0, 0, 1200, 900)
            self.scene = mujoco.MjvScene(self.model, maxgeom=500)
            self.opt = mujoco.MjvOption()
            self.input_handler = GLFWInputHandler(self.window, self.camera, self.scene, self.model)
        elif self.run_mode == "cv":
            self.renderer_top = mujoco.Renderer(self.model, height=640, width=1024)
            if self.record:
                self.video_recorder = VideoRecorder(width=1024, height=640)
                self.video_recorder.start_recording()
            else:
                self.video_recorder = None

        # IPC Setup
        self.ipc = ZMQPubSub()
        self.subscriber = self.ipc.create_subscriber()
        
        self.subscriber.subscribe("target_base", self._on_target_base)
        self.subscriber.subscribe("target_left", self._on_target_left)
        self.subscriber.subscribe("target_right", self._on_target_right)
        self.subscriber.subscribe("ik_mode", self._on_ik_mode)
        self.subscriber.subscribe("u_control", self._on_arm_control)
        self.subscriber.subscribe("wbc_mode", self._on_wbc_mode)
        self.subscriber.subscribe("wbc_target", self._on_wbc_target)
        self.subscriber.subscribe("wbc_target_pitch", self._on_wbc_target_pitch)
        
        self.subscriber.subscribe("wbc/left/enabled", lambda msg: self._set_wbc_enabled("left", msg))
        self.subscriber.subscribe("wbc/right/enabled", lambda msg: self._set_wbc_enabled("right", msg))
        self.subscriber.subscribe("wbc/left/target", lambda msg: self._set_wbc_target("left", msg))
        self.subscriber.subscribe("wbc/right/target", lambda msg: self._set_wbc_target("right", msg))
        self.subscriber.subscribe("wbc/left/pitch", lambda msg: self._set_wbc_pitch("left", msg))
        self.subscriber.subscribe("wbc/right/pitch", lambda msg: self._set_wbc_pitch("right", msg))
        self.subscriber.subscribe("wbc/arm_side", self._on_wbc_arm_side)
        
        self.subscriber.start()
        
        self.pcd_publisher = self.ipc.create_publisher()

        dt = self.model.opt.timestep 
        self.pid_x      = SimplePID(kp=60.0, ki=2.0, kd=10.0, dt=dt, wrap=False)
        self.pid_y      = SimplePID(kp=60.0, ki=2.0, kd=10.0, dt=dt, wrap=False)
        self.pid_yaw    = SimplePID(kp=120.0, ki=2.0, kd=10.0, dt=dt, wrap=True)

        # Height controller (h)
        h_kp, h_ki, h_kd = 4000.0, 10000.0, 150.0

        # Arm joints (a1 left/right)
        a1_left_kp,  a1_left_ki,  a1_left_kd  = 4000.0, 12000.0, 350.0
        a1_right_kp, a1_right_ki, a1_right_kd = 4000.0, 12000.0, 350.0

        rot_kp = 20.0
        rot_ki = 0.0
        rot_kd = 3.0
        
        self.pids = {
            'h1_left':   SimplePID(h_kp, h_ki, h_kd, dt, wrap=False),
            'h2_left':   SimplePID(h_kp, h_ki, h_kd, dt, wrap=False),
            'h1_right':  SimplePID(h_kp, h_ki, h_kd, dt, wrap=False),
            'h2_right':  SimplePID(h_kp, h_ki, h_kd, dt, wrap=False),

            'a1_left':   SimplePID(a1_left_kp, a1_left_ki, a1_left_kd, dt, wrap=False),
            'a1_right':  SimplePID(a1_right_kp, a1_right_ki, a1_right_kd, dt, wrap=False),

            'theta_left':  SimplePID(rot_kp, rot_ki, rot_kd, dt, wrap=True),
            'phi_left':    SimplePID(rot_kp, rot_ki, rot_kd, dt, wrap=True),
            'theta_right': SimplePID(rot_kp, rot_ki, rot_kd, dt, wrap=True),
            'phi_right':   SimplePID(rot_kp, rot_ki, rot_kd, dt, wrap=True),
        }
        
        # Arm base joint PID states (for yaw joints)
        self.base_integral_1 = 0.0
        self.base_prev_error_1 = 0.0
        self.base_integral_2 = 0.0
        self.base_prev_error_2 = 0.0

        # Legacy WBC state (for backward compatibility)
        self.wbc_enabled = False
        self.wbc_target_arm = None      
        self.wbc_arm_side = "left"     
        self._last_wbc_targets = {"left": None, "right": None}
        self._last_wbc_pitch = {"left": None, "right": None}
        self._last_wbc_update = 0.0
        self.WBC_UPDATE_DT = 0.2 
        self.use_wbc_joint_targets = False
        
        self.wbc_enabled_per_arm = {"left": False, "right": False}
        self.wbc_targets_per_arm = {"left": None, "right": None}
        self.wbc_pitch_targets_per_arm = {"left": 0.0, "right": 0.0}
        
        # OMPL trajectory execution
        self.ompl_trajectory = None
        self.ompl_traj_idx = 0
        self.use_ompl_trajectory = False
        
        # Initialize OMPL planner
        self.setup_ompl_planner()
        
        # Arm offset
        self.arm_offsets = {
            "left":     np.array([0.15 + 0.01,   0.15 - 0.00465966,    0.158566 + 0.127 + 0.0542705]),
            "right":    np.array([0.16,        -(0.16 - 0.00465966),   0.158566 + 0.127 + 0.0896705])
        }
            
        self.target_pitch_left = 0.0
        self.odom_pose = np.array([0.0, 0.0, 0.0])
        
        self.pov_camera_name = "pov_1"
        self.pov_camera_id = self.model.cam(self.pov_camera_name).id
        H, W = 480, 640
        if self.run_mode == "cv":
            self.pov_renderer = mujoco.Renderer(self.model, height=480, width=680)
        else:
            self.pov_renderer = None
        self.pcd_queue = None
        self.pcd_publisher_process = None

    def _initialize_ids(self):
        self.arm_link_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "Arm")
        self.base_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "base_footprint")
        self.gt_left_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SITE, "attachment_site_1"
        )
        self.gt_right_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SITE, "attachment_site_2"
        )
        self.tgt_left_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SITE, "target_site_1"
        )
        self.tgt_right_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SITE, "target_site_2"
        )
        
        # self.mocap_right_id = self.model.body("target_right").mocapid[0]
        # self.mocap_left_id = self.model.body("target_left").mocapid[0]
        
        self.actuator_ids = np.array([self.model.actuator(name).id for name in ACTUATOR_NAMES])
        self.gripper_ids_left = [
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            for name in GRIPPER_ACT_LEFT
        ]
        self.gripper_ids_right = [
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            for name in GRIPPER_ACT_RIGHT
        ]
        
        self.wheel_joint_indices = np.array([
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            for name in WHEEL_JOINT_NAMES
        ])

    def _initialize_arrays(self):
        self.jacp = np.zeros((3, self.model.nv))
        self.jacr = np.zeros((3, self.model.nv))

    def reset(self, keyframe_name: str = "home"):
        key_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, keyframe_name)
        mujoco.mj_resetDataKeyframe(self.model, self.data, key_id)

    def get_keyframe(self, keyframe_name: str):
        fmt = lambda arr: " ".join(f"{x:.6f}" for x in arr)
        print(f'<key name="{keyframe_name}" qpos="  {fmt(self.data.qpos)}" '
              f'qvel="  {fmt(self.data.qvel)}" ctrl="  {fmt(self.data.ctrl)}"/>')

    def get_joint_qpos_addr(self, joint_name):
        return self.model.jnt_qposadr[self.model.joint(joint_name).id]

    def localization(self, full=False):
        x, y, z = self.data.xpos[self.base_id, 0], self.data.xpos[self.base_id, 1], self.data.xpos[self.base_id, 2]
        w, xq, yq, zq = self.data.xquat[self.base_id]
        yaw = np.arctan2(2 * (w * zq + xq * yq), 1 - 2 * (yq**2 + zq**2))
        
        if full :
            return np.array([x, y, z, yaw])
        
        return np.array([x, y, yaw])

    def get_encoder(self):
        def q(name): return self.data.qpos[self.model.jnt_qposadr[self.model.joint(name).id]]
        return (
            np.array([  q("ColumnLeftBearingJoint_1"), 
                        q("ColumnRightBearingJoint_1"),
                        q("ArmLeftJoint_1"), 
                        q("BaseJoint_1"), 
                        q("HandBearingJoint_1")]),  
            np.array([  q("ColumnLeftBearingJoint_2"), 
                        q("ColumnRightBearingJoint_2"),
                        q("ArmLeftJoint_2"),
                        q("BaseJoint_2"), 
                        q("HandBearingJoint_2")])
        )

    def send_command_arm(self, u_control):
        u_control = np.asarray(u_control)
        if u_control.shape != (len(self.actuator_ids),):
            raise ValueError(f"Control input shape mismatch")
        lo, hi = self.model.actuator_ctrlrange[self.actuator_ids].T
        self.data.ctrl[self.actuator_ids] = np.clip(u_control, lo, hi)

    # =============== IPC Callbacks ===============
    def _on_wbc_mode(self, msg):
        try:
            enabled = bool(msg)
            with self._target_lock:
                self.wbc_enabled = enabled
                self.wbc_enabled_per_arm[self.wbc_arm_side] = enabled
                self.use_ik = enabled
            status = "ENABLED" if enabled else "DISABLED"
            print(f"[WBC] Legacy mode: {status} for {self.wbc_arm_side} arm")
        except Exception as e:
            print(f"[WBC] Invalid wbc_mode message: {msg}, error: {e}")

    def _on_wbc_target(self, msg):
        try:
            arr = np.array(msg, dtype=float)
            if arr.shape != (3,):
                raise ValueError(f"Expected shape (3,), got {arr.shape}")
            with self._target_lock:
                self.wbc_target_arm = arr.copy()
                self.wbc_targets_per_arm[self.wbc_arm_side] = arr.copy()
                self.wbc_enabled = True
                self.wbc_enabled_per_arm[self.wbc_arm_side] = True
                self.use_ik = True
            print(f"[WBC] Legacy target set for {self.wbc_arm_side} arm: {arr}")
        except Exception as e:
            print(f"[WBC] Invalid wbc_target message: {msg}, error: {e}")

    def _on_ik_mode(self, msg):
        try:
            enabled = bool(msg)
            with self._target_lock:
                self.use_ik = enabled
            print(f"[INFO] IK mode: {'ENABLED' if enabled else 'DISABLED'}")
        except Exception as e:
            print(f"[ERROR] Invalid ik_mode message: {msg}, error: {e}")

    def _on_arm_control(self, msg):
        try:
            raw = np.array(msg, dtype=float)
            h_min, h_max = 0.0, 1.5
            h_out_min, h_out_max = 0.0, 1.5
            a_min, a_max = 0.0, 0.625
            a_out_min, a_out_max = 0.0, 0.625

            def remap(value, in_min, in_max, out_min, out_max):
                return out_min + (value - in_min) * (out_max - out_min) / (in_max - in_min)

            l_h1 = np.clip(remap(raw[0], h_min, h_max, h_out_min, h_out_max), h_out_min, h_out_max)
            l_h2 = np.clip(remap(raw[1], h_min, h_max, h_out_min, h_out_max), h_out_min, h_out_max)
            l_a1 = np.clip(remap(raw[2], a_min, a_max, a_out_min, a_out_max), a_out_min, a_out_max)
            l_theta = np.deg2rad(raw[3])
            l_phi   = np.deg2rad(raw[4])
            r_h1 = np.clip(remap(raw[5], h_min, h_max, h_out_min, h_out_max), h_out_min, h_out_max)
            r_h2 = np.clip(remap(raw[6], h_min, h_max, h_out_min, h_out_max), h_out_min, h_out_max)
            r_a1 = np.clip(remap(raw[7], a_min, a_max, a_out_min, a_out_max), a_out_min, a_out_max)
            r_theta = np.deg2rad(raw[8])
            r_phi   = np.deg2rad(raw[9])
            mapped = np.array([l_h1, l_h2, l_a1, l_theta, l_phi, r_h1, r_h2, r_a1, r_theta, r_phi])

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
            
    def _on_wbc_target_pitch(self, msg):
        try:
            pitch = float(msg)
            with self._target_lock:
                self.wbc_pitch_target = pitch
                self.wbc_pitch_targets_per_arm[self.wbc_arm_side] = pitch
            print(f"[WBC] Legacy pitch target set to {pitch:.1f}° for {self.wbc_arm_side} arm")
        except Exception as e:
            print(f"[WBC] Invalid pitch target: {msg}, error: {e}")
    
    def _on_wbc_arm_side(self, msg):
        """New callback to set which arm legacy commands apply to"""
        side = str(msg).lower()
        if side in ["left", "right"]:
            with self._target_lock:
                self.wbc_arm_side = side
            print(f"[WBC] Legacy commands now apply to {side.upper()} arm")
        else:
            print(f"[WBC] Invalid arm side: {msg}. Use 'left' or 'right'")
            
    # ====== PER-ARM CALLBACKS ======
    def _set_wbc_enabled(self, arm_side, msg):
        try:
            enabled = bool(msg)
            with self._target_lock:
                self.wbc_enabled_per_arm[arm_side] = enabled
                if enabled:
                    self.wbc_enabled = False
                self.use_ik = self.wbc_enabled_per_arm["left"] or self.wbc_enabled_per_arm["right"]
            status = "ENABLED" if enabled else "DISABLED"
            print(f"[WBC] {arm_side.upper()} arm control {status}")
        except Exception as e:
            print(f"[WBC] Invalid enable for {arm_side}: {msg}, error: {e}")

    def _set_wbc_target(self, arm_side, msg):
        try:
            arr = np.array(msg, dtype=float)
            if arr.shape != (3,):
                raise ValueError(f"Expected shape (3,) for {arm_side}, got {arr.shape}")
            with self._target_lock:
                if arm_side.lower() == "left":
                    self.data.mocap_pos[self.mocap_left_id][:] = arr
                if arm_side.lower() == "right":
                    self.data.mocap_pos[self.mocap_right_id][:] = arr

                self.wbc_targets_per_arm[arm_side] = arr.copy()
                self.wbc_enabled_per_arm[arm_side] = True
                self.use_ik = True
            print(f"[WBC] {arm_side.upper()} target set to {arr}")
        except Exception as e:
            print(f"[WBC] Invalid target for {arm_side}: {msg}, error: {e}")

    def _set_wbc_pitch(self, arm_side, msg):
        try:
            pitch = float(msg)
            with self._target_lock:
                self.wbc_pitch_targets_per_arm[arm_side] = pitch
            print(f"[WBC] {arm_side.upper()} pitch target: {pitch:.1f}°")
        except Exception as e:
            print(f"[WBC] Invalid pitch for {arm_side}: {msg}, error: {e}")
            
    # =============== OMPL HELPER METHODS ===============
    def get_body_name_from_geom_id(self, geom_id):
        if geom_id < 0 or geom_id >= self.model.ngeom:
            return None
        
        body_id = self.model.geom_bodyid[geom_id]
        if body_id < 0:
            return "floor"
        
        body_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, body_id)
        return body_name

    def should_ignore_contact(self, body1_name, body2_name):
        is_body1_floor = (body1_name == "floor")
        is_body2_floor = (body2_name == "floor")
        
        if not (is_body1_floor or is_body2_floor):
            return False
            
        non_floor_body = body2_name if is_body1_floor else body1_name
        
        if non_floor_body is None:
            return False
            
        if "trashbin" in non_floor_body.lower():
            return True
        if "one_nescaafe_taster" in non_floor_body.lower():
            return True
        if "roller" in non_floor_body.lower():
            return True
            
        return False

    def debug_state_validity(self):
        """Debug current state validity with intelligent filtering"""
        mujoco.mj_forward(self.model, self.data)
        
        print(f"Total contacts: {self.data.ncon}")
        real_collisions = 0
        ignored_contacts = 0
        
        for i in range(self.data.ncon):
            contact = self.data.contact[i]
            geom1_id = contact.geom1
            geom2_id = contact.geom2
            
            body1_name = self.get_body_name_from_geom_id(geom1_id)
            body2_name = self.get_body_name_from_geom_id(geom2_id)
            
            if self.should_ignore_contact(body1_name, body2_name):
                print(f"  Contact {i}: ({body1_name}) <-> ({body2_name}) (IGNORED)")
                ignored_contacts += 1
            else:
                print(f"  Contact {i}: ({body1_name}) <-> ({body2_name}) (REAL COLLISION!)")
                real_collisions += 1
        
        print(f"Ignored contacts: {ignored_contacts}")
        if real_collisions == 0:
            print("✅ No real collisions detected!")
            return True
        else:
            print(f"❌ {real_collisions} real collision(s) detected!")
            return False
    
    # =============== OMPL SETUP ===============
    def setup_ompl_planner(self):
        self.ompl_dim = 13  # [x, y, yaw, qL(5), qR(5)]
        space = ob.RealVectorStateSpace(self.ompl_dim)
        bounds = ob.RealVectorBounds(self.ompl_dim)

        # Base bounds
        bounds.setLow(0, -10); bounds.setHigh(0, 10)
        bounds.setLow(1, -10); bounds.setHigh(1, 10)
        bounds.setLow(2, -np.pi); bounds.setHigh(2, np.pi)

        # Arm bounds
        arm_bounds = [
            self.manipulator_left.bounds_h,
            self.manipulator_left.bounds_h,
            self.manipulator_left.bounds_a,
            self.manipulator_left.bounds_theta,
            self.manipulator_left.bounds_phi
        ]
        print(self.manipulator_left.bounds_theta)
        for i, (low, high) in enumerate(arm_bounds):
            bounds.setLow(3 + i, low)
            bounds.setHigh(3 + i, high)
            bounds.setLow(8 + i, low)
            bounds.setHigh(8 + i, high)

        space.setBounds(bounds)
        si = ob.SpaceInformation(space)

        def is_state_valid(ompl_state):
            state = np.array([ompl_state[i] for i in range(13)])
            x, y, yaw = state[0:3]
            qL = state[3:8]
            qR = state[8:13]

            for q, manip in [(qL, self.manipulator_left), (qR, self.manipulator_right)]:
                h1, h2, a1, theta, phi = q
                alpha = np.arctan2(abs(h2 - h1), manip.d2)
                if alpha < manip.alpha_min_rad:
                    return False

            original_qpos = self.data.qpos.copy()
            try:
                adr = self.get_joint_qpos_addr('base_footprint')
                self.data.qpos[adr + 0] = x
                self.data.qpos[adr + 1] = y
                cy = np.cos(yaw * 0.5)
                sy = np.sin(yaw * 0.5)
                self.data.qpos[adr + 3] = cy
                self.data.qpos[adr + 4] = 0.0
                self.data.qpos[adr + 5] = 0.0
                self.data.qpos[adr + 6] = sy

                # Set arm joints
                for i, name in enumerate(JOINT_NAMES_LEFT):
                    self.data.qpos[self.get_joint_qpos_addr(name)] = qL[i]
                for i, name in enumerate(JOINT_NAMES_RIGHT):
                    self.data.qpos[self.get_joint_qpos_addr(name)] = qR[i]

                mujoco.mj_forward(self.model, self.data)
                
                for i in range(self.data.ncon):
                    contact = self.data.contact[i]
                    geom1_id = contact.geom1
                    geom2_id = contact.geom2
                    
                    body1_name = self.get_body_name_from_geom_id(geom1_id)
                    body2_name = self.get_body_name_from_geom_id(geom2_id)
                    
                    if self.should_ignore_contact(body1_name, body2_name):
                        continue
                    return False
                return True
                
            finally:
                self.data.qpos[:] = original_qpos
                mujoco.mj_forward(self.model, self.data)

        si.setStateValidityChecker(ob.StateValidityCheckerFn(is_state_valid))
        si.setup()

        self.ompl_planner = og.RRT(si)
        # self.ompl_planner = og.RRTConnect(si)
        # self.ompl_planner = og.BITstar(si)        
        # self.ompl_planner = og.RRTstar(si)     
        # self.ompl_planner = og.PRMstar(si)      
        # self.ompl_planner = og.KPIECE1(si)     
        self.ompl_planner.setRange(0.01)
        self.ompl_si = si
        self.ompl_space = space
        print("[OMPL] Initialized")

    def plan_and_execute_trajectory(self, base_goal, qL_goal, qR_goal, max_time=3.0):
        base_curr = self.localization()
        qL_curr, qR_curr = self.get_encoder()
        start = np.concatenate([base_curr, qL_curr, qR_curr])
        goal = np.concatenate([base_goal, qL_goal, qR_goal])

        # Create OMPL states
        start_state = ob.State(self.ompl_space)
        goal_state = ob.State(self.ompl_space)
        for i in range(13):
            start_state[i] = start[i]
            goal_state[i] = goal[i]

        pdef = ob.ProblemDefinition(self.ompl_si)
        pdef.setStartAndGoalStates(start_state, goal_state)
        opt = ob.PathLengthOptimizationObjective(self.ompl_si)
        pdef.setOptimizationObjective(opt)
        self.ompl_planner.setIntermediateStates(True)
        self.ompl_planner.setProblemDefinition(pdef)
        self.ompl_planner.setup()

        print("[OMPL] Planning to WBC goal...")
        solved = self.ompl_planner.solve(max_time)
        if solved:
            approx_info = pdef.getSolutionDifference()
            path = pdef.getSolutionPath()
            path.interpolate(100)
            waypoints = []
            for i in range(path.getStateCount()):
                state = path.getState(i)
                wp = np.array([state[j] for j in range(13)])
                waypoints.append(wp)
            
            # Switch to trajectory mode
            self.ompl_trajectory = waypoints
            self.ompl_traj_idx = 0
            self.use_ompl_trajectory = True
            self._current_traj_qL = None
            self._current_traj_qR = None
            print(f"[OMPL] Trajectory ready ({len(waypoints)} waypoints)")
            print(f"[OMPL] Last WP : {fmt(waypoints[-1])}")
        else:
            print("[OMPL] Failed! Using direct WBC command.")
            with self._target_lock:
                self.target_base = base_goal.copy()
                self.wbc_joint_target_left = qL_goal.copy()
                self.wbc_joint_target_right = qR_goal.copy()
                self.use_wbc_joint_targets = True
                self.use_ik = True
                self.use_ompl_trajectory = False
                self.ompl_trajectory = None

    # =============== Control Logic ===============     
    def control_base(self):
        with self._target_lock:
            target = self.target_base.copy()  

        current_base = self.localization()
        x, y, current_yaw = current_base[0], current_base[1], current_base[2] 

        dt = self.model.opt.timestep

        LIN_VEL_MAX = 1000.0      # m/s
        LIN_ACC_MAX = 100.0      # m/s²
        ROT_VEL_MAX = np.radians(90)   # 90 deg/s
        ROT_ACC_MAX = np.radians(180)  # 180 deg/s²

        if not hasattr(self, '_base_cmd_pos'):
            self._base_cmd_pos = current_base.copy()
            self._base_cmd_vel = np.zeros(3)

        smooth_target = self._base_cmd_pos.copy()
        for i in range(3):
            is_yaw = (i == 2)
            vel_max = ROT_VEL_MAX if is_yaw else LIN_VEL_MAX
            acc_max = ROT_ACC_MAX if is_yaw else LIN_ACC_MAX

            pos_cmd = self._base_cmd_pos[i]
            vel_cmd = self._base_cmd_vel[i]
            target_i = target[i]

            if is_yaw:
                error_raw = (target_i - pos_cmd + np.pi) % (2 * np.pi) - np.pi
            else:
                error_raw = target_i - pos_cmd

            desired_vel = np.clip(2.0 * error_raw, -vel_max, vel_max)
            dv = np.clip(desired_vel - vel_cmd, -acc_max * dt, acc_max * dt)
            new_vel = vel_cmd + dv
            new_vel = np.clip(new_vel, -vel_max, vel_max)
            new_pos = pos_cmd + new_vel * dt

            if is_yaw:
                new_pos = (new_pos + np.pi) % (2 * np.pi) - np.pi

            smooth_target[i] = new_pos
            self._base_cmd_vel[i] = new_vel
            self._base_cmd_pos[i] = new_pos

        x_cmd, y_cmd, yaw_cmd = smooth_target

        dx = x_cmd - x
        dy = y_cmd - y
        dyaw = (yaw_cmd - current_yaw + np.pi) % (2 * np.pi) - np.pi

        c, s = np.cos(current_yaw), np.sin(current_yaw)
        dx_b = c * dx + s * dy
        dy_b = -s * dx + c * dy

        vx_b = self.pid_x.update(target=0.0, current=-dx_b)
        vy_b = self.pid_y.update(target=0.0, current=-dy_b)
        wz   = self.pid_yaw.update(target=0.0, current=-dyaw) 

        wheel_vel_des_fk = mobile_base_ik(vx_b, vy_b, wz, r=R_WHEEL, L=L_DIAGONAL)
        wheel_vel_des_ctrl = np.array([
            wheel_vel_des_fk[1],  # FR → ctrl0
            wheel_vel_des_fk[0],  # FL → ctrl1
            wheel_vel_des_fk[3],  # BR → ctrl2
            wheel_vel_des_fk[2]   # BL → ctrl3
        ])

        wheel_vel_actual_fk = self.data.qvel[self.wheel_joint_indices]
        wheel_vel_actual_ctrl = np.array([
            wheel_vel_actual_fk[1],
            wheel_vel_actual_fk[0],
            wheel_vel_actual_fk[3],
            wheel_vel_actual_fk[2]
        ])

        K_wheel = 3.0
        wheel_torque_cmd = K_wheel * (wheel_vel_des_ctrl - wheel_vel_actual_ctrl)
        # wheel_torque_cmd = np.clip(wheel_torque_cmd, -10.0, 10.0)
        self.data.ctrl[0:4] = wheel_torque_cmd
        body_twist = mobile_base_fk(wheel_vel_actual_fk, r=0.1, L=L_DIAGONAL)
        self.odom_pose = integrate_odometry(self.odom_pose, body_twist, dt)
               
    def control_arms(self):
        q_left, q_right = self.get_encoder()
        dt = self.model.opt.timestep
    
        with self._target_lock:
            u_left = self.direct_arm_commands[0:5].copy()
            u_right = self.direct_arm_commands[5:10].copy()
            
            if self.use_ik:
                if getattr(self, 'use_wbc_joint_targets', False):
                    u_left  = self._current_traj_qL
                    u_right = self._current_traj_qR
                else:
                    u_left = self.manipulator_left.ik(target=self.target_left, q_guess=q_left, pitch_target=self.target_pitch_left)
                    u_right = self.manipulator_right.ik(target=self.target_right, q_guess=q_right, pitch_target=0.0)

        LIN_VEL_MAX = 100.0      # m/s
        LIN_ACC_MAX = 1000.0      # m/s²

        ROT_VEL_MAX = np.radians(180)   # 90 deg/s
        ROT_ACC_MAX = np.radians(270)  # 180 deg/s²

        left_joints = ['h1_left', 'h2_left', 'a1_left', 'theta_left']
        right_joints = ['h1_right', 'h2_right', 'a1_right', 'theta_right']
        all_targets = {
            'h1_left':  u_left[0], 'h2_left': u_left[1], 'a1_left': u_left[2], 'theta_left': u_left[3],
            'h1_right': u_right[0], 'h2_right': u_right[1], 'a1_right': u_right[2], 'theta_right': u_right[3],
        }
        all_current = {
            'h1_left':  q_left[0], 'h2_left': q_left[1], 'a1_left': q_left[2], 'theta_left': q_left[3],
            'h1_right': q_right[0], 'h2_right': q_right[1], 'a1_right': q_right[2], 'theta_right': q_right[3],
        }

        if not hasattr(self, '_cmd_pos'):
            self._cmd_pos = {}
            self._cmd_vel = {}
            for name in list(all_targets.keys()):
                self._cmd_pos[name] = all_current[name]
                self._cmd_vel[name] = 0.0

        smooth_targets = {}
        for name in all_targets:
            target = all_targets[name]
            current_cmd = self._cmd_pos[name]
            current_vel = self._cmd_vel[name]

            if not 'theta' in name:
                VEL_MAX = LIN_VEL_MAX
                ACC_MAX = LIN_ACC_MAX
                wrap = False

            if wrap:
                error_raw = (target - current_cmd + np.pi) % (2 * np.pi) - np.pi
            else:
                error_raw = target - current_cmd

            desired_vel = error_raw
            dv = np.clip(desired_vel - current_vel, -ACC_MAX * dt, ACC_MAX * dt)
            new_vel = current_vel + dv
            # new_vel = np.clip(new_vel, -VEL_MAX, VEL_MAX)
            new_pos = current_cmd + new_vel * dt

            if wrap:
                new_pos = (new_pos + np.pi) % (2 * np.pi) - np.pi

            self._cmd_vel[name] = new_vel
            self._cmd_pos[name] = new_pos
            smooth_targets[name] = new_pos

        cmd = []
        for name in left_joints:
            torque = self.pids[name].update(smooth_targets[name], all_current[name])
            cmd.append(torque)
        cmd.append(u_left[4]) 

        for name in right_joints:
            torque = self.pids[name].update(smooth_targets[name], all_current[name])
            cmd.append(torque)
        cmd.append(u_right[4])
        
        smooth_left = [smooth_targets[name] for name in left_joints]  
        smooth_right = [smooth_targets[name] for name in right_joints]
        
        u_cmd = np.array(cmd, dtype=np.float64)
        self.send_command_arm(u_cmd)
                    
    def _whole_body_control_step(self):
        current_time = self.data.time
        if (current_time - self._last_wbc_update) < self.WBC_UPDATE_DT:
            return

        with self._target_lock:
            left_active = self.wbc_enabled_per_arm["left"] and self.wbc_targets_per_arm["left"] is not None
            right_active = self.wbc_enabled_per_arm["right"] and self.wbc_targets_per_arm["right"] is not None

            if not (left_active or right_active) and self.wbc_enabled and self.wbc_target_arm is not None:
                if self.wbc_arm_side == "left":
                    left_active = True
                    self.wbc_targets_per_arm["left"] = self.wbc_target_arm.copy()
                    self.wbc_pitch_targets_per_arm["left"] = self.wbc_pitch_target
                else:
                    right_active = True
                    self.wbc_targets_per_arm["right"] = self.wbc_target_arm.copy()
                    self.wbc_pitch_targets_per_arm["right"] = self.wbc_pitch_target

            if not (left_active or right_active):
                return

            current_targets = {
                "left": self.wbc_targets_per_arm["left"].copy() if left_active else None,
                "right": self.wbc_targets_per_arm["right"].copy() if right_active else None
            }
            current_pitch = {
                "left": self.wbc_pitch_targets_per_arm["left"],
                "right": self.wbc_pitch_targets_per_arm["right"]
            }

        targets_changed = False
        for arm in ["left", "right"]:
            if current_targets[arm] is not None:
                if (self._last_wbc_targets[arm] is None or 
                    not np.array_equal(current_targets[arm], self._last_wbc_targets[arm]) or
                    current_pitch[arm] != self._last_wbc_pitch[arm]):
                    targets_changed = True
                    break
            elif self._last_wbc_targets[arm] is not None:
                targets_changed = True
                break

        if not targets_changed:
            return  

        self._last_wbc_targets = {
            "left": current_targets["left"].copy() if current_targets["left"] is not None else None,
            "right": current_targets["right"].copy() if current_targets["right"] is not None else None
        }
        self._last_wbc_pitch = current_pitch.copy()

        base_current = self.localization()
        q_left_enc, q_right_enc = self.get_encoder()

        q_left_guess = q_left_enc.copy()
        q_right_guess = q_right_enc.copy()

        try:
            if current_targets["left"] is not None and current_targets["right"] is not None:
                base_sol, q_left_sol, q_right_sol, used_wbc = dual_arm_wbc_collision(
                    base_pose_initial=base_current,
                    left_target=current_targets["left"],
                    right_target=current_targets["right"],
                    q_left_guess=q_left_guess,
                    q_right_guess=q_right_guess,
                    manip_left=self.manipulator_left,
                    manip_right=self.manipulator_right,
                    left_offset=self.arm_offsets["left"],
                    right_offset=self.arm_offsets["right"],
                    joint_names_left=JOINT_NAMES_LEFT,
                    joint_names_right=JOINT_NAMES_RIGHT,
                    pitch_targets=current_pitch,
                    max_iter=100,
                    mujoco_model=self.model,
                    mujoco_data=self.data
                ) # type: ignore
                self.use_wbc_joint_targets = used_wbc
                # self._current_traj_qL = q_left_sol
                # self._current_traj_qR = q_right_sol
                # self.target_base      = base_sol
            
                print(f"[WBC] Target : {fmt(base_sol)} {fmt(q_left_sol)}, {fmt(q_right_sol)}")
                self.plan_and_execute_trajectory(base_sol, q_left_sol, q_right_sol, max_time=10.0)

            else:
                # Single-arm fallback
                arm_side = "left" if current_targets["left"] is not None else "right"
                target = current_targets[arm_side]
                pitch_target = current_pitch[arm_side]
                manip = self.manipulator_left if arm_side == "left" else self.manipulator_right
                q_arm_guess = q_left_guess if arm_side == "left" else q_right_guess
                offset = self.arm_offsets[arm_side]
                
                base_sol, arm_sol, used_wbc = hierarchical_wbc(
                    target_world=target,
                    base_pose=base_current,
                    arm_q_guess=q_arm_guess,
                    manipulator=manip,
                    arm_offset=offset,
                    position_tolerance=0.03,
                    pitch_target=pitch_target
                )
                
                with self._target_lock:
                    self.target_base = base_sol.copy()
                    self.use_ik = True
                    if arm_side == "left":
                        self.target_left = manip.fk(arm_sol)
                        self.target_pitch_left = arm_sol[-1]
                    else:
                        self.target_right = manip.fk(arm_sol)
                
                mode = "SINGLE-ARM WBC" if used_wbc else "SINGLE-ARM IK"

        except Exception as e:
            print(f"[WBC] Dual-arm optimization failed: {str(e)}")
            with self._target_lock:
                if current_targets["left"] is not None and current_targets["right"] is not None:
                    self.wbc_enabled_per_arm["left"] = False
                    self.wbc_enabled_per_arm["right"] = False
                elif current_targets["left"] is not None:
                    self.wbc_enabled_per_arm["left"] = False
                else:
                    self.wbc_enabled_per_arm["right"] = False
                self.use_ik = self.wbc_enabled_per_arm["left"] or self.wbc_enabled_per_arm["right"]

        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_yaw.reset()
        
        self._last_wbc_update = current_time
        
    def get_pov_pointcloud(self, depth_trunc: float = 5.0) -> np.ndarray:
        H, W = 480, 640
        fovy = self.model.cam_fovy[self.pov_camera_id]
        theta = np.deg2rad(fovy)
        
        f = H / (2 * np.tan(theta / 2))
        intr = np.array([
            [f, 0, (W - 1) / 2.0],
            [0, f, (H - 1) / 2.0],
            [0, 0, 1]
        ])

        cam_pos = self.data.cam_xpos[self.pov_camera_id]
        cam_rot = self.data.cam_xmat[self.pov_camera_id].reshape(3, 3) 
        
        extr = np.eye(4)
        extr[:3, :3] = cam_rot  
        extr[:3, 3] = cam_pos   

        self.pov_renderer.update_scene(self.data, camera=self.pov_camera_id)
        self.pov_renderer.enable_depth_rendering()
        depth = self.pov_renderer.render()
        self.pov_renderer.disable_depth_rendering()
        rgb = self.pov_renderer.render()

        def rgbd_to_world_pc(rgb, depth, intr, extr, depth_trunc):
            h, w = depth.shape
            c, r = np.meshgrid(np.arange(w), np.arange(h))
            
            mask = (depth > 0) & (depth < depth_trunc)
            z = depth[mask]
            u = c[mask]
            v = r[mask]
            x_cam = (u - intr[0, 2]) * z / intr[0, 0]
            y_cam = (v - intr[1, 2]) * z / intr[1, 1]
            points_cam = np.vstack([x_cam, -y_cam, -z, np.ones_like(z)])
            
            points_world = (extr @ points_cam).T
            
            colors = rgb[mask] / 255.0
            return np.hstack([points_world[:, :3], colors])

        if self.run_mode != "glfw":
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            cv2.imshow("MuJoCo POV", bgr)
            cv2.waitKey(1)

        return rgbd_to_world_pc(rgb, depth, intr, extr, depth_trunc)
    
    def step_simulation(self, render=True):
        # self.get_keyframe("home")
        current_time = self.data.time
        q_left, q_right = self.get_encoder()
        current_base    = self.localization()
        
        # self.debug_state_validity()
        self.control_arms()
        self.control_base()

        if (current_time - self._last_wbc_update) >= self.WBC_UPDATE_DT:
            self._whole_body_control_step()
            
        if self.use_ompl_trajectory and self.ompl_trajectory is not None:
            if self.ompl_traj_idx < len(self.ompl_trajectory):
                wp = self.ompl_trajectory[self.ompl_traj_idx]
                base_target = wp[0:3]
                qL_wp = wp[3:8]
                qR_wp = wp[8:13]
                
                self.target_base = base_target
                self._current_traj_qL = qL_wp
                self._current_traj_qR = qR_wp
                
                # adr = self.get_joint_qpos_addr('base_footprint')
                # self.data.qpos[adr + 0] = base_target[0]
                # self.data.qpos[adr + 1] = base_target[1]
                # cy = np.cos(base_target[2] * 0.5)
                # sy = np.sin(base_target[2] * 0.5)
                # self.data.qpos[adr + 3] = cy
                # self.data.qpos[adr + 4] = 0.0
                # self.data.qpos[adr + 5] = 0.0
                # self.data.qpos[adr + 6] = sy
                
                qB_error = np.linalg.norm(base_target[:2] - current_base[:2])
                qL_error = np.linalg.norm(q_left  - self._current_traj_qL)
                qR_error = np.linalg.norm(q_right - self._current_traj_qR)
                self.use_ik = True 
                # print(f"[OMPL] Error : {fmt([qB_error])} {fmt([qL_error])} {fmt([qR_error])}")
                
                POSITION_TOLERANCE = 0.02  # 2 cm / ~1 degree tolerance
                BASE_TOLERANCE = 0.01
                if qB_error < BASE_TOLERANCE and qL_error < POSITION_TOLERANCE and qR_error < POSITION_TOLERANCE:
                    self.ompl_traj_idx += 1
                    if self.ompl_traj_idx < len(self.ompl_trajectory): pass
                        # print(f"[OMPL] Reached waypoint {self.ompl_traj_idx-1}, moving to waypoint {self.ompl_traj_idx}")
                    else:
                        print(f"[OMPL] Reached final waypoint {self.ompl_traj_idx-1}")
            elif self.ompl_traj_idx >= len(self.ompl_trajectory):
                base_target = self.ompl_trajectory[-1][0:3]
                qL_wp = self.ompl_trajectory[-1][3:8]
                qR_wp = self.ompl_trajectory[-1][8:13]
                self.target_base = base_target
                self._current_traj_qL = qL_wp
                self._current_traj_qR = qR_wp
                adr = self.get_joint_qpos_addr('base_footprint')
                self.data.qpos[adr + 0] = base_target[0]
                self.data.qpos[adr + 1] = base_target[1]
                cy = np.cos(base_target[2] * 0.5)
                sy = np.sin(base_target[2] * 0.5)
                self.data.qpos[adr + 3] = cy
                self.data.qpos[adr + 4] = 0.0
                self.data.qpos[adr + 5] = 0.0
                self.data.qpos[adr + 6] = sy
        
        pos_left_gt = self.data.site_xpos[self.gt_left_id]
        pos_left_tgt = self.data.site_xpos[self.tgt_left_id]

        ee_left_world = world_ee_pose(
            base_pose=self.localization(),
            q_arm=q_left,
            arm_offset=self.arm_offsets["left"],
            manipulator=self.manipulator_left
        )

        ee_left_frame = self.manipulator_left.fk(q_left)

        jnt1_id = self.model.joint("BaseJoint_1").id
        qpos1 = self.data.qpos[self.model.jnt_qposadr[jnt1_id]]
        pos_right_gt = self.data.site_xpos[self.gt_right_id]
        pos_right_tgt = self.data.site_xpos[self.tgt_right_id]

        ee_right_world = world_ee_pose(
            base_pose=self.localization(),
            q_arm=q_right,
            arm_offset=self.arm_offsets["right"],
            manipulator=self.manipulator_right
        )

        ee_right_frame = self.manipulator_right.fk(q_right)
        
        # print("\n🟦 LEFT ARM")
        # print(f"  GT  WORLD  (site)  : {fmt(pos_left_gt)}")
        # print(f"  TGT WORLD  (site)  : {fmt(pos_left_tgt)}")
        # print(f"  FK  WORLD  (calc)  : {fmt(ee_left_world)}")

        # print("\n🟥 RIGHT ARM")
        # print(f"  GT  WORLD  (site)  : {fmt(pos_right_gt)}")
        # print(f"  TGT WORLD  (site)  : {fmt(pos_right_tgt)}")
        # print(f"  FK  WORLD  (calc)  : {fmt(ee_right_world)}")
        
        print("\n BASE")
        print(f"  CURR WORLD  (site) : {fmt(self.localization())}")
        print(f"  TGT  WORLD  (site) : {fmt(self.target_base)}")
        
        mujoco.mj_step(self.model, self.data, nstep=3)
        cur_pos = self.localization(full=True)
        self.pcd_publisher.publish_array("localization", cur_pos)
        current_time = self.data.time
        
        if self.run_mode == "glfw":
            mujoco.mjv_updateScene(self.model, self.data, self.opt, None, self.camera, 0xFFFF, self.scene)
            glfw.swap_buffers(self.window)
            glfw.poll_events()
        else:
            if not hasattr(self, '_last_pcd_pub_time'):
                self._last_pcd_pub_time = 0.0
            if current_time - self._last_pcd_pub_time > 0.1:  # 5 Hz
                pcd = self.get_pov_pointcloud(depth_trunc=3.0)
                self.pcd_publisher.publish_array("/sensor/pointcloud", pcd)
                self._last_pcd_pub_time = current_time
            
    def run_glfw(self):
        mujoco.mj_step(self.model, self.data, nstep=1)
        while not glfw.window_should_close(self.window) and not self._terminate:
            self.step_simulation()
            mujoco.mjv_updateScene(self.model, self.data, self.opt, None, self.camera, mujoco.mjtCatBit.mjCAT_ALL, self.scene)
            mujoco.mjr_render(self.viewport, self.scene, self.ctx)
        glfw.terminate()

    def run_cv(self):
        mujoco.mj_step(self.model, self.data, nstep=1)
        MOVE_STEP = 0.03
        YAW_STEP = np.radians(5.0)  # Optional: Q/E for rotation
        
        try:
            while not self._terminate:
                self.step_simulation(render=False)
                self.renderer_top.update_scene(self.data, self.camera)
                rgb = self.renderer_top.render()
                if rgb is not None and rgb.size > 0:
                    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                    if self.video_recorder:
                        self.video_recorder.write_frame(bgr)
                    cv2.imshow("MuJoCo Top View", bgr)
                    
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('t'):
                        self._terminate = True
                    elif key == ord('w'):  # Forward (+Y)
                        with self._target_lock:
                            self.target_base[1] += MOVE_STEP
                    elif key == ord('s'):  # Backward (-Y)
                        with self._target_lock:
                            self.target_base[1] -= MOVE_STEP
                    elif key == ord('a'):  # Left (-X)
                        with self._target_lock:
                            self.target_base[0] -= MOVE_STEP
                    elif key == ord('d'):  # Right (+X)
                        with self._target_lock:
                            self.target_base[0] += MOVE_STEP
                    elif key == ord('q'):
                        with self._target_lock:
                            self.target_base[2] -= YAW_STEP
                    elif key == ord('e'):
                        with self._target_lock:
                            self.target_base[2] += YAW_STEP
                    elif key == ord('r'):
                        with self._target_lock:
                            self.target_base[:2] = self.localization()[:2]
        finally:
            if self.video_recorder:
                self.video_recorder.release()
            cv2.destroyAllWindows()
            self.renderer_top.close()