import threading
import numpy as np
import mujoco
from typing import Optional

from config import SimulationConfig
from rendering.viewer_base import Viewer
from data.base import DataSource
from geometry.kinematics import KinematicsSolver
from geometry.transform import StaticTransform
from rendering.visualizer import LandmarkVisualizer
from core.controllers import BaseJointPIDController, MobileBaseController, GripperController
from utils.logger import setup_logger

from core.profiles.base import MultiDOFProfile
from core.profiles.trapezoidal import TrapezoidalProfile
from core.profiles.s_curve import SCurveProfile
from core.profiles.sinusoidal import SinusoidalProfile
from core.profiles.exponential import ExponentialProfile

from core.task_manager import TaskManager
from control.keyboard_controller import KeyboardController
from planning.task_definitions import build_pick_and_place_sequence

logger = setup_logger("ParallelRobot")

try:
    from modules.pubsub import IPCPubSub
except ImportError:
    class IPCPubSub:
        def create_subscriber(self): return self
        def subscribe(self, *args): pass
        def start(self): pass

class ParallelRobot:
    DAMPING = 8e-4; DT = 0.002
    JOINT_NAMES = ["ColumnLeftBearingJoint_1", "ColumnRightBearingJoint_1", "ArmLeftJoint_1", "BaseJoint_1", "ColumnLeftBearingJoint_2", "ColumnRightBearingJoint_2", "ArmLeftJoint_2", "BaseJoint_2"]
    ACTUATOR_NAMES = ["ColumnLeftBearingJointMotor_1", "ColumnRightBearingJointMotor_1", "ArmLeftJointMotor_1", "BaseJointMotor_1", "ColumnLeftBearingJointMotor_2", "ColumnRightBearingJointMotor_2", "ArmLeftJointMotor_2", "BaseJointMotor_2"]
    GRIPPER_ACT_LEFT = ["finger_c_joint_1_1", "finger_c_joint_2_1", "finger_c_joint_3_1", "finger_b_joint_1_1", "finger_b_joint_2_1", "finger_b_joint_3_1", "finger_a_joint_1_1", "finger_a_joint_2_1", "finger_a_joint_3_1", "palm_finger_c_joint_1", "palm_finger_b_joint_1", "wrist_X_1", "wrist_Y_1", "wrist_Z_1", "HandBearing_1"]
    GRIPPER_ACT_RIGHT = ["finger_c_joint_1_2", "finger_c_joint_2_2", "finger_c_joint_3_2", "finger_b_joint_1_2", "finger_b_joint_2_2", "finger_b_joint_3_2", "finger_a_joint_1_2", "finger_a_joint_2_2", "finger_a_joint_3_2", "palm_finger_c_joint_2", "palm_finger_b_joint_2", "wrist_X_2", "wrist_Y_2", "wrist_Z_2", "HandBearing_2"]

    def __init__(self, config: SimulationConfig, viewer: Viewer, data_source: DataSource, kinematics: KinematicsSolver):
        self.config = config; self.viewer = viewer; self.data_source = data_source; self.kinematics = kinematics
        self.model = mujoco.MjModel.from_xml_path(config.xml_path)
        self.data = mujoco.MjData(self.model)
        self.configure_model(); self.reset("home")
        
        self._target_lock = threading.Lock()
        self._initialize_ids(); self._initialize_arrays(); self._calibrate_arm_offsets()
        
        self._terminate = False
        self.current_ctrl = np.zeros(len(self.ACTUATOR_NAMES))
        self.camera = mujoco.MjvCamera()
        self.camera.distance = 5.0
        self.camera.azimuth = 90
        self.camera.elevation = -45
        self.camera.lookat[:] = [0, 0, 0]
        self.camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        self.scene = mujoco.MjvScene(self.model, maxgeom=10000); self.opt = mujoco.MjvOption()
        
        self.use_ik = False 
        self.direct_arm_commands = np.concatenate([self.data.ctrl[self.actuator_ids[0:3]]/100, [0], self.data.ctrl[self.actuator_ids[4:7]]/100, [0]])

        self.ipc = IPCPubSub(); self.subscriber = self.ipc.create_subscriber()
        self.subscriber.subscribe("target_base", self._on_target_base)
        self.subscriber.subscribe("target_left", self._on_target_left)
        self.subscriber.subscribe("target_right", self._on_target_right)
        self.subscriber.subscribe("ik_mode", self._on_ik_mode)
        self.subscriber.subscribe("u_control", self._on_arm_control)
        self.subscriber.start()
        
        if config.control_mode == "landmark":
            self.landmark_viz = LandmarkVisualizer(
                data_source=self.data_source,
                transform=StaticTransform(x=0.3, y=0.0, z=1.2, yaw=3.14159265)
            )
        else:
            self.landmark_viz = None     

        # Controllers
        self.base_joint_pid = BaseJointPIDController()
        self.mobile_base_ctrl = MobileBaseController(r=0.1, D=0.55)
        self.gripper_ctrl = GripperController()

        pc = config.motion_profile
        def make_profile():
            if pc.profile_type == "trapezoidal":
                return TrapezoidalProfile(max_vel=pc.max_vel, max_accel=pc.max_accel)
            elif pc.profile_type == "s_curve":
                return SCurveProfile(max_vel=pc.max_vel, max_accel=pc.max_accel, max_jerk=pc.max_jerk)
            elif pc.profile_type == "sinusoidal":
                return SinusoidalProfile(max_vel=pc.max_vel)
            elif pc.profile_type == "exponential":
                return ExponentialProfile(alpha=pc.alpha)
            else:
                logger.warning(f"Unknown profile '{pc.profile_type}', falling back to exponential.")
                return ExponentialProfile(alpha=pc.alpha)
            
        self.arm_profile_left = MultiDOFProfile(make_profile, num_dof=3)
        self.arm_profile_right = MultiDOFProfile(make_profile, num_dof=3)
        self.arm_profile_left.reset(self.data.ctrl[self.actuator_ids[0:3]])
        self.arm_profile_right.reset(self.data.ctrl[self.actuator_ids[4:7]])
        
        q_left, q_right = self.get_encoder()
        self.target_left = np.array(self.kinematics.forward(*q_left))
        self.target_right = np.array(self.kinematics.forward(*q_right))
        self.prev_target_left = self.target_left.copy()
        self.prev_target_right = self.target_right.copy()
        self.target_base = self.localization()
        self.target_left_global = None; self.target_right_global = None

        self.task_manager = None
        self.keyboard_controller = None

        if config.control_mode == "trajectory":
            self.task_manager = TaskManager(self)
        elif config.control_mode == "keyboard":
            self.keyboard_controller = KeyboardController(self)
            KeyboardController.print_controls()


    def _initialize_ids(self):
        self.arm_link_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "Arm")
        self.end_effector_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "Gripper_Link1")
        self.base_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "base_footprint")
        self.ee_site_1_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "ee_site_1")
        self.ee_site_2_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "ee_site_2")
        self.dof_ids = np.array([self.model.joint(name).id for name in self.JOINT_NAMES])
        self.actuator_ids = np.array([self.model.actuator(name).id for name in self.ACTUATOR_NAMES])
        self.gripper_ids_left = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name) for name in self.GRIPPER_ACT_LEFT]
        self.gripper_ids_right = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name) for name in self.GRIPPER_ACT_RIGHT]
        
        self.qpos_indices = []; self.qvel_indices = []
        for name in self.JOINT_NAMES:
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            qpos_adr = self.model.jnt_qposadr[joint_id]; dof_adr = self.model.jnt_dofadr[joint_id]; joint_type = self.model.jnt_type[joint_id]
            if joint_type in [3, 2]: self.qpos_indices.append(qpos_adr); self.qvel_indices.append(dof_adr)
            elif joint_type == 1: self.qpos_indices.extend(range(qpos_adr, qpos_adr + 3)); self.qvel_indices.extend(range(dof_adr, dof_adr + 3))
            elif joint_type == 0: self.qpos_indices.extend(range(qpos_adr, qpos_adr + 6)); self.qvel_indices.extend(range(dof_adr, dof_adr + 6))
        self.qpos_indices = np.array(self.qpos_indices); self.qvel_indices = np.array(self.qvel_indices)

    def _initialize_arrays(self):
        self.jacp = np.zeros((3, self.model.nv)); self.jacr = np.zeros((3, self.model.nv))
        self.error = np.zeros(6); self.error_pos = np.zeros(3); self.error_ori = np.zeros(3)
        self.site_quat = np.zeros(4); self.site_quat_conj = np.zeros(4); self.error_quat = np.zeros(4)

    def handle_key_press(self, key: int) -> None:
        if self.keyboard_controller is not None:
            self.keyboard_controller.handle_key(key)

    def configure_model(self):
        self.model.opt.timestep = self.DT; self.model.body_gravcomp[:] = True

    def _calibrate_arm_offsets(self):
        try:
            base_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "base_footprint")
            arm_l_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "Arm_1")
            arm_r_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "Arm_2")
            self.reset("home"); mujoco.mj_step(self.model, self.data, nstep=1)
            pos_base = self.data.xpos[base_id]; pos_l = self.data.xpos[arm_l_id]; pos_r = self.data.xpos[arm_r_id]
            ik_arm_offset_left = (pos_l - pos_base); ik_arm_offset_left[2] += 0.0543365
            ik_arm_offset_right = (pos_r - pos_base); ik_arm_offset_right[2] += 0.0543365; ik_arm_offset_right[1] -= 0.01
            self.kinematics.ik_arm_offset_left = ik_arm_offset_left; self.kinematics.ik_arm_offset_right = ik_arm_offset_right
            logger.info(f"Auto-calibrated IK offsets: L={ik_arm_offset_left.round(4)}, R={ik_arm_offset_right.round(4)}")
        except Exception as e:
            logger.warning(f"Could not auto-calibrate arm offsets: {e}")
            self.kinematics.ik_arm_offset_left = np.array([0.16, 0.15 - 0.00465966, 0.158566 + 0.127 + 0.0542705])
            self.kinematics.ik_arm_offset_right = np.array([0.16, -(0.16 - 0.00465966), 0.158566 + 0.127 + 0.0896705])

    def reset(self, keyframe_name: str):
        key_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, keyframe_name)
        mujoco.mj_resetDataKeyframe(self.model, self.data, key_id)
        
    def localization(self) -> np.ndarray:
        x, y = self.data.xpos[self.base_id, 0], self.data.xpos[self.base_id, 1]
        w, xq, yq, zq = self.data.xquat[self.base_id]
        yaw = np.arctan2(2 * (w * zq + xq * yq), 1 - 2 * (yq**2 + zq**2))
        return np.array([x, y, yaw])
    
    def get_encoder(self):
        def get_addr(name): 
            return self.model.jnt_qposadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)]
        z1_l = self.data.qpos[get_addr("ColumnLeftBearingJoint_1")]
        z1_r = self.data.qpos[get_addr("ColumnRightBearingJoint_1")]
        z2_l = self.data.qpos[get_addr("ColumnLeftBearingJoint_2")]
        z2_r = self.data.qpos[get_addr("ColumnRightBearingJoint_2")]

        h_l = self.data.qpos[get_addr("ArmLeftJoint_1")]
        h_r = self.data.qpos[get_addr("ArmLeftJoint_2")]
        y_l = self.data.qpos[get_addr("BaseJoint_1")]
        y_r = self.data.qpos[get_addr("BaseJoint_2")]
        return np.array([z1_l, z1_r, h_l, y_l]), np.array([z2_l, z2_r, h_r, y_r])

    def send_command_arm(self, u_control: np.ndarray):
        ctrl_ranges = self.model.actuator_ctrlrange[self.actuator_ids]  
        u_clipped = np.clip(u_control, ctrl_ranges[:, 0], ctrl_ranges[:, 1])
        self.data.ctrl[self.actuator_ids] = u_clipped

    def control_base(self, target: np.ndarray, alpha=0.0):
        wheel_cmds = self.mobile_base_ctrl.compute_wheel_commands(self.model, self.data, self.base_id, target, alpha)
        self.data.ctrl[0:4] = wheel_cmds

    def control_arms(self):
        q_left, q_right = self.get_encoder()        
        current_u_left = self.data.ctrl[self.actuator_ids[0:3]]
        current_u_right = self.data.ctrl[self.actuator_ids[4:7]]
        
        # Get targets based on control mode
        if self.landmark_viz is not None:
            hand_a, hand_b, a_present, b_present, a_grab, b_grab = self.landmark_viz.get_current_targets()
        else:
            hand_a = self.target_left_global if hasattr(self, 'target_left_global') and self.target_left_global is not None else None
            hand_b = self.target_right_global if hasattr(self, 'target_right_global') and self.target_right_global is not None else None
            a_present = hand_a is not None
            b_present = hand_b is not None
            a_grab = False
            b_grab = False

        # Assignment logic (closest or fixed mode)
        if self.config.target_mode == "closest":
            arm_l_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "Arm_1")
            arm_r_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "Arm_2")
            pos_l = self.data.xpos[arm_l_id]
            pos_r = self.data.xpos[arm_r_id]
            
            if a_present and b_present:
                dist_a_l = np.linalg.norm(np.array(hand_a) - pos_l)
                dist_a_r = np.linalg.norm(np.array(hand_a) - pos_r)
                dist_b_l = np.linalg.norm(np.array(hand_b) - pos_l)
                dist_b_r = np.linalg.norm(np.array(hand_b) - pos_r)
                
                if (dist_a_l + dist_b_r) <= (dist_a_r + dist_b_l):
                    left_target_global, right_target_global = hand_a, hand_b
                    left_is_grab, right_is_grab = a_grab, b_grab
                else:
                    left_target_global, right_target_global = hand_b, hand_a
                    left_is_grab, right_is_grab = b_grab, a_grab
                left_present = right_present = True
                
            elif a_present:
                if np.linalg.norm(np.array(hand_a) - pos_l) <= np.linalg.norm(np.array(hand_a) - pos_r):
                    left_target_global, left_is_grab, left_present = hand_a, a_grab, True
                    right_target_global, right_is_grab, right_present = None, False, False
                else:
                    right_target_global, right_is_grab, right_present = hand_a, a_grab, True
                    left_target_global, left_is_grab, left_present = None, False, False
                    
            elif b_present:
                if np.linalg.norm(np.array(hand_b) - pos_l) <= np.linalg.norm(np.array(hand_b) - pos_r):
                    left_target_global, left_is_grab, left_present = hand_b, b_grab, True
                    right_target_global, right_is_grab, right_present = None, False, False
                else:
                    right_target_global, right_is_grab, right_present = hand_b, b_grab, True
                    left_target_global, left_is_grab, left_present = None, False, False
            else:
                left_target_global = right_target_global = None
                left_present = right_present = False
                left_is_grab = right_is_grab = False
        else:
            left_target_global, right_target_global = hand_a, hand_b
            left_present, right_present = a_present, b_present
            left_is_grab, right_is_grab = a_grab, b_grab

        raw_cmd_L = current_u_left.copy()
        raw_cmd_R = current_u_right.copy()
        
        # Initialize stored theta values if not present
        if not hasattr(self, 'prev_theta_left'):
            self.prev_theta_left = q_left[3]
        if not hasattr(self, 'prev_theta_right'):
            self.prev_theta_right = q_right[3]
        
        target_theta_left = self.prev_theta_left
        target_theta_right = self.prev_theta_right
        
        ee1_pos = self.data.site_xpos[self.ee_site_1_id] if self.ee_site_1_id != -1 else None
        ee2_pos = self.data.site_xpos[self.ee_site_2_id] if self.ee_site_2_id != -1 else None
        
        self.use_ik = False
        base_pose = self.localization()
        dt = self.model.opt.timestep * 5
        
        offset = np.array([-0.0036, -0.0062, -0.0006])

        # LEFT ARM IK
        if left_present and left_target_global is not None:
            self.target_left_global = np.array(left_target_global)
            ik_sol_left = self.kinematics.inverse(
                target=self.target_left_global,
                arm="left",
                current_encoders=q_left,
                base_pose=base_pose,
                coord="global"
            )
            if ik_sol_left is not None:
                raw_cmd_L = (ik_sol_left[:3] + offset) * 100
                target_theta_left = ik_sol_left[3]
                self.prev_theta_left = target_theta_left  # Store for next iteration
                self.use_ik = True
                logger.debug(f"[TARGET L] : {self.target_left_global.round(3)} || [CURRENT L] : {ee1_pos.round(3) if ee1_pos is not None else 'N/A'}")
            self.prev_target_left = self.target_left_global.copy()
        else:
            # No new target - use stored theta to hold position
            target_theta_left = self.prev_theta_left

        # RIGHT ARM IK
        if right_present and right_target_global is not None:
            self.target_right_global = np.array(right_target_global)
            ik_sol_right = self.kinematics.inverse(
                target=self.target_right_global,
                arm="right",
                current_encoders=q_right,
                base_pose=base_pose,
                coord="global"
            )
            if ik_sol_right is not None:
                raw_cmd_R = (ik_sol_right[:3] + offset) * 100
                target_theta_right = ik_sol_right[3]
                self.prev_theta_right = target_theta_right  # Store for next iteration
                self.use_ik = True
                logger.debug(f"[TARGET R] : {self.target_right_global.round(3)} || [CURRENT R] : {ee2_pos.round(3) if ee2_pos is not None else 'N/A'}")
            self.prev_target_right = self.target_right_global.copy()
        else:
            # No new target - use stored theta to hold position
            target_theta_right = self.prev_theta_right

        # Apply motion profiles
        smooth_cmd_L = self.arm_profile_left.step(raw_cmd_L, dt)
        smooth_cmd_R = self.arm_profile_right.step(raw_cmd_R, dt)
        
        # Base joint PID with FIXED targets
        u_base_left, u_base_right = self.base_joint_pid.compute_torques(
            self.model, self.data, target_theta_left, target_theta_right
        )
        
        u_cmd = np.concatenate([smooth_cmd_L, [u_base_left], smooth_cmd_R, [u_base_right]])
        self.send_command_arm(u_cmd)
        self.gripper_ctrl.apply_commands(
            self.data, self.gripper_ids_left, self.gripper_ids_right, 
            left_is_grab, right_is_grab
        )
        
    def step_simulation(self):
        current_time = self.data.time

        if self.task_manager is not None and not self.task_manager.is_active:
            initial_base = self.localization()
            task_config = self.config.task_config
            pick_pos = np.array(task_config.pick_position)
            drop_pos = np.array(task_config.drop_position)
            sequence = build_pick_and_place_sequence(initial_base, pick_pos, drop_pos)
            self.task_manager.set_sequence(sequence, current_time)

        if self.task_manager is not None and self.task_manager.is_active:
            self.task_manager.update(current_time)

        if self.landmark_viz is not None:
            self.landmark_viz.step()

        self.control_base(target=self.target_base, alpha=0.1)
        self.control_arms()

        mujoco.mj_step(self.model, self.data, nstep=5)

    def _add_target_sphere(self, scene, pos, radius=0.025, color=(1.0, 0.0, 0.0, 0.6)):
        if scene.ngeom >= scene.maxgeom - 50: return
        pos_arr = np.array(pos, dtype=np.float32)
        if np.any(np.isnan(pos_arr)): return
        geom = scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(geom, type=mujoco.mjtGeom.mjGEOM_SPHERE, size=np.array([radius, radius, radius], dtype=np.float32), pos=pos_arr, mat=np.eye(3, dtype=np.float32).flatten(), rgba=np.array(color, dtype=np.float32))
        scene.ngeom += 1

    def render_target_spheres(self, scene: mujoco.MjvScene):
        if self.target_left_global is not None: self._add_target_sphere(scene, self.target_left_global, radius=0.025, color=(1.0, 0.0, 0.0, 0.6))
        if self.target_right_global is not None: self._add_target_sphere(scene, self.target_right_global, radius=0.025, color=(0.0, 0.5, 1.0, 0.6))

    def _on_ik_mode(self, msg): 
        try: self.use_ik = bool(msg); logger.info(f"IK mode: {'ENABLED' if self.use_ik else 'DISABLED'}")
        except Exception as e: logger.error(f"Invalid ik_mode message: {e}")

    def _on_arm_control(self, msg):
        try:
            raw = np.array(msg, dtype=float)
            if raw.shape != (8,): raise ValueError(f"Expected 8 values, got {raw.size}")
            def remap(v, i1, i2, o1, o2): return o1 + (v - i1) * (o2 - o1) / (i2 - i1)
            mapped = np.array([
                np.clip(remap(raw[0], -75, 75, 0, 1.5), 0, 1.5), np.clip(remap(raw[1], -75, 75, 0, 1.5), 0, 1.5),
                np.clip(remap(raw[2], -30, 30, 0, 0.6), 0, 0.6), np.deg2rad(raw[3]),
                np.clip(remap(raw[4], -75, 75, 0, 1.5), 0, 1.5), np.clip(remap(raw[5], -75, 75, 0, 1.5), 0, 1.5),
                np.clip(remap(raw[6], -30, 30, 0, 0.6), 0, 0.6), np.deg2rad(raw[7])
            ])
            with self._target_lock: self.direct_arm_commands = mapped.copy()
        except Exception as e: logger.error(f"Invalid u_control message: {e}")

    def _on_target_base(self, msg):
        try:
            arr = np.array(msg, dtype=float)
            if arr.shape != (3,): raise ValueError()
            with self._target_lock: self.target_base = arr.copy()
        except Exception as e: logger.error(f"Invalid target_base message: {e}")

    def _on_target_left(self, msg):
        try:
            arr = np.array(msg, dtype=float)
            if arr.shape != (3,): raise ValueError()
            with self._target_lock: self.target_left = arr.copy()
        except Exception as e: logger.error(f"Invalid target_left message: {e}")

    def _on_target_right(self, msg):
        try:
            arr = np.array(msg, dtype=float)
            if arr.shape != (3,): raise ValueError()
            with self._target_lock: self.target_right = arr.copy()
        except Exception as e: logger.error(f"Invalid target_right message: {e}")

    def run(self):
        self.viewer.setup(self.model, self.camera, self.scene, self.opt)
        self.viewer.set_reset_callback(lambda: self.reset("home"))
        if self.keyboard_controller is not None:
            self.viewer.set_key_callback(self.handle_key_press)
        mujoco.mj_step(self.model, self.data, nstep=1)
        try:
            while not self.viewer.should_close() and not self._terminate:
                self.step_simulation()
                scene = self.viewer.prepare_scene(self.data)
                
                if self.landmark_viz is not None:
                    self.landmark_viz.render(scene)
                    self.render_target_spheres(scene)
                
                self.viewer.render_frame(scene)
        except Exception as e:
            logger.error(f"Simulation error: {e}")
        finally:
            self.viewer.close()