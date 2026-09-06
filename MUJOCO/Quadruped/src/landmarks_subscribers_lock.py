from pathlib import Path
import threading
from typing import Optional, Dict, List, Tuple
from dataclasses import dataclass, field
import mujoco
import numpy as np
from loop_rate_limiters import RateLimiter
import babyros
from geometry.transform import StaticTransform, AnchoredTransform
import mink
import glfw
from scipy.spatial.transform import Rotation
from rendering.glfw_viewer import GlfwViewer

@dataclass
class Config:
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
        "primary_column_lift":    8.5,
        "secondary_column_lift":  8.5,
        "telescopic_extend":      8.5,
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
    human_z_max: List[float] = field(default_factory=lambda: [ 0.250,  0.250])

    human_x_mid: List[float] = field(default_factory=lambda: [0.0, 0.0])
    human_y_mid: List[float] = field(default_factory=lambda: [0.0, 0.0])
    human_z_mid: List[float] = field(default_factory=lambda: [0.975, 0.975])
    
    robot_x_min: List[float] = field(default_factory=lambda: [-0.60, -0.60])
    robot_x_max: List[float] = field(default_factory=lambda: [ 0.60,  0.60])
    robot_y_min: List[float] = field(default_factory=lambda: [-0.60, -0.60])
    robot_y_max: List[float] = field(default_factory=lambda: [ 0.60,  0.60])
    robot_z_min: List[float] = field(default_factory=lambda: [-0.625, -0.625])
    robot_z_max: List[float] = field(default_factory=lambda: [ 0.675,  0.675])
    robot_x_mid: List[float] = field(default_factory=lambda: [0.0, 0.0])
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

class RobotInterface:
    def __init__(self, config: Config):
        self.config = config
        self.model = mujoco.MjModel.from_xml_path(config.xml_path.as_posix())
        self.data = mujoco.MjData(self.model)
        self.configuration = mink.Configuration(self.model)
        
        self.motor_ctrl_ids: np.ndarray = np.array([])
        self.motor_qpos_adrs: np.ndarray = np.array([])
        self.finger_ctrl_ids: Dict = {}
        self.finger_qpos_adrs: Dict = {}
        self.finger_body_ids: Dict = {
            "left": {"a": [], "b": [], "c": []},
            "right": {"a": [], "b": [], "c": []}
        }
        
        self.finger_close_targets = {"joint1": 0.33, "joint2": 0.0, "joint3": 0.0}
        self.finger_open_targets = {"joint1": 0.0, "joint2": 0.0, "joint3": 0.0}
        
        self.tasks: List = []
        self.limits: List = []
        self.left_end_effector_task = None
        self.right_end_effector_task = None
        self.posture_task = None
        
        self.left_mocap_id = self.model.body("left_ik_target").mocapid[0]
        self.right_mocap_id = self.model.body("right_ik_target").mocapid[0]
        
        self._setup_motors()
        self._setup_fingers()
        self._setup_finger_body_ids()
        self._setup_tasks_and_limits()
    
    def _setup_motors(self):
        motor_joint_names = [
            f"{self.config.robot_prefix}_{arm}_{motor}"
            for arm in self.config.arm_prefixes
            for motor in self.config.arm_motors
        ]
        ctrl_ids = []
        qpos_adrs = []
        for name in motor_joint_names:
            try:
                jnt_id = self.model.joint(name).id
            except KeyError:
                continue
            actuator_indices = np.where(self.model.actuator_trnid[:, 0] == jnt_id)[0]
            if len(actuator_indices) > 0:
                ctrl_ids.append(actuator_indices[0])
                qpos_adrs.append(self.model.jnt_qposadr[jnt_id])
        self.motor_ctrl_ids = np.array(ctrl_ids)
        self.motor_qpos_adrs = np.array(qpos_adrs)
    
    def _setup_fingers(self):
        self.finger_ctrl_ids = {
            side: {letter: {"joint1": [], "joint2": [], "joint3": []} for letter in "abc"}
            for side in ("left", "right")
        }
        self.finger_qpos_adrs = {
            side: {letter: {"joint1": [], "joint2": [], "joint3": []} for letter in "abc"}
            for side in ("left", "right")
        }
        
        for i in range(self.model.nu):
            jnt_id = self.model.actuator_trnid[i, 0]
            jnt_name = self.model.joint(jnt_id).name
            if "finger" not in jnt_name:
                continue
            arm_side = "left" if "left" in jnt_name else "right"
            qpos_adr = self.model.jnt_qposadr[jnt_id]
            
            if "finger_a" in jnt_name:
                letter = "a"
            elif "finger_b" in jnt_name:
                letter = "b"
            elif "finger_c" in jnt_name:
                letter = "c"
            else:
                continue
            
            if jnt_name.endswith("joint1"):
                key = "joint1"
            elif jnt_name.endswith("joint2"):
                key = "joint2"
            elif jnt_name.endswith("joint3"):
                key = "joint3"
            else:
                continue
            
            self.finger_ctrl_ids[arm_side][letter][key].append(i)
            self.finger_qpos_adrs[arm_side][letter][key].append(qpos_adr)
        
        for side in self.finger_ctrl_ids:
            for letter in self.finger_ctrl_ids[side]:
                for key in self.finger_ctrl_ids[side][letter]:
                    self.finger_ctrl_ids[side][letter][key] = np.array(self.finger_ctrl_ids[side][letter][key])
                    self.finger_qpos_adrs[side][letter][key] = np.array(self.finger_qpos_adrs[side][letter][key])
    
    def _setup_finger_body_ids(self):
        for i in range(1, self.model.nbody):
            name = self.model.body(i).name
            if not name:
                continue
            side = "left" if "left" in name else "right" if "right" in name else None
            if not side:
                continue
            if "finger_a" in name and any(p in name for p in ("proximal", "middle", "distal")):
                self.finger_body_ids[side]["a"].append(i)
            elif "finger_b" in name and any(p in name for p in ("proximal", "middle", "distal")):
                self.finger_body_ids[side]["b"].append(i)
            elif "finger_c" in name and any(p in name for p in ("proximal", "middle", "distal")):
                self.finger_body_ids[side]["c"].append(i)
        
        for side in self.finger_body_ids:
            for letter in self.finger_body_ids[side]:
                self.finger_body_ids[side][letter] = np.array(self.finger_body_ids[side][letter], dtype=int)
    
    def _setup_tasks_and_limits(self):
        p = self.config.robot_prefix
        
        self.left_end_effector_task = mink.FrameTask(
            frame_name=f"{p}_{self.config.arm_prefixes[0]}_ee_site", frame_type="site",
            position_cost=5.0,  
            orientation_cost=5.0, 
            lm_damping=1e-2,      
        )
        self.right_end_effector_task = mink.FrameTask(
            frame_name=f"{p}_{self.config.arm_prefixes[1]}_ee_site", frame_type="site",
            position_cost=5.0,   
            orientation_cost=5.0,
            lm_damping=1e-2,      
        )
        
        self.posture_task = mink.PostureTask(model=self.model, cost=1e-3, lm_damping=1e-2)
        equality_task = mink.EqualityConstraintTask(model=self.model, cost=5000.0, gain=1.0, lm_damping=1e-3)
        
        self.tasks = [
            self.left_end_effector_task,
            self.right_end_effector_task,
            self.posture_task,
            equality_task,
        ]
        
        left_arm_geoms = mink.get_subtree_geom_ids(self.model, self.model.body(f"{p}_{self.config.arm_prefixes[0]}_arm_base").id)
        right_arm_geoms = mink.get_subtree_geom_ids(self.model, self.model.body(f"{p}_{self.config.arm_prefixes[1]}_arm_base").id)
        base_geoms = mink.get_body_geom_ids(self.model, self.model.body(f"{p}_base").id)
        floor_geom = mink.get_body_geom_ids(self.model, self.model.body("world_floor").id)

        collision_limit = mink.CollisionAvoidanceLimit(
            model=self.model,
            geom_pairs=[
                (floor_geom, left_arm_geoms),
                (floor_geom, right_arm_geoms),
                (left_arm_geoms, right_arm_geoms),
                (left_arm_geoms, base_geoms),
                (right_arm_geoms, base_geoms),
            ],
            gain=0.2, 
            minimum_distance_from_collisions=0.02, 
            collision_detection_distance=0.25, 
            bound_relaxation=0.01, 
            broadphase=True, 
        )
        
        max_velocities = {
            f"{p}_{arm}_{motor}": vel
            for arm in self.config.arm_prefixes
            for motor, vel in self.config.arm_motors_max_vel.items()
        }
        velocity_limit = mink.VelocityLimit(self.model, max_velocities)
        
        self.limits = [
            mink.ConfigurationLimit(model=self.configuration.model),
            # collision_limit,
            velocity_limit,
        ]
    
    def reset(self):
        if self.model.nkey > 0 and "home" in [self.model.key(i).name for i in range(self.model.nkey)]:
            mujoco.mj_resetDataKeyframe(self.model, self.data, self.model.key("home").id)
        else:
            mujoco.mj_resetData(self.model, self.data)
        
        self.configuration.update(self.data.qpos)
        self.posture_task.set_target_from_configuration(self.configuration)
        
        mujoco.mj_forward(self.model, self.data)
        p = self.config.robot_prefix
        mink.move_mocap_to_frame(self.model, self.data, "left_ik_target", f"{p}_left_ee_site", "site")
        mink.move_mocap_to_frame(self.model, self.data, "right_ik_target", f"{p}_right_ee_site", "site")
        
        print("[ROBOT] Reset to home keyframe, IK targets synced")
    
    def get_gripper_contact_forces(self) -> Dict:
        forces = {'left': {'a': 0.0, 'b': 0.0, 'c': 0.0},
                  'right': {'a': 0.0, 'b': 0.0, 'c': 0.0}}
        cfrc = self.data.cfrc_ext
        for side in ("left", "right"):
            for letter, ids in self.finger_body_ids[side].items():
                if len(ids) > 0:
                    forces[side][letter] = np.sum(np.linalg.norm(cfrc[ids, 3:6], axis=1))
        return forces
    
    def get_gripper_roll_xy(self, arm_idx: int) -> np.ndarray:
        body_name = f"{self.config.robot_prefix}_{self.config.arm_prefixes[arm_idx]}_gripper_roll"
        try:
            body_id = self.model.body(body_name).id
            return self.data.xpos[body_id, :2].copy()
        except KeyError:
            return np.array([0.0, 0.0])
    
    def get_robot_arm_y(self, side: str) -> float:
        name = self.config.robot_arm_left_name if side == "left" else self.config.robot_arm_right_name
        try:
            body_id = self.model.body(name).id
            return self.data.xpos[body_id, 1]
        except KeyError:
            return 0.0
    
    def set_mocap_pos(self, side: str, pos: np.ndarray):
        mocap_id = self.left_mocap_id if side == "left" else self.right_mocap_id
        self.data.mocap_pos[mocap_id] = pos
    
    def get_mocap_pos(self, side: str) -> np.ndarray:
        mocap_id = self.left_mocap_id if side == "left" else self.right_mocap_id
        return self.data.mocap_pos[mocap_id].copy()
    
    def set_mocap_quat(self, side: str, quat: np.ndarray):
        mocap_id = self.left_mocap_id if side == "left" else self.right_mocap_id
        self.data.mocap_quat[mocap_id] = quat
    
    def set_ee_targets(self):
        T_left = mink.SE3.from_mocap_name(self.model, self.data, "left_ik_target")
        self.left_end_effector_task.set_target(T_left)
        T_right = mink.SE3.from_mocap_name(self.model, self.data, "right_ik_target")
        self.right_end_effector_task.set_target(T_right)
    
    def solve_and_step(self, dt: float):
        try:
            vel = mink.solve_ik(
                self.configuration, self.tasks, dt, self.config.solver,
                safety_break=False, damping=self.config.ik_damping, limits=self.limits
            )
        except Exception:
            return
        
        self.configuration.integrate_inplace(vel, dt)
        self.data.ctrl[self.motor_ctrl_ids] = self.configuration.q[self.motor_qpos_adrs]
        mujoco.mj_step(self.model, self.data, nstep=self.config.sim_steps_per_ctrl)
        self.configuration.update(self.data.qpos)
    
    def apply_gripper_commands(self, commands: Dict):
        if not commands:
            return
        
        new_posture_target = self.configuration.q.copy()
        all_forces = self.get_gripper_contact_forces()
        
        for (side, letter, joint), target_val in commands.items():
            is_closing = np.isclose(target_val, self.finger_close_targets[joint], atol=1e-3)
            threshold = self.config.force_threshold_left if side == "left" else self.config.force_threshold_right
            finger_force = all_forces[side][letter]
            
            if is_closing and finger_force > threshold:
                current_qpos = self.data.qpos[self.finger_qpos_adrs[side][letter][joint]]
                self.data.ctrl[self.finger_ctrl_ids[side][letter][joint]] = current_qpos
                new_posture_target[self.finger_qpos_adrs[side][letter][joint]] = current_qpos
            else:
                self.data.ctrl[self.finger_ctrl_ids[side][letter][joint]] = target_val
                new_posture_target[self.finger_qpos_adrs[side][letter][joint]] = target_val
        
        self.posture_task.target = new_posture_target

class LandmarkPipeline:
    LEFT_INDICES = [15, 17, 19, 21] + list(range(33, 54))
    RIGHT_INDICES = [16, 18, 20, 22] + list(range(54, 75))
    PALM_LANDMARKS = [0, 5, 9, 13, 17]
    LEFT_HAND_OFFSET = 33
    RIGHT_HAND_OFFSET = 54
    
    def __init__(self, config: Config):
        self.config = config
        self.smoothed_pts: Optional[np.ndarray] = None
        
        self.static_tf_full = StaticTransform(
            x=0.3, y=0.0, z=config.seated_hip_height,
            roll=90.0, pitch=0.0, yaw=90.0,
            use_degrees=True,
        )
        self.static_tf_upper = StaticTransform(
            x=0.3, y=0.0, z=config.seated_hip_height,
            roll=90.0, pitch=0.0, yaw=90.0,
            use_degrees=True,
        )
        self.full_body_transform = AnchoredTransform(
            anchor_init=True,
            foot_on_ground=True, ground_level=0.0, foot_offset=0.0,
        )
        self.upper_only_transform = AnchoredTransform(
            anchor_init=False, origin='hip',
            foot_on_ground=False, ground_level=0.0, foot_offset=0.0,
            lock_ori="shoulder",
        )
    
    def reset(self):
        self.smoothed_pts = None
        self.full_body_transform.reset()
        self.upper_only_transform.reset()
        print(f"[PIPELINE] Smoothing reset (factor={self.config.smoothing_factor:.2f})")
    
    def _transform(self, raw_pts: np.ndarray, parent_pos: np.ndarray, parent_mat: np.ndarray, mode: str) -> np.ndarray:
        if mode == "upper-only":
            parent_pts = self.upper_only_transform.transform(raw_pts, parent_pos, parent_mat)
            return self.static_tf_upper.transform_points(parent_pts)
        
        parent_pts = self.full_body_transform.transform(raw_pts, parent_pos, parent_mat)
        return self.static_tf_full.transform_points(parent_pts)
    
    def _smooth(self, world_pts: np.ndarray, n_pts: int) -> np.ndarray:
        if self.smoothed_pts is None or self.smoothed_pts.shape != world_pts.shape:
            self.smoothed_pts = world_pts.copy()
            return self.smoothed_pts
        
        alpha = self.config.smoothing_factor
        beta = 1.0 - alpha
        
        if n_pts > 0:
            end = min(15, n_pts)
            self.smoothed_pts[0:end] = beta * self.smoothed_pts[0:end] + alpha * world_pts[0:end]
        
        if n_pts > 15:
            self.smoothed_pts[15:min(23, n_pts)] = world_pts[15:min(23, n_pts)]
        
        if n_pts > 23:
            self.smoothed_pts[23:min(25, n_pts)] = world_pts[23:min(25, n_pts)]
        
        if n_pts > 25:
            self.smoothed_pts[25:min(33, n_pts)] = beta * self.smoothed_pts[25:min(33, n_pts)] + alpha * world_pts[25:min(33, n_pts)]
        
        if n_pts > 33:
            self.smoothed_pts[33:min(75, n_pts)] = beta * self.smoothed_pts[33:min(75, n_pts)] + alpha * world_pts[33:min(75, n_pts)]
        
        return self.smoothed_pts
    
    def _scale_point_for_ik(self, point: np.ndarray, side: str, world_pts: np.ndarray, n_pts: int, robot: RobotInterface) -> np.ndarray:
        side_idx = 0 if side == "left" else 1
        c = self.config
        
        human_y_mid = c.human_y_mid[side_idx]
        if self.config.shoulder_to_y_mid and n_pts > 12:
            human_y_mid = world_pts[11 if side == "left" else 12, 1]
        
        human_x_mid = c.human_x_mid[side_idx]
        if self.config.elbow_to_x_scale and n_pts > 14:
            shoulder_idx = 11 if side == "left" else 12
            elbow_idx = 13 if side == "left" else 14
            shoulder = world_pts[shoulder_idx, :2]
            elbow = world_pts[elbow_idx, :2]
            human_x_mid = np.linalg.norm(elbow - shoulder)
        
        robot_y_mid = c.robot_y_mid[side_idx]
        if self.config.arm_to_y_mid:
            robot_y_mid = robot.get_robot_arm_y(side)
        
        out = point.copy()
        
        local_x = out[0] - human_x_mid
        scale_x = c.x_scale_pos[side_idx] if local_x >= 0 else c.x_scale_neg[side_idx]
        out[0] = local_x * scale_x + c.robot_x_mid[side_idx]
        
        local_y = out[1] - human_y_mid
        scale_y = c.y_scale_pos[side_idx] if local_y >= 0 else c.y_scale_neg[side_idx]
        out[1] = local_y * scale_y + robot_y_mid
        
        local_z = out[2] - c.human_z_mid[side_idx]
        scale_z = c.z_scale_pos[side_idx] if local_z >= 0 else c.z_scale_neg[side_idx]
        out[2] = local_z * scale_z + c.robot_z_mid[side_idx]
        
        return out

    def _compute_palm_from_pose(self, pts: np.ndarray, n_pts: int, side: str) -> Optional[np.ndarray]:
        if side == "left":
            mcp_indices = [17, 19, 21]
            wrist_idx = 15
        else: 
            mcp_indices = [18, 20, 22]
            wrist_idx = 16
        
        max_idx = max(mcp_indices)
        if n_pts <= max_idx:
            if n_pts > wrist_idx:
                return pts[wrist_idx].copy()
            return None
        
        palm_pos = pts[mcp_indices].mean(axis=0)
        return palm_pos

    def _extract_palms(self, pts: np.ndarray, n_pts: int) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        left_hand_exists = n_pts >= 54
        right_hand_exists = n_pts >= 75
        
        if left_hand_exists and right_hand_exists:
            left_palm_idx = [self.LEFT_HAND_OFFSET + i for i in self.PALM_LANDMARKS]
            left_palm = pts[left_palm_idx].mean(axis=0)
            right_palm_idx = [self.RIGHT_HAND_OFFSET + i for i in self.PALM_LANDMARKS]
            right_palm = pts[right_palm_idx].mean(axis=0)
            return left_palm, right_palm
        
        left_palm = self._compute_palm_from_pose(pts, n_pts, "left")
        right_palm = self._compute_palm_from_pose(pts, n_pts, "right")
        
        return left_palm, right_palm
    
    def process(self, raw_pts: np.ndarray, n_pts: int, parent_pos: np.ndarray, parent_mat: np.ndarray,
                robot: RobotInterface) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]]:
        if n_pts < 18:
            return None, None, None
        
        world_pts = self._transform(raw_pts, parent_pos, parent_mat, self.config.tracking_mode)
        world_pts = self._smooth(world_pts, n_pts)
        
        left_palm_world, right_palm_world = self._extract_palms(world_pts, n_pts)
        
        left_palm = self._scale_point_for_ik(left_palm_world, "left", world_pts, n_pts, robot) if left_palm_world is not None else None
        right_palm = self._scale_point_for_ik(right_palm_world, "right", world_pts, n_pts, robot) if right_palm_world is not None else None
        return world_pts, left_palm, right_palm

class InputManager:
    def __init__(self, config: Config):
        self.config = config
        
        self.tracking_data = {'poses': []}
        self.tracking_lock = threading.Lock()
        
        self.gesture_data = {'left': 'unknown', 'right': 'unknown'}
        self.gesture_lock = threading.Lock()
        
        self.tracking_sub = babyros.node.Subscriber(
            topic="landmarks",
            callback=self._tracking_callback
        )
        self.gesture_sub = babyros.node.Subscriber(
            topic="hand_gestures",
            callback=self._gesture_callback
        )
        
        self._kb_flags = {
            'backspace': False, 'm': False, 'f': False,
            'lb': False, 'rb': False,
            'lc': False, 'lo': False, 'rc': False, 'ro': False,
        }
    
    def _tracking_callback(self, msg: dict):
        with self.tracking_lock:
            self.tracking_data['poses'] = msg.get('poses', [])
    
    def _gesture_callback(self, msg: dict):
        with self.gesture_lock:
            self.gesture_data['left'] = msg.get('left_hand', 'unknown')
            self.gesture_data['right'] = msg.get('right_hand', 'unknown')
    
    def get_tracking_poses(self) -> List:
        with self.tracking_lock:
            return self.tracking_data['poses']
    
    def get_gestures(self) -> Tuple[str, str]:
        with self.gesture_lock:
            return self.gesture_data.get('left', 'unknown'), self.gesture_data.get('right', 'unknown')
    
    def _edge_triggered(self, flag_name: str, pressed: bool) -> bool:
        was_pressed = self._kb_flags[flag_name]
        self._kb_flags[flag_name] = pressed
        return pressed and not was_pressed
    
    def handle_keyboard(self, viewer, robot: RobotInterface, pipeline: LandmarkPipeline) -> Dict:
        gripper_commands = {}
        window = glfw.get_current_context()
        if not window:
            return gripper_commands
        
        l_shift = glfw.get_key(window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
        r_shift = glfw.get_key(window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS
        
        if self._edge_triggered('backspace', glfw.get_key(window, glfw.KEY_BACKSPACE) == glfw.PRESS):
            robot.reset()
            pipeline.reset()
        
        if self._edge_triggered('m', glfw.get_key(window, glfw.KEY_M) == glfw.PRESS):
            self.config.tracking_mode = "full-body" if self.config.tracking_mode == "upper-only" else "upper-only"
            print(f"[MODE] Switched to: {self.config.tracking_mode}")
        
        if self._edge_triggered('f', glfw.get_key(window, glfw.KEY_F) == glfw.PRESS):
            self.config.face_target_enabled = not self.config.face_target_enabled
            print(f"[FACE] Gripper face-target: {'ON' if self.config.face_target_enabled else 'OFF'}")
        
        if self._edge_triggered('lb', glfw.get_key(window, glfw.KEY_LEFT_BRACKET) == glfw.PRESS):
            self.config.smoothing_factor = max(0.0, self.config.smoothing_factor - self.config.smoothing_step)
            pipeline.smoothed_pts = None
            print(f"[SMOOTH] factor={self.config.smoothing_factor:.2f} (more responsive)")
        
        if self._edge_triggered('rb', glfw.get_key(window, glfw.KEY_RIGHT_BRACKET) == glfw.PRESS):
            self.config.smoothing_factor = min(1.0, self.config.smoothing_factor + self.config.smoothing_step)
            pipeline.smoothed_pts = None
            print(f"[SMOOTH] factor={self.config.smoothing_factor:.2f} (smoother)")
        
        if l_shift:
            pos = robot.get_mocap_pos("left")
            if glfw.get_key(window, glfw.KEY_UP) == glfw.PRESS: pos[2] += self.config.target_move_step
            if glfw.get_key(window, glfw.KEY_DOWN) == glfw.PRESS: pos[2] -= self.config.target_move_step
            if glfw.get_key(window, glfw.KEY_LEFT) == glfw.PRESS: pos[1] += self.config.target_move_step
            if glfw.get_key(window, glfw.KEY_RIGHT) == glfw.PRESS: pos[1] -= self.config.target_move_step
            if glfw.get_key(window, glfw.KEY_W) == glfw.PRESS: pos[0] += self.config.target_move_step
            if glfw.get_key(window, glfw.KEY_S) == glfw.PRESS: pos[0] -= self.config.target_move_step
            robot.set_mocap_pos("left", pos)
        
        if r_shift:
            pos = robot.get_mocap_pos("right")
            if glfw.get_key(window, glfw.KEY_UP) == glfw.PRESS: pos[2] += self.config.target_move_step
            if glfw.get_key(window, glfw.KEY_DOWN) == glfw.PRESS: pos[2] -= self.config.target_move_step
            if glfw.get_key(window, glfw.KEY_LEFT) == glfw.PRESS: pos[1] += self.config.target_move_step
            if glfw.get_key(window, glfw.KEY_RIGHT) == glfw.PRESS: pos[1] -= self.config.target_move_step
            if glfw.get_key(window, glfw.KEY_W) == glfw.PRESS: pos[0] += self.config.target_move_step
            if glfw.get_key(window, glfw.KEY_S) == glfw.PRESS: pos[0] -= self.config.target_move_step
            robot.set_mocap_pos("right", pos)
        
        c_key = glfw.get_key(window, glfw.KEY_C) == glfw.PRESS
        o_key = glfw.get_key(window, glfw.KEY_O) == glfw.PRESS
        
        if l_shift and c_key and self._edge_triggered('lc', True):
            for letter in "abc":
                for joint in ("joint1", "joint2", "joint3"):
                    gripper_commands[("left", letter, joint)] = robot.finger_close_targets[joint]
        else:
            self._kb_flags['lc'] = False
        
        if l_shift and o_key and self._edge_triggered('lo', True):
            for letter in "abc":
                for joint in ("joint1", "joint2", "joint3"):
                    gripper_commands[("left", letter, joint)] = robot.finger_open_targets[joint]
        else:
            self._kb_flags['lo'] = False
        
        if r_shift and c_key and self._edge_triggered('rc', True):
            for letter in "abc":
                for joint in ("joint1", "joint2", "joint3"):
                    gripper_commands[("right", letter, joint)] = robot.finger_close_targets[joint]
        else:
            self._kb_flags['rc'] = False
        
        if r_shift and o_key and self._edge_triggered('ro', True):
            for letter in "abc":
                for joint in ("joint1", "joint2", "joint3"):
                    gripper_commands[("right", letter, joint)] = robot.finger_open_targets[joint]
        else:
            self._kb_flags['ro'] = False
        
        left_gesture, right_gesture = self.get_gestures()
        if left_gesture == "close":
            for letter in "abc":
                for joint in ("joint1", "joint2", "joint3"):
                    gripper_commands[("left", letter, joint)] = robot.finger_close_targets[joint]
        elif left_gesture == "open":
            for letter in "abc":
                for joint in ("joint1", "joint2", "joint3"):
                    gripper_commands[("left", letter, joint)] = robot.finger_open_targets[joint]
        
        if right_gesture == "close":
            for letter in "abc":
                for joint in ("joint1", "joint2", "joint3"):
                    gripper_commands[("right", letter, joint)] = robot.finger_close_targets[joint]
        elif right_gesture == "open":
            for letter in "abc":
                for joint in ("joint1", "joint2", "joint3"):
                    gripper_commands[("right", letter, joint)] = robot.finger_open_targets[joint]
        
        return gripper_commands
    
    def is_kb_active(self, side: str) -> bool:
        window = glfw.get_current_context()
        if not window:
            return False
        if side == "left":
            return glfw.get_key(window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
        return glfw.get_key(window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS
    
    def cleanup(self):
        self.tracking_sub.delete()
        self.gesture_sub.delete()


class Renderer:
    POSE_CONNECTIONS = [
        (0,1),(1,2),(2,3),(3,7),(0,4),(4,5),(5,6),(6,8),(9,10),
        (11,12),(11,13),(13,15),(12,14),(14,16),
        (15,17),(15,19),(15,21),(16,18),(16,20),(16,22),
        (11,23),(12,24),(23,24),(23,25),(25,27),(24,26),(26,28),
        (27,29),(27,31),(28,30),(28,32),
    ]
    HAND_CONNECTIONS = [
        (0,1),(1,2),(2,3),(3,4),(0,5),(5,6),(6,7),(7,8),
        (5,9),(9,10),(10,11),(11,12),(9,13),(13,14),(14,15),(15,16),
        (13,17),(17,18),(18,19),(19,20),(0,17),
    ]
    
    EYE_MAT_FLAT = np.eye(3).flatten()
    POINT_SIZE = (0.01, 0.0, 0.0)
    SKELETON_RGBA = (0.5, 0.5, 0.5, 1.0)
    COLORS = [(0,1,1,1)]*33 + [(1,0,1,1)]*21 + [(1,0.5,0,1)]*21
    
    def __init__(self, config: Config, model: mujoco.MjModel):
        self.config = config
        self.model = model
        
        self.skeleton_connections = list(self.POSE_CONNECTIONS)
        for c in self.HAND_CONNECTIONS:
            self.skeleton_connections.append((c[0]+33, c[1]+33))
        for c in self.HAND_CONNECTIONS:
            self.skeleton_connections.append((c[0]+54, c[1]+54))
        
        self.camera = mujoco.MjvCamera()
        self.camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        self.camera.distance = 3.0
        self.camera.azimuth = 135.0
        self.camera.elevation = -20.0
        self.camera.lookat[:] = [0.0, 0.0, 0.8]
        
        self.scene = mujoco.MjvScene(model, maxgeom=2000)
        self.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW.value] = 0
        self.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION.value] = 0
        self.scene.flags[mujoco.mjtRndFlag.mjRND_FOG.value] = 1
        
        self.opt = mujoco.MjvOption()
        self.opt.frame = mujoco.mjtFrame.mjFRAME_NONE
        
        self.viewer = GlfwViewer(
            width=config.viewer_width,
            height=config.viewer_height,
            title=config.viewer_title
        )
        self.viewer.setup(model, self.camera, self.scene, self.opt)
    
    def should_close(self) -> bool:
        return self.viewer.should_close()
    
    def render_frame(self, data: mujoco.MjData, world_pts: Optional[np.ndarray],
                     n_pts: int, tracking_mode: str):
        scene = self.viewer.prepare_scene(data)
        
        if world_pts is not None and n_pts > 0:
            self._draw_points(scene, world_pts, n_pts, tracking_mode)
            self._draw_skeleton(scene, world_pts, n_pts, tracking_mode)
        
        self.viewer.render_frame(scene)
    
    def _draw_points(self, scene, world_pts: np.ndarray, n_pts: int, tracking_mode: str):
        for i in range(n_pts):
            if tracking_mode == "upper-only" and 25 <= i <= 32:
                continue
            if scene.ngeom >= scene.maxgeom:
                break
            g = scene.geoms[scene.ngeom]
            mujoco.mjv_initGeom(
                g, mujoco.mjtGeom.mjGEOM_SPHERE,
                size=self.POINT_SIZE, pos=world_pts[i],
                mat=self.EYE_MAT_FLAT, rgba=self.COLORS[i]
            )
            scene.ngeom += 1
    
    def _draw_skeleton(self, scene, world_pts: np.ndarray, n_pts: int, tracking_mode: str):
        for c in self.skeleton_connections:
            if tracking_mode == "upper-only":
                if (25 <= c[0] <= 32) or (25 <= c[1] <= 32):
                    continue
            if c[0] >= n_pts or c[1] >= n_pts:
                continue
            if scene.ngeom >= scene.maxgeom:
                break
            
            p1, p2 = world_pts[c[0]], world_pts[c[1]]
            vec = p2 - p1
            length = np.linalg.norm(vec)
            if length < 1e-5:
                continue
            
            z_ax = vec / length
            x_ax = np.cross(z_ax, [1,0,0]) if abs(z_ax[0]) < 0.9 else np.cross(z_ax, [0,1,0])
            x_ax /= np.linalg.norm(x_ax)
            y_ax = np.cross(z_ax, x_ax)
            mat = np.array([x_ax, y_ax, z_ax]).T.flatten()
            
            g = scene.geoms[scene.ngeom]
            mujoco.mjv_initGeom(
                g, mujoco.mjtGeom.mjGEOM_CAPSULE,
                size=(0.005, length/2.0, 0.0),
                pos=(p1+p2)/2.0, mat=mat,
                rgba=self.SKELETON_RGBA
            )
            scene.ngeom += 1
    
    def close(self):
        self.viewer.close()


def compute_face_target_quat(target_pos: np.ndarray, base_xy: np.ndarray, yaw_offset: float = 0.0) -> np.ndarray:
    direction = target_pos[:2] - base_xy
    if np.linalg.norm(direction) < 1e-5:
        return np.array([1.0, 0.0, 0.0, 0.0])
    yaw = np.arctan2(direction[1], direction[0]) + yaw_offset
    rot = Rotation.from_euler('z', yaw)
    quat_xyzw = rot.as_quat()
    return np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]])


def get_body_world_pose(data: mujoco.MjData, body_name: str):
    body_id = data.model.body(body_name).id
    return data.xpos[body_id].copy(), data.xmat[body_id].reshape(3, 3).copy()

class TeleopController:
    def __init__(self):
        self.config = Config()
        self.robot = RobotInterface(self.config)
        self.pipeline = LandmarkPipeline(self.config)
        self.input = InputManager(self.config)
        self.renderer = Renderer(self.config, self.robot.model)
        
        self.parent_body_name = f"{self.config.robot_prefix}_mobile_base"
        self.robot_base_xy = np.array([0.0, 0.0])
        self.force_log_counter = 0
        self.rte = RateLimiter(frequency=self.config.control_freq, warn=False)
    
    def _print_controls(self):
        print("\n" + "=" * 60)
        print("KEYBOARD CONTROLS")
        print("=" * 60)
        print("[GLOBAL]")
        print("  BACKSPACE        -> Full reset (home pose)")
        print("  M                -> Toggle tracking mode (upper-only / full-body)")
        print("  F                -> Toggle gripper yaw face-target (ON/OFF)")
        print("\n[SMOOTHING]")
        print("  [ / ]            -> Decrease / increase landmark smoothing")
        print(f"                     (current: {self.config.smoothing_factor:.2f})")
        print("\n[LEFT ARM]  (hold Left Shift + key)")
        print("  W / S            -> Move left target  forward / backward  (X)")
        print("  <- / ->          -> Move left target  right / left        (Y)")
        print("  ↑ / ↓            -> Move left target  up / down           (Z)")
        print("  C                -> Close left gripper")
        print("  O                -> Open left gripper")
        print("\n[RIGHT ARM] (hold Right Shift + key)")
        print("  W / S            -> Move right target forward / backward  (X)")
        print("  <- / ->          -> Move right target right / left        (Y)")
        print("  ↑ / ↓            -> Move right target up / down           (Z)")
        print("  C                -> Close right gripper")
        print("  O                -> Open right gripper")
        print("=" * 60 + "\n")
    
    def _update_face_target(self, left_kb_active: bool, right_kb_active: bool,
                            left_palm: Optional[np.ndarray], right_palm: Optional[np.ndarray]):
        if not self.config.face_target_enabled:
            return
        
        left_pos = self.robot.get_mocap_pos("left") if left_kb_active else left_palm
        right_pos = self.robot.get_mocap_pos("right") if right_kb_active else right_palm
        
        left_gripper_xy = self.robot.get_gripper_roll_xy(0)
        right_gripper_xy = self.robot.get_gripper_roll_xy(1)
        
        if left_pos is not None:
            self.robot.set_mocap_quat(
                "left",
                compute_face_target_quat(left_pos, left_gripper_xy, self.config.yaw_offset)
            )
        if right_pos is not None:
            self.robot.set_mocap_quat(
                "right",
                compute_face_target_quat(right_pos, right_gripper_xy, self.config.yaw_offset)
            )
    
    def run(self):
        self.robot.reset()
        self.pipeline.reset()
        self._print_controls()
        
        print(f"[MODE] Tracking mode: {self.config.tracking_mode}")
        print(f"[FACE] Gripper face-target: {'ON' if self.config.face_target_enabled else 'OFF'}")
        print(f"[SHOULDER Y-MID] {'ON' if self.config.shoulder_to_y_mid else 'OFF'}")
        print(f"[ELBOW X-MID] {'ON' if self.config.elbow_to_x_scale else 'OFF'}")
        print(f"[ARM Y-MID] {'ON' if self.config.arm_to_y_mid else 'OFF'}")
        
        try:
            while not self.renderer.should_close():
                parent_pos, parent_mat = get_body_world_pose(self.robot.data, self.parent_body_name)
                
                poses_list = self.input.get_tracking_poses()
                n_pts = min(len(poses_list), self.config.max_points) if poses_list else 0
                
                world_pts = None
                left_palm = None
                right_palm = None
                
                if n_pts > 0:
                    raw_pts = np.array(
                        [[p['position']['x'], p['position']['y'], p['position']['z']]
                         for p in poses_list[:n_pts]],
                        dtype=np.float64
                    )
                    world_pts, left_palm, right_palm = self.pipeline.process(
                        raw_pts, n_pts, parent_pos, parent_mat, self.robot
                    )
                
                left_kb_active = self.input.is_kb_active("left")
                right_kb_active = self.input.is_kb_active("right")
                
                gripper_commands = self.input.handle_keyboard(
                    self.renderer.viewer, self.robot, self.pipeline
                )
                
                if not left_kb_active and left_palm is not None:
                    self.robot.set_mocap_pos("left", left_palm)
                if not right_kb_active and right_palm is not None:
                    self.robot.set_mocap_pos("right", right_palm)
                
                self._update_face_target(left_kb_active, right_kb_active, left_palm, right_palm)
                
                self.robot.set_ee_targets()
                self.robot.apply_gripper_commands(gripper_commands)
                
                self.robot.solve_and_step(self.rte.dt)
                
                self.renderer.render_frame(self.robot.data, world_pts, n_pts, self.config.tracking_mode)
                
                self.rte.sleep()
                self.force_log_counter += 1
        
        except KeyboardInterrupt:
            print("\nInterrupted by user")
        finally:
            self.input.cleanup()
            self.renderer.close()
            print("Cleanup complete")


if __name__ == "__main__":
    controller = TeleopController()
    controller.run()