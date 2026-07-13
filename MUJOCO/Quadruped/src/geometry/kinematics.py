from abc import ABC, abstractmethod
import numpy as np
from scipy.optimize import minimize
from utils.logger import setup_logger
import mujoco
import mink
from pathlib import Path
from typing import Union
from loop_rate_limiters import RateLimiter

logger = setup_logger("Kinematics")

class KinematicsSolver(ABC):
    @abstractmethod
    def forward(self, left_column_lift: float, right_column_lift: float, telescopic_extend: float, base_rotation: float, **kwargs) -> np.ndarray: ...
    
    @abstractmethod
    def inverse(self, target: np.ndarray, arm: str, current_encoders: np.ndarray, 
                base_pose: np.ndarray = None, coord: str = "global", **kwargs) -> np.ndarray: ...


class ParallelArmKinematics(KinematicsSolver):
    def __init__(self, ik_arm_offset_left: np.ndarray, ik_arm_offset_right: np.ndarray):
        self.ik_arm_offset_left = ik_arm_offset_left
        self.ik_arm_offset_right = ik_arm_offset_right
        self._ik_cache = {'left': {'target': None, 'result': None}, 'right': {'target': None, 'result': None}}

    def forward(self, left_column_lift: float, right_column_lift: float, telescopic_extend: float, base_rotation: float, 
                phi: float = 0.0, d2: float = 0.1, l3_max: float = 0.7, wrist_length: float = 0.25, eps: float = 1e-12) -> np.ndarray:
        """
        Forward kinematics using explicit joint names.
        
        Args:
            left_column_lift: Left column height
            right_column_lift: Right column height
            telescopic_extend: Arm extension length
            base_rotation: Base rotation angle (radians)
            ...
        Returns:
            End-effector position in local frame [x, y, z]
        """
        p1 = np.array([0.0, 0.0, left_column_lift])
        p2 = np.array([-d2 * np.cos(base_rotation), -d2 * np.sin(base_rotation), right_column_lift])
        
        v = p1 - p2; dist = np.linalg.norm(v)
        if dist < eps: dist = eps
        u = v / dist; wb = p1 + telescopic_extend * u
        
        z = u 
        ref = np.array([0.0, 0.0, 1.0]) if np.abs(z[2]) < 0.99 else np.array([1.0, 0.0, 0.0])
        proj = np.dot(ref, z)
        xl = ref - proj * z
        n = np.linalg.norm(xl)
        if n < eps: ref = np.array([1.0, 0.0, 0.0])
        proj = np.dot(ref, z)
        xl = ref - proj * z
        n = np.linalg.norm(xl)
        if n < eps: n = 1.0
        xl /= n
        
        wz = xl * -np.sin(phi) + z * np.cos(phi)
        ee_local = wb + wrist_length * wz
        return ee_local

    def inverse(self, target: np.ndarray, arm: str, current_encoders: np.ndarray, 
                base_pose: np.ndarray = None, coord: str = "global",
                d2: float = 0.1, l3_max: float = 0.7, alpha_min_deg: float = 10.0, 
                bounds_h: tuple = (0.0, 1.5), bounds_a: tuple = (0.0, 0.7), 
                cache_threshold: float = 0.001) -> np.ndarray:
        """
        Inverse kinematics - solves for joint angles to reach target.
        current_encoders order: [left_column_lift, right_column_lift, telescopic_extend, base_rotation]
        """
        target = np.array(target, dtype=np.float64)
        arm_offset = self.ik_arm_offset_left if arm == "left" else self.ik_arm_offset_right
        
        # Check cache
        cache = self._ik_cache[arm]
        if cache['target'] is not None and np.linalg.norm(target - cache['target']) <= cache_threshold:
            return cache['result'].copy()
        
        # Convert global target to local if needed
        if coord == "global":
            if base_pose is None:
                raise ValueError("base_pose is required for coord='global'")
            
            x_b, y_b, theta_b = base_pose
            c, s = np.cos(theta_b), np.sin(theta_b)
            R = np.array([[c, -s], [s, c]])
            
            arm_base_world_xy = R @ arm_offset[:2]
            arm_base_z = arm_offset[2]
            
            target_local_xy = R.T @ (target[:2] - arm_base_world_xy)
            target_local_z = target[2] - arm_base_z
            target_local = np.array([target_local_xy[0], target_local_xy[1], target_local_z])
        else:
            target_local = target
        
        alpha_min_rad = np.deg2rad(alpha_min_deg)

        def get_ee_local(vars):
            """Compute end-effector position in local frame"""
            # vars: [left_column_lift, right_column_lift, telescopic_extend, base_rotation]
            return self.forward(vars[0], vars[1], vars[2], vars[3], d2=d2, l3_max=l3_max)

        def cost(vars, w_a1=1e-2):
            """Cost function: distance to target + regularization"""
            ee_local = get_ee_local(vars)
            dist_err = np.sum((ee_local - target_local)**2)
            # Regularize telescopic_extend (vars[2])
            return float(dist_err + w_a1 * vars[2])
        
        def min_angle_con(vars):
            """Constraint: minimum column angle between left and right columns"""
            # vars[0] = left_column_lift, vars[1] = right_column_lift
            return np.arctan2(np.abs(vars[1] - vars[0]), d2) - alpha_min_rad
        
        # Solve optimization
        res = minimize(
            cost, 
            current_encoders, 
            method='SLSQP', 
            bounds=[bounds_h, bounds_h, bounds_a, (-np.pi, np.pi)], 
            constraints={'type': 'ineq', 'fun': min_angle_con},
            options={'ftol': 1e-9, 'maxiter': 50, 'disp': False}
        )
        
        if not res.success:
            logger.warning(f"IK failed for {arm} arm. Using fallback.")
            return cache['result'].copy() if cache['result'] is not None else current_encoders
            
        result = np.array([float(res.x[0]), float(res.x[1]), float(res.x[2]), float(res.x[3])])
        cache['target'] = target.copy()
        cache['result'] = result.copy()
        return result


logger = setup_logger("MinkKinematics")

class MinkSolver:
    def __init__(self, 
                 mj_model, 
                 mj_data,
                 rate: RateLimiter,
                 solver: str = "daqp", 
                 max_iters: int = 20, 
                 pos_threshold: float = 5e-3, 
                 ori_threshold: float = 5e-3):
        self.model = mj_model
        self.data = mj_data
        self.configuration = mink.Configuration(self.model)
    
        self.tasks, self.left_task, self.right_task, self.posture_task = self._create_tasks()
        self.limits = self._create_limits()

        # Order MUST match the forward/inverse argument order
        self.arm_joints = [
            "left_column_lift",  
            "right_column_lift", 
            "telescopic_extend",  
            "base_rotation"      
        ]
        self.wrist_joints = [
            "gripper_roll_joint", 
            "gripper_pitch_joint", 
            "gripper_yaw_joint"
        ]
        self.controlled_joints = self.arm_joints + self.wrist_joints

        self.solver = solver
        self.max_iters = max_iters
        self.pos_threshold = pos_threshold
        self.ori_threshold = ori_threshold
        self.rate = rate  # Store the RateLimiter object
        
        self.arm_joint_adr = self._get_joint_adr(self.arm_joints)
        self.wrist_joint_adr = self._get_joint_adr(self.wrist_joints)
        
        mujoco.mj_resetData(self.model, self.data)
        self.configuration.update(self.data.qpos)
        self.posture_task.set_target_from_configuration(self.configuration)
        mujoco.mj_forward(self.model, self.data)

    def _create_tasks(self) -> tuple:
        left_end_effector_task = mink.FrameTask(
            frame_name="left_ee_site", frame_type="site",
            position_cost=1.0, orientation_cost=1.0, lm_damping=1e-2,          
        )
        
        right_end_effector_task = mink.FrameTask(
            frame_name="right_ee_site", frame_type="site",
            position_cost=1.0, orientation_cost=1.0, lm_damping=1e-2,         
        )

        posture_task = mink.PostureTask(model=self.model, cost=1e-2)

        equality_task = mink.EqualityConstraintTask(
            model=self.model, cost=500.0, gain=1.0, lm_damping=1e-3,
        )
        
        tasks = [left_end_effector_task, right_end_effector_task, posture_task, equality_task]
        return tasks, left_end_effector_task, right_end_effector_task, posture_task
    
    def _create_limits(self) -> list:
        try:
            left_arm_geoms = mink.get_subtree_geom_ids(self.model, self.model.body("left_arm_base").id)
            right_arm_geoms = mink.get_subtree_geom_ids(self.model, self.model.body("right_arm_base").id)
            base_geoms = mink.get_body_geom_ids(self.model, self.model.body("mobile_base").id)
            
            collision_pairs = [
                (left_arm_geoms, right_arm_geoms),
                (left_arm_geoms, base_geoms),
                (right_arm_geoms, base_geoms),
            ]
            
            collision_avoidance_limit = mink.CollisionAvoidanceLimit(
                model=self.model, geom_pairs=collision_pairs,
                minimum_distance_from_collisions=0.05, collision_detection_distance=0.1,
            )
        except KeyError:
            collision_avoidance_limit = None

        velocity_limit = mink.VelocityLimit(model=self.model)
        
        # Freeze all joints NOT in the controlled list (e.g., all 11 finger joints)
        controlled_dofs = set()
        for name in self.controlled_joints:
            try:
                jnt_id = self.model.joint(name).id
                dof_adr = self.model.jnt_dofadr[jnt_id]
                n_dof = 6 if self.model.jnt_type[jnt_id] == mujoco.mjtJoint.mjJNT_FREE else 1
                controlled_dofs.update(range(dof_adr, dof_adr + n_dof))
            except KeyError:
                pass
                
        for dof_idx in range(self.model.nv):
            if dof_idx not in controlled_dofs:
                velocity_limit.limit[dof_idx] = 0.0

        limits = [
            mink.ConfigurationLimit(model=self.model), 
            velocity_limit
        ]
        if collision_avoidance_limit is not None:
            limits.append(collision_avoidance_limit)
            
        return limits

    def _get_joint_adr(self, joint_names: list) -> np.ndarray:
        adr = []
        for name in joint_names:
            try:
                adr.append(self.model.jnt_qposadr[self.model.joint(name).id])
            except KeyError:
                pass
        return np.array(adr)

    def forward(self, left_column_lift: float, right_column_lift: float, telescopic_extend: float, base_rotation: float, arm: str = "left", **kwargs) -> np.ndarray:
        """Compute Forward Kinematics using MuJoCo physics."""
        if len(self.arm_joint_adr) == 4:
            self.data.qpos[self.arm_joint_adr] = [left_column_lift, right_column_lift, telescopic_extend, base_rotation]
            
        mujoco.mj_forward(self.model, self.data)
        site_name = f"{arm}_ee_site" 
        site_id = self.model.site(site_name).id
        return self.data.site_xpos[site_id].copy()

    def inverse(self, target_pose: np.ndarray, arm: str) -> np.ndarray:
        """Compute Inverse Kinematics using Mink QP solver."""
        target_pose = np.asarray(target_pose, dtype=np.float64)
        
        mocap_name = f"{arm}_ik_target"
        mocap_id = self.model.body(mocap_name).mocapid[0]
        self.data.mocap_pos[mocap_id, :3] = target_pose[:3]
    
        if target_pose.size >= 7:
            self.data.mocap_quat[mocap_id] = target_pose[3:7]
        else:
            self.data.mocap_quat[mocap_id] = [1.0, 0.0, 0.0, 0.0]
            
        T_target = mink.SE3.from_mocap_name(self.model, self.data, mocap_name)
        task = self.left_task if arm == "left" else self.right_task
        task.set_target(T_target)
        
        for _ in range(self.max_iters):
            vel = mink.solve_ik(
                self.configuration, self.tasks, self.rate.dt, self.solver,
                safety_break=False, damping=1e-2, limits=self.limits
            )
            self.configuration.integrate_inplace(vel, self.rate.dt) 
            err = task.compute_error(self.configuration)
            if np.linalg.norm(err[:3]) <= self.pos_threshold and np.linalg.norm(err[3:]) <= self.ori_threshold:
                break
                
        result_qpos = self.configuration.q.copy()
        return result_qpos[self.arm_joint_adr]