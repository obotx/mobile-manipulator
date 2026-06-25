from abc import ABC, abstractmethod
import numpy as np
from scipy.optimize import minimize
from utils.logger import setup_logger

logger = setup_logger("Kinematics")

class KinematicsSolver(ABC):
    @abstractmethod
    def forward(self, h1: float, h2: float, a1: float, theta: float, **kwargs) -> np.ndarray: ...
    
    @abstractmethod
    def inverse(self, target: np.ndarray, arm: str, current_encoders: np.ndarray, 
                base_pose: np.ndarray = None, coord: str = "global", **kwargs) -> np.ndarray: ...

class ParallelArmKinematics(KinematicsSolver):
    def __init__(self, ik_arm_offset_left: np.ndarray, ik_arm_offset_right: np.ndarray):
        self.ik_arm_offset_left = ik_arm_offset_left
        self.ik_arm_offset_right = ik_arm_offset_right
        self._ik_cache = {'left': {'target': None, 'result': None}, 'right': {'target': None, 'result': None}}

    def forward(self, h1: float, h2: float, a1: float, theta: float, phi: float = 0.0, d2: float = 0.1, l3_max: float = 0.7, wrist_length: float = 0.25, eps: float = 1e-12) -> np.ndarray:
        """
        Forward kinematics 
        
        Args:
            h1, h2: Column heights
            a1: Arm extension
            theta: Base rotation
            phi: Wrist roll angle
            d2: Column offset distance
            l3_max: Maximum arm length
            wrist_length: Distance from arm to end-effector
            eps: Numerical stability threshold
            
        Returns:
            End-effector position in local frame [x, y, z]
        """
        p1 = np.array([0.0, 0.0, h1])
        p2 = np.array([-d2 * np.cos(theta), -d2 * np.sin(theta), h2])
        v = p1 - p2; dist = np.linalg.norm(v)
        if dist < eps: dist = eps
        u = v / dist; wb = p1 + a1 * u
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
        
        Args:
            target: Target position
            arm: "left" or "right"
            current_encoders: Current joint angles [h1, h2, a1, theta]
            base_pose: Robot base pose [x, y, yaw] (required for coord="global")
            coord: "global" (world frame) or "local" (arm-relative frame)
            d2: Column offset distance
            l3_max: Maximum arm length
            alpha_min_deg: Minimum column angle constraint
            bounds_h: Bounds for h1, h2
            bounds_a: Bounds for a1
            cache_threshold: Cache hit threshold
            
        Returns:
            Joint angles [h1, h2, a1, theta]
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
            
            # Transform world target to arm-local frame
            arm_base_world_xy = R @ arm_offset[:2]
            arm_base_z = arm_offset[2]
            
            # Target in arm frame
            target_local_xy = R.T @ (target[:2] - arm_base_world_xy)
            target_local_z = target[2] - arm_base_z
            target_local = np.array([target_local_xy[0], target_local_xy[1], target_local_z])
        else:
            # Already in local frame
            target_local = target
        
        alpha_min_rad = np.deg2rad(alpha_min_deg)

        def get_ee_local(vars):
            """Compute end-effector position in local frame"""
            return self.forward(vars[0], vars[1], vars[2], vars[3], d2=d2, l3_max=l3_max)

        def cost(vars, w_a1=1e-2):
            """Cost function: distance to target + regularization"""
            ee_local = get_ee_local(vars)
            dist_err = np.sum((ee_local - target_local)**2)
            return float(dist_err + w_a1 * vars[2])
        
        def min_angle_con(vars):
            """Constraint: minimum column angle"""
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