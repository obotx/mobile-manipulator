import numpy as np
from typing import Optional, List, Tuple, Dict, Any

class StaticTransform:
    """Rigid transform (rotation + translation) with optional per-axis rigidity.
    
    When rigid_<axis>=True, the output coordinate along that axis is clamped
    to the fixed translation value, ignoring the transformed input.
    """
    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0,
                 roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0,
                 use_degrees: bool = False,
                 rigid_x: bool = False, rigid_y: bool = False, rigid_z: bool = False):
        self.t = np.array([x, y, z], dtype=np.float64)
        self.rigid = np.array([rigid_x, rigid_y, rigid_z], dtype=bool)

        if use_degrees:
            roll, pitch, yaw = np.deg2rad([roll, pitch, yaw])
        cx, sx = np.cos(roll), np.sin(roll)
        cy, sy = np.cos(pitch), np.sin(pitch)
        cz, sz = np.cos(yaw), np.sin(yaw)
        Rx = np.array([[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]])
        Ry = np.array([[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]])
        Rz = np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]])
        self.R = Rz @ Ry @ Rx

    def transform_point(self, point: np.ndarray) -> np.ndarray:
        out = (self.R @ np.asarray(point, dtype=np.float64)) + self.t
        # Override rigid axes with the fixed translation
        out[self.rigid] = self.t[self.rigid]
        return out

    def transform_points(self, points: np.ndarray) -> np.ndarray:
        out = (np.asarray(points, dtype=np.float64) @ self.R.T) + self.t
        # Override rigid axes with the fixed translation (broadcast across all points)
        if self.rigid.any():
            out[:, self.rigid] = self.t[self.rigid]
        return out
    
class HipAnchoredTransform:    
    LEFT_ANKLE = 27
    LEFT_HEEL = 29
    LEFT_FOOT = 31
    RIGHT_ANKLE = 28
    RIGHT_HEEL = 30
    RIGHT_FOOT = 32
    FOOT_INDICES = [LEFT_ANKLE, LEFT_HEEL, LEFT_FOOT, RIGHT_ANKLE, RIGHT_HEEL, RIGHT_FOOT]
    
    def __init__(self, static_tf: StaticTransform,
                 anchor_init: bool = True,
                 foot_on_ground: bool = False,
                 ground_level: float = 0.0,
                 foot_offset: float = 0.0):
        self.static_tf = static_tf
        self.anchor_init = anchor_init
        self.foot_on_ground = foot_on_ground
        self.ground_level = ground_level
        self.foot_offset = foot_offset
        self.initial_hip_pos: Optional[np.ndarray] = None
        self.current_hip_pos: Optional[np.ndarray] = None
        self.initialized = False
    
    def reset(self):
        self.initial_hip_pos = None
        self.current_hip_pos = None
        self.initialized = False
    
    def transform(self, points: np.ndarray, parent_pos: np.ndarray, parent_mat: np.ndarray) -> np.ndarray:
        if len(points) < 25:
            return points
        hip_center = (points[23] + points[24]) / 2.0
        if not self.initialized:
            self.initial_hip_pos = hip_center.copy()
            self.initialized = True
        self.current_hip_pos = hip_center.copy()
        relative_points = points - hip_center
        local_points = relative_points @ self.static_tf.R.T
        if self.anchor_init:
            translation = np.array([self.static_tf.t[0], self.static_tf.t[1], hip_center[2]], dtype=np.float64)
        else:
            translation = hip_center.copy()
        for i in range(3):
            if self.static_tf.rigid[i]:
                translation[i] = self.static_tf.t[i]
        local_points = local_points + translation
        if self.foot_on_ground and len(points) > max(self.FOOT_INDICES):
            foot_points = local_points[self.FOOT_INDICES]
            lowest_foot_z = np.min(foot_points[:, 2])
            target_foot_z = self.ground_level + self.foot_offset
            target_local = parent_mat.T @ (np.array([0, 0, target_foot_z]) - parent_pos)
            z_shift = target_local[2] - lowest_foot_z
            local_points[:, 2] += z_shift
        world_points = (local_points @ parent_mat.T) + parent_pos
        return world_points
    
    def transform_single_point(self, point: np.ndarray, parent_pos: np.ndarray, parent_mat: np.ndarray) -> np.ndarray:
        if self.current_hip_pos is None:
            local_point = self.static_tf.transform_point(point)
            return (local_point @ parent_mat.T) + parent_pos
        relative_point = point - self.current_hip_pos
        local_point = relative_point @ self.static_tf.R.T
        if self.anchor_init:
            translation = np.array([self.static_tf.t[0], self.static_tf.t[1], self.current_hip_pos[2]], dtype=np.float64)
        else:
            translation = self.current_hip_pos.copy()
        for i in range(3):
            if self.static_tf.rigid[i]:
                translation[i] = self.static_tf.t[i]
        local_point = local_point + translation
        return (local_point @ parent_mat.T) + parent_pos