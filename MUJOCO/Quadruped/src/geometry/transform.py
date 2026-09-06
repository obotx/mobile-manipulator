import numpy as np
from typing import Optional, List, Union

class StaticTransform:
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
        out[self.rigid] = self.t[self.rigid]
        return out

    def transform_points(self, points: np.ndarray) -> np.ndarray:
        out = (np.asarray(points, dtype=np.float64) @ self.R.T) + self.t
        if self.rigid.any():
            out[:, self.rigid] = self.t[self.rigid]
        return out

class AnchoredTransform:
    LEFT_ANKLE = 27; LEFT_HEEL = 29; LEFT_FOOT = 31
    RIGHT_ANKLE = 28; RIGHT_HEEL = 30; RIGHT_FOOT = 32
    FOOT_INDICES = [LEFT_ANKLE, LEFT_HEEL, LEFT_FOOT, RIGHT_ANKLE, RIGHT_HEEL, RIGHT_FOOT]

    LEFT_HAND_WRIST = 33; RIGHT_HAND_WRIST = 54
    LEFT_POSE_WRIST = 15; RIGHT_POSE_WRIST = 16

    def __init__(self, 
                 origin: str = 'hip',
                 lock_ori: Union[Optional[str], List[str]] = None,
                 anchor_init: bool = True,
                 foot_on_ground: bool = False,
                 ground_level: float = 0.0,
                 foot_offset: float = 0.0):
        
        if origin not in ('hip', 'shoulder', 'source'):
            raise ValueError("origin must be 'hip', 'shoulder', or 'source'")
            
        # Normalize lock_ori into a list
        if lock_ori is None:
            self.lock_ori_list = []
        elif isinstance(lock_ori, str):
            self.lock_ori_list = [lock_ori]
        else:
            self.lock_ori_list = list(lock_ori)
            
        for item in self.lock_ori_list:
            if item not in ('hip', 'shoulder'):
                raise ValueError("lock_ori items must be 'hip' or 'shoulder'")
                
        self.origin = origin
        self.anchor_init = anchor_init  
        self.foot_on_ground = foot_on_ground
        self.ground_level = ground_level
        self.foot_offset = foot_offset

        self.initial_hip_pos: Optional[np.ndarray] = None
        self.current_hip_pos: Optional[np.ndarray] = None
        self.current_shoulder_pos: Optional[np.ndarray] = None
        
        self.initial_ref_yaw: float = 0.0
        self.current_ref_yaw: float = 0.0
        self.initialized = False

    def reset(self):
        self.initial_hip_pos = None
        self.current_hip_pos = None
        self.current_shoulder_pos = None
        self.initial_ref_yaw = 0.0
        self.current_ref_yaw = 0.0
        self.initialized = False

    def _glue_hand_to_wrist(self, pts: np.ndarray, hand_start: int, hand_end: int, pose_wrist_idx: int) -> None:
        if hand_end > len(pts) or pose_wrist_idx >= len(pts):
            return
        offset = pts[pose_wrist_idx] - pts[hand_start]
        pts[hand_start:hand_end] += offset

    def _level_segment(self, pts: np.ndarray, seg_start: int, seg_end: int, pivot: np.ndarray) -> None:
        """Rotates the whole skeleton around `pivot` to make the segment perfectly horizontal (Z=0)."""
        vec = pts[seg_start] - pts[seg_end]
        vec_target = np.array([vec[0], vec[1], 0.0])
        
        norm_orig = np.linalg.norm(vec)
        norm_target = np.linalg.norm(vec_target)
        
        if norm_orig > 1e-5 and norm_target > 1e-5:
            u = vec / norm_orig
            v = vec_target / norm_target
            
            axis = np.cross(u, v)
            axis_norm = np.linalg.norm(axis)
            
            if axis_norm > 1e-5:
                axis = axis / axis_norm
                angle = np.arccos(np.clip(np.dot(u, v), -1.0, 1.0))
                
                K = np.array([
                    [0, -axis[2], axis[1]],
                    [axis[2], 0, -axis[0]],
                    [-axis[1], axis[0], 0]
                ])
                R_level = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)
                
                pts[:] = ((pts - pivot) @ R_level.T) + pivot

    def transform(self, points: np.ndarray, parent_pos: np.ndarray, parent_mat: np.ndarray) -> np.ndarray:
        if len(points) < 25:
            # If no lock_ori, just return raw points without parent transform
            if not self.lock_ori_list:
                return points.copy()
            return (points @ parent_mat.T) + parent_pos

        pts = points.copy()
        
        self._glue_hand_to_wrist(pts, self.LEFT_HAND_WRIST, 54, self.LEFT_POSE_WRIST)
        self._glue_hand_to_wrist(pts, self.RIGHT_HAND_WRIST, 75, self.RIGHT_POSE_WRIST)

        for align_target in self.lock_ori_list:
            if align_target == 'hip':
                hip_center = (pts[23] + pts[24]) / 2.0
                self._level_segment(pts, 23, 24, hip_center)
            elif align_target == 'shoulder':
                shoulder_center = (pts[11] + pts[12]) / 2.0
                self._level_segment(pts, 11, 12, shoulder_center)

        hip_center = (pts[23] + pts[24]) / 2.0
        shoulder_center = (pts[11] + pts[12]) / 2.0
        hip_vec_leveled = pts[23] - pts[24]
        shoulder_vec_leveled = pts[11] - pts[12]

        if self.origin == 'hip':
            current_origin_point = hip_center
        elif self.origin == 'shoulder':
            current_origin_point = shoulder_center
        else: # 'source'
            current_origin_point = np.zeros(3, dtype=np.float64)

        if self.lock_ori_list:
            ref_target = self.lock_ori_list[-1]
            ref_vec = hip_vec_leveled if ref_target == "hip" else shoulder_vec_leveled
            ref_angle = np.arctan2(ref_vec[1], ref_vec[0])
            self.current_ref_yaw = ref_angle

            if not self.initialized:
                self.initial_hip_pos = hip_center.copy()
                self.initial_ref_yaw = ref_angle
                self.initialized = True

            global_forward_angle = 0.0  
            delta_rotation = global_forward_angle - ref_angle
            
            c, s = np.cos(delta_rotation), np.sin(delta_rotation)
            Rz_body = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])

            # --- UPDATED LOGIC FOR HIP VS SHOULDER LOCK ---
            if ref_target == "shoulder":
                # Pivot exactly at the shoulder center
                pivot_point = shoulder_center
                # Rotate ONLY shoulders (11, 12), arms/hands/face (13-22), and extra hand joints (33-74)
                rotate_indices = list(range(11, 23)) + list(range(33, 75))
            else:
                # 'hip': Pivot around the main origin point (hip/shoulder/source)
                pivot_point = current_origin_point
                # Rotate ALL upper-body (0-22) and extra hand joints (33-74)
                rotate_indices = list(range(0, 23)) + list(range(33, 75))
            
            for idx in rotate_indices:
                if idx < len(pts):
                    pts[idx] = (Rz_body @ (pts[idx] - pivot_point)) + pivot_point
            # ----------------------------------------------
            
            pts = pts - current_origin_point
            parent_pts = (pts @ parent_mat.T) + parent_pos
        else:
            if not self.initialized:
                self.initial_hip_pos = hip_center.copy()
                self.initialized = True
            pts = pts - current_origin_point
            parent_pts = pts 

        self.current_hip_pos = (pts[23] + pts[24]) / 2.0
        self.current_shoulder_pos = (pts[11] + pts[12]) / 2.0

        if self.foot_on_ground and len(points) > max(self.FOOT_INDICES):
            foot_points = parent_pts[self.FOOT_INDICES]
            lowest_foot_z = np.min(foot_points[:, 2])
            target_foot_z = self.ground_level + self.foot_offset
            z_shift = target_foot_z - lowest_foot_z
            parent_pts[:, 2] += z_shift

        return parent_pts

    def transform_single_point(self, point: np.ndarray, parent_pos: np.ndarray, parent_mat: np.ndarray) -> np.ndarray:
        if self.current_hip_pos is None:
            if not self.lock_ori_list:
                return point.copy()
            return (point @ parent_mat.T) + parent_pos

        if self.origin == 'hip':
            origin_point = self.current_hip_pos
        elif self.origin == 'shoulder':
            origin_point = self.current_shoulder_pos
        else:
            origin_point = np.zeros(3, dtype=np.float64)
            
        point = point - origin_point
        
        if self.lock_ori_list:
            delta_yaw = self.current_ref_yaw - self.initial_ref_yaw
            c, s = np.cos(-delta_yaw), np.sin(-delta_yaw)
            Rz_inv = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
            point = Rz_inv @ point
            return (point @ parent_mat.T) + parent_pos
        else:
            return point