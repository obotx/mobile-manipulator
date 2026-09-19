import numpy as np
from typing import Optional, List, Union
from utils.skeleton import SKELETON_FORMATS

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
    def __init__(self,
                 skeleton_format: str = 'coco_wholebody_133',
                 origin: str = 'shoulder',
                 foot_on_ground: bool = False,
                 ground_level: float = 0.0,
                 foot_offset: float = 0.0):

        if origin not in ('hip', 'shoulder', 'foot', 'source'):
            raise ValueError("origin must be 'hip', 'shoulder', 'foot', or 'source'")

        self.skeleton_config = SKELETON_FORMATS.get(skeleton_format, SKELETON_FORMATS["coco_wholebody_133"])
        parts = self.skeleton_config["parts"]

        self.shoulder_indices = parts.get("shoulder", {}).get("indices", [11, 12])
        self.hip_indices = parts.get("hip", {}).get("indices", [23, 24])

        if "foot" in parts:
            self.foot_indices = parts["foot"]["indices"]
        else:
            left_foot = parts.get("left_foot", {}).get("indices", [])
            right_foot = parts.get("right_foot", {}).get("indices", [])
            self.foot_indices = left_foot + right_foot

        self.origin = origin
        self.foot_on_ground = foot_on_ground
        self.ground_level = ground_level
        self.foot_offset = foot_offset

        self.current_hip_pos: Optional[np.ndarray] = None
        self.current_shoulder_pos: Optional[np.ndarray] = None

        self.last_anchor: np.ndarray = np.zeros(3)

    def reset(self):
        self.current_hip_pos = None
        self.current_shoulder_pos = None
        self.last_anchor = np.zeros(3)

    @staticmethod
    def _is_valid(pt: np.ndarray) -> bool:
        return np.linalg.norm(pt) > 1e-3

    def _get_robust_center(self, pts: np.ndarray, indices: List[int], fallback: np.ndarray) -> np.ndarray:
        valid_pts = [pts[i] for i in indices if i < len(pts) and self._is_valid(pts[i])]
        if valid_pts:
            return np.mean(valid_pts, axis=0)
        return fallback if fallback is not None else np.zeros(3)

    def transform(self, points: np.ndarray, parent_pos: np.ndarray, parent_mat: np.ndarray) -> np.ndarray:
        if len(points) < 5:
            return points.copy()

        pts = points.copy()

        # Calculate ALL centers (but we'll only use the one we need)
        hip_center = self._get_robust_center(pts, self.hip_indices, self.current_hip_pos)
        shoulder_center = self._get_robust_center(pts, self.shoulder_indices, self.current_shoulder_pos)

        valid_foot_pts = [pts[i] for i in self.foot_indices if i < len(pts) and self._is_valid(pts[i])]
        foot_center = np.mean(valid_foot_pts, axis=0) if valid_foot_pts else np.zeros(3)

        # COMPLETELY ISOLATED ANCHOR LOGIC
        # Each origin mode ONLY looks at its own points - no cross-contamination

        if self.origin == 'shoulder':
            # STRICTLY use shoulders only
            if self._is_valid(shoulder_center):
                anchor = shoulder_center
            elif self.current_shoulder_pos is not None:
                anchor = self.current_shoulder_pos
            else:
                anchor = np.zeros(3)

        elif self.origin == 'hip':
            # STRICTLY use hips only
            if self._is_valid(hip_center):
                anchor = hip_center
            elif self.current_hip_pos is not None:
                anchor = self.current_hip_pos
            else:
                anchor = np.zeros(3)

        elif self.origin == 'foot':
            # STRICTLY use feet only
            if self._is_valid(foot_center):
                anchor = foot_center
            elif self.current_hip_pos is not None:
                anchor = self.current_hip_pos
            else:
                anchor = np.zeros(3)

        else:  # 'source'
            anchor = np.zeros(3, dtype=np.float64)

        # Save anchor for visualization
        self.last_anchor = anchor.copy()

        # "Glue" the skeleton to the anchor
        pts = pts - anchor

        # Apply parent transform
        parent_pts = (pts @ parent_mat.T) + parent_pos

        # Update state - ONLY update the relevant anchor's history
        if self.origin == 'shoulder' and self._is_valid(shoulder_center):
            self.current_shoulder_pos = shoulder_center
        elif self.origin == 'hip' and self._is_valid(hip_center):
            self.current_hip_pos = hip_center
        elif self.origin == 'foot' and self._is_valid(foot_center):
            self.current_hip_pos = hip_center  # Store as fallback

        # Keep feet on ground (only for foot origin)
        if self.origin == 'foot' and self.foot_on_ground and len(self.foot_indices) > 0 and len(points) > max(self.foot_indices):
            foot_points = parent_pts[self.foot_indices]
            valid_feet = [fp for fp in foot_points if self._is_valid(fp)]
            if valid_feet:
                lowest_foot_z = np.min([fp[2] for fp in valid_feet])
                target_foot_z = self.ground_level + self.foot_offset
                z_shift = target_foot_z - lowest_foot_z
                parent_pts[:, 2] += z_shift

        return parent_pts
