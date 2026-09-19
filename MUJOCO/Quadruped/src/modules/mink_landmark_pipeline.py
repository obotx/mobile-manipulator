import numpy as np
from scipy.spatial.transform import Rotation
from typing import Optional, Tuple
from geometry.transform import StaticTransform, AnchoredTransform
from config import TeleopConfig
from typing import TYPE_CHECKING

from utils.skeleton import SKELETON_FORMATS

if TYPE_CHECKING:
    from control.mink_robot import MinkRobotInterface

SKELETON_TYPE="coco_wholebody_133"
_POSE_PARTS = SKELETON_FORMATS[SKELETON_TYPE]["parts"]
LEFT_SHOULDER_IDX, RIGHT_SHOULDER_IDX = _POSE_PARTS["shoulder"]["indices"]
LEFT_ELBOW_IDX = _POSE_PARTS["left_elbow"]["indices"][0]
RIGHT_ELBOW_IDX = _POSE_PARTS["right_elbow"]["indices"][0]

_HAND_PARTS = SKELETON_FORMATS[SKELETON_TYPE]["parts"]
LEFT_HAND_START = _HAND_PARTS["left_hand"]["indices"][0]
LEFT_HAND_END = _HAND_PARTS["left_hand"]["indices"][-1] + 1
RIGHT_HAND_START = _HAND_PARTS["right_hand"]["indices"][0]
RIGHT_HAND_END = _HAND_PARTS["right_hand"]["indices"][-1] + 1

WRIST_OFFSET = 0
INDEX_MCP_OFFSET = 5
MIDDLE_MCP_OFFSET = 9
PALM_LANDMARK_OFFSETS = [0, 1, 5, 9, 13, 17]

MIN_TRACKED_KEYPOINTS = 18

class MinkLandmarkPipeline:
    def __init__(self, config: TeleopConfig):
        self.config = config
        self.smoothed_pts: Optional[np.ndarray] = None

        self.static_tf_full = StaticTransform(
            x=0.0, y=0.0, z=config.anchor_height,
            roll=0.0, pitch=45.0, yaw=90.0, use_degrees=True
        )
        self.static_tf_upper = StaticTransform(
            x=0.3, y=0.0, z=config.anchor_height,
            roll=0.0, pitch=45.0, yaw=0.0, use_degrees=True
        )

        self.full_body_transform = AnchoredTransform(
            origin='foot',
            foot_on_ground=True,
            ground_level=0.0,
            foot_offset=0.0
        )

        self.upper_only_transform = AnchoredTransform(
            origin='shoulder',
            foot_on_ground=False,
            ground_level=0.0,
            foot_offset=0.0
        )

    def reset(self):
        self.smoothed_pts = None
        self.full_body_transform.reset()
        self.upper_only_transform.reset()

    def _transform(self, raw_pts: np.ndarray, parent_pos: np.ndarray, parent_mat: np.ndarray, mode: str) -> np.ndarray:
        if mode == "upper-only":
            parent_pts = self.upper_only_transform.transform(raw_pts, parent_pos, parent_mat)
            return self.static_tf_upper.transform_points(parent_pts)
        parent_pts = self.full_body_transform.transform(raw_pts, parent_pos, parent_mat)
        return self.static_tf_full.transform_points(parent_pts)

    def _scale_point_for_ik(self, point: np.ndarray, side: str, world_pts: np.ndarray, n_pts: int, robot: 'MinkRobotInterface') -> np.ndarray:
        side_idx = 0 if side == "left" else 1
        c = self.config

        shoulder_idx = LEFT_SHOULDER_IDX if side == "left" else RIGHT_SHOULDER_IDX
        elbow_idx = LEFT_ELBOW_IDX if side == "left" else RIGHT_ELBOW_IDX

        human_y_mid = c.human_y_mid[side_idx]
        if self.config.shoulder_to_y_mid and n_pts > shoulder_idx:
            human_y_mid = world_pts[shoulder_idx, 1]

        human_x_mid = c.human_x_mid[side_idx]
        if self.config.elbow_to_x_scale and n_pts > elbow_idx:
            shoulder = world_pts[shoulder_idx, :2]
            elbow = world_pts[elbow_idx, :2]
            human_x_mid = np.linalg.norm(elbow - shoulder)

        robot_y_mid = c.robot_y_mid[side_idx]
        if self.config.arm_to_y_mid:
            robot_y_mid = robot.get_robot_arm_y(side)

        out = point.copy()
        local_x = out[0] - human_x_mid
        out[0] = local_x * (c.x_scale_pos[side_idx] if local_x >= 0 else c.x_scale_neg[side_idx]) + c.robot_x_mid[side_idx]

        local_y = out[1] - human_y_mid
        out[1] = local_y * (c.y_scale_pos[side_idx] if local_y >= 0 else c.y_scale_neg[side_idx]) + robot_y_mid

        local_z = out[2] - c.human_z_mid[side_idx]
        out[2] = local_z * (c.z_scale_pos[side_idx] if local_z >= 0 else c.z_scale_neg[side_idx]) + c.robot_z_mid[side_idx]

        return out

    def _extract_palm(self, pts: np.ndarray, start_idx: int, end_idx: int, mode: str = "wrist") -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        if mode == "wrist":
            wrist = pts[start_idx + WRIST_OFFSET]
            middle_mcp = pts[start_idx + MIDDLE_MCP_OFFSET]
            if np.linalg.norm(wrist) < 1e-5 or np.linalg.norm(middle_mcp) < 1e-5:
                return None, None
            pos = (wrist + middle_mcp) / 2.0
        elif mode == "palm":
            palm_pts = [pts[start_idx + i] for i in PALM_LANDMARK_OFFSETS if np.linalg.norm(pts[start_idx + i]) > 1e-5]
            if len(palm_pts) < 3:
                return None, None
            pos = np.mean(palm_pts, axis=0)
        else:
            raise ValueError(f"Invalid mode: {mode}. Use 'wrist' or 'palm'")

        wrist = pts[start_idx + WRIST_OFFSET]
        index_mcp = pts[start_idx + INDEX_MCP_OFFSET]
        middle_mcp = pts[start_idx + MIDDLE_MCP_OFFSET]

        if np.linalg.norm(wrist) > 1e-5 and np.linalg.norm(index_mcp) > 1e-5 and np.linalg.norm(middle_mcp) > 1e-5:
            forward = middle_mcp - wrist
            lateral = index_mcp - wrist

            if np.linalg.norm(forward) > 1e-5 and np.linalg.norm(lateral) > 1e-5:
                forward = forward / np.linalg.norm(forward)
                lateral = lateral / np.linalg.norm(lateral)

                normal = np.cross(forward, lateral)

                if np.linalg.norm(normal) > 1e-5:
                    normal = normal / np.linalg.norm(normal)
                    right = np.cross(normal, forward)

                    R = np.column_stack((right, normal, forward))
                    quat_xyzw = Rotation.from_matrix(R).as_quat()
                    quat = np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]]).flatten()

                    return pos.flatten(), quat

        return pos.flatten(), None

    def _extract_palms(self, pts: np.ndarray, n_pts: int, mode: str = "wrist"):
        left_pos, left_quat = self._extract_palm(pts, LEFT_HAND_START, LEFT_HAND_END, mode)
        right_pos, right_quat = self._extract_palm(pts, RIGHT_HAND_START, RIGHT_HAND_END, mode)
        return (left_pos, left_quat), (right_pos, right_quat)

    def _smooth(self, world_pts: np.ndarray, n_pts: int) -> np.ndarray:
        if self.smoothed_pts is None or self.smoothed_pts.shape != world_pts.shape:
            self.smoothed_pts = world_pts.copy()
            return self.smoothed_pts

        alpha = self.config.smoothing_factor
        max_movement_per_frame = 0.08
        jitter_deadband = getattr(self.config, "jitter_deadband", 0.01)

        for i in range(n_pts):
            if np.linalg.norm(world_pts[i]) < 1e-5:
                continue
            current_pt = self.smoothed_pts[i]
            target_pt = world_pts[i]
            diff = target_pt - current_pt
            dist = np.linalg.norm(diff)

            if dist < jitter_deadband:
                continue

            if dist > max_movement_per_frame:
                target_pt = current_pt + (diff / dist) * max_movement_per_frame
            self.smoothed_pts[i] = (1.0 - alpha) * current_pt + alpha * target_pt
        return self.smoothed_pts

    def process(self, raw_pts: np.ndarray, n_pts: int, parent_pos: np.ndarray, parent_mat: np.ndarray, robot: 'MinkRobotInterface'):
        if n_pts < MIN_TRACKED_KEYPOINTS:
            return None, None, None

        world_pts = self._transform(raw_pts, parent_pos, parent_mat, self.config.tracking_mode)
        world_pts = self._smooth(world_pts, n_pts)
        (left_pos, left_quat), (right_pos, right_quat) = self._extract_palms(
            world_pts, n_pts, mode=self.config.hand_target_mode
        )
        left_palm = self._scale_point_for_ik(left_pos, "left", world_pts, n_pts, robot) if left_pos is not None else None
        right_palm = self._scale_point_for_ik(right_pos, "right", world_pts, n_pts, robot) if right_pos is not None else None
        return world_pts, (left_palm, left_quat), (right_palm, right_quat)
