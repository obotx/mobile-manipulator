import numpy as np
import mujoco
from typing import Optional, List, Tuple, Dict, Any
from data.base import DataSource
from geometry.transform import StaticTransform

class LandmarkVisualizer:
    def __init__(self, data_source: DataSource, transform: Optional[StaticTransform] = None):
        self.data_source = data_source
        self.tf = transform
        self.hand_connections = [   (0,1),  (1,2),
                                    (2,3),  (3,4),
                                    (0,5),  (5,6),
                                    (6,7),  (7,8),
                                    (0,9),  (9,10),
                                    (10,11),(11,12),
                                    (0,13), (13,14),
                                    (14,15),(15,16),
                                    (0,17),(17,18),
                                    (18,19),(19,20),
                                    (5,9),(9,13),
                                    (13,17)]
        self.body_connections = [   ("nose","shoulder_L"), ("nose","shoulder_R"),
                                    ("shoulder_L","shoulder_R"),("shoulder_L","elbow_L"),
                                    ("elbow_L","wrist_L"), ("shoulder_R","elbow_R"),
                                    ("elbow_R","wrist_R"), ("shoulder_L","hip_L"),
                                    ("shoulder_R","hip_R"), ("hip_L","hip_R")
                                ]

    def step(self): self.data_source.step()

    def get_current_targets(self) -> Tuple[Optional[List[float]], Optional[List[float]], bool, bool, bool, bool]:
        frame = self.data_source.get_current_frame()
        if not frame: return None, None, False, False, False, False
        data = frame.data
        
        hand_a_global = None; hand_b_global = None; a_is_grab = False; b_is_grab = False
        
        # Hand A (from 'left_hand' in data)
        if getattr(frame, 'left_present') and 'left_hand' in data:
            w = data['left_hand'].get('wrist_m', {})
            if w and 'x' in w: 
                local_pos = np.array([w['x'], w['y'], w['z']], dtype=np.float64)
                hand_a_global = (self.tf.transform_point(local_pos) if self.tf else local_pos).tolist()
            a_is_grab = bool(data['left_hand'].get('is_grab', False))
            
        # Hand B (from 'right_hand' in data)
        if getattr(frame, 'right_present') and 'right_hand' in data:
            w = data['right_hand'].get('wrist_m', {})
            if w and 'x' in w: 
                local_pos = np.array([w['x'], w['y'], w['z']], dtype=np.float64)
                hand_b_global = (self.tf.transform_point(local_pos) if self.tf else local_pos).tolist()
            b_is_grab = bool(data['right_hand'].get('is_grab', False))
            
        # Returns: hand_a, hand_b, a_present, b_present, a_grab, b_grab
        return hand_a_global, hand_b_global, getattr(frame, 'left_present'), getattr(frame, 'right_present'), a_is_grab, b_is_grab

    def render(self, scene: mujoco.MjvScene):
        frame = self.data_source.get_current_frame()
        if not frame: return
        data = frame.data
        def apply_tf(pos): return self.tf.transform_point(pos).tolist() if self.tf else pos
        body_dict = {}
        for b in data.get('body_landmarks', []):
            name = b['joint_name']; raw_pos = [b['x'], b['y'], b['z']]
            body_dict[name] = apply_tf(raw_pos)
            self._add_sphere(scene, body_dict[name], radius=0.015, color=(0.0, 0.8, 1.0, 1.0))
        for j1, j2 in self.body_connections:
            if j1 in body_dict and j2 in body_dict: self._add_line(scene, body_dict[j1], body_dict[j2], width=0.006, color=(0.0, 0.6, 0.8, 1.0))
        for side, color_joint, color_link in [('left', (0.0, 1.0, 0.0, 1.0), (0.0, 0.6, 0.0, 1.0)), ('right', (1.0, 0.5, 0.0, 1.0), (0.8, 0.3, 0.0, 1.0))]:
            if getattr(frame, f'{side}_present') and f'{side}_hand' in data:
                hand = data[f'{side}_hand']
                joints_raw = [[j['x'], j['y'], j['z']] for j in hand.get('joints_m', [])]
                if len(joints_raw) == 21:
                    joints_transformed = self.tf.transform_points(joints_raw) if self.tf else np.array(joints_raw)
                    for i, j in self.hand_connections: self._add_line(scene, joints_transformed[i], joints_transformed[j], width=0.004, color=color_link)
                    for j_pos in joints_transformed: self._add_sphere(scene, j_pos, radius=0.008, color=color_joint)

    def _add_sphere(self, scene, pos, radius=0.01, color=(1.0, 0.0, 0.0, 1.0)):
        if scene.ngeom >= scene.maxgeom - 50: return
        pos_arr = np.array(pos, dtype=np.float32)
        if np.any(np.isnan(pos_arr)): return
        geom = scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(geom, type=mujoco.mjtGeom.mjGEOM_SPHERE, size=np.array([radius, radius, radius], dtype=np.float32), pos=pos_arr, mat=np.eye(3, dtype=np.float32).flatten(), rgba=np.array(color, dtype=np.float32))
        scene.ngeom += 1

    def _add_line(self, scene, start_pos, end_pos, width=0.005, color=(1.0, 1.0, 1.0, 1.0)):
        if scene.ngeom >= scene.maxgeom - 50: return
        start = np.ascontiguousarray(start_pos, dtype=np.float32); end = np.ascontiguousarray(end_pos, dtype=np.float32)
        if np.any(np.isnan(start)) or np.any(np.isnan(end)): return
        geom = scene.geoms[scene.ngeom]; center = (start + end) / 2.0; vec = end - start; length = np.linalg.norm(vec)
        if length < 1e-6: return
        direction = vec / length; z_axis = np.array([0.0, 0.0, 1.0], dtype=np.float32); axis = np.cross(z_axis, direction); axis_norm = np.linalg.norm(axis)
        if axis_norm < 1e-6: mat = np.array([-1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0], dtype=np.float32) if direction[2] < 0 else np.eye(3, dtype=np.float32).flatten()
        else:
            axis = axis / axis_norm; angle = np.arccos(np.clip(np.dot(z_axis, direction), -1.0, 1.0))
            c, s, t = np.cos(angle), np.sin(angle), 1 - np.cos(angle); x, y, z = axis
            mat = np.array([t*x*x+c, t*x*y-s*z, t*x*z+s*y, t*x*y+s*z, t*y*y+c, t*y*z-s*x, t*x*z-s*y, t*y*z+s*x, t*z*z+c], dtype=np.float32)
        mujoco.mjv_initGeom(geom, type=mujoco.mjtGeom.mjGEOM_BOX, size=np.array([width, width, length / 2.0], dtype=np.float32), pos=center, mat=mat, rgba=np.array(color, dtype=np.float32))
        scene.ngeom += 1