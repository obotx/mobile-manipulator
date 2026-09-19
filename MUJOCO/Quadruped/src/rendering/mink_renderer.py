import numpy as np
import mujoco
from typing import Optional, List, Tuple
from config import TeleopConfig
from rendering.glfw_viewer import GlfwViewer
from utils.skeleton import SKELETON_FORMATS

class MinkRenderer:
    def __init__(self, config: TeleopConfig, model: mujoco.MjModel):
        self.config = config
        self.model = model

        self.skeleton_config = SKELETON_FORMATS.get("coco_wholebody_133", SKELETON_FORMATS["coco_wholebody_133"])
        self.parts = self.skeleton_config["parts"]

        self.visible_parts = ["shoulder", "hip", "upper_body", "left_hand", "right_hand"]

        self.skeleton_connections: List[Tuple[int, int, Tuple[float, float, float, float], float]] = []
        self.point_styles: dict = {}
        self.VISIBLE_KEYPOINTS = set()

        scale_factor = 1.5
        size_multiplier = 0.003

        for part_name in self.visible_parts:
            part_info = self.parts[part_name]

            r, g, b = part_info["color"]
            rgba = (r / 255.0, g / 255.0, b / 255.0, 1.0)

            base_radius = part_info["radius"] * scale_factor * size_multiplier
            base_thickness = part_info["thickness"] * scale_factor * size_multiplier

            for idx in part_info["indices"]:
                self.point_styles[idx] = (rgba, base_radius)
                self.VISIBLE_KEYPOINTS.add(idx)

            for conn in part_info["skeleton"]:
                self.skeleton_connections.append((conn[0], conn[1], rgba, base_thickness))

        self.EYE_MAT_FLAT = np.eye(3).flatten()

        self.camera = mujoco.MjvCamera()
        self.camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        self.camera.distance = 3.0
        self.camera.azimuth = 135.0
        self.camera.elevation = -20.0
        self.camera.lookat[:] = [0.0, 0.0, 0.8]

        self.scene = mujoco.MjvScene(model, maxgeom=3000)
        self.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW.value] = 0
        self.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION.value] = 0
        self.scene.flags[mujoco.mjtRndFlag.mjRND_FOG.value] = 1
        self.opt = mujoco.MjvOption()
        self.opt.frame = mujoco.mjtFrame.mjFRAME_NONE

        self.viewer = GlfwViewer(width=config.viewer_width, height=config.viewer_height, title=config.viewer_title)
        self.viewer.setup(model, self.camera, self.scene, self.opt)

    def should_close(self) -> bool:
        return self.viewer.should_close()

    def render_frame(self, data: mujoco.MjData, world_pts: Optional[np.ndarray], n_pts: int, tracking_mode: str,
                     left_gesture: str = "unknown", right_gesture: str = "unknown",
                     robot_base_pos: Optional[np.ndarray] = None, valid_mask: Optional[np.ndarray] = None,
                     anchor_pos: Optional[np.ndarray] = None):

        scene = self.viewer.prepare_scene(data)

        if world_pts is not None and len(world_pts) > 0:
            self._draw_points(scene, world_pts, valid_mask, tracking_mode)
            self._draw_skeleton(scene, world_pts, valid_mask, tracking_mode)

            if self.config.show_gesture_indicators:
                if valid_mask is not None and 111 < len(valid_mask) and any(valid_mask[91:112]):
                    self._draw_gesture_indicator(scene, world_pts[91:112].mean(axis=0), left_gesture)
                if valid_mask is not None and 132 < len(valid_mask) and any(valid_mask[112:133]):
                    self._draw_gesture_indicator(scene, world_pts[112:133].mean(axis=0), right_gesture)

        if self.config.show_coordinate_axes and robot_base_pos is not None:
            self._draw_coordinate_axes(scene, robot_base_pos, scale=0.2)

        self.viewer.render_frame(scene)

    def _draw_anchor_sphere(self, scene, pos: np.ndarray):
        if scene.ngeom >= scene.maxgeom: return
        g = scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_SPHERE, size=(0.04, 0.0, 0.0), pos=pos, mat=self.EYE_MAT_FLAT, rgba=(1.0, 1.0, 0.0, 0.9))
        scene.ngeom += 1

    def _draw_gesture_indicator(self, scene, pos: np.ndarray, gesture: str):
        if scene.ngeom >= scene.maxgeom: return
        rgba = (0, 1, 0, 0.8) if gesture == "open" else (1, 0, 0, 0.8) if gesture == "close" else (1, 1, 1, 0.5)
        g = scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_SPHERE, size=(0.03, 0.0, 0.0), pos=pos + np.array([0, 0, 0.05]), mat=self.EYE_MAT_FLAT, rgba=rgba)
        scene.ngeom += 1

    def _draw_coordinate_axes(self, scene, origin: np.ndarray, scale: float = 0.1):
        for direction, color in [(np.array([1, 0, 0]), (1, 0, 0, 1)), (np.array([0, 1, 0]), (0, 1, 0, 1)), (np.array([0, 0, 1]), (0, 0, 1, 1))]:
            if scene.ngeom >= scene.maxgeom: break
            end_point = origin + direction * scale
            vec = end_point - origin
            length = np.linalg.norm(vec)
            if length < 1e-5: continue
            z_ax = vec / length
            x_ax = np.cross(z_ax, [0, 1, 0]) if np.linalg.norm(np.cross(z_ax, [0, 1, 0])) > 1e-5 else np.cross(z_ax, [1, 0, 0])
            x_ax /= np.linalg.norm(x_ax)
            y_ax = np.cross(z_ax, x_ax)
            mat = np.array([x_ax, y_ax, z_ax]).T.flatten()
            g = scene.geoms[scene.ngeom]
            mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_CAPSULE, size=(0.005, length/2.0, 0.0), pos=(origin + end_point)/2.0, mat=mat, rgba=color)
            scene.ngeom += 1

    def _draw_points(self, scene, world_pts: np.ndarray, valid_mask: Optional[np.ndarray], tracking_mode: str):
        for i in self.VISIBLE_KEYPOINTS:
            if i >= len(world_pts) or scene.ngeom >= scene.maxgeom:
                continue

            if valid_mask is not None and not valid_mask[i]:
                continue

            rgba, radius = self.point_styles.get(i, ((1.0, 1.0, 1.0, 1.0), 0.01))
            g = scene.geoms[scene.ngeom]
            mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_SPHERE, size=(radius, 0.0, 0.0), pos=world_pts[i], mat=self.EYE_MAT_FLAT, rgba=rgba)
            scene.ngeom += 1

    def _draw_skeleton(self, scene, world_pts: np.ndarray, valid_mask: Optional[np.ndarray], tracking_mode: str):
        for c1, c2, rgba, thickness in self.skeleton_connections:
            if c1 >= len(world_pts) or c2 >= len(world_pts) or scene.ngeom >= scene.maxgeom:
                continue

            if valid_mask is not None and (not valid_mask[c1] or not valid_mask[c2]):
                continue

            p1, p2 = world_pts[c1], world_pts[c2]
            vec = p2 - p1
            length = np.linalg.norm(vec)
            if length < 1e-5: continue

            z_ax = vec / length
            x_ax = np.cross(z_ax, [1,0,0]) if abs(z_ax[0]) < 0.9 else np.cross(z_ax, [0,1,0])
            x_ax /= np.linalg.norm(x_ax)
            y_ax = np.cross(z_ax, x_ax)
            mat = np.array([x_ax, y_ax, z_ax]).T.flatten()
            g = scene.geoms[scene.ngeom]
            mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_CAPSULE, size=(thickness, length/2.0, 0.0), pos=(p1+p2)/2.0, mat=mat, rgba=rgba)
            scene.ngeom += 1

    def close(self):
        self.viewer.close()
