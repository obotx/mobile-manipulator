import numpy as np
import mujoco

def _get_body_id(name:str):
    pass

def _get_site_id(name:str):
    pass

def _get_act_id(name:str):
    pass

def _get_joint_id(name:str):
    pass

def _get_qvel_from_id(id:int):
    pass

def _get_qvel_from_name(name:str):
    pass

def _get_joint_qpos_addr(name:str):
    pass

# class MuJoCoCameraRenderer:
#     """Camera utility class for MuJoCo rendering, segmentation, and point cloud generation."""
    
#     def __init__(
#         self,
#         model: mujoco.MjModel,
#         data: mujoco.MjData,
#         height: int = 480,
#         width: int = 640,
#     ):
#         """
#         Initialize with MuJoCo model and data. Renderer is built internally.
        
#         Args:
#             model: MuJoCo model instance
#              MuJoCo data instance
#             height: Renderer height (default: 480)
#             width: Renderer width (default: 640)
#         """
#         self.model = model
#         self.data = data
#         self.renderer = mujoco.Renderer(model, height=height, width=width)

#     def get_camera_intrinsics(self, cam_id: int) -> np.ndarray:
#         """Compute camera intrinsic matrix from renderer and model parameters."""
#         H, W = self.renderer.height, self.renderer.width
#         fov = self.model.cam_fovy[cam_id]
#         theta = np.deg2rad(fov)
#         fx = W / 2 / np.tan(theta / 2)
#         fy = H / 2 / np.tan(theta / 2)
#         cx = (W - 1) / 2.0
#         cy = (H - 1) / 2.0
#         intr = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
#         return intr

#     def get_camera_extrinsic(self, cam_id: int) -> np.ndarray:
#         """Compute camera extrinsic matrix (world-to-camera) from data."""
#         cam_pos = self.data.cam_xpos[cam_id]
#         cam_rot = self.data.cam_xmat[cam_id].reshape(3, 3)
#         extr = np.eye(4)
#         extr[:3, :3] = cam_rot.T
#         extr[:3, 3] = cam_rot.T @ cam_pos
#         return extr

#     def get_masked_segmentation(self, segmentation: np.ndarray, geom_name: str) -> np.ndarray:
#         """Mask segmentation image to retain only specified geometry."""
#         geom_obj = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
#         mask = np.isin(segmentation, geom_obj)
#         segmentation[~mask] = 0
#         return segmentation

#     def render_rgbd(self, cam_id: int) -> tuple[np.ndarray, np.ndarray]:
#         """Render RGB and depth images for specified camera."""
#         self.renderer.update_scene(self.data, camera=cam_id)
#         self.renderer.enable_depth_rendering()
#         depth = self.renderer.render()
#         self.renderer.disable_depth_rendering()
#         rgb = self.renderer.render()
#         return rgb, depth

#     def get_segmentation_image(self, cam_id: int) -> np.ndarray:
#         """Render segmentation image for specified camera."""
#         self.renderer.enable_segmentation_rendering()
#         self.renderer.update_scene(self.data, camera=cam_id)
#         seg = self.renderer.render()
#         segmentation = seg[:, :, 0].astype(np.uint8)
#         self.renderer.disable_segmentation_rendering()
#         return segmentation

#     def rgbd_to_pointcloud(
#         self,
#         rgb: np.ndarray,
#         depth: np.ndarray,
#         intr: np.ndarray,
#         extr: np.ndarray,
#         depth_trunc: float = 20.0,
#     ) -> np.ndarray:
#         """
#         Convert RGB-D images to colored point cloud in world coordinates.
        
#         Args:
#             rgb: RGB image (H, W, 3) with values in [0, 255]
#             depth: Depth image (H, W)
#             intr: Camera intrinsic matrix (3, 3)
#             extr: Camera extrinsic matrix (4, 4)
#             depth_trunc: Maximum valid depth value
        
#         Returns:
#             Nx6 array of [x, y, z, r, g, b] points in world coordinates
#         """
#         H, W = self.renderer.height, self.renderer.width
#         cc, rr = np.meshgrid(np.arange(W), np.arange(H), sparse=True)
#         valid = (depth > 0) & (depth < depth_trunc)
#         z = np.where(valid, depth, np.nan)
#         x = np.where(valid, z * (cc - intr[0, 2]) / intr[0, 0], 0)
#         y = np.where(valid, z * (rr - intr[1, 2]) / intr[1, 1], 0)
#         xyz = np.vstack([e.flatten() for e in [x, y, z]]).T
#         color = rgb.transpose([2, 0, 1]).reshape((3, -1)).T / 255.0
#         mask = np.isnan(xyz[:, 2])
#         xyz = xyz[~mask]
#         color = color[~mask]
#         xyz_h = np.hstack([xyz, np.ones((xyz.shape[0], 1))])
#         xyz_t = (extr @ xyz_h.T).T
#         xyzrgb = np.hstack([xyz_t[:, :3], color])
#         return xyzrgb

#     def __del__(self):
#         """Clean up renderer on deletion."""
#         if hasattr(self, 'renderer') and self.renderer is not None:
#             self.renderer.close()
            
import numpy as np
import mujoco
    
def get_camera_intrinsics(renderer, model, cam_id):
    H, W = renderer.height, renderer.width
    fov = model.cam_fovy[cam_id]
    theta = np.deg2rad(fov)
    fx = W / 2 / np.tan(theta / 2)
    fy = H / 2 / np.tan(theta / 2)
    cx = (W - 1) / 2.0
    cy = (H - 1) / 2.0
    intr = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
    return intr

def get_camera_extrinsic(data, cam_id):
    cam_pos = data.cam_xpos[cam_id]
    cam_rot = data.cam_xmat[cam_id].reshape(3, 3)
    extr = np.eye(4)
    extr[:3, :3] = cam_rot.T
    extr[:3, 3] = cam_rot.T @ cam_pos
    return extr

def get_masked_segmentation(model, segmentation, geom_name):
    geom_obj = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
    mask = np.isin(segmentation, geom_obj)
    segmentation[~mask] = 0
    return segmentation

def render_rgbd(renderer, data, cam_id) -> tuple[np.ndarray, np.ndarray]:
    renderer.update_scene(data, camera=cam_id)
    renderer.enable_depth_rendering()
    depth = renderer.render()
    renderer.disable_depth_rendering()
    rgb = renderer.render()
    return rgb, depth

def get_segmentation_image(renderer, data, cam_id):
    renderer.enable_segmentation_rendering()
    renderer.update_scene(data, camera=cam_id)
    seg = renderer.render()
    segmentation = seg[:, :, 0].astype(np.uint8)
    renderer.disable_segmentation_rendering()
    return segmentation

def rgbd_to_pointcloud(rgb, depth, intr, extr, depth_trunc=40.0):
    h, w = depth.shape
    cc, rr = np.meshgrid(np.arange(w), np.arange(h))
    valid = (depth > 0) & (depth < depth_trunc)
    z = np.where(valid, depth, np.nan)
    x = (cc - intr[0, 2]) * z / intr[0, 0]
    y = (rr - intr[1, 2]) * z / intr[1, 1]
    xyz = np.stack([x, y, z], axis=-1).reshape(-1, 3)
    color = rgb.reshape(-1, 3) / 255.0
    mask = ~np.isnan(xyz[:, 2])
    xyz = xyz[mask]
    color = color[mask]
    xyz_h = np.hstack([xyz, np.ones((xyz.shape[0], 1))])
    xyz_world = (extr @ xyz_h.T).T[:, :3]
    return xyz_world, color

