import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import cv2
import open3d as o3d
import numpy as np
import time
import argparse
import sys
import json
import os
import warnings
import ompl.base as ob
import ompl.geometric as og

from datetime import datetime
from core.manipulator.kinematics import MorphIManipulator
from typing import List, Tuple, Dict, Optional
from core.basic.pubsub import ZMQPubSub

warnings.filterwarnings("ignore", category=UserWarning)
o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Error)

def get_face_diagonals(faces: List[List[int]]) -> List[Tuple[int, int]]:
    diagonals = []
    for face in faces:
        if len(face) == 4:
            diagonals.append((face[0], face[2]))
            diagonals.append((face[1], face[3]))
    return diagonals

def get_box_edges_from_segment(
    start: np.ndarray,
    end: np.ndarray,
    radius_x: float,
    radius_y: float = None,
    edge_diagonal: bool = False
) -> List[Tuple[np.ndarray, np.ndarray]]:
    if radius_y is None:
        radius_y = radius_x

    vec = end - start
    length = np.linalg.norm(vec)
    if length < 1e-12:
        return []

    direction = vec / length
    center = (start + end) / 2.0

    if abs(direction[2]) < 0.99:
        up = np.array([0.0, 0.0, 1.0])
    else:
        up = np.array([1.0, 0.0, 0.0])
    local_x = np.cross(up, direction)
    local_x_norm = np.linalg.norm(local_x)
    if local_x_norm < 1e-12:
        local_x = np.array([1.0, 0.0, 0.0])
    else:
        local_x /= local_x_norm
    local_y = np.cross(direction, local_x)
    local_y /= np.linalg.norm(local_y) + 1e-12

    R = np.column_stack((local_x, local_y, direction))

    dx, dy, dz = radius_x, radius_y, length / 2.0
    local_corners = np.array([
        [-dx, -dy, -dz],
        [+dx, -dy, -dz],
        [+dx, +dy, -dz],
        [-dx, +dy, -dz],
        [-dx, -dy, +dz],
        [+dx, -dy, +dz],
        [+dx, +dy, +dz],
        [-dx, +dy, +dz]
    ])

    world_corners = (R @ local_corners.T).T + center

    edge_indices = [
        (0,1), (1,2), (2,3), (3,0),
        (4,5), (5,6), (6,7), (7,4),
        (0,4), (1,5), (2,6), (3,7)
    ]
    edges = [(world_corners[i], world_corners[j]) for i, j in edge_indices]

    if edge_diagonal:
        FACES = [
            [0,1,2,3], [4,5,6,7],
            [0,1,5,4], [2,3,7,6],
            [0,3,7,4], [1,2,6,5]
        ]
        for face in FACES:
            edges.append((world_corners[face[0]], world_corners[face[2]]))
            edges.append((world_corners[face[1]], world_corners[face[3]]))

    return edges

def get_3radius_diamond_edges(
    start: np.ndarray,
    end: np.ndarray,
    r0: float = 0.02,
    r1: float = 0.008,
    r2: float = 0.015,
    edge_diagonal: bool = False
):
    start = np.asarray(start, dtype=float)
    end = np.asarray(end, dtype=float)
    vec = end - start
    length = np.linalg.norm(vec)
    if length < 1e-8:
        return []

    direction = vec / length
    if abs(direction[2]) < 0.99:
        ref = np.array([0.0, 0.0, 1.0])
    else:
        ref = np.array([1.0, 0.0, 0.0])
    u = np.cross(ref, direction)
    u_norm = np.linalg.norm(u)
    if u_norm < 1e-12:
        u = np.array([1.0, 0.0, 0.0])
        v = np.cross(direction, u)
        v /= np.linalg.norm(v) + 1e-12
    else:
        u /= u_norm
        v = np.cross(direction, u)
        v /= np.linalg.norm(v) + 1e-12

    p0 = start
    p1 = (start + end) / 2.0
    p2 = end

    def square_corners(pos, r):
        return [
            pos + r * (u + v),
            pos + r * (-u + v),
            pos + r * (-u - v),
            pos + r * (u - v)
        ]

    corners0 = square_corners(p0, r0)
    corners1 = square_corners(p1, r1)
    corners2 = square_corners(p2, r2)
    all_points = corners0 + corners1 + corners2

    edges_indices = set()
    def add_edge(i, j):
        if i > j:
            i, j = j, i
        edges_indices.add((i, j))

    for base in [0, 4, 8]:
        for i in range(4):
            j = (i + 1) % 4
            add_edge(base + i, base + j)

    for i in range(4):
        add_edge(i, 4 + i)
        add_edge(4 + i, 8 + i)

    if edge_diagonal:
        DIAMOND_QUAD_FACES = [
            [0, 1, 2, 3],   # slice0
            [4, 5, 6, 7],   # slice1
            [8, 9, 10, 11], # slice2
            [0, 1, 5, 4],
            [1, 2, 6, 5],
            [2, 3, 7, 6],
            [3, 0, 4, 7],
            [4, 5, 9, 8],
            [5, 6, 10, 9],
            [6, 7, 11, 10],
            [7, 4, 8, 11]
        ]
        for face in DIAMOND_QUAD_FACES:
            add_edge(face[0], face[2])
            add_edge(face[1], face[3])

    return [(all_points[i], all_points[j]) for i, j in sorted(edges_indices)]

def get_all_robot_edges(
    waypoints_yaw: List[Tuple[float, float, float, float]],
    base_size: np.ndarray,
    vertical_size: np.ndarray,
    mobile2basearm: Dict[str, np.ndarray],
    basearm2verticalarm: Dict[str, np.ndarray],
    arm_left: 'MorphIManipulator',
    arm_right: 'MorphIManipulator',
    q_left: np.ndarray,
    q_right: np.ndarray,
    sampling: int = 1,
    scale: float = 1.0,  
    diamond_radii: Tuple[float, float, float] = (0.05, 0.08, 0.05),
    edge_diagonal: bool = True  
) -> List[Tuple[np.ndarray, np.ndarray]]:
    
    base_size_scaled = base_size * (scale)
    vertical_size_scaled = vertical_size * (scale)
    r0, r1, r2 = [r * (scale) for r in diamond_radii]

    LOCAL_CORNERS = np.array([
        [dx, dy, dz] 
        for dx in (-base_size_scaled[0], base_size_scaled[0])
        for dy in (-base_size_scaled[1], base_size_scaled[1])
        for dz in (-base_size_scaled[2], base_size_scaled[2])
    ])

    def get_box_edges(corners):
        edges = []
        n = len(corners)
        for i in range(n):
            for j in range(i + 1, n):
                if np.sum(corners[i] != corners[j]) == 1:
                    edges.append((i, j))
        return edges

    BOX_EDGES = get_box_edges(LOCAL_CORNERS)
    
    FACES_BASE = [
        [0, 1, 3, 2],  # x = -Lx
        [4, 5, 7, 6],  # x = +Lx
        [0, 1, 5, 4],  # y = -Ly
        [2, 3, 7, 6],  # y = +Ly
        [0, 2, 6, 4],  # z = -Lz
        [1, 3, 7, 5]   # z = +Lz
    ]

    FACES_ARM = [
        [0, 1, 2, 3],  # bottom
        [4, 5, 6, 7],  # top
        [0, 1, 5, 4],  # front
        [3, 2, 6, 7],  # back
        [0, 3, 7, 4],  # left
        [1, 2, 6, 5]   # right
    ]
    
    all_edges = []
    indices = [0] if sampling <= 1 else list(range(0, len(waypoints_yaw), sampling))
    if sampling <= 1:
        indices = list(range(len(waypoints_yaw)))

    for idx in indices:
        pose = waypoints_yaw[idx]
        x, y, z, yaw = pose
        base_center = np.array([x, y, z])
        c, s = np.cos(yaw), np.sin(yaw)
        Rz = np.array([[c, -s, 0],
                       [s,  c, 0],
                       [0,  0, 1]])

        # === 1. Scaled Base ===
        world_corners = (Rz @ LOCAL_CORNERS.T).T + base_center
        for i, j in BOX_EDGES:
            all_edges.append((world_corners[i], world_corners[j]))
        if edge_diagonal:
            diag_edges = get_face_diagonals(FACES_BASE)
            for i, j in diag_edges:
                all_edges.append((world_corners[i], world_corners[j]))

        Lx, Ly, Lz = 2 * vertical_size_scaled
        half_height = vertical_size[2]
        dx, dy, dz = Lx/2, Ly/2, Lz/2
        local_box_corners = np.array([
            [-dx,-dy,-dz],[+dx,-dy,-dz],[+dx,+dy,-dz],[-dx,+dy,-dz],
            [-dx,-dy,+dz],[+dx,-dy,+dz],[+dx,+dy,+dz],[-dx,+dy,+dz]
        ])
        edge_indices_box = [
            (0,1),(1,2),(2,3),(3,0),
            (4,5),(5,6),(6,7),(7,4),
            (0,4),(1,5),(2,6),(3,7)
        ]

        theta_left = q_left[3]
        theta_right = q_right[3]

        for side in ["left", "right"]:
            if side == "left":
                theta_arm = theta_left
                p_arm_base_local = mobile2basearm["left"]
            else:
                theta_arm = theta_right
                p_arm_base_local = mobile2basearm["right"]

            c_b, s_b = np.cos(yaw), np.sin(yaw)
            R_base = np.array([[c_b, -s_b, 0],
                            [s_b,  c_b, 0],
                            [0,    0,   1]])

            for seg_id in ["_1", "_2"]:
                key = side + seg_id
                if key not in basearm2verticalarm:
                    continue

                p_bottom_local = p_arm_base_local + basearm2verticalarm[key]
                p_center_local = p_bottom_local + np.array([0.0, 0.0, half_height])
                p_center_world = R_base @ p_center_local + base_center

                total_yaw = yaw + theta_arm
                c_t, s_t = np.cos(total_yaw), np.sin(total_yaw)
                R_total = np.array([[c_t, -s_t, 0],
                                    [s_t,  c_t, 0],
                                    [0,    0,   1]])

                world_box_corners = (R_total @ local_box_corners.T).T + p_center_world

                for i, j in edge_indices_box:
                    all_edges.append((world_box_corners[i], world_box_corners[j]))
                if edge_diagonal:
                    diag_edges = get_face_diagonals(FACES_ARM)
                    for i, j in diag_edges:
                        all_edges.append((world_box_corners[i], world_box_corners[j]))

        base_pose_world = (x, y, z, yaw)

        # ---- LEFT ARM ----
        pts_l = fk_points_to_world(base_pose_world, mobile2basearm["left"], arm_left, q_left)
        col_edges_l = get_box_edges_from_segment(
            pts_l["p_inner"], pts_l["wrist_base"],
            radius_x=0.02 * (scale),
            radius_y=0.04 * (scale),
            edge_diagonal=edge_diagonal
        )
        # Scale wrist segment length
        wrist_vec_l = pts_l["ee"] - pts_l["wrist_base"]
        scaled_ee_l = pts_l["wrist_base"] + wrist_vec_l * (scale)
        wrist_edges_l = get_3radius_diamond_edges(
            pts_l["wrist_base"], scaled_ee_l,
            r0=r0, r1=r1, r2=r2,
            edge_diagonal=edge_diagonal
        )
        all_edges.extend(col_edges_l)
        all_edges.extend(wrist_edges_l)

        # ---- RIGHT ARM ----
        pts_r = fk_points_to_world(base_pose_world, mobile2basearm["right"], arm_right, q_right)
        col_edges_r = get_box_edges_from_segment(
            pts_r["p_inner"], pts_r["wrist_base"],
            radius_x=0.02 * (scale),
            radius_y=0.04 * (scale),
            edge_diagonal=edge_diagonal
        )
        wrist_vec_r = pts_r["ee"] - pts_r["wrist_base"]
        scaled_ee_r = pts_r["wrist_base"] + wrist_vec_r * (scale)
        wrist_edges_r = get_3radius_diamond_edges(
            pts_r["wrist_base"], scaled_ee_r,
            r0=r0, r1=r1, r2=r2,
            edge_diagonal=edge_diagonal
        )
        all_edges.extend(col_edges_r)
        all_edges.extend(wrist_edges_r)

    return all_edges

def fk_points_to_world(base_pose, arm_offset, arm, q):
       pts = arm.fk_all_points(q)
       x_b, y_b, z_b, yaw_b = base_pose
       c, s = np.cos(yaw_b), np.sin(yaw_b)
       R2d = np.array([[c, -s], [s, c]])
       arm_xy = R2d @ arm_offset[:2]
       arm_z = z_b + arm_offset[2]
       world_pts = {}
       for k, pt in pts.items():
           pt_xy = R2d @ pt[:2]
           world_pts[k] = np.array([
               x_b + arm_xy[0] + pt_xy[0],
               y_b + arm_xy[1] + pt_xy[1],
               arm_z + pt[2]
           ])
       return world_pts

def height_to_rgb(z_vals, z_min=None, z_max=None):
    if z_min is None:
        z_min = z_vals.min() if z_vals.size > 0 else 0.0
    if z_max is None:
        z_max = z_vals.max() if z_vals.size > 0 else 1.0
    dz = z_max - z_min + 1e-8
    t = (z_vals - z_min) / dz
    t = np.clip(t, 0.0, 1.0)
    r = np.clip(1.5 * t - 0.5, 0, 1)
    g = np.clip(-2.0 * t * (t - 1), 0, 1)
    b = np.clip(1.5 - 1.5 * t, 0, 1)
    return np.stack([r, g, b], axis=-1)

class MapWorld:
    def __init__(self, 
                 mode="point", 
                 voxel_size=0.05, 
                 color_by="rgb", 
                 inflate_radius=0.0, 
                 min_height=-np.inf,
                 offscreen=False,
                 width=400, 
                 height=300,
                 load_path=None):
        assert mode in ("point", "voxel"), "Mode must be 'point' or 'voxel'"
        self.mode = mode
        self.voxel_size = voxel_size
        self.color_by = color_by
        self.inflate_radius = inflate_radius
        self.min_height = min_height
        self.inflate_color = np.array([0.6, 0.6, 1.0])
        
        self.running = True
        self.latest_pcd = None
        self.latest_pose = None
        self.save_map_path = None
        
        self.width=width
        self.height=height

        self.subscriber = ZMQPubSub(port=5555).create_subscriber()
        self.publisher  = ZMQPubSub(port=5556).create_publisher()
        self.offscreen = offscreen
    
        if self.offscreen :
            self.renderer = o3d.visualization.rendering.OffscreenRenderer(
                                width, height
                            )
            self.scene = self.renderer.scene
            self.scene.set_background([0.1, 0.1, 0.1, 1.0]) 
            self.mat = o3d.visualization.rendering.MaterialRecord()
            self.mat.shader = "defaultUnlit"
            self._setup_offscreen_camera()
        else :
            self.renderer = o3d.visualization.Visualizer()
            window_title = f"Loaded Map ({mode})" if load_path else f"Mapping ({mode}, {color_by}"
            self.renderer.create_window(window_name=window_title, width=width, height=height)
                                
        self._robot_edge_geometries = []
        
        self.subscriber.subscribe(
            "/sensor/pointcloud",
            lambda points: self.update_pcd(points)
        )
        self.subscriber.subscribe(
            "localization",
            lambda pose: self.update_pose(pose)
        )
        self.subscriber.start()    
            
        # Initialize geometry
        if self.mode == "point":
            self.geometry = o3d.geometry.PointCloud()
            self.geometry.points = o3d.utility.Vector3dVector(np.zeros((0, 3)))
            self.geometry.colors = o3d.utility.Vector3dVector(np.zeros((0, 3)))
        else:
            dummy_pcd = o3d.geometry.PointCloud()
            self.geometry = o3d.geometry.VoxelGrid.create_from_point_cloud(
                dummy_pcd, voxel_size=self.voxel_size
            )
        
        if self.offscreen :
            self.scene.add_geometry("voxel", self.geometry, self.mat)
        else :
            self.renderer.add_geometry(self.geometry, reset_bounding_box=False)
        
        self.robot_frame = None
        self.first_update = True
        self.total_voxels = 0
        
        # LOS visuals
        self.los_points = o3d.geometry.PointCloud()
        self.los_line = o3d.geometry.LineSet()
        if self.offscreen:
            self.scene.add_geometry("points", self.los_points, self.mat)
            self.scene.add_geometry("points", self.los_line, self.mat)
        else:
            self.renderer.add_geometry(self.los_points)
            self.renderer.add_geometry(self.los_line)
        
        if not load_path:
            if self.inflate_radius > 0:
                print(f"   Radius: {self.inflate_radius:.2f}m | Min height: {self.min_height:.2f}m")
                print(f"   → Inflated voxels shown in LIGHT BLUE")

    def _setup_offscreen_camera(self, bounds=None):
        if bounds is None and hasattr(self, 'geometry'):
            if self.mode == "point" and len(self.geometry.points) > 0:
                pts = np.asarray(self.geometry.points)
                bounds = (pts.min(axis=0), pts.max(axis=0))
            elif self.mode == "voxel" and hasattr(self.geometry, 'get_voxels'):
                voxels = self.geometry.get_voxels()
                if voxels:
                    centers = np.array([(v.grid_index + 0.5) * self.voxel_size for v in voxels])
                    bounds = (centers.min(axis=0), centers.max(axis=0))
        
        if bounds is not None:
            min_b, max_b = bounds
            center = (min_b + max_b) / 2  # lookat
            extent = np.linalg.norm(max_b - min_b)
            eye = center + np.array([extent, extent * 0.5, extent * 0.5])
            up = np.array([0, 0, 1])
        else:
            center = np.array([0, 0, 0])
            eye = np.array([2, -2, 2])
            up = np.array([0, 0, 1])
        
        aspect = self.width / self.height
        self.scene.camera.set_projection(
            60.0,                         
            aspect,                        
            0.1,                           
            100.0,                        
            o3d.visualization.rendering.Camera.FovType.Vertical,
        )
        
        # Set view direction
        self.scene.camera.look_at(
            center.tolist(),               # lookat
            eye.tolist(),                  # eye position
            up.tolist(),                   # up vector
        )
        
    def set_save_path(self, path_base):
        self.save_map_path = path_base
        print(f"\n MAP SAVING ENABLED: will save to 'map/map_<timestamp>/' on exit\n")

    def update_pcd(self, points):
        if points.size == 0 or points.shape[1] != 6:
            return

        xyz = points[:, :3].astype(np.float64)
        rgb_input = points[:, 3:] 

        if self.color_by == "rgb":
            rgb = rgb_input
        else:
            if xyz.shape[0] == 0:
                return
            z_vals = xyz[:, 2]
            rgb = height_to_rgb(z_vals)

        self.latest_pcd = (xyz, rgb.astype(np.float64))

    def update_pose(self, pose):
        if pose is not None and len(pose) == 4:
            self.latest_pose = np.array(pose, dtype=np.float64)

    def _update_robot_frame(self):
        if self.latest_pose is None:
            return
        x, y, z, yaw = self.latest_pose
        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)
        T = np.eye(4)
        T[:3, :3] = [[cos_yaw, -sin_yaw, 0],
                     [sin_yaw,  cos_yaw, 0],
                     [0,        0,       1]]
        T[:3, 3] = [x, y, z]

        if self.robot_frame is not None:
            self.renderer.remove_geometry(self.robot_frame, reset_bounding_box=False)
        self.robot_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3)
        self.robot_frame.transform(T)
        
        if self.offscreen:
            self.renderer.add_geometry("frame", self.robot_frame, self.mat)
        else:
            self.renderer.add_geometry(self.robot_frame, reset_bounding_box=False)

    def visualize_robot_edges_with_collision(self, edges: List[Tuple[np.ndarray, np.ndarray]]):
        if not edges:
            return

        all_points = []
        all_lines = []
        all_colors = []

        point_map = {}
        line_count = 0

        for start, end in edges:
            start = np.asarray(start)
            end = np.asarray(end)

            is_blocked = self.is_los_blocked(start, end)
            color = [1.0, 0.0, 0.0] if is_blocked else [0.0, 1.0, 0.0]  # red or green

            for pt in [start, end]:
                pt_key = tuple(np.round(pt, 8))
                if pt_key not in point_map:
                    point_map[pt_key] = len(all_points)
                    all_points.append(pt)

            i1 = point_map[tuple(np.round(start, 8))]
            i2 = point_map[tuple(np.round(end, 8))]
            all_lines.append([i1, i2])
            all_colors.append(color)

        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(np.array(all_points))
        line_set.lines = o3d.utility.Vector2iVector(np.array(all_lines))

        free_lines = []
        blocked_lines = []

        for (i1, i2), color in zip(all_lines, all_colors):
            if color == [1.0, 0.0, 0.0]:  
                blocked_lines.append([i1, i2])
            else:
                free_lines.append([i1, i2])

        if hasattr(self, '_collision_free_edges'):
            self.renderer.remove_geometry(self._collision_free_edges, reset_bounding_box=False)
        if hasattr(self, '_collision_blocked_edges'):
            self.renderer.remove_geometry(self._collision_blocked_edges, reset_bounding_box=False)

        if free_lines:
            free_set = o3d.geometry.LineSet()
            free_set.points = o3d.utility.Vector3dVector(np.array(all_points))
            free_set.lines = o3d.utility.Vector2iVector(np.array(free_lines))
            free_set.paint_uniform_color([0.0, 1.0, 0.0])  # green
            self.renderer.add_geometry(free_set, reset_bounding_box=False)
            self._collision_free_edges = free_set

        if blocked_lines:
            blocked_set = o3d.geometry.LineSet()
            blocked_set.points = o3d.utility.Vector3dVector(np.array(all_points))
            blocked_set.lines = o3d.utility.Vector2iVector(np.array(blocked_lines))
            blocked_set.paint_uniform_color([1.0, 0.0, 0.0])  # red
            self.renderer.add_geometry(blocked_set, reset_bounding_box=False)
            self._collision_blocked_edges = blocked_set

        self.renderer.update_renderer()

    def is_los_blocked(self, start, end):
        diff = np.array(end) - np.array(start)
        dist = np.linalg.norm(diff)

        step_size = self.voxel_size if self.mode == "voxel" else 0.01
        num_steps = int(np.ceil(dist / step_size))
        
        t_values = np.linspace(0, 1, num_steps)
        ray_points = start + t_values[:, np.newaxis] * diff
        
        if self.mode == "voxel":
            queries = o3d.utility.Vector3dVector(ray_points)
            hits = self.geometry.check_if_included(queries)
            return any(hits)
        else:
            if len(self.geometry.points) == 0:
                return False
            pcd_tree = o3d.geometry.KDTreeFlann(self.geometry)
            for pt in ray_points:
                _, _, dists = pcd_tree.search_knn_vector_3d(pt, 1)
                if dists[0] < (self.voxel_size * 1.5):
                    return True
            return False

    def _inflate_around_voxel(self, center_idx: np.ndarray) -> int:
        if self.inflate_radius <= 0:
            return 0

        inflate_voxels = int(np.ceil(self.inflate_radius / self.voxel_size))
        inflate_voxels_sq = (self.inflate_radius / self.voxel_size) ** 2
        added = 0

        for dx in range(-inflate_voxels, inflate_voxels + 1):
            for dy in range(-inflate_voxels, inflate_voxels + 1):
                for dz in range(-inflate_voxels, inflate_voxels + 1):
                    if dx == 0 and dy == 0 and dz == 0:
                        continue
                    if dx*dx + dy*dy + dz*dz > inflate_voxels_sq:
                        continue

                    neighbor_idx = center_idx + np.array([dx, dy, dz])
                    neighbor_world = (neighbor_idx + 0.5) * self.voxel_size

                    if self.geometry.check_if_included(
                        o3d.utility.Vector3dVector([neighbor_world])
                    )[0]:
                        continue

                    inflate_voxel = o3d.geometry.Voxel()
                    inflate_voxel.grid_index = tuple(neighbor_idx.astype(int))
                    inflate_voxel.color = self.inflate_color
                    self.geometry.add_voxel(inflate_voxel)
                    added += 1

        return added

    def apply_inflation_to_voxel_grid(self):
        if self.mode != "voxel" or self.inflate_radius <= 0:
            return

        print(f" Applying inflation (radius={self.inflate_radius:.2f}m, min_height={self.min_height:.2f}m)...")
        original_voxels = list(self.geometry.get_voxels())
        inflated_count = 0

        for voxel in original_voxels:
            pt_world = (np.array(voxel.grid_index) + 0.5) * self.voxel_size
            if pt_world[2] < self.min_height:
                continue
            added = self._inflate_around_voxel(np.array(voxel.grid_index))
            inflated_count += added

        self.total_voxels += inflated_count
        print(f"    Added {inflated_count:,} inflated voxels")

    def load_map(self, folder_path):
        ply_path = os.path.join(folder_path, "map.ply")
        meta_path = os.path.join(folder_path, "metadata.json")
        
        if not os.path.exists(ply_path):
            raise FileNotFoundError(f"map.ply not found in {folder_path}")
        if not os.path.exists(meta_path):
            raise FileNotFoundError(f"metadata.json not found in {folder_path}")
        
        with open(meta_path, 'r') as f:
            meta = json.load(f)

        if self.mode == "voxel":
            saved_voxel_size = meta.get("voxel_size", self.voxel_size)
            if abs(saved_voxel_size - self.voxel_size) > 1e-6:
                print(f" Overriding voxel_size: {self.voxel_size:.4f} → {saved_voxel_size:.4f} (from metadata)")
            self.voxel_size = saved_voxel_size 
        
        pcd = o3d.io.read_point_cloud(ply_path)
        if not pcd.has_points():
            raise ValueError("Loaded point cloud is empty")
        
        if self.mode == "point":
            self.geometry = pcd
        else:  # voxel
            points = np.asarray(pcd.points)
            colors = np.asarray(pcd.colors)
            
            self.geometry = o3d.geometry.VoxelGrid()
            self.geometry.voxel_size = self.voxel_size 
            
            for pt, col in zip(points, colors):
                idx = self.geometry.get_voxel(pt)
                voxel = o3d.geometry.Voxel()
                voxel.grid_index = tuple(idx)
                voxel.color = col
                self.geometry.add_voxel(voxel)
            
            self.apply_inflation_to_voxel_grid()
        
        self.renderer.clear_geometries()
        self.renderer.add_geometry(self.geometry, reset_bounding_box=True)
        self.renderer.add_geometry(self.los_points)
        self.renderer.add_geometry(self.los_line)
        self.first_update = False
        print(f" Loaded map from: {folder_path}")

    def _integrate_new_data(self, xyz, rgb):
        if xyz.shape[0] == 0:
            return
            
        if self.mode == "point":
            self.geometry.points.extend(o3d.utility.Vector3dVector(xyz))
            self.geometry.colors.extend(o3d.utility.Vector3dVector(rgb))
            self.renderer.update_geometry(self.geometry)
        else:
            queries = o3d.utility.Vector3dVector(xyz)
            included = self.geometry.check_if_included(queries)
            
            new_count = 0
            for i, is_included in enumerate(included):
                if is_included:
                    continue
                
                pt = xyz[i]
                orig_color = rgb[i]
                
                voxel = o3d.geometry.Voxel()
                voxel.grid_index = self.geometry.get_voxel(pt)
                voxel.color = orig_color
                self.geometry.add_voxel(voxel)
                new_count += 1
                
                if self.inflate_radius > 0 and pt[2] >= self.min_height:
                    added = self._inflate_around_voxel(np.array(voxel.grid_index))
                    new_count += added
            
            if new_count > 0:
                if self.offscreen:
                    self.scene.add_geometry("voxel", self.geometry, self.mat)
                else :
                    self.renderer.update_geometry(self.geometry)

                self.total_voxels += new_count
                
    def update_robot_edges(
            self,
            edges: List[Tuple[np.ndarray, np.ndarray]],
            scale: float = 1.0,
            offset: np.ndarray = None,
            show_original: bool = True,
            original_color: List[float] = [1.0, 0.65, 0.0],  # orange
            modified_color: List[float] = [0.0, 1.0, 1.0]      # cyan
        ):
            if not edges:
                return

            def build_lineset(edge_list, color):
                points = []
                lines = []
                point_map = {}
                for p1, p2 in edge_list:
                    p1_key = tuple(np.round(p1, 8))
                    p2_key = tuple(np.round(p2, 8))
                    if p1_key not in point_map:
                        point_map[p1_key] = len(points)
                        points.append(p1)
                    if p2_key not in point_map:
                        point_map[p2_key] = len(points)
                        points.append(p2)
                    lines.append([point_map[p1_key], point_map[p2_key]])
                
                ls = o3d.geometry.LineSet()
                ls.points = o3d.utility.Vector3dVector(np.array(points))
                ls.lines = o3d.utility.Vector2iVector(np.array(lines))
                ls.paint_uniform_color(color)
                return ls

            if show_original:
                original_lineset = build_lineset(edges, original_color)
                if hasattr(self, '_robot_edges_original'):
                    self.renderer.remove_geometry(self._robot_edges_original, reset_bounding_box=False)
                self._robot_edges_original = original_lineset
                self.renderer.add_geometry(self._robot_edges_original, reset_bounding_box=False)
                self.renderer.update_geometry(self._robot_edges_original)

            if scale != 1.0 or (offset is not None and np.any(offset)):
                offset_vec = np.array(offset) if offset is not None else np.zeros(3)
                modified_edges = []
                for p1, p2 in edges:
                    p1_mod = np.array(p1) * scale + offset_vec
                    p2_mod = np.array(p2) * scale + offset_vec
                    modified_edges.append((p1_mod, p2_mod))
                
                modified_lineset = build_lineset(modified_edges, modified_color)
                if hasattr(self, '_robot_edges_modified'):
                    self.renderer.remove_geometry(self._robot_edges_modified, reset_bounding_box=False)
                self._robot_edges_modified = modified_lineset
                self.renderer.add_geometry(self._robot_edges_modified, reset_bounding_box=False)
                self.renderer.update_geometry(self._robot_edges_modified)
            else:
                if hasattr(self, '_robot_edges_modified'):
                    self.renderer.remove_geometry(self._robot_edges_modified, reset_bounding_box=False)
                    del self._robot_edges_modified
                
    def save_map(self):
        if self.save_map_path is None:
            return
        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            base_dir = "map"
            save_dir = os.path.join(base_dir, f"map_{timestamp}")
            os.makedirs(save_dir, exist_ok=True)
            if self.mode == "point":
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(np.asarray(self.geometry.points))
                pcd.colors = o3d.utility.Vector3dVector(np.asarray(self.geometry.colors))
                
                filename = os.path.join(save_dir, "map.ply")
                o3d.io.write_point_cloud(filename, pcd, write_ascii=False, compressed=True)
                print(f"\n✅ Saved point cloud map: {filename}")
                print(f"   Points: {len(pcd.points):,}")
            else:
                voxels = self.geometry.get_voxels()
                if not voxels:
                    print("\n⚠️  No voxels to save!")
                    return
                voxel_indices = np.array([v.grid_index for v in voxels])
                voxel_colors = np.array([v.color for v in voxels])
                voxel_centers = (voxel_indices + 0.5) * self.voxel_size
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(voxel_centers)
                pcd.colors = o3d.utility.Vector3dVector(voxel_colors)
                geo_filename = os.path.join(save_dir, "map.ply")
                o3d.io.write_point_cloud(geo_filename, pcd, write_ascii=False, compressed=True)
                meta = {
                    "voxel_size": self.voxel_size,
                    "total_voxels": len(voxels),
                    "inflate_radius": self.inflate_radius,
                    "min_height": self.min_height,
                    "inflate_color": self.inflate_color.tolist(),
                    "mode": "voxel",
                    "color_by": self.color_by,
                    "timestamp": timestamp,
                    "original_points_estimate": self.total_voxels,
                    "saved_at": datetime.now().isoformat()
                }
                meta_filename = os.path.join(save_dir, "metadata.json")
                with open(meta_filename, 'w') as f:
                    json.dump(meta, f, indent=2)
                blue_mask = np.all(np.isclose(voxel_colors, self.inflate_color, atol=0.05), axis=1)
                inflated_count = np.sum(blue_mask)
                original_count = len(voxels) - inflated_count
                print(f"\n✅ Saved voxel map in folder: {save_dir}")
                print(f"   Geometry: map.ply")
                print(f"   Metadata: metadata.json")
                print(f"   Total voxels: {len(voxels):,} ({original_count:,} original + {inflated_count:,} inflated)")
                if self.inflate_radius > 0:
                    print(f"   Inflation: {self.inflate_radius:.2f}m radius (min_height={self.min_height:.2f}m)")
        except Exception as e:
            print(f"\n Failed to save map: {e}", file=sys.stderr)
            import traceback
            traceback.print_exc()
    
    def get_frame(self,):
        img = self.renderer.render_to_image()
        return np.asarray(img)

    def run(self):
        last_time = time.time()
        last_write_time = time.time()
        frame_count = 0
        
        while self.running:
            updated = False
            
            if self.latest_pcd is not None:
                xyz, rgb = self.latest_pcd
                self._integrate_new_data(xyz, rgb)
                self.latest_pcd = None
                updated = True
                
            # if self.latest_pose is not None:
            #     self._update_robot_frame()
            #     updated = True
            
            if updated:
                if self.first_update:
                    if self.offscreen is False :
                        self.renderer.reset_view_point(True)
                        self.first_update = False
                    
                frame_count += 1
                now = time.time()
                if now - last_time > 1.0:
                    if self.mode == "point":
                        total = len(self.geometry.points)
                        print(f"📊 FPS: {frame_count:.1f} | Points: {total:,}")
                    else:
                        print(f"📊 FPS: {frame_count:.1f} | Voxels: {self.total_voxels:,}")
                    frame_count = 0
                    last_time = now
                    
            if self.offscreen is False:
                if not self.renderer.poll_events():
                    self.running = False  

                self.renderer.update_renderer()
                cam = self.renderer.get_view_control().convert_to_pinhole_camera_parameters()
                self.renderer.reset_view_point(True)
                self.renderer.get_view_control().convert_from_pinhole_camera_parameters(cam)

            if self.offscreen:
                rgb = vis.get_frame()
                print(rgb)
                bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                
                cv2.imshow("Map World", bgr)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    vis.running = False
                    break
            
            time.sleep(0.001)
            
    def add_robot_edges(
        self,
        edges: List[Tuple[np.ndarray, np.ndarray]],
        color: List[float] = [1.0, 0.65, 0.0],
        name: str = None  
    ):
        if not edges:
            return

        points = []
        lines = []
        point_map = {}
        for p1, p2 in edges:
            p1_key = tuple(np.round(p1, 8))
            p2_key = tuple(np.round(p2, 8))
            if p1_key not in point_map:
                point_map[p1_key] = len(points)
                points.append(p1)
            if p2_key not in point_map:
                point_map[p2_key] = len(points)
                points.append(p2)
            lines.append([point_map[p1_key], point_map[p2_key]])

        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(np.array(points))
        line_set.lines = o3d.utility.Vector2iVector(np.array(lines))
        line_set.paint_uniform_color(color)

        self.renderer.add_geometry(line_set, reset_bounding_box=False)
        self._robot_edge_geometries.append(line_set)  
        self.renderer.update_geometry(line_set)

    def clear_robot_edges(self):
        for geom in self._robot_edge_geometries:
            self.renderer.remove_geometry(geom, reset_bounding_box=False)
        self._robot_edge_geometries.clear()
        
    def get_bounds(self) -> Tuple[np.ndarray, np.ndarray]:
        if self.mode == "point":
            if len(self.geometry.points) == 0:
                raise ValueError("Point cloud is empty")
            points = np.asarray(self.geometry.points)
        else:  # voxel mode
            voxels = self.geometry.get_voxels()
            if not voxels:
                raise ValueError("Voxel grid is empty")
            indices = np.array([v.grid_index for v in voxels])
            points = (indices + 0.5) * self.voxel_size

        min_bound = points.min(axis=0)
        max_bound = points.max(axis=0)
        return min_bound, max_bound

    def stop(self):
        self.running = False
        if self.offscreen:
            cv2.destroyAllWindows()  
            del self.renderer
        else:
            self.renderer.destroy_window()

 
       
class Planner:
    def __init__(
        self,
        map_world: 'MapWorld',
        manip_left: 'MorphIManipulator',
        manip_right: 'MorphIManipulator', 
        base_size: np.ndarray,
        vertical_size: np.ndarray,
        mobile2basearm: Dict[str, np.ndarray],
        basearm2verticalarm: Dict[str, np.ndarray],
        base_z:  float = 0.3,
        diamond_radii: Tuple[float, float, float] = (0.05, 0.08, 0.05),
        resolution: float = 0.1,
        timeout: float = 30.0
    ):
        if ob is None:
            raise RuntimeError("OMPL is required for planning but not installed.")
        
        self.map_ = map_world
        self.manip_left = manip_left
        self.manip_right = manip_right
        self.resolution = resolution
        self.timeout = timeout  
        self.base_z = base_z
        self.space = None
        self.si = None
        self.problem = None
        self.planner = None

        self.mobile2basearm = mobile2basearm
        self.basearm2verticalarm = basearm2verticalarm
        self.base_size = base_size
        self.vertical_size = vertical_size
        self.diamond_radii = diamond_radii
        self.min_lateral_dist_right = self.manip_right.min_lateral_dist
        self.min_lateral_dist_left = self.manip_left.min_lateral_dist

        self._define_problem_core()

    def _define_problem_core(self):
        try:
            min_bound, max_bound = self.map_.get_bounds()
        except ValueError as e:
            raise RuntimeError(f"Cannot plan: {e}")

        self.space = ob.RealVectorStateSpace(13)
        bounds = ob.RealVectorBounds(13)

        # Base
        bounds.setLow(0, min_bound[0]); bounds.setHigh(0, max_bound[0])
        bounds.setLow(1, min_bound[1]); bounds.setHigh(1, max_bound[1])
        bounds.setLow(2, -np.pi);      bounds.setHigh(2, np.pi)

        # Left arm
        left_bounds = [self.manip_left.bounds_h, self.manip_left.bounds_h,
                       self.manip_left.bounds_a, self.manip_left.bounds_theta,
                       self.manip_left.bounds_phi]
        for i, (low, high) in enumerate(left_bounds):
            bounds.setLow(3 + i, low); bounds.setHigh(3 + i, high)

        # Right arm
        right_bounds = [self.manip_right.bounds_h, self.manip_right.bounds_h,
                        self.manip_right.bounds_a, self.manip_right.bounds_theta,
                        self.manip_right.bounds_phi]
        for i, (low, high) in enumerate(right_bounds):
            bounds.setLow(8 + i, low); bounds.setHigh(8 + i, high)

        self.space.setBounds(bounds)
        self.si = ob.SpaceInformation(self.space)

        def is_state_valid(state):
            x, y, yaw = state[0], state[1], state[2]
            q_left = np.array([state[i] for i in range(3, 8)])
            q_right = np.array([state[i] for i in range(8, 13)])

            # Base bounds check
            if not (min_bound[0] <= x <= max_bound[0] and min_bound[1] <= y <= max_bound[1]):
                return False

            try:
                base_pose = (x, y, self.base_z, yaw)
                
                # Left arm EE
                pts_l = fk_points_to_world(base_pose, self.mobile2basearm["left"], self.manip_left, q_left)
                ee_left = pts_l["ee"]
                if np.sqrt(ee_left[0]**2 + ee_left[1]**2) < self.min_lateral_dist_left:
                    return False

                # Right arm EE
                pts_r = fk_points_to_world(base_pose, self.mobile2basearm["right"], self.manip_right, q_right)
                ee_right = pts_r["ee"]
                if np.sqrt(ee_right[0]**2 + ee_right[1]**2) < self.min_lateral_dist_right:
                    return False

            except Exception as e:
                print(f"FK error in lateral check: {e}", file=sys.stderr)
                return False

            # Collision check 
            try:
                edges = get_all_robot_edges(
                    waypoints_yaw=[base_pose],
                    base_size=self.base_size,
                    vertical_size=self.vertical_size,
                    mobile2basearm=self.mobile2basearm,
                    basearm2verticalarm=self.basearm2verticalarm,
                    arm_left=self.manip_left,
                    arm_right=self.manip_right,
                    q_left=q_left,
                    q_right=q_right,
                    scale=1.0,
                    diamond_radii=self.diamond_radii,
                    edge_diagonal=True
                )
            except Exception as e:
                print(f"Edge generation error: {e}", file=sys.stderr)
                return False

            for p1, p2 in edges:
                if self.map_.is_los_blocked(p1, p2):
                    return False

            return True

        self.si.setStateValidityChecker(ob.StateValidityCheckerFn(is_state_valid))
        self.si.setup()

        self.problem = ob.ProblemDefinition(self.si)
        self.problem.setOptimizationObjective(ob.PathLengthOptimizationObjective(self.si))

        self.planner = og.RRTConnect(self.si)
        self.planner.setIntermediateStates(True)
        self.planner.setRange(self.resolution)
        self.planner.setProblemDefinition(self.problem)
        self.planner.setup()

    def solve(self, start_state: np.ndarray, goal_state: np.ndarray):
        if start_state.shape != (13,) or goal_state.shape != (13,):
            raise ValueError("States must be 13-dimensional arrays")

        start_ompl = ob.State(self.space)
        goal_ompl = ob.State(self.space)
        for i in range(13):
            start_ompl[i] = start_state[i]
            goal_ompl[i] = goal_state[i]

        if not self.si.satisfiesBounds(start_ompl.get()):
            print("Start state is out of bounds!")
            return None
        if not self.si.satisfiesBounds(goal_ompl.get()):
            print("Goal state is out of bounds!")
            return None

        if not self.si.isValid(start_ompl.get()):
            print("Start state is in collision or violates constraints!")
            return None
        if not self.si.isValid(goal_ompl.get()):
            print("Goal state is in collision or violates constraints!")
            return None

        print("Start and goal states are valid.")

        self.problem.setStartAndGoalStates(start_ompl, goal_ompl)

        print(f"Solving 13D whole-body planning (timeout={self.timeout}s)...")
        solved = self.planner.solve(self.timeout)

        if solved and self.problem.hasExactSolution():
            path = self.problem.getSolutionPath()
            print(f"Found path with {path.getStateCount()} states.")

            base_traj = [(s[0], s[1], s[2]) for s in path.getStates()]
            left_traj = [np.array([s[i] for i in range(3, 8)]) for s in path.getStates()]
            right_traj = [np.array([s[i] for i in range(8, 13)]) for s in path.getStates()]

            return base_traj, left_traj, right_traj
        else:
            print("No solution found.")
            return None

 
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Real-time robot mapping visualizer or offline map viewer")
    parser.add_argument("--mode", choices=["point", "voxel"], default="voxel",
                        help="Mapping mode (required even when loading)")
    parser.add_argument("--color_by", choices=["rgb", "height"], default="rgb",
                        help="Color scheme (ignored in load mode)")
    parser.add_argument("--voxel_size", type=float, default=0.05,
                        help="Voxel size (ignored in load mode)")
    parser.add_argument("--transform", action="store_true",
                        help="Transform points using pose (ignored in load mode)")
    parser.add_argument("--port", type=int, default=5555,
                        help="ZMQ port (ignored in load mode)")
    parser.add_argument("--inflate_radius", type=float, default=0.0,
                        help="Inflation radius (ignored in load mode)")
    parser.add_argument("--min_height", type=float, default=0.1,
                        help="Min height for inflation (ignored in load mode)")
    parser.add_argument("--save_map", type=str, nargs='?', const="map", default=None,
                        help="Save map on exit (only in live mode)")
    parser.add_argument("--load_map", type=str, default=None,
                        help="Path to saved map folder (e.g., 'map/map_20260131_143022'). Disables live mode.")
    args = parser.parse_args()
    
    base_z = 0.3
    mobile2basearm = {
        "left":  np.array([0.15 + 0.01,  +(0.15 - 0.00465966), 0.127]),
        "right": np.array([0.15 + 0.01,  -(0.15 - 0.00465966), 0.127])
    }
    basearm2verticalarm = {
        "left_1":  np.array([-0.05, +0.04, 0]),
        "left_2":  np.array([+0.05, -0.04, 0]),
        "right_1": np.array([-0.05, +0.04, 0]),
        "right_2": np.array([+0.05, -0.04, 0]),
    }
    vertical_box_size = np.array([0.04, 0.02, 0.715])

    base_size = np.array([0.3, 0.3, 0.15])
    arm_left = MorphIManipulator("left")
    arm_right = MorphIManipulator("right")

    vis = MapWorld(
        mode=args.mode,
        voxel_size=args.voxel_size,
        color_by=args.color_by,
        inflate_radius=args.inflate_radius,
        min_height=args.min_height,
        load_path=args.load_map,
        offscreen=False
    )
    
    try:
        if args.load_map:
            vis.load_map(args.load_map)

            min_xyz, max_xyz = vis.get_bounds()
            print(f"Map bounds: X[{min_xyz[0]:.2f}, {max_xyz[0]:.2f}], "
                  f"Y[{min_xyz[1]:.2f}, {max_xyz[1]:.2f}], "
                  f"Z[{min_xyz[2]:.2f}, {max_xyz[2]:.2f}]")

            start_base = (0.0, 0.0, 0.0)
            goal_base = (3.0, -2.5, 1.54)

            q_left_start = np.array([0.70854689, 0.65545256, 0.19454412, 0.46312342, 0.48814012])
            q_right_start = np.array([0.5, 0.58, 0.3, 0.0, 0.0])
            q_left_goal = np.array([0.5, 0.58, 0.0, 0.6, 1.54])
            q_right_goal = np.array([0.5, 0.58, 0.0, 0.6, 0.0])

            start_state = np.array(list(start_base) + list(q_left_start) + list(q_right_start))
            goal_state = np.array(list(goal_base) + list(q_left_goal) + list(q_right_goal))

            planner = Planner(
                map_world=vis,
                manip_left=arm_left,
                manip_right=arm_right,
                base_size=base_size,
                vertical_size=vertical_box_size,
                mobile2basearm=mobile2basearm,
                basearm2verticalarm=basearm2verticalarm,
                timeout=25.0
            )

            result = planner.solve(start_state, goal_state)

            if result:
                base_traj, left_traj, right_traj = result
                print(f"Visualizing solution path ({len(base_traj)} states)...")

                # --- 1. First state (green) ---
                x, y, yaw = base_traj[0]
                q_l = left_traj[0]
                q_r = right_traj[0]
                edges_first = get_all_robot_edges(
                    waypoints_yaw=[(x, y, base_z, yaw)],
                    base_size=base_size,
                    vertical_size=vertical_box_size,
                    mobile2basearm=mobile2basearm,
                    basearm2verticalarm=basearm2verticalarm,
                    arm_left=arm_left,
                    arm_right=arm_right,
                    q_left=q_l,
                    q_right=q_r,
                    scale=1.0, 
                    diamond_radii=(0.02, 0.08, 0.02),
                    edge_diagonal=True
                )
                vis.add_robot_edges(edges_first, color=[0.0, 1.0, 0.0])  # green

                # --- 2. Last state (red) ---
                x, y, yaw = base_traj[-1]
                q_l = left_traj[-1]
                q_r = right_traj[-1]
                edges_last = get_all_robot_edges(
                    waypoints_yaw=[(x, y, base_z, yaw)],
                    base_size=base_size,
                    vertical_size=vertical_box_size,
                    mobile2basearm=mobile2basearm,
                    basearm2verticalarm=basearm2verticalarm,
                    arm_left=arm_left,
                    arm_right=arm_right,
                    q_left=q_l,
                    q_right=q_r,
                    scale=1.0,
                    diamond_radii=(0.02, 0.08, 0.02),
                    edge_diagonal=True
                )
                vis.add_robot_edges(edges_last, color=[1.0, 0.0, 0.0])  # red

                step = 20
                for i in range(step, len(base_traj) - step, step):  # skip first and last
                    x, y, yaw = base_traj[i]
                    q_l = left_traj[i]
                    q_r = right_traj[i]
                    edges = get_all_robot_edges(
                        waypoints_yaw=[(x, y, base_z, yaw)],
                        base_size=base_size,
                        vertical_size=vertical_box_size,
                        mobile2basearm=mobile2basearm,
                        basearm2verticalarm=basearm2verticalarm,
                        arm_left=arm_left,
                        arm_right=arm_right,
                        q_left=q_l,
                        q_right=q_r,
                        scale=1.0,
                        diamond_radii=(0.02, 0.08, 0.02),
                        edge_diagonal=True
                    )
                    vis.add_robot_edges(edges, color=[0.0, 0.0, 1.0])  # blue

                print(f"✅ Added trajectory visualization: green=start, red=goal, blue=intermediate.")
            vis.run()

        else:
            if args.save_map is not None:
                vis.set_save_path(args.save_map)
            vis.run()

    except KeyboardInterrupt:
        print("\n Interrupted by user (Ctrl+C)")
    except Exception as e:
        print(f"\n Fatal error: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()
    finally:
        if 'vis' in locals():
            vis.stop()
            if hasattr(vis, 'geometry'):
                if vis.mode == "point":
                    total = len(vis.geometry.points) if hasattr(vis.geometry, 'points') else 0
                    print(f"\n Final map: {total:,} points")
                else:
                    total_voxels = len(vis.geometry.get_voxels()) if hasattr(vis.geometry, 'get_voxels') else 0
                    print(f"\n Final map: {total_voxels:,} voxels")

        