#!/usr/bin/env python3
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import time
import bisect
import mujoco
import glfw
import numpy as np
import datetime
import cv2
import json
import csv
from scipy.optimize import minimize
import argparse
from modules.pubsub import IPCPubSub
import threading
import asyncio
import websockets

np.set_printoptions(suppress=True, precision=4)

class StaticTransform:
    def __init__(self, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0, use_degrees=False):
        self.t = np.array([x, y, z], dtype=np.float64)
        if use_degrees:
            roll = np.deg2rad(roll)
            pitch = np.deg2rad(pitch)
            yaw = np.deg2rad(yaw)
        cx, sx = np.cos(roll), np.sin(roll)
        cy, sy = np.cos(pitch), np.sin(pitch)
        cz, sz = np.cos(yaw), np.sin(yaw)
        Rx = np.array([[1.0,  0.0,  0.0], [0.0,  cx, -sx], [0.0,  sx,  cx]])
        Ry = np.array([[ cy, 0.0,  sy], [0.0, 1.0, 0.0], [-sy, 0.0,  cy]])
        Rz = np.array([[cz, -sz, 0.0], [sz,  cz, 0.0], [0.0, 0.0, 1.0]])
        self.R = Rz @ Ry @ Rx

    def transform_point(self, point):
        p = np.asarray(point, dtype=np.float64)
        return (self.R @ p) + self.t

    def transform_points(self, points):
        pts = np.asarray(points, dtype=np.float64)
        return (pts @ self.R.T) + self.t

class LandmarkVisualizer:
    def __init__(self, mode="csv", csv_path=None, ws_url="ws://localhost:9090", apply_tf=False, parent_frame="base_footprint", 
                 x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0, playback_rate=1.0):
        
        self.mode = mode
        self.csv_path = csv_path
        self.ws_url = ws_url
        self.parent_frame = parent_frame
        self.apply_tf = apply_tf
        self.playback_rate = playback_rate
        
        if self.apply_tf:
            self.tf = StaticTransform(x=x, y=y, z=z, roll=roll, pitch=pitch, yaw=yaw)
        else:
            self.tf = None
            
        self.hand_connections = [
            (0, 1),  (1, 2),  (2, 3),  (3, 4), (0, 5),  (5, 6),  (6, 7),  (7, 8),
            (0, 9),  (9, 10), (10, 11),(11, 12),(0, 13), (13, 14),(14, 15),(15, 16),
            (0, 17), (17, 18),(18, 19),(19, 20),(5, 9),  (9, 13), (13, 17),
        ]
        self.body_connections = [
            ("nose", "shoulder_L"), ("nose", "shoulder_R"), ("shoulder_L", "shoulder_R"),
            ("shoulder_L", "elbow_L"), ("elbow_L", "wrist_L"), ("shoulder_R", "elbow_R"),
            ("elbow_R", "wrist_R"), ("shoulder_L", "hip_L"), ("shoulder_R", "hip_R"), ("hip_L", "hip_R"),
        ]
        
        self.frames = []
        self.current_frame_idx = 0
        self._timestamps = []
        self._start_wall_time = None
        self._first_csv_ts = 0.0
        
        self.current_ws_frame = None
        self.ws_lock = threading.Lock()
        
        if self.mode == "ws":
            self._start_ws_client()
        else:
            self._load_csv()

    def _start_ws_client(self):
        try:
            import websockets
        except ImportError:
            print("[WS Client] Error: 'websockets' library is not installed. Please run: pip install websockets")
            return

        print(f"[WS Client] Starting WebSocket client thread for {self.ws_url}...")

        async def ws_client():
            while True:
                try:
                    async with websockets.connect(self.ws_url) as ws:
                        print(f"[WS Client] Connected to {self.ws_url}")
                        async for message in ws:
                            try:
                                data = json.loads(message)
                                frame = {
                                    'timestamp': data.get('t', 0.0),
                                    'left_present': data.get('left_hand', {}).get('present', False),
                                    'right_present': data.get('right_hand', {}).get('present', False),
                                    'data': data
                                }
                                with self.ws_lock:
                                    self.current_ws_frame = frame
                            except Exception as e:
                                print(f"[WS Client] Error parsing message: {e}")
                except Exception as e:
                    print(f"[WS Client] Connection error: {e}. Retrying in 3 seconds...")
                    await asyncio.sleep(3)

        def run_loop():
            asyncio.run(ws_client())
            
        self.ws_thread = threading.Thread(target=run_loop, daemon=True)
        self.ws_thread.start()

    def _load_csv(self):
        if not self.csv_path or not os.path.exists(self.csv_path):
            print(f"[WARN] Landmark CSV not found at '{self.csv_path}'. Visualization disabled.")
            return
        with open(self.csv_path, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                self.frames.append({
                    'timestamp': float(row['timestamp_sec']),
                    'left_present': row['left_hand_present'].strip().lower() == 'true',
                    'right_present': row['right_hand_present'].strip().lower() == 'true',
                    'data': json.loads(row['processed_json'])
                })
        print(f"[INFO] Loaded {len(self.frames)} landmark frames.")
        self._timestamps = [f['timestamp'] for f in self.frames]
        self._first_csv_ts = self._timestamps[0] if self._timestamps else 0.0

    def step(self):
        if self.mode == "ws":
            return 
            
        if not self.frames: return
        if self._start_wall_time is None:
            self._start_wall_time = time.perf_counter()
            return
        wall_elapsed = time.perf_counter() - self._start_wall_time
        sim_elapsed = wall_elapsed * self.playback_rate
        current_sim_ts = self._first_csv_ts + sim_elapsed
        idx = bisect.bisect_right(self._timestamps, current_sim_ts) - 1
        if idx >= len(self.frames):
            self._start_wall_time = time.perf_counter() 
            idx = 0
        elif idx < 0:
            idx = 0
        self.current_frame_idx = idx
    
    def get_current_targets(self):
        if self.mode == "ws":
            with self.ws_lock:
                frame = self.current_ws_frame
            if frame is None:
                return None, None, False, False, False, False
        else:
            if not self.frames:
                return None, None, False, False, False, False
            frame = self.frames[self.current_frame_idx]

        data = frame['data']
        left_target_global = None
        right_target_global = None
        left_is_grab = False
        right_is_grab = False
        
        if frame['left_present'] and 'left_hand' in data:
            w = data['left_hand'].get('wrist_m', {})
            if w and 'x' in w and 'y' in w and 'z' in w:
                local_pos = np.array([w['x'], w['y'], w['z']], dtype=np.float64)
                left_target_global = self.tf.transform_point(local_pos).tolist() if self.tf else local_pos.tolist()
            left_is_grab = bool(data['left_hand'].get('is_grab', False))
            
        if frame['right_present'] and 'right_hand' in data:
            w = data['right_hand'].get('wrist_m', {})
            if w and 'x' in w and 'y' in w and 'z' in w:
                local_pos = np.array([w['x'], w['y'], w['z']], dtype=np.float64)
                right_target_global = self.tf.transform_point(local_pos).tolist() if self.tf else local_pos.tolist()
            right_is_grab = bool(data['right_hand'].get('is_grab', False))
            
        # return  left_target_global, right_target_global, frame['left_present'],  frame['right_present'], left_is_grab,  right_is_grab
        return  right_target_global, left_target_global,  frame['right_present'], frame['left_present'], right_is_grab, left_is_grab

    def _add_sphere(self, scene, pos, radius=0.01, color=(1.0, 0.0, 0.0, 1.0)):
        if scene.ngeom >= scene.maxgeom - 50: return
        pos_arr = np.array(pos, dtype=np.float32)
        if np.any(np.isnan(pos_arr)): return
        geom = scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(geom, type=mujoco.mjtGeom.mjGEOM_SPHERE, size=np.array([radius, radius, radius], dtype=np.float32), pos=pos_arr, mat=np.eye(3, dtype=np.float32).flatten(), rgba=np.array(color, dtype=np.float32))
        scene.ngeom += 1

    def _add_line(self, scene, start_pos, end_pos, width=0.005, color=(1.0, 1.0, 1.0, 1.0)):
        if scene.ngeom >= scene.maxgeom - 50: return
        start = np.ascontiguousarray(start_pos, dtype=np.float32)
        end = np.ascontiguousarray(end_pos, dtype=np.float32)
        if np.any(np.isnan(start)) or np.any(np.isnan(end)): return
        geom = scene.geoms[scene.ngeom]
        center = (start + end) / 2.0
        vec = end - start
        length = np.linalg.norm(vec)
        if length < 1e-6: return
        direction = vec / length
        z_axis = np.array([0.0, 0.0, 1.0], dtype=np.float32)
        axis = np.cross(z_axis, direction)
        axis_norm = np.linalg.norm(axis)
        if axis_norm < 1e-6:
            mat = np.array([-1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0], dtype=np.float32) if direction[2] < 0 else np.eye(3, dtype=np.float32).flatten()
        else:
            axis = axis / axis_norm
            angle = np.arccos(np.clip(np.dot(z_axis, direction), -1.0, 1.0))
            c, s, t = np.cos(angle), np.sin(angle), 1 - np.cos(angle)
            x, y, z = axis
            mat = np.array([t*x*x+c, t*x*y-s*z, t*x*z+s*y, t*x*y+s*z, t*y*y+c, t*y*z-s*x, t*x*z-s*y, t*y*z+s*x, t*z*z+c], dtype=np.float32)
        mujoco.mjv_initGeom(geom, type=mujoco.mjtGeom.mjGEOM_BOX, size=np.array([width, width, length / 2.0], dtype=np.float32), pos=center, mat=mat, rgba=np.array(color, dtype=np.float32))
        scene.ngeom += 1

    def render(self, scene):        
        if self.mode == "ws":
            with self.ws_lock:
                frame = self.current_ws_frame
            if frame is None: 
                return
        else:
            if not self.frames: 
                return
            frame = self.frames[self.current_frame_idx]
            
        data = frame['data']
        
        def apply_tf(pos):
            return self.tf.transform_point(pos).tolist() if self.tf else pos
            
        body_dict = {}
        for b in data.get('body_landmarks', []):
            name = b['joint_name']
            raw_pos = [b['x'], b['y'], b['z']]
            body_dict[name] = apply_tf(raw_pos)
            self._add_sphere(scene, body_dict[name], radius=0.015, color=(0.0, 0.8, 1.0, 1.0))
        for j1_name, j2_name in self.body_connections:
            if j1_name in body_dict and j2_name in body_dict:
                self._add_line(scene, body_dict[j1_name], body_dict[j2_name], width=0.006, color=(0.0, 0.6, 0.8, 1.0))
                
        for side, color_joint, color_link in [('left', (0.0, 1.0, 0.0, 1.0), (0.0, 0.6, 0.0, 1.0)), 
                                               ('right', (1.0, 0.5, 0.0, 1.0), (0.8, 0.3, 0.0, 1.0))]:
            if frame[f'{side}_present'] and f'{side}_hand' in data:
                hand = data[f'{side}_hand']
                joints_raw = [[j['x'], j['y'], j['z']] for j in hand.get('joints_m', [])]
                if len(joints_raw) == 21:
                    joints_transformed = self.tf.transform_points(joints_raw) if self.tf else np.array(joints_raw)
                    for i, j in self.hand_connections:
                        self._add_line(scene, joints_transformed[i], joints_transformed[j], width=0.004, color=color_link)
                    for j_pos in joints_transformed:
                        self._add_sphere(scene, j_pos, radius=0.008, color=color_joint)

def quaternion_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return np.array([w, x, y, z])

def quaternion_to_matrix(q):
    w, x, y, z = q
    n = w*w + x*x + y*y + z*z
    if n < 1e-10:
        raise ValueError("Quaternion has near-zero norm")
    s = 2.0 / n
    wx, wy, wz = s * w * x, s * w * y, s * w * z
    xx, xy, xz = s * x * x, s * x * y, s * x * z
    yy, yz, zz = s * y * y, s * y * z, s * z * z
    R = np.array([
        [1.0 - (yy + zz),        xy - wz,        xz + wy],
        [       xy + wz, 1.0 - (xx + zz),        yz - wx],
        [       xz - wy,        yz + wx, 1.0 - (xx + yy)]
    ])
    return R

def quaternion_inverse(q):
    q_conj = quaternion_conjugate(q)
    norm_sq = np.dot(q, q)
    return q_conj / norm_sq

def rotate_quaternion(quat, axis, angle):
    angle_rad = np.deg2rad(angle)
    axis = axis / np.linalg.norm(axis)
    cos_half = np.cos(angle_rad / 2)
    sin_half = np.sin(angle_rad / 2)
    delta_quat = np.array([cos_half, sin_half * axis[0], sin_half * axis[1], sin_half * axis[2]])
    new_quat = quaternion_multiply(quat, delta_quat)
    new_quat /= np.linalg.norm(new_quat)
    return new_quat

def quaternion_conjugate(q):
    w, x, y, z = q
    return np.array([w, -x, -y, -z])

def quaternion_rotate_vector(quat, vec):
    vec_quat = np.array([0.0, vec[0], vec[1], vec[2]])
    qv = quaternion_multiply(quat, vec_quat)
    rotated_quat = quaternion_multiply(qv, quaternion_conjugate(quat))
    return rotated_quat[1:]


class ParallelRobot:
    DAMPING = 8e-4
    DT = 0.002
        
    base_integral_1 = 0 
    base_prev_error_1 = 0
    base_integral_2 = 0 
    base_prev_error_2 = 0

    r = 0.1  
    D = 0.55   

    mobile_dot = np.zeros(4)
    target_vel = np.zeros(4)
    command = np.zeros(4)

    integral_x = 0.0
    integral_y = 0.0
    integral_yaw = 0.0
    prev_delta_x = 0.0
    prev_delta_y = 0.0
    prev_delta_yaw = 0.0
    
    JOINT_NAMES = [
        "ColumnLeftBearingJoint_1", "ColumnRightBearingJoint_1", "ArmLeftJoint_1", "BaseJoint_1",
        "ColumnLeftBearingJoint_2", "ColumnRightBearingJoint_2", "ArmLeftJoint_2", "BaseJoint_2",
    ]
    ACTUATOR_NAMES = [
        "ColumnLeftBearingJointMotor_1", "ColumnRightBearingJointMotor_1", "ArmLeftJointMotor_1", "BaseJointMotor_1",
        "ColumnLeftBearingJointMotor_2", "ColumnRightBearingJointMotor_2", "ArmLeftJointMotor_2", "BaseJointMotor_2",
    ]
    
    GRIPPER_ACT_LEFT = [
        "finger_c_joint_1_1", "finger_c_joint_2_1", "finger_c_joint_3_1", "finger_b_joint_1_1",
        "finger_b_joint_2_1", "finger_b_joint_3_1", "finger_a_joint_1_1", "finger_a_joint_2_1",
        "finger_a_joint_3_1", "palm_finger_c_joint_1", "palm_finger_b_joint_1", "wrist_X_1",
        "wrist_Y_1", "wrist_Z_1", "HandBearing_1"
    ]
    
    GRIPPER_ACT_RIGHT = [
        "finger_c_joint_1_2", "finger_c_joint_2_2", "finger_c_joint_3_2", "finger_b_joint_1_2",
        "finger_b_joint_2_2", "finger_b_joint_3_2", "finger_a_joint_1_2", "finger_a_joint_2_2",
        "finger_a_joint_3_2", "palm_finger_c_joint_2", "palm_finger_b_joint_2", "wrist_X_2",
        "wrist_Y_2", "wrist_Z_2", "HandBearing_2"
    ]
        
    def __init__(self, path: str, run_mode: str, record: bool, landmark_csv: str = None, 
                 playback_rate: float = 1.0, data_mode: str = "csv", ws_url: str = "ws://localhost:9090"):
        self.model = mujoco.MjModel.from_xml_path(path)
        self.data = mujoco.MjData(self.model)
        self.reset("home")
        self._target_lock = threading.Lock()
        self._initialize_ids()
        self._initialize_arrays()
        try:
            base_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "base_footprint")
            arm_l_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "Arm_1")
            arm_r_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "Arm_2")
            
            self.reset("home")
            mujoco.mj_step(self.model, self.data, nstep=1)
            
            pos_base = self.data.xpos[base_id]
            pos_l = self.data.xpos[arm_l_id]
            pos_r = self.data.xpos[arm_r_id]
            
            self.ik_arm_offset_left     = (pos_l - pos_base)
            self.ik_arm_offset_left[2]  =self.ik_arm_offset_left[2] + 0.0543365
            self.ik_arm_offset_right    = (pos_r - pos_base)
            self.ik_arm_offset_right[2] = self.ik_arm_offset_right[2] + 0.0543365
            self.ik_arm_offset_right[1] = self.ik_arm_offset_right[1] - 0.01
            print(f"[INFO] Auto-calibrated IK arm offsets:")
            print(f"       Left:  {self.ik_arm_offset_left.round(4)}")
            print(f"       Right: {self.ik_arm_offset_right.round(4)}")
        except Exception as e:
            print(f"[WARN] Could not auto-calibrate arm offsets: {e}")
            self.ik_arm_offset_left = np.array([0.16,   0.15 - 0.00465966,    0.158566 + 0.127 + 0.0542705])
            self.ik_arm_offset_right = np.array([0.16,        -(0.16 - 0.00465966),   0.158566 + 0.127 + 0.0896705])
        
        self._terminate = False  
        self.paused = False 
        self.run_mode = run_mode.lower()
        self.record = record
        self.current_ctrl = np.zeros(len(self.ACTUATOR_NAMES))
        
        self.camera = mujoco.MjvCamera()
        self.camera.distance = 5.0         
        self.camera.azimuth = 90            
        self.camera.elevation = -45         
        self.camera.lookat[:] = [0, 0, 0]
        self.use_ik = False 
        self.direct_arm_commands = np.concatenate([self.data.ctrl[self.actuator_ids[0:3]]/100, 
                                                   [0], 
                                                   self.data.ctrl[self.actuator_ids[4:7]]/100, 
                                                   [0]])

        if self.run_mode == "glfw":
            if not glfw.init():
                raise RuntimeError("GLFW failed to initialize")
            self.window = glfw.create_window(1023, 1080, "MORPH-I Simulation", None, None)
            if not self.window:
                glfw.terminate()
                raise RuntimeError("GLFW failed to create window")
            glfw.make_context_current(self.window)
            self.ctx = mujoco.MjrContext(self.model, mujoco.mjtFontScale.mjFONTSCALE_150)
            self.viewport = mujoco.MjrRect(0, 0, 1023, 1020)
            
            self.scene = mujoco.MjvScene(self.model, maxgeom=10000)
            self.opt = mujoco.MjvOption()
            

            self.camera.type = mujoco.mjtCamera.mjCAMERA_FREE

            glfw.set_key_callback(self.window, self.on_key)
            glfw.set_cursor_pos_callback(self.window, self._cursor_pos_callback)
            glfw.set_mouse_button_callback(self.window, self._mouse_button_callback)
            glfw.set_scroll_callback(self.window, self._scroll_callback)
            
            self._last_mouse_x = 0
            self._last_mouse_y = 0
            self._mouse_left_pressed = False
            self._mouse_right_pressed = False
            self._mouse_middle_pressed = False
            
        if self.run_mode == "cv":
            self.renderer_top = mujoco.Renderer(self.model, height=640, width=1024)
            self.model.vis.global_.offheight = 640
            self.model.vis.global_.offwidth = 1024
        
        self.current_waypoint_idx = 0
        self.time_at_current_waypoint = 0.0
        self.speed = 3  
        self.angular_speed = 3  
        self.grab_time = 0.0
        self.grab_hold_duration = 0.5
        self.grab_move_speed = 0.2
        self.progress = 0
        
        self.h1, self.h2, self.l1, self.l2 = 0, 0, 0, 0
        self.top_video_writer = None
        self.pov_video_writer = None

        self.ipc = IPCPubSub()
        self.subscriber = self.ipc.create_subscriber()
        self.subscriber.subscribe("target_base", self._on_target_base)
        self.subscriber.subscribe("target_left", self._on_target_left)
        self.subscriber.subscribe("target_right", self._on_target_right)
        self.subscriber.subscribe("ik_mode", self._on_ik_mode)
        self.subscriber.subscribe("u_control", self._on_arm_control)
        self.subscriber.start()
        self.landmark_viz = LandmarkVisualizer(
            mode=data_mode,
            csv_path=landmark_csv,
            ws_url=ws_url,
            apply_tf=True,
            parent_frame="base_footprint", 
            x=0.3, y=0.0, z=1.2, roll=0.0, pitch=0.0, yaw=3.14159265,
            playback_rate=playback_rate
        )

    def _initialize_ids(self):
        self.arm_link_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "Arm")
        self.end_effector_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "Gripper_Link1")
        self.base_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "base_footprint")
        self.ee_site_1_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "ee_site_1")
        self.ee_site_2_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "ee_site_2")
        self.dof_ids = np.array([self.model.joint(name).id for name in self.JOINT_NAMES])
        self.actuator_ids = np.array([self.model.actuator(name).id for name in self.ACTUATOR_NAMES])
        self.gripper_ids_left = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name) for name in self.GRIPPER_ACT_LEFT]
        self.gripper_ids_right = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name) for name in self.GRIPPER_ACT_RIGHT]
        
        self.qpos_indices = []
        self.qvel_indices = []

        for name in self.JOINT_NAMES:
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            qpos_adr = self.model.jnt_qposadr[joint_id]
            dof_adr = self.model.jnt_dofadr[joint_id]
            joint_type = self.model.jnt_type[joint_id]

            if joint_type in [3, 2]:
                self.qpos_indices.append(qpos_adr)
                self.qvel_indices.append(dof_adr)
            elif joint_type == 1:
                self.qpos_indices.extend(range(qpos_adr, qpos_adr + 3))
                self.qvel_indices.extend(range(dof_adr, dof_adr + 3))
            elif joint_type == 0:
                self.qpos_indices.extend(range(qpos_adr, qpos_adr + 6))
                self.qvel_indices.extend(range(dof_adr, dof_adr + 6))

        self.qpos_indices = np.array(self.qpos_indices)
        self.qvel_indices = np.array(self.qvel_indices)

        l0, r0 = self.get_encoder()
        self.target_left = np.array(self.fk(l0[0], l0[1], l0[2], l0[3]))
        self.target_right = np.array(self.fk(r0[0], r0[1], r0[2], r0[3]))
        self.prev_target_left = np.array(self.fk(l0[0], l0[1], l0[2], l0[3]))
        self.prev_target_right = np.array(self.fk(r0[0], r0[1], r0[2], r0[3]))
        self.target_base = self.localization()

    def _initialize_arrays(self):
        self.jacp = np.zeros((3, self.model.nv))
        self.jacr = np.zeros((3, self.model.nv))
        self.error = np.zeros(6)
        self.error_pos = np.zeros(3)
        self.error_ori = np.zeros(3)
        self.site_quat = np.zeros(4)
        self.site_quat_conj = np.zeros(4)
        self.error_quat = np.zeros(4)
        
    def _on_ik_mode(self, msg):
        try:
            enabled = bool(msg)
            with self._target_lock:
                self.use_ik = enabled
            print(f"[INFO] IK mode: {'ENABLED' if enabled else 'DISABLED'}")
        except Exception as e:
            print(f"[ERROR] Invalid ik_mode message: {msg}, error: {e}")

    def _on_arm_control(self, msg):
        try:
            raw = np.array(msg, dtype=float)
            if raw.shape != (8,):
                raise ValueError(f"Expected 8 values, got {raw.size}")
            h_min, h_max = -75.0, 75.0   
            h_out_min, h_out_max = 0.0, 1.5  
            a_min, a_max = -30.0, 30.0    
            a_out_min, a_out_max = 0.0, 0.6
            def remap(value, in_min, in_max, out_min, out_max):
                return out_min + (value - in_min) * (out_max - out_min) / (in_max - in_min)
            l_h1 = np.clip(remap(raw[0], h_min, h_max, h_out_min, h_out_max), h_out_min, h_out_max)
            l_h2 = np.clip(remap(raw[1], h_min, h_max, h_out_min, h_out_max), h_out_min, h_out_max)
            l_a1 = np.clip(remap(raw[2], a_min, a_max, a_out_min, a_out_max), a_out_min, a_out_max)
            l_theta = np.deg2rad(raw[3])  
            r_h1 = np.clip(remap(raw[4], h_min, h_max, h_out_min, h_out_max), h_out_min, h_out_max)
            r_h2 = np.clip(remap(raw[5], h_min, h_max, h_out_min, h_out_max), h_out_min, h_out_max)
            r_a1 = np.clip(remap(raw[6], a_min, a_max, a_out_min, a_out_max), a_out_min, a_out_max)
            r_theta = np.deg2rad(raw[7])  
            mapped = np.array([l_h1, l_h2, l_a1, l_theta, r_h1, r_h2, r_a1, r_theta])
            with self._target_lock:
                self.direct_arm_commands = mapped.copy()
        except Exception as e:
            print(f"[ERROR] Invalid u_control message: {msg}, error: {e}")

    def _on_target_base(self, msg):
        try:
            arr = np.array(msg, dtype=float)
            if arr.shape != (3,):
                raise ValueError(f"Expected shape (3,), got {arr.shape}")
            with self._target_lock:
                self.target_base = arr.copy()
        except Exception as e:
            print(f"[PubSub] Invalid target_base message: {msg}, error: {e}")

    def _on_target_left(self, msg):
        try:
            arr = np.array(msg, dtype=float)
            if arr.shape != (3,):
                raise ValueError(f"Expected shape (3,), got {arr.shape}")
            with self._target_lock:
                self.target_left = arr.copy()
        except Exception as e:
            print(f"[PubSub] Invalid target_left message: {msg}, error: {e}")

    def _on_target_right(self, msg):
        try:
            arr = np.array(msg, dtype=float)
            if arr.shape != (3,):
                raise ValueError(f"Expected shape (3,), got {arr.shape}")
            with self._target_lock:
                self.target_right = arr.copy()
        except Exception as e:
            print(f"[PubSub] Invalid target_right message: {msg}, error: {e}")

    def reset(self, keyframe_name:str):
        key_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, keyframe_name)
        mujoco.mj_resetDataKeyframe(self.model, self.data, key_id)
        
    def get_keyframe(self, keyframe_name:str):
        def format_array(arr):
            return " ".join(f"{x:.6f}" for x in arr)
        print(f'<key\n    name="{keyframe_name}"')
        print(f'    qpos="  {format_array(self.data.qpos)}"')
        print(f'    qvel="  {format_array(self.data.qvel)}"')
        print(f'    ctrl="  {format_array(self.data.ctrl)}"')
        print('/>')
        
    def configure_model(self):
        self.model.opt.timestep = self.DT
        self.model.body_gravcomp[:] = True
        
    def get_joint_qpos_addr(self, joint_name):
        jnt_id = self.model.joint(joint_name).id
        return self.model.jnt_qposadr[jnt_id]
    
    def _add_target_sphere(self, scene, pos, radius=0.025, color=(1.0, 0.0, 0.0, 0.6)):
        """Draws a semi-transparent sphere to visualize IK targets."""
        if scene.ngeom >= scene.maxgeom - 50: 
            return
        pos_arr = np.array(pos, dtype=np.float32)
        if np.any(np.isnan(pos_arr)): 
            return
            
        geom = scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(
            geom, 
            type=mujoco.mjtGeom.mjGEOM_SPHERE, 
            size=np.array([radius, radius, radius], dtype=np.float32), 
            pos=pos_arr, 
            mat=np.eye(3, dtype=np.float32).flatten(), 
            rgba=np.array(color, dtype=np.float32)
        )
        scene.ngeom += 1

    def render_target_spheres(self, scene):
        if hasattr(self, 'target_left_global') and self.target_left_global is not None:
            self._add_target_sphere(scene, self.target_left_global, radius=0.025, color=(1.0, 0.0, 0.0, 0.6))
        if hasattr(self, 'target_right_global') and self.target_right_global is not None:
            self._add_target_sphere(scene, self.target_right_global, radius=0.025, color=(0.0, 0.5, 1.0, 0.6))

    def localization(self):
        x, y = self.data.xpos[self.base_id, 0], self.data.xpos[self.base_id, 1]
        w, xq, yq, zq = self.data.xquat[self.base_id]
        yaw = np.arctan2(2 * (w * zq + xq * yq), 1 - 2 * (yq**2 + zq**2))
        return np.array([x, y, yaw])
    
    def get_encoder(self):
        z1_left  = self.data.qpos[self.get_joint_qpos_addr("ColumnLeftBearingJoint_1")]
        z1_right = self.data.qpos[self.get_joint_qpos_addr("ColumnRightBearingJoint_1")]
        z2_left  = self.data.qpos[self.get_joint_qpos_addr("ColumnLeftBearingJoint_2")]
        z2_right = self.data.qpos[self.get_joint_qpos_addr("ColumnRightBearingJoint_2")]
        horizontal_1 = self.data.qpos[self.get_joint_qpos_addr("ArmLeftJoint_1")]
        horizontal_2 = self.data.qpos[self.get_joint_qpos_addr("ArmLeftJoint_2")]
        yaw_1 = self.data.qpos[self.get_joint_qpos_addr("BaseJoint_1")]
        yaw_2 = self.data.qpos[self.get_joint_qpos_addr("BaseJoint_2")]
        encoder_left = np.array([z1_left, z1_right, horizontal_1, yaw_1])
        encoder_right = np.array([z2_left, z2_right, horizontal_2, yaw_2])
        return encoder_left, encoder_right
        
    def send_command_arm(self, u_control):
        u_control = np.asarray(u_control)
        if u_control.shape != (len(self.actuator_ids),):
            raise ValueError(f"Control input shape {u_control.shape} does not match number of actuators ({len(self.actuator_ids)})")
        ctrl_ranges = self.model.actuator_ctrlrange[self.actuator_ids]  
        lo = ctrl_ranges[:, 0]
        hi = ctrl_ranges[:, 1]
        u_clipped = np.clip(u_control, lo, hi)
        self.data.ctrl[self.actuator_ids] = u_clipped

    def get_world_ee_pose(self, arm="left"):
        arm_body_name = "Arm_1" if arm == "left" else "Arm_2"
        try:
            arm_base_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, arm_body_name)
            arm_base_world = self.data.xpos[arm_base_id].copy()
        except Exception:
            print(f"[WARN] Could not find body '{arm_body_name}'. Falling back to manual offset.")
            arm_base_world = np.array([0.0, 0.0, 0.0])
        encoder_left, encoder_right = self.get_encoder()
        q_arm = encoder_left if arm == "left" else encoder_right
        ee_local = self.fk(*q_arm)
        if isinstance(ee_local, tuple) and ee_local[0] is None:
            return None  
        true_ee_local = np.array([-ee_local[0], -ee_local[1], ee_local[2]])
        theta = q_arm[3]
        c, s = np.cos(theta), np.sin(theta)
        R_arm = np.array([[c, -s, 0.0],
                          [s,  c, 0.0],
                          [0.0, 0.0, 1.0]])
        
        ee_world = arm_base_world + (R_arm @ true_ee_local)
        return ee_world
    
    def fk(self, h1, h2, a1, theta, phi=0.0, d2=0.1, l3_max=0.7, wrist_length=0.25, eps=1e-12, intermediate=False):
        p1 = np.array([0.0, 0.0, h1])
        p2 = np.array([-d2 * np.cos(theta), -d2 * np.sin(theta), h2])
        v = p1 - p2
        dist = np.linalg.norm(v)
        if dist < eps:
            dist = eps
        u = v / dist
        wb = p1 + a1 * u
        if intermediate:
            p_inner = p1 - (l3_max - a1) * u
            return p_inner
        z = u 
        if np.abs(z[2]) < 0.99:
            ref = np.array([0.0, 0.0, 1.0])
        else:
            ref = np.array([1.0, 0.0, 0.0])
            
        proj = np.dot(ref, z)
        xl = ref - proj * z
        n = np.linalg.norm(xl)
        if n < eps:
            ref = np.array([1.0, 0.0, 0.0])
            proj = np.dot(ref, z)
            xl = ref - proj * z
            n = np.linalg.norm(xl)
            if n < eps:
                n = 1.0
        xl /= n
        yl = np.cross(z, xl)
        sb = -np.sin(phi)
        cb = np.cos(phi)
        wz = xl * sb + z * cb
        ee = wb + wrist_length * wz
        return ee
    
    def ik(self, target_world, arm="left", d2=0.1, l3_max=0.7, alpha_min_deg=10.0,
           bounds_h=(0.0, 1.5), bounds_a=(0.0, 0.7), cache_threshold=0.001):

        target_world = np.array(target_world, dtype=np.float64)
        if target_world.shape != (3,):
            raise ValueError("target_world must be (3,)")
            
        arm_offset = self.ik_arm_offset_left if arm == "left" else self.ik_arm_offset_right
        if not hasattr(self, '_ik_cache'):
            self._ik_cache = {
                'left': {'target': None, 'result': None},
                'right': {'target': None, 'result': None}
            }

        cache = self._ik_cache[arm]
        if cache['target'] is not None:
            dist = np.linalg.norm(target_world - cache['target'])
            if dist <= cache_threshold:
                return cache['result'].copy()
                
        base_pose = self.localization()
        x_b, y_b, theta_b = base_pose
        c, s = np.cos(theta_b), np.sin(theta_b)
        R = np.array([[c, -s], 
                      [s,  c]])
        
        arm_base_world_xy = R @ arm_offset[:2]
        arm_base_z = arm_offset[2]
        alpha_min_rad = np.deg2rad(alpha_min_deg)

        def get_ee_world(vars):
            h1, h2, a1, theta = vars
            ee_local = self.fk(h1, h2, a1, theta, d2=d2, l3_max=l3_max)
            if isinstance(ee_local, tuple) and ee_local[0] is None:
                return None
            true_ee_local = np.array([ee_local[0], ee_local[1], ee_local[2]])
            ee_world_xy = arm_base_world_xy + R @ true_ee_local[:2]
            ee_world_z = arm_base_z + true_ee_local[2]
            return np.array([ee_world_xy[0], ee_world_xy[1], ee_world_z])

        def cost(vars, w_a1=1e-2):
            ee_world = get_ee_world(vars)
            if ee_world is None:
                return 1e3 + 1e2 * np.linalg.norm(np.array(vars[:3]) - 0.5)
            dist_err = np.sum((ee_world - target_world)**2)
            return float(dist_err + w_a1 * vars[2])
        
        def min_angle_con(vars):
            h1, h2, a1, theta = vars
            alpha = np.arctan2(np.abs(h2 - h1), d2)
            return alpha - alpha_min_rad
        
        cons = ({'type': 'ineq', 'fun': min_angle_con},)
        b = [(bounds_h[0], bounds_h[1]),
             (bounds_h[0], bounds_h[1]),
             (bounds_a[0], bounds_a[1]),
             (-np.pi, np.pi)]
        
        if arm == "left":
            x0, _ = self.get_encoder()
        elif arm == "right":
            _, x0 = self.get_encoder()
        else:
            x0 = np.array([0.0, 0.0, 0.0, 0.0])

        res = minimize(cost, x0, method='SLSQP', bounds=b, constraints=cons,
                       options={'ftol': 1e-9, 'maxiter': 50, 'disp': False})
        
        if not res.success:
            print(f"[IK] Failed for {arm} arm! Using fallback.")
            if cache['result'] is not None:
                return cache['result'].copy()
            else:
                return x0
            
        result = np.array([float(res.x[0]), float(res.x[1]), float(res.x[2]), float(res.x[3])])
        cache['target'] = target_world.copy()
        cache['result'] = result.copy()
        return result
            
    def pid_base_joints(self, target_angle_1, target_angle_2, kp=20, ki=0.1, kd=7):
        dt = self.model.opt.timestep
        jnt1_id = self.model.joint("BaseJoint_1").id
        qpos1 = self.data.qpos[self.model.jnt_qposadr[jnt1_id]]
        error1 = (target_angle_1 - qpos1 + np.pi) % (2 * np.pi) - np.pi
        self.base_integral_1 += error1 * dt
        derivative1 = (error1 - self.base_prev_error_1) / dt 
        self.base_prev_error_1 = error1
        torque1 = kp * error1 + ki * self.base_integral_1 + kd * derivative1
        jnt2_id = self.model.joint("BaseJoint_2").id
        qpos2 = self.data.qpos[self.model.jnt_qposadr[jnt2_id]]
        error2 = (target_angle_2 - qpos2 + np.pi) % (2 * np.pi) - np.pi
        self.base_integral_2 += error2 * dt
        derivative2 = (error2 - self.base_prev_error_2) / dt
        self.base_prev_error_2 = error2
        torque2 = kp * error2 + ki * self.base_integral_2 + kd * derivative2
        return torque1, torque2
    
    def control_grippers(self, left_is_grab, right_is_grab):
        open_pos = [
            0.0, 0.0,                   # palm_finger_[c, b]_joint
            0.0610865, 0.0, -0.0872665, # finger_c_[1, 2, 3]
            0.0610865, 0.0, -0.0872665, # finger_b_[1, 2, 3]
            0.0610865, 0.0, -0.0872665, # finger_a_[1, 2, 3]
        ]
        close_pos = [
            0.0, 0.0,                       # palm_finger_[c, b]_joint
            0.8066, 0.174533, -0.610865,    # finger_c_[1, 2, 3]
            0.8066, 0.174533, -0.610865,    # finger_b_[1, 2, 3]
            0.8066, 0.174533, -0.610865,    # finger_a_[1, 2, 3]
        ]
        def apply_gripper_cmd(gripper_ids, is_grabbing):
            cmd = self.data.ctrl[gripper_ids].copy()
            target_11 = close_pos if is_grabbing else open_pos
            # Indices 9, 10: Palm joints
            cmd[9] = target_11[0]
            cmd[10] = target_11[1]
            # Indices 0, 1, 2: Finger C joints
            cmd[0] = target_11[2]
            cmd[1] = target_11[3]
            cmd[2] = target_11[4]
            # Indices 3, 4, 5: Finger B joints
            cmd[3] = target_11[5]
            cmd[4] = target_11[6]
            cmd[5] = target_11[7]
            # Indices 6, 7, 8: Finger A joints
            cmd[6] = target_11[8]
            cmd[7] = target_11[9]
            cmd[8] = target_11[10]
            self.data.ctrl[gripper_ids] = cmd
        apply_gripper_cmd(self.gripper_ids_left, left_is_grab)
        apply_gripper_cmd(self.gripper_ids_right, right_is_grab)
                
    def control_base(self, target, alpha=0):
        k_p = 5.0; k_i = 0.1; k_d = 0.8
        k_p_theta = 5.0; k_i_theta = 0.1; k_d_theta = 0.8
        self.mobile_dot[0] = self.data.qvel[19]
        self.mobile_dot[1] = self.data.qvel[6] 
        self.mobile_dot[2] = self.data.qvel[45]
        self.mobile_dot[3] = self.data.qvel[32]
        target_x, target_y, target_yaw = target 
        current_x, current_y = self.data.xpos[self.base_id, 0], self.data.xpos[self.base_id, 1]
        w, x, y, z = self.data.xquat[self.base_id]
        current_yaw = np.arctan2(2*(w*z + x*y), 1 - 2*(y**2 + z**2))
        delta_x = target_x - current_x
        delta_y = target_y - current_y
        delta_yaw = np.arctan2(np.sin(target_yaw - current_yaw), np.cos(target_yaw - current_yaw))
        delta_x_local = np.cos(current_yaw) * delta_x + np.sin(current_yaw) * delta_y
        delta_y_local = -np.sin(current_yaw) * delta_x + np.cos(current_yaw) * delta_y
        self.integral_x += delta_x_local * self.model.opt.timestep
        self.integral_y += delta_y_local * self.model.opt.timestep
        self.integral_yaw += delta_yaw * self.model.opt.timestep
        deriv_x_local = (delta_x_local - self.prev_delta_x) / self.model.opt.timestep
        deriv_y_local = (delta_y_local - self.prev_delta_y) / self.model.opt.timestep
        deriv_yaw = (delta_yaw - self.prev_delta_yaw) / self.model.opt.timestep
        self.deriv_x = alpha * getattr(self, "deriv_x", 0.0) + (1 - alpha) * deriv_x_local
        self.deriv_y = alpha * getattr(self, "deriv_y", 0.0) + (1 - alpha) * deriv_y_local
        self.deriv_yaw = alpha * getattr(self, "deriv_yaw", 0.0) + (1 - alpha) * deriv_yaw
        self.prev_delta_x = delta_x_local
        self.prev_delta_y = delta_y_local
        self.prev_delta_yaw = delta_yaw
        v_x_local = k_p * delta_x_local + k_i * self.integral_x + k_d * self.deriv_x
        v_y_local = k_p * delta_y_local + k_i * self.integral_y + k_d * self.deriv_y
        omega = k_p_theta * delta_yaw + k_i_theta * self.integral_yaw + k_d_theta * self.deriv_yaw
        self.target_vel[0] = (v_x_local - v_y_local - omega * self.D) / self.r
        self.target_vel[1] = (v_x_local + v_y_local + omega * self.D) / self.r
        self.target_vel[2] = (v_x_local + v_y_local - omega * self.D) / self.r
        self.target_vel[3] = (v_x_local - v_y_local + omega * self.D) / self.r
        self.command = self.target_vel - self.mobile_dot
        eps = 0.05
        self.command[np.abs(self.command) < eps] = 0.0
        self.data.ctrl[0] = self.command[1]
        self.data.ctrl[1] = self.command[0]
        self.data.ctrl[2] = self.command[3]
        self.data.ctrl[3] = self.command[2]
        
    def control_arms(self):
        q_left, q_right = self.get_encoder()        
        current_u_left = self.data.ctrl[self.actuator_ids[0:3]]
        current_u_right = self.data.ctrl[self.actuator_ids[4:7]]
        
        left_target_global, right_target_global, left_present, right_present, left_is_grab, right_is_grab = self.landmark_viz.get_current_targets()
        
        raw_cmd_L = current_u_left
        raw_cmd_R = current_u_right
        target_theta_left = q_left[3]
        target_theta_right = q_right[3]
        
        ee1_pos = self.data.site_xpos[self.ee_site_1_id] if self.ee_site_1_id != -1 else None
        ee2_pos = self.data.site_xpos[self.ee_site_2_id] if self.ee_site_2_id != -1 else None
        
        self.use_ik = False
    
        if left_present and left_target_global is not None:
            new_target_L = np.array(left_target_global)
            
            if left_present:
                self.target_left_global = new_target_L
                ik_sol_left = self.ik(target_world=self.target_left_global, arm="left")
                if ik_sol_left is not None:
                    raw_cmd_L = (ik_sol_left[:3]) * 100
                    target_theta_left = ik_sol_left[3]
                    self.use_ik = True
                    print(f"[TARGET L] : {self.target_left_global.round(3)} || [CURRENT L] : {ee1_pos.round(3) if ee1_pos is not None else 'N/A'}")
                else:
                    raw_cmd_L = current_u_left
                    target_theta_left = q_left[3]
                self.prev_target_left = self.target_left_global.copy()
            else:
                print("NO LEFT HAND (or Glitch Rejected)")
                if self.prev_target_left is not None:
                    self.target_left_global = self.prev_target_left
                    ik_sol_left = self.ik(target_world=self.target_left_global, arm="left")
                    if ik_sol_left is not None:
                        raw_cmd_L = (ik_sol_left[:3]) * 100
                        target_theta_left = ik_sol_left[3]
                        self.use_ik = True

        if right_present and right_target_global is not None:
            new_target_R = np.array(right_target_global)
            if right_present:
                self.target_right_global = new_target_R
                ik_sol_right = self.ik(target_world=self.target_right_global, arm="right")
                if ik_sol_right is not None:
                    raw_cmd_R = (ik_sol_right[:3]) * 100
                    target_theta_right = ik_sol_right[3]
                    self.use_ik = True
                    print(f"[TARGET R] : {self.target_right_global.round(3)} || [CURRENT R] : {ee2_pos.round(3) if ee2_pos is not None else 'N/A'}")
                else:
                    raw_cmd_R = current_u_right
                    target_theta_right = q_right[3]
                self.prev_target_right = self.target_right_global.copy()
            else:
                print("NO RIGHT HAND (or Glitch Rejected)")
                if self.prev_target_right is not None:
                    self.target_right_global = self.prev_target_right 
                    ik_sol_right = self.ik(target_world=self.target_right_global, arm="right")
                    if ik_sol_right is not None:
                        raw_cmd_R = (ik_sol_right[:3]) * 100
                        target_theta_right = ik_sol_right[3]
                        self.use_ik = True

        u_base_left, u_base_right = self.pid_base_joints(target_theta_left, target_theta_right)
        alpha = 0.1
        self._smooth_cmd_L = (1 - alpha) * current_u_left + alpha * raw_cmd_L
        self._smooth_cmd_R = (1 - alpha) * current_u_right + alpha * raw_cmd_R
        
        u_cmd = np.concatenate([
            self._smooth_cmd_L,  
            [u_base_left],      
            self._smooth_cmd_R,  
            [u_base_right]      
        ])
        
        self.send_command_arm(u_cmd)
        self.control_grippers(left_is_grab, right_is_grab)
        
    def step_simulation(self, render=True):
        self.control_base(target=self.target_base, alpha=0.1)
        self.control_arms()
        mujoco.mj_step(self.model, self.data, nstep=5)

    def camera_display(self):
        self.frame_count = getattr(self, 'frame_count', 0)
        self.renderer_top.update_scene(self.data, self.camera)
        rgb_top = self.renderer_top.render()
        if rgb_top is None or rgb_top.size == 0:
            print("Error: Top view rendering failed")
            return
        bgr_top = cv2.cvtColor(rgb_top, cv2.COLOR_RGB2BGR)
        if self.top_video_writer is not None and self.top_video_writer.isOpened():
            self.top_video_writer.write(bgr_top)
        self.frame_count += 1
        cv2.imshow("MuJoCo Top View", bgr_top)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self._terminate = True

    def run_cv(self):
        if self.record:
            output_dir = "output_videos"
            os.makedirs(output_dir, exist_ok=True)
            frame_width, frame_height = self.model.vis.global_.offwidth, self.model.vis.global_.offheight 
            fps = 30
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            top_video_file = os.path.join(output_dir, f"top_view_{timestamp}.mp4")
            pov_video_file = os.path.join(output_dir, f"pov_view_{timestamp}.mp4")
            fourcc = cv2.VideoWriter_fourcc(*'MJPG') 
            self.top_video_writer = cv2.VideoWriter(top_video_file, fourcc, fps, (frame_width, frame_height))
            self.pov_video_writer = cv2.VideoWriter(pov_video_file, fourcc, fps, (frame_width, frame_height))

        try:
            mujoco.mj_step(self.model, self.data, nstep=1)
            self._terminate = False
            while not self._terminate:
                self.step_simulation(render=False)
                self.landmark_viz.step()
                self.renderer_top.update_scene(self.data, self.camera)
                self.landmark_viz.render(self.renderer_top.scene) 
                self.camera_display()
        except Exception as e:
            print(f"Simulation error: {e}")
        finally:
            if self.record:
                if self.top_video_writer is not None:
                    self.top_video_writer.release()
                if self.pov_video_writer is not None:
                    self.pov_video_writer.release()
            cv2.destroyAllWindows()
            self.renderer_top.close()

    def run_glfw(self):
        mujoco.mj_step(self.model, self.data, nstep=1)
        while not glfw.window_should_close(self.window) and not self._terminate:
            self.step_simulation(render=False)
            self.landmark_viz.step()
            mujoco.mjv_updateScene(self.model, self.data, self.opt, None, self.camera, mujoco.mjtCatBit.mjCAT_ALL, self.scene)
            self.landmark_viz.render(self.scene)            
            self.render_target_spheres(self.scene)
            mujoco.mjr_render(self.viewport, self.scene, self.ctx) 
            glfw.swap_buffers(self.window)
            glfw.poll_events()
        glfw.terminate()
           
    def on_key(self, window, key, scancode, action, mods):
        if action not in (glfw.PRESS, glfw.REPEAT):
            return
        if key == glfw.KEY_ESCAPE:
            glfw.set_window_should_close(self.window, True)
            return
        if key == glfw.KEY_ENTER:
            self.reset("home")
            self.gripper_ctrl = 0.0
            return

    def _scroll_callback(self, window, xoffset, yoffset):
        if self.camera.type != mujoco.mjtCamera.mjCAMERA_FREE:
            return
        factor = 0.05
        mujoco.mjv_moveCamera(self.model, mujoco.mjtMouse.mjMOUSE_ZOOM, 0, yoffset*factor, self.scene, self.camera)

    def _mouse_button_callback(self, window, button, action, mods):
        if self.camera.type != mujoco.mjtCamera.mjCAMERA_FREE:
            return
            
        pressed = (action == glfw.PRESS)
        if button == glfw.MOUSE_BUTTON_LEFT:
            self._mouse_left_pressed = pressed
        elif button == glfw.MOUSE_BUTTON_RIGHT:
            self._mouse_right_pressed = pressed
        elif button == glfw.MOUSE_BUTTON_MIDDLE:
            self._mouse_middle_pressed = pressed

    def _cursor_pos_callback(self, window, xpos, ypos):
        if self.camera.type != mujoco.mjtCamera.mjCAMERA_FREE:
            self._last_mouse_x, self._last_mouse_y = xpos, ypos
            return
        dx = xpos - self._last_mouse_x
        dy = ypos - self._last_mouse_y
        self._last_mouse_x, self._last_mouse_y = xpos, ypos
        factor = 0.005 
        if self._mouse_left_pressed:
            mujoco.mjv_moveCamera(
                self.model, mujoco.mjtMouse.mjMOUSE_ROTATE_H,
                dx * factor, dy * factor, self.scene, self.camera
            )
        elif self._mouse_right_pressed:
            mujoco.mjv_moveCamera(
                self.model, mujoco.mjtMouse.mjMOUSE_MOVE_H,
                dx * factor, dy * factor, self.scene, self.camera
            )
        elif self._mouse_middle_pressed:
            mujoco.mjv_moveCamera(
                self.model, mujoco.mjtMouse.mjMOUSE_ZOOM,
                0.0, dy * factor * 10.0, self.scene, self.camera
            )

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run MuJoCo Parallel Robot Simulation")
    parser.add_argument("--run", choices=["glfw", "cv"], default="glfw", help="Run mode: 'glfw' or 'cv'")
    parser.add_argument("--record", action="store_true", help="Record video output to MP4 (only applicable with --run cv)")
    parser.add_argument("--landmarks", type=str, default="processed_landmarks.csv", help="Path to processed landmarks CSV for visual-only overlay")
    parser.add_argument("--playback-rate", type=float, default=1.0, help="Playback speed multiplier (e.g., 0.3 for 30% speed, 2.0 for 2x)")
    parser.add_argument("--data-mode", choices=["csv", "ws"], default="csv", help="Data source: 'csv' for file playback, 'ws' for real-time WebSocket")
    parser.add_argument("--ws-url", type=str, default="ws://localhost:9090", help="WebSocket URL to connect to (for ws mode)")

    args = parser.parse_args()

    if args.record and args.run != "cv":
        print("Warning: --record is only applicable with --run cv. Ignoring --record.")
        args.record = False
        
    xml_path = os.path.join(os.path.dirname(__file__), '..', 'env', 'market_world_plain.xml')
    xml_path = os.path.abspath(xml_path)
    
    sim = ParallelRobot(xml_path, args.run, args.record, landmark_csv=args.landmarks, 
                        playback_rate=args.playback_rate, data_mode=args.data_mode, ws_url=args.ws_url)
        
    if args.run == "glfw":
        sim.run_glfw()
    else:
        sim.run_cv()