from pathlib import Path
import threading
from typing import Optional

import mujoco
import numpy as np
from loop_rate_limiters import RateLimiter
import babyros
from geometry.transform import StaticTransform, HipAnchoredTransform
import mink
import glfw
from scipy.spatial.transform import Rotation
from rendering.glfw_viewer import GlfwViewer

_HERE = Path(__file__).parent
_XML = _HERE / "xmls" / "combine" / "floor_morph_i.xml"

ROBOT_PREFIXES = "robot"
ARM_PREFIXES = ["left", "right"]
ARM_MOTORS = [
    "base_rotation", "primary_column_lift", "secondary_column_lift",
    "telescopic_extend", "wrist_pitch", "gripper_roll_joint",
    "gripper_pitch_joint", "gripper_yaw_joint"
]

ARM_MOTORS_MAX_VEL = {
    "base_rotation":          np.pi * 4,
    "primary_column_lift":    10.5,
    "secondary_column_lift":  10.5,
    "telescopic_extend":      10.5,
    "wrist_pitch":            np.pi,
    "gripper_roll_joint":     np.pi,
    "gripper_pitch_joint":    np.pi,
    "gripper_yaw_joint":      np.pi,
}

FORCE_THRESHOLD_LEFT = 15.0
FORCE_THRESHOLD_RIGHT = 15.0

TARGET_MOVE_STEP = 0.005  
TRACKING_MODE = "upper-only"  # default mode
MAX_ITER = 20

HUMAN_SEATED_Z_MIN = 0.70
HUMAN_SEATED_Z_MAX = 1.25

ROBOT_Z_MIN = 0.00
ROBOT_Z_MAX = 1.30

Z_SCALE_FACTOR = (ROBOT_Z_MAX - ROBOT_Z_MIN) / (HUMAN_SEATED_Z_MAX - HUMAN_SEATED_Z_MIN)
Z_OFFSET = ROBOT_Z_MIN - (HUMAN_SEATED_Z_MIN * Z_SCALE_FACTOR)

# Z_SCALE_FACTOR = 1.0
# Z_OFFSET = 0.0

SEATED_HIP_HEIGHT = 0.65

# --- Lateral (X/Y) amplification ---
X_SCALE_FACTOR = 1.8
Y_SCALE_FACTOR = 1.8
xy_center_initialized = False
XY_CENTER_X = 0.0             
XY_CENTER_Y = 0.0

# --- Robot base XY position (for gripper yaw calculation) ---
ROBOT_BASE_XY = np.array([0.0, 0.0])

# --- Face target toggle ---
FACE_TARGET_ENABLED = True
YAW_OFFSET = 0.0

# --- Landmark smoothing (EMA low-pass filter) ---
# 0.0 = no smoothing, 1.0 = fully frozen
SMOOTHING_FACTOR = 0.8
SMOOTHING_STEP = 0.05
smoothed_pts = None 

EYE_MAT_FLAT         = np.eye(3).flatten()
POINT_SIZE_GEOM      = (0.01, 0.0, 0.0)
SKELETON_RGBA_GEOM   = (0.5, 0.5, 0.5, 1.0)
COLORS_GEOM          = [(0,1,1,1)]*33 + [(1,0,1,1)]*21 + [(1,0.5,0,1)]*21
 
FINGER_BODY_IDS = {
    "left": {"a": [], "b": [], "c": []},
    "right": {"a": [], "b": [], "c": []}
}

PARENT_BODY_NAME = f"{ROBOT_PREFIXES}_mobile_base"
BASE_TRANSFORM = StaticTransform(
    x=0.3, y=0.0, z=SEATED_HIP_HEIGHT,
    roll=90.0, pitch=0.0, yaw=90.0,
    use_degrees=True,
)

FULL_BODY_TRANSFORM = HipAnchoredTransform(
    static_tf=BASE_TRANSFORM, anchor_init=True,
    foot_on_ground=True, ground_level=0.0, foot_offset=0.0,
)

UPPER_ONLY_TRANSFORM = HipAnchoredTransform(
    static_tf=BASE_TRANSFORM, anchor_init=False,
    foot_on_ground=False, ground_level=0.0, foot_offset=0.0,
    fixed_hip=True,
)

def full_reset():
    global xy_center_initialized, smoothed_pts
    if model.nkey > 0 and "home" in [model.key(i).name for i in range(model.nkey)]:
        mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    else:
        mujoco.mj_resetData(model, data)
    
    configuration.update(data.qpos)
    posture_task.set_target_from_configuration(configuration)
    
    mujoco.mj_forward(model, data)
    mink.move_mocap_to_frame(model, data, "left_ik_target", f"{ROBOT_PREFIXES}_left_ee_site", "site")
    mink.move_mocap_to_frame(model, data, "right_ik_target", f"{ROBOT_PREFIXES}_right_ee_site", "site")
    
    xy_center_initialized = False
    smoothed_pts = None
    print("[FULL RESET] Robot reset to home keyframe, IK targets synced to end-effectors")
    print("[CENTER] XY neutral will be re-captured on next wrist detection")
    print(f"[SMOOTH] Landmark smoothing reset (factor={SMOOTHING_FACTOR:.2f})")

def get_gripper_roll_xy(data: mujoco.MjData, arm_idx: int) -> np.ndarray:
    body_name = f"{ROBOT_PREFIXES}_{ARM_PREFIXES[arm_idx]}_gripper_roll"
    try:
        body_id = data.model.body(body_name).id
        return data.xpos[body_id, :2].copy()
    except KeyError:
        return ROBOT_BASE_XY.copy()
    
def get_gripper_contact_forces(data: mujoco.MjData) -> dict:
    forces = {'left': {'a': 0.0, 'b': 0.0, 'c': 0.0}, 
              'right': {'a': 0.0, 'b': 0.0, 'c': 0.0}}
    cfrc = data.cfrc_ext
    for side in ("left", "right"):
        for finger, ids in FINGER_BODY_IDS[side].items():
            if len(ids) > 0:
                forces[side][finger] = np.sum(np.linalg.norm(cfrc[ids, 3:6], axis=1))
    return forces

def get_body_world_pose(data: mujoco.MjData, body_name: str):
    body_id = data.model.body(body_name).id
    return data.xpos[body_id].copy(), data.xmat[body_id].reshape(3, 3).copy()

def quat_to_mat(quat):
    w, x, y, z = quat
    return np.array([
        [1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
        [2*x*y + 2*z*w, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
        [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x**2 - 2*y**2]
    ])

def compute_face_target_quat(target_pos, base_xy=ROBOT_BASE_XY, yaw_offset=0.0):
    direction = target_pos[:2] - base_xy
    if np.linalg.norm(direction) < 1e-5:
        return np.array([1.0, 0.0, 0.0, 0.0])
    yaw = np.arctan2(direction[1], direction[0]) + yaw_offset
    rot = Rotation.from_euler('z', yaw)
    quat_xyzw = rot.as_quat()
    return np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]])

if __name__ == "__main__":
    model = mujoco.MjModel.from_xml_path(_XML.as_posix())

    data = mujoco.MjData(model)
    configuration = mink.Configuration(model)

    motor_joint_names = [f"{ROBOT_PREFIXES}_{arm_prefix}_{motor}" for arm_prefix in ARM_PREFIXES for motor in ARM_MOTORS]
    motor_ctrl_ids = []
    motor_qpos_adrs = []
    for name in motor_joint_names:
        try:
            jnt_id = model.joint(name).id
        except KeyError:
            continue
        actuator_indices = np.where(model.actuator_trnid[:, 0] == jnt_id)[0]
        if len(actuator_indices) > 0:
            motor_ctrl_ids.append(actuator_indices[0])
            motor_qpos_adrs.append(model.jnt_qposadr[jnt_id])
    motor_ctrl_ids = np.array(motor_ctrl_ids)
    motor_qpos_adrs = np.array(motor_qpos_adrs)

    finger_ctrl_ids = {
        "left": {"a": {"joint1": [], "joint2": [], "joint3": []},
                 "b": {"joint1": [], "joint2": [], "joint3": []},
                 "c": {"joint1": [], "joint2": [], "joint3": []}},
        "right": {"a": {"joint1": [], "joint2": [], "joint3": []},
                  "b": {"joint1": [], "joint2": [], "joint3": []},
                  "c": {"joint1": [], "joint2": [], "joint3": []}}
    }
    finger_qpos_adrs = {
        "left": {"a": {"joint1": [], "joint2": [], "joint3": []},
                 "b": {"joint1": [], "joint2": [], "joint3": []},
                 "c": {"joint1": [], "joint2": [], "joint3": []}},
        "right": {"a": {"joint1": [], "joint2": [], "joint3": []},
                  "b": {"joint1": [], "joint2": [], "joint3": []},
                  "c": {"joint1": [], "joint2": [], "joint3": []}}
    }
    
    for i in range(model.nu):
        jnt_id = model.actuator_trnid[i, 0]
        jnt_name = model.joint(jnt_id).name
        if "finger" in jnt_name:
            arm_side = "left" if "left" in jnt_name else "right"
            qpos_adr = model.jnt_qposadr[jnt_id]
            
            if "finger_a" in jnt_name:
                finger_letter = "a"
            elif "finger_b" in jnt_name:
                finger_letter = "b"
            elif "finger_c" in jnt_name:
                finger_letter = "c"
            else:
                continue
                
            if jnt_name.endswith("joint1"):
                finger_ctrl_ids[arm_side][finger_letter]["joint1"].append(i)
                finger_qpos_adrs[arm_side][finger_letter]["joint1"].append(qpos_adr)
            elif jnt_name.endswith("joint2"):
                finger_ctrl_ids[arm_side][finger_letter]["joint2"].append(i)
                finger_qpos_adrs[arm_side][finger_letter]["joint2"].append(qpos_adr)
            elif jnt_name.endswith("joint3"):
                finger_ctrl_ids[arm_side][finger_letter]["joint3"].append(i)
                finger_qpos_adrs[arm_side][finger_letter]["joint3"].append(qpos_adr)

    for side in finger_ctrl_ids:
        for finger in finger_ctrl_ids[side]:
            for j in finger_ctrl_ids[side][finger]:
                finger_ctrl_ids[side][finger][j] = np.array(finger_ctrl_ids[side][finger][j])
                finger_qpos_adrs[side][finger][j] = np.array(finger_qpos_adrs[side][finger][j])

    for i in range(1, model.nbody):
        name = model.body(i).name
        if not name: 
            continue
        
        side = "left" if "left" in name else "right" if "right" in name else None
        if not side: 
            continue
        
        if "finger_a" in name and ("proximal" in name or "middle" in name or "distal" in name):
            FINGER_BODY_IDS[side]["a"].append(i)
        elif "finger_b" in name and ("proximal" in name or "middle" in name or "distal" in name):
            FINGER_BODY_IDS[side]["b"].append(i)
        elif "finger_c" in name and ("proximal" in name or "middle" in name or "distal" in name):
            FINGER_BODY_IDS[side]["c"].append(i)

    for side in FINGER_BODY_IDS:
        for finger in FINGER_BODY_IDS[side]:
            FINGER_BODY_IDS[side][finger] = np.array(FINGER_BODY_IDS[side][finger], dtype=int)

    finger_close_targets = {"joint1": 0.33, "joint2": 0.0, "joint3": 0.0}
    finger_open_targets  = {"joint1": 0.0,  "joint2": 0.0, "joint3": 0.0}

    tasks = [
        left_end_effector_task := mink.FrameTask(
            frame_name=f"{ROBOT_PREFIXES}_{ARM_PREFIXES[0]}_ee_site", frame_type="site",
            position_cost=5.0, orientation_cost=1.0, lm_damping=1e-2,
        ),
        right_end_effector_task := mink.FrameTask(
            frame_name=f"{ROBOT_PREFIXES}_{ARM_PREFIXES[1]}_ee_site", frame_type="site",
            position_cost=5.0, orientation_cost=1.0, lm_damping=1e-2,
        ),
        posture_task := mink.PostureTask(model=model, cost=1e-3, lm_damping=1e-2),
    ]
    equality_task = mink.EqualityConstraintTask(model=model, cost=1000.0, gain=1.0, lm_damping=1e-3)
    tasks.append(equality_task)

    left_arm_geoms = mink.get_subtree_geom_ids(model, model.body(f"{ROBOT_PREFIXES}_{ARM_PREFIXES[0]}_arm_base").id)
    right_arm_geoms = mink.get_subtree_geom_ids(model, model.body(f"{ROBOT_PREFIXES}_{ARM_PREFIXES[1]}_arm_base").id)
    base_geoms = mink.get_body_geom_ids(model, model.body(f"{ROBOT_PREFIXES}_base").id)
    floor_geom = mink.get_body_geom_ids(model, model.body("world_floor").id)

    collision_avoidance_limit = mink.CollisionAvoidanceLimit(
        model=model,
        geom_pairs=[
            (floor_geom, left_arm_geoms),
            (floor_geom, right_arm_geoms),
            (left_arm_geoms, right_arm_geoms),
            (left_arm_geoms, base_geoms),
            (right_arm_geoms, base_geoms),
        ],
        minimum_distance_from_collisions=0.012,
        collision_detection_distance=0.012,
    )

    max_velocities = {
        f"{ROBOT_PREFIXES}_{arm}_{motor}": vel 
        for arm in ARM_PREFIXES 
        for motor, vel in ARM_MOTORS_MAX_VEL.items()
    }
    velocity_limit = mink.VelocityLimit(model, max_velocities)

    limits = [
        mink.ConfigurationLimit(model=configuration.model),
        collision_avoidance_limit,
        velocity_limit,
    ]

    solver = "daqp"
    tracking_data = {'poses': []}
    tracking_lock = threading.Lock()

    def tracking_callback(msg: dict):
        with tracking_lock:
            tracking_data['poses'] = msg.get('poses', [])

    tracking_sub = babyros.node.Subscriber(topic="landmarks", callback=tracking_callback)

    gesture_data = {'left': 'unknown', 'right': 'unknown'}
    gesture_lock = threading.Lock()

    def gesture_callback(msg: dict):
        with gesture_lock:
            gesture_data['left'] = msg.get('left_hand', 'unknown')
            gesture_data['right'] = msg.get('right_hand', 'unknown')

    gesture_sub = babyros.node.Subscriber(topic="hand_gestures", callback=gesture_callback)

    left_mocap_id = model.body("left_ik_target").mocapid[0]
    right_mocap_id = model.body("right_ik_target").mocapid[0]

    MAX_POINTS = 75
    POSE_CONNECTIONS = [
        (0,1),(1,2),(2,3),(3,7),(0,4),(4,5),(5,6),(6,8),(9,10),
        (11,12),(11,13),(13,15),(12,14),(14,16),
        (15,17),(15,19),(15,21),(16,18),(16,20),(16,22),
        (11,23),(12,24),(23,24),(23,25),(25,27),(24,26),(26,28),
        (27,29),(27,31),(28,30),(28,32),
    ]
    HAND_CONNECTIONS = [
        (0,1),(1,2),(2,3),(3,4),(0,5),(5,6),(6,7),(7,8),
        (5,9),(9,10),(10,11),(11,12),(9,13),(13,14),(14,15),(15,16),
        (13,17),(17,18),(18,19),(19,20),(0,17),
    ]
    SKELETON_CONNECTIONS = list(POSE_CONNECTIONS)
    for c in HAND_CONNECTIONS:
        SKELETON_CONNECTIONS.append((c[0]+33, c[1]+33))
    for c in HAND_CONNECTIONS:
        SKELETON_CONNECTIONS.append((c[0]+54, c[1]+54))

    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.distance = 3.0
    camera.azimuth = 135.0
    camera.elevation = -20.0
    camera.lookat[:] = [0.0, 0.0, 0.8]

    scene = mujoco.MjvScene(model, maxgeom=2000)
    scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW.value] = 0
    scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION.value] = 0
    scene.flags[mujoco.mjtRndFlag.mjRND_FOG.value] = 1

    opt = mujoco.MjvOption()
    opt.frame = mujoco.mjtFrame.mjFRAME_NONE

    viewer = GlfwViewer(width=960, height=1000, title="MORPH I <-> FLOOR")
    viewer.setup(model, camera, scene, opt)

    # Print all keyboard callbacks
    print("\n" + "=" * 60)
    print("KEYBOARD CONTROLS")
    print("=" * 60)
    print("[GLOBAL]")
    print("  BACKSPACE        -> Full reset (home pose + re-capture XY center)")
    print("  M                -> Toggle tracking mode (upper-only / full-body)")
    print("  F                -> Toggle gripper yaw face-target (ON/OFF)")
    print("\n[SMOOTHING]")
    print("  [ / ]            -> Decrease / increase landmark smoothing")
    print(f"                     (current: {SMOOTHING_FACTOR:.2f})")
    print("\n[LEFT ARM]  (hold Left Shift + key)")
    print("  W / S            -> Move left target  forward / backward  (X)")
    print("  <- / ->          -> Move left target  right / left        (Y)")
    print("  ↑ / ↓            -> Move left target  up / down           (Z)")
    print("  C                -> Close left gripper")
    print("  O                -> Open left gripper")
    print("\n[RIGHT ARM] (hold Right Shift + key)")
    print("  W / S            -> Move right target forward / backward  (X)")
    print("  <- / ->          -> Move right target right / left        (Y)")
    print("  ↑ / ↓            -> Move right target up / down           (Z)")
    print("  C                -> Close right gripper")
    print("  O                -> Open right gripper")
    print("=" * 60 + "\n")

    full_reset()
    print(f"[MODE] Tracking mode: {TRACKING_MODE}")
    print(f"[FACE] Gripper face-target: {'ON' if FACE_TARGET_ENABLED else 'OFF'}")

    rte = RateLimiter(frequency=200.0, warn=False)
    poses = {'left': None, 'right': None}
    pose_lock = threading.Lock()
    force_log_counter = 0

    try:
        while not viewer.should_close():
            parent_pos, parent_mat = get_body_world_pose(data, PARENT_BODY_NAME)

            left_mid_palm = None
            right_mid_palm = None
            left_quat = None
            right_quat = None
            world_pts = None
            n_pts = 0

            with tracking_lock:
                poses_list = tracking_data['poses']

            if len(poses_list) > 0:
                n_pts = min(len(poses_list), MAX_POINTS)
                
                raw_pts = np.array(
                    [[p['position']['x'], p['position']['y'], p['position']['z']] for p in poses_list[:n_pts]], 
                    dtype=np.float64
                )
                
                if TRACKING_MODE == "upper-only":
                    world_pts = UPPER_ONLY_TRANSFORM.transform(raw_pts, parent_pos, parent_mat)
                else:
                    world_pts = FULL_BODY_TRANSFORM.transform(raw_pts, parent_pos, parent_mat)

                # --- LANDMARK SMOOTHING (EMA) ---
                if world_pts is not None:
                    if smoothed_pts is None or smoothed_pts.shape != world_pts.shape:
                        smoothed_pts = world_pts.copy()
                    else:
                        smoothed_pts = (1.0 - SMOOTHING_FACTOR) * smoothed_pts + SMOOTHING_FACTOR * world_pts
                    world_pts = smoothed_pts

                if n_pts > 18: 
                    scaled_pts = world_pts.copy()

                    if not xy_center_initialized:
                        wrist_mid = 0.5 * (scaled_pts[15, :2] + scaled_pts[16, :2])
                        XY_CENTER_X = wrist_mid[0]
                        XY_CENTER_Y = wrist_mid[1]
                        xy_center_initialized = True
                        print(f"[CENTER] XY neutral locked at X={XY_CENTER_X:.3f}, Y={XY_CENTER_Y:.3f}")

                    scaled_pts[:, 0] = XY_CENTER_X + (scaled_pts[:, 0] - XY_CENTER_X) * X_SCALE_FACTOR
                    scaled_pts[:, 1] = XY_CENTER_Y + (scaled_pts[:, 1] - XY_CENTER_Y) * Y_SCALE_FACTOR

                    if TRACKING_MODE == "upper-only":
                        scaled_pts[:, 2] = scaled_pts[:, 2] * Z_SCALE_FACTOR + Z_OFFSET

                    LEFT_HAND_OFFSET = 33
                    RIGHT_HAND_OFFSET = 54
                    PALM_LANDMARKS = [0, 5, 9, 13, 17]

                    left_hand_exists = n_pts >= 54
                    right_hand_exists = n_pts >= 75

                    if left_hand_exists:
                        left_palm_idx = [LEFT_HAND_OFFSET + i for i in PALM_LANDMARKS]
                        left_mid_palm = scaled_pts[left_palm_idx].mean(axis=0)
                    else:
                        left_mid_palm = scaled_pts[19].copy()

                    if right_hand_exists:
                        right_palm_idx = [RIGHT_HAND_OFFSET + i for i in PALM_LANDMARKS]
                        right_mid_palm = scaled_pts[right_palm_idx].mean(axis=0)
                    else:
                        right_mid_palm = scaled_pts[20].copy()

            window = glfw.get_current_context()
            left_kb_active = False
            right_kb_active = False

            if window:
                l_shift = glfw.get_key(window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
                r_shift = glfw.get_key(window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS
                
                # --- GLOBAL KEYS ---
                if glfw.get_key(window, glfw.KEY_BACKSPACE) == glfw.PRESS:
                    if not getattr(viewer, '_backspace', False):
                        full_reset()
                        viewer._backspace = True
                else:
                    viewer._backspace = False

                if glfw.get_key(window, glfw.KEY_M) == glfw.PRESS:
                    if not getattr(viewer, '_m', False):
                        TRACKING_MODE = "full-body" if TRACKING_MODE == "upper-only" else "upper-only"
                        print(f"[MODE] Switched to: {TRACKING_MODE}")
                        viewer._m = True
                else:
                    viewer._m = False

                # --- F KEY: Toggle face-target ---
                if glfw.get_key(window, glfw.KEY_F) == glfw.PRESS:
                    if not getattr(viewer, '_f', False):
                        FACE_TARGET_ENABLED = not FACE_TARGET_ENABLED
                        print(f"[FACE] Gripper face-target: {'ON' if FACE_TARGET_ENABLED else 'OFF'}")
                        viewer._f = True
                else:
                    viewer._f = False
                
                # --- SMOOTHING CONTROLS ---
                # Decrease smoothing (more responsive)
                if glfw.get_key(window, glfw.KEY_LEFT_BRACKET) == glfw.PRESS:
                    if not getattr(viewer, '_lb', False):
                        SMOOTHING_FACTOR = max(0.0, SMOOTHING_FACTOR - SMOOTHING_STEP)
                        smoothed_pts = None  # reset EMA to avoid jump
                        print(f"[SMOOTH] factor={SMOOTHING_FACTOR:.2f} (more responsive)")
                        viewer._lb = True
                else:
                    viewer._lb = False

                # Increase smoothing (smoother, more lag)
                if glfw.get_key(window, glfw.KEY_RIGHT_BRACKET) == glfw.PRESS:
                    if not getattr(viewer, '_rb', False):
                        SMOOTHING_FACTOR = min(1.0, SMOOTHING_FACTOR + SMOOTHING_STEP)
                        smoothed_pts = None
                        print(f"[SMOOTH] factor={SMOOTHING_FACTOR:.2f} (smoother)")
                        viewer._rb = True
                else:
                    viewer._rb = False
                
                # --- LEFT ARM KEYBOARD ---
                if l_shift:
                    left_kb_active = True
                    pos = data.mocap_pos[left_mocap_id].copy()
                    if glfw.get_key(window, glfw.KEY_UP) == glfw.PRESS: pos[2] += TARGET_MOVE_STEP
                    if glfw.get_key(window, glfw.KEY_DOWN) == glfw.PRESS: pos[2] -= TARGET_MOVE_STEP
                    if glfw.get_key(window, glfw.KEY_LEFT) == glfw.PRESS: pos[1] += TARGET_MOVE_STEP
                    if glfw.get_key(window, glfw.KEY_RIGHT) == glfw.PRESS: pos[1] -= TARGET_MOVE_STEP
                    if glfw.get_key(window, glfw.KEY_W) == glfw.PRESS: pos[0] += TARGET_MOVE_STEP
                    if glfw.get_key(window, glfw.KEY_S) == glfw.PRESS: pos[0] -= TARGET_MOVE_STEP
                    data.mocap_pos[left_mocap_id] = pos
                    
                # --- RIGHT ARM KEYBOARD ---
                if r_shift:
                    right_kb_active = True
                    pos = data.mocap_pos[right_mocap_id].copy()
                    if glfw.get_key(window, glfw.KEY_UP) == glfw.PRESS: pos[2] += TARGET_MOVE_STEP
                    if glfw.get_key(window, glfw.KEY_DOWN) == glfw.PRESS: pos[2] -= TARGET_MOVE_STEP
                    if glfw.get_key(window, glfw.KEY_LEFT) == glfw.PRESS: pos[1] += TARGET_MOVE_STEP
                    if glfw.get_key(window, glfw.KEY_RIGHT) == glfw.PRESS: pos[1] -= TARGET_MOVE_STEP
                    if glfw.get_key(window, glfw.KEY_W) == glfw.PRESS: pos[0] += TARGET_MOVE_STEP
                    if glfw.get_key(window, glfw.KEY_S) == glfw.PRESS: pos[0] -= TARGET_MOVE_STEP
                    data.mocap_pos[right_mocap_id] = pos

            if not left_kb_active and left_mid_palm is not None:
                data.mocap_pos[left_mocap_id] = left_mid_palm

            if not right_kb_active and right_mid_palm is not None:
                data.mocap_pos[right_mocap_id] = right_mid_palm

            if FACE_TARGET_ENABLED:
                left_pos_for_yaw = data.mocap_pos[left_mocap_id] if left_kb_active else left_mid_palm
                right_pos_for_yaw = data.mocap_pos[right_mocap_id] if right_kb_active else right_mid_palm

                left_gripper_xy  = get_gripper_roll_xy(data, arm_idx=0)
                right_gripper_xy = get_gripper_roll_xy(data, arm_idx=1)

                if left_pos_for_yaw is not None:
                    data.mocap_quat[left_mocap_id] = compute_face_target_quat(
                        left_pos_for_yaw, base_xy=left_gripper_xy, yaw_offset=YAW_OFFSET
                    )
                if right_pos_for_yaw is not None:
                    data.mocap_quat[right_mocap_id] = compute_face_target_quat(
                        right_pos_for_yaw, base_xy=right_gripper_xy, yaw_offset=YAW_OFFSET
                    )

            T_left = mink.SE3.from_mocap_name(model, data, "left_ik_target")
            left_end_effector_task.set_target(T_left)
            T_right = mink.SE3.from_mocap_name(model, data, "right_ik_target")
            right_end_effector_task.set_target(T_right)

            gripper_commands = {}

            if window:
                l_shift = glfw.get_key(window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
                r_shift = glfw.get_key(window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS
                c_key   = glfw.get_key(window, glfw.KEY_C) == glfw.PRESS
                o_key   = glfw.get_key(window, glfw.KEY_O) == glfw.PRESS

                if l_shift and c_key:
                    if not getattr(viewer, '_lc', False):
                        for finger in ("a", "b", "c"):
                            for j in ("joint1","joint2","joint3"):
                                gripper_commands[("left", finger, j)] = finger_close_targets[j]
                        viewer._lc = True
                else:
                    viewer._lc = False

                if l_shift and o_key:
                    if not getattr(viewer, '_lo', False):
                        for finger in ("a", "b", "c"):
                            for j in ("joint1","joint2","joint3"):
                                gripper_commands[("left", finger, j)] = finger_open_targets[j]
                        viewer._lo = True
                else:
                    viewer._lo = False

                if r_shift and c_key:
                    if not getattr(viewer, '_rc', False):
                        for finger in ("a", "b", "c"):
                            for j in ("joint1","joint2","joint3"):
                                gripper_commands[("right", finger, j)] = finger_close_targets[j]
                        viewer._rc = True
                else:
                    viewer._rc = False

                if r_shift and o_key:
                    if not getattr(viewer, '_ro', False):
                        for finger in ("a", "b", "c"):
                            for j in ("joint1","joint2","joint3"):
                                gripper_commands[("right", finger, j)] = finger_open_targets[j]
                        viewer._ro = True
                else:
                    viewer._ro = False

            with gesture_lock:
                left_gesture = gesture_data.get('left', 'unknown')
                right_gesture = gesture_data.get('right', 'unknown')

            if left_gesture == "close":
                for finger in ("a", "b", "c"):
                    for j in ("joint1", "joint2", "joint3"):
                        gripper_commands[("left", finger, j)] = finger_close_targets[j]
            elif left_gesture == "open":
                for finger in ("a", "b", "c"):
                    for j in ("joint1", "joint2", "joint3"):
                        gripper_commands[("left", finger, j)] = finger_open_targets[j]

            if right_gesture == "close":
                for finger in ("a", "b", "c"):
                    for j in ("joint1", "joint2", "joint3"):
                        gripper_commands[("right", finger, j)] = finger_close_targets[j]
            elif right_gesture == "open":
                for finger in ("a", "b", "c"):
                    for j in ("joint1", "joint2", "joint3"):
                        gripper_commands[("right", finger, j)] = finger_open_targets[j]

            if gripper_commands:
                new_posture_target = configuration.q.copy()
                all_forces = get_gripper_contact_forces(data)

                for (side, finger, j), target_val in gripper_commands.items():
                    is_closing = np.isclose(target_val, finger_close_targets[j], atol=1e-3)
                    threshold = FORCE_THRESHOLD_LEFT if side == "left" else FORCE_THRESHOLD_RIGHT
                    finger_force = all_forces[side][finger]
                    
                    if is_closing and finger_force > threshold:
                        current_qpos = data.qpos[finger_qpos_adrs[side][finger][j]]
                        data.ctrl[finger_ctrl_ids[side][finger][j]] = current_qpos
                        new_posture_target[finger_qpos_adrs[side][finger][j]] = current_qpos
                        
                        if force_log_counter % 100 == 0:
                            print(f"  -> [STOPPED] {side.upper()} Finger {finger.upper()} held due to high force ({finger_force:.2f} N > {threshold} N)")
                    else:
                        data.ctrl[finger_ctrl_ids[side][finger][j]] = target_val
                        new_posture_target[finger_qpos_adrs[side][finger][j]] = target_val
                        
                posture_task.target = new_posture_target
            
            vel = mink.solve_ik(configuration, tasks, rte.dt, solver, safety_break=False, damping=1e-4, limits=limits)
            configuration.integrate_inplace(vel, rte.dt)

            data.ctrl[motor_ctrl_ids] = configuration.q[motor_qpos_adrs]
            mujoco.mj_step(model, data, nstep=10)
            configuration.update(data.qpos)

            scene = viewer.prepare_scene(data)

            if world_pts is not None and n_pts > 0:
                for i in range(n_pts):
                    if TRACKING_MODE == "upper-only" and 25 <= i <= 32:
                        continue
                    
                    if scene.ngeom >= scene.maxgeom:
                        break
                    g = scene.geoms[scene.ngeom]
                    mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_SPHERE,
                        size=POINT_SIZE_GEOM,
                        pos=world_pts[i],
                        mat=EYE_MAT_FLAT,
                        rgba=COLORS_GEOM[i])
                    scene.ngeom += 1

                for c in SKELETON_CONNECTIONS:
                    if TRACKING_MODE == "upper-only":
                        if (25 <= c[0] <= 32) or (25 <= c[1] <= 32):
                            continue
                    
                    if c[0] >= n_pts or c[1] >= n_pts:
                        continue
                    if scene.ngeom >= scene.maxgeom:
                        break
                    p1, p2 = world_pts[c[0]], world_pts[c[1]]
                    vec = p2 - p1
                    length = np.linalg.norm(vec)
                    if length < 1e-5:
                        continue
                    z_ax = vec / length
                    x_ax = np.cross(z_ax, [1,0,0]) if abs(z_ax[0])<0.9 else np.cross(z_ax, [0,1,0])
                    x_ax /= np.linalg.norm(x_ax)
                    y_ax = np.cross(z_ax, x_ax)
                    mat = np.array([x_ax, y_ax, z_ax]).T.flatten()
                    g = scene.geoms[scene.ngeom]
                    mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_CAPSULE,
                        size=(0.005, length/2.0, 0.0),
                        pos=(p1+p2)/2.0, 
                        mat=mat,
                        rgba=SKELETON_RGBA_GEOM)
                    scene.ngeom += 1
                    
            viewer.render_frame(scene)
            rte.sleep()
            force_log_counter += 1

    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        tracking_sub.delete()
        gesture_sub.delete()
        viewer.close()
        print("Subscriber cleanup complete")