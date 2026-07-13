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
_XML = _HERE / "env" / "robot_description" / "robot" / "morph_i.xml"

# Separate thresholds for left and right hands (in Newtons)
FORCE_THRESHOLD_LEFT = 15.0
FORCE_THRESHOLD_RIGHT = 15.0

# Movement step size for IK targets (in meters)
TARGET_MOVE_STEP = 0.005  # 1 cm per frame

def get_gripper_contact_force_per_finger(data: mujoco.MjData, side: str) -> dict:
    """
    Evaluate the contact force for each finger (A, B, C) individually on a given side.
    Returns a dict like {'a': force_a, 'b': force_b, 'c': force_c}
    """
    forces = {'a': 0.0, 'b': 0.0, 'c': 0.0}
    for i in range(1, data.model.nbody):
        body_name = data.model.body(i).name
        if body_name and side in body_name:
            if "finger_a" in body_name and ("proximal" in body_name or "middle" in body_name or "distal" in body_name):
                forces['a'] += np.linalg.norm(data.cfrc_ext[i, 3:6])
            elif "finger_b" in body_name and ("proximal" in body_name or "middle" in body_name or "distal" in body_name):
                forces['b'] += np.linalg.norm(data.cfrc_ext[i, 3:6])
            elif "finger_c" in body_name and ("proximal" in body_name or "middle" in body_name or "distal" in body_name):
                forces['c'] += np.linalg.norm(data.cfrc_ext[i, 3:6])
    return forces

PARENT_BODY_NAME = "mobile_base"
BASE_TRANSFORM = StaticTransform(
    x=0.3, y=0.0, z=0.0,
    roll=90.0, pitch=0.0, yaw=90.0,
    use_degrees=True,
)
HIP_TRANSFORM = HipAnchoredTransform(
    static_tf=BASE_TRANSFORM, anchor_init=True,
    foot_on_ground=True, ground_level=0.0, foot_offset=0.0,
)

def get_body_world_pose(data: mujoco.MjData, body_name: str):
    body_id = data.model.body(body_name).id
    return data.xpos[body_id].copy(), data.xmat[body_id].reshape(3, 3).copy()

def quat_to_mat(quat):
    """Convert MuJoCo quaternion (w, x, y, z) to a 3x3 rotation matrix."""
    w, x, y, z = quat
    return np.array([
        [1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
        [2*x*y + 2*z*w, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
        [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x**2 - 2*y**2]
    ])

if __name__ == "__main__":
    model = mujoco.MjModel.from_xml_path(_XML.as_posix())

    model.vis.quality.shadowsize = 0
    for i in range(model.nmat):
        model.mat_reflectance[i] = 0.0
    for i in range(model.nlight):
        model.light_castshadow[i] = 0

    data = mujoco.MjData(model)
    configuration = mink.Configuration(model)

    PREFIXES = ["left", "right"]
    ARM_MOTORS = [
        "base_rotation", "left_column_lift", "right_column_lift",
        "telescopic_extend", "wrist_pitch", "gripper_roll_joint",
        "gripper_pitch_joint", "gripper_yaw_joint"
    ]

    motor_joint_names = [f"{prefix}_{motor}" for prefix in PREFIXES for motor in ARM_MOTORS]
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

    finger_close_targets = {"joint1": 0.33, "joint2": 0.0, "joint3": 0.0}
    finger_open_targets  = {"joint1": 0.0,  "joint2": 0.0, "joint3": 0.0}

    tasks = [
        left_end_effector_task := mink.FrameTask(
            frame_name="left_ee_site", frame_type="site",
            position_cost=100.0, orientation_cost=20.0, lm_damping=1e-6,
        ),
        right_end_effector_task := mink.FrameTask(
            frame_name="right_ee_site", frame_type="site",
            position_cost=100.0, orientation_cost=20.0, lm_damping=1e-6,
        ),
        posture_task := mink.PostureTask(model=model, cost=1e-3),
    ]
    equality_task = mink.EqualityConstraintTask(model=model, cost=1000.0, gain=1.0, lm_damping=1e-3)
    tasks.append(equality_task)

    left_arm_geoms = mink.get_subtree_geom_ids(model, model.body("left_arm_base").id)
    right_arm_geoms = mink.get_subtree_geom_ids(model, model.body("right_arm_base").id)
    base_geoms = mink.get_body_geom_ids(model, model.body("mobile_base").id)

    collision_avoidance_limit = mink.CollisionAvoidanceLimit(
        model=model,
        geom_pairs=[
            (left_arm_geoms, right_arm_geoms),
            (left_arm_geoms, base_geoms),
            (right_arm_geoms, base_geoms),
        ],
        minimum_distance_from_collisions=0.05,
        collision_detection_distance=0.1,
    )
    limits = [
        mink.ConfigurationLimit(model=model),
        collision_avoidance_limit,
        mink.VelocityLimit(model=model),
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
    scene.flags[mujoco.mjtRndFlag.mjRND_FOG.value] = 0
    opt = mujoco.MjvOption()
    opt.frame = mujoco.mjtFrame.mjFRAME_NONE

    viewer = GlfwViewer(width=960, height=1000, title="Pick and Place Tracking")
    viewer.setup(model, camera, scene, opt)

    # Helper function to perform a full reset
    def full_reset():
        """Reset the entire robot state to the home keyframe."""
        if model.nkey > 0 and "home" in [model.key(i).name for i in range(model.nkey)]:
            mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
        else:
            mujoco.mj_resetData(model, data)
        
        # Re-initialize configuration and posture task
        configuration.update(data.qpos)
        posture_task.set_target_from_configuration(configuration)
        
        # Reset IK targets to current end-effector positions
        mujoco.mj_forward(model, data)
        mink.move_mocap_to_frame(model, data, "left_ik_target", "left_ee_site", "site")
        mink.move_mocap_to_frame(model, data, "right_ik_target", "right_ee_site", "site")
        
        print("[FULL RESET] Robot reset to home keyframe, IK targets synced to end-effectors")

    # Initial setup
    full_reset()

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
                raw_pts = np.zeros((n_pts, 3), dtype=np.float64)
                for i in range(n_pts):
                    p = poses_list[i]['position']
                    raw_pts[i] = [p['x'], p['y'], p['z']]
                world_pts = HIP_TRANSFORM.transform(raw_pts, parent_pos, parent_mat)

                # ====================================================================
                # EXTRACT WRIST POSITIONS AND ORIENTATIONS
                # ====================================================================
                if n_pts > 18:  # Ensure we have enough landmarks (up to right index)
                    left_mid_palm = world_pts[15].copy()
                    right_mid_palm = world_pts[16].copy()
                    
                    # Left wrist (15) to left index (17)
                    vec_left = world_pts[17] - world_pts[15]
                    if np.linalg.norm(vec_left) > 1e-5:
                        z_left = vec_left / np.linalg.norm(vec_left)
                        x_left = np.cross(z_left, [0, 0, 1])
                        if np.linalg.norm(x_left) < 1e-5:
                            x_left = np.cross(z_left, [0, 1, 0])
                        x_left /= np.linalg.norm(x_left)
                        y_left = np.cross(z_left, x_left)
                        mat_left = np.array([x_left, y_left, z_left]).T
                        rot_left = Rotation.from_matrix(mat_left)
                        # MuJoCo expects w, x, y, z
                        left_quat = np.array([rot_left.as_quat()[3], rot_left.as_quat()[0], rot_left.as_quat()[1], rot_left.as_quat()[2]])
                    else:
                        left_quat = np.array([1.0, 0.0, 0.0, 0.0])

                    # Right wrist (16) to right index (18)
                    vec_right = world_pts[18] - world_pts[16]
                    if np.linalg.norm(vec_right) > 1e-5:
                        z_right = vec_right / np.linalg.norm(vec_right)
                        x_right = np.cross(z_right, [0, 0, 1])
                        if np.linalg.norm(x_right) < 1e-5:
                            x_right = np.cross(z_right, [0, 1, 0])
                        x_right /= np.linalg.norm(x_right)
                        y_right = np.cross(z_right, x_right)
                        mat_right = np.array([x_right, y_right, z_right]).T
                        rot_right = Rotation.from_matrix(mat_right)
                        right_quat = np.array([rot_right.as_quat()[3], rot_right.as_quat()[0], rot_right.as_quat()[1], rot_right.as_quat()[2]])
                    else:
                        right_quat = np.array([1.0, 0.0, 0.0, 0.0])
                # ====================================================================

            window = glfw.get_current_context()
            left_kb_active = False
            right_kb_active = False

            if window:
                l_shift = glfw.get_key(window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
                r_shift = glfw.get_key(window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS
                
                # ====================================================================
                # FULL RESET WITH BACKSPACE
                # ====================================================================
                if glfw.get_key(window, glfw.KEY_BACKSPACE) == glfw.PRESS:
                    if not getattr(viewer, '_backspace', False):
                        full_reset()
                        viewer._backspace = True
                else:
                    viewer._backspace = False
                # ====================================================================
                
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

            # Apply tracking data to IK targets (Position + Orientation)
            if not left_kb_active and left_mid_palm is not None and left_quat is not None:
                data.mocap_pos[left_mocap_id] = left_mid_palm
                # data.mocap_quat[left_mocap_id] = left_quat

            if not right_kb_active and right_mid_palm is not None and right_quat is not None:
                data.mocap_pos[right_mocap_id] = right_mid_palm
                # data.mocap_quat[right_mocap_id] = right_quat

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
                
                left_forces = get_gripper_contact_force_per_finger(data, "left")
                right_forces = get_gripper_contact_force_per_finger(data, "right")
                
                force_log_counter += 1
                if force_log_counter % 100 == 0:
                    print(f"\n[LEFT HAND Forces]  A:{left_forces['a']:.2f} N | B:{left_forces['b']:.2f} N | C:{left_forces['c']:.2f} N (Threshold: {FORCE_THRESHOLD_LEFT} N)")
                    print(f"[RIGHT HAND Forces] A:{right_forces['a']:.2f} N | B:{right_forces['b']:.2f} N | C:{right_forces['c']:.2f} N (Threshold: {FORCE_THRESHOLD_RIGHT} N)")
                
                for (side, finger, j), target_val in gripper_commands.items():
                    is_closing = np.isclose(target_val, finger_close_targets[j], atol=1e-3)
                    
                    if side == "left":
                        forces = left_forces
                        threshold = FORCE_THRESHOLD_LEFT
                    else:
                        forces = right_forces
                        threshold = FORCE_THRESHOLD_RIGHT
                    
                    finger_force = forces[finger]
                    
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

            vel = mink.solve_ik(configuration, tasks, rte.dt, solver,
                                safety_break=False, damping=1e-6, limits=limits)
            configuration.integrate_inplace(vel, rte.dt)

            data.ctrl[motor_ctrl_ids] = configuration.q[motor_qpos_adrs]
            mujoco.mj_step(model, data, nstep=5)
            configuration.update(data.qpos)

            scene = viewer.prepare_scene(data)

            if world_pts is not None and n_pts > 0:
                colors = [[0,1,1,1]]*33 + [[1,0,1,1]]*21 + [[1,0.5,0,1]]*21

                for i in range(n_pts):
                    if scene.ngeom >= scene.maxgeom:
                        break
                    g = scene.geoms[scene.ngeom]
                    mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_SPHERE,
                        size=np.array([0.01,0,0]), pos=world_pts[i],
                        mat=np.eye(3).flatten(), rgba=np.array(colors[i]))
                    scene.ngeom += 1

                for c in SKELETON_CONNECTIONS:
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
                        size=np.array([0.005, length/2.0, 0.0]),
                        pos=(p1+p2)/2.0, mat=mat,
                        rgba=np.array([0.5,0.5,0.5,1.0]))
                    scene.ngeom += 1

                if left_mid_palm is not None and scene.ngeom < scene.maxgeom:
                    g = scene.geoms[scene.ngeom]
                    mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_SPHERE,
                        size=np.array([0.025, 0, 0]), pos=left_mid_palm,
                        mat=np.eye(3).flatten(), rgba=np.array([1.0, 0.0, 0.0, 1.0]))
                    scene.ngeom += 1
                    
                if right_mid_palm is not None and scene.ngeom < scene.maxgeom:
                    g = scene.geoms[scene.ngeom]
                    mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_SPHERE,
                        size=np.array([0.025, 0, 0]), pos=right_mid_palm,
                        mat=np.eye(3).flatten(), rgba=np.array([0.0, 0.0, 1.0, 1.0]))
                    scene.ngeom += 1
                    
            viewer.render_frame(scene)
            rte.sleep()

    except KeyboardInterrupt:
        print("\n[Main] Interrupted by user.")
    finally:
        tracking_sub.delete()
        gesture_sub.delete()
        viewer.close()
        print("Subscriber cleanup complete.")