from core.manipulator.kinematics import MorphIManipulator, _fk_kernel
import numpy as np
from typing import Optional
from scipy.optimize import minimize
import mujoco

def world_ee_pose(base_pose, q_arm, arm_offset, manipulator: MorphIManipulator = None):
    x_b, y_b, theta_b = base_pose
    ee_local = manipulator.fk(q_arm)

    c, s = np.cos(theta_b), np.sin(theta_b)
    R = np.array([[c, -s],
                  [s,  c]])

    arm_base_world = R @ arm_offset[:2]
    ee_local_world = R @ ee_local[:2]
    world_x = x_b + arm_base_world[0] + ee_local_world[0]
    world_y = y_b + arm_base_world[1] + ee_local_world[1]
    world_z = arm_offset[2] + ee_local[2]

    return np.array([world_x, world_y, world_z])


def hierarchical_wbc(
    target_world: np.ndarray,
    base_pose: np.ndarray,
    arm_q_guess: np.ndarray,
    manipulator: MorphIManipulator,
    arm_offset: np.ndarray = np.array([0.15, 0.15, 0.158566]),
    position_tolerance: float = 0.03,
    base_bounds: tuple = ((-5, 5), (-5, 5), (-np.pi, np.pi)),
    max_wbc_iter: int = 30,
    pitch_target: Optional[float] = None,
) -> tuple[np.ndarray, np.ndarray, bool]:

    target_world = np.asarray(target_world, dtype=np.float64)
    base_pose = np.asarray(base_pose, dtype=np.float64)
    if target_world.shape != (3,):
        raise ValueError("target must be (3,)")

    x_b, y_b, theta_b = base_pose

    c, s = np.cos(theta_b), np.sin(theta_b)
    R = np.array([[c, -s],
                  [s,  c]])

    arm_base_xy = np.array([x_b, y_b]) + R @ arm_offset[:2]
    arm_base_z = arm_offset[2]

    vec_world_xy = target_world[:2] - arm_base_xy
    vec_base_xy = R.T @ vec_world_xy

    target_arm = np.array([
        -vec_base_xy[0],   
        -vec_base_xy[1],  
        target_world[2] - arm_base_z
    ])

    try:
        arm_sol = manipulator.ik(target_arm, arm_q_guess, pitch_target=pitch_target)
        ee_world_check = world_ee_pose(base_pose, arm_sol, arm_offset, manipulator)
        if np.linalg.norm(ee_world_check - target_world) <= position_tolerance:
           return base_pose.copy(), arm_sol, False
    except Exception:
        pass

    def total_cost(vars):
        x, y, th, h1, h2, a1, theta, phi = vars
        base_vars = np.array([x, y, th])
        arm_vars = np.array([h1, h2, a1, theta, phi])
        ee_world = world_ee_pose(base_vars, arm_vars, arm_offset, manipulator)
        pos_err = np.sum((ee_world - target_world) ** 2)
        if pitch_target is not None:
            pos_err += 50.0 * (phi - pitch_target) ** 2
        base_reg = 1e-3 * (x**2 + y**2 + th**2)
        return pos_err + base_reg

    def min_angle_con(vars):
        _, _, _, h1, h2, a1, theta, phi = vars
        alpha = np.arctan2(np.abs(h2 - h1), manipulator.d2)
        return alpha - manipulator.alpha_min_rad

    def lateral_con(vars):
        _, _, _, h1, h2, a1, theta, phi = vars
        ee = _fk_kernel(h1, h2, a1, theta, phi, manipulator.d2, manipulator.l3_max, manipulator.wrist_length)
        lat = np.sqrt(ee[0]**2 + ee[1]**2)
        return lat - manipulator.min_lateral_dist

    arm_bounds = [
        manipulator.bounds_h,
        manipulator.bounds_h,
        manipulator.bounds_a,
        manipulator.bounds_theta,
        manipulator.bounds_phi
    ]
    bounds = [base_bounds[0], base_bounds[1], base_bounds[2], *arm_bounds]
    x0 = np.concatenate([base_pose, arm_q_guess])

    constraints = [
        {"type": "ineq", "fun": min_angle_con},
        {"type": "ineq", "fun": lateral_con}
    ]

    res = minimize(
        total_cost,
        x0,
        method="SLSQP",
        bounds=bounds,
        constraints=constraints,
        options={"ftol": 1e-6, "maxiter": max_wbc_iter, "disp": False}
    )

    if res.success:
        return res.x[:3], res.x[3:], True
    else:
        return base_pose.copy(), arm_q_guess.copy(), True
    
def dual_arm_wbc(
    base_pose_initial: np.ndarray,
    left_target: np.ndarray,
    right_target: np.ndarray,
    q_left_guess: np.ndarray,
    q_right_guess: np.ndarray,
    manip_left: MorphIManipulator,
    manip_right: MorphIManipulator,
    left_offset: np.ndarray,
    right_offset: np.ndarray,
    pitch_targets: dict = None,
    max_iter: int = 20  
):
    if pitch_targets is None:
        pitch_targets = {"left": None, "right": None}

    x0 = np.concatenate([base_pose_initial, q_left_guess, q_right_guess])

    # Bounds: [base_x, base_y, base_yaw, left_arm, right_arm]
    print(manip_left.bounds_theta)
    bounds = [
        (-np.inf, np.inf), (-np.inf, np.inf), (-np.pi, np.pi),  # base
        *[
            manip_left.bounds_h,
            manip_left.bounds_h,
            manip_left.bounds_a,
            manip_left.bounds_theta,
            manip_left.bounds_phi
        ],
        *[
            manip_right.bounds_h,
            manip_right.bounds_h,
            manip_right.bounds_a,
            manip_right.bounds_theta,
            manip_right.bounds_phi
        ]
    ]

    def cost_function(x):
        base = x[0:3]
        qL = x[3:8]
        qR = x[8:13]

        eeL = world_ee_pose(base, qL, left_offset, manip_left)
        eeR = world_ee_pose(base, qR, right_offset, manip_right)

        errL = np.sum((eeL - left_target) ** 2)
        errR = np.sum((eeR - right_target) ** 2)

        cost = errL + errR

        if pitch_targets["left"] is not None:
            cost += 5.0 * (qL[4] - np.radians(pitch_targets["left"])) ** 2
        if pitch_targets["right"] is not None:
            cost += 5.0 * (qR[4] - np.radians(pitch_targets["right"])) ** 2

        cost += 1e-5 * (base[0]**2 + base[1]**2 + base[2]**2)

        return cost
    
    def min_angle_con_left(x):
        _, _, _, h1, h2, a1, theta, phi = x[0:8]
        alpha = np.arctan2(np.abs(h2 - h1), manip_left.d2)
        return alpha - manip_left.alpha_min_rad

    def min_angle_con_right(x):
        _, _, _, _, _, _, _, _, h1, h2, a1, theta, phi = x
        alpha = np.arctan2(np.abs(h2 - h1), manip_right.d2)
        return alpha - manip_right.alpha_min_rad

    def lateral_con_left(x):
        _, _, _, h1, h2, a1, theta, phi = x[0:8]
        ee = _fk_kernel(h1, h2, a1, theta, phi, manip_left.d2, manip_left.l3_max, manip_left.wrist_length)
        lat = np.sqrt(ee[0]**2 + ee[1]**2)
        return lat - manip_left.min_lateral_dist

    def lateral_con_right(x):
        _, _, _, _, _, _, _, _, h1, h2, a1, theta, phi = x
        ee = _fk_kernel(h1, h2, a1, theta, phi, manip_right.d2, manip_right.l3_max, manip_right.wrist_length)
        lat = np.sqrt(ee[0]**2 + ee[1]**2)
        return lat - manip_right.min_lateral_dist

    constraints = [
        {"type": "ineq", "fun": min_angle_con_left},
        {"type": "ineq", "fun": min_angle_con_right},
        {"type": "ineq", "fun": lateral_con_left},
        {"type": "ineq", "fun": lateral_con_right}
    ]

    res = minimize(
        cost_function,
        x0,
        method='SLSQP',
        bounds=bounds,
        constraints=constraints,
        options={
            'ftol': 1e-8,      
            'maxiter': max_iter,     
            'disp': True
        }
    )

    if res.success:
        base_sol = res.x[0:3]
        qL_sol = res.x[3:8]
        qR_sol = res.x[8:13]
        used_wbc = True
        return base_sol, qL_sol, qR_sol, used_wbc

def get_body_name_from_geom_id(model, geom_id):
    """Get the body name that owns a given geom ID."""
    if geom_id < 0 or geom_id >= model.ngeom:
        return None
    
    body_id = model.geom_bodyid[geom_id]
    if body_id < 0:
        return "world"
    
    body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
    return body_name

def should_ignore_contact(body1_name, body2_name):
    is_body1_floor = (body1_name == "floor")
    is_body2_floor = (body2_name == "floor")
    
    if not (is_body1_floor or is_body2_floor):
        return False
        
    non_floor_body = body2_name if is_body1_floor else body1_name
    
    if non_floor_body is None:
        return False
        
    if "roller" in non_floor_body.lower():
        return True
    if "trashbin" in non_floor_body.lower():
        return True
    if "one_nescaafe_taster" in non_floor_body.lower():
        return True
        
    return False

def is_configuration_valid(model, data, base, qL, qR, joint_names_left, joint_names_right):
    """Check if configuration is valid (collision-free)"""
    original_qpos = data.qpos.copy()
    try:
        # Find base joint address (assuming floating base is first joint)
        base_adr = 0
        for jnt_id in range(model.njnt):
            if model.jnt_type[jnt_id] == 0:  # free joint
                base_adr = model.jnt_qposadr[jnt_id]
                break
        
        # Set base pose
        data.qpos[base_adr + 0] = base[0]
        data.qpos[base_adr + 1] = base[1]
        # data.qpos[base_adr + 2] = 0.0  # z height
        cy = np.cos(base[2] * 0.5)
        sy = np.sin(base[2] * 0.5)
        data.qpos[base_adr + 3] = cy
        data.qpos[base_adr + 4] = 0.0
        data.qpos[base_adr + 5] = 0.0
        data.qpos[base_adr + 6] = sy

        # Set left arm joints
        for i, name in enumerate(joint_names_left):
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            addr = model.jnt_qposadr[joint_id]
            data.qpos[addr] = qL[i]
            
        # Set right arm joints
        for i, name in enumerate(joint_names_right):
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            addr = model.jnt_qposadr[joint_id]
            data.qpos[addr] = qR[i]

        mujoco.mj_forward(model, data)
        
        # Check for real collisions
        for i in range(data.ncon):
            contact = data.contact[i]
            geom1_id = contact.geom1
            geom2_id = contact.geom2
            
            body1_name = get_body_name_from_geom_id(model, geom1_id)
            body2_name = get_body_name_from_geom_id(model, geom2_id)
            
            if not should_ignore_contact(body1_name, body2_name):
                return False
        return True
        
    finally:
        data.qpos[:] = original_qpos
        mujoco.mj_forward(model, data)

def compute_collision_penalty(model, data, original_qpos, base, qL, qR, joint_names_left, joint_names_right, penalty_weight=1000.0):
    """Compute collision penalty for optimization"""
    try:
        # Find base joint address
        base_adr = 0
        for jnt_id in range(model.njnt):
            if model.jnt_type[jnt_id] == 0:  # free joint
                base_adr = model.jnt_qposadr[jnt_id]
                break
        
        # Set robot state
        data.qpos[base_adr + 0] = base[0]
        data.qpos[base_adr + 1] = base[1]
        data.qpos[base_adr + 2] = 0.0
        cy = np.cos(base[2] * 0.5)
        sy = np.sin(base[2] * 0.5)
        data.qpos[base_adr + 3] = cy
        data.qpos[base_adr + 4] = 0.0
        data.qpos[base_adr + 5] = 0.0
        data.qpos[base_adr + 6] = sy

        for i, name in enumerate(joint_names_left):
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            addr = model.jnt_qposadr[joint_id]
            data.qpos[addr] = qL[i]
        for i, name in enumerate(joint_names_right):
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            addr = model.jnt_qposadr[joint_id]
            data.qpos[addr] = qR[i]

        mujoco.mj_forward(model, data)
        
        # Count real collisions
        collision_count = 0
        for i in range(data.ncon):
            contact = data.contact[i]
            geom1_id = contact.geom1
            geom2_id = contact.geom2
            
            body1_name = get_body_name_from_geom_id(model, geom1_id)
            body2_name = get_body_name_from_geom_id(model, geom2_id)
            
            if not should_ignore_contact(body1_name, body2_name):
                collision_count += 1
        
        return penalty_weight * collision_count
        
    finally:
        data.qpos[:] = original_qpos
        mujoco.mj_forward(model, data)

def dual_arm_wbc_collision(
    base_pose_initial: np.ndarray,
    left_target: np.ndarray,
    right_target: np.ndarray,
    q_left_guess: np.ndarray,
    q_right_guess: np.ndarray,
    manip_left,
    manip_right,
    left_offset: np.ndarray,
    right_offset: np.ndarray,
    joint_names_left,
    joint_names_right,
    pitch_targets: dict = None,
    max_iter: int = 20,
    mujoco_model=None,
    mujoco_data=None
):
    if pitch_targets is None:
        pitch_targets = {"left": None, "right": None}

    x0 = np.concatenate([base_pose_initial, q_left_guess, q_right_guess])
    print(manip_left.bounds_theta)

    # Bounds: [base_x, base_y, base_yaw, left_arm, right_arm]
    bounds = [
        (-np.inf, np.inf), (-np.inf, np.inf), (-np.pi, np.pi),  # base
        *[
            manip_left.bounds_h,
            manip_left.bounds_h,
            manip_left.bounds_a,
            manip_left.bounds_theta,
            manip_left.bounds_phi
        ],
        *[
            manip_right.bounds_h,
            manip_right.bounds_h,
            manip_right.bounds_a,
            manip_right.bounds_theta,
            manip_right.bounds_phi
        ]
    ]
    
    original_qpos = mujoco_data.qpos.copy()
    
    def cost_function(x):
        base = x[0:3]
        qL = x[3:8]
        qR = x[8:13]

        eeL = world_ee_pose(base, qL, left_offset, manip_left)
        eeR = world_ee_pose(base, qR, right_offset, manip_right)

        errL = np.sum((eeL - left_target) ** 2)
        errR = np.sum((eeR - right_target) ** 2)

        cost = errL + errR

        if pitch_targets["left"] is not None:
            cost += 5.0 * (qL[4] - np.radians(pitch_targets["left"])) ** 2
        if pitch_targets["right"] is not None:
            cost += 5.0 * (qR[4] - np.radians(pitch_targets["right"])) ** 2

        cost += 1e-5 * (base[0]**2 + base[1]**2 + base[2]**2)

        if mujoco_model is not None and mujoco_data is not None:
            collision_penalty = compute_collision_penalty(
                mujoco_model, mujoco_data, original_qpos, base, qL, qR,
                joint_names_left, joint_names_right,
                penalty_weight=1000.0
            )
            cost += collision_penalty

        return cost
    
    def min_angle_con_left(x):
        _, _, _, h1, h2, a1, theta, phi = x[0:8]
        alpha = np.arctan2(np.abs(h2 - h1), manip_left.d2)
        return alpha - manip_left.alpha_min_rad

    def min_angle_con_right(x):
        _, _, _, _, _, _, _, _, h1, h2, a1, theta, phi = x
        alpha = np.arctan2(np.abs(h2 - h1), manip_right.d2)
        return alpha - manip_right.alpha_min_rad

    def lateral_con_left(x):
        _, _, _, h1, h2, a1, theta, phi = x[0:8]
        ee = _fk_kernel(h1, h2, a1, theta, phi, manip_left.d2, manip_left.l3_max, manip_left.wrist_length)
        lat = np.sqrt(ee[0]**2 + ee[1]**2)
        return lat - manip_left.min_lateral_dist

    def lateral_con_right(x):
        _, _, _, _, _, _, _, _, h1, h2, a1, theta, phi = x
        ee = _fk_kernel(h1, h2, a1, theta, phi, manip_right.d2, manip_right.l3_max, manip_right.wrist_length)
        lat = np.sqrt(ee[0]**2 + ee[1]**2)
        return lat - manip_right.min_lateral_dist

    constraints = [
        {"type": "ineq", "fun": min_angle_con_left},
        {"type": "ineq", "fun": min_angle_con_right},
        {"type": "ineq", "fun": lateral_con_left},
        {"type": "ineq", "fun": lateral_con_right}
    ]

    res = minimize(
        cost_function,
        x0,
        method='SLSQP',
        bounds=bounds,
        constraints=constraints,
        options={
            'ftol': 1e-8,      
            'maxiter': max_iter,     
            'disp': True
        }
    )

    if res.success:
        base_sol = res.x[0:3]
        qL_sol = res.x[3:8]
        qR_sol = res.x[8:13]
        used_wbc = True
        return base_sol, qL_sol, qR_sol, used_wbc