import numpy as np
from scipy.optimize import minimize, Bounds
from scipy.spatial.transform import Rotation as R

class MorphIManipulator:
    def __init__(self, distance_between_vertical: float, vertical_height: float, horizontal_length:float, eps: float):
        self.distance_between_vertical = distance_between_vertical
        self.horizontal_length = horizontal_length 
        self.vertical_height = vertical_height 
        self.eps = eps
    
    def fk(self, vertical_left: float, vertical_right: float, horizontal: float, base_angle: float):
        """Forward kinematics: returns end-effector position."""
        p1 = np.array([0.0, 0.0, vertical_left])
        p2 = np.array([
            self.distance_between_vertical * np.cos(base_angle),
            self.distance_between_vertical * np.sin(base_angle),
            vertical_right
        ])
        vec = p1 - p2
        dist = np.linalg.norm(vec)
        if dist < self.eps:
            return None

        u = vec / dist
        ee = p1 + horizontal * u

        if np.linalg.norm(ee - p1) > self.vertical_height + 1e-9:
            return None

        return ee

    def ik(self, target, alpha_min_deg=10.0,
           tol=1e-6,
           cache_threshold=0.001):
        """
        Solve inverse kinematics for the manipulator.
        """
        target = np.array(target, dtype=float)
        d2 = d2 if d2 is not None else self.distance_between_vertical

        # Use single cache slot
        if not hasattr(self, '_ik_cache'):
            self._ik_cache = {'target': None, 'result': None}

        cache = self._ik_cache
        if cache['target'] is not None:
            if np.linalg.norm(target - cache['target']) <= cache_threshold:
                return cache['result'].copy()

        prev_target = cache['target'] if cache['target'] is not None else np.zeros(3)
        delta_norm = np.linalg.norm(target - prev_target)
        print(f"[IK] Recomputing: target = {target} (Δ = {delta_norm:.4f} m)")

        def cost(vars, w_a1=1e-2):
            h1, h2, a1, theta = vars
            ee = self.fk(h1, h2, a1, theta)
            if ee is None:
                return 1e3 + 1e2 * np.linalg.norm(np.array([h1, h2, a1]) - 0.5)
            dist_err = np.sum((ee - target) ** 2)
            return float(dist_err + w_a1 * a1)

        def angle_ineq(vars):
            h1, h2, a1, theta = vars
            alpha_deg = np.degrees(np.arctan2(h2 - h1, d2))
            return alpha_deg - alpha_min_deg  # >= 0 when constraint satisfied

        constraints = ({'type': 'ineq', 'fun': angle_ineq},)
        bounds = [
            (bounds_h[0], bounds_h[1]),  # h1
            (bounds_h[0], bounds_h[1]),  # h2
            (bounds_a[0], bounds_a[1]),  # a1
            (-np.pi, np.pi)              # theta
        ]

        x0 = self.get_encoders()  # full initial guess

        res = minimize(
            cost, x0,
            method='SLSQP',
            bounds=bounds,
            constraints=constraints,
            options={'ftol': 1e-9, 'maxiter': 50, 'disp': False}
        )

        if not res.success:
            print("[IK] Failed! Using fallback.")
            if cache['result'] is not None:
                return cache['result'].copy()
            else:
                return x0.copy()

        result = np.array([float(v) for v in res.x])

        cache['target'] = target.copy()
        cache['result'] = result.copy()
        return result

class MorphIManipulatorV2:
    def __init__(self, 
                 actuator_offset: float = 0.1,
                 l_wrist: float = 0.035,
                 eps: float = 1e-9,
                 alpha_min_deg: float = 10.0,
                 bound_z_act: tuple = (0.0, 1.5),
                 bound_telescopic: tuple = (0.0, 0.7),
                 bound_theta_base: tuple = (-np.pi, np.pi),
                 bound_phi_wrist: tuple = (-np.pi, np.pi),
                 bound_roll_gripper: tuple = (-0.8, 0.8),
                 bound_pitch_gripper: tuple = (-np.pi, np.pi),
                 bound_yaw_gripper: tuple = (-2*np.pi, 2*np.pi)):

        self.d2 = actuator_offset          # d2 from fk_8dof_full
        self.L_w = l_wrist                 # L_w from fk_8dof_full
        self.eps = eps
        self.alpha_min_deg = alpha_min_deg
        
        # Bounds (theta_base was missing before)
        self.bound_z_act = bound_z_act
        self.bound_telescopic = bound_telescopic
        self.bound_theta_base = bound_theta_base
        self.bound_phi_wrist = bound_phi_wrist
        self.bound_roll_gripper = bound_roll_gripper
        self.bound_pitch_gripper = bound_pitch_gripper
        self.bound_yaw_gripper = bound_yaw_gripper

    def _get_joint_bounds(self):
        return [
            self.bound_z_act,           # h1
            self.bound_z_act,           # h2  
            self.bound_telescopic,      # a1
            self.bound_theta_base,      # theta
            self.bound_phi_wrist,       # phi
            self.bound_roll_gripper,    # roll
            self.bound_pitch_gripper,   # pitch
            self.bound_yaw_gripper      # yaw
        ]

    def get_joint_bounds(self):
        bounds = self._get_joint_bounds()
        lower = [b[0] for b in bounds]
        upper = [b[1] for b in bounds]
        return lower, upper

    def fk(self, q):
        """
        q = [h1, h2, a1, theta, phi, roll, pitch, yaw]
        """
        h1, h2, a1, theta, phi, roll, pitch, yaw = q

        p1 = np.array([0.0, 0.0, h1])
        p2 = np.array([self.d2 * np.cos(theta), 
                       self.d2 * np.sin(theta), h2])
        vec = p1 - p2
        dist = np.linalg.norm(vec)
        if dist < self.eps:
            dist = self.eps
        u = vec / dist
        wrist_base = p1 + a1 * u
        z_local = u

        if abs(z_local[2]) < 0.99:
            ref = np.array([0.0, 0.0, 1.0])
        else:
            ref = np.array([1.0, 0.0, 0.0])
        x_local = ref - np.dot(ref, z_local) * z_local
        x_local_norm = np.linalg.norm(x_local)
        if x_local_norm < self.eps:
            ref_alt = np.array([1.0, 0.0, 0.0])
            x_local = ref_alt - np.dot(ref_alt, z_local) * z_local
            x_local_norm = np.linalg.norm(x_local)
        x_local = x_local / x_local_norm
        y_local = np.cross(z_local, x_local)
        R0 = np.column_stack((x_local, y_local, z_local))

        wrist_bend_local = np.array([np.sin(phi), 0.0, np.cos(phi)])
        wrist_z = R0 @ wrist_bend_local
        wrist_tip = wrist_base + self.L_w * wrist_z

        x1 = R0 @ np.array([1.0, 0.0, 0.0])
        x1 = x1 - np.dot(x1, wrist_z) * wrist_z
        x1_norm = np.linalg.norm(x1)
        if x1_norm < self.eps:
            x1 = np.array([1.0, 0.0, 0.0])
            x1 = x1 - np.dot(x1, wrist_z) * wrist_z
            x1_norm = np.linalg.norm(x1)
        x1 = x1 / x1_norm
        y1 = np.cross(wrist_z, x1)
        R_wrist = np.column_stack((x1, y1, wrist_z))

        R_gripper = R.from_euler('xyz', [roll, pitch, yaw]).as_matrix()
        R_total = R_wrist @ R_gripper

        ee_pos = wrist_tip
        quat = R.from_matrix(R_total).as_quat()
        return np.hstack([ee_pos, quat])
    
    def pose_error(self, q, target_pos, target_rot):
        try:
            pose = self.fk(q)
            curr_pos = pose[:3]
            curr_rot = R.from_quat(pose[3:]).as_matrix()
        except Exception:
            return np.array([1e6] * 6)

        pos_error = target_pos - curr_pos
        rot_error = R.from_matrix(target_rot @ curr_rot.T)
        ori_error = rot_error.as_rotvec()
        
        angle = np.linalg.norm(ori_error)
        if angle > 1e-6:
            axis = ori_error / angle
            wrapped_angle = (angle + np.pi) % (2*np.pi) - np.pi
            ori_error = wrapped_angle * axis
        
        return np.hstack([pos_error, ori_error])
    
    def cost(self, q, target_pos, target_rot, pos_weight=1.0, ori_weight=1.0):
        error = self.pose_error(q, target_pos, target_rot)
        return pos_weight * np.sum(error[:3]**2) + ori_weight * np.sum(error[3:]**2)
    
    def jacobian(self, q, eps=1e-8):
        def pose_error_rel(qq, q_ref):
            pose_qq = self.fk(qq)
            pose_ref = self.fk(q_ref)
            pos_qq, pos_ref = pose_qq[:3], pose_ref[:3]
            rot_qq = R.from_quat(pose_qq[3:]).as_matrix()
            rot_ref = R.from_quat(pose_ref[3:]).as_matrix()
            pos_err = pos_qq - pos_ref
            rot_err = R.from_matrix(rot_qq @ rot_ref.T)
            ori_err = rot_err.as_rotvec()
            return np.hstack([pos_err, ori_err])
        
        J = np.zeros((6, 8))
        for i in range(8):
            q_pert = q.copy()
            q_pert[i] += eps
            error = pose_error_rel(q_pert, q)
            J[:, i] = error / eps
        return J
    
    def angle_ineq(self, q):
        h1, h2 = q[0], q[1]
        alpha_rad = np.arctan2(h2 - h1, self.d2)
        alpha_deg = np.abs(np.degrees(alpha_rad))
        return alpha_deg - self.alpha_min_deg
    
    def ik(self, target_pos, target_rot, q0,
           pos_weight=1.0, ori_weight=0.3,
           max_iter=50, tol=1e-6,
           joint_bounds=None):

        target_pos = np.asarray(target_pos, dtype=float)
        target_rot = np.asarray(target_rot, dtype=float)
        q0 = np.asarray(q0, dtype=float)

        if joint_bounds is not None:
            lower, upper = joint_bounds
        else:
            lower, upper = self.get_joint_bounds()
        
        bounds = Bounds(lb=lower, ub=upper)
        cons = {'type': 'ineq', 'fun': self.angle_ineq}
        
        result = minimize(
            self.cost,
            q0,
            args=(target_pos, target_rot, pos_weight, ori_weight),
            method='SLSQP',
            bounds=bounds,
            constraints=cons,
            options={'maxiter': max_iter, 'ftol': tol}
        )

        if result.success:
            return result.x

    def validate_configuration(self, q):
        lower, upper = self.get_joint_bounds()
        q = np.asarray(q)
        return np.all(q >= lower) and np.all(q <= upper)