import numpy as np

def mecanum_ik_matrix(r: float, Lx: float, Ly: float) -> np.ndarray:
    H = (r / 4.0) * np.array([
        [ 1,  1,  1,  1],
        [-1,  1,  1, -1],
        [-1/(Lx+Ly), 1/(Lx+Ly), -1/(Lx+Ly), 1/(Lx+Ly)]
    ])
    return H

def mecanum_fk(wheel_vels: np.ndarray, r: float, Lx: float, Ly: float) -> np.ndarray:
    H = mecanum_ik_matrix(r, Lx, Ly)
    return H @ wheel_vels

def mecanum_ik(body_twist: np.ndarray, r: float, Lx: float, Ly: float) -> np.ndarray:
    H = mecanum_ik_matrix(r, Lx, Ly)
    try:
        H_inv = np.linalg.pinv(H) 
        return H_inv @ body_twist
    except np.linalg.LinAlgError:
        raise ValueError("Singular kinematics matrix")

def integrate_odometry(
    pose: np.ndarray,
    body_twist: np.ndarray,
    dt: float,
    method: str = "first-order"
) -> np.ndarray:
    x, y, theta = pose
    vx_b, vy_b, wz = body_twist

    if method == "first-order":
        cos_t = np.cos(theta)
        sin_t = np.sin(theta)
        vx_w = cos_t * vx_b - sin_t * vy_b
        vy_w = sin_t * vx_b + cos_t * vy_b

        x += vx_w * dt
        y += vy_w * dt
        theta += wz * dt

    elif method == "midpoint":
        theta_mid = theta + 0.5 * wz * dt
        cos_t = np.cos(theta_mid)
        sin_t = np.sin(theta_mid)
        vx_w = cos_t * vx_b - sin_t * vy_b
        vy_w = sin_t * vx_b + cos_t * vy_b

        x += vx_w * dt
        y += vy_w * dt
        theta += wz * dt

    theta = (theta + np.pi) % (2 * np.pi) - np.pi
    return np.array([x, y, theta])

def mobile_base_ik(vx, vy, wz, r, L):
    return np.array([
        ( vx - vy - L * wz) / r,  # FL
        ( vx + vy + L * wz) / r,  # FR
        ( vx + vy - L * wz) / r,  # RL
        ( vx - vy + L * wz) / r   # RR
    ])

def mobile_base_fk(wheels, r, L):
    """
    wheels = [w_FL, w_FR, w_RL, w_RR]
    returns [vx, vy, wz]
    """
    w_fl, w_fr, w_rl, w_rr = wheels

    vx = (r / 4) * (w_fl + w_fr + w_rl + w_rr)
    vy = (r / 4) * (-w_fl + w_fr + w_rl - w_rr)
    wz = (r / (4 * L)) * (-w_fl + w_fr - w_rl + w_rr)

    return np.array([vx, vy, wz])