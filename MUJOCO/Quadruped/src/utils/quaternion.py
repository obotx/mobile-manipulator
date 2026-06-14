import numpy as np

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

def quat2yaw(q):
    w, x, y, z = q
    return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))