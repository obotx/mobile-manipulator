import numpy as np

class StaticTransform:
    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0, 
                 roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0, 
                 use_degrees: bool = False):
        self.t = np.array([x, y, z], dtype=np.float64)
        if use_degrees: roll, pitch, yaw = np.deg2rad([roll, pitch, yaw])
        cx, sx = np.cos(roll), np.sin(roll)
        cy, sy = np.cos(pitch), np.sin(pitch)
        cz, sz = np.cos(yaw), np.sin(yaw)
        Rx = np.array([[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]])
        Ry = np.array([[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]])
        Rz = np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]])
        self.R = Rz @ Ry @ Rx

    def transform_point(self, point: np.ndarray) -> np.ndarray:
        return (self.R @ np.asarray(point, dtype=np.float64)) + self.t

    def transform_points(self, points: np.ndarray) -> np.ndarray:
        return (np.asarray(points, dtype=np.float64) @ self.R.T) + self.t

def quaternion_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = q1; w2, x2, y2, z2 = q2
    return np.array([w1*w2 - x1*x2 - y1*y2 - z1*z2, w1*x2 + x1*w2 + y1*z2 - z1*y2,
                     w1*y2 - x1*z2 + y1*w2 + z1*x2, w1*z2 + x1*y2 - y1*x2 + z1*w2])

def quaternion_conjugate(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]])