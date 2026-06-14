import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.optimize import minimize
from numba import njit
from typing import Optional

import numpy as np
from scipy.optimize import minimize
from numba import njit, float64
from typing import Optional

@njit
def compute_ee_pitch(h1, h2, a1, theta, phi, d2, l3_max=0.625, wrist_length=0.035, eps=1e-12):
    p1x, p1y, p1z = 0.0, 0.0, h1
    p2x = -d2 * np.cos(theta)
    p2y = -d2 * np.sin(theta)
    p2z = h2
    
    vx = p1x - p2x
    vy = p1y - p2y
    vz = p1z - p2z
    dist = np.sqrt(vx*vx + vy*vy + vz*vz)
    if dist < eps:
        dist = eps
    
    zx = vx / dist
    zy = vy / dist
    zz = vz / dist
    
    if np.abs(zz) < 0.99:
        rx, ry, rz = 0.0, 0.0, 1.0
    else:
        rx, ry, rz = 1.0, 0.0, 0.0
    
    proj = rx*zx + ry*zy + rz*zz
    xl_x = rx - proj*zx
    xl_y = ry - proj*zy
    xl_z = rz - proj*zz
    
    n = np.sqrt(xl_x*xl_x + xl_y*xl_y + xl_z*xl_z)
    if n < eps:
        rx, ry, rz = 1.0, 0.0, 0.0
        proj = rx*zx + ry*zy + rz*zz
        xl_x = rx - proj*zx
        xl_y = ry - proj*zy
        xl_z = rz - proj*zz
        n = np.sqrt(xl_x*xl_x + xl_y*xl_y + xl_z*xl_z)
        if n < eps:
            n = 1.0
    
    xl_x /= n
    xl_y /= n
    xl_z /= n
    
    sb = -np.sin(phi)  
    cb = np.cos(phi)
    wz_x = xl_x * sb + zx * cb
    wz_y = xl_y * sb + zy * cb
    wz_z = xl_z * sb + zz * cb
    
    horizontal_mag = np.sqrt(wz_x*wz_x + wz_y*wz_y)
    pitch = np.arctan2(wz_z, horizontal_mag)
    
    return pitch

@njit
def _fk_kernel(h1, h2, a1, theta, phi, d2, l3_max, wrist_length=0.035, eps=1e-12, intermediate=False):
    p1x, p1y, p1z = 0.0, 0.0, h1
    p2x = -d2 * np.cos(theta)   
    p2y = -d2 * np.sin(theta) 
    p2z = h2

    # Vector from p2 to p1
    vx = p1x - p2x
    vy = p1y - p2y
    vz = p1z - p2z
    dist = np.sqrt(vx*vx + vy*vy + vz*vz)
    if dist < eps:
        dist = eps

    # Unit vector along column (from p2 to p1)
    ux = vx / dist
    uy = vy / dist
    uz = vz / dist

    # Wrist base = end of extended part
    wb_x = p1x + a1 * ux
    wb_y = p1y + a1 * uy
    wb_z = p1z + a1 * uz
    wrist = np.array([wb_x, wb_y, wb_z])
    
    if intermediate :
        inner_x = p1x - (l3_max - a1) * ux
        inner_y = p1y - (l3_max - a1) * uy
        inner_z = p1z - (l3_max - a1) * uz
        p_inner = np.array([inner_x, inner_y, inner_z])
        return p_inner
    
    # Local frame construction
    zx, zy, zz = ux, uy, uz  # Z = column direction

    if np.abs(zz) < 0.99:
        rx, ry, rz = 0.0, 0.0, 1.0
    else:
        rx, ry, rz = 1.0, 0.0, 0.0

    # X = ref - proj(ref, Z)
    proj = rx*zx + ry*zy + rz*zz
    xl_x = rx - proj*zx
    xl_y = ry - proj*zy
    xl_z = rz - proj*zz

    n = np.sqrt(xl_x*xl_x + xl_y*xl_y + xl_z*xl_z)
    if n < eps:
        rx, ry, rz = 1.0, 0.0, 0.0

        proj = rx*zx + ry*zy + rz*zz
        xl_x = rx - proj*zx
        xl_y = ry - proj*zy
        xl_z = rz - proj*zz
        n = np.sqrt(xl_x*xl_x + xl_y*xl_y + xl_z*xl_z)
        if n < eps:
            n = 1.0

    xl_x /= n
    xl_y /= n
    xl_z /= n

    yl_x = zy*xl_z - zz*xl_y
    yl_y = zz*xl_x - zx*xl_z
    yl_z = zx*xl_y - zy*xl_x

    # Wrist bend
    sb = -np.sin(phi)
    cb = np.cos(phi)
    wz_x = xl_x * sb + zx * cb
    wz_y = xl_y * sb + zy * cb
    wz_z = xl_z * sb + zz * cb

    ee_x = wb_x + wrist_length * wz_x
    ee_y = wb_y + wrist_length * wz_y
    ee_z = wb_z + wrist_length * wz_z

    ee = np.array([ee_x, ee_y, ee_z])
    return ee

class MorphIManipulator:
    DEFAULT_PARAMS = {
        "d2": 0.1,
        "l3_max": 0.625,
        "wrist_length": 0.111 + 0.035 + 0.13,         
        "alpha_min_deg": 20.0,
        "bounds_h": (0.0, 1.43),
        "bounds_a": (0.0, 0.625),
        "bounds_theta_deg": (-15.0, 45.0),
        "bounds_phi_deg": (-180, 180), 
        "min_lateral_dist": 0.35,
        "arm_base_height" : 0.0 
    }

    def __init__(self, name: str = "arm", **kwargs):
        params = self.DEFAULT_PARAMS.copy()
        params.update(kwargs)

        self.name = name
        self.d2 = float(params["d2"])
        self.l3_max = float(params["l3_max"])
        self.wrist_length = float(params["wrist_length"]) 
        self.alpha_min_deg = float(params["alpha_min_deg"])
        self.alpha_min_rad = np.radians(self.alpha_min_deg)
        self.bounds_h = (float(params["bounds_h"][0]), float(params["bounds_h"][1]))
        self.bounds_a = (float(params["bounds_a"][0]), float(params["bounds_a"][1]))
        theta_min_deg = float(params["bounds_theta_deg"][0])
        theta_max_deg = float(params["bounds_theta_deg"][1])
        self.bounds_theta = (np.radians(theta_min_deg), np.radians(theta_max_deg))
        self.arm_base_height = float(params["arm_base_height"])
        
        phi_min_deg = float(params["bounds_phi_deg"][0])
        phi_max_deg = float(params["bounds_phi_deg"][1])
        self.bounds_phi = (np.radians(phi_min_deg), np.radians(phi_max_deg)) 
        
        self.min_lateral_dist = float(params["min_lateral_dist"])
        self.pitch_weight = 50.0

        _fk_kernel(0.5, 0.5, 0.3, 0.0, 0.0, self.d2, self.l3_max, self.wrist_length)

        self._ik_cache_target = None
        self._ik_cache_result = None
        self._cache_threshold = 0.001

    def fk(self, q: np.ndarray) -> np.ndarray:
        h1, h2, a1, theta, phi = q  
        return _fk_kernel(h1, h2, a1, theta, phi, self.d2, self.l3_max, self.wrist_length)

    def get_p_inner(self, q: np.ndarray) -> np.ndarray :
        h1, h2, a1, theta, phi = q  
        p_inner = _fk_kernel(h1, h2, a1, theta, phi, self.d2, self.l3_max, self.wrist_length, intermediate=True)
        return p_inner

    def ik(self, 
        target: np.ndarray, 
        q_guess: np.ndarray, 
        pitch_target: Optional[float] = None) -> np.ndarray:
        
        target = np.asarray(target, dtype=np.float64)
        if target.shape != (3,):
            raise ValueError("target must be shape (3,)")

        # Cache check: recompute if target OR pitch_target changed
        should_use_cache = (
            self._ik_cache_target is not None and
            np.linalg.norm(target - self._ik_cache_target) <= self._cache_threshold and
            (  # Pitch target matches (both None OR both close numerically)
                (pitch_target is None and self._ik_cache_pitch_target is None) or
                (pitch_target is not None and 
                self._ik_cache_pitch_target is not None and
                abs(pitch_target - self._ik_cache_pitch_target) <= self._cache_threshold)
            )
        )
        
        if should_use_cache:
            return self._ik_cache_result.copy()

        def cost(vars):
            h1, h2, a1, theta, phi = vars
            ee = _fk_kernel(h1, h2, a1, theta, phi, self.d2, self.l3_max, self.wrist_length)
            pos_err = np.sum((ee - target) ** 2)

            # Fix lateral distance constraint
            lateral_dist = np.sqrt(ee[0]**2 + ee[1]**2)
            if lateral_dist < self.min_lateral_dist:
                pos_err += 10000.0 * (self.min_lateral_dist - lateral_dist) ** 2

            if pitch_target is not None:
                p1x, p1y, p1z = 0.0, 0.0, h1
                p2x = -self.d2 * np.cos(theta)
                p2y = -self.d2 * np.sin(theta)
                p2z = h2
                
                vx = p1x - p2x
                vy = p1y - p2y
                vz = p1z - p2z
                dist = np.sqrt(vx*vx + vy*vy + vz*vz + 1e-12)
                ux, uy, uz = vx/dist, vy/dist, vz/dist  
                
                if np.abs(uz) < 0.99:
                    rx, ry, rz = 0.0, 0.0, 1.0
                else:
                    rx, ry, rz = 1.0, 0.0, 0.0
                proj = rx*ux + ry*uy + rz*uz
                xl_x = rx - proj*ux
                xl_y = ry - proj*uy
                xl_z = rz - proj*uz
                n = np.sqrt(xl_x*xl_x + xl_y*xl_y + xl_z*xl_z + 1e-12)
                xl_x, xl_y, xl_z = xl_x/n, xl_y/n, xl_z/n
                
                sb = -np.sin(phi) 
                cb = np.cos(phi)
                wz_x = xl_x * sb + ux * cb
                wz_y = xl_y * sb + uy * cb
                wz_z = xl_z * sb + uz * cb
                
                horizontal_norm = np.sqrt(wz_x*wz_x + wz_y*wz_y + 1e-12)
                actual_pitch = np.arctan2(-wz_z, horizontal_norm)  
                
                pitch_err = (actual_pitch - pitch_target) ** 2
                pos_err += self.pitch_weight * pitch_err

            return pos_err

        def p_inner_above_base(vars):
            p_inner = self.get_p_inner(vars)
            return p_inner[2] - self.arm_base_height

        def min_angle_constraint(vars):
            h1, h2, a1, theta, phi = vars
            alpha = np.arctan2(np.abs(h1 - h2), self.d2)
            return alpha - self.alpha_min_rad

        constraints = [
            {"type": "ineq", "fun": min_angle_constraint},
            {"type": "ineq", "fun": p_inner_above_base}
        ]
        
        bounds = [
            self.bounds_h,      # h1
            self.bounds_h,      # h2
            self.bounds_a,      # a1
            self.bounds_theta,  # theta
            self.bounds_phi     # phi 
        ]

        res = minimize(
            cost,
            np.asarray(q_guess, dtype=np.float64),
            method="SLSQP",
            bounds=bounds,
            constraints=constraints,
            options={"ftol": 1e-6, "maxiter": 40, "disp": False}
        )

        if res.success:
            q_sol = res.x.astype(float)
        else:
            if self._ik_cache_result is not None:
                q_sol = self._ik_cache_result.copy()
            else:
                q_sol = np.asarray(q_guess, dtype=np.float64)

        # Update cache with BOTH target and pitch
        self._ik_cache_target = target.copy()
        self._ik_cache_pitch_target = pitch_target
        self._ik_cache_result = q_sol.copy()
        return q_sol
    
    def get_fk_boxes(self, q: np.ndarray, column_radius_xy: float = 0.02, wrist_radius_xy: float = 0.015):
        """
        Compute two oriented boxes for visualization:
        - Column: from p_inner to wrist_base
        - Wrist:  from wrist_base to ee

        Args:
            q: joint vector [h1, h2, a1, theta, phi]
            column_radius_xy: half-size (radius) in X/Y for column cross-section
            wrist_radius_xy: half-size (radius) in X/Y for wrist cross-section

        Returns:
            dict with:
            - 'column': { 'center', 'direction', 'length', 'radius' }
            - 'wrist':  { 'center', 'direction', 'length', 'radius' }
        """
        h1, h2, a1, theta, phi = q

        # Reuse FK logic to get all points
        p1x, p1y, p1z = 0.0, 0.0, h1
        p2x = -self.d2 * np.cos(theta)
        p2y = -self.d2 * np.sin(theta)
        p2z = h2

        vx = p1x - p2x
        vy = p1y - p2y
        vz = p1z - p2z
        dist = np.sqrt(vx*vx + vy*vy + vz*vz)
        eps = 1e-12
        if dist < eps:
            dist = eps

        ux = vx / dist
        uy = vy / dist
        uz = vz / dist

        # Key points
        wrist_base = np.array([p1x + a1 * ux, p1y + a1 * uy, p1z + a1 * uz])
        p_inner = np.array([p1x - (self.l3_max - a1) * ux, p1y - (self.l3_max - a1) * uy, p1z - (self.l3_max - a1) * uz])
        ee = _fk_kernel(h1, h2, a1, theta, phi, self.d2, self.l3_max, self.wrist_length)

        # Column vector
        col_vec = wrist_base - p_inner
        col_len = np.linalg.norm(col_vec)
        col_dir = col_vec / col_len if col_len > eps else np.array([0, 0, 1])
        col_center = (p_inner + wrist_base) / 2.0

        # Wrist vector
        wrist_vec = ee - wrist_base
        wrist_len = np.linalg.norm(wrist_vec)
        wrist_dir = wrist_vec / wrist_len if wrist_len > eps else col_dir
        wrist_center = (wrist_base + ee) / 2.0

        return {
            "column": {
                "center": col_center,
                "direction": col_dir,
                "length": col_len,
                "radius": column_radius_xy
            },
            "wrist": {
                "center": wrist_center,
                "direction": wrist_dir,
                "length": wrist_len,
                "radius": wrist_radius_xy
            }
        }

    def fk_all_points(self, q: np.ndarray):
        """
        Compute p_inner, wrist_base, and ee in arm's local frame (arm mounted at origin).
        
        Returns:
            dict with keys: 'p_inner', 'wrist_base', 'ee' (each shape (3,))
        """
        h1, h2, a1, theta, phi = q

        p1x, p1y, p1z = 0.0, 0.0, h1
        p2x = -self.d2 * np.cos(theta)
        p2y = -self.d2 * np.sin(theta)
        p2z = h2

        vx = p1x - p2x
        vy = p1y - p2y
        vz = p1z - p2z
        dist = np.sqrt(vx*vx + vy*vy + vz*vz)
        eps = 1e-12
        if dist < eps:
            dist = eps

        ux = vx / dist
        uy = vy / dist
        uz = vz / dist

        # Wrist base
        wb = np.array([p1x + a1 * ux, p1y + a1 * uy, p1z + a1 * uz])

        # p_inner
        inner = np.array([
            p1x - (self.l3_max - a1) * ux,
            p1y - (self.l3_max - a1) * uy,
            p1z - (self.l3_max - a1) * uz
        ])

        # End-effector
        ee = _fk_kernel(h1, h2, a1, theta, phi, self.d2, self.l3_max, self.wrist_length)

        return {
            "p_inner": inner,
            "wrist_base": wb,
            "ee": ee
        }

    def set_cache_threshold(self, thresh: float):
        self._cache_threshold = float(thresh)

    def clear_cache(self):
        self._ik_cache_target = None
        self._ik_cache_result = None

    def __repr__(self):
        return (f"MorphIManipulator(name='{self.name}', d2={self.d2}, "
                f"l3_max={self.l3_max}, α_min={self.alpha_min_deg}°, "
                f"θ∈[{np.degrees(self.bounds_theta[0]):.1f}°, {np.degrees(self.bounds_theta[1]):.1f}°])")

import plotly.graph_objects as go

def plot_morph_arm(manip: MorphIManipulator, q: np.ndarray, target: Optional[np.ndarray] = None):
    """
    Plot the MorphIManipulator configuration using its own parameters.
    
    Args:
        manip: MorphIManipulator instance
        q: joint vector [h1, h2, a1, theta, phi]
        target: optional target position (3,)
    """
    h1, h2, a1, theta, phi = q
    d2 = manip.d2
    l3_max = manip.l3_max
    wrist_length = manip.wrist_length
    base_height = manip.arm_base_height

    # Compute points
    p1 = np.array([0.0, 0.0, h1])
    p2 = np.array([-d2 * np.cos(theta), -d2 * np.sin(theta), h2])  # note sign matches FK
    ee = manip.fk(q)
    p_inner  = manip.get_p_inner(q)

    # Unit vector from p2 to p1
    vec = p1 - p2
    u = vec / np.linalg.norm(vec)
    ee_base = p1 + a1 * u

    # Setup figure
    fig = go.Figure()

    # Guides
    z_max = max(h1, h2, ee[2], p_inner[2], base_height) + 0.2
    z_cyl = np.linspace(base_height, z_max, 30)

    # p2 guide
    cyl_x = -d2 * np.cos(theta) * np.ones_like(z_cyl)
    cyl_y = -d2 * np.sin(theta) * np.ones_like(z_cyl)
    fig.add_trace(go.Scatter3d(
        x=cyl_x, y=cyl_y, z=z_cyl,
        mode='lines',
        line=dict(color='green', width=2, dash='dash'),
        name='p2 guide'
    ))

    # p1 guide (z-axis)
    fig.add_trace(go.Scatter3d(
        x=np.zeros_like(z_cyl),
        y=np.zeros_like(z_cyl),
        z=z_cyl,
        mode='lines',
        line=dict(color='blue', width=2, dash='dash'),
        name='p1 guide'
    ))

    # Ground plane at base_height (optional but helpful)
    plane_size = 0.8
    xx, yy = np.meshgrid(
        np.linspace(-plane_size, plane_size, 10),
        np.linspace(-plane_size, plane_size, 10)
    )
    zz = np.full_like(xx, base_height)
    fig.add_trace(go.Surface(
        x=xx, y=yy, z=zz,
        colorscale=[[0, 'lightgray'], [1, 'lightgray']],
        opacity=0.2,
        showscale=False,
        name='ground'
    ))

    # Points
    fig.add_trace(go.Scatter3d(x=[p1[0]], y=[p1[1]], z=[p1[2]],
                               mode='markers+text', text=['p1'],
                               textposition='top center',
                               marker=dict(size=6, color='blue')))
    fig.add_trace(go.Scatter3d(x=[p2[0]], y=[p2[1]], z=[p2[2]],
                               mode='markers+text', text=['p2'],
                               textposition='top center',
                               marker=dict(size=6, color='green')))
    fig.add_trace(go.Scatter3d(x=[ee_base[0]], y=[ee_base[1]], z=[ee_base[2]],
                               mode='markers', marker=dict(size=5, color='orange'),
                               name='wrist base'))
    fig.add_trace(go.Scatter3d(x=[ee[0]], y=[ee[1]], z=[ee[2]],
                               mode='markers+text', text=['EE'],
                               textposition='top center',
                               marker=dict(size=8, color='red', symbol='diamond')))
    fig.add_trace(go.Scatter3d(x=[p_inner[0]], y=[p_inner[1]], z=[p_inner[2]],
                               mode='markers+text', text=['inner'],
                               textposition='bottom center',
                               marker=dict(size=6, color='cyan'),
                               name='inner end'))

    # Links
    fig.add_trace(go.Scatter3d(x=[p1[0], ee_base[0]], y=[p1[1], ee_base[1]], z=[p1[2], ee_base[2]],
                               mode='lines', line=dict(color='red', width=4), name='extended'))
    fig.add_trace(go.Scatter3d(x=[ee_base[0], ee[0]], y=[ee_base[1], ee[1]], z=[ee_base[2], ee[2]],
                               mode='lines', line=dict(color='magenta', width=5), name='wrist'))
    fig.add_trace(go.Scatter3d(x=[p_inner[0], ee_base[0]], y=[p_inner[1], ee_base[1]], z=[p_inner[2], ee_base[2]],
                               mode='lines', line=dict(color='red', width=2, dash='dot'),
                               name=f'full rod (l3_max={l3_max:.3f})'))

    # Max reach sphere
    u_ang = np.linspace(0, 2*np.pi, 20)
    v_ang = np.linspace(0, np.pi, 10)
    xs, ys, zs = [], [], []
    for uu in u_ang:
        for vv in v_ang:
            xs.append(p1[0] + l3_max * np.cos(uu) * np.sin(vv))
            ys.append(p1[1] + l3_max * np.sin(uu) * np.sin(vv))
            zs.append(p1[2] + l3_max * np.cos(vv))
    fig.add_trace(go.Scatter3d(x=xs, y=ys, z=zs, mode='markers',
                               marker=dict(size=0.5, color='lightgray'),
                               name='max reach'))

    # Target
    if target is not None:
        fig.add_trace(go.Scatter3d(x=[target[0]], y=[target[1]], z=[target[2]],
                                   mode='markers+text', text=['target'],
                                   textposition='bottom center',
                                   marker=dict(size=8, color='purple', symbol='x')))

    fig.update_layout(
        scene=dict(
            xaxis_title='X (m)',
            yaxis_title='Y (m)',
            zaxis_title='Z (m)',
            aspectmode='cube'
        ),
        title=f"{manip.name}: θ={theta:.3f} rad, φ={phi:.3f} rad | p_inner.z={p_inner[2]:.3f} ≥ {base_height}",
        width=900, height=700
    )
    fig.show()
    
if __name__ == "__main__" :
    arm = MorphIManipulator(name="left_arm")

    # Solve IK
    target = np.array([0.4, 0.2, 0.8])
    q_guess = np.array([0.8, 0.6, 0.3, 0.2, 0.0])
    q_sol = arm.ik(target, q_guess, pitch_target=np.pi)
    print(q_sol)

    # Plot
    plot_morph_arm(arm, q_sol, target=target)