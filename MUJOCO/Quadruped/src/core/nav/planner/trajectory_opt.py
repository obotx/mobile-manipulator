import numpy as np
from scipy.special import perm
from scipy.interpolate import BSpline
import matplotlib.pyplot as plt
from cvxopt import matrix, solvers

solvers.options['show_progress'] = False

class PolynomialOptimizer:
    def __init__(self, n_coeffs: list, derivatives: list, times: list, robot_radius: float = 0.0):
        self.n_coeffs = n_coeffs  
        self.derivatives = derivatives 
        self.times = np.array(times)
        self.T = np.ediff1d(times)
        self.robot_radius = robot_radius

    @staticmethod
    def poly_coeff(n, d, t):
        D = n - 1 - np.arange(n)
        j = np.arange(d)[:, None]
        factors = np.where(D - j >= 0, D - j, 0)
        prod = np.prod(factors, axis=0)
        exponents = np.maximum(D - d, 0)
        cc = prod * t**exponents
        return cc[::-1].astype(float)

    def q_block(self):
        """Builds the Hessian Q matrix for N-dimensions."""
        n_segs = len(self.T)
        total_size = sum(self.n_coeffs) * n_segs
        Q_block = np.zeros((total_size, total_size))
        
        cum_idx = 0
        for i, (order, d) in enumerate(zip(self.n_coeffs, self.derivatives)):
            dim_size = order * n_segs
            Qi = np.zeros((dim_size, dim_size))
            for seg in range(n_segs):
                s_idx = seg * order
                for l in range(order):
                    for k in range(order):
                        if l >= d and k >= d:
                            pow_term = l + k - 2 * d + 1
                            val = 2 * (perm(l, d) * perm(k, d)) * (self.T[seg] ** pow_term / pow_term)
                            Qi[s_idx + l, s_idx + k] = val
            
            Q_block[cum_idx:cum_idx + dim_size, cum_idx:cum_idx + dim_size] = Qi
            cum_idx += dim_size
        return Q_block

    def get_constraints(self, waypoint, corridors=None):
        n_T = len(self.T)
        n_dims = len(self.n_coeffs)
        n_coeffs_total = sum(self.n_coeffs) * n_T
        
        # 1. Equality Constraints (A, b)
        n_eq = sum((n_T * 2 + (n_T - 1) * d) for d in self.derivatives)
        A = np.zeros((n_eq, n_coeffs_total))
        b_eq = np.zeros(n_eq)

        # Start/End Positions for each segment across all dimensions
        row_offset = 0
        col_offset = 0
        for i in range(n_dims):
            n = self.n_coeffs[i]
            for j in range(n_T):
                A[row_offset, col_offset + j*n : col_offset + (j+1)*n] = self.poly_coeff(n, 0, 0)
                b_eq[row_offset] = waypoint[j, i]
                row_offset += 1
                A[row_offset, col_offset + j*n : col_offset + (j+1)*n] = self.poly_coeff(n, 0, self.T[j])
                b_eq[row_offset] = waypoint[j+1, i]
                row_offset += 1
            
            for d in range(1, self.derivatives[i] + 1):
                for j in range(n_T - 1):
                    A[row_offset, col_offset + j*n : col_offset + (j+1)*n] = self.poly_coeff(n, d, self.T[j])
                    A[row_offset, col_offset + (j+1)*n : col_offset + (j+2)*n] = -self.poly_coeff(n, d, 0)
                    row_offset += 1
            col_offset += n * n_T

        G_list, h_list = [], []
        if corridors:
            samples = np.linspace(0.1, 0.9, 3) # Sample middle of segments
            col_offset = 0
            for i in range(n_dims):
                n = self.n_coeffs[i]
                dim_key = f'dim{i}'
                for seg_idx in range(n_T):
                    if dim_key in corridors[seg_idx]:
                        b_min = corridors[seg_idx][dim_key][0] + self.robot_radius
                        b_max = corridors[seg_idx][dim_key][1] - self.robot_radius
                        
                        for t_f in samples:
                            t_val = t_f * self.T[seg_idx]
                            coeffs = self.poly_coeff(n, 0, t_val)
                            row = np.zeros(n_coeffs_total)
                            row[col_offset + seg_idx*n : col_offset + (seg_idx+1)*n] = coeffs
                            
                            G_list.append(row); h_list.append(float(b_max))
                            G_list.append(-row); h_list.append(-float(b_min))
                col_offset += n * n_T

        G_mat = matrix(np.array(G_list), tc='d') if G_list else None
        h_mat = matrix(np.array(h_list).reshape(-1, 1), tc='d') if h_list else None
        
        return matrix(A, tc='d'), matrix(b_eq, tc='d'), G_mat, h_mat

    def solve(self, waypoint, corridors=None):
        Q = matrix(self.q_block(), tc='d')
        f = matrix(np.zeros(Q.size[0]), tc='d')
        A, b, G, h = self.get_constraints(waypoint, corridors)
        sol = solvers.qp(Q, f, G, h, A, b)
        return np.array(sol['x']).flatten()

    def get_states(self, coeff, num_points=500):
        t_all = np.linspace(self.times[0], self.times[-1], num_points)
        all_states = []
        n_T = len(self.T)
        col_offset = 0
        for i in range(len(self.n_coeffs)):
            n = self.n_coeffs[i]
            d_order = self.derivatives[i]
            d_states = np.zeros((d_order + 1, num_points))
            for step, t in enumerate(t_all):
                seg = max(0, min(np.searchsorted(self.times, t) - 1, n_T - 1))
                ti = t - self.times[seg]
                c = coeff[col_offset + seg*n : col_offset + (seg+1)*n]
                c_flipped = np.flip(c)
                
                curr_c = c_flipped
                for d in range(d_order + 1):
                    d_states[d, step] = np.polyval(curr_c, ti)
                    curr_c = np.polyder(curr_c)
            all_states.append(d_states)
            col_offset += n * n_T
        return t_all, all_states
   
class BsplineOptimizer:
    def __init__(self, degree=3, robot_radius=0.0):
        self.k = degree
        self.robot_radius = robot_radius

    def _get_basis_matrix(self, t_samples, knots):
        n_ctrl = len(knots) - self.k - 1
        A = np.zeros((len(t_samples), n_ctrl))
        for i in range(n_ctrl):
            coeffs = np.zeros(n_ctrl)
            coeffs[i] = 1.0
            spl = BSpline(knots, coeffs, self.k)
            A[:, i] = spl(t_samples)
        return A

    def solve(self, waypoints, corridors):
        n_pts, n_dims = waypoints.shape
        num_segments = n_pts - 1
        
        knots = np.concatenate(([0]*self.k, np.arange(num_segments + 1), [num_segments]*self.k))
        n_ctrl = len(knots) - self.k - 1
        n_vars = n_dims * n_ctrl
        
        P = np.eye(n_vars) * 0.05
        for d in range(n_dims):
            off = d * n_ctrl
            for i in range(n_ctrl - 1):
                idx = off + i
                P[idx, idx] += 1.0
                P[idx+1, idx+1] += 1.0
                P[idx, idx+1] -= 1.0
                P[idx+1, idx] -= 1.0
        
        q = np.zeros(n_vars)

        A_eq, b_eq = [], []
        for i in range(n_pts):
            basis_row = self._get_basis_matrix([i], knots)[0]
            for d in range(n_dims):
                row = np.zeros(n_vars)
                row[d * n_ctrl : (d + 1) * n_ctrl] = basis_row
                A_eq.append(row)
                b_eq.append(waypoints[i, d])

        G_ineq, h_ineq = [], []
        if corridors:
            for j in range(n_ctrl):
                seg_idx = min(int(np.floor(knots[j + 1])), num_segments - 1)
                box = corridors[seg_idx]
                
                for d in range(n_dims):
                    dim_key = f'dim{d}'
                    if dim_key in box:
                        low_lim = box[dim_key][0] + self.robot_radius
                        high_lim = box[dim_key][1] - self.robot_radius
                        
                        var_idx = d * n_ctrl + j
                        
                        row_up = np.zeros(n_vars)
                        row_up[var_idx] = 1.0
                        G_ineq.append(row_up); h_ineq.append(float(high_lim))
                        
                        row_low = np.zeros(n_vars)
                        row_low[var_idx] = -1.0
                        G_ineq.append(row_low); h_ineq.append(-float(low_lim))

        # Solve QP
        sol = solvers.qp(matrix(P), matrix(q), 
                         matrix(np.array(G_ineq)) if G_ineq else None, 
                         matrix(np.array(h_ineq).reshape(-1,1)) if h_ineq else None, 
                         matrix(np.array(A_eq)), matrix(np.array(b_eq)))
        
        ctrl_pts = np.array(sol['x']).flatten().reshape(n_dims, n_ctrl)
        return knots, ctrl_pts

    def get_path(self, knots, ctrl_pts, n_samples=200):
        t_range = np.linspace(0, knots[-1] - 0.001, n_samples)
        n_dims = ctrl_pts.shape[0]
        path = np.zeros((n_samples, n_dims))
        for d in range(n_dims):
            path[:, d] = BSpline(knots, ctrl_pts[d], self.k)(t_range)
        return path
    