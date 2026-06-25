"""
Polynomial trajectory optimization using quadratic programming.
Generates smooth trajectories through waypoints with continuity constraints.
"""
import numpy as np
from scipy.special import perm
from cvxopt import matrix, solvers
from typing import List, Tuple


class TrajectoryOptimizer:
    """
    Optimizes polynomial trajectories through waypoints using QP.
    
    Minimizes the integral of squared derivatives (smoothness) subject to:
    - Waypoint position constraints
    - Continuity constraints on derivatives across segments
    """

    def __init__(self, n_coeffs: List[int], derivatives: List[int], times: List[float]):
        """
        Args:
            n_coeffs: Polynomial order per DOF (e.g., [5, 5, 5] for x, y, z)
            derivatives: Continuity order per DOF (e.g., [2, 2, 2] for C2 continuity)
            times: Time points at waypoints (e.g., [0.0, 2.0, 4.0])
        """
        self.n_coeffs = n_coeffs
        self.derivatives = derivatives
        self.times = np.array(times)
        self.T = np.ediff1d(self.times)  # Segment durations
        self._cached_coeff = None

    @staticmethod
    def poly_coeff(n: int, d: int, t: float) -> np.ndarray:
        """
        Compute polynomial coefficients for derivative constraints.
        
        Args:
            n: Polynomial order
            d: Derivative order
            t: Time value
            
        Returns:
            Coefficient array for evaluating the d-th derivative at time t
        """
        assert n > 0 and d >= 0
        D = n - 1 - np.arange(n)
        j = np.arange(d)[:, None]
        factors = np.where(D - j >= 0, D - j, 0)
        prod = np.prod(factors, axis=0)
        exponents = np.maximum(D - d, 0)
        cc = prod * t**exponents
        return cc[::-1].astype(float)

    def hessian(self, n: int, d: int, t: np.ndarray) -> np.ndarray:
        """
        Compute the Hessian matrix Q for a single segment.
        Minimizes the integral of the squared d-th derivative.
        """
        num_t = len(t)
        Q_size = num_t * n
        Qi = np.zeros((Q_size, Q_size))

        for i in range(num_t):
            start_idx = i * n
            end_idx = start_idx + n

            for l in range(n):
                for k in range(n):
                    if l >= d and k >= d:
                        pow_term = l + k - 2 * d + 1
                        product = perm(l, d) * perm(k, d)
                        Qi[start_idx + l, start_idx + k] = 2 * product * (t[i] ** pow_term / pow_term)

        return Qi

    def q_block(self) -> np.ndarray:
        """Generate block-diagonal Hessian matrix for all segments."""
        size = sum(self.n_coeffs) * len(self.T)
        Q_block = np.zeros((size, size))
        cum_idx = 0

        for i, (order, d) in enumerate(zip(self.n_coeffs, self.derivatives)):
            Qi = self.hessian(order, d, self.T)
            block_size = Qi.shape[0]
            Q_block[cum_idx:cum_idx + block_size, cum_idx:cum_idx + block_size] = Qi
            cum_idx += block_size

        return Q_block

    def constraint(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Generate constraint matrix A and target vector f.
        Enforces waypoint positions and derivative continuity.
        """
        n_T = len(self.T)
        n_segments = n_T - 1
        n_axes = len(self.n_coeffs)
        n_constraints = sum((n_T * 2 + n_segments * d) for d in self.derivatives)
        n_coeffs_total = sum(self.n_coeffs) * n_T
        A = np.zeros((n_constraints, n_coeffs_total))
        f = np.zeros(n_coeffs_total)

        start_idx = 0
        
        # Start & end position constraints
        for i in range(n_axes):
            for j in range(n_T):
                idx = i * n_T + j
                end_idx = start_idx + self.n_coeffs[i]
                A[idx, start_idx: end_idx] = self.poly_coeff(self.n_coeffs[i], 0, 0)
                A[n_axes * n_T + idx, start_idx: end_idx] = self.poly_coeff(self.n_coeffs[i], 0, self.T[j])
                start_idx = end_idx

        # Continuous derivatives constraints
        num_pos_constraints = n_axes * n_T * 2
        cumulative_offset = 0

        for j in range(1, max(self.derivatives) + 1):
            valid_axes = np.where(j <= np.array(self.derivatives))[0]
            n_valid_axes = len(valid_axes)
            for k in range(n_segments):
                for i in valid_axes:
                    row_index = num_pos_constraints + cumulative_offset + (i * n_segments + k)
                    start_col = sum(self.n_coeffs[:i]) * n_T + k * self.n_coeffs[i]
                    end_col = start_col + self.n_coeffs[i] * 2
                    coeffs_left = self.poly_coeff(self.n_coeffs[i], j, self.T[k])
                    coeffs_right = -self.poly_coeff(self.n_coeffs[i], j, 0)
                    A[row_index, start_col: end_col] = np.concatenate([coeffs_left, coeffs_right])        
            cumulative_offset += n_valid_axes * n_segments

        return A, f    

    def target(self, waypoint: np.ndarray) -> np.ndarray:
        """Generate the target vector b for the constraints."""
        if waypoint.ndim == 1:
            waypoint = np.expand_dims(waypoint, axis=1)
        n_wp, n_axes = waypoint.shape
        n_T = len(self.T)
        n_segments = n_T - 1
        n_constraints = sum((n_T * 2 + n_segments * d) for d in self.derivatives)
        b = np.zeros(n_constraints)

        for axis in range(n_axes):
            b[axis * n_T : (axis + 1) * n_T] = waypoint[:-1, axis]
            b[n_axes * n_T + axis * n_T : n_axes * n_T + (axis + 1) * n_T] = waypoint[1:, axis]
        return b

    def _solve_qp(self, waypoint: np.ndarray) -> List[float]:
        """Solve the QP problem and return coefficients."""
        Q = matrix(self.q_block())
        A, f = self.constraint()
        f = matrix(f)
        A = matrix(A)
        b = matrix(self.target(waypoint))
        
        # Suppress solver output
        solvers.options['show_progress'] = False
        sol = solvers.qp(Q, f, None, None, A, b)
        
        if sol['status'] != 'optimal':
            raise RuntimeError(f"QP solver failed with status: {sol['status']}")
        
        return list(sol['x'])

    def generate_trajectory(self, waypoint: np.ndarray, num_points: int = 100) -> Tuple[List[np.ndarray], List[float]]:
        """
        Solve the optimization problem and generate the trajectory.
        
        Args:
            waypoint: Shape (n_waypoints, n_dof). Target positions at each waypoint.
            num_points: Number of points to sample in the output trajectory.
            
        Returns:
            states: List of arrays, one per DOF. Each has shape (n_derivatives+1, num_points).
            coeff: Polynomial coefficients.
        """
        coeff = self._solve_qp(waypoint)
        self._cached_coeff = coeff

        N = num_points
        t = np.linspace(self.times[0], self.times[-1], N)
        states = []

        for axis in range(len(self.n_coeffs)):
            d_states = np.zeros((self.derivatives[axis] + 1, N))
            for i in range(N):
                j = np.nonzero(t[i] <= self.times)[0][0] - 1
                j = max(j, 0)
                ti = t[i] - self.times[j]
                start_idx = sum(self.n_coeffs[:axis]) * len(self.T) + self.n_coeffs[axis] * j
                end_idx = start_idx + self.n_coeffs[axis]
                c = np.flip(coeff[start_idx:end_idx])
                current_coeff = c
                for d in range(self.derivatives[axis] + 1):
                    d_states[d, i] = np.polyval(current_coeff, ti)
                    current_coeff = np.polyder(current_coeff)
            states.append(d_states)
        
        return states, coeff

    def solve_at_time(self, waypoint: np.ndarray, t: float) -> List[np.ndarray]:
        """
        Evaluate the trajectory at a specific time t.
        
        Args:
            waypoint: Waypoints (same as generate_trajectory)
            t: Time at which to evaluate
            
        Returns:
            List of arrays, one per DOF, containing [position, velocity, acceleration, ...]
        """
        if self._cached_coeff is None:
            self._cached_coeff = self._solve_qp(waypoint)
        
        coeff = self._cached_coeff
        states = []

        for axis in range(len(self.n_coeffs)):
            d_states = np.zeros(self.derivatives[axis] + 1)
            j = np.nonzero(t <= self.times)[0][0] - 1
            j = max(j, 0)
            ti = t - self.times[j]
            start_idx = sum(self.n_coeffs[:axis]) * len(self.T) + self.n_coeffs[axis] * j
            end_idx = start_idx + self.n_coeffs[axis]
            c = np.flip(coeff[start_idx:end_idx])
            current_coeff = c
            for d in range(self.derivatives[axis] + 1):
                d_states[d] = np.polyval(current_coeff, ti)
                current_coeff = np.polyder(current_coeff)
            states.append(d_states)
        
        return states


class HeadingTracker:
    """
    Tracks heading/yaw from velocity vectors.
    Separated from TrajectoryOptimizer for single responsibility.
    """
    
    def __init__(self, initial_yaw: float = 0.0):
        self.yaw = initial_yaw
        self.heading = np.array([1.0, 0.0])  # Initial heading along +x
    
    def update(self, velocity: np.ndarray, dt: float = 0.005) -> Tuple[float, float]:
        """
        Update heading based on velocity vector.
        
        Args:
            velocity: 2D velocity vector [vx, vy]
            dt: Time step for yaw rate calculation
            
        Returns:
            yaw: Current heading angle in radians
            yawdot: Angular velocity (rad/s)
        """
        vel_norm = np.linalg.norm(velocity)
        if vel_norm < 1e-8:
            return self.yaw, 0.0
        
        curr_heading = velocity / vel_norm
        prev_heading = self.heading
        
        cosine = np.clip(np.dot(prev_heading, curr_heading), -1.0, 1.0)
        dyaw = np.arccos(cosine)
        
        norm_v = np.cross(prev_heading, curr_heading)
        self.yaw += np.sign(norm_v) * dyaw

        # Wrap to [-pi, pi]
        if self.yaw > np.pi:
            self.yaw -= 2 * np.pi
        if self.yaw < -np.pi:
            self.yaw += 2 * np.pi

        self.heading = curr_heading
        yawdot = np.clip(dyaw / dt, -30.0, 30.0)
        
        return self.yaw, yawdot