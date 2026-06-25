import numpy as np
from planning.trajectory_optimizer import TrajectoryOptimizer


class PlannedTrajectory:
    """Container for a planned trajectory with interpolation support."""

    def __init__(self, states: list, time_array: np.ndarray):
        """
        Args:
            states: List of arrays, one per DOF. Each has shape (n_derivatives, n_points).
            time_array: 1D array of time samples.
        """
        self.states = states
        self.time_array = time_array
        self.duration = float(time_array[-1] - time_array[0])

    def sample(self, t_local: float) -> np.ndarray:
        """Interpolate all DOFs at time t_local (relative to trajectory start)."""
        t_local = np.clip(t_local, self.time_array[0], self.time_array[-1])
        values = np.array([
            np.interp(t_local, self.time_array, state[0, :])
            for state in self.states
        ])
        return values

    def sample_dof(self, dof_index: int, t_local: float) -> float:
        """Interpolate a single DOF at time t_local."""
        t_local = np.clip(t_local, self.time_array[0], self.time_array[-1])
        return float(np.interp(t_local, self.time_array, self.states[dof_index][0, :]))

    def final_values(self) -> np.ndarray:
        """Return the final values of all DOFs."""
        return np.array([state[0, -1] for state in self.states])


class TrajectoryPlanner:
    """
    Plans smooth trajectories through waypoints using polynomial optimization.
    """

    def __init__(self, num_points: int = 50):
        self.num_points = num_points

    def plan(
        self,
        waypoints: np.ndarray,
        times: list,
        n_coeffs: list,
        derivatives: list,
    ) -> PlannedTrajectory:
        """
        Plan a trajectory through waypoints.

        Args:
            waypoints: Shape (n_waypoints, n_dof). Target positions at each waypoint.
            times: List of times at which to reach each waypoint.
            n_coeffs: Polynomial order per DOF.
            derivatives: Derivative continuity order per DOF.

        Returns:
            PlannedTrajectory object with interpolation support.
        """
        optimizer = TrajectoryOptimizer(n_coeffs, derivatives, times)
        states, coeffs = optimizer.generate_trajectory(waypoints, num_points=self.num_points)
        time_array = np.linspace(times[0], times[-1], self.num_points)
        return PlannedTrajectory(states, time_array)