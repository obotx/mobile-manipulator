import numpy as np
from scipy.interpolate import CubicSpline
from typing import List, Tuple

class TrajectoryGenerator:
    def __init__(self, max_accel: float = 0.5, max_jerk: float = 1.0):
        self.max_accel = max_accel
        self.max_jerk = max_jerk
        
    def generate_smooth_trajectory(self, start: float, end: float, duration: float, steps: int) -> List[float]:
        """Generate smooth trajectory using cubic spline"""
        time_points = np.linspace(0, duration, steps)
        spline = CubicSpline([0, duration/2, duration], [start, (start+end)/2, end], bc_type='clamped')
        return spline(time_points)
    
    def generate_velocity_profile(self, start_vel: float, target_vel: float, current_time: float, max_time: float) -> float:
        """Generate smooth velocity profile with acceleration limits"""
        if max_time <= 0:
            return target_vel
            
        # Normalized time (0 to 1)
        t = min(current_time / max_time, 1.0)
        
        # Cubic easing function for smooth acceleration/deceleration
        if t < 0.5:
            t2 = 2 * t * t
        else:
            t = 2 * t - 1
            t2 = 1 - (1 - t) * (1 - t)
            
        return start_vel + (target_vel - start_vel) * t2

class MobileBaseTrajectory(TrajectoryGenerator):
    def __init__(self, max_accel: float = 0.5, max_jerk: float = 1.0):
        super().__init__(max_accel, max_jerk)
        self.current_velocity = np.zeros(3)
        
    def update_velocity(self, target_velocity: np.ndarray, dt: float) -> np.ndarray:
        """Update velocity with smooth transition"""
        for i in range(3):
            self.current_velocity[i] = self.generate_velocity_profile(
                self.current_velocity[i], 
                target_velocity[i],
                dt,
                0.3  # Transition time
            )
        return self.current_velocity.copy()