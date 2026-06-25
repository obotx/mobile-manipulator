import numpy as np
from core.profiles.base import MotionProfile

class TrapezoidalProfile(MotionProfile):
    """
    Trapezoidal velocity profile.
    Accelerates at max_accel up to max_vel, cruises, then decelerates.
    Produces piecewise-linear velocity (trapezoid shape).
    """

    def __init__(self, max_vel: float = 5.0, max_accel: float = 15.0):
        self.max_vel = max_vel
        self.max_accel = max_accel
        self._pos = 0.0
        self._vel = 0.0

    def reset(self, initial_value: float = 0.0) -> None:
        self._pos = initial_value
        self._vel = 0.0

    def step(self, target: float, dt: float) -> float:
        error = target - self._pos

        # Determine direction
        direction = np.sign(error) if abs(error) > 1e-10 else 0.0

        # Calculate stopping distance at current velocity
        stopping_dist = (self._vel ** 2) / (2.0 * self.max_accel) if self.max_accel > 0 else 0.0

        # Decide whether to accelerate or decelerate
        if abs(error) <= stopping_dist + 1e-8:
            # Deceleration phase
            desired_vel = 0.0
        else:
            # Acceleration or cruise phase
            desired_vel = direction * self.max_vel

        # Compute acceleration needed
        vel_error = desired_vel - self._vel
        max_dv = self.max_accel * dt
        dv = np.clip(vel_error, -max_dv, max_dv)

        self._vel += dv

        # Clamp velocity
        self._vel = np.clip(self._vel, -self.max_vel, self.max_vel)

        # If very close to target and slow, snap to target
        if abs(error) < 1e-6 and abs(self._vel) < 1e-6:
            self._pos = target
            self._vel = 0.0
        else:
            self._pos += self._vel * dt

        return self._pos