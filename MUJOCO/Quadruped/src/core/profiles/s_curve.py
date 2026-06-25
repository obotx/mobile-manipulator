import numpy as np
from core.profiles.base import MotionProfile

class SCurveProfile(MotionProfile):
    """
    S-Curve (jerk-limited) motion profile.
    Limits jerk (rate of change of acceleration) for ultra-smooth motion.
    Produces smooth, continuous acceleration curves (S-shape velocity).
    """

    def __init__(self, max_vel: float = 5.0, max_accel: float = 15.0, max_jerk: float = 80.0):
        self.max_vel = max_vel
        self.max_accel = max_accel
        self.max_jerk = max_jerk
        self._pos = 0.0
        self._vel = 0.0
        self._accel = 0.0

    def reset(self, initial_value: float = 0.0) -> None:
        self._pos = initial_value
        self._vel = 0.0
        self._accel = 0.0

    def step(self, target: float, dt: float) -> float:
        error = target - self._pos

        # Stopping distance at current velocity
        stopping_dist = (self._vel ** 2) / (2.0 * self.max_accel) if self.max_accel > 0 else 0.0
        direction = np.sign(error) if abs(error) > 1e-10 else 0.0

        # Determine desired acceleration
        if abs(error) <= stopping_dist + 1e-8:
            # Need to decelerate
            if abs(self._vel) > 1e-8:
                desired_accel = -(self._vel ** 2) / (2.0 * max(abs(error), 1e-8)) * np.sign(self._vel)
            else:
                desired_accel = 0.0
        else:
            # Accelerate toward max_vel in correct direction
            desired_accel = direction * self.max_accel

        # Clamp desired acceleration
        desired_accel = np.clip(desired_accel, -self.max_accel, self.max_accel)

        # Apply jerk limit to reach desired acceleration
        accel_error = desired_accel - self._accel
        max_da = self.max_jerk * dt
        da = np.clip(accel_error, -max_da, max_da)
        self._accel += da

        # Clamp acceleration
        self._accel = np.clip(self._accel, -self.max_accel, self.max_accel)

        # Update velocity
        self._vel += self._accel * dt
        self._vel = np.clip(self._vel, -self.max_vel, self.max_vel)

        # Snap to target if close enough
        if abs(error) < 1e-6 and abs(self._vel) < 1e-6:
            self._pos = target
            self._vel = 0.0
            self._accel = 0.0
        else:
            self._pos += self._vel * dt

        return self._pos