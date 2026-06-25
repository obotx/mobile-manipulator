import numpy as np
from core.profiles.base import MotionProfile

class SinusoidalProfile(MotionProfile):
    """
    Sinusoidal motion profile.
    Uses a sine-based interpolation for inherently smooth start and stop.
    The velocity follows a raised-cosine curve (zero at start and end).
    """

    def __init__(self, max_vel: float = 5.0):
        self.max_vel = max_vel
        self._pos = 0.0
        self._target = 0.0
        self._start = 0.0
        self._progress = 1.0  # 1.0 means idle (motion complete)

    def reset(self, initial_value: float = 0.0) -> None:
        self._pos = initial_value
        self._target = initial_value
        self._start = initial_value
        self._progress = 1.0

    def step(self, target: float, dt: float) -> float:
        # Detect new target
        if abs(target - self._target) > 1e-8:
            self._start = self._pos
            self._target = target
            self._progress = 0.0

        # If idle, just track target
        if self._progress >= 1.0:
            self._pos = self._target
            return self._pos

        # Calculate total distance and duration
        distance = abs(self._target - self._start)
        if distance < 1e-10:
            self._pos = self._target
            self._progress = 1.0
            return self._pos

        # Duration based on max_vel (average vel = max_vel * 2/pi for sine)
        avg_vel = self.max_vel * (2.0 / np.pi)
        duration = max(distance / avg_vel, 0.01)

        # Advance progress
        self._progress += dt / duration
        self._progress = min(self._progress, 1.0)

        # Sinusoidal interpolation: smooth ease-in/ease-out
        # s(t) = 0.5 * (1 - cos(pi * t)) maps [0,1] -> [0,1] smoothly
        s = 0.5 * (1.0 - np.cos(np.pi * self._progress))

        direction = np.sign(self._target - self._start)
        self._pos = self._start + direction * distance * s

        if self._progress >= 1.0:
            self._pos = self._target

        return self._pos