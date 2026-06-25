import numpy as np
from core.profiles.base import MotionProfile

class ExponentialProfile(MotionProfile):
    """
    Exponential smoothing profile.
    This replicates your original alpha-based smoothing:
        output = (1 - alpha) * current + alpha * target
    Useful as a baseline for comparison.
    """

    def __init__(self, alpha: float = 0.1):
        self.alpha = alpha
        self._pos = 0.0

    def reset(self, initial_value: float = 0.0) -> None:
        self._pos = initial_value

    def step(self, target: float, dt: float) -> float:
        # Time-compensated alpha so behavior is consistent across framerates
        # effective_alpha = 1 - (1 - alpha)^(dt / reference_dt)
        # For simplicity and to match original behavior, we use raw alpha
        self._pos = (1.0 - self.alpha) * self._pos + self.alpha * target
        return self._pos