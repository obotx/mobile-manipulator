from abc import ABC, abstractmethod
import numpy as np

class MotionProfile(ABC):
    """
    Abstract base class for single-axis motion profiles.
    Each profile is stateful: it tracks its own position and velocity.
    """

    @abstractmethod
    def reset(self, initial_value: float = 0.0) -> None:
        """Reset the profile to a known starting state."""
        ...

    @abstractmethod
    def step(self, target: float, dt: float) -> float:
        """
        Given a target value and timestep, compute the next
        smoothed command value respecting the profile constraints.
        """
        ...


class MultiDOFProfile:
    """
    Wraps a MotionProfile factory and applies an independent
    instance of that profile to each DOF in a vector.
    """

    def __init__(self, profile_factory, num_dof: int):
        """
        Args:
            profile_factory: A callable that returns a new MotionProfile instance.
                             Example: lambda: TrapezoidalProfile(max_vel=5.0, max_accel=10.0)
            num_dof: Number of degrees of freedom to control.
        """
        self.profiles = [profile_factory() for _ in range(num_dof)]

    def reset(self, initial_values: np.ndarray) -> None:
        for i, profile in enumerate(self.profiles):
            profile.reset(float(initial_values[i]))

    def step(self, targets: np.ndarray, dt: float) -> np.ndarray:
        result = np.zeros(len(self.profiles))
        for i, profile in enumerate(self.profiles):
            result[i] = profile.step(float(targets[i]), dt)
        return result