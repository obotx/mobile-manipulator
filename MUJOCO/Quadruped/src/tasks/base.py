"""
Abstract base class for all tasks.
A task represents a single phase of the robot's mission.
"""
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional
import numpy as np


@dataclass
class TaskOutput:
    """Output of a task step: targets for base, arm, and gripper commands."""
    base_target: Optional[np.ndarray] = None      # [x, y, yaw] or None
    arm_left_target: Optional[np.ndarray] = None  # [x, y, z] or None
    arm_right_target: Optional[np.ndarray] = None # [x, y, z] or None
    gripper_left: Optional[str] = None            # "open", "close", or None
    gripper_right: Optional[str] = None           # "open", "close", or None
    wrist_left_pitch: Optional[float] = None
    wrist_left_roll: Optional[float] = None
    wrist_right_pitch: Optional[float] = None
    wrist_right_roll: Optional[float] = None


class Task(ABC):
    """
    Abstract task interface.
    Each task represents one phase of the mission.
    """

    def __init__(self, name: str):
        self.name = name
        self._started = False
        self._start_time: Optional[float] = None

    def on_enter(self, current_time: float) -> None:
        """Called once when the task becomes active."""
        self._started = True
        self._start_time = current_time

    @abstractmethod
    def update(self, current_time: float, dt: float) -> TaskOutput:
        """
        Called every simulation step while the task is active.

        Args:
            current_time: Current simulation time.
            dt: Time step since last call.

        Returns:
            TaskOutput with targets for this step.
        """
        ...

    @abstractmethod
    def is_complete(self, current_time: float) -> bool:
        """Returns True when the task is done and the next task should start."""
        ...

    def on_exit(self) -> None:
        """Called once when the task completes."""
        pass

    @property
    def elapsed(self) -> float:
        """Time elapsed since task started."""
        if self._start_time is None:
            return 0.0
        return self._start_time