"""
Task that holds a position for a specified duration.
Typically used for gripping actions.
"""
import numpy as np
from tasks.base import Task, TaskOutput


class HoldTask(Task):
    """
    Holds the arm at a fixed position for a duration while performing a gripper action.
    """

    def __init__(
        self,
        name: str,
        hold_duration: float,
        gripper_action: str,
        hold_position: np.ndarray,
        pitch_override: float = None,
        roll_override: float = None,
        target_type: str = "arm_left",
    ):
        super().__init__(name)
        self.hold_duration = hold_duration
        self.gripper_action = gripper_action
        self.hold_position = hold_position
        self.pitch_override = pitch_override
        self.roll_override = roll_override
        self.target_type = target_type
        self._elapsed = 0.0

    def on_enter(self, current_time: float) -> None:
        super().on_enter(current_time)
        self._elapsed = 0.0

    def update(self, current_time: float, dt: float) -> TaskOutput:
        self._elapsed += dt
        output = TaskOutput()

        if self.target_type == "arm_left":
            output.arm_left_target = self.hold_position.copy()
            if self.pitch_override is not None:
                output.wrist_left_pitch = self.pitch_override
            if self.roll_override is not None:
                output.wrist_left_roll = self.roll_override
            output.gripper_left = self.gripper_action
        elif self.target_type == "arm_right":
            output.arm_right_target = self.hold_position.copy()
            if self.pitch_override is not None:
                output.wrist_right_pitch = self.pitch_override
            if self.roll_override is not None:
                output.wrist_right_roll = self.roll_override
            output.gripper_right = self.gripper_action

        return output

    def is_complete(self, current_time: float) -> bool:
        return self._elapsed >= self.hold_duration