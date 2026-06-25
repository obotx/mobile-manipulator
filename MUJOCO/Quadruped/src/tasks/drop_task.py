"""
Task for dropping an object: moves to drop position and opens gripper.
"""
import numpy as np
from tasks.base import Task, TaskOutput


class DropTask(Task):
    """
    Moves arm to a drop position and opens the gripper to release the object.
    """

    def __init__(
        self,
        name: str,
        drop_position: np.ndarray,
        open_duration: float,
        final_roll: float = 0.0,
        final_pitch: float = 0.0,
        target_type: str = "arm_left",
    ):
        super().__init__(name)
        self.drop_position = drop_position
        self.open_duration = open_duration
        self.final_roll = final_roll
        self.final_pitch = final_pitch
        self.target_type = target_type
        self._elapsed = 0.0

    def on_enter(self, current_time: float) -> None:
        super().on_enter(current_time)
        self._elapsed = 0.0

    def update(self, current_time: float, dt: float) -> TaskOutput:
        self._elapsed += dt
        output = TaskOutput()

        if self.target_type == "arm_left":
            output.arm_left_target = self.drop_position.copy()
            output.wrist_left_roll = self.final_roll
            output.wrist_left_pitch = self.final_pitch
            output.gripper_left = "open"
        elif self.target_type == "arm_right":
            output.arm_right_target = self.drop_position.copy()
            output.wrist_right_roll = self.final_roll
            output.wrist_right_pitch = self.final_pitch
            output.gripper_right = "open"

        return output

    def is_complete(self, current_time: float) -> bool:
        return self._elapsed >= self.open_duration