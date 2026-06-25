"""
Task that follows a pre-planned trajectory.
"""
import numpy as np
from tasks.base import Task, TaskOutput
from planning.trajectory_planner import PlannedTrajectory


class TrajectoryTask(Task):
    """
    Follows a trajectory for a specific target (base or arm).
    """

    def __init__(
        self,
        name: str,
        trajectory: PlannedTrajectory,
        target_type: str,  # "base", "arm_left", "arm_right"
        gripper_action: str = None,  # "open", "close", or None
        pitch_override: float = None,
        roll_override: float = None,
    ):
        super().__init__(name)
        self.trajectory = trajectory
        self.target_type = target_type
        self.gripper_action = gripper_action
        self.pitch_override = pitch_override
        self.roll_override = roll_override
        self._local_time = 0.0

    def on_enter(self, current_time: float) -> None:
        super().on_enter(current_time)
        self._local_time = 0.0

    def update(self, current_time: float, dt: float) -> TaskOutput:
        self._local_time += dt
        values = self.trajectory.sample(self._local_time)

        output = TaskOutput()

        if self.target_type == "base":
            output.base_target = values[:3]
        elif self.target_type == "arm_left":
            output.arm_left_target = values[:3]
            if self.pitch_override is not None:
                output.wrist_left_pitch = self.pitch_override
            elif len(values) > 3:
                output.wrist_left_pitch = values[3]
            if self.roll_override is not None:
                output.wrist_left_roll = self.roll_override
            elif len(values) > 4:
                output.wrist_left_roll = values[4]
        elif self.target_type == "arm_right":
            output.arm_right_target = values[:3]
            if self.pitch_override is not None:
                output.wrist_right_pitch = self.pitch_override
            elif len(values) > 3:
                output.wrist_right_pitch = values[3]
            if self.roll_override is not None:
                output.wrist_right_roll = self.roll_override
            elif len(values) > 4:
                output.wrist_right_roll = values[4]

        if self.gripper_action:
            if self.target_type == "arm_left":
                output.gripper_left = self.gripper_action
            elif self.target_type == "arm_right":
                output.gripper_right = self.gripper_action

        return output

    def is_complete(self, current_time: float) -> bool:
        return self._local_time >= self.trajectory.duration