"""
TaskManager: bridges the TaskSequence with the robot's control loop.
Aggregates task outputs and applies them to the robot.
"""
import numpy as np
import mujoco
from tasks.sequence import TaskSequence
from tasks.base import TaskOutput
from utils.logger import setup_logger

logger = setup_logger("TaskManager")


class TaskManager:
    """
    Manages the execution of a task sequence and applies outputs to the robot.
    """

    def __init__(self, robot):
        self.robot = robot
        self.sequence: TaskSequence = None
        self._last_time: float = 0.0
        self._active: bool = False

        # Gripper state
        self._gripper_open_pos = -0.3
        self._gripper_closed_pos = 0.3

    def set_sequence(self, sequence: TaskSequence, start_time: float) -> None:
        """Load and start a task sequence."""
        self.sequence = sequence
        self.sequence.start(start_time)
        self._last_time = start_time
        self._active = True
        logger.info(f"Task sequence started with {len(sequence._tasks)} tasks")

    def update(self, current_time: float) -> None:
        """
        Step the task sequence and apply outputs to the robot.
        """
        if not self._active or self.sequence is None:
            return

        dt = current_time - self._last_time
        self._last_time = current_time

        output = self.sequence.update(current_time, dt)
        self._apply_output(output)

        if self.sequence.is_complete:
            logger.info("Task sequence completed")
            self._active = False

    def _apply_output(self, output: TaskOutput) -> None:
        """Apply a TaskOutput to the robot's control targets."""
        if output.base_target is not None:
            self.robot.target_base = output.base_target.copy()

        if output.arm_left_target is not None:
            self.robot.target_left_global = np.array(output.arm_left_target)
            self.robot.target_left = np.array(output.arm_left_target)

        if output.arm_right_target is not None:
            self.robot.target_right_global = np.array(output.arm_right_target)
            self.robot.target_right = np.array(output.arm_right_target)

        # Apply gripper commands
        if output.gripper_left is not None:
            self._apply_gripper(self.robot.gripper_ids_left, output.gripper_left)
        if output.gripper_right is not None:
            self._apply_gripper(self.robot.gripper_ids_right, output.gripper_right)

        # Apply wrist pitch/roll
        if output.wrist_left_pitch is not None:
            # Index 13 is the wrist pitch actuator
            self.robot.data.ctrl[self.robot.gripper_ids_left[13]] = output.wrist_left_pitch
        if output.wrist_left_roll is not None:
            # Index 14 is the wrist roll actuator
            self.robot.data.ctrl[self.robot.gripper_ids_left[14]] = output.wrist_left_roll
        if output.wrist_right_pitch is not None:
            self.robot.data.ctrl[self.robot.gripper_ids_right[13]] = output.wrist_right_pitch
        if output.wrist_right_roll is not None:
            self.robot.data.ctrl[self.robot.gripper_ids_right[14]] = output.wrist_right_roll

    def _apply_gripper(self, gripper_ids: list, action: str) -> None:
        """Apply open/close command to a gripper's finger joints."""
        finger_indices = [0, 3, 6]  # finger_c_1, finger_b_1, finger_a_1
        target_pos = self._gripper_closed_pos if action == "close" else self._gripper_open_pos
        for idx in finger_indices:
            self.robot.data.ctrl[gripper_ids[idx]] = target_pos

    @property
    def is_active(self) -> bool:
        return self._active

    @property
    def current_task_name(self):
        return self.sequence.current_task_name if self.sequence else None