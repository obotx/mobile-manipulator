"""
Handles keyboard input for manual robot control.
Extracted from the monolithic robot class for clarity.
"""
import numpy as np
import glfw
from utils.logger import setup_logger

logger = setup_logger("KeyboardController")


class KeyboardController:
    """
    Processes keyboard input and applies it to the robot.
    """

    GRIPPER_OPEN_POS = -1.0
    GRIPPER_CLOSED_POS = 1.2218
    GRIPPER_INCREMENT = 0.01

    BEARING_INCREMENT = 0.01
    BEARING_MIN = -1.57
    BEARING_MAX = 1.57

    WRIST_Z_INCREMENT = 0.05
    WRIST_Z_MIN = -3.14159
    WRIST_Z_MAX = 3.14159

    EE_INCREMENT = 0.01

    def __init__(self, robot):
        self.robot = robot

    def handle_key(self, key: int) -> None:
        """Process a single key press."""
        self._handle_gripper_keys(key)
        self._handle_bearing_keys(key)
        self._handle_wrist_keys(key)
        self._handle_ee_position_keys(key)

    def _handle_gripper_keys(self, key: int) -> None:
        """Z/X for left gripper, C/V for right gripper."""
        if key == glfw.KEY_Z:
            self._adjust_gripper(self.robot.gripper_ids_left, +self.GRIPPER_INCREMENT)
        elif key == glfw.KEY_X:
            self._adjust_gripper(self.robot.gripper_ids_left, -self.GRIPPER_INCREMENT)
        elif key == glfw.KEY_C:
            self._adjust_gripper(self.robot.gripper_ids_right, +self.GRIPPER_INCREMENT)
        elif key == glfw.KEY_V:
            self._adjust_gripper(self.robot.gripper_ids_right, -self.GRIPPER_INCREMENT)

    def _adjust_gripper(self, gripper_ids: list, increment: float) -> None:
        finger_indices = [0, 3, 6]
        for idx in finger_indices:
            act_id = gripper_ids[idx]
            current = self.robot.data.ctrl[act_id]
            new_val = np.clip(
                current + increment,
                self.GRIPPER_OPEN_POS,
                self.GRIPPER_CLOSED_POS,
            )
            self.robot.data.ctrl[act_id] = new_val

    def _handle_bearing_keys(self, key: int) -> None:
        """W/S for left bearing, UP/DOWN for right bearing."""
        if key == glfw.KEY_W:
            self._adjust_bearing(self.robot.gripper_ids_left[14], +self.BEARING_INCREMENT)
        elif key == glfw.KEY_S:
            self._adjust_bearing(self.robot.gripper_ids_left[14], -self.BEARING_INCREMENT)
        elif key == glfw.KEY_UP:
            self._adjust_bearing(self.robot.gripper_ids_right[14], +self.BEARING_INCREMENT)
        elif key == glfw.KEY_DOWN:
            self._adjust_bearing(self.robot.gripper_ids_right[14], -self.BEARING_INCREMENT)

    def _adjust_bearing(self, act_id: int, increment: float) -> None:
        current = self.robot.data.ctrl[act_id]
        new_val = np.clip(current + increment, self.BEARING_MIN, self.BEARING_MAX)
        self.robot.data.ctrl[act_id] = new_val
        logger.debug(f"EE BEARING: {new_val:.4f}")

    def _handle_wrist_keys(self, key: int) -> None:
        """A/D for left wrist Z, LEFT/RIGHT for right wrist Z."""
        if key == glfw.KEY_A:
            self._adjust_wrist_z(self.robot.gripper_ids_left[13], +self.WRIST_Z_INCREMENT)
        elif key == glfw.KEY_D:
            self._adjust_wrist_z(self.robot.gripper_ids_left[13], -self.WRIST_Z_INCREMENT)
        elif key == glfw.KEY_LEFT:
            self._adjust_wrist_z(self.robot.gripper_ids_right[13], +self.WRIST_Z_INCREMENT)
        elif key == glfw.KEY_RIGHT:
            self._adjust_wrist_z(self.robot.gripper_ids_right[13], -self.WRIST_Z_INCREMENT)

    def _adjust_wrist_z(self, act_id: int, increment: float) -> None:
        current = self.robot.data.ctrl[act_id]
        new_val = np.clip(current + increment, self.WRIST_Z_MIN, self.WRIST_Z_MAX)
        self.robot.data.ctrl[act_id] = new_val

    def _handle_ee_position_keys(self, key: int) -> None:
        """R/T/F/G/Y/H for left arm, U/I/J/K/O/L for right arm."""
        if key == glfw.KEY_R:
            self.robot.target_left[0] += self.EE_INCREMENT
        elif key == glfw.KEY_T:
            self.robot.target_left[0] -= self.EE_INCREMENT
        elif key == glfw.KEY_F:
            self.robot.target_left[1] += self.EE_INCREMENT
        elif key == glfw.KEY_G:
            self.robot.target_left[1] -= self.EE_INCREMENT
        elif key == glfw.KEY_Y:
            self.robot.target_left[2] += self.EE_INCREMENT
        elif key == glfw.KEY_H:
            self.robot.target_left[2] -= self.EE_INCREMENT
        elif key == glfw.KEY_U:
            self.robot.target_right[0] += self.EE_INCREMENT
        elif key == glfw.KEY_I:
            self.robot.target_right[0] -= self.EE_INCREMENT
        elif key == glfw.KEY_J:
            self.robot.target_right[1] += self.EE_INCREMENT
        elif key == glfw.KEY_K:
            self.robot.target_right[1] -= self.EE_INCREMENT
        elif key == glfw.KEY_O:
            self.robot.target_right[2] += self.EE_INCREMENT
        elif key == glfw.KEY_L:
            self.robot.target_right[2] -= self.EE_INCREMENT

    @staticmethod
    def print_controls() -> None:
        """Print keyboard bindings to console."""
        lines = [
            "MuJoCo Controller - Key Bindings",
            "",
            "[ General ]",
            "  ESC      : Exit simulation",
            "  ENTER    : Reset simulation",
            "",
            "[ Gripper Control ]",
            "  Z        : Close left gripper",
            "  X        : Open left gripper",
            "  C        : Close right gripper",
            "  V        : Open right gripper",
            "",
            "[ Bearing Control ]",
            "  W / S    : Left bearing + / -",
            "  UP / DOWN: Right bearing + / -",
            "",
            "[ Wrist Z Control ]",
            "  A / D    : Left wrist + / -",
            "  LEFT / RIGHT: Right wrist + / -",
            "",
            "[ End-Effector Position ]",
            "  R / T    : Left arm X + / -",
            "  F / G    : Left arm Y + / -",
            "  Y / H    : Left arm Z + / -",
            "  U / I    : Right arm X + / -",
            "  J / K    : Right arm Y + / -",
            "  O / L    : Right arm Z + / -",
        ]
        width = max(len(line) for line in lines)
        border = "─" * (width + 4)
        lines[1] = "─" * width
        print("┌" + border + "┐")
        for line in lines:
            print(f"│  {line.ljust(width)}  │")
        print("└" + border + "┘")