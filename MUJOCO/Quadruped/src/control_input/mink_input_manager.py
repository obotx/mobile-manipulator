import glfw
from typing import Dict, Tuple, TYPE_CHECKING
from config import TeleopConfig
from utils.landmark_subscriber import LandmarkSubscriber
from utils.logger import log_input, log_success

if TYPE_CHECKING:
    from control.mink_robot import MinkRobotInterface
    from modules.mink_landmark_pipeline import MinkLandmarkPipeline

class MinkInputManager:
    def __init__(self, config: TeleopConfig):
        self.config = config
        self.landmark_sub = LandmarkSubscriber(max_keypoints=config.max_points)
        self._kb_flags = {'backspace': False, 'm': False, 'f': False, 'lb': False, 'rb': False, 'lc': False, 'lo': False, 'rc': False, 'ro': False}
        log_input("Initialized")

    def get_tracking_poses(self) -> list:
        return self.landmark_sub.get_poses()

    def get_gestures(self) -> Tuple[str, str]:
        return self.landmark_sub.get_gestures()

    def _edge_triggered(self, flag_name: str, pressed: bool) -> bool:
        was_pressed = self._kb_flags[flag_name]
        self._kb_flags[flag_name] = pressed
        return pressed and not was_pressed

    def handle_keyboard(self, viewer, robot: 'MinkRobotInterface', pipeline: 'MinkLandmarkPipeline') -> Dict:
        gripper_commands = {}
        window = glfw.get_current_context()
        if not window:
            return gripper_commands

        l_shift = glfw.get_key(window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
        r_shift = glfw.get_key(window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS

        if self._edge_triggered('backspace', glfw.get_key(window, glfw.KEY_BACKSPACE) == glfw.PRESS):
            robot.reset()
            pipeline.reset()
            log_success("Reset to home position")

        if self._edge_triggered('m', glfw.get_key(window, glfw.KEY_M) == glfw.PRESS):
            self.config.tracking_mode = "full-body" if self.config.tracking_mode == "upper-only" else "upper-only"
            log_input(f"Mode switched to: [bold]{self.config.tracking_mode}[/bold]")

        if self._edge_triggered('f', glfw.get_key(window, glfw.KEY_F) == glfw.PRESS):
            self.config.face_target_enabled = not self.config.face_target_enabled
            status = "ON" if self.config.face_target_enabled else "OFF"
            log_input(f"Face-target: [bold]{status}[/bold]")

        if self._edge_triggered('lb', glfw.get_key(window, glfw.KEY_LEFT_BRACKET) == glfw.PRESS):
            self.config.smoothing_factor = max(0.0, self.config.smoothing_factor - self.config.smoothing_step)
            pipeline.smoothed_pts = None
            log_input(f"Smoothing decreased to: [bold]{self.config.smoothing_factor:.2f}[/bold]")

        if self._edge_triggered('rb', glfw.get_key(window, glfw.KEY_RIGHT_BRACKET) == glfw.PRESS):
            self.config.smoothing_factor = min(1.0, self.config.smoothing_factor + self.config.smoothing_step)
            pipeline.smoothed_pts = None
            log_input(f"Smoothing increased to: [bold]{self.config.smoothing_factor:.2f}[/bold]")

        if l_shift:
            pos = robot.get_mocap_pos("left")
            if glfw.get_key(window, glfw.KEY_UP) == glfw.PRESS: pos[2] += self.config.target_move_step
            if glfw.get_key(window, glfw.KEY_DOWN) == glfw.PRESS: pos[2] -= self.config.target_move_step
            if glfw.get_key(window, glfw.KEY_LEFT) == glfw.PRESS: pos[1] += self.config.target_move_step
            if glfw.get_key(window, glfw.KEY_RIGHT) == glfw.PRESS: pos[1] -= self.config.target_move_step
            if glfw.get_key(window, glfw.KEY_W) == glfw.PRESS: pos[0] += self.config.target_move_step
            if glfw.get_key(window, glfw.KEY_S) == glfw.PRESS: pos[0] -= self.config.target_move_step
            robot.set_mocap_pos("left", pos)

        if r_shift:
            pos = robot.get_mocap_pos("right")
            if glfw.get_key(window, glfw.KEY_UP) == glfw.PRESS: pos[2] += self.config.target_move_step
            if glfw.get_key(window, glfw.KEY_DOWN) == glfw.PRESS: pos[2] -= self.config.target_move_step
            if glfw.get_key(window, glfw.KEY_LEFT) == glfw.PRESS: pos[1] += self.config.target_move_step
            if glfw.get_key(window, glfw.KEY_RIGHT) == glfw.PRESS: pos[1] -= self.config.target_move_step
            if glfw.get_key(window, glfw.KEY_W) == glfw.PRESS: pos[0] += self.config.target_move_step
            if glfw.get_key(window, glfw.KEY_S) == glfw.PRESS: pos[0] -= self.config.target_move_step
            robot.set_mocap_pos("right", pos)

        c_key = glfw.get_key(window, glfw.KEY_C) == glfw.PRESS
        o_key = glfw.get_key(window, glfw.KEY_O) == glfw.PRESS

        if l_shift and c_key and self._edge_triggered('lc', True):
            for letter in "abc":
                for joint in ("joint1", "joint2", "joint3"):
                    gripper_commands[("left", letter, joint)] = robot.finger_close_targets[joint]
            log_input("Left gripper: [bold red]CLOSE[/bold red]")
        else:
            self._kb_flags['lc'] = False

        if l_shift and o_key and self._edge_triggered('lo', True):
            for letter in "abc":
                for joint in ("joint1", "joint2", "joint3"):
                    gripper_commands[("left", letter, joint)] = robot.finger_open_targets[joint]
            log_input("Left gripper: [bold green]OPEN[/bold green]")
        else:
            self._kb_flags['lo'] = False

        if r_shift and c_key and self._edge_triggered('rc', True):
            for letter in "abc":
                for joint in ("joint1", "joint2", "joint3"):
                    gripper_commands[("right", letter, joint)] = robot.finger_close_targets[joint]
            log_input("Right gripper: [bold red]CLOSE[/bold red]")
        else:
            self._kb_flags['rc'] = False

        if r_shift and o_key and self._edge_triggered('ro', True):
            for letter in "abc":
                for joint in ("joint1", "joint2", "joint3"):
                    gripper_commands[("right", letter, joint)] = robot.finger_open_targets[joint]
            log_input("Right gripper: [bold green]OPEN[/bold green]")
        else:
            self._kb_flags['ro'] = False

        left_gesture, right_gesture = self.get_gestures()
        if left_gesture == "close":
            for letter in "abc":
                for joint in ("joint1", "joint2", "joint3"):
                    gripper_commands[("left", letter, joint)] = robot.finger_close_targets[joint]
        elif left_gesture == "open":
            for letter in "abc":
                for joint in ("joint1", "joint2", "joint3"):
                    gripper_commands[("left", letter, joint)] = robot.finger_open_targets[joint]

        if right_gesture == "close":
            for letter in "abc":
                for joint in ("joint1", "joint2", "joint3"):
                    gripper_commands[("right", letter, joint)] = robot.finger_close_targets[joint]
        elif right_gesture == "open":
            for letter in "abc":
                for joint in ("joint1", "joint2", "joint3"):
                    gripper_commands[("right", letter, joint)] = robot.finger_open_targets[joint]

        return gripper_commands

    def is_kb_active(self, side: str) -> bool:
        window = glfw.get_current_context()
        if not window: return False
        return glfw.get_key(window, glfw.KEY_LEFT_SHIFT if side == "left" else glfw.KEY_RIGHT_SHIFT) == glfw.PRESS

    def cleanup(self):
        self.landmark_sub.cleanup()
        log_input("Cleaned up")
