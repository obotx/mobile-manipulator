import numpy as np
from scipy.spatial.transform import Rotation
from loop_rate_limiters import RateLimiter
from typing import Optional, Tuple
from rich.panel import Panel
from rich.table import Table
from config import TeleopConfig
from core.mink_robot import MinkRobotInterface
from modules.mink_landmark_pipeline import MinkLandmarkPipeline
from control_input.mink_input_manager import MinkInputManager
from rendering.mink_renderer import MinkRenderer
from utils.logger import log_info, log_success, log_warning, log_error, log_robot, log_pipeline, log_input, log_render, log_debug, console

def compute_face_target_quat(target_pos: np.ndarray, base_xy: np.ndarray, yaw_offset: float = 0.0) -> np.ndarray:
    direction = target_pos[:2] - base_xy
    if np.linalg.norm(direction) < 1e-5:
        return np.array([1.0, 0.0, 0.0, 0.0])
    yaw = np.arctan2(direction[1], direction[0]) + yaw_offset
    quat_xyzw = Rotation.from_euler('z', yaw).as_quat()
    return np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]])

def get_body_world_pose(data, body_name: str):
    body_id = data.model.body(body_name).id
    return data.xpos[body_id].copy(), data.xmat[body_id].reshape(3, 3).copy()

class MinkTeleopController:
    def __init__(self):
        self.config = TeleopConfig()
        self.robot = MinkRobotInterface(self.config)
        self.pipeline = MinkLandmarkPipeline(self.config)
        self.input = MinkInputManager(self.config)
        self.renderer = MinkRenderer(self.config, self.robot.model)
        self.parent_body_name = f"{self.config.robot_prefix}_mobile_base"
        self.rte = RateLimiter(frequency=self.config.control_freq, warn=False)

        self.prev_left_target = None
        self.prev_right_target = None
        self.max_target_velocity = 0.02

    def _print_controls(self):
        table = Table(show_header=False, box=None, padding=(0, 2))
        table.add_column("Category", style="bold cyan", width=12)
        table.add_column("Keys", style="bold white", width=20)
        table.add_column("Action", style="dim")

        table.add_row("GLOBAL", "BACKSPACE", "Reset")
        table.add_row("", "M", "Toggle Mode")
        table.add_row("", "F", "Toggle Face-Target")
        table.add_row("", "[ / ]", "Decrease / Increase Smoothing")
        table.add_section()
        table.add_row("LEFT ARM", "Hold L-Shift", "Activate")
        table.add_row("", "W / S", "Move X")
        table.add_row("", "← / →", "Move Y")
        table.add_row("", "↑ / ↓", "Move Z")
        table.add_row("", "C / O", "Close / Open Gripper")
        table.add_section()
        table.add_row("RIGHT ARM", "Hold R-Shift", "Activate")
        table.add_row("", "W / S", "Move X")
        table.add_row("", "← / →", "Move Y")
        table.add_row("", "↑ / ↓", "Move Z")
        table.add_row("", "C / O", "Close / Open Gripper")

        panel = Panel(
            table,
            title="[bold yellow]⌨  KEYBOARD CONTROLS[/bold yellow]",
            subtitle=f"[dim]Mode: {self.config.tracking_mode} | Hand: {self.config.hand_target_mode}[/dim]",
            border_style="bold blue",
            expand=False,
            padding=(1, 2),
        )

        console.print()
        console.print(panel)
        console.print()

    def _update_face_target(self, left_kb_active: bool, right_kb_active: bool,
                            left_palm_pos: Optional[np.ndarray], right_palm_pos: Optional[np.ndarray]):
        if not self.config.face_target_enabled:
            return

        left_pos = self.robot.get_mocap_pos("left") if left_kb_active else left_palm_pos
        right_pos = self.robot.get_mocap_pos("right") if right_kb_active else right_palm_pos

        if left_pos is not None:
            self.robot.set_mocap_quat("left", compute_face_target_quat(left_pos, self.robot.get_gripper_roll_xy(0), self.config.yaw_offset))
        if right_pos is not None:
            self.robot.set_mocap_quat("right", compute_face_target_quat(right_pos, self.robot.get_gripper_roll_xy(1), self.config.yaw_offset))

    def run(self):
        self.robot.reset()
        self.pipeline.reset()
        self._print_controls()

        mode = self.config.tracking_mode
        face = "ON" if self.config.face_target_enabled else "OFF"
        hand_mode = self.config.hand_target_mode
        log_success(f"MODE: {mode} | FACE: {face} | HAND: {hand_mode}")

        try:
            while not self.renderer.should_close():
                parent_pos, parent_mat = get_body_world_pose(self.robot.data, self.parent_body_name)
                poses_list = self.input.get_tracking_poses()

                has_valid_data = any(p is not None for p in poses_list)
                n_pts = min(len(poses_list), self.config.max_points) if has_valid_data else 0

                world_pts = None
                left_palm_pos, left_palm_quat = None, None
                right_palm_pos, right_palm_quat = None, None
                valid_mask = None

                has_valid_hands = False
                if poses_list:
                    for i in range(91, 133):
                        if i < len(poses_list) and poses_list[i] is not None and poses_list[i].get("valid", False):
                            has_valid_hands = True
                            break

                if has_valid_hands:
                    raw_pts = np.array([
                        [p['position']['x'], p['position']['y'], p['position']['z']] if p is not None else [0.0, 0.0, 0.0]
                        for p in poses_list[:n_pts]
                    ], dtype=np.float64)

                    valid_mask = np.array([p.get("valid", False) if p is not None else False for p in poses_list[:n_pts]], dtype=bool)

                    world_pts, left_palm_data, right_palm_data = self.pipeline.process(
                        raw_pts, n_pts, parent_pos, parent_mat, self.robot
                    )

                    if left_palm_data:
                        left_palm_pos, left_palm_quat = left_palm_data
                    if right_palm_data:
                        right_palm_pos, right_palm_quat = right_palm_data

                left_kb_active = self.input.is_kb_active("left")
                right_kb_active = self.input.is_kb_active("right")
                left_gesture, right_gesture = self.input.get_gestures()
                gripper_commands = self.input.handle_keyboard(self.renderer.viewer, self.robot, self.pipeline)

                # Left Arm
                if not left_kb_active:
                    if left_palm_pos is not None:
                        # Apply velocity limiting
                        if self.prev_left_target is not None:
                            diff = left_palm_pos - self.prev_left_target
                            dist = np.linalg.norm(diff)
                            if dist > self.max_target_velocity:
                                left_palm_pos = self.prev_left_target + (diff / dist) * self.max_target_velocity

                        self.robot.set_mocap_pos("left", left_palm_pos)
                        self.prev_left_target = left_palm_pos.copy()

                        if left_palm_quat is not None:
                            self.robot.set_mocap_quat("left", left_palm_quat)
                    else:
                        self.robot.sync_mocap_to_ee("left")
                        self.prev_left_target = None

                if not right_kb_active:
                    if right_palm_pos is not None:
                        if self.prev_right_target is not None:
                            diff = right_palm_pos - self.prev_right_target
                            dist = np.linalg.norm(diff)
                            if dist > self.max_target_velocity:
                                right_palm_pos = self.prev_right_target + (diff / dist) * self.max_target_velocity

                        self.robot.set_mocap_pos("right", right_palm_pos)
                        self.prev_right_target = right_palm_pos.copy()

                        if right_palm_quat is not None:
                            self.robot.set_mocap_quat("right", right_palm_quat)
                    else:
                        self.robot.sync_mocap_to_ee("right")
                        self.prev_right_target = None

                self._update_face_target(left_kb_active, right_kb_active, left_palm_pos, right_palm_pos)

                self.robot.set_ee_targets()
                self.robot.apply_gripper_commands(gripper_commands)
                self.robot.solve_and_step(self.rte.dt)

                robot_base_pos = self.robot.data.xpos[self.robot.model.body(self.parent_body_name).id]

                if self.config.tracking_mode == "upper-only":
                    anchor_pos = self.pipeline.upper_only_transform.last_anchor
                else:
                    anchor_pos = self.pipeline.full_body_transform.last_anchor

                self.renderer.render_frame(
                    self.robot.data, world_pts, n_pts, self.config.tracking_mode,
                    left_gesture, right_gesture, robot_base_pos, valid_mask, anchor_pos
                )
                self.rte.sleep()

        except KeyboardInterrupt:
            log_warning("Interrupted by user")
        finally:
            self.input.cleanup()
            self.renderer.close()
            log_success("Cleanup complete")
