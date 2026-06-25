import mujoco
import glfw
from typing import Optional, Callable
from rendering.viewer_base import Viewer
from utils.logger import setup_logger

logger = setup_logger("GlfwViewer")


class GlfwViewer(Viewer):
    def __init__(self, width: int = 1200, height: int = 900, title: str = "Gripper Simulation"):
        super().__init__()
        self.width = width
        self.height = height
        self.title = title
        self.window = None
        self.ctx = None
        self.viewport = None
        self.model = None
        self.camera = None
        self.scene = None
        self.opt = None
        self._last_mouse_x = 0
        self._last_mouse_y = 0
        self._mouse_left_pressed = False
        self._mouse_right_pressed = False
        self._mouse_middle_pressed = False

    def setup(self, model: mujoco.MjModel, camera: mujoco.MjvCamera, 
              scene: mujoco.MjvScene, opt: mujoco.MjvOption) -> None:
        if not glfw.init():
            raise RuntimeError("GLFW failed to initialize")
        self.window = glfw.create_window(self.width, self.height, self.title, None, None)
        if not self.window:
            glfw.terminate()
            raise RuntimeError("GLFW failed to create window")
        glfw.make_context_current(self.window)
        
        self.ctx = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150)
        self.viewport = mujoco.MjrRect(0, 0, self.width, self.height)
        self.model = model
        self.camera = camera
        self.scene = scene
        self.opt = opt
        
        # Set camera to free mode
        self.camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        
        # Register callbacks
        glfw.set_key_callback(self.window, self._on_key)
        glfw.set_cursor_pos_callback(self.window, self._cursor_pos_callback)
        glfw.set_mouse_button_callback(self.window, self._mouse_button_callback)
        glfw.set_scroll_callback(self.window, self._scroll_callback)

    def set_reset_callback(self, callback: Callable) -> None:
        """Set callback for reset action (Enter key)."""
        self.reset_callback = callback

    def set_key_callback(self, callback: Callable) -> None:
        """Set callback for general key presses."""
        self.key_callback = callback

    def _on_key(self, window, key, scancode, action, mods):
        if action not in (glfw.PRESS, glfw.REPEAT):
            return
        if key == glfw.KEY_ESCAPE:
            glfw.set_window_should_close(self.window, True)
            return
        if key == glfw.KEY_ENTER and self.reset_callback:
            self.reset_callback()
            return
        # Route other keys to the robot's handler
        if self.key_callback:
            self.key_callback(key)

    def _scroll_callback(self, window, xoffset, yoffset):
        if self.camera.type != mujoco.mjtCamera.mjCAMERA_FREE:
            return
        factor = 0.05
        mujoco.mjv_moveCamera(
            self.model, mujoco.mjtMouse.mjMOUSE_ZOOM,
            0, yoffset * factor, self.scene, self.camera
        )

    def _mouse_button_callback(self, window, button, action, mods):
        if self.camera.type != mujoco.mjtCamera.mjCAMERA_FREE:
            return
        pressed = (action == glfw.PRESS)
        if button == glfw.MOUSE_BUTTON_LEFT:
            self._mouse_left_pressed = pressed
        elif button == glfw.MOUSE_BUTTON_RIGHT:
            self._mouse_right_pressed = pressed
        elif button == glfw.MOUSE_BUTTON_MIDDLE:
            self._mouse_middle_pressed = pressed

    def _cursor_pos_callback(self, window, xpos, ypos):
        if self.camera.type != mujoco.mjtCamera.mjCAMERA_FREE:
            self._last_mouse_x, self._last_mouse_y = xpos, ypos
            return
        dx = xpos - self._last_mouse_x
        dy = ypos - self._last_mouse_y
        self._last_mouse_x, self._last_mouse_y = xpos, ypos
        factor = 0.005
        if self._mouse_left_pressed:
            mujoco.mjv_moveCamera(
                self.model, mujoco.mjtMouse.mjMOUSE_ROTATE_H,
                dx * factor, dy * factor, self.scene, self.camera
            )
        elif self._mouse_right_pressed:
            mujoco.mjv_moveCamera(
                self.model, mujoco.mjtMouse.mjMOUSE_MOVE_H,
                dx * factor, dy * factor, self.scene, self.camera
            )
        elif self._mouse_middle_pressed:
            mujoco.mjv_moveCamera(
                self.model, mujoco.mjtMouse.mjMOUSE_ZOOM,
                0.0, dy * factor * 10.0, self.scene, self.camera
            )

    def prepare_scene(self, data: mujoco.MjData) -> mujoco.MjvScene:
        mujoco.mjv_updateScene(
            self.model, data, self.opt, None, self.camera,
            mujoco.mjtCatBit.mjCAT_ALL, self.scene
        )
        return self.scene

    def render_frame(self, scene: mujoco.MjvScene) -> None:
        mujoco.mjr_render(self.viewport, scene, self.ctx)
        glfw.swap_buffers(self.window)
        glfw.poll_events()

    def should_close(self) -> bool:
        return glfw.window_should_close(self.window)

    def close(self) -> None:
        glfw.terminate()