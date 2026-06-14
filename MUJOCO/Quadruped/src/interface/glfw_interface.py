# interface/glfw_interface.py
import glfw
import mujoco

def setup_glfw_window(width=1200, height=900, title="Gripper Simulation"):
    if not glfw.init():
        raise RuntimeError("GLFW failed to initialize")
    window = glfw.create_window(width, height, title, None, None)
    if not window:
        glfw.terminate()
        raise RuntimeError("GLFW failed to create window")
    glfw.make_context_current(window)
    return window

class GLFWInputHandler:
    def __init__(self, window, camera, scene, model):
        self.window = window
        self.camera = camera
        self.scene = scene
        self.model = model
        self.glfw = glfw  # expose module for termination
        self._reset_mouse_state()
        self.register_callbacks()

    def _reset_mouse_state(self):
        self._last_mouse_x = 0
        self._last_mouse_y = 0
        self._mouse_left_pressed = False
        self._mouse_right_pressed = False
        self._mouse_middle_pressed = False

    def register_callbacks(self):
        glfw.set_key_callback(self.window, self.on_key)
        glfw.set_cursor_pos_callback(self.window, self._cursor_pos_callback)
        glfw.set_mouse_button_callback(self.window, self._mouse_button_callback)
        glfw.set_scroll_callback(self.window, self._scroll_callback)

    def on_key(self, window, key, scancode, action, mods):
        if action not in (glfw.PRESS, glfw.REPEAT):
            return
        if key == glfw.KEY_ESCAPE:
            glfw.set_window_should_close(window, True)

    def _cursor_pos_callback(self, window, xpos, ypos):
        if self.camera.type != mujoco.mjtCamera.mjCAMERA_FREE:
            self._last_mouse_x, self._last_mouse_y = xpos, ypos
            return

        dx = xpos - self._last_mouse_x
        dy = ypos - self._last_mouse_y
        self._last_mouse_x, self._last_mouse_y = xpos, ypos
        factor = 0.001

        if self._mouse_left_pressed:
            mujoco.mjv_moveCamera(self.model, mujoco.mjtMouse.mjMOUSE_ROTATE_H, dx*factor, dy*factor, self.scene, self.camera)
        elif self._mouse_right_pressed:
            mujoco.mjv_moveCamera(self.model, mujoco.mjtMouse.mjMOUSE_MOVE_H, dx*factor, dy*factor, self.scene, self.camera)
        elif self._mouse_middle_pressed:
            mujoco.mjv_moveCamera(self.model, mujoco.mjtMouse.mjMOUSE_ZOOM, dx*factor, dy*factor, self.scene, self.camera)

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

    def _scroll_callback(self, window, xoffset, yoffset):
        if self.camera.type != mujoco.mjtCamera.mjCAMERA_FREE:
            return
        factor = 0.05
        mujoco.mjv_moveCamera(self.model, mujoco.mjtMouse.mjMOUSE_ZOOM, 0, yoffset*factor, self.scene, self.camera)