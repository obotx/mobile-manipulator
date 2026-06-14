import glfw
import imgui
from imgui.integrations.glfw import GlfwRenderer
from OpenGL import GL
import open3d as o3d
import numpy as np
import math
import sys

class Open3DBackend:
    def __init__(self, width, height):
        self.width = width
        self.height = height

        self.renderer = o3d.visualization.rendering.OffscreenRenderer(
            width, height
        )
        self.scene = self.renderer.scene
        self.scene.set_background([0.1, 0.1, 0.1, 1.0])

        self._create_scene()

    def _create_scene(self):
        points = np.random.uniform(-1, 1, (8000, 3))
        colors = np.random.uniform(0, 1, (8000, 3))

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(colors)

        voxel = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, 0.1)

        mat = o3d.visualization.rendering.MaterialRecord()
        mat.shader = "defaultUnlit"

        self.scene.add_geometry("voxel", voxel, mat)

    def resize(self, width, height):
        width = max(1, width)
        height = max(1, height)

        if width == self.width and height == self.height:
            return

        self.width = width
        self.height = height
        self.scene.set_view_size(width, height)

    def render(self, camera_state):
        self.resize(camera_state["width"], camera_state["height"])

        self.scene.camera.set_projection(
            60.0,
            camera_state["aspect"],
            0.1,
            100.0,
            o3d.visualization.rendering.Camera.FovType.Vertical,
        )

        self.scene.camera.look_at(
            camera_state["lookat"],
            camera_state["eye"],
            camera_state["up"],
        )

        img = self.renderer.render_to_image()
        return np.asarray(img)

class OpenCVBackend:
    def __init__(self, width, height, color=(0, 0, 255)):
        self.width = width
        self.height = height
        self.color = color  # BGR

    def resize(self, width, height):
        width = max(1, width)
        height = max(1, height)

        if width == self.width and height == self.height:
            return

        self.width = width
        self.height = height

    def render(self, camera_state, data=None):
        self.resize(camera_state["width"], camera_state["height"])

        if data is not None:
            img = cv2.resize(data, (self.width, self.height))

            if img.shape[2] == 3:
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGBA)
            elif img.shape[2] == 4:
                pass
            else:
                raise RuntimeError("Unsupported image format")

            return img

        img = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        img[:] = self.color
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGBA)
        return img

class ImGuiViewport:
    def __init__(self, width=800, height=600, name="Viewport"):
        self.width = width
        self.height = height
        self.name = name

        self.yaw = 0.0
        self.pitch = 0.0
        self.distance = 3.0
        self.pan_x = 0.0
        self.pan_y = 0.0
        self.center = np.array([0.0, 0.0, 0.0])
        self.last_mouse = None

        self.tex_id = GL.glGenTextures(1)
        GL.glBindTexture(GL.GL_TEXTURE_2D, self.tex_id)
        GL.glTexImage2D(
            GL.GL_TEXTURE_2D,
            0,
            GL.GL_RGBA,
            self.width,
            self.height,
            0,
            GL.GL_RGBA,
            GL.GL_UNSIGNED_BYTE,
            None,
        )
        GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MIN_FILTER, GL.GL_LINEAR)
        GL.glTexParameteri(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MAG_FILTER, GL.GL_LINEAR)
        GL.glBindTexture(GL.GL_TEXTURE_2D, 0)

    def get_camera_state(self):
        x = self.distance * math.cos(self.pitch) * math.sin(self.yaw)
        y = self.distance * math.sin(self.pitch)
        z = self.distance * math.cos(self.pitch) * math.cos(self.yaw)

        eye = self.center + np.array([x, y, z]) + np.array([self.pan_x, self.pan_y, 0])
        lookat = self.center + np.array([self.pan_x, self.pan_y, 0])
        up = np.array([0, 1, 0])

        return {
            "eye": eye,
            "lookat": lookat,
            "up": up,
        }

    def update_texture(self, image):
        image = np.flipud(image)
        image = np.ascontiguousarray(image)

        h, w = image.shape[:2]
        channels = image.shape[2]

        if channels == 4:
            fmt = GL.GL_RGBA
            internal = GL.GL_RGBA
        elif channels == 3:
            fmt = GL.GL_RGB
            internal = GL.GL_RGB
        else:
            raise RuntimeError("Unsupported image format")

        GL.glBindTexture(GL.GL_TEXTURE_2D, self.tex_id)

        GL.glPixelStorei(GL.GL_UNPACK_ALIGNMENT, 1)

        GL.glTexImage2D(
            GL.GL_TEXTURE_2D,
            0,
            internal,
            w,
            h,
            0,
            fmt,
            GL.GL_UNSIGNED_BYTE,
            image,
        )

        GL.glBindTexture(GL.GL_TEXTURE_2D, 0)

    def draw(
        self,
        render_callback,
        resizable=True,
        scrollable=False,
    ):
        flags = 0

        if not scrollable:
            flags |= imgui.WINDOW_NO_SCROLLBAR
            flags |= imgui.WINDOW_NO_SCROLL_WITH_MOUSE

        if not resizable:
            flags |= imgui.WINDOW_NO_RESIZE

        imgui.begin(self.name, flags=flags)
        imgui.text("Left drag: rotate | Right drag: pan | Scroll: zoom")

        if resizable:
            content_w, content_h = imgui.get_content_region_available()

            new_w = max(1, int(content_w))
            new_h = max(1, int(content_h))

            if new_w != self.width or new_h != self.height:
                self.width = new_w
                self.height = new_h

                GL.glBindTexture(GL.GL_TEXTURE_2D, self.tex_id)
                GL.glTexImage2D(
                    GL.GL_TEXTURE_2D,
                    0,
                    GL.GL_RGBA,
                    self.width,
                    self.height,
                    0,
                    GL.GL_RGBA,
                    GL.GL_UNSIGNED_BYTE,
                    None,
                )
                GL.glBindTexture(GL.GL_TEXTURE_2D, 0)

        imgui.image(self.tex_id, self.width, self.height)

        imgui.set_item_allow_overlap()
        imgui.set_cursor_pos(imgui.get_item_rect_min())
        imgui.invisible_button("viewport_btn", self.width, self.height)

        hovered = imgui.is_item_hovered()
        mouse_pos = imgui.get_mouse_pos()

        if hovered:
            if self.last_mouse is None:
                self.last_mouse = mouse_pos

            dx = mouse_pos.x - self.last_mouse.x
            dy = mouse_pos.y - self.last_mouse.y

            # Rotate
            if imgui.is_mouse_down(0):
                self.yaw += dx * 0.01
                self.pitch -= dy * 0.01
                self.pitch = max(-1.5, min(1.5, self.pitch))

            # Pan
            if imgui.is_mouse_down(1):
                self.pan_x += dx * 0.005
                self.pan_y -= dy * 0.005

            # Zoom
            scroll = imgui.get_io().mouse_wheel
            if scroll != 0:
                self.distance -= scroll * 0.2
                self.distance = max(0.5, self.distance)

            self.last_mouse = mouse_pos
        else:
            self.last_mouse = None

        imgui.end()
        camera_state = self.get_camera_state()
        camera_state["width"] = self.width
        camera_state["height"] = self.height
        camera_state["aspect"] = self.width / self.height

        image = render_callback(camera_state)

        if image is not None:
            self.update_texture(image)

WIN_W, WIN_H = 1200, 800

if not glfw.init():
    sys.exit(1)

window = glfw.create_window(WIN_W, WIN_H, "ImGui + Open3D Viewport", None, None)
glfw.make_context_current(window)

imgui.create_context()
impl = GlfwRenderer(window)

backend = Open3DBackend(800, 600)
viewport_o3d = ImGuiViewport(800, 600, name="Open3D View")
viewport_cv2 = ImGuiViewport(800, 600, name="OpenCV View")

while not glfw.window_should_close(window):
    glfw.poll_events()
    impl.process_inputs()
    imgui.new_frame()

    # Open3D viewport
    viewport_o3d.draw(
        backend.render,
        resizable=True,
        scrollable=False
    )

    # OpenCV viewport
    viewport_cv2.draw(
        red_render,
        resizable=True,
        scrollable=False
    )

    GL.glClearColor(0.2, 0.2, 0.2, 1)
    GL.glClear(GL.GL_COLOR_BUFFER_BIT)

    imgui.render()
    impl.render(imgui.get_draw_data())
    glfw.swap_buffers(window)

impl.shutdown()
glfw.terminate()