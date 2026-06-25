import os
import datetime
import cv2
import mujoco
from rendering.viewer_base import Viewer
from utils.logger import setup_logger

logger = setup_logger("OpenCvViewer")

class OpenCvViewer(Viewer):
    def __init__(self, width: int = 1024, height: int = 640, record: bool = False):
        super().__init__()
        self.width = width; self.height = height; self.record = record
        self.renderer = None; self.video_writer = None; self._terminate = False; self.camera = None

    def setup(self, model: mujoco.MjModel, camera: mujoco.MjvCamera, scene: mujoco.MjvScene, opt: mujoco.MjvOption) -> None:
        self.camera = camera
        self.renderer = mujoco.Renderer(model, height=self.height, width=self.width)
        model.vis.global_.offheight = self.height; model.vis.global_.offwidth = self.width
        if self.record:
            output_dir = "output_videos"; os.makedirs(output_dir, exist_ok=True)
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            video_file = os.path.join(output_dir, f"top_view_{timestamp}.mp4")
            fourcc = cv2.VideoWriter_fourcc(*'MJPG') 
            self.video_writer = cv2.VideoWriter(video_file, fourcc, 30, (self.width, self.height))

    def prepare_scene(self, data: mujoco.MjData) -> mujoco.MjvScene:
        self.renderer.update_scene(data, self.camera)
        return self.renderer.scene

    def render_frame(self, scene: mujoco.MjvScene) -> None:
        rgb = self.renderer.render()
        if rgb is None or rgb.size == 0: return
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        if self.video_writer and self.video_writer.isOpened(): self.video_writer.write(bgr)
        cv2.imshow("MuJoCo Top View", bgr)
        if cv2.waitKey(1) & 0xFF == ord('q'): self._terminate = True

    def should_close(self) -> bool: return self._terminate
    def close(self) -> None:
        if self.video_writer: self.video_writer.release()
        cv2.destroyAllWindows()
        if self.renderer: self.renderer.close()