# utils/camera.py
import cv2
import os
import datetime

class VideoRecorder:
    def __init__(self, width, height, fps=30, output_dir="output_videos"):
        self.width = width
        self.height = height
        self.fps = fps
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)
        self.top_writer = None
        self.pov_writer = None

    def start_recording(self):
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        top_path = os.path.join(self.output_dir, f"top_view_{timestamp}.mp4")
        pov_path = os.path.join(self.output_dir, f"pov_view_{timestamp}.mp4")

        self.top_writer = cv2.VideoWriter(top_path, fourcc, self.fps, (self.width, self.height))
        self.pov_writer = cv2.VideoWriter(pov_path, fourcc, self.fps, (self.width, self.height))

        if not self.top_writer.isOpened():
            print(f"Warning: Top video writer failed to open: {top_path}")
        if not self.pov_writer.isOpened():
            print(f"Warning: POV video writer failed to open: {pov_path}")

    def write_frame(self, top_frame, pov_frame=None):
        if self.top_writer and top_frame is not None:
            self.top_writer.write(top_frame)
        if self.pov_writer and pov_frame is not None:
            self.pov_writer.write(pov_frame)

    def release(self):
        if self.top_writer:
            self.top_writer.release()
        if self.pov_writer:
            self.pov_writer.release()
        cv2.destroyAllWindows()