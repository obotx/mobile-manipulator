from abc import ABC, abstractmethod
import mujoco
from typing import Optional, Callable


class Viewer(ABC):
    def __init__(self):
        self.reset_callback: Optional[Callable] = None
        self.key_callback: Optional[Callable] = None

    def set_reset_callback(self, callback: Callable):
        """Set callback for reset action."""
        self.reset_callback = callback

    def set_key_callback(self, callback: Callable):
        """Set callback for general key presses."""
        self.key_callback = callback

    @abstractmethod
    def setup(self, model: mujoco.MjModel, camera: mujoco.MjvCamera, 
              scene: mujoco.MjvScene, opt: mujoco.MjvOption) -> None: ...
    
    @abstractmethod
    def prepare_scene(self, data: mujoco.MjData) -> mujoco.MjvScene: ...
    
    @abstractmethod
    def render_frame(self, scene: mujoco.MjvScene) -> None: ...
    
    @abstractmethod
    def should_close(self) -> bool: ...
    
    @abstractmethod
    def close(self) -> None: ...