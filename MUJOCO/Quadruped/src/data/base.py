from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional, Dict, Any

@dataclass
class LandmarkFrame:
    timestamp: float
    left_present: bool
    right_present: bool
    data: Dict[str, Any]

class DataSource(ABC):
    @abstractmethod
    def start(self) -> None: ...
    @abstractmethod
    def step(self) -> None: ...
    @abstractmethod
    def get_current_frame(self) -> Optional[LandmarkFrame]: ...