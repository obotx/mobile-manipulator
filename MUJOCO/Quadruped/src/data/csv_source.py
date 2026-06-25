import os
import time
import bisect
import json
import csv
from typing import List, Optional
from data.base import DataSource, LandmarkFrame
from utils.logger import setup_logger

logger = setup_logger("CsvDataSource")

class CsvDataSource(DataSource):
    def __init__(self, csv_path: str, playback_rate: float = 1.0):
        self.csv_path = csv_path
        self.playback_rate = playback_rate
        self.frames: List[LandmarkFrame] = []
        self._timestamps: List[float] = []
        self._start_wall_time: Optional[float] = None
        self._first_csv_ts: float = 0.0
        self.current_frame_idx: int = 0

    def start(self) -> None:
        if not self.csv_path or not os.path.exists(self.csv_path):
            logger.warning(f"Landmark CSV not found at '{self.csv_path}'.")
            return
        with open(self.csv_path, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                self.frames.append(LandmarkFrame(
                    timestamp=float(row['timestamp_sec']),
                    left_present=row['left_hand_present'].strip().lower() == 'true',
                    right_present=row['right_hand_present'].strip().lower() == 'true',
                    data=json.loads(row['processed_json'])
                ))
        logger.info(f"Loaded {len(self.frames)} landmark frames.")
        self._timestamps = [f.timestamp for f in self.frames]
        self._first_csv_ts = self._timestamps[0] if self._timestamps else 0.0

    def step(self) -> None:
        if not self.frames: return
        if self._start_wall_time is None:
            self._start_wall_time = time.perf_counter()
            return
        wall_elapsed = time.perf_counter() - self._start_wall_time
        sim_elapsed = wall_elapsed * self.playback_rate
        current_sim_ts = self._first_csv_ts + sim_elapsed
        idx = bisect.bisect_right(self._timestamps, current_sim_ts) - 1
        if idx >= len(self.frames): self._start_wall_time = time.perf_counter(); idx = 0
        elif idx < 0: idx = 0
        self.current_frame_idx = idx
    
    def get_current_frame(self) -> Optional[LandmarkFrame]:
        return self.frames[self.current_frame_idx] if self.frames else None