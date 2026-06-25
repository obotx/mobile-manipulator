"""
A no-op data source for modes that don't need landmark data.
"""
from typing import Optional
from data.base import DataSource, LandmarkFrame


class NullDataSource(DataSource):
    """Does nothing — used when landmark data isn't needed."""

    def start(self) -> None:
        pass

    def step(self) -> None:
        pass

    def get_current_frame(self) -> Optional[LandmarkFrame]:
        return None