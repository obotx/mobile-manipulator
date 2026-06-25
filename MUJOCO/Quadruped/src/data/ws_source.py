import json
import threading
import asyncio
import websockets
from typing import Optional
from data.base import DataSource, LandmarkFrame
from utils.logger import setup_logger

logger = setup_logger("WebSocketDataSource")

class WebSocketDataSource(DataSource):
    def __init__(self, ws_url: str):
        self.ws_url = ws_url
        self.current_frame: Optional[LandmarkFrame] = None
        self.lock = threading.Lock()

    def start(self) -> None:
        logger.info(f"Starting WebSocket client for {self.ws_url}...")
        threading.Thread(target=lambda: asyncio.run(self._ws_client()), daemon=True).start()

    async def _ws_client(self):
        while True:
            try:
                async with websockets.connect(self.ws_url) as ws:
                    logger.info(f"Connected to {self.ws_url}")
                    async for message in ws:
                        try:
                            data = json.loads(message)
                            frame = LandmarkFrame(
                                timestamp=data.get('t', 0.0),
                                left_present=data.get('left_hand', {}).get('present', False),
                                right_present=data.get('right_hand', {}).get('present', False),
                                data=data
                            )
                            with self.lock: self.current_frame = frame
                        except Exception as e: logger.error(f"WS parse error: {e}")
            except Exception as e:
                logger.error(f"WS connection error: {e}. Retrying in 3s...")
                await asyncio.sleep(3)

    def step(self) -> None: pass
    
    def get_current_frame(self) -> Optional[LandmarkFrame]:
        with self.lock: return self.current_frame