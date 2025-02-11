# display_service.py
import time
import logging
from typing import Any, Optional, Tuple

from src.display_port import DisplayPort, WindowConfig, Frame


class DisplayService:
    """Service for managing a single video display window"""

    def __init__(self, display_port: DisplayPort):
        self._display = display_port
        self._logger = logging.getLogger(__name__)

    def initialize(
        self,
        window_name: str,
        window_size: Optional[Tuple[int, int]] = None,
        window_position: Optional[Tuple[int, int]] = None,
    ) -> None:
        """Initialize the display window

        Args:
            window_name: Name of the window
            window_size: Optional (width, height) for window
            window_position: Optional (x, y) position for window
        """
        config = WindowConfig(
            name=window_name, size=window_size, position=window_position
        )
        self._display.initialize(config)
        self._logger.info(f"Display initialized: {window_name}")

    def display_frame(self, frame_data: Any, timestamp: Optional[float] = None) -> None:
        """Display a single frame

        Args:
            frame_data: The frame to display
            timestamp: Optional timestamp for the frame
        """
        frame = Frame(data=frame_data, timestamp=timestamp or time.time())
        self._display.show_frame(frame)

    def is_active(self) -> bool:
        """Check if display window is still active"""
        return self._display.is_window_open()

    def cleanup(self) -> None:
        """Clean up display resources"""
        self._display.cleanup()
        self._logger.info("Display resources cleaned up")
