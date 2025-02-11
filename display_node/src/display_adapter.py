# display_adapter.py
import cv2
import numpy as np
import logging
from typing import Optional
from src.display_port import DisplayPort, WindowConfig, Frame  # Using relative import


class OpenCvDisplayAdapter(DisplayPort):
    """OpenCV implementation of the DisplayPort for a single window"""

    def __init__(self):
        self._window_name: Optional[str] = None
        self._logger = logging.getLogger(__name__)

    def initialize(self, config: WindowConfig) -> None:
        """Initialize a single OpenCV window"""
        try:
            self._window_name = config.name
            cv2.namedWindow(self._window_name, cv2.WINDOW_AUTOSIZE)

            if config.position:
                x, y = config.position
                cv2.moveWindow(self._window_name, x, y)

            if config.size:
                width, height = config.size
                cv2.resizeWindow(self._window_name, width, height)

        except Exception as e:
            self._logger.error(f"Failed to initialize display: {e}")
            raise RuntimeError(f"Display initialization failed: {e}")

    def show_frame(self, frame: Frame) -> None:
        """Display a single frame using OpenCV"""
        if not self._window_name:
            raise RuntimeError("Display not initialized")

        try:
            if not isinstance(frame.data, np.ndarray):
                raise ValueError("Frame data must be a numpy array")

            cv2.imshow(self._window_name, frame.data)

            # Small wait time to update display and check for window closure
            if cv2.waitKey(1) & 0xFF == 27:  # ESC key
                self._logger.info("ESC pressed - initiating display cleanup")
                self.cleanup()

        except Exception as e:
            self._logger.error(f"Error displaying frame: {e}")
            raise

    def is_window_open(self) -> bool:
        """Check if window is still open"""
        if not self._window_name:
            return False

        try:
            return cv2.getWindowProperty(self._window_name, cv2.WND_PROP_VISIBLE) >= 1
        except:
            return False

    def cleanup(self) -> None:
        """Clean up the OpenCV window"""
        try:
            if self._window_name:
                cv2.destroyWindow(self._window_name)
                self._window_name = None
        except Exception as e:
            self._logger.error(f"Error during cleanup: {e}")

    def test_x11_connection(self):
        """Test X11 connection for OpenCV window"""
        try:
            cv2.namedWindow("Test", cv2.WINDOW_AUTOSIZE)
            cv2.destroyAllWindows()
            print("X11 connection test successful")
            return True
        except Exception as e:
            print(f"X11 connection failed: {e}")
            return False
