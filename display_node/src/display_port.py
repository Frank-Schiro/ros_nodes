# display_port.py
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Any, Optional, Tuple
import numpy as np
import numpy.typing as npt


@dataclass
class WindowConfig:
    """Configuration for a single display window"""

    name: str
    size: Optional[Tuple[int, int]] = None
    position: Optional[Tuple[int, int]] = None


@dataclass
class Frame:
    """Represents a single frame to be displayed"""

    data: npt.NDArray[Any]
    timestamp: Optional[float] = None


class DisplayPort(ABC):
    """Abstract base class defining the interface for a single display window"""

    @abstractmethod
    def initialize(self, config: WindowConfig) -> None:
        """Initialize a single display window with given configuration"""
        pass

    @abstractmethod
    def show_frame(self, frame: Frame) -> None:
        """Display a single frame in the window"""
        pass

    @abstractmethod
    def is_window_open(self) -> bool:
        """Check if the window is still open"""
        pass

    @abstractmethod
    def cleanup(self) -> None:
        """Clean up display resources"""
        pass

    @abstractmethod
    def test_x11_connection() -> None:
        """Test the X11 connection"""
        pass
