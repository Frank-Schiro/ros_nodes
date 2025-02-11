from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple
import numpy as np


@dataclass
class HandLandmarks:
    """Data class to store 2D hand landmark information"""

    landmark_points: Dict[
        str, Tuple[float, float]
    ]  # Maps landmark names to (x,y) coordinates
    annotated_image: Optional[np.ndarray] = (
        None  # Image with landmarks drawn (if visualization enabled)
    )


class HandTrackerPort(ABC):
    """Abstract interface for hand tracking implementations"""

    @abstractmethod
    def process_frame(self, frame: np.ndarray) -> List[HandLandmarks]:
        """Process a color frame to detect and track hands

        Args:
            frame: BGR color image

        Returns:
            List of HandLandmarks containing detected hand data
        """
        pass

    @abstractmethod
    def draw_landmarks(
        self, image: np.ndarray, landmarks: List[HandLandmarks]
    ) -> np.ndarray:
        """Draw hand landmarks on the image

        Args:
            image: BGR image to draw on
            landmarks: List of detected hand landmarks

        Returns:
            Image with landmarks drawn
        """
        pass
