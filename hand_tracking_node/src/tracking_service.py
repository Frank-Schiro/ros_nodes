from typing import List
import numpy as np
from src.hand_tracker_port import HandTrackerPort, HandLandmarks


class HandTrackingService:
    """Service that coordinates hand tracking"""

    def __init__(self, hand_tracker: HandTrackerPort):
        """Initialize the hand tracking service

        Args:
            hand_tracker: Implementation of HandTrackerPort interface
        """
        self._hand_tracker = hand_tracker

    def process_frame(self, frame: np.ndarray) -> List[HandLandmarks]:
        """Process new color frame

        Args:
            frame: BGR color image
            visualize: Whether to annotate the image with landmarks

        Returns:
            List of HandLandmarks containing hand data
        """
        return self._hand_tracker.process_frame2(frame)

    def draw_landmarks(
        self, image: np.ndarray, landmarks: List[HandLandmarks]
    ) -> np.ndarray:
        """Draw landmarks on the image using the underlying tracker

        Args:
            image: BGR image to draw on
            landmarks: List of detected hand landmarks

        Returns:
            Image with landmarks drawn
        """
        return self._hand_tracker.draw_landmarks(image, landmarks)
