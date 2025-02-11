import cv2
import numpy as np
import mediapipe as mp
from typing import Dict, List, Tuple

from src.hand_tracker_port import HandTrackerPort, HandLandmarks


class MediaPipeHandTrackerAdapter(HandTrackerPort):
    """Adapter that implements hand tracking using MediaPipe"""

    # Dictionary mapping landmark names to indices
    HAND_LANDMARKS = {
        "WRIST": 0,
        "THUMB_CMC": 1,
        "THUMB_MCP": 2,
        "THUMB_IP": 3,
        "THUMB_TIP": 4,
        "INDEX_FINGER_MCP": 5,
        "INDEX_FINGER_PIP": 6,
        "INDEX_FINGER_DIP": 7,
        "INDEX_FINGER_TIP": 8,
        "MIDDLE_FINGER_MCP": 9,
        "MIDDLE_FINGER_PIP": 10,
        "MIDDLE_FINGER_DIP": 11,
        "MIDDLE_FINGER_TIP": 12,
        "RING_FINGER_MCP": 13,
        "RING_FINGER_PIP": 14,
        "RING_FINGER_DIP": 15,
        "RING_FINGER_TIP": 16,
        "PINKY_MCP": 17,
        "PINKY_PIP": 18,
        "PINKY_DIP": 19,
        "PINKY_TIP": 20,
    }

    def __init__(self):
        """Initialize MediaPipe hands module"""
        self.mp_hands = mp.solutions.hands
        self.mp_draw = mp.solutions.drawing_utils
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=0.8,
            min_tracking_confidence=0.7,
        )

    def process_frame(self, frame: np.ndarray) -> List[HandLandmarks]:
        """Process frame to detect and track hands

        Args:
            frame: BGR color image

        Returns:
            List of HandLandmarks with hand tracking data
        """
        # Process hands
        results = self.hands.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

        # Clear previous landmarks
        self._latest_mp_landmarks = []

        if not results.multi_hand_landmarks:
            return []

        detected_hands = []
        height, width = frame.shape[:2]

        for hand_landmarks in results.multi_hand_landmarks:
            # Store MediaPipe landmarks for drawing
            self._latest_mp_landmarks.append(hand_landmarks)

            # Store 2D coordinates for each landmark
            landmarks_2d = {}

            for landmark_name, idx in self.HAND_LANDMARKS.items():
                landmark = hand_landmarks.landmark[idx]
                # Store normalized coordinates (0-1 range)
                # landmarks_2d[landmark_name] = (landmark.x, landmark.y)
                # Convert normalized coordinates to pixel coordinates
                pixel_x = int(landmark.x * width)
                pixel_y = int(landmark.y * height)
                landmarks_2d[landmark_name] = (pixel_x, pixel_y)

            detected_hands.append(HandLandmarks(landmark_points=landmarks_2d))

        return detected_hands

    def draw_landmarks(
        self, image: np.ndarray, landmarks: List[HandLandmarks]
    ) -> np.ndarray:
        """Draw hand landmarks on the image using MediaPipe's visualization

        Args:
            image: BGR image to draw on
            landmarks: List of detected hand landmarks
                - landmarks is here to fulfill port but not needed
                - we dont use because it would be pixel coordinates
                - we get normalized coordinates from mediapipe and use that below

        Returns:
            Image with landmarks drawn
        """
        if not self._latest_mp_landmarks:
            return image

        annotated_image = image.copy()
        for hand_landmark in self._latest_mp_landmarks:
            self.mp_draw.draw_landmarks(
                annotated_image, hand_landmark, self.mp_hands.HAND_CONNECTIONS
            )
        return annotated_image
