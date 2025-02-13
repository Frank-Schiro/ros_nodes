from dataclasses import dataclass
import cv2
import numpy as np
import mediapipe as mp
from typing import Dict, List, Tuple
import os
from src.hand_tracker_port import HandTrackerPort, HandLandmarks


@dataclass
class HandInfo:
    """Enhanced hand tracking information"""

    landmark_points: Dict[str, Tuple[float, float]]
    handedness: str
    tracking_confidence: float
    landmark_confidences: Dict[str, float]
    tracking_status: str


class MediaPipeHandTrackerAdapter(HandTrackerPort):
    """
    Adapter that implements hand tracking using MediaPipe
    https://github.com/google-ai-edge/mediapipe/blob/master/mediapipe/python/solutions/hands.py
    """

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

    def _determine_tracking_status(self, confidence: float) -> str:
        """Determine tracking status based on confidence score"""
        if confidence >= 0.8:
            return "TRACKING"
        elif confidence >= 0.5:
            return "UNCERTAIN"
        else:
            return "LOST"

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

        for idx, (hand_landmarks, hand_handedness) in enumerate(
            zip(results.multi_hand_landmarks, results.multi_handedness)
        ):
            # Store MediaPipe landmarks for drawing
            self._latest_mp_landmarks.append(hand_landmarks)

            # Get handedness (LEFT/RIGHT) and confidence
            handedness = hand_handedness.classification[0].label.upper()
            tracking_confidence = hand_handedness.classification[0].score
            # Determine tracking status
            tracking_status = self._determine_tracking_status(tracking_confidence)

            # Store 2D coordinates for each landmark
            landmarks_2d = {}
            landmark_confidences = {}

            for landmark_name, idx in self.HAND_LANDMARKS.items():
                landmark = hand_landmarks.landmark[idx]
                # Store normalized coordinates (0-1 range)
                # landmarks_2d[landmark_name] = (landmark.x, landmark.y)
                # Convert normalized coordinates to pixel coordinates
                # pixel_x = int(landmark.x * width)
                # pixel_y = int(landmark.y * height)
                # Because width is out of bounds need width - 1 since starts at zero.

                pixel_x = max(0, int(landmark.x * width) - 1)
                pixel_y = max(0, int(landmark.y * height) - 1)
                landmarks_2d[landmark_name] = (pixel_x, pixel_y)
                if os.getenv("DEBUG") == "true":
                    print(f"landmark_name: {landmark_name}", flush=True)
                    print(
                        f"landmark.x: {landmark.x}, landmark.y: {landmark.y}",
                        flush=True,
                    )
                    print(f"height: {height}, width: {width}", flush=True)
                    print(f"pixel_x: {pixel_x}, pixel_y: {pixel_y}", flush=True)

                landmark_confidences[landmark_name] = landmark.visibility

            # detected_hands.append(HandLandmarks(landmark_points=landmarks_2d))

            # Create HandInfo object with all tracking data
            hand_info = HandInfo(
                landmark_points=landmarks_2d,
                handedness=handedness,
                tracking_confidence=tracking_confidence,
                landmark_confidences=landmark_confidences,
                tracking_status=tracking_status,
            )

            detected_hands.append(hand_info)

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
