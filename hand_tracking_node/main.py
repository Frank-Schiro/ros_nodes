"""
Overview:

We publish our hands as an array of HandLandmark called HandLandmarkArray.

HandLandmarkArray[
    HandLandmark: {
        landmark_name: str,
        x: float,
        y: float
    }
]

the problem is two hands becomes two separate publications.

We should design a better return object. 


Example Usage: 

from vision_interfaces.msg import HandLandmark, HandLandmarkArray

self.hand_landmarks_pub = self.create_publisher(
    HandLandmarkArray, 
    "hand_landmarks", 
    10
)

def image_callback(self, msg):
    try:
        # Convert ROS Image message to OpenCV image
        color_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Process frame and get hand landmarks
        hand_results = self.tracking_service.process_frame(color_frame)

        # Create messages for each detected hand
        for hand_idx, hand in enumerate(hand_results):
            hand_msg = HandLandmarkArray()
            hand_msg.header = msg.header  # Use original image timestamp
            hand_msg.hand_id = f"hand_{hand_idx}"
            
            # Convert each landmark to message format
            for landmark_name, (x, y) in hand.landmark_points.items():
                landmark = HandLandmark()
                landmark.landmark_name = landmark_name
                landmark.x = float(x)
                landmark.y = float(y)
                # Initialize 3D fields (will be filled by the 3D tracking node)
                landmark.pixel_x = int(x * msg.width)
                landmark.pixel_y = int(y * msg.height)
                landmark.has_3d_point = False
                hand_msg.landmarks.append(landmark)
            
            # Publish the landmarks
            self.hand_landmarks_pub.publish(hand_msg)

            if len(hand_results) > 0:
                self.get_logger().info(
                    f"Published Hand Landmarks for hand_{hand_idx}"
                )

        # Handle visualization if enabled
        if self.show_video:
            annotated_frame = self.tracking_service.draw_landmarks(
                color_frame, hand_results
            )
            display_msg = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
            self.display_pub.publish(display_msg)

    except Exception as e:
        self.get_logger().error(f"Error processing image: {str(e)}")


"""

#!/usr/bin/env python3
import os
import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2

from src.tracking_service import HandTrackingService
from src.mediapipe_hand_tracker_adapter import MediaPipeHandTrackerAdapter

from vision_interfaces.msg import HandLandmark2D, Hand2D, HandDetection2D


class HandTrackingNode(Node):
    """ROS node that coordinates hand tracking and detection publishing"""

    def __init__(self):
        super().__init__("hand_tracking_node")

        debug_mode = os.getenv("DEBUG", "false").lower() == "true"
        self.get_logger().info(f"Debug mode: {debug_mode}")

        if debug_mode:
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        else:
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)

        # Check if visualization is enabled via environment variable
        show_video_str = os.getenv("SHOW_VIDEO", "false").lower()
        self.show_video = show_video_str in ("true", "1", "yes", "on")

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Initialize tracking service
        self.tracking_service = HandTrackingService(
            hand_tracker=MediaPipeHandTrackerAdapter()
        )
        if self.show_video:
            self.display_pub = self.create_publisher(
                Image, "/display/realsense/hand_tracking/annotated", 10
            )

        # Create subscriptions
        self.color_sub = self.create_subscription(
            Image, "/camera/camera/color/image_raw", self.image_callback, 10
        )

        self.hand_detection_pub = self.create_publisher(
            HandDetection2D, "hand_detection_2d", 10
        )

        self.get_logger().info(
            f"HandTrackingNode initialized (visualization: {self.show_video})"
        )

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            color_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Process frame and get hand landmarks
            hand_results = self.tracking_service.process_frame(color_frame)

            # Create HandDetection2D message
            detection_msg = HandDetection2D()
            detection_msg.header = msg.header  # Use original image timestamp
            detection_msg.image_width = msg.width
            detection_msg.image_height = msg.height

            # Process each detected hand
            # 1. Refactor service to Return Hand2D object
            # 2. Wrap Hand2D object in HandDetection2D (the hand detection msg)
            for hand_idx, hand in enumerate(hand_results):
                hand_msg = Hand2D()
                hand_msg.hand_id = f"hand_{hand_idx}"
                hand_msg.handedness = hand.handedness  # Now using actual handedness

                # Convert each landmark to 2D message format
                for landmark_name, (x, y) in hand.landmark_points.items():
                    landmark = HandLandmark2D()
                    landmark.name = landmark_name
                    landmark.pixel_x = int(x)
                    landmark.pixel_y = int(y)
                    hand_msg.landmarks.append(landmark)

                detection_msg.hands.append(hand_msg)

            # Publish the hand detection message
            self.hand_detection_pub.publish(detection_msg)

            if len(hand_results) > 0:
                self.get_logger().info(
                    f"Published object: HandDetection2D to topic: hand_detection_2d - with {len(hand_results)} hands"
                )

            self.get_logger().debug(f"HandDetection2D: {detection_msg}")

            # Handle visualization if enabled
            if self.show_video:
                annotated_frame = self.tracking_service.draw_landmarks(
                    color_frame, hand_results
                )
                display_msg = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
                self.display_pub.publish(display_msg)

        except Exception as e:
            self.get_logger().error(f"Error in image_callback: {str(e)}")


def main(args=None):
    print("Starting Hand Tracking Node...")

    rclpy.init(args=args)
    node = HandTrackingNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
