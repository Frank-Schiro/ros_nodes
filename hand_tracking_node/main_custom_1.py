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

            self.get_logger().info(f"A")
            self.get_logger().info(
                f"image_width, image_height: {msg.width}, {msg.height}"
            )
            self.get_logger().info(f"B")
            # Process each detected hand
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

                    # Check if pixel_x or pixel_y exceeds image dimensions
                    # if landmark.pixel_x >= msg.width or landmark.pixel_y >= msg.height:
                    #     error_msg = f"Landmark {landmark_name} has pixel coordinates ({landmark.pixel_x}, {landmark.pixel_y})\nout of bounds for image size ({msg.width}, {msg.height})"
                    #     self.get_logger().info(f"C")
                    #     self.get_logger().error(error_msg)
                    #     self.get_logger().info(f"D")
                    #     raise ValueError(error_msg)

                    hand_msg.landmarks.append(landmark)

                detection_msg.hands.append(hand_msg)

            # Publish the hand detection message
            self.hand_detection_pub.publish(detection_msg)

            if len(hand_results) > 0:
                self.get_logger().info(f"E")
                self.get_logger().debug(
                    f"Published HandDetection2D with {len(hand_results)} hands"
                )
                self.get_logger().info(f"F")

            self.get_logger().info(f"detection_msg: {detection_msg}")
            self.get_logger().info(f"G")
            # Handle visualization if enabled
            if self.show_video:
                annotated_frame = self.tracking_service.draw_landmarks(
                    color_frame, hand_results
                )
                display_msg = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
                self.display_pub.publish(display_msg)

        except Exception as e:
            self.get_logger().info(f"h")
            self.get_logger().error(f"Error processing image: {str(e)}")
            self.get_logger().info(f"i")


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
