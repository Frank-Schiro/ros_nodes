"""
# Overview

Output a single JSON message containing all detected hand landmarks. 
The JSON message should be published as a String message on the `hand_landmarks_json` topic.
This message type has no header. Some downstream synchronization  methods need a separate header.

# Example published output
d435i_docker_ros-hand_tracking-1  | [INFO] [1739223778.199769436] [hand_tracking_node]: Published Hand Landmarks JSON: {"header": {"stamp": {"sec": 1739223778, "nanosec": 134649658}, "frame_id": "camera_color_optical_frame"}, "landmarks": {"hand_0": {"WRIST": {"x": 0.5241169929504395, "y": 0.5886802673339844}, "THUMB_CMC": {"x": 0.4962902367115021, "y": 0.5704261660575867}, "THUMB_MCP": {"x": 0.4780610501766205, "y": 0.5262446403503418}, "THUMB_IP": {"x": 0.4746929109096527, "y": 0.4782750606536865}, "THUMB_TIP": {"x": 0.47294333577156067, "y": 0.44099998474121094}, "INDEX_FINGER_MCP": {"x": 0.48114606738090515, "y": 0.4205304682254791}, "INDEX_FINGER_PIP": {"x": 0.47989997267723083, "y": 0.37179869413375854}, "INDEX_FINGER_DIP": {"x": 0.4763927459716797, "y": 0.39470359683036804}, "INDEX_FINGER_TIP": {"x": 0.47377750277519226, "y": 0.42689403891563416}, "MIDDLE_FINGER_MCP": {"x": 0.5057605504989624, "y": 0.4171253442764282}, "MIDDLE_FINGER_PIP": {"x": 0.5124513506889343, "y": 0.3858081102371216}, "MIDDLE_FINGER_DIP": {"x": 0.5152814388275146, "y": 0.43137845396995544}, "MIDDLE_FINGER_TIP": {"x": 0.5145381689071655, "y": 0.4584219753742218}, "RING_FINGER_MCP": {"x": 0.5286127924919128, "y": 0.4258017838001251}, "RING_FINGER_PIP": {"x": 0.5370239615440369, "y": 0.3986867666244507}, "RING_FINGER_DIP": {"x": 0.5369855165481567, "y": 0.43826615810394287}, "RING_FINGER_TIP": {"x": 0.5342483520507812, "y": 0.46505501866340637}, "PINKY_MCP": {"x": 0.5485732555389404, "y": 0.4433487057685852}, "PINKY_PIP": {"x": 0.5560735464096069, "y": 0.4182702600955963}, "PINKY_DIP": {"x": 0.5530518293380737, "y": 0.4468615651130676}, "PINKY_TIP": {"x": 0.5477012395858765, "y": 0.4689025580883026}}}}
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

from vision_interfaces.msg import HandLandmark, HandLandmarkArray


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

        # Create publishers
        self.hand_landmarks_pub = self.create_publisher(
            String, "hand_landmarks_json", 10
        )

        self.get_logger().info(
            f"HandTrackingNode initialized (visualization: {self.show_video})"
        )

    def image_callback(self, msg):
        """Handle new image frame"""
        try:
            # Convert ROS Image message to OpenCV image
            color_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Process frame and get hand landmarks
            hand_results = self.tracking_service.process_frame(color_frame)

            # Create a dictionary for JSON output
            landmarks_dict = {}

            for hand_idx, hand in enumerate(hand_results):
                landmarks_dict[f"hand_{hand_idx}"] = {
                    landmark_name: {"x": float(x), "y": float(y)}
                    for landmark_name, (x, y) in hand.landmark_points.items()
                }

            # Convert dictionary to JSON string
            json_msg = String()

            json_msg.data = json.dumps(
                {
                    "header": {
                        "stamp": {
                            "sec": msg.header.stamp.sec,
                            "nanosec": msg.header.stamp.nanosec,
                        },
                        "frame_id": msg.header.frame_id,
                    },
                    "landmarks": landmarks_dict,
                }
            )
            # json_msg.data = json.dumps(landmarks_dict)

            # Publish JSON message
            self.hand_landmarks_pub.publish(json_msg)

            if len(hand_results) > 0:
                self.get_logger().info(
                    f"Published Hand Landmarks JSON: {json_msg.data}"
                )

            # Display the frame if visualization is enabled
            if self.show_video:

                annotated_frame = self.tracking_service.draw_landmarks(
                    color_frame, hand_results
                )

                # Publish annotated frame to display topic
                display_msg = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
                self.display_pub.publish(display_msg)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")


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
