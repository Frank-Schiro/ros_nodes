import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from typing import Dict


import sys
import numpy as np
import platform

print(f"Python {platform.python_version()}", flush=True)
print(f"Python full: {sys.version}", flush=True)
print(f"Numpy: {np.__version__}", flush=True)

from src.display_service import DisplayService
from src.display_adapter import OpenCvDisplayAdapter


class DisplayNode(Node):
    """ROS node for displaying images from multiple topics"""

    def __init__(self):
        super().__init__("display_node")
        self.bridge = CvBridge()
        self.displays: Dict[str, DisplayService] = {}
        self.subscribed_topics = {}

        # Create a timer to check for new topics
        self.create_timer(1.0, self.check_for_new_topics)
        self.get_logger().info("Display node initialized")

    def check_for_new_topics(self):
        """Check for new display topics and subscribe to them"""
        topic_names_and_types = self.get_topic_names_and_types()

        for topic_name, topic_types in topic_names_and_types:
            if (
                topic_name.startswith("/display/")
                and "sensor_msgs/msg/Image" in topic_types
                and topic_name not in self.subscribed_topics
            ):

                self.get_logger().info(f"New display topic found: {topic_name}")
                self.subscribe_to_topic(topic_name)

    def subscribe_to_topic(self, topic_name: str):
        """Subscribe to a new image topic and create display service"""
        # Create display service for this topic
        display_service = DisplayService(OpenCvDisplayAdapter())
        window_name = topic_name.split("/")[-1]
        display_service.initialize(window_name=window_name)
        self.displays[topic_name] = display_service

        # Create subscription
        sub = self.create_subscription(
            Image,
            topic_name,
            lambda msg, topic=topic_name: self.image_callback(msg, topic),
            10,
        )
        self.subscribed_topics[topic_name] = sub

    def image_callback(self, msg: Image, topic_name: str):
        """Handle new image on any subscribed topic"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.displays[topic_name].display_frame(cv_image)
        except Exception as e:
            self.get_logger().error(
                f"Error processing image from {topic_name}: {str(e)}"
            )

    def cleanup(self):
        """Clean up all display services"""
        for display in self.displays.values():
            display.cleanup()


def main(args=None):
    rclpy.init(args=args)
    node = DisplayNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
