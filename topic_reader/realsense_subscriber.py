#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import cv2
import numpy as np


class RealsenseSubscriber(Node):
    def __init__(self):
        super().__init__("realsense_subscriber")
        self.bridge = CvBridge()

        # ROS publishers for display topics
        self.color_pub = self.create_publisher(Image, "/display/realsense/color", 10)
        self.depth_pub = self.create_publisher(Image, "/display/realsense/depth", 10)

        # Subscribe to color image
        self.color_sub = self.create_subscription(
            Image, "/camera/camera/color/image_raw", self.color_callback, 10
        )

        # Subscribe to depth image
        self.depth_sub = self.create_subscription(
            Image, "/camera/camera/depth/image_rect_raw", self.depth_callback, 10
        )

        # Subscribe to point cloud
        self.pointcloud_sub = self.create_subscription(
            PointCloud2, "/camera/pointcloud", self.pointcloud_callback, 10
        )

        self.get_logger().info("RealsenseSubscriber initialized")

    def color_callback(self, msg):
        """Handle incoming color images and publish them to the display topic."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            display_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")

            # Publish the image to the display node
            self.color_pub.publish(display_msg)

        except Exception as e:
            self.get_logger().error(f"Error processing color image: {str(e)}")

    def depth_callback(self, msg):
        """Handle incoming depth images, normalize them, and publish to the display topic."""
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

            # Normalize depth values to 0-255
            depth_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
            depth_normalized = np.uint8(depth_normalized)

            # Apply a colormap for visualization
            depth_colormap = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)

            # Convert to ROS Image message and publish
            display_msg = self.bridge.cv2_to_imgmsg(depth_colormap, "bgr8")
            self.depth_pub.publish(display_msg)

        except Exception as e:
            self.get_logger().error(f"Error processing depth image: {str(e)}")

    def pointcloud_callback(self, msg):
        """Log the number of points in the point cloud."""
        self.get_logger().info(
            f"Received pointcloud with {msg.width * msg.height} points"
        )


def main(args=None):
    rclpy.init(args=args)
    node = RealsenseSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
