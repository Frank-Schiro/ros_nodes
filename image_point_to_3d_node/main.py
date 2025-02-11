#!/usr/bin/env python3
import json
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import CameraInfo, PointCloud2, Image
from vision_interfaces.msg import (
    HandDetection2D,
    HandDetection3D,
    Hand3D,
    HandLandmark3D,
)


class ImagePointTo3D(Node):
    """ROS node that combines 2D hand landmarks with depth data to get 3D hand positions"""

    def __init__(self):
        super().__init__("hand_3d_tracking_node")

        # Set up QoS profile for point cloud subscription
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Create synchronized subscribers
        self.hand_detection_sub = Subscriber(self, HandDetection2D, "hand_detection_2d")

        self.pointcloud_sub = Subscriber(
            self,
            PointCloud2,
            "/camera/camera/depth/color/points",
            qos_profile=qos_profile,
        )

        # Create synchronizer
        # Queue size calculation:
        # 30fps = 1 frame every 33ms
        # To buffer 1 second of data: 1000ms/33ms ≈ 30 frames
        # Using 2x for safety: 60 frames
        self.ts = ApproximateTimeSynchronizer(
            [self.hand_detection_sub, self.pointcloud_sub],
            queue_size=60,  # Buffer up to 2 seconds of frames
            slop=0.1,  # 100ms time difference tolerance
        )

        # Add some debugging
        self.last_msg_times = {"hand": None, "cloud": None}
        self.create_timer(5.0, self.check_sync_status)  # Check every 5 seconds
        self.ts.registerCallback(self.sync_callback)

        # Create publisher for 3D hand landmarks
        self.hand_3d_pub = self.create_publisher(
            HandDetection3D, "hand_detection_3d", 10
        )

        self.get_logger().info("Hand3DTrackingNode initialized")

    def check_sync_status(self):
        """Monitor synchronization status and report issues"""
        if None not in self.last_msg_times.values():
            time_diff = abs(self.last_msg_times["hand"] - self.last_msg_times["cloud"])
            if time_diff > 0.1:  # If difference is greater than 100ms
                self.get_logger().warning(
                    f"Large synchronization delay detected: {time_diff:.3f}s"
                )

    def get_point_from_cloud(
        self,
        point_cloud,
        pixel_x: int,
        pixel_y: int,
        image_width: int,
        image_height: int,
    ) -> tuple:
        """Extract 3D point from a flattened point cloud using image coordinates"""

        try:

            # Read the point directly using index
            points = pc2.read_points_numpy(
                point_cloud, field_names=["x", "y", "z"], skip_nans=False
            ).reshape(image_height, image_width, 3)

            # Get the corresponding point from the flattened array
            point = points[pixel_y, pixel_x]

            if np.any(np.isnan(point)):
                self.get_logger().debug(
                    f"NaN values found at pixel ({pixel_x}, {pixel_y})"
                )
                return None

            return tuple(point)  # Convert numpy array to tuple

        except Exception as e:
            self.get_logger().warning(
                f"Error getting point from cloud at pixel ({pixel_x}, {pixel_y}): {str(e)}\n"
                f"Point cloud frame_id: {point_cloud.header.frame_id}"
            )
            return None

    def sync_callback(
        self, hand_detection_msg: HandDetection2D, cloud_msg: PointCloud2
    ):
        """Handle synchronized hand landmarks and point cloud messages"""
        try:
            # Create 3D detection message
            detection_3d = HandDetection3D()
            detection_3d.header.stamp = hand_detection_msg.header.stamp
            detection_3d.header.frame_id = (
                "camera_link"  # 3D data is in camera_link frame
            )
            image_width = hand_detection_msg.image_width
            image_height = hand_detection_msg.image_height

            # Process each hand
            for hand_2d in hand_detection_msg.hands:
                hand_3d = Hand3D()
                hand_3d.hand_id = hand_2d.hand_id

                # Process each landmark in the hand
                for landmark_2d in hand_2d.landmarks:
                    landmark_3d = HandLandmark3D()
                    landmark_3d.name = landmark_2d.name

                    # Get 3D point from point cloud
                    point_3d = self.get_point_from_cloud(
                        cloud_msg,
                        landmark_2d.pixel_x,
                        landmark_2d.pixel_y,
                        image_width,
                        image_height,
                    )

                    if point_3d is not None:
                        landmark_3d.position.x = float(point_3d[0])
                        landmark_3d.position.y = float(point_3d[1])
                        landmark_3d.position.z = float(point_3d[2])
                        landmark_3d.position_confidence = (
                            landmark_2d.detection_confidence
                        )  # maybe multiply by some camera factor
                        hand_3d.landmarks.append(landmark_3d)

                # Only add hand if we got some 3D points
                if hand_3d.landmarks:
                    detection_3d.hands.append(hand_3d)

            # Publish results if we have any hands
            self.get_logger().info(f"hand_3d: {detection_3d}")

            if detection_3d.hands:
                self.hand_3d_pub.publish(detection_3d)
                self.get_logger().info(
                    f"Published HandDetection3D with {len(detection_3d.hands)} hands"
                )

        except Exception as e:
            self.get_logger().error(f"Error in sync callback: {str(e)}")


def main(args=None):
    print("Starting Hand 3D Tracking Node...")

    rclpy.init(args=args)
    node = ImagePointTo3D()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
