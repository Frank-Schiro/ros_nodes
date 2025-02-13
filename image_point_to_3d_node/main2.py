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
    Hand2D,
    Hand3D,
    HandLandmark3D,
)
import transforms3d as t3d


class HandsService:
    """Processes 2D hand landmarks and maps them to 3D coordinates using a point cloud."""

    def __init__(self, node):
        self.node = node

    def hand_map(
        self, hand_detection_msg: HandDetection2D, cloud_msg: PointCloud2
    ) -> HandDetection3D:
        detection_3d = HandDetection3D()
        detection_3d.header.stamp = hand_detection_msg.header.stamp
        detection_3d.header.frame_id = "camera_link"

        for hand_2d in hand_detection_msg.hands:
            hand_3d = self.process_joints(hand_2d, cloud_msg)
            detection_3d.hands.append(hand_3d)

        return detection_3d

    def process_joints(self, hand_2d: Hand2D, cloud_msg: PointCloud2) -> Hand3D:
        """Convert 2D hand detection to 3D with joint quaternions."""
        hand_3d = Hand3D()
        hand_3d.hand_id = hand_2d.hand_id

        points_3d = {}
        for landmark in hand_2d.landmarks:
            point_3d = self.get_point_from_cloud(
                cloud_msg, landmark.pixel_x, landmark.pixel_y
            )
            if point_3d is not None:
                points_3d[landmark.name] = np.array(point_3d)

        for landmark in hand_2d.landmarks:
            if landmark.name in points_3d:
                landmark_3d = HandLandmark3D()
                landmark_3d.name = landmark.name
                p = points_3d[landmark.name]
                (
                    landmark_3d.position.x,
                    landmark_3d.position.y,
                    landmark_3d.position.z,
                ) = p.tolist()
                hand_3d.landmarks.append(landmark_3d)

        return hand_3d

    def get_point_from_cloud(self, point_cloud, pixel_x: int, pixel_y: int):
        """Extract 3D point from point cloud using image coordinates."""
        try:
            points = pc2.read_points_numpy(
                point_cloud, field_names=["x", "y", "z"], skip_nans=False
            )
            return tuple(points[pixel_y * point_cloud.width + pixel_x])
        except Exception as e:
            self.node.get_logger().warning(
                f"Error extracting point from cloud: {str(e)}"
            )
            return None
