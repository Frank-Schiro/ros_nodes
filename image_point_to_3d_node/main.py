import json
from typing import List
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
import os

# Define joint relationships (which points to use for orientation)
joint_connections = {
    # Thumb joints
    "THUMB_CMC": ("WRIST", "THUMB_MCP"),
    "THUMB_MCP": ("THUMB_CMC", "THUMB_IP"),
    "THUMB_IP": ("THUMB_MCP", "THUMB_TIP"),
    # Index finger joints
    "INDEX_FINGER_MCP": ("WRIST", "INDEX_FINGER_PIP"),
    "INDEX_FINGER_PIP": ("INDEX_FINGER_MCP", "INDEX_FINGER_DIP"),
    "INDEX_FINGER_DIP": ("INDEX_FINGER_PIP", "INDEX_FINGER_TIP"),
    # Middle finger joints
    "MIDDLE_FINGER_MCP": ("WRIST", "MIDDLE_FINGER_PIP"),
    "MIDDLE_FINGER_PIP": ("MIDDLE_FINGER_MCP", "MIDDLE_FINGER_DIP"),
    "MIDDLE_FINGER_DIP": ("MIDDLE_FINGER_PIP", "MIDDLE_FINGER_TIP"),
    # Ring finger joints
    "RING_FINGER_MCP": ("WRIST", "RING_FINGER_PIP"),
    "RING_FINGER_PIP": ("RING_FINGER_MCP", "RING_FINGER_DIP"),
    "RING_FINGER_DIP": ("RING_FINGER_PIP", "RING_FINGER_TIP"),
    # Pinky joints
    "PINKY_MCP": ("WRIST", "PINKY_PIP"),
    "PINKY_PIP": ("PINKY_MCP", "PINKY_DIP"),
    "PINKY_DIP": ("PINKY_PIP", "PINKY_TIP"),
}


class HandsService:
    """Processes 2D hand landmarks and maps them to 3D coordinates using a point cloud."""

    def __init__(self, node):
        self.node = node
        self.image_width = 0
        self.image_height = 0

    # Update this to return array of Hand3D instead of HandDetection3D
    # Because HandDetection3D has ROS header which HandsService doesn't need to know about
    # Build HandDetection3D in application
    # How to define array of Hand3D?
    def hand_map(
        self, hands: List[Hand2D], image_width, image_height, cloud_msg: PointCloud2
    ) -> List[Hand3D]:
        hands_3d = []
        self.image_width = image_width
        self.image_height = image_height

        for hand_2d in hands:
            hand_3d = self.process_joints(hand_2d, cloud_msg)
            hands_3d.append(hand_3d)

        return hands_3d

    def process_joints(self, hand_2d: Hand2D, cloud_msg: PointCloud2) -> Hand3D:
        """Convert 2D hand detection to 3D with joint quaternions."""
        hand_3d = Hand3D()
        hand_3d.hand_id = hand_2d.hand_id

        # This loop attempts to get 3D coordinates for each 2D landmark
        points_3d = {}
        for landmark in hand_2d.landmarks:
            point_3d = self.get_point_from_cloud(
                cloud_msg, landmark.pixel_x, landmark.pixel_y
            )
            if point_3d is not None:
                points_3d[landmark.name] = np.array(point_3d)

        # This loop inserts the 3D coordinates into hand_3d object
        for landmark in hand_2d.landmarks:
            if landmark.name in points_3d:
                landmark_3d = HandLandmark3D()
                landmark_3d.name = landmark.name

                p = points_3d[landmark.name]
                landmark_3d.position.x = float(p[0])
                landmark_3d.position.y = float(p[1])
                landmark_3d.position.z = float(p[2])

                hand_3d.landmarks.append(landmark_3d)

        landmark_with_3d = [landmark.name for landmark in hand_3d.landmarks]
        for landmark in hand_3d.landmarks:
            # Special case, use camera to or wrist to palm center
            if landmark.name == "WRIST":
                wrist_coordinates = (
                    landmark.position.x,
                    landmark.position.y,
                    landmark.position.z,
                )
                wrist_quat = self.get_wrist_quaternion(wrist_coordinates)
                landmark.orientation.w = float(wrist_quat[0])
                landmark.orientation.x = float(wrist_quat[1])
                landmark.orientation.y = float(wrist_quat[2])
                landmark.orientation.z = float(wrist_quat[3])
            else:
                if landmark.name in joint_connections:
                    prev_point_name, next_point_name = joint_connections[landmark.name]

                    # Only calculate orientation if we have both reference points
                    if (
                        prev_point_name in landmark_with_3d
                        and next_point_name in landmark_with_3d
                    ):
                        # Find the previous and next landmarks
                        prev_landmark = next(
                            (
                                lm
                                for lm in hand_3d.landmarks
                                if lm.name == prev_point_name
                            ),
                            None,
                        )
                        next_landmark = next(
                            (
                                lm
                                for lm in hand_3d.landmarks
                                if lm.name == next_point_name
                            ),
                            None,
                        )

                        # Get joint frame quaternion
                        if prev_landmark and next_landmark:
                            frame_quat = self.get_joint_frame(
                                np.array(
                                    [
                                        prev_landmark.position.x,
                                        prev_landmark.position.y,
                                        prev_landmark.position.z,
                                    ]
                                ),
                                np.array(
                                    [
                                        next_landmark.position.x,
                                        next_landmark.position.y,
                                        next_landmark.position.z,
                                    ]
                                ),
                            )

                        # Convert from [w,x,y,z] to ROS quaternion [x,y,z,w]
                        landmark.orientation.w = float(frame_quat[0])
                        landmark.orientation.x = float(frame_quat[1])
                        landmark.orientation.y = float(frame_quat[2])
                        landmark.orientation.z = float(frame_quat[3])
        return hand_3d

    def get_wrist_quaternion(self, wrist_position: np.ndarray) -> np.ndarray:
        """
        Calculate wrist quaternion relative to camera origin.
        Camera origin is at (0,0,0).

        Args:
            wrist_position: np.ndarray [x,y,z] position of wrist in camera frame

        Returns:
            np.ndarray quaternion [w,x,y,z] representing wrist orientation
        """
        # Vector from camera to wrist (direction only)
        direction = wrist_position / np.linalg.norm(wrist_position)

        # Calculate rotation from camera's forward vector [0,0,1] to this direction
        # This gives us orientation of wrist relative to camera
        forward = np.array([0, 0, 1])
        rotation_axis = np.cross(forward, direction)

        if np.allclose(rotation_axis, 0):
            # Vectors are parallel, no rotation needed
            return np.array([1, 0, 0, 0])

        rotation_axis = rotation_axis / np.linalg.norm(rotation_axis)
        angle = np.arccos(np.dot(forward, direction))

        # Convert axis-angle to quaternion
        qw = np.cos(angle / 2)
        qx = rotation_axis[0] * np.sin(angle / 2)
        qy = rotation_axis[1] * np.sin(angle / 2)
        qz = rotation_axis[2] * np.sin(angle / 2)

        return np.array([qw, qx, qy, qz])

    def get_finger_quaternions(
        self,
        wrist: np.ndarray,
        mcp: np.ndarray,
        pip: np.ndarray,
        dip: np.ndarray,
        tip: np.ndarray,
    ) -> list:
        """
        Get quaternions representing relative rotations between joints
        Returns [wrist_to_mcp, mcp_to_pip, pip_to_dip] quaternions
        """
        # Get frame quaternions for each joint
        wrist_frame = self.get_joint_frame(wrist, mcp)
        mcp_frame = self.get_joint_frame(mcp, pip)
        pip_frame = self.get_joint_frame(pip, dip)
        dip_frame = self.get_joint_frame(dip, tip)

        # Get relative rotations between frames
        # qmult(qinverse(q1), q2) gives rotation from frame1 to frame2
        wrist_to_mcp = t3d.quaternions.qmult(
            t3d.quaternions.qinverse(wrist_frame), mcp_frame
        )
        mcp_to_pip = t3d.quaternions.qmult(
            t3d.quaternions.qinverse(mcp_frame), pip_frame
        )
        pip_to_dip = t3d.quaternions.qmult(
            t3d.quaternions.qinverse(pip_frame), dip_frame
        )

        return [wrist_to_mcp, mcp_to_pip, pip_to_dip]

    def get_point_from_cloud(
        self, point_cloud: PointCloud2, pixel_x: int, pixel_y: int
    ):
        """
        Extract 3D point from point cloud using image coordinates.
        Why we need raw_image dimensions:
        We can use point_cloud.width, but sometimes the point cloud is smaller than the image
        In that case we should interpolate and for that we need the image dimensions.
        So we need the image the initial points came from.
        """
        try:
            points = pc2.read_points_numpy(
                point_cloud, field_names=["x", "y", "z"], skip_nans=False
            ).reshape(self.image_height, self.image_width, 3)

            # Transform to [0, 1]
            pixel_x = min(pixel_x, self.image_width - 1)
            pixel_y = min(pixel_y, self.image_height - 1)

            point = points[pixel_y, pixel_x]
            return tuple(point)

        except Exception as e:
            self.node.get_logger().warning(
                f"Error in HandsService.get_point_from_cloud: {str(e)}"
            )
            return None

    def get_joint_frame(self, p1: np.ndarray, p2: np.ndarray) -> np.ndarray:
        """
        Create frame quaternion using direction to next joint and camera up

        Args:
            p1: Current joint position (e.g., WRIST, MCP, PIP, DIP)
            p2: Next joint position (e.g., MCP, PIP, DIP, TIP)

        Returns:
            Quaternion representing joint frame orientation [w, x, y, z]
        """
        # Forward vector to next joint
        forward = p2 - p1
        forward = forward / np.linalg.norm(forward)

        # Use camera up as reference
        camera_up = np.array([0, 0, 1])

        # Calculate rotation axis (cross product of vectors)
        rotation_axis = np.cross(camera_up, forward)
        if np.allclose(rotation_axis, 0):
            # Vectors are parallel, return identity quaternion
            return np.array([1.0, 0.0, 0.0, 0.0])

        rotation_axis = rotation_axis / np.linalg.norm(rotation_axis)

        # Calculate rotation angle
        cos_angle = np.clip(np.dot(camera_up, forward), -1.0, 1.0)
        angle = np.arccos(cos_angle)

        # Get rotation matrix using axis-angle representation
        rot_mat = t3d.axangles.axangle2mat(rotation_axis, angle)

        # Convert to quaternion [w, x, y, z]
        return t3d.quaternions.mat2quat(rot_mat)


class ImagePointTo3D(Node):
    """ROS node that synchronizes hand detection and point cloud data, then converts 2D hand landmarks to 3D."""

    def __init__(self):
        super().__init__("hand_3d_tracking_node")

        debug_mode = os.getenv("DEBUG", "false").lower() == "true"

        self.get_logger().info(f"Debug mode: {debug_mode}")

        if debug_mode:
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        else:
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)

        self.get_logger().debug(f"debug mode test (this should print if debug is true)")

        self.hand_service = HandsService(self)

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.hand_detection_sub = Subscriber(self, HandDetection2D, "hand_detection_2d")
        self.pointcloud_sub = Subscriber(
            self,
            PointCloud2,
            "/camera/camera/depth/color/points",
            qos_profile=qos_profile,
        )

        self.ts = ApproximateTimeSynchronizer(
            [self.hand_detection_sub, self.pointcloud_sub],
            queue_size=60,  # Buffer up to 2 seconds of frames
            slop=0.1,
        )
        self.ts.registerCallback(self.sync_callback)

        self.hand_3d_pub = self.create_publisher(
            HandDetection3D, "hand_detection_3d", 10
        )

        # Add message counters
        self.hand_detection_count = 0
        self.pointcloud_count = 0
        self.sync_callback_count = 0

        self.create_timer(5.0, self.diagnostic_callback)

        # Add direct callbacks to track individual messages
        self.direct_hand_sub = self.create_subscription(
            HandDetection2D, "hand_detection_2d", self.hand_detection_callback, 10
        )

        self.direct_pointcloud_sub = self.create_subscription(
            PointCloud2,
            "/camera/camera/depth/color/points",
            self.pointcloud_callback,
            qos_profile,
        )

        self.get_logger().info("Hand3DTrackingNode initialized")

    def diagnostic_callback(self):
        self.get_logger().info(
            f"Diagnostics:\n"
            f"  Hand detections received: {self.hand_detection_count}\n"
            f"  Pointclouds received: {self.pointcloud_count}\n"
            f"  Synchronized callbacks: {self.sync_callback_count}"
        )

    def hand_detection_callback(self, msg):
        self.hand_detection_count += 1
        self.get_logger().debug(
            f"Received hand detection message #{self.hand_detection_count}"
        )

    def pointcloud_callback(self, msg):
        self.pointcloud_count += 1
        self.get_logger().debug(f"Received pointcloud message #{self.pointcloud_count}")

    def sync_callback(
        self, hand_detection_msg: HandDetection2D, cloud_msg: PointCloud2
    ):
        """
        Process synchronized messages and publish 3D hand detection.
        hand_detection_3d is my ros message containing 3D hand landmarks and a header
        Future work: improve the name to specify its the main ROS message.
        Note: Other objects are also ROS message type.
        """
        try:

            self.get_logger().debug(
                f"Received HandDetection2D message: {hand_detection_msg}"
            )
            self.get_logger().debug(f"Received PointCloud message: {cloud_msg}")

            # parse input message
            hands = hand_detection_msg.hands
            image_width = hand_detection_msg.image_width
            image_height = hand_detection_msg.image_height

            # create output message
            output_msg = HandDetection3D()

            self.get_logger().debug(
                f"Processing HandDetection2D message: {hand_detection_msg}"
            )
            # output message payload
            output_msg.hands = self.hand_service.hand_map(
                hands, image_width, image_height, cloud_msg
            )

            # output message header
            output_msg.header = hand_detection_msg.header
            output_msg.header.frame_id = "camera_link"

            # publish output message
            if output_msg.hands:
                self.hand_3d_pub.publish(output_msg)
                self.get_logger().info(
                    f"Published HandDetection3D object to topic: 'hand_detection_3d' with {len(output_msg.hands)} hands"
                )
                self.get_logger().debug(f"HandDetection3D: {output_msg}")

        except Exception as e:
            self.get_logger().error(f"Error in sync callback: {str(e)}")


def main(args=None):
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
