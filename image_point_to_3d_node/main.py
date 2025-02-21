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
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
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
import cv2

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
        # Matrix dimensions for hand joints (21 joints x 3 coordinates)
        self.matrix_shape = (21, 3)  # [num_joints, (x,y,z)]

        # Initialize Kalman filter
        flattened_dim = np.prod(self.matrix_shape)  # 21 * 3 = 63
        self.kalman = cv2.KalmanFilter(flattened_dim, flattened_dim)
        self.kalman.measurementMatrix = np.eye(flattened_dim, dtype=np.float32)
        self.kalman.transitionMatrix = np.eye(flattened_dim, dtype=np.float32)
        self.kalman.processNoiseCov = np.eye(flattened_dim, dtype=np.float32) * 0.03
        self.kalman.measurementNoiseCov = np.eye(flattened_dim, dtype=np.float32) * 0.1

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

        # Create and filter joint matrix
        joint_matrix = np.zeros(self.matrix_shape)
        for i, landmark in enumerate(hand_3d.landmarks):
            joint_matrix[i] = [
                landmark.position.x,
                landmark.position.y,
                landmark.position.z,
            ]

        # Flatten, filter, and reshape
        flattened = joint_matrix.flatten()
        prediction = self.kalman.predict()
        filtered_flat = self.kalman.correct(flattened.astype(np.float32))
        filtered_matrix = filtered_flat.reshape(self.matrix_shape)

        # Update landmark positions with filtered values
        for i, landmark in enumerate(hand_3d.landmarks):
            landmark.position.x = float(filtered_matrix[i][0])
            landmark.position.y = float(filtered_matrix[i][1])
            landmark.position.z = float(filtered_matrix[i][2])

        # Calculate orientations for each joint
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
            # Extract the width and height from the PointCloud2 message
            cloud_width = point_cloud.width
            cloud_height = point_cloud.height
            # points.shape = (407040, 3) where 407040 = 848 * 480
            # Read the points as a numpy array and reshape using the intrinsic width and height
            points = pc2.read_points_numpy(
                point_cloud, field_names=["x", "y", "z"], skip_nans=False
            ).reshape(cloud_height, cloud_width, 3)

            pixel_x = min(pixel_x, cloud_width - 1)
            pixel_y = min(pixel_y, cloud_height - 1)

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

        # Get width and height from parameters
        depth_profile = self.get_parameter_or(
            "depth_module.depth_profile", "848x480x30"
        )
        color_profile = self.get_parameter_or("rgb_camera.color_profile", "848x480x30")

        self.depth_width, self.depth_height, self.depth_fps = self.parse_profile(
            depth_profile
        )
        self.color_width, self.color_height, self.color_fps = self.parse_profile(
            color_profile
        )

        self.get_logger().info(
            f"Depth Profile: {self.depth_width}x{self.depth_height} @ {self.depth_fps} FPS"
        )
        self.get_logger().info(
            f"Color Profile: {self.color_width}x{self.color_height} @ {self.color_fps} FPS"
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
        self.publish_count = 0

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

        # publish markers for visualization in rviz
        self.marker_pub = self.create_publisher(MarkerArray, "hand_markers", 10)

        self.get_logger().info("Hand3DTrackingNode initialized")

    def parse_profile(self, profile):
        """Parse profile string like '848x480x30' into width, height, fps"""
        parts = profile.lower().replace(" ", "").split("x")
        return int(parts[0]), int(parts[1]), int(parts[2])

    def diagnostic_callback(self):
        self.get_logger().info(
            f"Stats:\n"
            f"  Hand detections received: {self.hand_detection_count}\n"
            f"  Pointclouds received: {self.pointcloud_count}\n"
            f"  Synchronized callbacks: {self.sync_callback_count}\n"
            f"  Published messages: {self.publish_count}"
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
            self.get_logger().debug(f"Received PointCloud message")

            # ================================================== #
            # Debug Loop - Print a line to verify point cloud to 3d conversion
            # ================================================== #
            # Add test code here before processing hands
            # Test middle row only
            middle_row = cloud_msg.height // 2  # Half of 480 = 240
            marker_array = MarkerArray()

            # Get point cloud data
            points = pc2.read_points_numpy(
                cloud_msg, field_names=["x", "y", "z"], skip_nans=False
            ).reshape(cloud_msg.height, cloud_msg.width, 3)

            # Sample points across the width
            for x in range(
                0, cloud_msg.width, 20
            ):  # Step by 20 to avoid too many markers
                point = points[middle_row, x]

                # Create marker for this point
                marker = Marker()
                # marker.header.frame_id = "camera_link" # When in camera_link, the coordinate axes were wrong
                marker.header.frame_id = cloud_msg.header.frame_id
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "test_row"
                marker.id = x
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                # The original version, the red line completly off the point cloud
                marker.pose.position.x = float(point[0])
                marker.pose.position.y = float(point[1])
                marker.pose.position.z = float(point[2])

                # Try 1: Negate y (WORKING!)
                # marker.pose.position.x = float(point[2])
                # marker.pose.position.y = -float(point[0])
                # marker.pose.position.z = float(point[1])

                marker.scale.x = 0.01
                marker.scale.y = 0.01
                marker.scale.z = 0.01
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0

                marker_array.markers.append(marker)

            # Publish test markers
            self.marker_pub.publish(marker_array)

            # ================================================== #
            # Debug Loop - end
            # ================================================== #

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
            # output_msg.header = hand_detection_msg.header
            output_msg.header = cloud_msg.header
            output_msg.header.frame_id = cloud_msg.header.frame_id

            # ================================================== #
            # Debug Loop - Print all finger points
            # ================================================== #
            for hand_idx, (hand_2d, hand_3d) in enumerate(zip(hands, output_msg.hands)):
                self.get_logger().info(f"\n=== Hand {hand_idx} Finger Points ===")

                # Get all 2D and 3D points
                points_2d = {
                    landmark.name: (landmark.pixel_x, landmark.pixel_y)
                    for landmark in hand_2d.landmarks
                }

                points_3d = {
                    landmark.name: (
                        landmark.position.x,
                        landmark.position.y,
                        landmark.position.z,
                    )
                    for landmark in hand_3d.landmarks
                }

                # Define finger groups and their points in order
                finger_groups = {
                    "Thumb": ["THUMB_CMC", "THUMB_MCP", "THUMB_IP", "THUMB_TIP"],
                    "Index": [
                        "INDEX_FINGER_MCP",
                        "INDEX_FINGER_PIP",
                        "INDEX_FINGER_DIP",
                        "INDEX_FINGER_TIP",
                    ],
                    "Middle": [
                        "MIDDLE_FINGER_MCP",
                        "MIDDLE_FINGER_PIP",
                        "MIDDLE_FINGER_DIP",
                        "MIDDLE_FINGER_TIP",
                    ],
                    "Ring": [
                        "RING_FINGER_MCP",
                        "RING_FINGER_PIP",
                        "RING_FINGER_DIP",
                        "RING_FINGER_TIP",
                    ],
                    "Pinky": ["PINKY_MCP", "PINKY_PIP", "PINKY_DIP", "PINKY_TIP"],
                }

                # Print wrist point first
                if "WRIST" in points_2d and "WRIST" in points_3d:
                    px, py = points_2d["WRIST"]
                    x, y, z = points_3d["WRIST"]
                    self.get_logger().info(
                        f"\nWRIST: [{px}, {py}], [{x:.3f}, {y:.3f}, {z:.3f}]"
                    )

                # Print each finger group
                for finger_name, points in finger_groups.items():
                    self.get_logger().info(f"\n{finger_name} Finger:")
                    for point_name in points:
                        if point_name in points_2d and point_name in points_3d:
                            px, py = points_2d[point_name]
                            x, y, z = points_3d[point_name]
                            self.get_logger().info(
                                f"{point_name}: [{px}, {py}], [{x:.3f}, {y:.3f}, {z:.3f}]"
                            )

            # ================================================== #
            # Debug Loop End
            # ================================================== #
            # publish output message
            if output_msg.hands:
                self.hand_3d_pub.publish(output_msg)
                self.publish_count += 1
                self.get_logger().info(
                    f"Published HandDetection3D object to topic: 'hand_detection_3d' with {len(output_msg.hands)} hands"
                )
                self.get_logger().debug(f"HandDetection3D: {output_msg}")

                self.get_logger().debug(f"starting marker topic")
                if os.getenv("MARKERS", "false").lower() == "true":
                    self.display(output_msg.hands)

        except Exception as e:
            self.get_logger().error(f"Error in sync callback: {str(e)}")

    def display(self, hands_3d):
        self.get_logger().debug(f"self.display initiated")
        marker_array = MarkerArray()

        for hand in hands_3d:
            # Create a dictionary to store the positions of the joints
            joint_positions = {}
            for landmark in hand.landmarks:
                joint_positions[landmark.name] = (
                    landmark.position.x,
                    landmark.position.y,
                    landmark.position.z,
                )

            # Define the joints we are interested in
            # points_of_interest = ["WRIST", "THUMB_TIP", "PINKY_TIP"]

            points_of_interest = [
                "WRIST",  # 0
                "THUMB_CMC",  # 1
                "THUMB_MCP",  # 2
                "THUMB_IP",  # 3
                "THUMB_TIP",  # 4
                "INDEX_FINGER_MCP",  # 5
                "INDEX_FINGER_PIP",  # 6
                "INDEX_FINGER_DIP",  # 7
                "INDEX_FINGER_TIP",  # 8
                "MIDDLE_FINGER_MCP",  # 9
                "MIDDLE_FINGER_PIP",  # 10
                "MIDDLE_FINGER_DIP",  # 11
                "MIDDLE_FINGER_TIP",  # 12
                "RING_FINGER_MCP",  # 13
                "RING_FINGER_PIP",  # 14
                "RING_FINGER_DIP",  # 15
                "RING_FINGER_TIP",  # 16
                "PINKY_MCP",  # 17
                "PINKY_PIP",  # 18
                "PINKY_DIP",  # 19
                "PINKY_TIP",  # 20
            ]

            self.get_logger().debug(f"points_of_interest: {points_of_interest}")

            # Log the coordinates for each point of interest
            for joint_name in points_of_interest:
                if joint_name in joint_positions:
                    x, y, z = joint_positions[joint_name]
                    self.get_logger().info(f"{joint_name}: ({x:.3f}, {y:.3f}, {z:.3f})")
                else:
                    self.get_logger().warn(f"{joint_name} not found in joint_positions")

            # Create joint markers
            for joint_name in points_of_interest:
                if joint_name in joint_positions:
                    x, y, z = joint_positions[joint_name]

                    joint_marker = Marker()
                    joint_marker.header.frame_id = "camera_depth_optical_frame"
                    joint_marker.header.stamp = self.get_clock().now().to_msg()
                    joint_marker.ns = "hand_joints"
                    joint_marker.id = points_of_interest.index(joint_name)
                    joint_marker.type = Marker.SPHERE
                    joint_marker.action = Marker.ADD
                    joint_marker.pose.position.x = x
                    joint_marker.pose.position.y = y
                    joint_marker.pose.position.z = z
                    joint_marker.scale.x = 0.015
                    joint_marker.scale.y = 0.015
                    joint_marker.scale.z = 0.015
                    joint_marker.color.a = 1.0
                    joint_marker.color.r = 1.0  # Red for joints
                    joint_marker.color.g = 0.0
                    joint_marker.color.b = 0.0
                    marker_array.markers.append(joint_marker)

                    # Create label marker (text) only for the tips of each finger
                    if joint_name in [
                        "THUMB_TIP",
                        "INDEX_FINGER_TIP",
                        "MIDDLE_FINGER_TIP",
                        "RING_FINGER_TIP",
                        "PINKY_TIP",
                        "WRIST",
                    ]:
                        label_marker = Marker()
                        label_marker.header.frame_id = "camera_depth_optical_frame"
                        label_marker.header.stamp = self.get_clock().now().to_msg()
                        label_marker.ns = "hand_labels"
                        label_marker.id = (
                            points_of_interest.index(joint_name) + 1000
                        )  # Unique ID for labels
                        label_marker.type = Marker.TEXT_VIEW_FACING
                        label_marker.action = Marker.ADD
                        label_marker.pose.position.x = x
                        label_marker.pose.position.y = y
                        label_marker.pose.position.z = (
                            z + 0.02
                        )  # Adjust the offset for better visibility
                        label_marker.scale.z = 0.01  # Adjust text size
                        label_marker.color.a = 1.0
                        label_marker.color.r = 1.0
                        label_marker.color.g = 1.0
                        label_marker.color.b = 1.0
                        label_marker.text = joint_name  # Simplified label text
                        marker_array.markers.append(label_marker)

            connections = [
                # Thumb connections (0-4)
                ("WRIST", "THUMB_CMC"),  # 0-1
                ("THUMB_CMC", "THUMB_MCP"),  # 1-2
                ("THUMB_MCP", "THUMB_IP"),  # 2-3
                ("THUMB_IP", "THUMB_TIP"),  # 3-4
                # Index finger connections (0,5-8)
                ("WRIST", "INDEX_FINGER_MCP"),  # 0-5
                # ("INDEX_FINGER_MCP", "INDEX_FINGER_TIP"),  # special
                ("INDEX_FINGER_MCP", "INDEX_FINGER_PIP"),  # 5-6
                ("INDEX_FINGER_PIP", "INDEX_FINGER_DIP"),  # 6-7
                ("INDEX_FINGER_DIP", "INDEX_FINGER_TIP"),  # 7-8
                # Middle finger connections (0,9-12)
                ("WRIST", "MIDDLE_FINGER_MCP"),  # 0-9
                ("MIDDLE_FINGER_MCP", "MIDDLE_FINGER_PIP"),  # 9-10
                ("MIDDLE_FINGER_PIP", "MIDDLE_FINGER_DIP"),  # 10-11
                ("MIDDLE_FINGER_DIP", "MIDDLE_FINGER_TIP"),  # 11-12
                # Ring finger connections (0,13-16)
                ("WRIST", "RING_FINGER_MCP"),  # 0-13
                ("RING_FINGER_MCP", "RING_FINGER_PIP"),  # 13-14
                ("RING_FINGER_PIP", "RING_FINGER_DIP"),  # 14-15
                ("RING_FINGER_DIP", "RING_FINGER_TIP"),  # 15-16
                # Pinky finger connections (0,17-20)
                ("WRIST", "PINKY_MCP"),  # 0-17
                ("PINKY_MCP", "PINKY_PIP"),  # 17-18
                ("PINKY_PIP", "PINKY_DIP"),  # 18-19
                ("PINKY_DIP", "PINKY_TIP"),  # 19-20
                # Palm connections (across finger MCPs)
                ("INDEX_FINGER_MCP", "MIDDLE_FINGER_MCP"),  # 5-9
                ("MIDDLE_FINGER_MCP", "RING_FINGER_MCP"),  # 9-13
                ("RING_FINGER_MCP", "PINKY_MCP"),  # 13-17
            ]

            self.get_logger().debug(f"connections: {connections}")

            for connection in connections:
                start_joint, end_joint = connection
                if start_joint in joint_positions and end_joint in joint_positions:
                    line_marker = Marker()
                    line_marker.header.frame_id = "camera_depth_optical_frame"
                    line_marker.header.stamp = self.get_clock().now().to_msg()
                    line_marker.ns = "hand_connections"
                    line_marker.id = connections.index(connection)
                    line_marker.type = Marker.LINE_STRIP
                    line_marker.action = Marker.ADD
                    line_marker.scale.x = 0.005
                    line_marker.color.a = 1.0
                    line_marker.color.r = 0.0
                    line_marker.color.g = 1.0  # Green for connections
                    line_marker.color.b = 0.0

                    start_point = Point()
                    start_point.x = joint_positions[start_joint][0]
                    start_point.y = joint_positions[start_joint][1]
                    start_point.z = joint_positions[start_joint][2]

                    end_point = Point()
                    end_point.x = joint_positions[end_joint][0]
                    end_point.y = joint_positions[end_joint][1]
                    end_point.z = joint_positions[end_joint][2]

                    line_marker.points.append(start_point)
                    line_marker.points.append(end_point)

                    marker_array.markers.append(line_marker)

        self.get_logger().debug(f"Publishing {len(marker_array.markers)} markers")
        self.marker_pub.publish(marker_array)


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
