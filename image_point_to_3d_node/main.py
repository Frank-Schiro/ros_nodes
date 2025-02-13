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
    Hand2D,
    Hand3D,
    HandLandmark3D,
)

print("A")
from vision_interfaces.msg import Hand2D
import transforms3d as t3d
import transforms3d.axangles
import transforms3d.quaternions

print("B")


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

        self.image_width = 0
        self.image_height = 0

        # Create synchronized subscribers
        self.hand_detection_sub = Subscriber(self, HandDetection2D, "hand_detection_2d")

        self.pointcloud_sub = Subscriber(
            self,
            PointCloud2,
            "/camera/camera/depth/color/points",
            qos_profile=qos_profile,
        )

        # self.pointcloud_sub = Subscriber(
        #     self,
        #     PointCloud2,
        #     "/camera/camera/depth/color/points",
        #     self.point_cloud_callback,
        # )
        # self.hand_detection_sub = Subscriber(
        #     self, HandDetection2D, "hand_detection_2d", self.hand_detection_callback
        # )

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
            self.get_logger().info("Checking synchronization status")
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

            self.get_logger().info(f"Color image size: {image_width}x{image_height}")
            self.get_logger().info(f"Point cloud size: {points.shape}")
            self.get_logger().info(f"pixel_y, pixel_x: {pixel_y} {pixel_x}")
            self.get_logger().info(f"fuck docker ")
            # Get the corresponding point from the flattened array
            # Sometimes mediapipe predicts points outside of image bounds
            # We should interpolate in future for now we clip
            pixel_x = min(pixel_x, image_width - 1)
            pixel_y = min(pixel_y, image_height - 1)

            point = points[pixel_y, pixel_x]

            self.get_logger().info(f"point: {point}")

            if np.any(np.isnan(point)):
                self.get_logger().debug(
                    f"NaN values found at pixel ({pixel_x}, {pixel_y})"
                )
                return None

            return tuple(point)  # Convert numpy array to tuple

        except Exception as e:
            self.get_logger().warning(
                f"Error getting point from cloud at pixel ({pixel_x}, {pixel_y}): {str(e)}\nPoint cloud frame_id: {point_cloud.header.frame_id}"
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

    # def get_joint_frame(self, p1: np.ndarray, p2: np.ndarray) -> np.ndarray:
    #     """
    #     Create frame quaternion using direction to next joint and camera up

    #     Args:
    #         p1: Current joint position (e.g., WRIST, MCP, PIP, DIP)
    #         p2: Next joint position (e.g., MCP, PIP, DIP, TIP)

    #     Returns:
    #         Quaternion representing joint frame orientation
    #     """
    #     # Forward vector to next joint
    #     forward = p2 - p1
    #     forward = forward / np.linalg.norm(forward)

    #     # Use camera up as reference
    #     camera_up = np.array([0, 0, 1])

    #     # transforms3d gives us rotation matrix aligning x with forward
    #     # and trying to align z with camera_up
    #     rot_mat = t3d.axangles.axangle2mat(forward, camera_up)

    #     # Convert to quaternion [w, x, y, z]
    #     return t3d.quaternions.mat2quat(rot_mat)

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

    def process_joints(self, hand_2d: Hand2D, cloud_msg: PointCloud2) -> Hand3D:
        """Convert 2D hand detection to 3D with joint quaternions for relevant landmarks"""
        # Create Hand3D message
        hand_3d = Hand3D()
        hand_3d.hand_id = hand_2d.hand_id

        # First pass: get all 3D points
        points_3d = {}
        for landmark in hand_2d.landmarks:
            point_3d = self.get_point_from_cloud(
                cloud_msg,
                landmark.pixel_x,
                landmark.pixel_y,
                self.image_width,
                self.image_height,
            )
            if point_3d is not None:
                points_3d[landmark.name] = np.array(point_3d)

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

        # Process each landmark
        for landmark in hand_2d.landmarks:
            name = landmark.name

            # Skip if we don't have 3D point for this landmark
            if name not in points_3d:
                continue

            landmark_3d = HandLandmark3D()
            landmark_3d.name = name

            # 1. Set position (x, y, z)
            p = points_3d[name]

            self.get_logger().info(
                f"{name} point_3d type: {type(p)}, shape: {p.shape}, dtype: {p.dtype}, values: {p}"
            )

            self.get_logger().info(f"p: {p}")
            self.get_logger().info(f"p[0]: {p[0]}")
            self.get_logger().info(f"p[1]: {p[1]}")
            self.get_logger().info(f"p[2]: {p[2]}")

            landmark_3d.position.x = float(p[0])
            landmark_3d.position.y = float(p[1])
            landmark_3d.position.z = float(p[2])
            # Special handling for wrist - calculate camera-relative quaternion
            if name == "WRIST":
                wrist_quat = self.get_wrist_quaternion(p)
                self.get_logger().info(f"wrist_quat={wrist_quat}")
                self.get_logger().info(f"wrist_quat[0]: {wrist_quat[0]}")
                self.get_logger().info(f"wrist_quat[1]: {wrist_quat[1]}")
                self.get_logger().info(f"wrist_quat[2]: {wrist_quat[2]}")
                self.get_logger().info(f"wrist_quat[3]: {wrist_quat[3]}")

                landmark_3d.orientation.w = float(wrist_quat[0])
                landmark_3d.orientation.x = float(wrist_quat[1])
                landmark_3d.orientation.y = float(wrist_quat[2])
                landmark_3d.orientation.z = float(wrist_quat[3])

            else:

                # 2. Set orientation if this is a joint
                if name in joint_connections:
                    self.get_logger().info(f"joint_connections: {joint_connections}")
                    prev_point_name, next_point_name = joint_connections[name]
                    self.get_logger().info(f"prev_point_name: {prev_point_name}")
                    self.get_logger().info(f"next_point_name: {next_point_name}")

                    # Only calculate orientation if we have both reference points
                    if prev_point_name in points_3d and next_point_name in points_3d:
                        self.get_logger().info(
                            f"points_3d[prev_point_name]: {points_3d[prev_point_name]}"
                        )
                        self.get_logger().info(
                            f"points_3d[next_point_name]: {points_3d[next_point_name]}"
                        )
                        # Get joint frame quaternion
                        frame_quat = self.get_joint_frame(
                            points_3d[prev_point_name], points_3d[next_point_name]
                        )
                        self.get_logger().info(f"frame_quat={frame_quat}")
                        self.get_logger().info(f"frame_quat[0]: {frame_quat[0]}")
                        self.get_logger().info(f"frame_quat[1]: {frame_quat[1]}")
                        self.get_logger().info(f"frame_quat[2]: {frame_quat[2]}")
                        self.get_logger().info(f"frame_quat[3]: {frame_quat[3]}")

                        # Convert from [w,x,y,z] to ROS quaternion [x,y,z,w]
                        landmark_3d.orientation.w = float(frame_quat[0])
                        landmark_3d.orientation.x = float(frame_quat[1])
                        landmark_3d.orientation.y = float(frame_quat[2])
                        landmark_3d.orientation.z = float(frame_quat[3])

            hand_3d.landmarks.append(landmark_3d)

        # if hand_3d.landmarks:
        #     palm_landmarks = [
        #         lm
        #         for lm in hand_3d.landmarks
        #         if lm.name in ["WRIST", "INDEX_FINGER_MCP", "PINKY_MCP"]
        #     ]

        #     if palm_landmarks:
        #         # Calculate average position for palm center
        #         palm_pos = np.mean(
        #             [
        #                 np.array([lm.position.x, lm.position.y, lm.position.z])
        #                 for lm in palm_landmarks
        #             ],
        #             axis=0,
        #         )

        #         # Set palm pose
        #         hand_3d.palm_pose.position.x = float(palm_pos[0])
        #         hand_3d.palm_pose.position.y = float(palm_pos[1])
        #         hand_3d.palm_pose.position.z = float(palm_pos[2])

        #         # Calculate palm orientation using wrist to MCP direction
        #         wrist_pos = next(
        #             np.array([lm.position.x, lm.position.y, lm.position.z])
        #             for lm in hand_3d.landmarks
        #             if lm.name == "WRIST"
        #         )
        #         mcp_pos = next(
        #             np.array([lm.position.x, lm.position.y, lm.position.z])
        #             for lm in hand_3d.landmarks
        #             if lm.name == "MIDDLE_FINGER_MCP"
        #         )

        #         # Get palm orientation using joint frame calculation
        #         palm_quat = self.get_joint_frame(wrist_pos, mcp_pos)
        #         hand_3d.palm_pose.orientation.w = float(palm_quat[0])
        #         hand_3d.palm_pose.orientation.x = float(palm_quat[1])
        #         hand_3d.palm_pose.orientation.y = float(palm_quat[2])
        #         hand_3d.palm_pose.orientation.z = float(palm_quat[3])

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

    def sync_callback(
        self, hand_detection_msg: HandDetection2D, cloud_msg: PointCloud2
    ):
        """Handle synchronized hand landmarks and point cloud messages"""
        try:
            self.get_logger().info("A")
            # Create 3D detection message
            detection_3d = HandDetection3D()
            detection_3d.header.stamp = hand_detection_msg.header.stamp
            detection_3d.header.frame_id = (
                "camera_link"  # 3D data is in camera_link frame
            )
            self.image_width = hand_detection_msg.image_width
            self.image_height = hand_detection_msg.image_height
            self.get_logger().info("B")
            # Process each hand
            for hand_2d in hand_detection_msg.hands:
                try:
                    self.get_logger().info("C")
                    hand_3d = self.process_joints(hand_2d, cloud_msg)
                    self.get_logger().info("D")
                    detection_3d.hands.append(hand_3d)

                except Exception as e:
                    self.get_logger().warning(
                        f"Error processing hand {hand_2d.hand_id}: {str(e)}"
                    )
                    continue

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
