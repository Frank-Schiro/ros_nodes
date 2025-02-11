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
from vision_interfaces.msg import HandLandmark, HandLandmarkArray


class ImagePointTo3D(Node):
    """ROS node that combines 2D hand landmarks with depth data to get 3D hand positions"""

    def __init__(self):
        super().__init__("hand_3d_tracking_node")

        # self.hand_landmarks_sub_debug = self.create_subscription(
        #     HandLandmarkArray,
        #     "hand_landmarks",
        #     self.simple_callback,  # New direct callback
        #     10,
        # )

        # Set up QoS profile for point cloud subscription
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # self.pointcloud_sub_debug = self.create_subscription(
        #     PointCloud2,
        #     "/camera/camera/depth/color/points",
        #     self.debug_pointcloud_callback,
        #     qos_profile,  # Use same QoS profile
        # )

        self.camera_info_sub = self.create_subscription(
            CameraInfo, "/camera/camera/color/camera_info", self.camera_info_callback, 1
        )
        self.image_width = None
        self.image_height = None

        # Create synchronized subscribers
        # self.hand_landmarks_sub = Subscriber(self, String, "hand_landmarks_json")
        self.hand_landmarks_sub = Subscriber(self, HandLandmarkArray, "hand_landmarks")

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
            [self.hand_landmarks_sub, self.pointcloud_sub],
            queue_size=60,  # Buffer up to 2 seconds of frames
            slop=0.1,  # 100ms time difference tolerance
        )

        # Add some debugging
        self.last_msg_times = {"hand": None, "cloud": None}
        self.create_timer(5.0, self.check_sync_status)  # Check every 5 seconds
        self.ts.registerCallback(self.sync_callback)

        # Create publisher for 3D hand landmarks
        self.hand_3d_pub = self.create_publisher(String, "hand_landmarks_3d", 10)

        self.get_logger().info("Hand3DTrackingNode initialized")

    def camera_info_callback(self, msg):
        """Store camera dimensions from info message"""
        if self.image_width is None:
            self.image_width = msg.width
            self.image_height = msg.height
            self.get_logger().info(
                f"Got camera dimensions: {self.image_width}x{self.image_height}"
            )

    # def debug_pointcloud_callback(self, msg):
    #     self.get_logger().info(f"Received pointcloud with stamp: {msg.header.stamp}")

    # def simple_callback(self, msg):
    #     """Simple callback just to verify message receipt"""
    #     self.get_logger().info(
    #         f"Received hand landmark message for hand: {msg.hand_id}"
    #     )

    def check_sync_status(self):
        """Monitor synchronization status and report issues"""
        # self.get_logger().info(f"Queue size: {self.ts.queue_size}")
        # self.get_logger().info(f"Slop: {self.ts.slop}")
        if None not in self.last_msg_times.values():
            time_diff = abs(self.last_msg_times["hand"] - self.last_msg_times["cloud"])
            # self.get_logger().info(
            #     f"Sync status: time difference = {time_diff*1000:.1f}ms "
            #     f"(target < {self.ts.slop*1000}ms)"
            # )

    def get_point_from_cloud(self, point_cloud, pixel_x: int, pixel_y: int) -> tuple:
        """Extract 3D point from point cloud at given pixel coordinates"""
        try:
            # Debug the point cloud structure
            # self.get_logger().info(f"\nPointCloud2 internal dimensions:")
            # self.get_logger().info(f"- Cloud width: {point_cloud.width}")
            # self.get_logger().info(f"- Cloud height: {point_cloud.height}")
            # self.get_logger().info(f"- Is dense: {point_cloud.is_dense}")

            # self.get_logger().info(
            #     f"- Fields: {[field.name for field in point_cloud.fields]}"
            # )
            # self.get_logger().info(f"- Point step: {point_cloud.point_step}")
            # self.get_logger().info(f"- Row step: {point_cloud.row_step}")
            # self.get_logger().info(f"- Data size: {len(point_cloud.data)}")

            # Get first few points to understand structure
            points = list(
                pc2.read_points(
                    point_cloud, field_names=["x", "y", "z"], skip_nans=False
                )
            )

            # if len(points) > 0:
            # self.get_logger().info(f"First point: {points[0]}")
            # self.get_logger().info(f"Total points: {len(points)}")
            # self.get_logger().info(f"Type of first point: {type(points[0])}")
            # if hasattr(points[0], "__len__"):
            #     self.get_logger().info(f"Length of first point: {len(points[0])}")

            # Calculate target index
            if self.image_width:
                index = pixel_y * self.image_width + pixel_x
                # self.get_logger().info(
                #     f"Calculated index: {index} for pixel ({pixel_x}, {pixel_y})"
                # )

            if points and not (
                np.isnan(points[0][0])
                or np.isnan(points[0][1])
                or np.isnan(points[0][2])
            ):
                # self.get_logger().info(
                #     f"Returning points: {points[0]}, {points[1]}, {points[2]}"
                # )
                return (points[0][0], points[0][1], points[0][2])
            else:
                self.get_logger().info(f"Returning No 3d points")
            return None

        except Exception as e:
            import traceback

            self.get_logger().warning(
                f"Error getting point from cloud: {str(e)}\n"
                f"Traceback: {traceback.format_exc()}"
            )
            return None

    def sync_callback(self, hand_msg: HandLandmarkArray, cloud_msg: PointCloud2):
        """Handle synchronized hand landmarks and point cloud messages"""
        try:
            # Create output dictionary with header info
            output_data = {
                "header": {
                    "stamp": {
                        "sec": hand_msg.header.stamp.sec,
                        "nanosec": hand_msg.header.stamp.nanosec,
                    },
                    "frame_id": hand_msg.header.frame_id,
                },
                "landmarks": {},
            }

            # We're receiving one hand per message in HandLandmarkArray
            output_data["landmarks"][hand_msg.hand_id] = {}

            # Process each landmark in the hand
            for landmark in hand_msg.landmarks:
                # Convert normalized coordinates to pixel coordinates
                pixel_x, pixel_y = landmark.x, landmark.y
                # self.get_logger().info(
                #     f"Processing landmark {landmark.landmark_name} at pixel ({pixel_x}, {pixel_y})"
                # )
                # Get 3D point from point cloud
                point_3d = self.get_point_from_cloud(cloud_msg, pixel_x, pixel_y)
                if point_3d is not None:
                    output_data["landmarks"][hand_msg.hand_id][
                        landmark.landmark_name
                    ] = {
                        "point3d": {
                            "x": float(point_3d[0]),
                            "y": float(point_3d[1]),
                            "z": float(point_3d[2]),
                        }
                    }
                # self.get_logger().info(f"3D point: {point_3d}")
                # Store results for this landmark
                # output_data["landmarks"][hand_msg.hand_id][landmark.landmark_name] = {
                #     "pixel": {"x": pixel_x, "y": pixel_y},
                #     "normalized": {"x": landmark.x, "y": landmark.y},
                # }

                # self.get_logger().info(f"Landmark data: {output_data}")

                # if point_3d is not None:
                #     # self.get_logger().info(f"building output data")
                #     output_data["landmarks"][hand_msg.hand_id][landmark.landmark_name][
                #         "point3d"
                #     ] = {
                #         "x": float(point_3d[0]),
                #         "y": float(point_3d[1]),
                #         "z": float(point_3d[2]),
                #     }
                # else:
                #     self.get_logger().warning(
                #         f"No 3D point found for {landmark.landmark_name} at ({pixel_x}, {pixel_y})"
                #     )

            # Publish results
            # self.get_logger().info(f"output_data: {output_data}")
            output_msg = String()
            output_msg.data = json.dumps(output_data)
            self.get_logger().info(f"Publishing 3D hand landmarks: {output_msg.data}")
            self.hand_3d_pub.publish(output_msg)
            self.get_logger().info("Published to hand_landmarks_3d topic")

            # Log if hands were detected
            if len(output_data["landmarks"]) > 0:
                self.get_logger().info("Published 3D hand landmarks")
                self.get_logger().debug(f"3D Landmarks JSON: {output_msg.data}")

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
