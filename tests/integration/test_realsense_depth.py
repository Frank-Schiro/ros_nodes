import pytest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import statistics
import sensor_msgs_py.point_cloud2 as pc2
from threading import Event, Timer
import math
import os

print("=== Debug: Loading test file ===", flush=True)
print("Current working directory:", os.getcwd(), flush=True)
print("Directory contents:", os.listdir(), flush=True)


class DepthAnalyzer(Node):
    def __init__(self):
        super().__init__("depth_analyzer")

        # Configure QoS for depth data
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.received_data = Event()
        self.point_stats = None

        depth_profile = self.get_parameter_or(
            "depth_module.depth_profile", "848x480x30"
        )

        self.get_logger().info(f"Depth profile: {depth_profile}")

        self.depth_width, self.depth_height, self.depth_fps = self.parse_profile(
            depth_profile
        )

        self.get_logger().info(
            f"DepthAnalyzer initialized with {self.depth_width}x{self.depth_height} resolution"
        )

        # Subscribe to pointcloud
        # self.subscription = self.create_subscription(
        #     PointCloud2,
        #     "/camera/camera/depth/color/points",
        #     self.pointcloud_callback,
        #     qos,
        # )

        self.subscription = self.create_subscription(
            PointCloud2,
            "/camera/camera/depth/color/points",
            self.pointcloud_callback,
            qos_profile,
        )

    def parse_profile(self, profile):
        """Parse profile string like '848x480x30' into width, height, fps"""
        parts = profile.lower().replace(" ", "").split("x")
        return int(parts[0]), int(parts[1]), int(parts[2])

    def pointcloud_callback(self, msg):
        """Process incoming point cloud and analyze depth patterns"""
        self.get_logger().info("pointcloud_callback")

        if msg.width == 0 or msg.height == 0:
            self.get_logger().warn("Received empty pointcloud")
            return

        # Read pointcloud as numpy array and reshape to 2D image
        points = pc2.read_points_numpy(
            msg, field_names=["x", "y", "z"], skip_nans=False
        ).reshape(self.depth_height, self.depth_width, 3)

        # Now points[row, col] gives us x,y,z at that pixel location
        # Convert pixel regions to actual regions in space

        # Center region (middle 10% of image)
        center_h = self.depth_height // 2
        center_w = self.depth_width // 2
        region_h = self.depth_height // 10
        region_w = self.depth_width // 10

        center_region = points[
            center_h - region_h // 2 : center_h + region_h // 2,
            center_w - region_w // 2 : center_w + region_w // 2,
        ]

        # Corner regions (10% squares in corners)
        top_left_region = points[:region_h, :region_w]  # Top rows  # Left columns

        top_right_region = points[:region_h, -region_w:]  # Top rows  # Right columns

        bottom_left_region = points[
            -region_h:, :region_w  # Bottom rows  # Left columns
        ]

        bottom_right_region = points[
            -region_h:, -region_w:  # Bottom rows  # Right columns
        ]

        # Filter out invalid points (too close/far) and calculate stats
        def get_region_stats(region):
            points_list = region.reshape(-1, 3)
            valid_points = points_list[
                (points_list[:, 2] > 0.1) & (points_list[:, 2] < 2.0)
            ]
            if len(valid_points) == 0:
                return {"mean": 0, "std": 0, "count": 0, "min": 0, "max": 0}
            return {
                "mean": np.mean(valid_points[:, 2]),
                "std": np.std(valid_points[:, 2]),
                "count": len(valid_points),
                "min": np.min(valid_points[:, 2]),
                "max": np.max(valid_points[:, 2]),
            }

        self.point_stats = {
            "center": get_region_stats(center_region),
            "corners": {
                "top_left": get_region_stats(top_left_region),
                "top_right": get_region_stats(top_right_region),
                "bottom_left": get_region_stats(bottom_left_region),
                "bottom_right": get_region_stats(bottom_right_region),
            },
            "overall": get_region_stats(points),
        }

        self.received_data.set()


import threading
import time


def test_wall_depth():
    """
    Test depth readings for a flat wall.

    Setup:
    1. Place camera 1 meter from a flat wall
    2. Wall should be perpendicular to camera's z-axis
    3. Wall should be plain (not textured) and non-reflective
    4. Lighting should be consistent
    """
    EXPECTED_DISTANCE = 1.0  # 1 meter from wall
    DISTANCE_TOLERANCE = 0.05  # ±5cm tolerance for depth
    MAX_STD_DEV = 0.02  # Maximum allowed standard deviation (2cm)
    MAX_CORNER_DIFFERENCE = 0.10  # Maximum allowed difference between corners (10cm)
    MIN_VALID_POINTS = 1000  # Minimum number of valid points expected

    # Initialize ROS and create analyzer node
    rclpy.init()
    analyzer = DepthAnalyzer()

    # Run the ROS2 event loop in a separate thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(analyzer,), daemon=True)
    spin_thread.start()

    try:
        # Give some time for the node to start and receive data
        time.sleep(2)

        if not analyzer.received_data.wait(timeout=10.0):
            pytest.fail("Timeout waiting for pointcloud data")

        stats = analyzer.point_stats

        # Basic data validity checks
        assert (
            stats["overall"]["count"] >= 1000
        ), f"Too few valid points: {stats['overall']['count']}"
        assert (
            abs(stats["center"]["mean"] - 1.0) <= 0.05
        ), f"Center depth {stats['center']['mean']:.3f}m not within tolerance"
        assert (
            stats["center"]["std"] <= MAX_STD_DEV
        ), f"Center depth variation too high: {stats['center']['std']:.3f}m"

        # Compare corner depths
        corner_depths = [
            stats["corners"]["top_left"]["mean"],
            stats["corners"]["top_right"]["mean"],
            stats["corners"]["bottom_left"]["mean"],
            stats["corners"]["bottom_right"]["mean"],
        ]

        max_corner_diff = max(corner_depths) - min(corner_depths)
        assert (
            max_corner_diff <= MAX_CORNER_DIFFERENCE
        ), f"Corner depth difference too high: {max_corner_diff:.3f}m"

        # Verify corner depths are slightly larger than center
        # (because wall is flat but we're viewing in a cone)
        for corner, data in stats["corners"].items():
            assert (
                data["mean"] >= stats["center"]["mean"]
            ), f"{corner} depth ({data['mean']:.3f}m) unexpectedly less than center ({stats['center']['mean']:.3f}m)"

        # Print detailed analysis
        print("\nDepth Analysis Results:")
        print(
            f"Overall depth range: {stats['overall']['min']:.3f}m to {stats['overall']['max']:.3f}m"
        )
        print(
            f"Center depth: {stats['center']['mean']:.3f}m ± {stats['center']['std']:.3f}m"
        )
        print("\nCorner Depths:")
        for corner, data in stats["corners"].items():
            print(f"{corner}: {data['mean']:.3f}m ± {data['std']:.3f}m")

    finally:
        analyzer.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1)  # Stop the thread cleanly

        # Check depth consistency (standard deviation)
