#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from vision_interfaces.msg import HandDetection3D
import numpy as np
from transforms3d import quaternions
import math


class HandJointStatePublisher(Node):
    def __init__(self):
        super().__init__("hand_joint_state_publisher")

        # Subscribe to 3D hand detections
        self.create_subscription(
            HandDetection3D, "hand_detection_3d", self.hand_detection_callback, 10
        )

        # Publisher for joint states
        self.joint_pub = self.create_publisher(JointState, "hand_joint_states", 10)

        # Previous joint positions for basic velocity calculation
        self.prev_positions = None
        self.prev_time = None

        # Define the joint names we want to control
        # These should match your robot's URDF
        self.joint_names = [
            "thumb_joint",  # Base thumb rotation
            "thumb_proximal_joint",  # First thumb joint
            "thumb_distal_joint",  # Second thumb joint
            "index_proximal_joint",
            "index_middle_joint",
            "index_distal_joint",
            "middle_proximal_joint",
            "middle_middle_joint",
            "middle_distal_joint",
            "ring_proximal_joint",
            "ring_middle_joint",
            "ring_distal_joint",
            "pinky_proximal_joint",
            "pinky_middle_joint",
            "pinky_distal_joint",
        ]

    def calculate_finger_angles(self, hand):
        """Calculate joint angles for each finger from landmark positions and orientations."""
        angles = []

        # Helper function to calculate angle between vectors
        def angle_between_vectors(v1, v2):
            v1_u = v1 / np.linalg.norm(v1)
            v2_u = v2 / np.linalg.norm(v2)
            return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

        # Get landmarks dictionary for easier access
        landmarks = {lm.name: lm for lm in hand.landmarks}

        # Calculate angles for each finger
        for finger in [
            "THUMB",
            "INDEX_FINGER",
            "MIDDLE_FINGER",
            "RING_FINGER",
            "PINKY",
        ]:
            if finger == "THUMB":
                # Thumb base rotation (from palm)
                if all(
                    name in landmarks for name in ["WRIST", "THUMB_CMC", "THUMB_MCP"]
                ):
                    wrist = np.array(
                        [
                            landmarks["WRIST"].position.x,
                            landmarks["WRIST"].position.y,
                            landmarks["WRIST"].position.z,
                        ]
                    )
                    cmc = np.array(
                        [
                            landmarks["THUMB_CMC"].position.x,
                            landmarks["THUMB_CMC"].position.y,
                            landmarks["THUMB_CMC"].position.z,
                        ]
                    )
                    mcp = np.array(
                        [
                            landmarks["THUMB_MCP"].position.x,
                            landmarks["THUMB_MCP"].position.y,
                            landmarks["THUMB_MCP"].position.z,
                        ]
                    )

                    # Calculate base angle
                    palm_vector = cmc - wrist
                    thumb_vector = mcp - cmc
                    base_angle = angle_between_vectors(palm_vector, thumb_vector)
                    angles.append(base_angle)

                    # Calculate proximal and distal angles
                    if "THUMB_IP" in landmarks and "THUMB_TIP" in landmarks:
                        ip = np.array(
                            [
                                landmarks["THUMB_IP"].position.x,
                                landmarks["THUMB_IP"].position.y,
                                landmarks["THUMB_IP"].position.z,
                            ]
                        )
                        tip = np.array(
                            [
                                landmarks["THUMB_TIP"].position.x,
                                landmarks["THUMB_TIP"].position.y,
                                landmarks["THUMB_TIP"].position.z,
                            ]
                        )

                        proximal_vector = ip - mcp
                        distal_vector = tip - ip

                        proximal_angle = angle_between_vectors(
                            thumb_vector, proximal_vector
                        )
                        distal_angle = angle_between_vectors(
                            proximal_vector, distal_vector
                        )

                        angles.extend([proximal_angle, distal_angle])
            else:
                # Other fingers
                base = f"{finger}_MCP"
                mid = f"{finger}_PIP"
                tip = f"{finger}_DIP"
                end = f"{finger}_TIP"

                if all(name in landmarks for name in [base, mid, tip, end]):
                    p1 = np.array(
                        [
                            landmarks[base].position.x,
                            landmarks[base].position.y,
                            landmarks[base].position.z,
                        ]
                    )
                    p2 = np.array(
                        [
                            landmarks[mid].position.x,
                            landmarks[mid].position.y,
                            landmarks[mid].position.z,
                        ]
                    )
                    p3 = np.array(
                        [
                            landmarks[tip].position.x,
                            landmarks[tip].position.y,
                            landmarks[tip].position.z,
                        ]
                    )
                    p4 = np.array(
                        [
                            landmarks[end].position.x,
                            landmarks[end].position.y,
                            landmarks[end].position.z,
                        ]
                    )

                    v1 = p2 - p1
                    v2 = p3 - p2
                    v3 = p4 - p3

                    proximal_angle = angle_between_vectors(v1, v2)
                    middle_angle = angle_between_vectors(v2, v3)
                    distal_angle = (
                        middle_angle * 0.67
                    )  # Typically distal joint moves about 2/3 as much

                    angles.extend([proximal_angle, middle_angle, distal_angle])

        return angles

    def get_constant_velocities(self, num_joints):
        """Return constant velocities for all joints."""
        CONSTANT_VELOCITY = 1.0  # radians per second, adjust as needed
        return [CONSTANT_VELOCITY] * num_joints

    def hand_detection_callback(self, msg):
        try:
            # Process only the first hand for now
            if not msg.hands:
                return

            hand = msg.hands[0]  # Process first hand

            # Calculate joint angles
            joint_angles = self.calculate_finger_angles(hand)

            # Use constant velocities
            velocities = self.get_constant_velocities(len(joint_angles))

            # Create and publish joint state message
            joint_state_msg = JointState()
            joint_state_msg.header = msg.header
            joint_state_msg.name = self.joint_names
            joint_state_msg.position = joint_angles
            joint_state_msg.velocity = velocities
            # We're not calculating effort, so leave it empty

            self.joint_pub.publish(joint_state_msg)

        except Exception as e:
            self.get_logger().error(f"Error processing hand detection: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = HandJointStatePublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
