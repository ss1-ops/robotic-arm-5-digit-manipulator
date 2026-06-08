#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from aero_hand_open_msgs.msg import JointControl, HandMocap
from aero_hand_open_retargeting.utils.normalize import normalize_joint_state
from aero_hand_open_retargeting.utils.load_normalize_config import load_normalize_config

import numpy as np
from aero_open_sdk.aero_hand_constants import AeroHandConstants


class AppleVisionProRetargeting(Node):
    def __init__(self):
        super().__init__("apple_vision_pro_retargeting")

        ## Subscribe to Hand Mocap data
        self.hand_mocap_sub = self.create_subscription(
            HandMocap, "mocap_data", self.hand_mocap_callback, 10
        )

        self.right_joint_names = [f"right_{name}" for name in AeroHandConstants.joint_names]
        self.left_joint_names = [f"left_{name}" for name in AeroHandConstants.joint_names]

        self.normalize_config = load_normalize_config("default_vision_pro")

        self.right_mins = np.ones(16) * 1000
        self.right_maxs = np.zeros(16)

        self.joint_control_pub_right = self.create_publisher(JointControl, "right/joint_control", 10)
        self.joint_control_pub_left = self.create_publisher(JointControl, "left/joint_control", 10)

        self.joint_states_pub_right = self.create_publisher(JointState, "right/joint_states", 10)
        self.joint_states_pub_left = self.create_publisher(JointState, "left/joint_states", 10)

        self.vision_pro_markers_pub_right = self.create_publisher(Marker, "right/vision_pro_markers", 10)
        self.vision_pro_markers_pub_left = self.create_publisher(Marker, "left/vision_pro_markers", 10)

    @staticmethod
    def _angle_between_three_points(a, b, c):
        ba = a - b
        bc = c - b

        cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
        angle = np.arccos(cosine_angle)
        return angle

    def _get_finger_joints(self, finger_landmarks):
        mcp = np.pi - self._angle_between_three_points(
            finger_landmarks[0], finger_landmarks[1], finger_landmarks[2]
        )
        pip = np.pi - self._angle_between_three_points(
            finger_landmarks[1], finger_landmarks[2], finger_landmarks[3]
        )
        dip = np.pi - self._angle_between_three_points(
            finger_landmarks[2], finger_landmarks[3], finger_landmarks[4]
        )
        return [mcp, pip, dip]

    def _get_thumb_joints(self, thumb_landmarks):
        ## CMC Abduction, angle of thumb mcp from the vertical plane
        a = np.array([thumb_landmarks[2][0], thumb_landmarks[1][1], 0])
        b = np.array([thumb_landmarks[1][0], thumb_landmarks[1][1], 0])
        c = np.array([thumb_landmarks[2][0], thumb_landmarks[2][1], 0])
        cmc_abd = self._angle_between_three_points(a, b, c)

        ## CMC Flexion
        a = np.array([thumb_landmarks[1][0], thumb_landmarks[1][1], thumb_landmarks[1][2] + 1])
        b = thumb_landmarks[1]
        c = thumb_landmarks[2]
        cmc_flex = abs(self._angle_between_three_points(a, b, c))

        cmc_flex = np.pi/3 - cmc_flex  # Adjusting to AeroHand
        mcp_flex = np.pi - self._angle_between_three_points(
            thumb_landmarks[1], thumb_landmarks[2], thumb_landmarks[3]
        )
        ip_flex = np.pi - self._angle_between_three_points(
            thumb_landmarks[2], thumb_landmarks[3], thumb_landmarks[4]
        )
        return [cmc_abd, cmc_flex, mcp_flex, ip_flex]

    def retarget_landmarks(self, landmarks):
        thumb_joints = self._get_thumb_joints(landmarks[0:5])
        index_joints = self._get_finger_joints(landmarks[5:10])
        middle_joints = self._get_finger_joints(landmarks[10:15])
        ring_joints = self._get_finger_joints(landmarks[15:20])
        pinky_joints = self._get_finger_joints(landmarks[20:25])
        return thumb_joints + index_joints + middle_joints + ring_joints + pinky_joints

    def publish_vision_pro_markers(self, landmarks: np.ndarray, hand_side: str):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = f"{hand_side}_base_link"
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD

        # Uniform sphere size
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01

        # Bright green color
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        # Add all Vision Pro landmarks as points
        for lm in landmarks:
            p = Point(x=float(lm[0]), y=float(lm[1]), z=float(lm[2]))
            marker.points.append(p)

        # Publish to appropriate topic
        if hand_side == "right":
            self.vision_pro_markers_pub_right.publish(marker)
        elif hand_side == "left":
            self.vision_pro_markers_pub_left.publish(marker)

    def publish_joint_controls(self, joint_values: list, hand_side: str):
        joint_control_msg = JointControl()
        joint_control_msg.header.stamp = self.get_clock().now().to_msg()
        joint_control_msg.target_positions = joint_values

        if hand_side == "right":
            self.joint_control_pub_right.publish(joint_control_msg)
        elif hand_side == "left":
            self.joint_control_pub_left.publish(joint_control_msg)

    def publish_joint_states(self, joint_values: list, hand_side: str):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.position = joint_values

        if hand_side == "right":
            joint_state_msg.name = self.right_joint_names
            self.joint_states_pub_right.publish(joint_state_msg)
        elif hand_side == "left":
            joint_state_msg.name = self.left_joint_names
            self.joint_states_pub_left.publish(joint_state_msg)

    def hand_mocap_callback(self, msg: HandMocap):
        hand_side = msg.side
        pose_array = msg.keypoints
        landmarks = np.array(
            [[lm.position.x, lm.position.y, lm.position.z] for lm in pose_array]
        )
        ## Clamp the positive y values to zero
        landmarks[:, 1] = np.clip(landmarks[:, 1], a_min=None, a_max=0.0)

        joint_angles = self.retarget_landmarks(landmarks)
        if hand_side == "left":
            joint_angles = [-angle for angle in joint_angles]
        joint_angles = np.clip(
            joint_angles, AeroHandConstants.joint_lower_limits, AeroHandConstants.joint_upper_limits
        ).tolist()

        for i in range(16):
            joint_angles[i] = normalize_joint_state(
                joint_angles[i], i, self.normalize_config
            )

        self.publish_joint_controls(joint_angles, hand_side)
        self.publish_joint_states(joint_angles, hand_side)
        self.publish_vision_pro_markers(landmarks, hand_side)


def main():
    rclpy.init()
    apple_vision_pro_retargeting = AppleVisionProRetargeting()
    rclpy.spin(apple_vision_pro_retargeting)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
