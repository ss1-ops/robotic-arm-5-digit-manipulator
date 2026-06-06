#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from aero_hand_open_msgs.msg import JointControl, HandMocap

import numpy as np
from aero_open_sdk.aero_hand_constants import AeroHandConstants


class MediapipeRetargeting(Node):
    def __init__(self):
        super().__init__("mediapipe_retargeting")

        ## Subscribe to Hand Mocap data
        self.hand_mocap_sub = self.create_subscription(
            HandMocap, "webcam_mocap_data", self.hand_mocap_callback, 10
        )

        self.joint_names = AeroHandConstants.joint_names

        ## Joint States publisher
        self.joint_states_right_pub = self.create_publisher(
            JointControl, "right/joint_control", 10
        )
        self.joint_states_left_pub = self.create_publisher(
            JointControl, "left/joint_control", 10
        )

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
        cmc_flex = np.pi - self._angle_between_three_points(
            thumb_landmarks[0], thumb_landmarks[1], thumb_landmarks[2]
        )
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

    def publish_joint_states(self, side: str, joint_values: list):
        js_msg = JointControl()
        js_msg.header.stamp = self.get_clock().now().to_msg()
        js_msg.target_positions = joint_values
        if side == "right":
            self.joint_states_right_pub.publish(js_msg)
        else:
            self.joint_states_left_pub.publish(js_msg)

    def hand_mocap_callback(self, msg: HandMocap):
        hand_side = msg.side
        pose_array = msg.keypoints
        landmarks = np.array(
            [[lm.position.x, lm.position.y, lm.position.z] for lm in pose_array]
        )
        joint_angles = self.retarget_landmarks(landmarks)
        self.publish_joint_states(hand_side, joint_angles)


def main():
    rclpy.init()
    mediapipe_retargeting = MediapipeRetargeting()
    rclpy.spin(mediapipe_retargeting)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
