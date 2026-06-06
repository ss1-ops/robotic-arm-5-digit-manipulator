#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from aero_hand_open_msgs.msg import HandMocap

import cv2
import numpy as np
import mediapipe as mp


class WebcamMocap(Node):
    def __init__(self):
        super().__init__("webcam_mocap")
        self.get_logger().info("Webcam Mocap Node Initialized")

        self.declare_parameter("camera_idx", 0)
        self.declare_parameter("fps", 30.0)
        self.declare_parameter("ema_alpha", 0.7)
        self.declare_parameter('mediapipe_model_complexity', 0)
        self.declare_parameter('viz', True)

        camera_idx = self.get_parameter("camera_idx").value
        fps = self.get_parameter("fps").value
        model_complexity = self.get_parameter('mediapipe_model_complexity').value
        self.ema_alpha = self.get_parameter("ema_alpha").value
        self.viz = self.get_parameter('viz').value

        ## Mediapipe Hands init
        self.hands = mp.solutions.hands.Hands(
            model_complexity=model_complexity,
        )
        ## Video Capture
        self.cap = cv2.VideoCapture(camera_idx)
        if not self.cap.isOpened():
            self.get_logger().error(f"Could not open video device at {camera_idx}.")
            return
        
        ## Landmark cache for smoothing
        self.landmark_cache_right = np.zeros((25, 3))
        self.landmark_cache_left = np.zeros((25, 3))

        ## Pose Publisher
        self.mocap_pub = self.create_publisher(HandMocap, "webcam_mocap_data", 10)
        
        ## Timer for processing frames
        self.timer = self.create_timer(1.0/fps, self.timer_callback)

    def process_landmarks(self, landmarks, hand_label):
        """
        Center the landmarks around the wrist and align the hand coordinate system.
        """
        ## X-axis index - ring
        x_axis = landmarks[5] - landmarks[13]
        x_axis = x_axis / np.linalg.norm(x_axis)
        if hand_label == "left":
            x_axis = -x_axis

        ## Z-axis middle - wrist
        z_axis = landmarks[9] - landmarks[0]
        z_axis = z_axis / np.linalg.norm(z_axis)

        ## Y-axis
        y_axis = np.cross(z_axis, x_axis)
        y_axis = y_axis / np.linalg.norm(y_axis)

        ## Re-orthogonalize x_axis
        x_axis = np.cross(y_axis, z_axis)
        x_axis = x_axis / np.linalg.norm(x_axis)

        base_translation = landmarks[0]
        base_rot_mat = np.array([x_axis, y_axis, z_axis]).T

        ## Translate
        landmarks = landmarks - base_translation
        landmarks = landmarks @ base_rot_mat

        return landmarks
    
    def publish_mocap_data(self, landmarks, side):
        mocap_msg = HandMocap()
        mocap_msg.header.stamp = self.get_clock().now().to_msg()
        mocap_msg.side = side

        ## As mediapipe doesn't provide CMC landmarks for fingers we add wrist point as cmc
        landmarks = np.array([
            landmarks[0], landmarks[1], landmarks[2], landmarks[3], landmarks[4],       ## Thumb
            landmarks[0], landmarks[5], landmarks[6], landmarks[7], landmarks[8],       ## Index
            landmarks[0], landmarks[9], landmarks[10], landmarks[11], landmarks[12],    ## Middle
            landmarks[0], landmarks[13], landmarks[14], landmarks[15], landmarks[16],   ## Ring
            landmarks[0], landmarks[17], landmarks[18], landmarks[19], landmarks[20]    ## Pinky
        ])

        ## Apply EMA smoothing to landmarks
        if side == 'right':
            self.landmark_cache_right = self.ema_alpha * landmarks + (1 - self.ema_alpha) * self.landmark_cache_right
            landmarks = self.landmark_cache_right
        elif side == 'left':
            self.landmark_cache_left = self.ema_alpha * landmarks + (1 - self.ema_alpha) * self.landmark_cache_left
            landmarks = self.landmark_cache_left
        else:
            self.get_logger().warn(f"Unknown hand side: {side}")
            return

        ## Fill mocap message, with Pose for each landmark
        mocap_msg.keypoints = [
            Pose(
                position=Point(x=float(lm[0]), y=float(lm[1]), z=float(lm[2])),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0) ## Setting neutral orientation as mediapipe doesn't provide orientation
            ) for lm in landmarks
        ]
        self.mocap_pub.publish(mocap_msg)

    def timer_callback(self):
        ret, frame_bgr = self.cap.read()
        frame_bgr = cv2.flip(frame_bgr, 1)  # Flip for mirror view
        if not ret:
            self.get_logger().error("Could not read frame from video device.")
            return
    
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        results = self.hands.process(frame_rgb)

        if results.multi_hand_landmarks and results.multi_hand_world_landmarks and results.multi_handedness:
            for _, (landmarks, world_landmarks, handedness) in enumerate(zip(results.multi_hand_landmarks, results.multi_hand_world_landmarks, results.multi_handedness)):
                side = handedness.classification[0].label.lower()

                ## -ve x to remove the effect of mirroring
                world_landmarks_np = np.array([[-lm.x, lm.y, lm.z] for lm in world_landmarks.landmark])
                processed_landmarks = self.process_landmarks(world_landmarks_np, side)
                self.publish_mocap_data(processed_landmarks, side)

                # Draw landmarks on the frame
                if self.viz:
                    mp.solutions.drawing_utils.draw_landmarks(frame_bgr, landmarks, mp.solutions.hands.HAND_CONNECTIONS)

        if self.viz:
            cv2.imshow('Hand Pose', frame_bgr)
            if cv2.waitKey(1) & 0xFF == 27:
                pass


def main(args=None):
    rclpy.init(args=args)
    webcam_mocap = WebcamMocap()
    rclpy.spin(webcam_mocap)
    webcam_mocap.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
