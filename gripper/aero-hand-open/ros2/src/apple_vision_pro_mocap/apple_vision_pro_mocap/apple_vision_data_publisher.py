#!/usr/bin/env python3
import numpy as np

# Compatibility shim for transforms3d on newer NumPy.
if not hasattr(np, "float"):
    np.float = float

import rclpy
from rclpy.node import Node
from tf_transformations import quaternion_from_matrix
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from avp_stream import VisionProStreamer
from aero_hand_open_msgs.msg import HandMocap

## Helper function
def get_rotation_matrix_from_axis_and_angle(axis='x', angle=90.0):
    """Return a 4x4 rotation matrix for a rotation about the given axis by angle in degrees."""
    angle_rad = np.deg2rad(angle)
    c = np.cos(angle_rad)
    s = np.sin(angle_rad)
    if axis == 'x':
        R = np.array([[1, 0, 0],
                      [0, c, -s],
                      [0, s, c]])
    elif axis == 'y':
        R = np.array([[c, 0, s],
                      [0, 1, 0],
                      [-s, 0, c]])
    elif axis == 'z':
        R = np.array([[c, -s, 0],
                      [s, c, 0],
                      [0, 0, 1]])
    else:
        raise ValueError("Axis must be 'x', 'y', or 'z'")
    R_homogeneous = np.eye(4)
    R_homogeneous[:3, :3] = R
    return R_homogeneous

class AppleVisionDataPublisher(Node):
    def __init__(self):
        super().__init__('apple_vision_data_publisher')

        ## Parameters
        self.declare_parameter('ip', '192.168.1.101')
        self.declare_parameter('viz', True)
        ip = self.get_parameter('ip').value
        self.viz = self.get_parameter('viz').value

        self.streamer = VisionProStreamer(ip=ip, record=True)

        ## Wrist Pose Publishers
        self.right_wrist_pub = self.create_publisher(PoseStamped, 'vision_pro/right_wrist', 10)
        self.left_wrist_pub = self.create_publisher(PoseStamped, 'vision_pro/left_wrist', 10)

        ## Hand Mocap Publisher
        self.mocap_pub = self.create_publisher(HandMocap, 'mocap_data', 10)
        if self.viz:
            self.head_pose_pub = self.create_publisher(PoseStamped, 'vision_pro/head_pose', 10)
            self.right_hand_marker_pub = self.create_publisher(MarkerArray, 'vision_pro/right_hand_markers', 10)
            self.left_hand_marker_pub = self.create_publisher(MarkerArray, 'vision_pro/left_hand_markers', 10)

        self.create_timer(0.01, self.timer_callback)

    def publish_pose(self, pose_matrix, publisher):
        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = pose_matrix[0, 3]
        pose.pose.position.y = pose_matrix[1, 3]
        pose.pose.position.z = pose_matrix[2, 3]
        q = quaternion_from_matrix(pose_matrix)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        publisher.publish(pose)

    def publish_hand_markers(self, hand_data, hand):
        """Publish small sphere to show the hand keypoints."""
        m = Marker()
        m.header.frame_id = 'world'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "hand_keypoints/" + hand
        m.id = 0
        m.type = Marker.SPHERE_LIST
        m.action = Marker.ADD
        m.scale.x = 0.01
        m.scale.y = 0.01
        m.scale.z = 0.01
        m.color.a = 1.0
        if hand == "right":
            color = (0.9, 0.2, 0.2)
            publisher = self.right_hand_marker_pub
        else:
            color = (0.2, 0.6, 0.9)
            publisher = self.left_hand_marker_pub
        m.color.r = color[0]
        m.color.g = color[1]
        m.color.b = color[2]
        for i in range(hand_data.shape[0]):
            point = Point()
            point.x = hand_data[i, 0, 3]
            point.y = hand_data[i, 1, 3]
            point.z = hand_data[i, 2, 3]
            m.points.append(point)
        ma = MarkerArray()
        ma.markers.append(m)
        publisher.publish(ma)

    def publish_hand_mocap_data(self, landmarks, side):
        mocap_msg = HandMocap()
        mocap_msg.header.stamp = self.get_clock().now().to_msg()
        mocap_msg.side = side

        ## Reshape landmarks from (25, 4, 4) to (25, 3)
        landmarks = landmarks[:, :3, 3]

        ## Fill mocap message, with Pose for each landmark
        mocap_msg.keypoints = [
            Pose(
                position=Point(x=float(lm[0]), y=float(lm[1]), z=float(lm[2])),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            ) for lm in landmarks
        ]
        self.mocap_pub.publish(mocap_msg)

    def timer_callback(self):
        ## Get the latest data from the streamer
        data = self.streamer.latest

        ## Rotate the data to our convention, -90 rotation around the z-axis
        rot_z_neg_90 = get_rotation_matrix_from_axis_and_angle(axis='z', angle=-90)

        # ## Publish Head Pose
        head_pose = rot_z_neg_90 @ data['head'][0]
        ## Rotate the head orientation to match our convention, 90 rot around z-axis
        rot_z_90 = get_rotation_matrix_from_axis_and_angle(axis='z', angle=90)
        head_pose[:3, :3] = head_pose[:3, :3] @ rot_z_90[:3, :3]

        ## Right Wrist 
        world_T_right_wrist = rot_z_neg_90 @ data['right_wrist'][0]
        ## Rotate the orientation to match our convention, -90 rotation around the y-axis
        rot_y_neg_90 = get_rotation_matrix_from_axis_and_angle(axis='y', angle=-90)
        world_T_right_wrist[:3, :3] = world_T_right_wrist[:3, :3] @ rot_y_neg_90[:3, :3]

        ## Left wrist
        world_T_left_wrist = rot_z_neg_90 @ data['left_wrist'][0]
        ## Rotate the orientation to match our convention, 90 rotation around the y-axis
        rot_y_90 = get_rotation_matrix_from_axis_and_angle(axis='y', angle=90)
        rot_x_180 = get_rotation_matrix_from_axis_and_angle(axis='x', angle=180)
        world_T_left_wrist[:3, :3] = world_T_left_wrist[:3, :3] @ rot_x_180[:3, :3] @ rot_y_90[:3, :3]

        ## Convert the wrist poses to head frame
        head_T_right_wrist = np.linalg.inv(head_pose) @ world_T_right_wrist
        head_T_left_wrist = np.linalg.inv(head_pose) @ world_T_left_wrist

        ## Shift the position by 1/2 meter up
        head_T_right_wrist[2, 3] += 0.5
        head_T_left_wrist[2, 3] += 0.5

        ## Publish wrist poses in head frame for visualization
        self.publish_pose(head_T_right_wrist, self.right_wrist_pub)
        self.publish_pose(head_T_left_wrist, self.left_wrist_pub)

        ## Hand Mocap Data
        # Right Hand
        right_hand_landmarks = data['right_fingers'] ## Shape (25, 4, 4)
        ## Correct orientation to match our convention
        right_hand_landmarks = rot_y_90 @ right_hand_landmarks
        self.publish_hand_mocap_data(right_hand_landmarks, side='right')

        ## Left Hand
        left_hand_landmarks = data['left_fingers']  ## Shape (25, 4, 4)
        ## Correct orientation to match our convention
        rot_z_180 = get_rotation_matrix_from_axis_and_angle(axis='z', angle=180)
        left_hand_landmarks = rot_y_90 @ rot_z_180 @ left_hand_landmarks
        self.publish_hand_mocap_data(left_hand_landmarks, side='left')

        ## Psuedo publish head pose as we transform everything to the head frame
        if self.viz:
            ## Head Pose Publishing
            head_pose = np.eye(4)
            head_pose[2, 3] = 0.5
            self.publish_pose(head_pose, self.head_pose_pub)

            ## Hand Mocap Data publishing
            right_hand_landmarks_in_wrist_frame = head_T_right_wrist @ right_hand_landmarks
            left_hand_landmarks_in_wrist_frame = head_T_left_wrist @ left_hand_landmarks

            self.publish_hand_markers(right_hand_landmarks_in_wrist_frame, hand="right")
            self.publish_hand_markers(left_hand_landmarks_in_wrist_frame, hand="left")

    def destroy_node(self):
        return super().destroy_node()


def main():
    rclpy.init()
    node = AppleVisionDataPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
    finally:
        rclpy.shutdown()
    

if __name__ == "__main__":
    main()
