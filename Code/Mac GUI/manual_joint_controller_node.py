#!/usr/bin/env python3
"""
Moveo Manual Joint Command Server
Runs on armpi, receives joint angle commands, publishes to /joint_states
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import json
import time


class ManualJointController(Node):
    """ROS2 node for manual joint control via string commands"""
    
    def __init__(self):
        super().__init__('manual_joint_controller')
        
        # Joint names matching the arm
        self.joint_names = ['Joint_1', 'Joint_2', 'Joint_3', 'Joint_4', 'Joint_5']
        self.current_positions = [0.0] * 5
        
        # Publisher for joint states
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        
        # Subscriber for manual commands (JSON format)
        self.subscriber = self.create_subscription(
            String,
            '/manual_joint_command',
            self.command_callback,
            10
        )
        
        # Timer for periodic publishing
        self.timer = self.create_timer(0.05, self.publish_joint_state)  # 20Hz
        
        self.get_logger().info('Manual Joint Controller initialized')
        self.get_logger().info('Subscribe to /manual_joint_command with String messages containing JSON: [angle1, angle2, ...]')
    
    def command_callback(self, msg: String):
        """Receive joint commands as JSON strings"""
        try:
            angles = json.loads(msg.data)
            if len(angles) == 5:
                self.current_positions = angles
                self.get_logger().info(f'Updated joint angles: {[f"{a:.3f}" for a in angles]}')
            else:
                self.get_logger().warn(f'Expected 5 angles, got {len(angles)}')
        except json.JSONDecodeError:
            self.get_logger().error(f'Failed to parse JSON: {msg.data}')
        except Exception as e:
            self.get_logger().error(f'Error processing command: {str(e)}')
    
    def publish_joint_state(self):
        """Publish current joint state"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.current_positions
        msg.velocity = [0.0] * 5
        msg.effort = [0.0] * 5
        
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ManualJointController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
