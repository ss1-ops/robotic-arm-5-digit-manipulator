#!/usr/bin/env python3
"""
Moveo Manual Joint Command Server
Runs on armpi. Receives JSON joint-angle commands on /manual_joint_command
and publishes JointState messages directly to /joint_commands for the ESP32.

The ESP32 subscribes to /joint_commands (sensor_msgs/JointState) and uses
the position array as its stepper target — completely separate from the
/joint_states feedback topic which joint_state_broadcaster owns.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import json


JOINT_NAMES = ['Joint_1', 'Joint_2', 'Joint_3', 'Joint_4', 'Joint_5']


class ManualJointController(Node):
    """Bridges /manual_joint_command (JSON String) -> /joint_commands (JointState)."""

    def __init__(self):
        super().__init__('manual_joint_controller')

        best_effort_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )

        self.publisher = self.create_publisher(
            JointState,
            '/joint_commands',
            best_effort_qos,
        )

        self.subscriber = self.create_subscription(
            String,
            '/manual_joint_command',
            self.command_callback,
            10,
        )

        self.get_logger().info('Manual Joint Controller ready')
        self.get_logger().info(
            'Listening on /manual_joint_command '
            '(JSON array of 5 angles in radians) -> /joint_commands'
        )

    def command_callback(self, msg: String):
        """Parse JSON command and publish as JointState to /joint_commands."""
        try:
            angles = json.loads(msg.data)
            if len(angles) != 5:
                self.get_logger().warn(
                    f'Expected 5 angles, got {len(angles)}: {msg.data}'
                )
                return

            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = JOINT_NAMES
            js.position = [float(a) for a in angles]
            js.velocity = [0.0] * 5
            js.effort = [0.0] * 5

            self.publisher.publish(js)

            self.get_logger().info(
                f'-> /joint_commands {[f"{a:.3f}" for a in angles]}'
            )

        except json.JSONDecodeError:
            self.get_logger().error(f'Bad JSON: {msg.data}')
        except Exception as e:
            self.get_logger().error(f'Error: {e}')


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

