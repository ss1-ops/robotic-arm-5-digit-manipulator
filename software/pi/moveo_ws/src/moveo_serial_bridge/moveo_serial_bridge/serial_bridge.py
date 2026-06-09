#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial
import time

# How often to send serial commands to the ESP32 (Hz).
# Lower = more time for the ESP32 to step between commands = smoother motion.
# 10 Hz = 100ms between commands, 5 Hz = 200ms, etc.
SEND_RATE_HZ = 5.0

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')

        self.ser = serial.Serial('/dev/moveo_arduino', 115200, timeout=1)
        time.sleep(2)  # give ESP32 time to reset

        self.last_positions = None
        self.last_send_time = 0.0
        self.min_interval = 1.0 / SEND_RATE_HZ

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10)

        self.get_logger().info(
            f'Serial Bridge started - rate-limited to {SEND_RATE_HZ:.0f} Hz '
            f'({self.min_interval*1000:.0f} ms between commands)')

    def joint_states_callback(self, msg):
        now = time.monotonic()
        if now - self.last_send_time < self.min_interval:
            return  # throttle - too soon to send again

        positions = tuple(round(p, 4) for p in msg.position)
        if positions == self.last_positions:
            return  # no change

        self.last_positions = positions
        self.last_send_time = now
        data_str = ','.join([f"{p:.4f}" for p in positions])
        self.ser.write((data_str + '\n').encode())
        self.get_logger().info(f'Sent to ESP32: {data_str}')

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
