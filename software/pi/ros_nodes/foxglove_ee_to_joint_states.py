#!/usr/bin/env python3
import copy

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import RobotState, MoveItErrorCodes


class FoxgloveEeToJointStates(Node):
    def __init__(self):
        super().__init__('foxglove_ee_to_joint_states')

        self.joint_names = ['Joint_1', 'Joint_2', 'Joint_3', 'Joint_4', 'Joint_5']
        self.max_rate_hz = 15.0
        self.min_period_ns = int(1e9 / self.max_rate_hz)

        self.last_request_time_ns = 0
        self.last_fail_log_time_ns = 0
        self.last_solution = None
        self.pending = False

        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.sub = self.create_subscription(PoseStamped, '/ee_target', self.ee_cb, 10)

        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        if not self.ik_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn('/compute_ik not available after 10s; will keep trying on callbacks')

        self.get_logger().info('foxglove_ee_to_joint_states started')

    def ee_cb(self, msg: PoseStamped):
        now_ns = self.get_clock().now().nanoseconds

        if self.pending:
            return

        if now_ns - self.last_request_time_ns < self.min_period_ns:
            return

        if (msg.pose.orientation.x == 0.0 and
            msg.pose.orientation.y == 0.0 and
            msg.pose.orientation.z == 0.0 and
            msg.pose.orientation.w == 0.0):
            msg = copy.deepcopy(msg)
            msg.pose.orientation.w = 1.0

        req = GetPositionIK.Request()
        req.ik_request.group_name = 'arm'
        req.ik_request.pose_stamped = msg
        req.ik_request.avoid_collisions = False

        if self.last_solution is not None:
            seed = RobotState()
            seed.joint_state.name = list(self.joint_names)
            seed.joint_state.position = list(self.last_solution)
            req.ik_request.robot_state = seed

        if not self.ik_client.service_is_ready():
            self._log_ik_failure_throttled('/compute_ik service is not ready')
            return

        self.pending = True
        self.last_request_time_ns = now_ns
        future = self.ik_client.call_async(req)
        future.add_done_callback(self._ik_done)

    def _ik_done(self, future):
        self.pending = False
        try:
            resp = future.result()
        except Exception as exc:
            self._log_ik_failure_throttled(f'IK call failed: {exc}')
            return

        if resp.error_code.val != MoveItErrorCodes.SUCCESS:
            self._log_ik_failure_throttled(f'IK failed with code {resp.error_code.val}')
            return

        sol_names = list(resp.solution.joint_state.name)
        sol_pos = list(resp.solution.joint_state.position)
        sol_map = {n: p for n, p in zip(sol_names, sol_pos)}

        try:
            ordered_positions = [sol_map[n] for n in self.joint_names]
        except KeyError as missing:
            self._log_ik_failure_throttled(f'IK solution missing joint: {missing}')
            return

        self.last_solution = ordered_positions

        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = list(self.joint_names)
        out.position = list(ordered_positions)
        self.pub.publish(out)

    def _log_ik_failure_throttled(self, text: str):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_fail_log_time_ns >= int(1e9):
            self.get_logger().warn(text)
            self.last_fail_log_time_ns = now_ns


def main():
    rclpy.init()
    node = FoxgloveEeToJointStates()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
