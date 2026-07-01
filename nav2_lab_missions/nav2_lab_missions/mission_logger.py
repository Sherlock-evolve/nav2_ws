import csv
import os
from datetime import datetime

import rclpy
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from rclpy.node import Node


class MissionLogger(Node):
    def __init__(self):
        super().__init__('mission_logger')
        self.declare_parameter('output_file', '')
        self.declare_parameter('sample_period_sec', 1.0)

        self._output_file = self._prepare_output_file(self.get_parameter('output_file').value)
        self._latest_cmd_vel = None
        self._latest_pose = None
        self._latest_status = None

        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._pose_callback, 10)
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self._status_callback, 10)

        self._csv_file = open(self._output_file, 'w', newline='', encoding='utf-8')
        self._writer = csv.DictWriter(
            self._csv_file,
            fieldnames=['stamp_sec', 'x', 'y', 'linear_x', 'angular_z', 'action_status'],
        )
        self._writer.writeheader()

        period = float(self.get_parameter('sample_period_sec').value)
        self.create_timer(period, self._sample)
        self.get_logger().info(f'Mission telemetry will be written to {self._output_file}')

    def destroy_node(self):
        if hasattr(self, '_csv_file') and not self._csv_file.closed:
            self._csv_file.close()
        super().destroy_node()

    def _prepare_output_file(self, path):
        if path:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            return path
        output_dir = '/tmp/nav2_lab_results'
        os.makedirs(output_dir, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        return os.path.join(output_dir, f'{timestamp}_telemetry.csv')

    def _cmd_vel_callback(self, msg):
        self._latest_cmd_vel = msg

    def _pose_callback(self, msg):
        self._latest_pose = msg

    def _status_callback(self, msg):
        if msg.status_list:
            self._latest_status = msg.status_list[-1].status

    def _sample(self):
        now = self.get_clock().now().nanoseconds / 1e9
        pose = self._latest_pose.pose.pose if self._latest_pose else None
        cmd_vel = self._latest_cmd_vel
        self._writer.writerow({
            'stamp_sec': f'{now:.3f}',
            'x': '' if pose is None else f'{pose.position.x:.3f}',
            'y': '' if pose is None else f'{pose.position.y:.3f}',
            'linear_x': '' if cmd_vel is None else f'{cmd_vel.linear.x:.3f}',
            'angular_z': '' if cmd_vel is None else f'{cmd_vel.angular.z:.3f}',
            'action_status': '' if self._latest_status is None else self._latest_status,
        })
        self._csv_file.flush()


def main(args=None):
    rclpy.init(args=args)
    node = MissionLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
