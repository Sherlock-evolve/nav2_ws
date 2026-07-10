import csv
import math
import os
import time
from datetime import datetime

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


def _yaw_to_quaternion(yaw):
    return {
        'x': 0.0,
        'y': 0.0,
        'z': math.sin(yaw * 0.5),
        'w': math.cos(yaw * 0.5),
    }


def _status_name(status):
    names = {
        GoalStatus.STATUS_UNKNOWN: 'unknown',
        GoalStatus.STATUS_ACCEPTED: 'accepted',
        GoalStatus.STATUS_EXECUTING: 'executing',
        GoalStatus.STATUS_CANCELING: 'canceling',
        GoalStatus.STATUS_SUCCEEDED: 'succeeded',
        GoalStatus.STATUS_CANCELED: 'canceled',
        GoalStatus.STATUS_ABORTED: 'aborted',
    }
    return names.get(status, f'status_{status}')


class MissionRunner(Node):
    def __init__(self):
        super().__init__('mission_runner')
        self.declare_parameter('mission_file', '')
        self.declare_parameter('result_file', '')
        self.declare_parameter('start_delay_sec', 0.0)
        self.declare_parameter('action_name', 'navigate_to_pose')
        self.declare_parameter('wait_for_amcl_timeout_sec', 30.0)
        self.declare_parameter('nav_activation_delay_sec', 5.0)
        self.declare_parameter('retry_backoff_sec', 2.0)
        self.declare_parameter('initial_pose_subscriber_timeout_sec', 10.0)
        self.declare_parameter('initial_pose_publish_duration_sec', 3.0)

        self._mission_file = self.get_parameter('mission_file').value
        self._result_file = self.get_parameter('result_file').value
        self._start_delay_sec = float(self.get_parameter('start_delay_sec').value)
        self._action_name = self.get_parameter('action_name').value
        self._wait_for_amcl_timeout_sec = float(self.get_parameter('wait_for_amcl_timeout_sec').value)
        self._nav_activation_delay_sec = float(self.get_parameter('nav_activation_delay_sec').value)
        self._retry_backoff_sec = float(self.get_parameter('retry_backoff_sec').value)
        self._initial_pose_subscriber_timeout_sec = float(
            self.get_parameter('initial_pose_subscriber_timeout_sec').value
        )
        self._initial_pose_publish_duration_sec = float(
            self.get_parameter('initial_pose_publish_duration_sec').value
        )
        self._client = ActionClient(self, NavigateToPose, self._action_name)
        self._initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._amcl_pose_callback, 10)

        self._last_feedback = ''
        self._last_recovery_count = 0
        self._amcl_pose_received = False

    def run(self):
        mission = self._load_mission(self._mission_file)
        result_file = self._prepare_result_file(self._result_file)

        if self._start_delay_sec > 0.0:
            self.get_logger().info(f'Waiting {self._start_delay_sec:.1f}s before mission start')
            time.sleep(self._start_delay_sec)

        self.get_logger().info(f'Waiting for action server: {self._action_name}')
        if not self._client.wait_for_server(timeout_sec=60.0):
            raise RuntimeError(f'Action server not available: {self._action_name}')

        if mission.get('publish_initial_pose', True):
            self._publish_initial_pose(mission)
        else:
            self.get_logger().info('Skipping initial pose publication for this mission')

        if mission.get('wait_for_localization', True):
            self._wait_for_localization()
        elif self._nav_activation_delay_sec > 0.0:
            self.get_logger().info(
                f'Skipping localization wait; waiting {self._nav_activation_delay_sec:.1f}s for Nav2 activation'
            )
            time.sleep(self._nav_activation_delay_sec)

        goals = mission.get('goals', [])
        frame_id = mission.get('frame_id', 'map')
        timeout_sec = float(mission.get('default_timeout_sec', 90.0))
        retry_count = int(mission.get('retry_count', 0))

        with open(result_file, 'w', newline='', encoding='utf-8') as csv_file:
            writer = csv.DictWriter(
                csv_file,
                fieldnames=[
                    'goal_name',
                    'attempt',
                    'status',
                    'duration_sec',
                    'recovery_count',
                    'message',
                ],
            )
            writer.writeheader()

            for goal in goals:
                final_status = 'not_started'
                for attempt in range(1, retry_count + 2):
                    row = self._run_goal(frame_id, goal, timeout_sec, attempt)
                    writer.writerow(row)
                    csv_file.flush()
                    final_status = row['status']
                    if final_status == 'succeeded':
                        break
                    if attempt < retry_count + 1:
                        self.get_logger().info(
                            f"Retrying goal '{goal.get('name', 'unnamed')}' after {self._retry_backoff_sec:.1f}s"
                        )
                        time.sleep(self._retry_backoff_sec)
                self.get_logger().info(f"Goal '{goal.get('name', 'unnamed')}' final status: {final_status}")

        self.get_logger().info(f'Mission results written to {result_file}')

    def _load_mission(self, path):
        if not path:
            raise ValueError('mission_file parameter is required')
        with open(path, 'r', encoding='utf-8') as mission_file:
            mission = yaml.safe_load(mission_file) or {}
        if not mission.get('goals'):
            raise ValueError(f'Mission has no goals: {path}')
        return mission

    def _prepare_result_file(self, path):
        if path:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            return path
        output_dir = os.environ.get('NAV2_LAB_RESULTS_DIR', '/tmp/nav2_lab_results')
        os.makedirs(output_dir, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        return os.path.join(output_dir, f'{timestamp}_mission.csv')

    def _run_goal(self, frame_id, goal, timeout_sec, attempt):
        name = goal.get('name', 'unnamed')
        pose = self._make_pose(frame_id, goal)
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = pose
        self._last_feedback = ''
        self._last_recovery_count = 0

        self.get_logger().info(f"Sending goal '{name}' attempt {attempt}: x={goal['x']}, y={goal['y']}, yaw={goal['yaw']}")
        start = time.monotonic()
        send_future = self._client.send_goal_async(nav_goal, feedback_callback=self._feedback_callback)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            return self._result_row(name, attempt, 'rejected', start, 'Goal rejected by action server')

        result_future = goal_handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            if time.monotonic() - start > timeout_sec:
                self.get_logger().warn(f"Goal '{name}' timed out; canceling")
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future)
                return self._result_row(name, attempt, 'timeout', start, self._last_feedback)
            rclpy.spin_once(self, timeout_sec=0.2)

        result = result_future.result()
        status = _status_name(result.status)
        return self._result_row(name, attempt, status, start, self._last_feedback)

    def _make_pose(self, frame_id, goal):
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(goal['x'])
        pose.pose.position.y = float(goal['y'])
        pose.pose.position.z = 0.0
        quat = _yaw_to_quaternion(float(goal.get('yaw', 0.0)))
        pose.pose.orientation.x = quat['x']
        pose.pose.orientation.y = quat['y']
        pose.pose.orientation.z = quat['z']
        pose.pose.orientation.w = quat['w']
        return pose

    def _publish_initial_pose(self, mission):
        initial_pose = mission.get('initial_pose')
        if not initial_pose:
            self.get_logger().warn('Mission has no initial_pose; AMCL must be initialized another way')
            return

        frame_id = mission.get('frame_id', 'map')
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = frame_id
        msg.pose.pose = self._make_pose(frame_id, initial_pose).pose
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.06853891945200942

        self.get_logger().info(
            f"Publishing initial pose: x={initial_pose['x']}, y={initial_pose['y']}, yaw={initial_pose.get('yaw', 0.0)}"
        )
        self._wait_for_initial_pose_subscriber()

        deadline = time.monotonic() + self._initial_pose_publish_duration_sec
        while rclpy.ok() and time.monotonic() < deadline:
            msg.header.stamp = self.get_clock().now().to_msg()
            self._initial_pose_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)

    def _wait_for_initial_pose_subscriber(self):
        deadline = time.monotonic() + self._initial_pose_subscriber_timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if self._initial_pose_pub.get_subscription_count() > 0:
                return
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().warn(
            'No /initialpose subscribers discovered before publishing; initial pose may be missed'
        )

    def _wait_for_localization(self):
        deadline = time.monotonic() + self._wait_for_amcl_timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if self._amcl_pose_received:
                if self._nav_activation_delay_sec > 0.0:
                    self.get_logger().info(
                        f'Localization received; waiting {self._nav_activation_delay_sec:.1f}s for Nav2 activation'
                    )
                    time.sleep(self._nav_activation_delay_sec)
                return
            rclpy.spin_once(self, timeout_sec=0.2)

        raise RuntimeError(
            f'No /amcl_pose received within {self._wait_for_amcl_timeout_sec:.1f}s after publishing initial pose'
        )

    def _amcl_pose_callback(self, _msg):
        self._amcl_pose_received = True

    def _feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        distance = getattr(feedback, 'distance_remaining', None)
        if distance is not None:
            self._last_feedback = f'distance_remaining={distance:.3f}'
        recoveries = getattr(feedback, 'number_of_recoveries', None)
        if recoveries is not None:
            self._last_recovery_count = int(recoveries)

    def _result_row(self, name, attempt, status, start, message):
        return {
            'goal_name': name,
            'attempt': attempt,
            'status': status,
            'duration_sec': f'{time.monotonic() - start:.3f}',
            'recovery_count': self._last_recovery_count,
            'message': message,
        }


def main(args=None):
    rclpy.init(args=args)
    node = MissionRunner()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
