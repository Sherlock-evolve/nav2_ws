"""Autonomous frontier-based exploration node.

Subscribes to the occupancy grid published by slam_toolbox, detects map
frontiers (free cells adjacent to unknown cells), and drives the robot toward
the nearest reachable frontier using the Nav2 ``NavigateToPose`` action. It
repeats until no frontiers remain (the map is fully explored) or a time budget
is hit. This replaces the manual ``teleop_keyboard`` step during SLAM mapping,
which is especially tedious for large worlds like ``real.world``.

The node mirrors ``mission_runner.py``: it is self-driving (no top-level spin),
uses ``ActionClient(self, NavigateToPose, ...)`` and resolves each goal with
``spin_until_future_complete`` / ``spin_once``.
"""

import csv
import math
import os
import time
from dataclasses import dataclass
from datetime import datetime

import numpy as np
import rclpy
import tf2_ros
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import TransformException


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


@dataclass(frozen=True)
class FrontierTarget:
    goal_x: float
    goal_y: float
    frontier_x: float
    frontier_y: float
    frontier_size: int
    distance: float


class ExploreRunner(Node):
    def __init__(self):
        super().__init__('explore_runner')

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('action_name', 'navigate_to_pose')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('goal_timeout_sec', 90.0)
        self.declare_parameter('explore_timeout_sec', 1800.0)
        self.declare_parameter('min_frontier_size', 5)
        self.declare_parameter('blacklist_radius_m', 1.0)
        self.declare_parameter('visited_radius_m', 0.8)
        self.declare_parameter('min_goal_distance_m', 0.5)
        self.declare_parameter('frontier_bin_size_m', 0.5)
        self.declare_parameter('min_cluster_size', 8)
        self.declare_parameter('frontier_setback_m', 0.5)
        self.declare_parameter('stuck_window_sec', 10.0)
        self.declare_parameter('stuck_threshold_m', 0.1)
        self.declare_parameter('post_goal_settle_sec', 1.0)
        self.declare_parameter('wait_first_map_sec', 60.0)
        self.declare_parameter('result_file', '')

        self._map_topic = self.get_parameter('map_topic').value
        self._action_name = self.get_parameter('action_name').value
        self._map_frame = self.get_parameter('map_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._goal_timeout = float(self.get_parameter('goal_timeout_sec').value)
        self._explore_timeout = float(self.get_parameter('explore_timeout_sec').value)
        self._min_frontier_size = int(self.get_parameter('min_frontier_size').value)
        self._blacklist_radius = float(self.get_parameter('blacklist_radius_m').value)
        self._visited_radius = float(self.get_parameter('visited_radius_m').value)
        self._min_goal_distance = float(self.get_parameter('min_goal_distance_m').value)
        self._frontier_bin_size = float(self.get_parameter('frontier_bin_size_m').value)
        self._min_cluster_size = int(self.get_parameter('min_cluster_size').value)
        self._frontier_setback = float(self.get_parameter('frontier_setback_m').value)
        self._stuck_window = float(self.get_parameter('stuck_window_sec').value)
        self._stuck_threshold = float(self.get_parameter('stuck_threshold_m').value)
        self._post_goal_settle = float(self.get_parameter('post_goal_settle_sec').value)
        self._wait_first_map = float(self.get_parameter('wait_first_map_sec').value)
        self._result_file = self._prepare_result_file(self.get_parameter('result_file').value)

        self._client = ActionClient(self, NavigateToPose, self._action_name)
        self._map_sub = self.create_subscription(
            OccupancyGrid, self._map_topic, self._map_callback, 10
        )
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._latest_map = None
        self._map_sequence = 0
        self._blacklist = []  # list of (x, y) world points that failed
        self._visited_frontiers = []  # list of (x, y) frontier points already reached

    # ----- public entry point -----

    def run(self):
        self.get_logger().info(f'Waiting for action server: {self._action_name}')
        if not self._client.wait_for_server(timeout_sec=60.0):
            self.get_logger().error(f'Action server not available: {self._action_name}')
            return

        self.get_logger().info(f'Waiting for first map on {self._map_topic}')
        deadline = time.monotonic() + self._wait_first_map
        while rclpy.ok() and self._latest_map is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.5)
        if self._latest_map is None:
            self.get_logger().error('No map received before timeout, exiting')
            return

        start = time.monotonic()
        iteration = 0
        with open(self._result_file, 'w', newline='', encoding='utf-8') as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=[
                'iter', 'target_x', 'target_y', 'frontier_x', 'frontier_y',
                'frontier_size', 'status', 'duration_sec',
                'frontiers_remaining', 'message',
            ])
            writer.writeheader()
            csv_file.flush()

            while rclpy.ok():
                if time.monotonic() - start > self._explore_timeout:
                    self.get_logger().info('Exploration time budget reached, stopping')
                    break

                rclpy.spin_once(self, timeout_sec=0.5)  # flush map / tf callbacks

                wx, wy = self._detect_frontiers(self._latest_map)
                if wx.size < self._min_frontier_size:
                    self.get_logger().info('No more frontiers detected — exploration complete')
                    break

                robot_pose = self._get_robot_pose()
                if robot_pose is None:
                    self.get_logger().warn('Robot pose unavailable, retrying in 1s')
                    time.sleep(1.0)
                    continue

                target = self._pick_target(wx, wy, robot_pose)
                if target is None:
                    self.get_logger().info(
                        'No unvisited frontier candidates remain; stopping exploration'
                    )
                    break

                iteration += 1
                tx, ty = target.goal_x, target.goal_y
                yaw = math.atan2(ty - robot_pose[1], tx - robot_pose[0])
                self.get_logger().info(
                    f'[{iteration}] goal=({tx:.2f},{ty:.2f}) '
                    f'frontier=({target.frontier_x:.2f},{target.frontier_y:.2f}) '
                    f'size={target.frontier_size} frontiers={wx.size} '
                    f'visited={len(self._visited_frontiers)} '
                    f'blacklisted={len(self._blacklist)}'
                )

                goal_start = time.monotonic()
                status, message = self._send_goal(tx, ty, yaw)
                duration = time.monotonic() - goal_start
                map_sequence_after_goal = self._map_sequence
                writer.writerow({
                    'iter': iteration,
                    'target_x': f'{tx:.3f}',
                    'target_y': f'{ty:.3f}',
                    'frontier_x': f'{target.frontier_x:.3f}',
                    'frontier_y': f'{target.frontier_y:.3f}',
                    'frontier_size': target.frontier_size,
                    'status': status,
                    'duration_sec': f'{duration:.3f}',
                    'frontiers_remaining': int(wx.size),
                    'message': message,
                })
                csv_file.flush()

                if status == 'succeeded':
                    self._visited_frontiers.append(
                        (target.frontier_x, target.frontier_y)
                    )
                    self.get_logger().info(
                        f'Visited frontier ({target.frontier_x:.2f},'
                        f'{target.frontier_y:.2f}); marked to avoid repeat goals'
                    )
                else:
                    self._blacklist.append((target.frontier_x, target.frontier_y))
                    self.get_logger().warn(
                        f'Goal {status} at ({tx:.2f},{ty:.2f}); blacklisted '
                        f'(total {len(self._blacklist)})'
                    )
                self._wait_for_map_settle(map_sequence_after_goal)

        self.get_logger().info(
            f'Exploration finished after {iteration} goal(s); CSV: {self._result_file}'
        )

    # ----- map / frontier handling -----

    def _map_callback(self, msg):
        self._latest_map = msg
        self._map_sequence += 1

    def _detect_frontiers(self, msg):
        """Return frontier cells as two numpy arrays (world_x, world_y).

        A frontier is a free cell (occupancy 0) that has at least one unknown
        neighbor (occupancy -1) in the 4-neighborhood. Edges do not wrap.
        """
        if msg is None:
            return np.empty(0), np.empty(0)
        info = msg.info
        h, w = info.height, info.width
        if h == 0 or w == 0:
            return np.empty(0), np.empty(0)

        arr = np.asarray(msg.data, dtype=np.int8).reshape((h, w))
        free = arr == 0
        unknown = arr == -1

        neighbor_unknown = np.zeros_like(unknown)
        neighbor_unknown[1:, :] |= unknown[:-1, :]
        neighbor_unknown[:-1, :] |= unknown[1:, :]
        neighbor_unknown[:, 1:] |= unknown[:, :-1]
        neighbor_unknown[:, :-1] |= unknown[:, 1:]

        ys, xs = np.nonzero(free & neighbor_unknown)
        if ys.size == 0:
            return np.empty(0), np.empty(0)

        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y
        wx = ox + (xs + 0.5) * res
        wy = oy + (ys + 0.5) * res
        return wx, wy

    def _pick_target(self, wx, wy, robot_pose):
        """Pick a stable frontier target.

        Frontier cells are coarsely bucketed (frontier_bin_size_m) so the goal
        is selected at the *region* level rather than chasing a single jittery
        boundary cell. The nearest qualifying bucket's centroid is chosen, then
        pulled back toward the robot (frontier_setback_m) so the goal lands in
        known-free space instead of on the moving unknown boundary.
        """
        rx, ry = robot_pose
        dist = np.hypot(wx - rx, wy - ry)
        valid = dist >= self._min_goal_distance
        for bx, by in self._blacklist:
            valid &= np.hypot(wx - bx, wy - by) >= self._blacklist_radius
        for vx, vy in self._visited_frontiers:
            valid &= np.hypot(wx - vx, wy - vy) >= self._visited_radius
        if not np.any(valid):
            return None

        wxv, wyv = wx[valid], wy[valid]
        bsize = self._frontier_bin_size
        # Non-negative combined bin key (offset keeps indices positive so the
        # scalar key is collision-free for any realistic map extent).
        bkx = np.floor(wxv / bsize).astype(np.int64) + 1_000_000
        bky = np.floor(wyv / bsize).astype(np.int64) + 1_000_000
        bkeys = bkx * 2_000_001 + bky
        _, inv, counts = np.unique(bkeys, return_inverse=True, return_counts=True)
        cx = np.bincount(inv, weights=wxv) / counts
        cy = np.bincount(inv, weights=wyv) / counts
        cdist = np.hypot(cx - rx, cy - ry)

        keep = counts >= self._min_cluster_size
        if not np.any(keep):
            keep = np.ones_like(counts, dtype=bool)  # fall back to any bucket
        cdist = np.where(keep, cdist, np.inf)
        idx = int(np.argmin(cdist))

        fx, fy = float(cx[idx]), float(cy[idx])
        tx, ty = fx, fy
        ax, ay = fx - rx, fy - ry
        length = math.hypot(ax, ay)
        if length > self._frontier_setback:
            scale = (length - self._frontier_setback) / length
            tx = rx + ax * scale
            ty = ry + ay * scale
        return FrontierTarget(
            goal_x=tx,
            goal_y=ty,
            frontier_x=fx,
            frontier_y=fy,
            frontier_size=int(counts[idx]),
            distance=float(cdist[idx]),
        )

    def _get_robot_pose(self, timeout_sec=1.0):
        """Return robot (x, y) in the map frame via TF, or None."""
        for frame in (self._base_frame, 'base_footprint'):
            try:
                t = self._tf_buffer.lookup_transform(
                    self._map_frame, frame, rclpy.time.Time(),
                    timeout=Duration(seconds=timeout_sec),
                )
                tr = t.transform.translation
                return tr.x, tr.y
            except TransformException as ex:
                self.get_logger().debug(f'TF {self._map_frame}->{frame} failed: {ex}')
        return None

    # ----- goal execution (mirrors mission_runner._run_goal) -----

    def _send_goal(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = self._map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        q = _yaw_to_quaternion(yaw)
        pose.pose.orientation.x = q['x']
        pose.pose.orientation.y = q['y']
        pose.pose.orientation.z = q['z']
        pose.pose.orientation.w = q['w']

        goal = NavigateToPose.Goal()
        goal.pose = pose

        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return 'rejected', 'goal not accepted'

        result_future = goal_handle.get_result_async()
        start = time.monotonic()
        last_pose = self._get_robot_pose(timeout_sec=0.1)
        last_move_time = start
        while rclpy.ok() and not result_future.done():
            now = time.monotonic()
            if now - start > self._goal_timeout:
                self.get_logger().warn('Goal timed out, canceling')
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future)
                return 'timeout', 'goal timed out'

            # Stuck watchdog: abort if the robot has not translated for a while.
            # This catches the "spinning in open space" case live instead of
            # waiting out the full goal timeout.
            pose = self._get_robot_pose(timeout_sec=0.1)
            if pose is not None:
                if last_pose is None or math.hypot(
                    pose[0] - last_pose[0], pose[1] - last_pose[1]
                ) > self._stuck_threshold:
                    last_pose = pose
                    last_move_time = now
                elif now - last_move_time > self._stuck_window:
                    self.get_logger().warn(
                        f'No movement for {self._stuck_window:.0f}s, canceling as stuck'
                    )
                    cancel_future = goal_handle.cancel_goal_async()
                    rclpy.spin_until_future_complete(self, cancel_future)
                    return 'stuck', 'robot did not translate'

            rclpy.spin_once(self, timeout_sec=0.2)

        result = result_future.result()
        return _status_name(result.status), f'result_status={result.status}'

    # ----- helpers -----

    def _prepare_result_file(self, path):
        if path:
            directory = os.path.dirname(path)
            if directory:
                os.makedirs(directory, exist_ok=True)
            return path
        output_dir = os.environ.get('NAV2_LAB_RESULTS_DIR', '/tmp/nav2_lab_results')
        os.makedirs(output_dir, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        return os.path.join(output_dir, f'{timestamp}_explore.csv')

    def _wait_for_map_settle(self, previous_sequence):
        if self._post_goal_settle <= 0.0:
            return

        deadline = time.monotonic() + self._post_goal_settle
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._map_sequence > previous_sequence:
                return


def main(args=None):
    rclpy.init(args=args)
    node = ExploreRunner()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
