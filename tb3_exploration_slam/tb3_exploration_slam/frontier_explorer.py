#!/usr/bin/env python3
"""Frontier-based autonomous exploration node.

Detects frontier cells (free cells bordering unknown space) on the live SLAM map,
clusters them, and iteratively sends the nearest cluster centroid as a Nav2 goal
until no frontiers remain.
"""

import math

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from scipy import ndimage
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


class FrontierExplorer(Node):
    _IDLE = 'idle'
    _NAVIGATING = 'navigating'
    _DONE = 'done'

    def __init__(self):
        super().__init__('frontier_explorer')

        self.declare_parameter('min_frontier_size', 5)
        self.declare_parameter('goal_timeout_sec', 30.0)
        self.declare_parameter('tried_goal_radius', 0.3)
        self.declare_parameter('min_frontier_dist', 0.25)
        self.declare_parameter('explore_hz', 1.0)
        self.declare_parameter('max_stuck_resets', 8)

        self._min_size = self.get_parameter('min_frontier_size').value
        self._timeout = self.get_parameter('goal_timeout_sec').value
        self._tried_r = self.get_parameter('tried_goal_radius').value
        self._min_dist = self.get_parameter('min_frontier_dist').value
        self._max_resets = self.get_parameter('max_stuck_resets').value

        self._state = self._IDLE
        self._map: OccupancyGrid | None = None
        self._goal_handle = None
        self._tried: list[tuple[float, float]] = []
        self._timeout_timer = None
        self._pose_when_sent: tuple[float, float] | None = None
        self._resets_without_progress = 0
        self._nav_ready = False
        self._nav_wait_logged_at: float = 0.0  

        self._tf_buf = Buffer()
        self._tf = TransformListener(self._tf_buf, self)

        self._nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.create_subscription(OccupancyGrid, '/map', self._map_cb, 10)
        self._marker_pub = self.create_publisher(MarkerArray, '/exploration/frontiers', 10)

        hz = self.get_parameter('explore_hz').value
        self.create_timer(1.0 / hz, self._step)
        self.get_logger().info('Frontier explorer started — waiting for /map and Nav2')

    # ------------------------------------------------------------------ callbacks

    def _map_cb(self, msg: OccupancyGrid):
        self._map = msg

    # ------------------------------------------------------------------ main loop

    def _step(self):
        if not self._nav_ready:
            import time as _time
            now = _time.monotonic()
            if self._nav.wait_for_server(timeout_sec=0.5):
                self._nav_ready = True
                self.get_logger().info('Nav2 navigate_to_pose ready — starting exploration')
            else:
                if now - self._nav_wait_logged_at > 10.0:
                    self.get_logger().info('Waiting for Nav2 navigate_to_pose server...')
                    self._nav_wait_logged_at = now
            return

        if self._state in (self._NAVIGATING, self._DONE):
            return
        if self._map is None:
            return

        robot = self._robot_pose()
        if robot is None:
            return

        frontiers = self._find_frontiers()
        self._publish_markers(frontiers)

        if not frontiers:
            self.get_logger().info('No frontiers remain — exploration complete!')
            self._state = self._DONE
            return

        rx, ry = robot

        def _near_tried(fx, fy):
            return any(math.hypot(fx - gx, fy - gy) < self._tried_r
                       for gx, gy in self._tried)

        valid = [f for f in frontiers
                 if math.hypot(f[0]-rx, f[1]-ry) >= self._min_dist
                 and not _near_tried(f[0], f[1])]

        if not valid:
            self._resets_without_progress += 1
            if self._resets_without_progress >= self._max_resets:
                self.get_logger().info(
                    f'Stuck after {self._max_resets} resets without progress — '
                    'exploration complete!'
                )
                self._state = self._DONE
                return
            if self._tried:
                self.get_logger().info(
                    f'All frontiers tried (reset #{self._resets_without_progress}/'
                    f'{self._max_resets}) — clearing memory'
                )
                self._tried.clear()
            valid = [f for f in frontiers
                     if math.hypot(f[0]-rx, f[1]-ry) >= self._min_dist]

        if not valid:
            self.get_logger().info('No frontiers beyond minimum distance — exploration complete!')
            self._state = self._DONE
            return

        valid.sort(key=lambda f: math.hypot(f[0] - rx, f[1] - ry))
        tx, ty, size = valid[0]
        dist = math.hypot(tx - rx, ty - ry)

        self.get_logger().info(
            f'Navigating to frontier ({tx:.2f}, {ty:.2f}) '
            f'dist={dist:.2f}m size={size}cells, {len(valid)} candidates'
        )
        self._send_goal(tx, ty)

    # ------------------------------------------------------------------ nav2 helpers

    def _send_goal(self, x: float, y: float):
        if not self._nav.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('NavigateToPose server lost — will retry')
            self._nav_ready = False
            return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0

        self._state = self._NAVIGATING
        self._current_goal_xy = (x, y)
        self._pose_when_sent = self._robot_pose()

        future = self._nav.send_goal_async(goal)
        future.add_done_callback(self._on_goal_accepted)

        self._timeout_timer = self.create_timer(self._timeout, self._on_timeout)

    def _on_goal_accepted(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2')
            self._state = self._IDLE
            self._cancel_timeout()
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        self._cancel_timeout()
        result = future.result()
        gx, gy = self._current_goal_xy

        pose_now = self._robot_pose()
        moved = False
        if pose_now and self._pose_when_sent:
            moved = math.hypot(pose_now[0] - self._pose_when_sent[0],
                               pose_now[1] - self._pose_when_sent[1]) > 0.08

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            if moved:
                self.get_logger().info(f'Reached frontier ({gx:.2f}, {gy:.2f})')
                self._resets_without_progress = 0   
            else:
                self.get_logger().warn(
                    f'Nav2 succeeded without movement for frontier ({gx:.2f}, {gy:.2f}) '
                    f'— marking as tried'
                )
        else:
            self.get_logger().warn(
                f'Frontier ({gx:.2f}, {gy:.2f}) failed (status {result.status})'
            )

        self._tried.append(self._current_goal_xy)
        self._state = self._IDLE

    def _on_timeout(self):
        self.get_logger().warn('Goal timed out — cancelling')
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        self._tried.append(self._current_goal_xy)
        self._state = self._IDLE
        self._cancel_timeout()

    def _cancel_timeout(self):
        if self._timeout_timer is not None:
            self._timeout_timer.cancel()
            self._timeout_timer = None

    # ------------------------------------------------------------------ frontier detection

    def _find_frontiers(self) -> list[tuple[float, float, int]]:
        """Return list of (world_x, world_y, cell_count) frontier centroids."""
        info = self._map.info
        w, h = info.width, info.height
        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y

        data = np.array(self._map.data, dtype=np.int8).reshape(h, w)

        free = (data == 0)
        unknown = (data == -1)

        unknown_dilated = ndimage.binary_dilation(unknown, structure=np.ones((3, 3)))
        frontier_mask = free & unknown_dilated

        labeled, n = ndimage.label(frontier_mask)
        frontiers = []
        for i in range(1, n + 1):
            cells = np.argwhere(labeled == i)   
            if len(cells) < self._min_size:
                continue
            row_c = float(np.mean(cells[:, 0]))
            col_c = float(np.mean(cells[:, 1]))
            wx = ox + (col_c + 0.5) * res
            wy = oy + (row_c + 0.5) * res
            frontiers.append((wx, wy, len(cells)))

        return frontiers

    # ------------------------------------------------------------------ TF helpers

    def _robot_pose(self) -> tuple[float, float] | None:
        try:
            tf = self._tf_buf.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time()
            )
            t = tf.transform.translation
            return t.x, t.y
        except Exception:
            return None

    # ------------------------------------------------------------------ visualisation

    def _publish_markers(self, frontiers: list[tuple[float, float, int]]):
        ma = MarkerArray()
        del_marker = Marker()
        del_marker.action = Marker.DELETEALL
        ma.markers.append(del_marker)

        for i, (fx, fy, size) in enumerate(frontiers):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'frontiers'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = fx
            m.pose.position.y = fy
            m.pose.position.z = 0.1
            m.pose.orientation.w = 1.0
            scale = min(0.5, max(0.15, size * 0.005))
            m.scale.x = m.scale.y = m.scale.z = scale
            m.color.a = 0.8
            m.color.r = 0.2
            m.color.g = 0.9
            m.color.b = 0.1
            ma.markers.append(m)

        self._marker_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
