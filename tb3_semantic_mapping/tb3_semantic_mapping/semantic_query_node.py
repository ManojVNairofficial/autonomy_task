#!/usr/bin/env python3
"""Semantic query node.

Exposes a ROS 2 service `/query_place` that accepts a free-form text prompt
("Where is the toilet?"), resolves it to a stored room location via a general
synonym + fuzzy-matching engine, and then sends
the robot there with a Nav2 NavigateToPose goal.

This is the "retrieve" half of the semantic system. It reads the JSON map that
`semantic_tagger_node` writes.
"""

from __future__ import annotations

import json
import os

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

from tb3_semantic_interfaces.srv import QueryPlace
from tb3_semantic_mapping.semantic_matching import resolve_query


class SemanticQuery(Node):

    def __init__(self):
        super().__init__('semantic_query')

        self.declare_parameter('map_file', os.path.expanduser('~/semantic_map.json'))
        self.declare_parameter('auto_navigate', True)

        self._map_file = self.get_parameter('map_file').value
        self._auto_nav = self.get_parameter('auto_navigate').value

        self._nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._srv = self.create_service(QueryPlace, 'query_place', self._on_query)

        self.get_logger().info(
            f'Semantic query service ready on /query_place (map: {self._map_file})'
        )

    # ---------------------------------------------------------------------------- #
    #                                    service                                   #
    # ---------------------------------------------------------------------------- #
    def _on_query(self, request, response):
        query = request.query
        self.get_logger().info(f'Query received: "{query}"')

        places = self._load_places()
        if not places:
            response.found = False
            response.message = 'Semantic map is empty — explore first.'
            self.get_logger().warn(response.message)
            return response

        label, score = resolve_query(query, list(places.keys()))
        if label is None:
            response.found = False
            response.message = (
                f'No known place matches "{query}". '
                f'Known places: {", ".join(places.keys())}'
            )
            self.get_logger().warn(response.message)
            return response

        info = places[label]
        response.found = True
        response.label = label
        response.confidence = float(info['confidence'])
        response.pose = self._make_pose(info['x'], info['y'])
        response.message = (
            f'Matched "{query}" -> {label} '
            f'(match={score:.2f}, conf={info["confidence"]:.2f}) '
            f'at ({info["x"]:.2f}, {info["y"]:.2f})'
        )
        self.get_logger().info(response.message)

        if self._auto_nav:
            self._navigate(response.pose, label)

        return response

    # ---------------------------------------------------------------------------- #
    #                                    helpers                                   #
    # ---------------------------------------------------------------------------- #
    def _load_places(self) -> dict[str, dict]:
        if not os.path.exists(self._map_file):
            return {}
        try:
            with open(self._map_file) as f:
                data = json.load(f)
            return {p['label']: p for p in data.get('places', [])}
        except Exception as e:                        
            self.get_logger().error(f'Failed to read map: {e}')
            return {}

    def _make_pose(self, x: float, y: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.w = 1.0
        return pose

    def _navigate(self, pose: PoseStamped, label: str):
        if not self._nav.wait_for_server(timeout_sec=3.0):
            self.get_logger().warn('Nav2 navigate_to_pose unavailable — returning pose only')
            return
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.get_logger().info(f'Navigating to {label}...')
        self._nav.send_goal_async(goal).add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Nav2 rejected the goal')
            return
        handle.get_result_async().add_done_callback(
            lambda f: self.get_logger().info('Navigation finished.')
        )


def main(args=None):
    rclpy.init(args=args)
    node = SemanticQuery()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
