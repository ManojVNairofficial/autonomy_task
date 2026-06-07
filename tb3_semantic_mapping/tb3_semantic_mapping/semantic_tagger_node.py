#!/usr/bin/env python3
"""Semantic tagger node.

Subscribes to the robot's camera, classifies each (throttled) frame with the
VLM backend, looks up where the robot is via TF, and records the best-seen
location of every room into a JSON semantic map. Also publishes RViz text
markers so the labelled places are visible on top of the SLAM map.

This is the "capture + store" half of the semantic system. The query node
("retrieve") consumes the JSON file this node writes.
"""

from __future__ import annotations

import json
import os

import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from tb3_semantic_mapping.vlm_interface import make_vlm


# Colours for the RViz text markers (RGBA).
_MARKER_COLOR = {
    'kitchen': (0.9, 0.1, 0.1, 1.0),
    'toilet':  (0.1, 0.3, 0.9, 1.0),
    'office':  (0.1, 0.8, 0.2, 1.0),
    'hallway': (0.7, 0.7, 0.7, 1.0),
}


class SemanticTagger(Node):

    def __init__(self):
        super().__init__('semantic_tagger')

        self.declare_parameter('vlm_backend', 'mock')
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('process_period', 0.5)       # seconds between classifications
        self.declare_parameter('min_confidence', 0.65)      # ignore weak detections
        self.declare_parameter('map_file', os.path.expanduser('~/semantic_map.json'))
        self.declare_parameter('snapshot_dir', os.path.expanduser('~/semantic_snapshots'))

        self._backend_name = self.get_parameter('vlm_backend').value
        self._map_frame = self.get_parameter('map_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._min_conf = self.get_parameter('min_confidence').value
        self._map_file = self.get_parameter('map_file').value
        self._snapshot_dir = self.get_parameter('snapshot_dir').value
        os.makedirs(self._snapshot_dir, exist_ok=True)

        self._vlm = make_vlm(self._backend_name)
        self._bridge = CvBridge()

        self._tf_buf = Buffer()
        self._tf = TransformListener(self._tf_buf, self)

        # --- best observation per label: {label: {x, y, confidence, description}} --- #
        self._places: dict[str, dict] = {}
        self._last_proc_time = 0.0
        self._proc_period = self.get_parameter('process_period').value

        cam_topic = self.get_parameter('camera_topic').value
        self.create_subscription(Image, cam_topic, self._image_cb, 10)
        self._marker_pub = self.create_publisher(MarkerArray, '/semantic_map', 10)
        self.create_timer(1.0, self._publish_markers)

        self.get_logger().info(
            f"Semantic tagger ready (backend='{self._backend_name}', "
            f"camera='{cam_topic}') -> writing {self._map_file}"
        )

    # ---------------------------------- camera ---------------------------------- #
    def _image_cb(self, msg: Image):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_proc_time < self._proc_period:
            return
        self._last_proc_time = now

        try:
            rgb = self._bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        except Exception as e:                       
            self.get_logger().warn(f'cv_bridge failed: {e}')
            return

        result = self._vlm.classify(rgb)
        if result.label == 'hallway' or result.confidence < self._min_conf:
            return

        pose = self._robot_pose()
        if pose is None:
            return
        rx, ry = pose

        prev = self._places.get(result.label)
        if prev is not None and prev['confidence'] >= result.confidence:
            return   # we already have a clearer view of this room

        self._places[result.label] = {
            'x': rx, 'y': ry,
            'confidence': round(result.confidence, 3),
            'description': result.description,
        }
        self.get_logger().info(
            f'[VLM] {result.description} (conf={result.confidence:.2f}) '
            f'-> tagged "{result.label}" at ({rx:.2f}, {ry:.2f})'
        )

        self._save_snapshot(result.label, rgb)
        self._save_map()

    # ------------------------------------ TF ------------------------------------ #
    def _robot_pose(self):
        try:
            tf = self._tf_buf.lookup_transform(
                self._map_frame, self._base_frame, rclpy.time.Time())
            t = tf.transform.translation
            return t.x, t.y
        except Exception:
            return None

    # ---------------------------------- storage --------------------------------- #
    def _save_map(self):
        data = {
            'frame_id': self._map_frame,
            'places': [
                {'label': label, **info} for label, info in self._places.items()
            ],
        }
        tmp = self._map_file + '.tmp'
        with open(tmp, 'w') as f:
            json.dump(data, f, indent=2)
        os.replace(tmp, self._map_file)

    def _save_snapshot(self, label: str, rgb):
        try:
            import cv2
            path = os.path.join(self._snapshot_dir, f'{label}.png')
            cv2.imwrite(path, cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        except Exception as e:                       # noqa: BLE001
            self.get_logger().warn(f'snapshot save failed: {e}')

    # ---------------------------------- markers --------------------------------- #
    def _publish_markers(self):
        ma = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)

        mid = 0
        for label, info in self._places.items():
            color = _MARKER_COLOR.get(label, (1.0, 1.0, 1.0, 1.0))

            # ---------------------------- Floating text label --------------------------- #
            txt = Marker()
            txt.header.frame_id = self._map_frame
            txt.header.stamp = self.get_clock().now().to_msg()
            txt.ns = 'semantic_text'
            txt.id = mid
            mid += 1
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position = Point(x=info['x'], y=info['y'], z=0.6)
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.4
            txt.color.r, txt.color.g, txt.color.b, txt.color.a = color
            txt.text = f"{label} ({info['confidence']:.2f})"
            ma.markers.append(txt)

            # ----------------------- Marker sphere at the location ---------------------- #
            sph = Marker()
            sph.header.frame_id = self._map_frame
            sph.header.stamp = self.get_clock().now().to_msg()
            sph.ns = 'semantic_dot'
            sph.id = mid
            mid += 1
            sph.type = Marker.SPHERE
            sph.action = Marker.ADD
            sph.pose.position = Point(x=info['x'], y=info['y'], z=0.2)
            sph.pose.orientation.w = 1.0
            sph.scale.x = sph.scale.y = sph.scale.z = 0.3
            sph.color.r, sph.color.g, sph.color.b, sph.color.a = color
            ma.markers.append(sph)

        self._marker_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = SemanticTagger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
