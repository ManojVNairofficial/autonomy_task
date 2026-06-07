import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

from .rrt import RRTPlanner


class RRTPlannerNode(Node):
    """
    # -------------------------------- Subscribers ------------------------------- #

    /map            nav_msgs/OccupancyGrid    – receives and stores occupancy grid
    /start_pose     geometry_msgs/PoseStamped – planning start
    /goal_pose      geometry_msgs/PoseStamped – planning goal; triggers RRT on receipt

    # -------------------------------- Publishers -------------------------------- #

    /rrt_path       nav_msgs/Path                  – planned path (frame_id = map)
    /rrt_tree       visualization_msgs/MarkerArray – full tree for RViz (LINE_LIST)

    # -------------------------------- Parameters -------------------------------- #
    
    step_size        float  -   metres between tree nodes
    max_iterations   int    -   RRT iteration cap
    goal_bias        float  -   fraction of samples that equal the goal
    goal_threshold   float  -   distance at which goal is considered reached
    inflation_radius float  -   obstacle dilation radius (>= robot radius 0.105 m)

    """

    def __init__(self):
        super().__init__('rrt_planner_node')

        self.declare_parameter('step_size',        0.1)
        self.declare_parameter('max_iterations',   5000)
        self.declare_parameter('goal_bias',        0.1)
        self.declare_parameter('goal_threshold',   0.2)
        self.declare_parameter('inflation_radius', 0.15)

        step_size      = self.get_parameter('step_size').value
        max_iter       = self.get_parameter('max_iterations').value
        goal_bias      = self.get_parameter('goal_bias').value
        goal_threshold = self.get_parameter('goal_threshold').value
        self._inflation = self.get_parameter('inflation_radius').value

        self._planner    = RRTPlanner(step_size, max_iter, goal_bias, goal_threshold)
        self._map_ready  = False
        self._start_pose = None

        map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(OccupancyGrid, '/map',        self._map_cb,   map_qos)
        self.create_subscription(PoseStamped,   '/start_pose', self._start_cb, 1)
        self.create_subscription(PoseStamped,   '/goal_pose',  self._goal_cb,  1)

        self._path_pub = self.create_publisher(Path,        '/rrt_path', 1)
        self._tree_pub = self.create_publisher(MarkerArray, '/rrt_tree', 1)

        self.get_logger().info(
            'RRT Planner Node ready — awaiting /map, /start_pose, /goal_pose'
        )


        # ---------------------------------------------------------------------------- #
        #                                   Callbacks                                  #
        # ---------------------------------------------------------------------------- #

    def _map_cb(self, msg: OccupancyGrid) -> None:
        self.get_logger().info(
            f'Map received: {msg.info.width}×{msg.info.height} cells, '
            f'res={msg.info.resolution:.4f} m/cell, '
            f'origin=({msg.info.origin.position.x:.2f}, '
            f'{msg.info.origin.position.y:.2f})'
        )
        inflated = self._inflate_obstacles(
            msg.data,
            msg.info.width, msg.info.height,
            msg.info.resolution, self._inflation,
        )
        self._planner.set_map(
            inflated,
            msg.info.width, msg.info.height,
            msg.info.resolution,
            msg.info.origin.position.x,
            msg.info.origin.position.y,
        )
        self._map_ready = True

    def _start_cb(self, msg: PoseStamped) -> None:
        self._start_pose = msg
        self.get_logger().info(
            f'Start pose set: '
            f'({msg.pose.position.x:.3f}, {msg.pose.position.y:.3f})'
        )

    def _goal_cb(self, msg: PoseStamped) -> None:
        gx, gy = msg.pose.position.x, msg.pose.position.y
        self.get_logger().info(f'Goal pose received: ({gx:.3f}, {gy:.3f})')

        if not self._map_ready:
            self.get_logger().warn('Map not yet available — ignoring goal.')
            return
        if self._start_pose is None:
            self.get_logger().warn('Start pose not yet set — ignoring goal.')
            return

        self._run_rrt(self._start_pose, msg)

   # ---------------------------------------------------------------------------- #
   #                                 CORE PLANNING                                #
   # ---------------------------------------------------------------------------- #

    def _run_rrt(self, start: PoseStamped, goal: PoseStamped) -> None:
        sx, sy = start.pose.position.x, start.pose.position.y
        gx, gy = goal.pose.position.x,  goal.pose.position.y

        self.get_logger().info(
            f'Running RRT: ({sx:.2f},{sy:.2f}) → ({gx:.2f},{gy:.2f}) …'
        )

        path, nodes = self._planner.plan(sx, sy, gx, gy)

        self._publish_tree(nodes)

        if path is None:
            self.get_logger().warn(
                f'RRT exhausted {self._planner.max_iter} iterations — '
                'no path found. Try increasing max_iterations or step_size.'
            )
            return

        self.get_logger().info(
            f'Path found: {len(path)} waypoints, '
            f'tree size: {len(nodes)} nodes.'
        )
        self._publish_path(path)


    # ---------------------------------------------------------------------------- #
    #                              Obstacle inflation                              #
    # ---------------------------------------------------------------------------- #


    @staticmethod
    def _inflate_obstacles(
        grid_data,
        width: int,
        height: int,
        resolution: float,
        radius: float,
    ) -> list:
        grid = np.array(grid_data, dtype=np.int8).reshape(height, width)
        inflated = grid.copy()
        cells = int(math.ceil(radius / resolution))

        # ------------------- Dilate every occupied or unknown cell ------------------ #
        occ_rows, occ_cols = np.where((grid == 100) | (grid == -1))
        for r, c in zip(occ_rows.tolist(), occ_cols.tolist()):
            r0 = max(0,      r - cells)
            r1 = min(height, r + cells + 1)
            c0 = max(0,      c - cells)
            c1 = min(width,  c + cells + 1)
            inflated[r0:r1, c0:c1] = 100

        return inflated.flatten().tolist()

    # ---------------------------------------------------------------------------- #
    #                                  Publishers                                  #
    # ---------------------------------------------------------------------------- #

    def _publish_path(self, waypoints: list) -> None:
        msg = Path()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        for x, y in waypoints:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x  = x
            ps.pose.position.y  = y
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)

        self._path_pub.publish(msg)

    def _publish_tree(self, nodes: list) -> None:
        marker = Marker()
        marker.header.stamp    = self.get_clock().now().to_msg()
        marker.header.frame_id = 'map'
        marker.ns              = 'rrt_tree'
        marker.id              = 0
        marker.type            = Marker.LINE_LIST
        marker.action          = Marker.ADD
        marker.scale.x         = 0.05   # line width in metres
        marker.color.r         = 0.0
        marker.color.g         = 0.8
        marker.color.b         = 0.8
        marker.color.a         = 0.5

        for node in nodes:
            if node.parent is not None:
                parent = nodes[node.parent]
                p1, p2 = Point(), Point()
                p1.x, p1.y = parent.x, parent.y
                p2.x, p2.y = node.x,   node.y
                marker.points.extend([p1, p2])

        ma = MarkerArray()
        ma.markers.append(marker)
        self._tree_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = RRTPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
