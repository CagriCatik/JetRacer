#!/usr/bin/env python3
"""
Hardware-Aware RRT* Local Path Planner

This node implements a purely local RRT* planner that uses LiDAR data to build
an egocentric occupancy grid, plans a path to a wandering goal (or avoids obstacles
while trying to drive forward), and follows the path using a built-in Pure Pursuit
controller. Output goes to `cmd_vel_rrt` (twist_mux priority 6).
"""

import math
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import tf2_ros

from geometry_msgs.msg import Twist, PointStamped, Point, PoseStamped
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Path
from visualization_msgs.msg import Marker, MarkerArray


class RRTNode:
    """A node in the RRT* tree."""
    __slots__ = ['x', 'y', 'cost', 'parent']
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y
        self.cost = 0.0
        self.parent = None


class RRTStarPlanner(Node):
    def __init__(self):
        super().__init__('rrt_star_planner')

        # Execution parameters
        self.declare_parameter('planning_frequency', 10.0)
        self.declare_parameter('max_planning_time_ms', 30)
        self.declare_parameter('max_iterations', 400)
        self.declare_parameter('dry_run', False)
        self.declare_parameter('safe_mode', False)

        # Grid parameters
        self.declare_parameter('grid_width_m', 6.0)
        self.declare_parameter('grid_height_m', 6.0)
        self.declare_parameter('grid_resolution', 0.05)
        self.declare_parameter('inflation_radius', 0.35)

        # RRT* parameters
        self.declare_parameter('step_size', 0.25)
        self.declare_parameter('search_radius', 0.6)
        self.declare_parameter('goal_tolerance', 0.3)
        self.declare_parameter('goal_sample_rate', 0.1)

        # Controller & Vehicle constraints
        self.declare_parameter('pure_pursuit_lookahead', 0.6)
        self.declare_parameter('max_speed', 0.3)
        self.declare_parameter('safe_mode_max_speed_ms', 0.20)
        self.declare_parameter('wheelbase', 0.14)
        self.declare_parameter('max_steering_angle', 0.6)
        self.declare_parameter('min_turning_radius', 0.25)

        # Toggles
        self.declare_parameter('start', False)
        self.declare_parameter('enable_markers', True)

        # Fetch params
        self._load_params()
        self.add_on_set_parameters_callback(self._param_callback)

        # TF and Subscriptions
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        
        self.create_subscription(LaserScan, 'scan', self._scan_callback, qos_profile_sensor_data)
        
        # Publishers
        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel_rrt', 10)
        self._grid_pub = self.create_publisher(OccupancyGrid, 'rrt_star/local_grid', 1)
        self._path_pub = self.create_publisher(Path, 'rrt_star/path', 1)
        self._marker_pub = self.create_publisher(MarkerArray, 'rrt_star/markers', 1)

        # Grid state
        self._grid = None
        self._last_scan_time = 0.0
        self._base_frame = 'base_footprint'

        # Main planning timer
        freq = self.get_parameter('planning_frequency').value
        self.create_timer(1.0 / freq, self._planning_loop)

        self.get_logger().info("RRT* Planner initialized.")

    def _load_params(self, updates=None):
        def fetch(name):
            return updates.get(name, self.get_parameter(name).value) if updates else self.get_parameter(name).value

        self._max_time = fetch('max_planning_time_ms') / 1000.0
        self._max_iter = fetch('max_iterations')
        self._dry_run = fetch('dry_run')
        self._safe_mode = fetch('safe_mode')
        
        self._res = fetch('grid_resolution')
        self._gw = int(fetch('grid_width_m') / self._res)
        self._gh = int(fetch('grid_height_m') / self._res)
        self._inflate_cells = int(fetch('inflation_radius') / self._res)
        
        self._step = fetch('step_size')
        self._search_r = fetch('search_radius')
        self._goal_tol = fetch('goal_tolerance')
        self._goal_rate = fetch('goal_sample_rate')
        
        self._lookahead = fetch('pure_pursuit_lookahead')
        self._max_v = fetch('max_speed')
        self._safe_v = fetch('safe_mode_max_speed_ms')
        self._wheelbase = fetch('wheelbase')
        self._max_steer = fetch('max_steering_angle')
        self._min_turn_r = fetch('min_turning_radius')
        self._start_enabled = fetch('start')
        self._show_markers = fetch('enable_markers')

        # We keep the robot at the center-bottom of the grid.
        self._origin_x = - (self._gw * self._res) * 0.1  # 10% behind robot
        self._origin_y = - (self._gh * self._res) * 0.5  # centered laterally

    def _param_callback(self, params):
        updated = {p.name: p.value for p in params}
        self._load_params(updated)
        return SetParametersResult(successful=True)

    def _world_to_grid(self, x, y):
        gx = int((x - self._origin_x) / self._res)
        gy = int((y - self._origin_y) / self._res)
        return gx, gy

    def _grid_to_world(self, gx, gy):
        x = gx * self._res + self._origin_x
        y = gy * self._res + self._origin_y
        return x, y

    def _is_valid(self, x, y):
        gx, gy = self._world_to_grid(x, y)
        if 0 <= gx < self._gw and 0 <= gy < self._gh:
            return self._grid[gy, gx] == 0
        return False

    def _check_collision(self, n1, n2):
        """Bresenham-like collision check."""
        dist = math.hypot(n2.x - n1.x, n2.y - n1.y)
        steps = int(dist / (self._res / 2.0))
        if steps == 0:
            return self._is_valid(n2.x, n2.y)
        for i in range(steps + 1):
            t = i / steps
            x = n1.x + t * (n2.x - n1.x)
            y = n1.y + t * (n2.y - n1.y)
            if not self._is_valid(x, y):
                return False
        return True

    def _scan_callback(self, msg: LaserScan):
        """Build local occupancy grid from laser scan."""
        self._last_scan_time = time.time()
        
        # Initialize an empty grid (0 = free, 100 = occupied)
        grid = np.zeros((self._gh, self._gw), dtype=np.int8)

        try:
            # Transform from laser frame to base_footprint
            trans = self._tf_buffer.lookup_transform(
                self._base_frame, msg.header.frame_id,
                rclpy.time.Time())
            
            # Simplified transform assuming 2D rigid transform (yaw & translation)
            qx = trans.transform.rotation.x
            qy = trans.transform.rotation.y
            qz = trans.transform.rotation.z
            qw = trans.transform.rotation.w
            yaw = math.atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz))
            tx = trans.transform.translation.x
            ty = trans.transform.translation.y

            angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
            ranges = np.array(msg.ranges)
            
            # Filter valid ranges
            valid = (ranges >= msg.range_min) & (ranges <= msg.range_max) & np.isfinite(ranges)
            
            # To laser frame Cartesian
            lx = ranges[valid] * np.cos(angles[valid])
            ly = ranges[valid] * np.sin(angles[valid])

            # To base footprint
            bx = tx + lx * math.cos(yaw) - ly * math.sin(yaw)
            by = ty + lx * math.sin(yaw) + ly * math.cos(yaw)

            # Map to grid
            g_xs = ((bx - self._origin_x) / self._res).astype(np.int32)
            g_ys = ((by - self._origin_y) / self._res).astype(np.int32)
            
            # Bounds check
            in_bounds = (g_xs >= 0) & (g_xs < self._gw) & (g_ys >= 0) & (g_ys < self._gh)
            g_xs = g_xs[in_bounds]
            g_ys = g_ys[in_bounds]

            # Mark occupied
            grid[g_ys, g_xs] = 100

            # Inflate obstacles using a simple morphological dilation
            if self._inflate_cells > 0:
                # We do a fast bounding-box inflation to save CPU
                inflated = np.copy(grid)
                r = self._inflate_cells
                for y, x in zip(g_ys, g_xs):
                    y0, y1 = max(0, y - r), min(self._gh, y + r + 1)
                    x0, x1 = max(0, x - r), min(self._gw, x + r + 1)
                    inflated[y0:y1, x0:x1] = 100
                grid = inflated

            self._grid = grid

            # Publish OccupancyGrid for visualization
            if self._show_markers:
                grid_msg = OccupancyGrid()
                grid_msg.header.stamp = self.get_clock().now().to_msg()
                grid_msg.header.frame_id = self._base_frame
                grid_msg.info.resolution = self._res
                grid_msg.info.width = self._gw
                grid_msg.info.height = self._gh
                grid_msg.info.origin.position.x = self._origin_x
                grid_msg.info.origin.position.y = self._origin_y
                grid_msg.data = grid.flatten().tolist()
                self._grid_pub.publish(grid_msg)

        except Exception as e:
            self.get_logger().warn(f"Failed to process scan: {e}")

    def _publish_stop(self):
        msg = Twist()
        self._cmd_pub.publish(msg)

    def _planning_loop(self):
        if not self._start_enabled:
            return

        if self._grid is None or (time.time() - self._last_scan_time > 1.0):
            self.get_logger().warn("Stale LiDAR data, stopping.", throttle_duration_sec=2.0)
            self._publish_stop()
            return

        start_time = time.time()
        
        # Exploratory local goal (e.g. 3m straight ahead)
        goal_x, goal_y = 3.0, 0.0
        
        # If the forward goal is occupied, sample a random valid target in the front half
        if not self._is_valid(goal_x, goal_y):
            for _ in range(10):
                rx = np.random.uniform(1.0, self._gw * self._res + self._origin_x)
                ry = np.random.uniform(self._origin_y, self._origin_y + self._gh * self._res)
                if self._is_valid(rx, ry):
                    goal_x, goal_y = rx, ry
                    break

        nodes = [RRTNode(0.0, 0.0)]
        
        # Fast numpy array for nearest neighbor search
        nodes_xy = np.zeros((self._max_iter + 1, 2))
        nodes_xy[0] = [0.0, 0.0]
        
        goal_reached = False
        best_node = None

        for i in range(1, self._max_iter):
            if time.time() - start_time > self._max_time:
                break

            # Sample
            if np.random.rand() < self._goal_rate:
                rnd_x, rnd_y = goal_x, goal_y
            else:
                rnd_x = np.random.uniform(self._origin_x, self._origin_x + self._gw * self._res)
                rnd_y = np.random.uniform(self._origin_y, self._origin_y + self._gh * self._res)

            # Nearest
            active_xy = nodes_xy[:len(nodes)]
            dists = np.sum((active_xy - [rnd_x, rnd_y])**2, axis=1)
            nearest_idx = np.argmin(dists)
            nearest = nodes[nearest_idx]

            # Steer
            theta = math.atan2(rnd_y - nearest.y, rnd_x - nearest.x)
            new_node = RRTNode(
                nearest.x + self._step * math.cos(theta),
                nearest.y + self._step * math.sin(theta)
            )

            # Kinematic constraint: prevent sharp turns (Ackermann limitation)
            yaw_nearest = 0.0
            if nearest.parent is not None:
                yaw_nearest = math.atan2(nearest.y - nearest.parent.y, nearest.x - nearest.parent.x)
            
            angle_diff = abs(math.atan2(math.sin(theta - yaw_nearest), math.cos(theta - yaw_nearest)))
            # A rough kinematic limit (JetRacer cannot turn on the spot)
            if nearest.parent is not None and angle_diff > math.radians(45.0):
                continue

            # Collision check
            if not self._check_collision(nearest, new_node):
                continue

            # Add node
            new_node.cost = nearest.cost + self._step
            new_node.parent = nearest
            nodes.append(new_node)
            nodes_xy[len(nodes)-1] = [new_node.x, new_node.y]

            # Rewire (RRT*)
            active_xy = nodes_xy[:len(nodes)-1]
            dists = np.sum((active_xy - [new_node.x, new_node.y])**2, axis=1)
            near_indices = np.where(dists <= self._search_r**2)[0]

            for n_idx in near_indices:
                near_node = nodes[n_idx]
                d = math.hypot(new_node.x - near_node.x, new_node.y - near_node.y)
                
                # Rewire to new_node as parent
                if new_node.cost + d < near_node.cost and self._check_collision(new_node, near_node):
                    # Kinematic check for rewire
                    yaw_new = math.atan2(new_node.y - new_node.parent.y, new_node.x - new_node.parent.x)
                    theta_rewire = math.atan2(near_node.y - new_node.y, near_node.x - new_node.x)
                    if abs(math.atan2(math.sin(theta_rewire - yaw_new), math.cos(theta_rewire - yaw_new))) < math.radians(45.0):
                        near_node.parent = new_node
                        near_node.cost = new_node.cost + d

            # Goal check
            if math.hypot(new_node.x - goal_x, new_node.y - goal_y) <= self._goal_tol:
                goal_reached = True
                best_node = new_node
                break

        # If goal not reached, find node closest to goal
        if not goal_reached and len(nodes) > 1:
            active_xy = nodes_xy[:len(nodes)]
            dists = np.sum((active_xy - [goal_x, goal_y])**2, axis=1)
            best_node = nodes[np.argmin(dists)]

        if best_node is None or best_node.parent is None:
            self.get_logger().warn("No path found!", throttle_duration_sec=2.0)
            self._publish_stop()
            return

        # Backtrack path
        path_nodes = []
        curr = best_node
        while curr is not None:
            path_nodes.append(curr)
            curr = curr.parent
        path_nodes.reverse()

        # Execute pure pursuit tracking
        self._execute_path(path_nodes)
        
        # Visualizations
        if self._show_markers:
            self._publish_visuals(path_nodes, nodes, goal_x, goal_y)

    def _execute_path(self, path_nodes):
        """Pure Pursuit path following."""
        if len(path_nodes) < 2:
            self._publish_stop()
            return

        # Find lookahead point
        target = path_nodes[-1]
        for node in path_nodes:
            d = math.hypot(node.x, node.y)
            if d >= self._lookahead:
                target = node
                break

        # Calculate steering (Pure Pursuit equation)
        # alpha = angle to target. Ld = distance to target.
        # steering = atan2(2 * L * sin(alpha) / Ld)
        Ld = math.hypot(target.x, target.y)
        if Ld < 0.05:
            self._publish_stop()
            return

        alpha = math.atan2(target.y, target.x)
        steering_angle = math.atan(2.0 * self._wheelbase * math.sin(alpha) / Ld)
        
        # Clamp steering
        steering_angle = max(min(steering_angle, self._max_steer), -self._max_steer)

        # Speed scaling based on curvature
        curvature_penalty = 1.0 - (abs(steering_angle) / self._max_steer) * 0.5
        v_target = (self._safe_v if self._safe_mode else self._max_v) * curvature_penalty

        msg = Twist()
        if not self._dry_run:
            msg.linear.x = float(v_target)
            msg.angular.z = float(steering_angle)
        self._cmd_pub.publish(msg)

    def _publish_visuals(self, path_nodes, all_nodes, gx, gy):
        now = self.get_clock().now().to_msg()

        # Publish nav_msgs/Path
        path_msg = Path()
        path_msg.header.stamp = now
        path_msg.header.frame_id = self._base_frame
        for node in path_nodes:
            pose = PoseStamped()
            pose.header.stamp = now
            pose.header.frame_id = self._base_frame
            pose.pose.position.x = node.x
            pose.pose.position.y = node.y
            path_msg.poses.append(pose)
        self._path_pub.publish(path_msg)

        # Publish MarkerArray (Tree and Goal)
        markers = MarkerArray()
        
        # Tree
        tree_marker = Marker()
        tree_marker.header.frame_id = self._base_frame
        tree_marker.header.stamp = now
        tree_marker.ns = 'rrt_tree'
        tree_marker.id = 0
        tree_marker.type = Marker.LINE_LIST
        tree_marker.action = Marker.ADD
        tree_marker.scale.x = 0.01
        tree_marker.color.a = 0.3
        tree_marker.color.r = 0.0
        tree_marker.color.g = 1.0
        tree_marker.color.b = 0.0
        
        for n in all_nodes:
            if n.parent is not None:
                p1 = Point(x=n.x, y=n.y, z=0.0)
                p2 = Point(x=n.parent.x, y=n.parent.y, z=0.0)
                tree_marker.points.extend([p1, p2])
        markers.markers.append(tree_marker)

        # Goal
        goal_marker = Marker()
        goal_marker.header.frame_id = self._base_frame
        goal_marker.header.stamp = now
        goal_marker.ns = 'rrt_goal'
        goal_marker.id = 1
        goal_marker.type = Marker.SPHERE
        goal_marker.action = Marker.ADD
        goal_marker.pose.position.x = gx
        goal_marker.pose.position.y = gy
        goal_marker.scale.x = 0.2
        goal_marker.scale.y = 0.2
        goal_marker.scale.z = 0.2
        goal_marker.color.a = 0.8
        goal_marker.color.r = 1.0
        goal_marker.color.g = 0.5
        goal_marker.color.b = 0.0
        markers.markers.append(goal_marker)

        self._marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = RRTStarPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
