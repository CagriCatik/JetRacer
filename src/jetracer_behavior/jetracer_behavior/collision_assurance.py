#!/usr/bin/env python3
"""
Physical Collision Assurance Node

This module subscribes to the hardware RPLidar array. It establishes an active,
parameterized mathematical frustum (cone) in front of the vehicle. If any point
inside this cone registers an obstacle below the minimum distance threshold, 
it aggressively overtakes the twist_mux (Priority 8) to halt the motors.
"""

import math

import rclpy
import tf2_ros
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from rclpy.timer import Timer
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class CollisionAssuranceNode(Node):
    """
    Evaluates 2D Laser Scan messages. Defines a mechanical forcefield to prevent
    the AI camera stack from blindly crashing into physical geometry.
    """

    def __init__(self) -> None:
        """Initializes thresholds and networking for the collision cone."""
        super().__init__('collision_assurance')
        
        self.declare_parameter('cone_angle_deg', 30.0)  # Total cone array size (±15 degrees from center)
        self.declare_parameter('min_distance_m', 0.45)

        self._load_params()
        self.add_on_set_parameters_callback(self._param_callback)

        self._timer_group = MutuallyExclusiveCallbackGroup()
        self._sub_group = MutuallyExclusiveCallbackGroup()

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._sub = self.create_subscription(LaserScan, 'scan', self._scan_callback, qos_profile_sensor_data, callback_group=self._sub_group)
        self._cmd_sub = self.create_subscription(Twist, 'cmd_vel', self._cmd_callback, 10, callback_group=self._sub_group)
        self._pub = self.create_publisher(Twist, 'cmd_vel_safety', 10)
        self._state_pub = self.create_publisher(Bool, '/control/collision_blocked', 10)

        self._is_blocked: bool = False
        self._cone_offset: float = 0.0  # Dynamic center of the detection cone

        # Failsafe loop: When blocked, spam Priority 8 Twist zeros aggressively
        self._publish_timer: Timer = self.create_timer(0.05, self._publish_loop, callback_group=self._timer_group)

        self.get_logger().info("LiDAR Collision Assurance initialized.")

    def _load_params(self, updates: dict[str, object] | None = None) -> None:
        """Translates degrees into radians and updates class constraints."""
        if updates is None:
            updates = {}

        def fetch(name: str):
            if name in updates:
                return updates[name]
            return self.get_parameter(name).value

        # A 30 degree cone implies ±15 degrees from straight forward.
        self._cone_rad = math.radians(float(fetch('cone_angle_deg')) / 2.0)
        self._min_dist = float(fetch('min_distance_m'))

    def _param_callback(self, params) -> SetParametersResult:
        updated = {p.name: p.value for p in params}
        self._load_params(updates=updated)
        return SetParametersResult(successful=True)

    def _cmd_callback(self, msg: Twist) -> None:
        """
        Calculates a dynamic steering offset for the LiDAR cone.
        As the car turns, we shift the 'Safe frustum' to look into the curve.
        """
        # We cap the offset at ±cone_angle/2 to avoid looking backward.
        # Sensitivity gain: Maps yaw-rate rad/s to cone-offset rad.
        sensitivity = 0.5 
        raw_offset = msg.angular.z * sensitivity
        self._cone_offset = max(min(raw_offset, self._cone_rad), -self._cone_rad)

    def _scan_callback(self, msg: LaserScan) -> None:
        """
        Parses the RPLidar geometry matrix.
        Maps the polar distances to Cartesian equivalents via angular increments.
        """
        # Expert TF integration: Dynamically read LiDAR rotation assuming geometry might change
        try:
            trans = self._tf_buffer.lookup_transform('base_footprint', msg.header.frame_id, rclpy.time.Time())
            q = trans.transform.rotation
            # Manual euler from quaternion for yaw (Z-axis rotation) to avoid tf_transformations pkg dependency
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw_offset = math.atan2(siny_cosp, cosy_cosp)
        except Exception:
            # Fallback to zero if TF tree is missing
            yaw_offset = 0.0

        blocked: bool = False
        angle: float = msg.angle_min

        for r in msg.ranges:
            # Guard: skip inf, nan, below range_min, and at/above range_max
            # (many LiDAR drivers encode out-of-range returns as range_max, not inf)
            if (not math.isinf(r) and not math.isnan(r)
                    and r > msg.range_min and r < msg.range_max):

                # Vector ray against the dynamic base_footprint transformation
                real_angle = angle + yaw_offset
                norm_angle: float = math.atan2(math.sin(real_angle), math.cos(real_angle))

                # Apply dynamic frustum logic: check if point is within the shifted cone
                if abs(norm_angle - self._cone_offset) <= self._cone_rad:
                    if r < self._min_dist:
                        blocked = True
                        break
            angle += msg.angle_increment

        # Edges trigger logging to trace the state machine manually
        if blocked and not self._is_blocked:
            self.get_logger().error(f"COLLISION RISK: Obstacle detected within {self._min_dist}m! Braking!")
        elif not blocked and self._is_blocked:
            self.get_logger().info("Path clear. Releasing brakes.")

        self._is_blocked = blocked
        state_msg = Bool()
        state_msg.data = blocked
        self._state_pub.publish(state_msg)

    def _publish_loop(self) -> None:
        """Drives the Twist Multiplexer when blocking conditions are met."""
        if self._is_blocked:
            t = Twist()
            t.linear.x = 0.0
            t.angular.z = 0.0
            self._pub.publish(t)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CollisionAssuranceNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
