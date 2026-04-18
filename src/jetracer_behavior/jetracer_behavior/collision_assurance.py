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
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.timer import Timer
from sensor_msgs.msg import LaserScan


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

        self._sub = self.create_subscription(LaserScan, 'scan', self._scan_callback, 10)
        self._pub = self.create_publisher(Twist, 'cmd_vel_behavior', 10)

        self._is_blocked: bool = False

        # Failsafe loop: When blocked, spam Priority 8 Twist zeros aggressively
        self._publish_timer: Timer = self.create_timer(0.05, self._publish_loop)

        self.get_logger().info("LiDAR Collision Assurance initialized.")

    def _load_params(self) -> None:
        """Translates degrees into radians and updates class constraints."""
        # A 30 degree cone implies ±15 degrees from straight forward.
        self._cone_rad: float = math.radians(float(self.get_parameter('cone_angle_deg').value) / 2.0)
        self._min_dist: float = float(self.get_parameter('min_distance_m').value)

    def _param_callback(self, params) -> SetParametersResult:
        self._load_params()
        return SetParametersResult(successful=True)

    def _scan_callback(self, msg: LaserScan) -> None:
        """
        Parses the RPLidar geometry matrix.
        Maps the polar distances to Cartesian equivalents via angular increments.
        """
        blocked: bool = False
        angle: float = msg.angle_min

        for r in msg.ranges:
            if not math.isinf(r) and not math.isnan(r) and r > msg.range_min:
                
                # Normalize angle to map strictly between -pi and +pi.
                # Assuming 0 is facing directly forward.
                norm_angle: float = math.atan2(math.sin(angle), math.cos(angle))
                
                if abs(norm_angle) <= self._cone_rad:
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
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
