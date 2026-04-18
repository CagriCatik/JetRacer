#!/usr/bin/env python3
"""
Ackermann Kinematics Converter

This node serves as the crucial mathematical bridge between the Navigation 2 (Nav2) outputs
and the physical RP2040 hardware chassis. While Nav2 computes routes using generic Yaw Rates 
(angular.z), a hardware car cannot rotate in place; it must turn its front wheels.

We compute the Ackermann Steering Angle mathematically:
    steering_angle = arctan(wheelbase * angular_z / linear_x)
This ensures physical geometric compliance before routing to the twist_mux.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node


class CmdVelToSteeringNode(Node):
    """
    Translates raw diff-drive Twist commands into pure Ackermann Twist commands
    mapped specifically to physical wheelbase dimensions.
    """

    def __init__(self) -> None:
        """Initializes the node, constants, and sets up networking paths."""
        super().__init__('cmd_vel_to_steering')
        
        self.declare_parameter('input_topic', 'cmd_vel_nav')
        self.declare_parameter('output_topic', 'cmd_vel_nav_steer')
        self.declare_parameter('wheelbase', 0.255)
        self.declare_parameter('max_steering_angle', 0.60)
        self.declare_parameter('min_speed_for_steering', 0.05)

        input_topic: str = str(self.get_parameter('input_topic').value)
        output_topic: str = str(self.get_parameter('output_topic').value)
        self._load_params()
        self.add_on_set_parameters_callback(self._param_callback)

        self.publisher = self.create_publisher(Twist, output_topic, 10)
        self.subscription = self.create_subscription(Twist, input_topic, self.callback, 10)

        self.get_logger().info("Ackermann Inverse-Kinematics Node Online.")

    def _load_params(self) -> None:
        """Load/reload tunable parameters from the ROS 2 parameter store."""
        self.wheelbase: float = float(self.get_parameter('wheelbase').value)
        self.max_steering_angle: float = float(self.get_parameter('max_steering_angle').value)
        self.min_speed: float = float(self.get_parameter('min_speed_for_steering').value)

    def _param_callback(self, params) -> SetParametersResult:
        """Live-update parameters without restarting the node."""
        self._load_params()
        self.get_logger().info(
            f'cmd_vel_to_steering params updated: wheelbase={self.wheelbase:.3f} '
            f'max_steering={self.max_steering_angle:.3f} min_speed={self.min_speed:.3f}'
        )
        return SetParametersResult(successful=True)

    def callback(self, msg: Twist) -> None:
        """
        Receives raw twist message. Calculates explicit wheel angle required 
        to execute the requested yaw rate at current linear speed.
        """
        command = Twist()
        # Preserve linear velocity exactly
        command.linear = msg.linear
        command.angular = msg.angular

        linear_velocity: float = msg.linear.x
        
        # Prevent Division-by-Zero singularity when stopped
        if abs(linear_velocity) < self.min_speed:
            steering: float = 0.0
        else:
            # Core Ackermann inverse kinematics equation
            steering = math.atan(self.wheelbase * msg.angular.z / linear_velocity)

        # Strictly clamp output to physical mechanical limits of servo to avoid burnout
        steering = max(-self.max_steering_angle, min(self.max_steering_angle, steering))
        
        # Override the yaw rate with the required mechanical steering angle
        command.angular.z = steering
        self.publisher.publish(command)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelToSteeringNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
