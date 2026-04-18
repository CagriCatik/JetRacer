#!/usr/bin/env python3
# Joystick teleoperation node for JetRacer (ROS 2)
# Migrated from jetracer_ros-ubuntu/scripts/teleop_joy.py (ROS 1 rospy)
# to rclpy (ROS 2).
#
# Button mapping (default — matches a standard PS3/Xbox dead-man trigger):
#   axes[3]   → forward/back (left stick vertical)
#   axes[0]   → left/right   (left stick horizontal)
#   buttons[6] → dead-man button (L2 / LT)
#
# Override via ROS 2 parameters: linear_axis, angular_axis, deadman_button.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class TeleopJoyNode(Node):
    def __init__(self) -> None:
        super().__init__('teleop_joy')

        self.declare_parameter('x_speed', 0.3)
        self.declare_parameter('y_speed', 0.0)
        self.declare_parameter('w_speed', 1.0)
        self.declare_parameter('linear_axis', 3)
        self.declare_parameter('angular_axis', 0)
        self.declare_parameter('deadman_button', 6)
        self.declare_parameter('hz', 20)

        self._x_speed = float(self.get_parameter('x_speed').value)
        self._y_speed = float(self.get_parameter('y_speed').value)
        self._w_speed = float(self.get_parameter('w_speed').value)
        self._linear_axis = int(self.get_parameter('linear_axis').value)
        self._angular_axis = int(self.get_parameter('angular_axis').value)
        self._deadman_button = int(self.get_parameter('deadman_button').value)

        hz = int(self.get_parameter('hz').value)

        self._cmd = Twist()
        self._active = False

        self._pub = self.create_publisher(Twist, 'cmd_vel_teleop', 10)
        self._sub = self.create_subscription(Joy, 'joy', self._joy_callback, 10)

        period = 1.0 / max(hz, 1)
        self._timer = self.create_timer(period, self._publish_cmd)

        self.get_logger().info(
            f'teleop_joy started — linear_axis={self._linear_axis}, '
            f'angular_axis={self._angular_axis}, '
            f'deadman_button={self._deadman_button}'
        )

    def _joy_callback(self, msg: Joy) -> None:
        """Process incoming Joy messages and update the current command."""
        if self._deadman_button >= len(msg.buttons):
            self.get_logger().warn_once(
                f'deadman_button index {self._deadman_button} out of range '
                f'(Joy has {len(msg.buttons)} buttons). '
                'Override with parameter "deadman_button".'
            )
            return

        if msg.buttons[self._deadman_button] == 1:
            self._cmd = Twist()
            if self._linear_axis < len(msg.axes):
                self._cmd.linear.x = self._x_speed * msg.axes[self._linear_axis]
            if self._angular_axis < len(msg.axes):
                self._cmd.angular.z = self._w_speed * msg.axes[self._angular_axis]
            # y_speed maps to linear.y for holonomic platforms.
            self._cmd.linear.y = self._y_speed
            self._active = True
        else:
            self._cmd = Twist()
            self._active = False

    def _publish_cmd(self) -> None:
        """Timer callback — publish the current command at the configured rate."""
        if self._active:
            self._pub.publish(self._cmd)
        else:
            # Publish an explicit stop so the hardware safety timeout is not
            # triggered solely by a silent topic.
            self._pub.publish(Twist())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TeleopJoyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Send one final stop command before shutting down.
        node._pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
