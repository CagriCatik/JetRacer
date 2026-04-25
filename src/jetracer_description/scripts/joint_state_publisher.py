#!/usr/bin/env python3
"""
JetRacer Joint State Publisher
================================
Publishes sensor_msgs/JointState from cmd_vel so the robot model in RViz2
animates during real operation, hardware dry-run, or replay from a bag file.

This node is VISUALIZATION-ONLY. It does not send any commands to hardware.
It can be disabled at any time without affecting autonomous or manual driving.

Origin / migration note
-----------------------
Inspired by the `joint_state.py` script in the `cytron_jetracer-master` ROS 1
tutorial package. That script subscribed to custom Float32 topics (`throttle`,
`steering`) and used `rospy`. This ROS 2 re-implementation subscribes to the
standard `cmd_vel` (geometry_msgs/Twist) that the current stack already publishes
and uses the correct joint names from `jetracer.urdf.xacro`.

Joint mapping
-------------
  cmd_vel.angular.z × steering_scale  → front_left_steering_joint
                                      → front_right_steering_joint (Ackermann: same angle approx)
  integrated cmd_vel.linear.x         → rear_left_wheel_joint (continuous rotation)
                                      → rear_right_wheel_joint
  front wheel joints (fl/fr_wheel_joint) are driven at the same rate as rear wheels.

All joint names match `jetracer.urdf.xacro` exactly:
  front_left_steering_joint
  front_right_steering_joint
  front_left_wheel_joint
  front_right_wheel_joint
  rear_left_wheel_joint
  rear_right_wheel_joint
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState


class JetRacerJointStatePublisher(Node):

    # Joint names must exactly match jetracer.urdf.xacro
    STEERING_JOINTS = [
        'front_left_steering_joint',
        'front_right_steering_joint',
    ]
    WHEEL_JOINTS = [
        'front_left_wheel_joint',
        'front_right_wheel_joint',
        'rear_left_wheel_joint',
        'rear_right_wheel_joint',
    ]

    def __init__(self):
        super().__init__('jetracer_joint_state_publisher')

        # Parameters
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('publish_rate_hz', 20.0)
        # cmd_vel.angular.z → steering joint position (radians)
        # The URDF limits are ±0.60 rad; cmd_vel.angular.z is already in radians
        self.declare_parameter('steering_scale', 1.0)
        # Wheel radius in metres (matches URDF: 0.035 m)
        self.declare_parameter('wheel_radius_m', 0.035)
        # Decay: how quickly steering returns to zero when no cmd_vel arrives
        self.declare_parameter('steering_decay_hz', 5.0)

        self._cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self._rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self._steering_scale = float(self.get_parameter('steering_scale').value)
        self._wheel_radius = float(self.get_parameter('wheel_radius_m').value)
        self._steering_decay_hz = float(self.get_parameter('steering_decay_hz').value)

        # State
        self._steering_rad: float = 0.0
        self._wheel_pos: float = 0.0   # accumulated wheel rotation (radians)
        self._last_cmd_vel_time: float | None = None
        self._last_tick_wall: float = time.monotonic()
        self._cmd_linear_x: float = 0.0
        self._cmd_angular_z: float = 0.0

        # Publishers / subscribers
        self._pub = self.create_publisher(JointState, 'joint_states', 10)
        self._sub = self.create_subscription(
            Twist,
            self._cmd_vel_topic,
            self._cmd_vel_callback,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT),
        )

        self._timer = self.create_timer(1.0 / self._rate_hz, self._timer_callback)

        self.get_logger().info(
            f"JetRacer joint state publisher online "
            f"(cmd_vel='{self._cmd_vel_topic}', "
            f"rate={self._rate_hz:.0f} Hz, "
            f"steering_scale={self._steering_scale:.2f}, "
            f"wheel_radius={self._wheel_radius:.3f} m)"
        )

    def _cmd_vel_callback(self, msg: Twist) -> None:
        self._cmd_linear_x = float(msg.linear.x)
        self._cmd_angular_z = float(msg.angular.z)
        self._last_cmd_vel_time = time.monotonic()

    def _timer_callback(self) -> None:
        now_wall = time.monotonic()
        dt = now_wall - self._last_tick_wall
        self._last_tick_wall = now_wall

        # Determine if cmd_vel is stale (>0.5 s old)
        cmd_stale = (
            self._last_cmd_vel_time is None
            or (now_wall - self._last_cmd_vel_time) > 0.5
        )

        # Steering: follow cmd_vel.angular.z; decay to 0 if stale
        target_steer = (
            0.0 if cmd_stale
            else self._cmd_angular_z * self._steering_scale
        )
        # Clamp to URDF limits
        target_steer = max(-0.60, min(0.60, target_steer))

        # Simple low-pass toward target (steering_decay_hz sets how fast it decays)
        alpha = min(1.0, dt * self._steering_decay_hz)
        self._steering_rad = (1.0 - alpha) * self._steering_rad + alpha * target_steer

        # Wheel position: integrate linear velocity
        linear_x = 0.0 if cmd_stale else self._cmd_linear_x
        if self._wheel_radius > 0.0:
            angular_vel = linear_x / self._wheel_radius
        else:
            angular_vel = 0.0
        self._wheel_pos += angular_vel * dt

        # Build and publish JointState
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = (self.STEERING_JOINTS + self.WHEEL_JOINTS)
        js.position = [
            self._steering_rad,   # front_left_steering_joint
            self._steering_rad,   # front_right_steering_joint (approx Ackermann)
            self._wheel_pos,      # front_left_wheel_joint
            self._wheel_pos,      # front_right_wheel_joint
            self._wheel_pos,      # rear_left_wheel_joint
            self._wheel_pos,      # rear_right_wheel_joint
        ]
        js.velocity = [0.0] * 6
        js.effort = []
        self._pub.publish(js)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JetRacerJointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
