#!/usr/bin/env python3
"""
Wheel Slip Monitoring Node

Compares physical yaw rate from the onboard AHRS/IMU against the theoretical
yaw rate derived from wheel encoders (Odometry). Significant divergence indicates
wheel slip (drifting or spinning), which can corrupt localization and navigation.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool


class SlipMonitorNode(Node):
    """
    Subscribes to IMU and Odometry. Detects divergence in yaw-rate reporting.
    """

    def __init__(self) -> None:
        super().__init__('slip_monitor')
        
        self.declare_parameter('diff_threshold', 0.3)   # rad/s difference to trigger warning
        self.declare_parameter('min_velocity', 0.1)     # m/s minimum speed to avoid noise at rest
        self.declare_parameter('window_size', 5)        # Number of samples to average for smoothing

        self._load_params()
        self.add_on_set_parameters_callback(self._param_callback)
        
        # Networking
        self._imu_sub = self.create_subscription(
            Imu, 'imu', self._imu_callback, qos_profile_sensor_data)
        self._odom_sub = self.create_subscription(
            Odometry, 'odom_raw', self._odom_callback, qos_profile_sensor_data)
        
        self._slip_pub = self.create_publisher(Bool, '/control/slipping', 10)

        # State cache
        self._last_imu_yaw_rate: float = 0.0
        self._last_odom_yaw_rate: float = 0.0
        self._last_odom_speed: float = 0.0
        
        self._diff_history = []
        
        self.get_logger().info("Wheel Slip Monitor initialized.")

    def _load_params(self, updates: dict[str, object] | None = None) -> None:
        if updates is None:
            updates = {}

        def fetch(name: str):
            if name in updates:
                return updates[name]
            return self.get_parameter(name).value

        self._threshold = float(fetch('diff_threshold'))
        self._min_speed = float(fetch('min_velocity'))
        self._window = int(fetch('window_size'))

    def _param_callback(self, params) -> SetParametersResult:
        from rcl_interfaces.msg import SetParametersResult
        updated = {p.name: p.value for p in params}
        self._load_params(updates=updated)
        return SetParametersResult(successful=True)

    def _imu_callback(self, msg: Imu) -> None:
        self._last_imu_yaw_rate = msg.angular_velocity.z
        self._process_sanity_check()

    def _odom_callback(self, msg: Odometry) -> None:
        self._last_odom_yaw_rate = msg.twist.twist.angular.z
        self._last_odom_speed = math.sqrt(msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2)
        self._process_sanity_check()

    def _process_sanity_check(self) -> None:
        """
        Core logic: Check if IMU and Odom report a consistent reality.
        """
        # Only check if moving significantly to avoid gimbal/encoder jitter at rest
        if self._last_odom_speed < self._min_speed:
            self._publish_slip(False)
            return

        diff = abs(self._last_imu_yaw_rate - self._last_odom_yaw_rate)
        
        # Simple moving average for smoothing
        self._diff_history.append(diff)
        if len(self._diff_history) > self._window:
            self._diff_history.pop(0)
            
        avg_diff = sum(self._diff_history) / len(self._diff_history)
        
        slipping = (avg_diff > self._threshold)
        
        if slipping:
            self.get_logger().warn(
                f"SLIP DETECTED! IMU Rate: {self._last_imu_yaw_rate:.2f} | Odom Rate: {self._last_odom_yaw_rate:.2f}",
                throttle_duration_sec=2.0
            )

        self._publish_slip(slipping)

    def _publish_slip(self, state: bool) -> None:
        msg = Bool()
        msg.data = state
        self._slip_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SlipMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
