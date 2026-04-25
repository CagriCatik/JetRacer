#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Point, Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf2_ros import Buffer, TransformException, TransformListener


class CalibrateLinearNode(Node):
    def __init__(self) -> None:
        super().__init__('calibrate_linear')
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('test_distance', 1.0)
        self.declare_parameter('speed', 0.3)
        self.declare_parameter('tolerance', 0.03)
        self.declare_parameter('odom_linear_scale_correction', 1.0)
        self.declare_parameter('start_test', False)
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('odom_frame', 'odom')

        self._load_parameters()
        self.add_on_set_parameters_callback(self._parameter_callback)

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 5)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.start_position: Optional[Point] = None
        self.timer = self.create_timer(1.0 / max(self.rate, 1e-6), self._update)
        self.get_logger().info('calibrate_linear ready. Set start_test=true to begin.')

    def _load_parameters(self) -> None:
        self.rate = float(self.get_parameter('rate').value)
        self.test_distance = float(self.get_parameter('test_distance').value)
        self.speed = float(self.get_parameter('speed').value)
        self.tolerance = float(self.get_parameter('tolerance').value)
        self.odom_linear_scale_correction = float(self.get_parameter('odom_linear_scale_correction').value)
        self.start_test = bool(self.get_parameter('start_test').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)

    def _parameter_callback(self, parameters):
        updated = {
            'rate': self.rate,
            'test_distance': self.test_distance,
            'speed': self.speed,
            'tolerance': self.tolerance,
            'odom_linear_scale_correction': self.odom_linear_scale_correction,
            'start_test': self.start_test,
            'base_frame': self.base_frame,
            'odom_frame': self.odom_frame,
        }
        for parameter in parameters:
            name = parameter.name
            if name not in updated:
                continue
            try:
                if name in {'rate', 'test_distance', 'speed', 'tolerance', 'odom_linear_scale_correction'}:
                    updated[name] = float(parameter.value)
                elif name == 'start_test':
                    updated[name] = bool(parameter.value)
                else:
                    updated[name] = str(parameter.value)
            except (TypeError, ValueError):
                return SetParametersResult(
                    successful=False,
                    reason=f'invalid value for parameter {name}',
                )

        if updated['rate'] <= 0.0:
            return SetParametersResult(successful=False, reason='rate must be > 0.0')
        if updated['tolerance'] < 0.0:
            return SetParametersResult(successful=False, reason='tolerance must be >= 0.0')
        if updated['speed'] <= 0.0:
            return SetParametersResult(successful=False, reason='speed must be > 0.0')

        previous_rate = self.rate
        self.rate = updated['rate']
        self.test_distance = updated['test_distance']
        self.speed = updated['speed']
        self.tolerance = updated['tolerance']
        self.odom_linear_scale_correction = updated['odom_linear_scale_correction']
        self.start_test = updated['start_test']
        self.base_frame = updated['base_frame']
        self.odom_frame = updated['odom_frame']

        if abs(self.rate - previous_rate) > 1e-9:
            self.timer.cancel()
            self.timer = self.create_timer(1.0 / max(self.rate, 1e-6), self._update)
        return SetParametersResult(successful=True)

    def _get_position(self) -> Optional[Point]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                rclpy.time.Time(),
            )
        except TransformException as exc:
            self.get_logger().debug(f'TF lookup failed: {exc}')
            return None

        point = Point()
        point.x = transform.transform.translation.x
        point.y = transform.transform.translation.y
        point.z = transform.transform.translation.z
        return point

    def _stop_robot(self) -> None:
        self.cmd_vel_pub.publish(Twist())

    def _update(self) -> None:
        current_position = self._get_position()
        if current_position is None:
            return

        if self.start_position is None or not self.start_test:
            self.start_position = current_position
            if not self.start_test:
                self._stop_robot()
                return

        assert self.start_position is not None
        distance = math.sqrt(
            (current_position.x - self.start_position.x) ** 2 +
            (current_position.y - self.start_position.y) ** 2
        )
        corrected_distance = distance * self.odom_linear_scale_correction
        error = corrected_distance - self.test_distance

        if abs(error) < self.tolerance:
            self.get_logger().info('Calibration target reached.')
            self.start_test = False
            self.set_parameters([Parameter('start_test', value=False)])
            self._stop_robot()
            self.start_position = current_position
            return

        move_cmd = Twist()
        move_cmd.linear.x = math.copysign(self.speed, -error)
        self.cmd_vel_pub.publish(move_cmd)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CalibrateLinearNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
