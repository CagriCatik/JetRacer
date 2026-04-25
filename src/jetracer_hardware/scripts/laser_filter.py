#!/usr/bin/env python3

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LaserFilterNode(Node):
    def __init__(self) -> None:
        super().__init__('laser_filter')
        self.declare_parameter('laser_angle', 180)
        self.declare_parameter('distance', 12.0)
        self.declare_parameter('input_topic', 'scan')
        self.declare_parameter('output_topic', 'filteredscan')

        self.laser_angle = int(self.get_parameter('laser_angle').value)
        self.distance = float(self.get_parameter('distance').value)
        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)

        self.publisher = self.create_publisher(LaserScan, output_topic, 10)
        self.subscription = self.create_subscription(LaserScan, input_topic, self.callback, 10)
        self.add_on_set_parameters_callback(self._parameter_callback)

    def _parameter_callback(self, parameters):
        for parameter in parameters:
            if parameter.name == 'laser_angle':
                self.laser_angle = int(parameter.value)
            elif parameter.name == 'distance':
                self.distance = float(parameter.value)
        return SetParametersResult(successful=True)

    def callback(self, msg: LaserScan) -> None:
        filtered = LaserScan()
        filtered.header = msg.header
        filtered.angle_min = msg.angle_min
        filtered.angle_max = msg.angle_max
        filtered.angle_increment = msg.angle_increment
        filtered.time_increment = msg.time_increment
        filtered.scan_time = msg.scan_time
        filtered.range_min = msg.range_min
        filtered.range_max = msg.range_max
        filtered.ranges = list(msg.ranges)
        filtered.intensities = list(msg.intensities)

        length = len(filtered.ranges)
        index = int(self.laser_angle / 2.0 * length / 360.0)

        for i in range(length):
            if filtered.ranges[i] > self.distance:
                filtered.ranges[i] = 0.0
                if i < len(filtered.intensities):
                    filtered.intensities[i] = 0.0

        # Blank the rear arc: rays between index and (length - index).
        # When index >= length - index the rear arc is zero-width; skip.
        rear_end = length - index
        for i in range(index, rear_end):
            filtered.ranges[i] = 0.0
            if i < len(filtered.intensities):
                filtered.intensities[i] = 0.0

        self.publisher.publish(filtered)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LaserFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()