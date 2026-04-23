#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from std_msgs.msg import Bool

class SafetySupervisorNode(Node):
    def __init__(self):
        super().__init__('safety_supervisor')
        
        self.declare_parameter('min_voltage', 9.5)
        self.declare_parameter('max_temp', 82.0)
        self.declare_parameter('publish_period_sec', 0.05)
        
        self._pub = self.create_publisher(Twist, 'cmd_vel_safety', 10)
        
        self._cb_group = MutuallyExclusiveCallbackGroup()
        self._battery_sub = self.create_subscription(
            BatteryState, 'battery_state', self._battery_callback, 10, callback_group=self._cb_group)
        self._diag_sub = self.create_subscription(
            DiagnosticArray, '/diagnostics', self._diag_callback, 10, callback_group=self._cb_group)
        self._slip_sub = self.create_subscription(
            Bool, '/control/slipping', self._slip_callback, 10, callback_group=self._cb_group)
        self._collision_sub = self.create_subscription(
            Bool, '/control/collision_blocked', self._collision_callback, 10, callback_group=self._cb_group)

        self._timer = self.create_timer(
            self.get_parameter('publish_period_sec').value,
            self._timer_callback,
            callback_group=self._cb_group,
        )
        
        self._faults = {
            'battery_low': False,
            'thermal_critical': False,
            'hardware_failed': False,
            'slipping': False,
            'collision_blocked': False,
        }
        
        self.get_logger().info("Safety supervisor initialized.")

    def _set_fault(self, name: str, active: bool, message: str) -> None:
        if active and not self._faults[name]:
            self.get_logger().error(message)
        self._faults[name] = active

    def _battery_callback(self, msg: BatteryState):
        self._set_fault(
            'battery_low',
            msg.voltage < self.get_parameter('min_voltage').value,
            f"CRITICAL BATTERY: {msg.voltage:.2f}V. Safety halt engaged.",
        )

    def _diag_callback(self, msg: DiagnosticArray):
        thermal_critical = False
        hardware_failed = False

        for status in msg.status:
            if 'Thermal' in status.name:
                measured_temp = None
                for kv in status.values:
                    if kv.key == 'Temperature (C)':
                        try:
                            measured_temp = float(kv.value)
                        except ValueError:
                            measured_temp = None
                        break

                thermal_critical = (
                    status.level == DiagnosticStatus.ERROR or
                    (measured_temp is not None and measured_temp >= self.get_parameter('max_temp').value)
                )

            if 'Serial Connection' in status.name:
                hardware_failed = status.level == DiagnosticStatus.ERROR

        self._set_fault('thermal_critical', thermal_critical, "THERMAL CRITICAL: Safety halt engaged.")
        self._set_fault('hardware_failed', hardware_failed, "HARDWARE LINK FAILED: Safety halt engaged.")

    def _slip_callback(self, msg: Bool):
        self._set_fault('slipping', msg.data, "WHEEL SLIP DETECTED: Safety halt engaged.")

    def _collision_callback(self, msg: Bool):
        self._set_fault('collision_blocked', msg.data, "COLLISION RISK: Safety halt engaged.")

    def _timer_callback(self):
        active_faults = [k for k, v in self._faults.items() if v]
        if active_faults:
            t = Twist()
            t.linear.x = 0.0
            t.angular.z = 0.0
            self._pub.publish(t)

def main(args=None):
    rclpy.init(args=args)
    node = SafetySupervisorNode()
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
