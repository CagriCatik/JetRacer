#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

class SafetySupervisorNode(Node):
    def __init__(self):
        super().__init__('safety_supervisor')
        
        self.declare_parameter('min_voltage', 9.5)
        self.declare_parameter('max_temp', 82.0)
        
        self._pub = self.create_publisher(Twist, 'cmd_vel_behavior', 10)
        
        self._cb_group = MutuallyExclusiveCallbackGroup()
        self._battery_sub = self.create_subscription(
            BatteryState, 'battery_state', self._battery_callback, 10, callback_group=self._cb_group)
        self._diag_sub = self.create_subscription(
            DiagnosticArray, '/diagnostics', self._diag_callback, 10, callback_group=self._cb_group)
            
        self._timer = self.create_timer(0.1, self._timer_callback, callback_group=self._cb_group)
        
        self._faults = {
            'battery_low': False,
            'thermal_critical': False,
            'hardware_failed': False
        }
        
        self.get_logger().info("Safety supervisor initialized.")

    def _battery_callback(self, msg: BatteryState):
        if msg.voltage < self.get_parameter('min_voltage').value:
            if not self._faults['battery_low']:
                self.get_logger().error(f"CRITICAL BATTERY: {msg.voltage:.2f}V. Safety halt engaged.")
            self._faults['battery_low'] = True
        else:
            self._faults['battery_low'] = False

    def _diag_callback(self, msg: DiagnosticArray):
        for status in msg.status:
            # Check thermal
            if 'Thermal' in status.name and status.level == DiagnosticStatus.ERROR:
                if not self._faults['thermal_critical']:
                    self.get_logger().error("THERMAL CRITICAL: Safety halt engaged.")
                self._faults['thermal_critical'] = True
            elif 'Thermal' in status.name:
                self._faults['thermal_critical'] = False
                
            # Check serial connection
            if 'Serial Connection' in status.name and status.level == DiagnosticStatus.ERROR:
                self._faults['hardware_failed'] = True
            elif 'Serial Connection' in status.name:
                self._faults['hardware_failed'] = False

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
