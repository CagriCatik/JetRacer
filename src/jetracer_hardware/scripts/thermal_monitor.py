#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class ThermalMonitorNode(Node):
    def __init__(self):
        super().__init__('thermal_monitor')
        self.declare_parameter('update_period', 2.0)
        self.declare_parameter('critical_temp', 85.0)
        self.declare_parameter('warning_temp', 75.0)
        
        self.timer = self.create_timer(
            self.get_parameter('update_period').value, 
            self.timer_callback
        )
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.get_logger().info("Thermal monitor initialized.")

    def get_temp(self):
        try:
            with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                temp_raw = f.read().strip()
                return float(temp_raw) / 1000.0
        except Exception:
            return None

    def timer_callback(self):
        temp = self.get_temp()
        if temp is None:
            return

        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        status = DiagnosticStatus()
        status.name = 'Thermal: Jetson Nano CPU'
        status.hardware_id = 'NVIDIA_Jetson_Nano_Thermal'
        
        crit = self.get_parameter('critical_temp').value
        warn = self.get_parameter('warning_temp').value

        if temp > crit:
            status.level = DiagnosticStatus.ERROR
            status.message = "CRITICAL TEMPERATURE"
        elif temp > warn:
            status.level = DiagnosticStatus.WARN
            status.message = "HIGH TEMPERATURE"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "Temperature Healthy"

        status.values = [
            KeyValue(key='Temperature (C)', value=f"{temp:.2f}"),
            KeyValue(key='Warning Threshold (C)', value=str(warn)),
            KeyValue(key='Critical Threshold (C)', value=str(crit))
        ]
        
        msg.status.append(status)
        self.diag_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ThermalMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
