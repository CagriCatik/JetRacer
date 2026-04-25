#!/usr/bin/env python3
"""
Thermal Monitor Node

Reads all available Jetson thermal zones from the Linux sysfs interface and
publishes a DiagnosticArray. Also computes a lightweight CPU usage estimate
from /proc/stat (no psutil dependency).

Improvements over original single-zone version:
  - Enumerates all /sys/class/thermal/thermal_zone* entries and reads their
    type alongside temperature, so GPU, PLL, and AO zones are all reported.
  - Adds a 'CPU Usage (%)' key derived from /proc/stat to distinguish a cool
    but fully-loaded CPU from a thermally idle one.
  - Retains full backwards-compatibility with safety_supervisor.py which checks
    for 'Thermal' in status.name and looks for the 'Temperature (C)' key.
"""

import glob
import os
import time

import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


def _read_thermal_zones() -> list[tuple[str, float]]:
    """Return [(zone_type, temp_celsius), ...] for all readable thermal zones."""
    results: list[tuple[str, float]] = []
    for zone_path in sorted(glob.glob('/sys/class/thermal/thermal_zone*/temp')):
        zone_dir = os.path.dirname(zone_path)
        type_path = os.path.join(zone_dir, 'type')
        try:
            with open(type_path, 'r') as f:
                zone_type = f.read().strip()
        except Exception:
            zone_type = os.path.basename(zone_dir)  # fallback: "thermal_zone0"
        try:
            with open(zone_path, 'r') as f:
                temp_celsius = float(f.read().strip()) / 1000.0
            results.append((zone_type, temp_celsius))
        except Exception:
            pass
    return results


class _CpuStatReader:
    """Minimal /proc/stat reader for overall CPU usage. No external dependencies."""

    def __init__(self) -> None:
        self._prev: tuple[int, int] | None = None  # (idle, total)

    def read_percent(self) -> float | None:
        try:
            with open('/proc/stat', 'r') as f:
                first_line = f.readline()
        except Exception:
            return None

        parts = first_line.split()
        if len(parts) < 5 or parts[0] != 'cpu':
            return None

        try:
            values = [int(p) for p in parts[1:]]
        except ValueError:
            return None

        # idle = idle + iowait (fields 4 and 5, 0-indexed from values)
        idle = values[3] + (values[4] if len(values) > 4 else 0)
        total = sum(values)

        if self._prev is None:
            self._prev = (idle, total)
            return None

        prev_idle, prev_total = self._prev
        self._prev = (idle, total)

        d_total = total - prev_total
        d_idle = idle - prev_idle
        if d_total <= 0:
            return None
        return 100.0 * (1.0 - d_idle / d_total)


class ThermalMonitorNode(Node):
    def __init__(self):
        super().__init__('thermal_monitor')
        self.declare_parameter('update_period', 2.0)
        self.declare_parameter('critical_temp', 85.0)
        self.declare_parameter('warning_temp', 75.0)

        self._cpu_stat = _CpuStatReader()

        self.timer = self.create_timer(
            self.get_parameter('update_period').value,
            self.timer_callback
        )
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.get_logger().info("Thermal monitor initialized (multi-zone).")

    def timer_callback(self):
        crit = self.get_parameter('critical_temp').value
        warn = self.get_parameter('warning_temp').value

        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()

        zones = _read_thermal_zones()
        if not zones:
            # Fallback: no readable zones
            status = DiagnosticStatus()
            status.name = 'Thermal: Jetson Nano'
            status.hardware_id = 'NVIDIA_Jetson_Nano_Thermal'
            status.level = DiagnosticStatus.WARN
            status.message = 'No thermal zones readable'
            msg.status.append(status)
        else:
            for zone_type, temp in zones:
                status = DiagnosticStatus()
                # Keep 'Thermal' in name for backwards-compatibility with safety_supervisor
                status.name = f'Thermal: {zone_type}'
                status.hardware_id = 'NVIDIA_Jetson_Nano_Thermal'

                if temp > crit:
                    status.level = DiagnosticStatus.ERROR
                    status.message = f'CRITICAL TEMPERATURE ({zone_type})'
                elif temp > warn:
                    status.level = DiagnosticStatus.WARN
                    status.message = f'HIGH TEMPERATURE ({zone_type})'
                else:
                    status.level = DiagnosticStatus.OK
                    status.message = f'Temperature Healthy ({zone_type})'

                status.values = [
                    KeyValue(key='Temperature (C)', value=f'{temp:.2f}'),
                    KeyValue(key='Zone Type', value=zone_type),
                    KeyValue(key='Warning Threshold (C)', value=str(warn)),
                    KeyValue(key='Critical Threshold (C)', value=str(crit)),
                ]
                msg.status.append(status)

        # CPU usage entry (separate status, not thermal — for observability)
        cpu_pct = self._cpu_stat.read_percent()
        if cpu_pct is not None:
            cpu_status = DiagnosticStatus()
            cpu_status.name = 'CPU Usage: Jetson Nano'
            cpu_status.hardware_id = 'NVIDIA_Jetson_Nano_CPU'
            if cpu_pct > 95.0:
                cpu_status.level = DiagnosticStatus.WARN
                cpu_status.message = f'CPU saturated: {cpu_pct:.0f}%'
            else:
                cpu_status.level = DiagnosticStatus.OK
                cpu_status.message = f'CPU: {cpu_pct:.0f}%'
            cpu_status.values = [
                KeyValue(key='CPU Usage (%)', value=f'{cpu_pct:.1f}'),
            ]
            msg.status.append(cpu_status)

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
