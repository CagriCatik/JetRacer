#!/usr/bin/env python3
"""
Safety Supervisor Node

Monitors multiple safety inputs (battery, thermal, hardware link, wheel slip,
collision) and publishes zero-velocity halt commands on cmd_vel_safety when any
fault is active. twist_mux routes this at priority 255 (highest).

Safety improvements added:
  - Per-input staleness watchdog: if a safety feed goes silent for longer than
    input_timeout_sec the corresponding fault activates (fail-safe behavior).
  - Teleop awareness: when a human operator is on the joystick, thermal/battery
    warnings are still logged but halt commands are suppressed so the operator
    retains authority. Hardware-link and collision faults override even teleop.
  - Clean shutdown: publishes three zero-velocity frames before destroying node.
"""

import time

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
        # Fail-safe: treat a safety input as faulted if it goes silent this long
        self.declare_parameter('input_timeout_sec', 2.0)

        self._pub = self.create_publisher(Twist, 'cmd_vel_safety', 10)

        self._cb_group = MutuallyExclusiveCallbackGroup()

        self._battery_sub = self.create_subscription(
            BatteryState, 'battery_state', self._battery_callback, 10,
            callback_group=self._cb_group)
        self._diag_sub = self.create_subscription(
            DiagnosticArray, '/diagnostics', self._diag_callback, 10,
            callback_group=self._cb_group)
        self._slip_sub = self.create_subscription(
            Bool, '/control/slipping', self._slip_callback, 10,
            callback_group=self._cb_group)
        self._collision_sub = self.create_subscription(
            Bool, '/control/collision_blocked', self._collision_callback, 10,
            callback_group=self._cb_group)
        # Teleop awareness: detect if a human operator is active
        self._teleop_sub = self.create_subscription(
            Twist, 'cmd_vel_teleop', self._teleop_callback, 10,
            callback_group=self._cb_group)

        self._timer = self.create_timer(
            self.get_parameter('publish_period_sec').value,
            self._timer_callback,
            callback_group=self._cb_group,
        )

        self._faults = {
            'battery_low':        False,
            'thermal_critical':   False,
            'hardware_failed':    False,
            'slipping':           False,
            'collision_blocked':  False,
        }

        # Timestamps of the last message received for each monitored input.
        # None means the input has never been received.
        now = time.monotonic()
        self._last_received: dict[str, float | None] = {
            'battery_state':        None,
            'diagnostics':          None,
            'slipping':             None,
            'collision_blocked':    None,
        }
        # Faults that are forcibly raised when their feed goes stale
        self._stale_fault_map: dict[str, str] = {
            'battery_state':        'battery_low',
            'diagnostics':          'thermal_critical',
            'slipping':             'slipping',
            'collision_blocked':    'collision_blocked',
        }

        # True while a human operator is publishing on cmd_vel_teleop
        self._teleop_active: bool = False
        self._last_teleop_time: float | None = None
        # Teleop inactivity threshold (3× the typical 20 Hz joystick period)
        self._teleop_timeout_sec: float = 0.5

        self.get_logger().info("Safety supervisor initialized.")

    # ── Fault helpers ──────────────────────────────────────────────────────

    def _set_fault(self, name: str, active: bool, message: str) -> None:
        if active and not self._faults[name]:
            self.get_logger().error(message)
        self._faults[name] = active

    def _touch(self, feed: str) -> None:
        """Record the current monotonic time as the last-received timestamp."""
        self._last_received[feed] = time.monotonic()

    def _check_staleness(self) -> None:
        """Raise faults for any safety feed that has gone silent."""
        timeout = self.get_parameter('input_timeout_sec').value
        now = time.monotonic()
        for feed, fault in self._stale_fault_map.items():
            last = self._last_received[feed]
            if last is None:
                # Input has never arrived — only warn after startup grace period
                continue
            stale = (now - last) > timeout
            if stale and not self._faults[fault]:
                self.get_logger().warn(
                    f"Safety input '{feed}' is stale (>{timeout:.1f} s) — "
                    f"activating fault '{fault}' (fail-safe).",
                    throttle_duration_sec=5.0)
            self._faults[fault] = stale or self._faults[fault]

    def _update_teleop_active(self) -> None:
        if self._last_teleop_time is None:
            self._teleop_active = False
            return
        self._teleop_active = (time.monotonic() - self._last_teleop_time) < self._teleop_timeout_sec

    # ── Callbacks ──────────────────────────────────────────────────────────

    def _battery_callback(self, msg: BatteryState):
        self._touch('battery_state')
        self._set_fault(
            'battery_low',
            msg.voltage < self.get_parameter('min_voltage').value,
            f"CRITICAL BATTERY: {msg.voltage:.2f}V. Safety halt engaged.",
        )

    def _diag_callback(self, msg: DiagnosticArray):
        self._touch('diagnostics')
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
                    (measured_temp is not None and
                     measured_temp >= self.get_parameter('max_temp').value)
                )
            if 'Serial Connection' in status.name:
                hardware_failed = status.level == DiagnosticStatus.ERROR

        self._set_fault('thermal_critical', thermal_critical, "THERMAL CRITICAL: Safety halt engaged.")
        self._set_fault('hardware_failed', hardware_failed, "HARDWARE LINK FAILED: Safety halt engaged.")

    def _slip_callback(self, msg: Bool):
        self._touch('slipping')
        self._set_fault('slipping', msg.data, "WHEEL SLIP DETECTED: Safety halt engaged.")

    def _collision_callback(self, msg: Bool):
        self._touch('collision_blocked')
        self._set_fault('collision_blocked', msg.data, "COLLISION RISK: Safety halt engaged.")

    def _teleop_callback(self, msg: Twist):
        self._last_teleop_time = time.monotonic()

    # ── Control loop ───────────────────────────────────────────────────────

    def _timer_callback(self):
        self._check_staleness()
        self._update_teleop_active()

        active_faults = [k for k, v in self._faults.items() if v]
        if not active_faults:
            return

        # When a human operator is active, suppress soft faults (battery, thermal,
        # slip) but still halt for hard faults (hardware failure, collision).
        # The hardware watchdog and collision_assurance operate independently.
        hard_faults = {'hardware_failed', 'collision_blocked'}
        if self._teleop_active:
            blocking_faults = [f for f in active_faults if f in hard_faults]
            if not blocking_faults:
                self.get_logger().warn(
                    f"Soft faults {active_faults} suppressed: teleop operator active.",
                    throttle_duration_sec=10.0)
                return

        t = Twist()
        t.linear.x = 0.0
        t.angular.z = 0.0
        self._pub.publish(t)

    def destroy_node(self):
        """Publish zero velocity on clean shutdown before tearing down."""
        try:
            stop = Twist()
            for _ in range(3):
                self._pub.publish(stop)
                time.sleep(0.02)
        except Exception:
            pass
        super().destroy_node()


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
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
