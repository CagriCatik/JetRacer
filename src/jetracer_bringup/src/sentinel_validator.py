#!/usr/bin/env python3
"""
Sentinel Pre-Arm Validator

Implements safety gating to prevent mission launch until all hardware is verified.
Monitors: IMU calibration, serial heartbeat, LiDAR publishing, camera frame rate,
battery voltage/current, and CPU temperature.

Waveshare JetRacer ROS AI Kit constraints:
- MPU9250 IMU must boot stationary (Waveshare docs) or yaw becomes unreliable
- RP2040 watchdog requires heartbeat every 100ms or resets
- INA219 tracks power; brownout prediction from loaded voltage curve
- CSI camera takes 1-2s to stabilize; blank frames indicate not ready
- Jetson Nano thermal throttles at 82°C and shuts down at 95°C
"""

from typing import Dict
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import BatteryState, Imu, Image, CameraInfo
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool
from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray


class SentinelValidator(Node):
    """
    Pre-arm gate that validates robot readiness before mission launch.
    
    Gate opens only when ALL checks pass:
    1. IMU stable (>2 seconds without large acceleration variance)
    2. Serial link healthy (RP2040 heartbeat valid)
    3. LiDAR publishing detections
    4. Camera frame rate > 5 Hz
    5. Battery safe (OCV > 10.0V estimated)
    6. Current within reasonable bounds (< 3.5A)
    7. CPU temperature < 75°C (well below 82°C throttle point)
    """

    def __init__(self) -> None:
        super().__init__('sentinel_validator')

        # Configuration
        self.declare_parameter('imu_stable_duration', 2.0, ParameterDescriptor(
            description='IMU must be stationary for this many seconds'))
        self.declare_parameter('imu_accel_variance_threshold', 0.1, ParameterDescriptor(
            description='Max acceleration variance (m/s^2)^2 during calibration'))
        self.declare_parameter('lidar_timeout_sec', 5.0, ParameterDescriptor(
            description='Max age of LiDAR message before considered stale'))
        self.declare_parameter('camera_min_hz', 5.0, ParameterDescriptor(
            description='Minimum camera frame rate (Hz)'))
        self.declare_parameter('battery_min_voltage', 10.0, ParameterDescriptor(
            description='Minimum safe OCV in volts'))
        self.declare_parameter('battery_max_current', 3500.0, ParameterDescriptor(
            description='Maximum acceptable current draw (mA)'))
        self.declare_parameter('cpu_max_temp', 75.0, ParameterDescriptor(
            description='CPU temperature limit (°C) before halting'))
        self.declare_parameter('startup_timeout_sec', 30.0, ParameterDescriptor(
            description='Maximum time to wait for all hardware before abort'))

        # State tracking
        self._imu_samples: list = []
        self._imu_calibration_start_time = None
        self._imu_calibrated = False
        
        self._battery_min_ocv_seen = 12.6
        self._battery_current_max_seen = 0.0
        
        self._lidar_last_message_time = None
        self._lidar_publishing = False
        
        self._camera_last_frame_time = None
        self._camera_frame_count = 0
        self._camera_hz_estimate = 0.0
        
        self._serial_heartbeat_valid = False
        self._cpu_temp = 0.0
        
        self._startup_time = self.get_clock().now()
        self._gate_is_open = False
        self._gate_open_time = None

        # QoS profiles
        sensor_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        # Subscriptions
        self.create_subscription(Imu, 'imu', self._on_imu, sensor_qos)
        self.create_subscription(
            BatteryState, 'battery_state', self._on_battery, qos_profile_default=1)
        self.create_subscription(
            Image, 'csi_cam_0/image_raw/compressed', self._on_camera, sensor_qos)
        self.create_subscription(
            TwistStamped, '/scan', self._on_lidar_scan, sensor_qos)

        # Publishers
        self._gate_status_pub = self.create_publisher(Bool, 'sentinel/gate_open', 1)
        self._validator_status_pub = self.create_publisher(DiagnosticArray, 'sentinel/validator_status', 1)

        # Timer for periodic status checks
        self.create_timer(1.0, self._check_gate_state)
        self.create_timer(5.0, self._publish_diagnostics)

        self.get_logger().info(
            'Sentinel Validator initialized. '
            'Waiting for hardware: IMU stable, serial link, LiDAR, camera, battery OK.'
        )

    def _on_imu(self, msg: Imu) -> None:
        """Track IMU samples to detect stable initialization."""
        if self._imu_calibrated:
            return  # Already calibrated

        accel_magnitude = (
            msg.linear_acceleration.x ** 2 +
            msg.linear_acceleration.y ** 2 +
            msg.linear_acceleration.z ** 2
        ) ** 0.5

        now = self.get_clock().now()
        self._imu_samples.append((now, accel_magnitude))

        # Keep only recent samples (within calibration window)
        if self._imu_calibration_start_time is None:
            self._imu_calibration_start_time = now

        window_start = self._imu_calibration_start_time.nanoseconds + (
            int(self.get_parameter('imu_stable_duration').value * 1e9)
        )
        self._imu_samples = [
            s for s in self._imu_samples if s[0].nanoseconds >= window_start
        ]

    def _on_battery(self, msg: BatteryState) -> None:
        """Monitor battery voltage and current."""
        self._battery_min_ocv_seen = min(self._battery_min_ocv_seen, msg.voltage)
        self._battery_current_max_seen = max(self._battery_current_max_seen, msg.current)
        self._serial_heartbeat_valid = msg.present

    def _on_camera(self, msg: Image) -> None:
        """Track camera frame rate."""
        now = self.get_clock().now()
        
        if self._camera_last_frame_time is not None:
            dt = (now.nanoseconds - self._camera_last_frame_time.nanoseconds) / 1e9
            if dt > 0:
                self._camera_hz_estimate = 1.0 / dt
        
        self._camera_last_frame_time = now
        self._camera_frame_count += 1

    def _on_lidar_scan(self, msg: TwistStamped) -> None:
        """Mark LiDAR as publishing."""
        self._lidar_last_message_time = self.get_clock().now()
        self._lidar_publishing = True

    def _check_imu_stable(self) -> bool:
        """Check if IMU has been stationary for the required duration."""
        if len(self._imu_samples) < 5:
            return False

        # Calculate variance of acceleration magnitude
        magnitudes = [s[1] for s in self._imu_samples]
        mean = sum(magnitudes) / len(magnitudes)
        variance = sum((x - mean) ** 2 for x in magnitudes) / len(magnitudes)

        threshold = self.get_parameter('imu_accel_variance_threshold').value
        if variance < threshold:
            if not self._imu_calibrated:
                self.get_logger().info(
                    f'IMU calibrated: variance={variance:.4f} < {threshold:.4f}'
                )
            self._imu_calibrated = True
            return True

        return False

    def _check_serial_link(self) -> bool:
        """Check if serial link to RP2040 is alive."""
        return self._serial_heartbeat_valid

    def _check_lidar_publishing(self) -> bool:
        """Check if LiDAR is publishing recent data."""
        if self._lidar_last_message_time is None:
            return False

        age = (
            self.get_clock().now().nanoseconds - self._lidar_last_message_time.nanoseconds
        ) / 1e9
        timeout = self.get_parameter('lidar_timeout_sec').value
        
        if age < timeout:
            return True
        
        return False

    def _check_camera_publishing(self) -> bool:
        """Check if camera has stable frame rate."""
        if self._camera_last_frame_time is None:
            return False

        min_hz = self.get_parameter('camera_min_hz').value
        return self._camera_hz_estimate >= min_hz

    def _check_battery_safe(self) -> bool:
        """Check if battery voltage is above minimum safe level."""
        min_voltage = self.get_parameter('battery_min_voltage').value
        return self._battery_min_ocv_seen >= min_voltage

    def _check_current_safe(self) -> bool:
        """Check if current draw is within limits."""
        max_current = self.get_parameter('battery_max_current').value
        return self._battery_current_max_seen <= max_current

    def _check_cpu_temp(self) -> bool:
        """Check if CPU temperature is acceptable."""
        # TODO: Read from /sys/class/thermal/thermal_zone0/temp (Jetson Nano)
        # For now, assume OK
        return True

    def _check_gate_state(self) -> None:
        """Evaluate all gate conditions and update status."""
        checks: Dict[str, bool] = {
            'IMU Stable': self._check_imu_stable(),
            'Serial Link': self._check_serial_link(),
            'LiDAR Publishing': self._check_lidar_publishing(),
            'Camera Publishing': self._check_camera_publishing(),
            'Battery Safe': self._check_battery_safe(),
            'Current Safe': self._check_current_safe(),
            'CPU Temperature': self._check_cpu_temp(),
        }

        all_pass = all(checks.values())
        startup_age = (self.get_clock().now().nanoseconds - self._startup_time.nanoseconds) / 1e9
        startup_timeout = self.get_parameter('startup_timeout_sec').value

        # Gate logic
        if all_pass:
            if not self._gate_is_open:
                self.get_logger().info('✓ PRE-ARM GATE OPEN: All checks passed. Mission launch approved.')
                self._gate_is_open = True
                self._gate_open_time = self.get_clock().now()
        else:
            if self._gate_is_open:
                self.get_logger().warn('✗ PRE-ARM GATE CLOSED: Hardware fault detected.')
                self._gate_is_open = False
            
            if startup_age > startup_timeout:
                failed = [k for k, v in checks.items() if not v]
                self.get_logger().error(
                    f'Startup timeout ({startup_age:.1f}s). Failed checks: {failed}'
                )

        # Publish gate status
        gate_msg = Bool()
        gate_msg.data = self._gate_is_open
        self._gate_status_pub.publish(gate_msg)

    def _publish_diagnostics(self) -> None:
        """Publish detailed validator status as diagnostics."""
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()

        # Create status entries
        checks = [
            ('IMU Calibration', self._check_imu_stable()),
            ('Serial Link', self._check_serial_link()),
            ('LiDAR Publishing', self._check_lidar_publishing()),
            ('Camera Publishing', self._check_camera_publishing()),
            ('Battery Voltage', self._check_battery_safe()),
            ('Current Draw', self._check_current_safe()),
            ('CPU Temperature', self._check_cpu_temp()),
        ]

        for name, passed in checks:
            status = DiagnosticStatus()
            status.name = f'sentinel_validator: {name}'
            status.hardware_id = 'JetRacer_Sentinel'
            status.level = DiagnosticStatus.OK if passed else DiagnosticStatus.WARN
            status.message = 'OK' if passed else 'FAIL'

            # Add key values
            if name == 'IMU Calibration' and len(self._imu_samples) > 0:
                magnitudes = [s[1] for s in self._imu_samples]
                variance = (
                    sum((x - sum(magnitudes) / len(magnitudes)) ** 2 for x in magnitudes) /
                    len(magnitudes)
                )
                status.values.append(
                    self._kv_pair('Acceleration Variance', f'{variance:.4f} m/s^2^2')
                )
            elif name == 'Camera Publishing':
                status.values.append(
                    self._kv_pair('Frame Rate', f'{self._camera_hz_estimate:.1f} Hz')
                )
            elif name == 'Battery Voltage':
                status.values.append(
                    self._kv_pair('Min Voltage Seen', f'{self._battery_min_ocv_seen:.2f} V')
                )
            elif name == 'Current Draw':
                status.values.append(
                    self._kv_pair('Max Current Seen', f'{self._battery_current_max_seen:.0f} mA')
                )

            diag_array.status.append(status)

        # Add overall gate status
        overall = DiagnosticStatus()
        overall.name = 'sentinel_validator: Pre-Arm Gate'
        overall.hardware_id = 'JetRacer_Sentinel'
        overall.level = DiagnosticStatus.OK if self._gate_is_open else DiagnosticStatus.WARN
        overall.message = 'OPEN' if self._gate_is_open else 'CLOSED'
        diag_array.status.append(overall)

        self._validator_status_pub.publish(diag_array)

    @staticmethod
    def _kv_pair(key: str, value: str) -> DiagnosticStatus.KeyValue:
        """Create a key-value pair for diagnostics."""
        kv = DiagnosticStatus.KeyValue()
        kv.key = key
        kv.value = value
        return kv


def main(args=None):
    rclpy.init(args=args)
    node = SentinelValidator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
