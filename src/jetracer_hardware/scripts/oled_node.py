#!/usr/bin/env python3
import os
import socket
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from std_msgs.msg import String

try:
    import board
    import busio
    from PIL import Image, ImageDraw, ImageFont
    import adafruit_ssd1306
    HAS_OLED_LIBS = True
except ImportError:
    HAS_OLED_LIBS = False

class OledNode(Node):
    def __init__(self):
        super().__init__('oled_node')
        
        if not HAS_OLED_LIBS:
            self.get_logger().error("Missing OLED dependencies (PIL, adafruit-ssd1306).")
            raise RuntimeError("Missing OLED dependencies")

        self.declare_parameter('width', 128)
        self.declare_parameter('height', 64)
        self.declare_parameter('i2c_bus', 1)

        # Initialize Hardware
        try:
            self.i2c = busio.I2C(board.SCL, board.SDA)
            self.display = adafruit_ssd1306.SSD1306_I2C(
                self.get_parameter('width').value,
                self.get_parameter('height').value,
                self.i2c
            )
        except Exception as e:
            self.get_logger().error(f"Failed to initialize OLED on I2C: {e}")
            raise e

        # State cache
        self._battery_pct = 0.0
        self._battery_v = 0.0
        self._temp = 0.0
        self._speed = 0.0
        self._faults = []
        self._active_mission = "BOOT"
        self._ip = self._get_ip() or "No IP"

        # Subscriptions
        self.create_subscription(BatteryState, 'battery_state', self._battery_cb, 10)
        self.create_subscription(DiagnosticArray, '/diagnostics', self._diag_cb, 10)
        self.create_subscription(Twist, 'cmd_vel', self._cmd_cb, 10)
        self.create_subscription(String, '/sentinel/status', self._sentinel_cb, 10)

        # Render Timer (2Hz is enough for OLED)
        self.timer = self.create_timer(0.5, self._render)
        
        self.get_logger().info("OLED node initialized.")

    def _get_ip(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception:
            return None

    def _battery_cb(self, msg: BatteryState):
        self._battery_pct = msg.percentage * 100.0
        self._battery_v = msg.voltage

    def _diag_cb(self, msg: DiagnosticArray):
        faults = []
        for status in msg.status:
            if 'Thermal' in status.name:
                for kv in status.values:
                    if kv.key == 'Temperature (C)':
                        self._temp = float(kv.value)
            if status.level >= 2: # ERROR or CRITICAL
                faults.append(status.name.split(':')[-1].strip())
        self._faults = faults[:2] # Only show first 2 faults

    def _cmd_cb(self, msg: Twist):
        self._speed = msg.linear.x

    def _sentinel_cb(self, msg: String):
        self._active_mission = msg.data

    def _render(self):
        # Create blank image for drawing.
        image = Image.new('1', (self.display.width, self.display.height))
        draw = ImageDraw.Draw(image)
        font = ImageFont.load_default()

        # Draw Headers
        draw.text((0, 0), f"MISSION: {self._active_mission}", font=font, fill=255)
        draw.line((0, 12, 128, 12), fill=255)

        # Line 1: IP Address
        draw.text((0, 15), f"IP: {self._ip}", font=font, fill=255)

        # Line 2: Battery & Temp
        draw.text((0, 27), f"BAT: {self._battery_v:.1f}V ({int(self._battery_pct)}%)", font=font, fill=255)
        draw.text((0, 39), f"CPU: {self._temp:.1f}C", font=font, fill=255)

        # Line 3: Speed & Faults
        if self._faults:
            draw.text((0, 51), f"ERR: {','.join(self._faults)}", font=font, fill=255)
        else:
            draw.text((0, 51), f"SPD: {self._speed:.2f} m/s", font=font, fill=255)

        # Display image
        self.display.image(image)
        self.display.show()

def main(args=None):
    rclpy.init(args=args)
    try:
        node = OledNode()
        rclpy.spin(node)
    except Exception:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
