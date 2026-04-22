#!/usr/bin/env python3
import subprocess
import os
import signal
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String

class ControllerRelayNode(Node):
    def __init__(self):
        super().__init__('controller_relay')
        
        # Indices for standard Waveshare Gamepad
        self.declare_parameter('select_btn', 8)
        self.declare_parameter('start_btn', 9)
        self.declare_parameter('x_btn', 0)
        self.declare_parameter('y_btn', 3)
        self.declare_parameter('mode_btn', 10)
        
        self._select = self.get_parameter('select_btn').value
        self._start = self.get_parameter('start_btn').value
        self._x = self.get_parameter('x_btn').value
        self._y = self.get_parameter('y_btn').value
        self._mode = self.get_parameter('mode_btn').value

        self._status_pub = self.create_publisher(String, '/sentinel/status', 10)
        self._sub = self.create_subscription(Joy, 'joy', self._joy_callback, 10)
        
        self._active_process = None
        self._active_mission = "IDLE"
        
        self.get_logger().info("Controller Relay Sentinel Active.")
        self._publish_status()

    def _publish_status(self):
        msg = String()
        # Missions: IDLE, AUTONOMY, SLAM
        msg.data = self._active_mission
        self._status_pub.publish(msg)

    def _kill_active(self):
        if self._active_process:
            self.get_logger().info(f"Terminating mission: {self._active_mission}")
            # Sends SIGINT to the whole process group
            os.killpg(os.getpgid(self._active_process.pid), signal.SIGINT)
            self._active_process.wait()
            self._active_process = None
        self._active_mission = "IDLE"
        self._publish_status()

    def _launch_mission(self, package, launch_file, mission_name):
        if self._active_process:
            self.get_logger().warn("A mission is already running. Kill it first!")
            return

        self.get_logger().info(f"Launching mission: {mission_name}...")
        cmd = f"ros2 launch {package} {launch_file}"
        # Start in a new process group so we can SIGINT the whole tree later
        self._active_process = subprocess.Popen(
            cmd, shell=True, preexec_fn=os.setsid
        )
        self._active_mission = mission_name
        self._publish_status()

    def _joy_callback(self, msg: Joy):
        # Safety: check bounds
        if len(msg.buttons) <= max(self._select, self._start, self._mode, self._x, self._y):
            return

        # Kill Command (MODE button)
        if msg.buttons[self._mode] == 1:
            self._kill_active()
            return

        # Multi-button combos (require SELECT to be held)
        if msg.buttons[self._select] == 1:
            # SELECT + START = Autonomy
            if msg.buttons[self._start] == 1:
                self._launch_mission('jetracer_bringup', 'autonomy.launch.py', 'AUTONOMY')
            
            # SELECT + X = SLAM
            elif msg.buttons[self._x] == 1:
                self._launch_mission('jetracer_navigation', 'slam_nav.launch.py', 'SLAM')

def main(args=None):
    rclpy.init(args=args)
    node = ControllerRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._kill_active()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
