#!/usr/bin/env python3
# Keyboard teleoperation node for JetRacer (ROS 2)
# Migrated from jetracer_ros-ubuntu/scripts/teleop_key.py (ROS 1 rospy)
# to rclpy (ROS 2).

import sys
import select
import threading

try:
    import termios
    import tty
except ImportError:
    termios = None  # type: ignore[assignment]
    tty = None  # type: ignore[assignment]

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

MSG = """
Reading from the keyboard and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

MOVE_BINDINGS = {
    'i': (1, 0, 0, 0),
    'o': (1, 0, 0, -1),
    'j': (0, 0, 0, 1),
    'l': (0, 0, 0, -1),
    'u': (1, 0, 0, 1),
    ',': (-1, 0, 0, 0),
    '.': (-1, 0, 0, 1),
    'm': (-1, 0, 0, -1),
    'O': (1, -1, 0, 0),
    'I': (1, 0, 0, 0),
    'J': (0, 1, 0, 0),
    'L': (0, -1, 0, 0),
    'U': (1, 1, 0, 0),
    '<': (-1, 0, 0, 0),
    '>': (-1, -1, 0, 0),
    'M': (-1, 1, 0, 0),
    't': (0, 0, 1, 0),
    'k': (0, 0, 0, 0),
    ' ': (0, 0, 0, 0),
    'b': (0, 0, -1, 0),
    'A': (1, 0, 0, 0),   # cursor-up
    'B': (-1, 0, 0, 0),  # cursor-down
    'C': (0, 0, 0, -1),  # cursor-right
    'D': (0, 0, 0, 1),   # cursor-left
}

SPEED_BINDINGS = {
    'q': (1.1, 1.1),
    'z': (0.9, 0.9),
    'w': (1.1, 1.0),
    'x': (0.9, 1.0),
    'e': (1.0, 1.1),
    'c': (1.0, 0.9),
}


def get_key(settings):
    """Read a single character from stdin without blocking on the terminal."""
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed: float, turn: float) -> str:
    return f'currently:\tspeed {speed:.3f}\tturn {turn:.3f}'


class TeleopKeyNode(Node):
    def __init__(self) -> None:
        super().__init__('teleop_twist_keyboard')

        self.declare_parameter('speed', 0.5)
        self.declare_parameter('turn', 1.0)

        self._speed = float(self.get_parameter('speed').value)
        self._turn = float(self.get_parameter('turn').value)

        self._pub = self.create_publisher(Twist, 'cmd_vel_teleop', 1)

        self._settings = termios.tcgetattr(sys.stdin) if termios is not None else None
        if termios is None:
            raise RuntimeError(
                'teleop_key requires a POSIX terminal (termios). '
                'Run on Linux/Jetson, not Windows/macOS.'
            )
        self._stop_event = threading.Event()

    def run(self) -> None:
        """Blocking keyboard-reading loop — call from the main thread."""
        print(MSG)
        print(vels(self._speed, self._turn))

        status = 0
        try:
            while not self._stop_event.is_set() and rclpy.ok():
                key = get_key(self._settings)

                if key in MOVE_BINDINGS:
                    x_f, y_f, z_f, th_f = MOVE_BINDINGS[key]
                    twist = Twist()
                    twist.linear.x = float(x_f) * self._speed
                    twist.linear.y = float(y_f) * self._speed
                    twist.linear.z = float(z_f) * self._speed
                    twist.angular.x = 0.0
                    twist.angular.y = 0.0
                    twist.angular.z = float(th_f) * self._turn
                    self._pub.publish(twist)

                elif key in SPEED_BINDINGS:
                    self._speed *= SPEED_BINDINGS[key][0]
                    self._turn *= SPEED_BINDINGS[key][1]
                    print(vels(self._speed, self._turn))
                    if status == 14:
                        print(MSG)
                    status = (status + 1) % 15

                else:
                    # Stop on any unrecognised key; CTRL-C exits
                    if key == '\x03':
                        break
                    twist = Twist()
                    self._pub.publish(twist)

        finally:
            # Always send a stop command when exiting
            self._pub.publish(Twist())
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._settings)

    def stop(self) -> None:
        self._stop_event.set()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TeleopKeyNode()

    # Spin the node in a background thread so the main thread can do the
    # blocking keyboard-read loop without starving the executor.
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.run()
    except Exception as exc:  # noqa: BLE001
        node.get_logger().error(f'teleop_key error: {exc}')
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == '__main__':
    main()
