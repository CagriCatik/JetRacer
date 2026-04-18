#!/usr/bin/env python3
"""
Semantic Behavior Node

This module implements a deterministic behavioral state machine that interfaces with
vision_msgs output from the YOLO detection stack. It identifies hazardous objects
(like Stop Signs) and evaluates their spatial risk based on a scaling threshold.
If an interception is triggered, it asserts priority control over the JetRacer's
twist_mux to physically halt the chassis, releasing control gracefully after a cooldown.
"""

from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import qos_profile_sensor_data
from rclpy.timer import Timer
from vision_msgs.msg import Detection2DArray


class SemanticBehaviorNode(Node):
    """
    ROS 2 Node that intercepts YOLO vision outputs and performs high-priority
    hardware overrides based on semantic identification.
    """

    def __init__(self) -> None:
        """Initializes the SemanticBehaviorNode and state machine flags."""
        super().__init__('semantic_behavior')

        # Parameter Declarations
        self.declare_parameter('trigger_class_name', 'stop sign')
        self.declare_parameter('camera_width', 640.0)
        self.declare_parameter('camera_height', 480.0)
        self.declare_parameter('box_size_trigger_pct', 0.15)
        self.declare_parameter('stop_duration_sec', 4.0)
        self.declare_parameter('cooldown_sec', 5.0)

        # Apply parameters
        self._load_params()
        self.add_on_set_parameters_callback(self._param_callback)

        # Expert ROS 2 Architecture: Isolate callbacks
        self._timer_group = MutuallyExclusiveCallbackGroup()
        self._sub_group = MutuallyExclusiveCallbackGroup()

        # Publishers / Subscribers
        self._sub = self.create_subscription(
            Detection2DArray,
            'perception/yolo_detections',
            self._detection_callback,
            qos_profile_sensor_data,
            callback_group=self._sub_group
        )
        self._pub = self.create_publisher(Twist, 'cmd_vel_behavior', 10)

        # State Machine Flags
        self._is_stopping: bool = False
        self._in_cooldown: bool = False
        
        # ROS 2 Timers
        self._stop_timer: Optional[Timer] = None
        self._cooldown_timer: Optional[Timer] = None

        # Failsafe Loop: Assert control over twist_mux actively
        self._publish_timer: Timer = self.create_timer(0.1, self._publish_loop, callback_group=self._timer_group)

        self.get_logger().info("Semantic behavior node initialized and scanning.")

    def _load_params(self) -> None:
        """Internal method to map ROS 2 parameters to class constants."""
        self._trigger_class: str = str(self.get_parameter('trigger_class_name').value).lower()
        self._cam_w: float = float(self.get_parameter('camera_width').value)
        self._cam_h: float = float(self.get_parameter('camera_height').value)
        self._trigger_pct: float = float(self.get_parameter('box_size_trigger_pct').value)
        self._stop_duration: float = float(self.get_parameter('stop_duration_sec').value)
        self._cooldown_duration: float = float(self.get_parameter('cooldown_sec').value)
        self._frame_area: float = self._cam_w * self._cam_h

    def _param_callback(self, params) -> SetParametersResult:
        """Dynamic reconfigure callback."""
        self._load_params()
        return SetParametersResult(successful=True)

    def _detection_callback(self, msg: Detection2DArray) -> None:
        """
        Processes incoming semantic detections. 
        Calculates scale-ratio to infer real-world proximity before triggering.
        """
        if self._is_stopping or self._in_cooldown:
            return  # State Machine ignores new input during active resolution

        for detection in msg.detections:
            for result in detection.results:
                class_id: str = str(result.hypothesis.class_id).lower()
                
                if class_id == self._trigger_class:
                    box_area: float = detection.bbox.size_x * detection.bbox.size_y
                    box_pct: float = box_area / self._frame_area

                    if box_pct >= self._trigger_pct:
                        self.get_logger().warn(
                            f"[{class_id.upper()}] detected! Size: {box_pct:.2%} > {self._trigger_pct:.2%}. "
                            "Initiating Emergency Stop!"
                        )
                        self._trigger_stop()
                        return

    def _trigger_stop(self) -> None:
        """Transitions into the active STOP state, starting the hardware intervention timer."""
        self._is_stopping = True
        if self._stop_timer is not None:
            self.destroy_timer(self._stop_timer)
            self._stop_timer = None
        self._stop_timer = self.create_timer(self._stop_duration, self._end_stop, callback_group=self._timer_group)

    def _end_stop(self) -> None:
        """Releases the STOP state and transitions into COOLDOWN to prevent infinite locking."""
        self.get_logger().info("Stop duration complete. Entering cooldown.")
        self._is_stopping = False
        if self._stop_timer is not None:
            self.destroy_timer(self._stop_timer)
            self._stop_timer = None

        self._in_cooldown = True
        if self._cooldown_timer is not None:
            self.destroy_timer(self._cooldown_timer)
            self._cooldown_timer = None
        self._cooldown_timer = self.create_timer(self._cooldown_duration, self._end_cooldown, callback_group=self._timer_group)

    def _end_cooldown(self) -> None:
        """Releases all locks, returning the machine to IDLE scanning."""
        self.get_logger().info("Cooldown complete. Resuming semantic scanning.")
        self._in_cooldown = False
        if self._cooldown_timer is not None:
            self.destroy_timer(self._cooldown_timer)
            self._cooldown_timer = None

    def _publish_loop(self) -> None:
        """
        High-frequency loop. Actively commands Priority 8 while in the stopping state.
        When not stopping, it is silent, allowing twist_mux to decay its connection and 
        yield to the Priority 5 lane follower.
        """
        if self._is_stopping:
            t = Twist()
            t.linear.x = 0.0
            t.angular.z = 0.0
            self._pub.publish(t)

def main(args=None) -> None:
    rclpy.init(args=args)
    node = SemanticBehaviorNode()
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
