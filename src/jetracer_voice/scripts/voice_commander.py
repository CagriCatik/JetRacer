#!/usr/bin/env python3
"""
Offline Voice Commander Node

This node utilizes the Vosk Offline Speech Recognition API to parse audio
from a USB microphone in real-time without internet connectivity.
It maps specifically recognized dictionary terms to physical coordinate poses,
allowing you to dispatch the robot to rooms via completely localized voice commands.
"""

import os
import json
from typing import Optional, Dict, Any
from ament_index_python.packages import get_package_share_directory

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.timer import Timer

# Graceful degradation logic if the underlying audio dependencies fail to compile
try:
    import pyaudio
    from vosk import KaldiRecognizer, Model
except ImportError:
    pyaudio = None
    Model = None

class VoiceCommanderNode(Node):
    def __init__(self) -> None:
        """Initializes the audio loop, attempts to link hardware, and creates ROS mappings."""
        super().__init__('voice_commander')
        
        if Model is None or pyaudio is None:
            self.get_logger().error("Critical missing dependencies. Requires: pip install vosk pyaudio")
            raise RuntimeError('missing dependencies: vosk/pyaudio')

        # Resolve package models directory
        package_share_dir = get_package_share_directory('jetracer_voice')
        default_model = os.path.join(package_share_dir, 'models', 'vosk-model-small-en-us-0.15')

        self._load_params()
        self.add_on_set_parameters_callback(self._param_callback)
        
        # Audio Initialization wrapped in a sandboxed try/except to prevent violent core dumps
        try:
            self.model = Model(self.model_path)
            self.recognizer = KaldiRecognizer(self.model, 16000)
            self.p = pyaudio.PyAudio()
            self.stream = self.p.open(
                format=pyaudio.paInt16,
                channels=1,
                rate=16000,
                input=True,
                frames_per_buffer=8000
            )
            self.stream.start_stream()
        except OSError as oe:
            self.get_logger().error(f"Hardware Error. Ensure USB Mic is plugged in and mapped via /dev/snd: {oe}")
            raise RuntimeError('audio device initialization failed') from oe
        except Exception as e:
            self.get_logger().error(f"Failed to initialize Audio/Vosk engine: {e}")
            raise RuntimeError('failed to initialize audio/vosk engine') from e

        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # High-frequency audio polling loop
        self.timer: Timer = self.create_timer(0.1, self._listen)
        
        self.get_logger().info("Offline Voice Commander Active. Listening for 'kitchen'...")

    def _load_params(self, updates: dict[str, object] | None = None) -> None:
        if updates is None:
            updates = {}

        def fetch(name: str):
            if name in updates:
                return updates[name]
            return self.get_parameter(name).value

        # Resolve package models directory
        package_share_dir = get_package_share_directory('jetracer_voice')
        
        m_path = str(fetch('model_path'))
        if not os.path.isabs(m_path):
            m_path = os.path.join(package_share_dir, 'models', m_path)
            
        self._model_path = m_path
        self._kitchen_x = float(fetch('kitchen_x'))
        self._kitchen_y = float(fetch('kitchen_y'))

    def _param_callback(self, params) -> SetParametersResult:
        from rcl_interfaces.msg import SetParametersResult
        updated = {p.name: p.value for p in params}
        
        # model_path is read-only at runtime to prevent reload crashes
        if 'model_path' in updated and str(updated['model_path']) != self._model_path:
             return SetParametersResult(successful=False, reason="model_path is read-only")

        self._load_params(updates=updated)
        return SetParametersResult(successful=True)

    def _listen(self) -> None:
        """
        Pulls chunks from the PyAudio stream. Bypasses overflow dropouts safely.
        Evaluates the text heuristically.
        """
        try:
            # exception_on_overflow=False guarantees we don't crash if ROS drops a frame
            data: bytes = self.stream.read(4000, exception_on_overflow=False)
            
            if self.recognizer.AcceptWaveform(data):
                result: Dict[str, Any] = json.loads(self.recognizer.Result())
                text: str = str(result.get('text', '')).lower()
                
                if text:
                    self.get_logger().info(f"Heard: {text}")
                
                if 'kitchen' in text:
                    self.get_logger().info("Semantic Command Executed: Dispatching delivery to Kitchen!")
                    self._dispatch_goal(self._kitchen_x, self._kitchen_y)
                    
        except IOError as ioe:
            self.get_logger().warn(f"Audio buffer underflow/overflow: {ioe}")
        except Exception as e:
            self.get_logger().error(f"Critical exception in audio listener thread: {e}")

    def _dispatch_goal(self, x: float, y: float) -> None:
        """
        Constructs a stamped pose targeting the /map frame and requests Nav2 execution.
        """
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        
        goal.pose.position.x = x
        goal.pose.position.y = y
        
        # Default forward orientation mathematically
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0  
        
        self.goal_pub.publish(goal)

    def _cleanup_audio(self) -> None:
        stream = getattr(self, 'stream', None)
        if stream is not None:
            try:
                stream.stop_stream()
            except Exception:
                pass
            try:
                stream.close()
            except Exception:
                pass
            self.stream = None

        audio = getattr(self, 'p', None)
        if audio is not None:
            try:
                audio.terminate()
            except Exception:
                pass
            self.p = None

    def destroy_node(self) -> bool:
        self._cleanup_audio()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = VoiceCommanderNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        print(f'voice_commander startup failed: {exc}')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
