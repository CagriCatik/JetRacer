#!/usr/bin/env python3

from pathlib import Path
import shlex
import subprocess
import tempfile

from gtts import gTTS
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TextToSpeechEnglishNode(Node):
    def __init__(self) -> None:
        super().__init__('jetracer_tts_en')
        self.declare_parameter('runtime_dir', '~/.ros/jetracer_voice')
        self.declare_parameter('player_command', 'play -q')
        self.runtime_dir = Path(str(self.get_parameter('runtime_dir').value)).expanduser()
        self.runtime_dir.mkdir(parents=True, exist_ok=True)
        self.player_command = shlex.split(str(self.get_parameter('player_command').value))
        self.subscription = self.create_subscription(String, 'speak', self.callback, 10)

    def callback(self, msg: String) -> None:
        if not msg.data.strip():
            return
        temp_file = tempfile.NamedTemporaryFile(
            suffix='.mp3', dir=self.runtime_dir, delete=False)
        temp_path = Path(temp_file.name)
        temp_file.close()
        try:
            gTTS(text=msg.data, lang='en').save(str(temp_path))
            subprocess.run([*self.player_command, str(temp_path)], check=False)
        except Exception as exc:
            self.get_logger().error(f'TTS playback failed: {exc}')
        finally:
            temp_path.unlink(missing_ok=True)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TextToSpeechEnglishNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
