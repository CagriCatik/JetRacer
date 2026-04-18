#!/usr/bin/env python3

import base64
from datetime import datetime
import hashlib
import hmac
from pathlib import Path
import json
from wsgiref.handlers import format_date_time
from time import mktime
import shlex
import ssl
import subprocess
import tempfile

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from urllib.parse import urlencode
import websocket


class TextToSpeechChineseNode(Node):
    def __init__(self) -> None:
        super().__init__('jetracer_tts_cn')
        self.declare_parameter('runtime_dir', '~/.ros/jetracer_voice')
        self.declare_parameter('player_command', 'play -q')
        self.declare_parameter('appid', '')
        self.declare_parameter('api_key', '')
        self.declare_parameter('api_secret', '')
        self.declare_parameter('voice_name', 'xiaoyan')

        self.runtime_dir = Path(str(self.get_parameter('runtime_dir').value)).expanduser()
        self.runtime_dir.mkdir(parents=True, exist_ok=True)
        self.player_command = shlex.split(str(self.get_parameter('player_command').value))
        self.appid = str(self.get_parameter('appid').value)
        self.api_key = str(self.get_parameter('api_key').value)
        self.api_secret = str(self.get_parameter('api_secret').value)
        self.voice_name = str(self.get_parameter('voice_name').value)

        self.subscription = self.create_subscription(String, 'speak', self.callback, 10)

    def create_url(self) -> str:
        url = 'wss://tts-api.xfyun.cn/v2/tts'
        date = format_date_time(mktime(datetime.now().timetuple()))
        signature_origin = 'host: ws-api.xfyun.cn\n'
        signature_origin += f'date: {date}\n'
        signature_origin += 'GET /v2/tts HTTP/1.1'
        signature_sha = hmac.new(
            self.api_secret.encode('utf-8'),
            signature_origin.encode('utf-8'),
            digestmod=hashlib.sha256,
        ).digest()
        authorization_origin = (
            f'api_key=\"{self.api_key}\", algorithm=\"hmac-sha256\", '
            f'headers=\"host date request-line\", signature=\"{base64.b64encode(signature_sha).decode()}\"'
        )
        return url + '?' + urlencode({
            'authorization': base64.b64encode(authorization_origin.encode('utf-8')).decode('utf-8'),
            'date': date,
            'host': 'ws-api.xfyun.cn',
        })

    def callback(self, msg: String) -> None:
        if not self.appid or not self.api_key or not self.api_secret:
            self.get_logger().error('Missing XFYun credentials for Chinese TTS.')
            return

        temp_file = tempfile.NamedTemporaryFile(
            suffix='.mp3', dir=self.runtime_dir, delete=False)
        temp_path = Path(temp_file.name)
        temp_file.close()
        audio_chunks: list[bytes] = []

        def on_message(ws, message) -> None:
            payload = json.loads(message)
            if payload.get('code') != 0:
                self.get_logger().error(f"XFYun TTS error: {payload.get('message', 'unknown')}")
                ws.close()
                return
            data = payload.get('data', {})
            audio = data.get('audio')
            if audio:
                audio_chunks.append(base64.b64decode(audio))
            if data.get('status') == 2:
                ws.close()

        def on_open(ws) -> None:
            payload = {
                'common': {'app_id': self.appid},
                'business': {
                    'aue': 'lame',
                    'auf': 'audio/L16;rate=16000',
                    'vcn': self.voice_name,
                    'tte': 'utf8',
                },
                'data': {
                    'status': 2,
                    'text': base64.b64encode(msg.data.encode('utf-8')).decode('utf-8'),
                },
            }
            ws.send(json.dumps(payload))

        ws = websocket.WebSocketApp(
            self.create_url(),
            on_message=on_message,
            on_open=on_open,
        )
        try:
            ws.run_forever(sslopt={'cert_reqs': ssl.CERT_NONE})
            temp_path.write_bytes(b''.join(audio_chunks))
            subprocess.run([*self.player_command, str(temp_path)], check=False)
        except Exception as exc:
            self.get_logger().error(f'Chinese TTS failed: {exc}')
        finally:
            temp_path.unlink(missing_ok=True)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TextToSpeechChineseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
