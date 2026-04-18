#!/usr/bin/env python3

from array import array
import collections
from pathlib import Path
import shlex
import subprocess
import sys
import time
import wave

import pyaudio
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import webrtcvad


FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000
CHUNK_DURATION_MS = 30
PADDING_DURATION_MS = 1500
CHUNK_SIZE = int(RATE * CHUNK_DURATION_MS / 1000)
NUM_PADDING_CHUNKS = int(PADDING_DURATION_MS / CHUNK_DURATION_MS)
NUM_WINDOW_CHUNKS = int(400 / CHUNK_DURATION_MS)
NUM_WINDOW_CHUNKS_END = NUM_WINDOW_CHUNKS * 2


def normalize(samples: array) -> array:
    maximum = max(abs(sample) for sample in samples) or 1
    scale = float(32767) / maximum
    normalized = array('h')
    for sample in samples:
        normalized.append(int(sample * scale))
    return normalized


def write_wave(path: Path, samples: array) -> None:
    with wave.open(str(path), 'wb') as wav_file:
        wav_file.setnchannels(CHANNELS)
        wav_file.setsampwidth(2)
        wav_file.setframerate(RATE)
        wav_file.writeframes(samples.tobytes())


class VoiceActivityNode(Node):
    def __init__(self) -> None:
        super().__init__('jetracer_vad_node')
        self.declare_parameter('mode', 'play')
        self.declare_parameter('runtime_dir', '~/.ros/jetracer_voice')
        self.declare_parameter('player_command', 'play -q')
        self.declare_parameter('assistant_player_command', 'aplay -q -r 16000 -f S16_LE')
        self.declare_parameter('assistant_credentials', '')
        self.declare_parameter('assistant_device_model_id', '')
        self.declare_parameter('assistant_device_id', '')
        self.declare_parameter('xfyun_iat_appid', '')
        self.declare_parameter('xfyun_iat_api_key', '')
        self.declare_parameter('xfyun_iat_api_secret', '')
        self.declare_parameter('xfyun_aiui_appid', '')
        self.declare_parameter('xfyun_aiui_api_key', '')
        self.declare_parameter('xfyun_aiui_auth_id', '')
        self.declare_parameter('xfyun_tts_appid', '')
        self.declare_parameter('xfyun_tts_api_key', '')
        self.declare_parameter('xfyun_tts_api_secret', '')

        self.mode = str(self.get_parameter('mode').value)
        self.runtime_dir = Path(str(self.get_parameter('runtime_dir').value)).expanduser()
        self.runtime_dir.mkdir(parents=True, exist_ok=True)
        self.player_command = shlex.split(str(self.get_parameter('player_command').value))
        self.assistant_player_command = shlex.split(str(self.get_parameter('assistant_player_command').value))
        self.publisher = self.create_publisher(String, 'chatter', 10)

    def record_sentence(self, record_path: Path) -> None:
        vad = webrtcvad.Vad(2)
        audio_interface = pyaudio.PyAudio()
        stream = audio_interface.open(
            format=FORMAT,
            channels=CHANNELS,
            rate=RATE,
            input=True,
            start=False,
            frames_per_buffer=CHUNK_SIZE,
        )
        raw_data = array('h')
        ring_buffer_flags = [0] * NUM_WINDOW_CHUNKS
        ring_buffer_index = 0
        ring_buffer_flags_end = [0] * NUM_WINDOW_CHUNKS_END
        ring_buffer_index_end = 0
        index = 0
        start_point = 0
        start_time = time.time()
        triggered = False

        stream.start_stream()
        try:
            while rclpy.ok():
                chunk = stream.read(CHUNK_SIZE, exception_on_overflow=False)
                raw_data.extend(array('h', chunk))
                index += CHUNK_SIZE
                active = vad.is_speech(chunk, RATE)
                ring_buffer_flags[ring_buffer_index] = 1 if active else 0
                ring_buffer_index = (ring_buffer_index + 1) % NUM_WINDOW_CHUNKS
                ring_buffer_flags_end[ring_buffer_index_end] = 1 if active else 0
                ring_buffer_index_end = (ring_buffer_index_end + 1) % NUM_WINDOW_CHUNKS_END

                if not triggered:
                    num_voiced = sum(ring_buffer_flags)
                    if num_voiced > 0.8 * NUM_WINDOW_CHUNKS:
                        triggered = True
                        start_time = time.time()
                        start_point = max(0, index - CHUNK_SIZE * 20)
                else:
                    num_unvoiced = NUM_WINDOW_CHUNKS_END - sum(ring_buffer_flags_end)
                    if num_unvoiced > 0.9 * NUM_WINDOW_CHUNKS_END or (time.time() - start_time) > 10:
                        break
        finally:
            stream.stop_stream()
            stream.close()
            audio_interface.terminate()

        clipped = raw_data[start_point:]
        write_wave(record_path, normalize(clipped))

    def run_helper(self, script_name: str, arguments: list[str]) -> None:
        script_path = Path(__file__).with_name(script_name)
        subprocess.run([sys.executable, str(script_path), *arguments], check=False)

    def publish_text_file(self, text_path: Path, publish_all_lines: bool) -> None:
        if not text_path.exists():
            return
        lines = [line.strip() for line in text_path.read_text(encoding='utf-8').splitlines() if line.strip()]
        if not lines:
            return
        payload = '\n'.join(lines) if publish_all_lines else lines[0]
        self.publisher.publish(String(data=payload))

    def play_audio(self, command: list[str], path: Path) -> None:
        if path.exists():
            subprocess.run([*command, str(path)], check=False)

    def run(self) -> None:
        record_path = self.runtime_dir / 'record.wav'
        text_path = self.runtime_dir / 'talk.txt'
        assistant_audio_path = self.runtime_dir / 'assistant.raw'
        cn_audio_path = self.runtime_dir / 'response.mp3'

        while rclpy.ok():
            self.record_sentence(record_path)
            text_path.unlink(missing_ok=True)
            assistant_audio_path.unlink(missing_ok=True)
            cn_audio_path.unlink(missing_ok=True)

            if self.mode == 'play':
                self.play_audio(self.player_command, record_path)
            elif self.mode in {'asr_en', 'talk_en'}:
                self.run_helper('ginput.py', [
                    '--input-audio-file', str(record_path),
                    '--output-audio-file', str(assistant_audio_path),
                    '--text-output-file', str(text_path),
                    '--credentials', str(self.get_parameter('assistant_credentials').value),
                    '--device-model-id', str(self.get_parameter('assistant_device_model_id').value),
                    '--device-id', str(self.get_parameter('assistant_device_id').value),
                ])
                self.publish_text_file(text_path, publish_all_lines=self.mode == 'talk_en')
                if self.mode == 'talk_en':
                    self.play_audio(self.assistant_player_command, assistant_audio_path)
            elif self.mode == 'asr_cn':
                self.run_helper('iat.py', [
                    '--appid', str(self.get_parameter('xfyun_iat_appid').value),
                    '--api-key', str(self.get_parameter('xfyun_iat_api_key').value),
                    '--api-secret', str(self.get_parameter('xfyun_iat_api_secret').value),
                    '--audio-file', str(record_path),
                    '--text-output-file', str(text_path),
                ])
                self.publish_text_file(text_path, publish_all_lines=False)
            elif self.mode == 'talk_cn':
                self.run_helper('aiui.py', [
                    '--aiui-appid', str(self.get_parameter('xfyun_aiui_appid').value),
                    '--aiui-api-key', str(self.get_parameter('xfyun_aiui_api_key').value),
                    '--auth-id', str(self.get_parameter('xfyun_aiui_auth_id').value),
                    '--audio-file', str(record_path),
                    '--text-output-file', str(text_path),
                    '--audio-output-file', str(cn_audio_path),
                    '--tts-appid', str(self.get_parameter('xfyun_tts_appid').value),
                    '--tts-api-key', str(self.get_parameter('xfyun_tts_api_key').value),
                    '--tts-api-secret', str(self.get_parameter('xfyun_tts_api_secret').value),
                ])
                self.publish_text_file(text_path, publish_all_lines=True)
                self.play_audio(self.player_command, cn_audio_path)
            else:
                self.get_logger().error(f'Unsupported mode: {self.mode}')
                return

            record_path.unlink(missing_ok=True)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VoiceActivityNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
