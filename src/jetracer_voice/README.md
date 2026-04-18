# jetracer_voice

ROS 2 migration of the ROS 1 JetRacer voice stack.

## Migrated nodes

- `tts_en.py`: English TTS subscriber using `gTTS`
- `tts_cn.py`: Chinese TTS subscriber using the XFYun websocket API
- `vad.py`: microphone capture and VAD entry point for ASR or assistant-style dialog

## Helper scripts

- `ginput.py`: Google Assistant audio-file helper
- `iat.py`: XFYun speech-to-text helper
- `aiui.py`: XFYun dialog helper with TTS output

## Notes

- API credentials are no longer hard-coded in source; they are passed as ROS parameters or CLI arguments.
- Runtime artifacts are written into a configurable runtime directory instead of the package source tree.
- Python dependencies such as `gtts`, `pyaudio`, `webrtcvad`, `websocket-client`, and Google Assistant client libraries remain external runtime requirements.
