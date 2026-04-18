#!/usr/bin/env python3

import argparse
import base64
from datetime import datetime
import hashlib
import hmac
import json
from pathlib import Path
import ssl
import time
from urllib.parse import urlencode
from wsgiref.handlers import format_date_time
from time import mktime

import requests
import websocket


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='XFYun AIUI helper with optional TTS response.')
    parser.add_argument('--aiui-appid', required=True)
    parser.add_argument('--aiui-api-key', required=True)
    parser.add_argument('--auth-id', required=True)
    parser.add_argument('--audio-file', required=True)
    parser.add_argument('--text-output-file', required=True)
    parser.add_argument('--audio-output-file', required=True)
    parser.add_argument('--tts-appid', required=True)
    parser.add_argument('--tts-api-key', required=True)
    parser.add_argument('--tts-api-secret', required=True)
    parser.add_argument('--tts-voice', default='xiaoyan')
    return parser.parse_args()


def build_aiui_headers(api_key: str, appid: str, auth_id: str) -> dict[str, str]:
    current_time = str(int(time.time()))
    param = json.dumps({
        'result_level': 'complete',
        'auth_id': auth_id,
        'data_type': 'audio',
        'sample_rate': '16000',
        'scene': 'main',
    }, separators=(',', ':'))
    param_base64 = base64.b64encode(param.encode('utf-8')).decode('utf-8')
    checksum = hashlib.md5((api_key + current_time + param_base64).encode('utf-8')).hexdigest()
    return {
        'X-CurTime': current_time,
        'X-Param': param_base64,
        'X-Appid': appid,
        'X-CheckSum': checksum,
    }


def create_tts_url(api_key: str, api_secret: str) -> str:
    date = format_date_time(mktime(datetime.now().timetuple()))
    signature_origin = 'host: ws-api.xfyun.cn\n'
    signature_origin += f'date: {date}\n'
    signature_origin += 'GET /v2/tts HTTP/1.1'
    signature_sha = hmac.new(
        api_secret.encode('utf-8'),
        signature_origin.encode('utf-8'),
        digestmod=hashlib.sha256,
    ).digest()
    authorization_origin = (
        f'api_key=\"{api_key}\", algorithm=\"hmac-sha256\", '
        f'headers=\"host date request-line\", signature=\"{base64.b64encode(signature_sha).decode()}\"'
    )
    return 'wss://tts-api.xfyun.cn/v2/tts?' + urlencode({
        'authorization': base64.b64encode(authorization_origin.encode('utf-8')).decode('utf-8'),
        'date': date,
        'host': 'ws-api.xfyun.cn',
    })


def synthesize_tts(appid: str, api_key: str, api_secret: str, voice: str, text: str, output_path: Path) -> None:
    audio_chunks: list[bytes] = []

    def on_message(ws, message) -> None:
        payload = json.loads(message)
        if payload.get('code') != 0:
            raise RuntimeError(payload.get('message', 'XFYun TTS error'))
        data = payload.get('data', {})
        audio = data.get('audio')
        if audio:
            audio_chunks.append(base64.b64decode(audio))
        if data.get('status') == 2:
            ws.close()

    def on_open(ws) -> None:
        ws.send(json.dumps({
            'common': {'app_id': appid},
            'business': {
                'aue': 'lame',
                'auf': 'audio/L16;rate=16000',
                'vcn': voice,
                'tte': 'utf8',
            },
            'data': {
                'status': 2,
                'text': base64.b64encode(text.encode('utf-8')).decode('utf-8'),
            },
        }))

    ws = websocket.WebSocketApp(create_tts_url(api_key, api_secret), on_message=on_message, on_open=on_open)
    ws.run_forever(sslopt={'cert_reqs': ssl.CERT_NONE})
    output_path.write_bytes(b''.join(audio_chunks))


def main() -> None:
    args = parse_args()
    audio_path = Path(args.audio_file)
    text_output_path = Path(args.text_output_file)
    audio_output_path = Path(args.audio_output_file)

    response = requests.post(
        'http://openapi.xfyun.cn/v2/aiui',
        headers=build_aiui_headers(args.aiui_api_key, args.aiui_appid, args.auth_id),
        data=audio_path.read_bytes(),
        timeout=60,
    )
    response.raise_for_status()
    payload = response.json()

    heard_text = ''
    answer_text = ''
    for item in payload.get('data', []):
        if item.get('sub') == 'nlp':
            intent = item.get('intent', {})
            heard_text = intent.get('text', '')
            answer_text = intent.get('answer', {}).get('text', '')
            if heard_text or answer_text:
                break

    text_output_path.parent.mkdir(parents=True, exist_ok=True)
    lines = [heard_text.strip()]
    if answer_text.strip():
        lines.append(answer_text.strip())
    text_output_path.write_text('\n'.join(filter(None, lines)), encoding='utf-8')

    if answer_text.strip():
        synthesize_tts(
            args.tts_appid,
            args.tts_api_key,
            args.tts_api_secret,
            args.tts_voice,
            answer_text,
            audio_output_path,
        )


if __name__ == '__main__':
    main()
