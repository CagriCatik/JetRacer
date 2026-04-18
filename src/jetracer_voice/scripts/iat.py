#!/usr/bin/env python3

import argparse
import base64
from datetime import datetime
import hashlib
import hmac
import json
import ssl
import time
from urllib.parse import urlencode
from wsgiref.handlers import format_date_time
from time import mktime

import websocket


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='XFYun speech-to-text helper.')
    parser.add_argument('--appid', required=True)
    parser.add_argument('--api-key', required=True)
    parser.add_argument('--api-secret', required=True)
    parser.add_argument('--audio-file', required=True)
    parser.add_argument('--text-output-file', required=True)
    return parser.parse_args()


def create_url(appid: str, api_key: str, api_secret: str) -> str:
    del appid
    date = format_date_time(mktime(datetime.now().timetuple()))
    signature_origin = 'host: ws-api.xfyun.cn\n'
    signature_origin += f'date: {date}\n'
    signature_origin += 'GET /v2/iat HTTP/1.1'
    signature_sha = hmac.new(
        api_secret.encode('utf-8'),
        signature_origin.encode('utf-8'),
        digestmod=hashlib.sha256,
    ).digest()
    authorization_origin = (
        f'api_key=\"{api_key}\", algorithm=\"hmac-sha256\", '
        f'headers=\"host date request-line\", signature=\"{base64.b64encode(signature_sha).decode()}\"'
    )
    return 'wss://ws-api.xfyun.cn/v2/iat?' + urlencode({
        'authorization': base64.b64encode(authorization_origin.encode('utf-8')).decode('utf-8'),
        'date': date,
        'host': 'ws-api.xfyun.cn',
    })


def main() -> None:
    args = parse_args()
    transcript_parts: list[str] = []

    def on_message(ws, message) -> None:
        payload = json.loads(message)
        if payload.get('code') != 0:
            raise RuntimeError(payload.get('message', 'XFYun IAT error'))
        for item in payload.get('data', {}).get('result', {}).get('ws', []):
            for candidate in item.get('cw', []):
                transcript_parts.append(candidate.get('w', ''))

    def on_open(ws) -> None:
        frame_size = 8000
        interval = 0.04
        status = 0
        with open(args.audio_file, 'rb') as audio_handle:
            while True:
                chunk = audio_handle.read(frame_size)
                if not chunk:
                    status = 2
                if status == 0:
                    payload = {
                        'common': {'app_id': args.appid},
                        'business': {
                            'domain': 'iat',
                            'language': 'zh_cn',
                            'accent': 'mandarin',
                            'vinfo': 1,
                            'vad_eos': 10000,
                        },
                        'data': {
                            'status': 0,
                            'format': 'audio/L16;rate=16000',
                            'audio': base64.b64encode(chunk).decode('utf-8'),
                            'encoding': 'raw',
                        },
                    }
                    ws.send(json.dumps(payload))
                    status = 1
                elif status == 1:
                    ws.send(json.dumps({
                        'data': {
                            'status': 1,
                            'format': 'audio/L16;rate=16000',
                            'audio': base64.b64encode(chunk).decode('utf-8'),
                            'encoding': 'raw',
                        }
                    }))
                else:
                    ws.send(json.dumps({
                        'data': {
                            'status': 2,
                            'format': 'audio/L16;rate=16000',
                            'audio': base64.b64encode(chunk).decode('utf-8'),
                            'encoding': 'raw',
                        }
                    }))
                    time.sleep(1.0)
                    ws.close()
                    break
                time.sleep(interval)

    ws = websocket.WebSocketApp(
        create_url(args.appid, args.api_key, args.api_secret),
        on_message=on_message,
        on_open=on_open,
    )
    ws.run_forever(sslopt={'cert_reqs': ssl.CERT_NONE})
    with open(args.text_output_file, 'w', encoding='utf-8') as output_handle:
        output_handle.write(''.join(transcript_parts).strip())


if __name__ == '__main__':
    main()
