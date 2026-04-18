#!/usr/bin/env python3

import argparse
import json
from pathlib import Path

import google.auth.transport.grpc
import google.auth.transport.requests
import google.oauth2.credentials
from google.assistant.embedded.v1alpha2 import embedded_assistant_pb2
from google.assistant.embedded.v1alpha2 import embedded_assistant_pb2_grpc


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Google Assistant audio-file helper.')
    parser.add_argument('-i', '--input-audio-file', required=True)
    parser.add_argument('-o', '--output-audio-file', required=True)
    parser.add_argument('--text-output-file', required=True)
    parser.add_argument('--credentials', required=True)
    parser.add_argument('--lang', default='en-US')
    parser.add_argument('--api-endpoint', default='embeddedassistant.googleapis.com')
    parser.add_argument('--device-model-id', required=True)
    parser.add_argument('--device-id', required=True)
    parser.add_argument('--block-size', type=int, default=1024)
    parser.add_argument('--grpc-deadline', type=int, default=300)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    with open(args.credentials, 'r', encoding='utf-8') as handle:
        credentials = google.oauth2.credentials.Credentials(token=None, **json.load(handle))
    http_request = google.auth.transport.requests.Request()
    credentials.refresh(http_request)

    channel = google.auth.transport.grpc.secure_authorized_channel(
        credentials, http_request, args.api_endpoint)
    assistant = embedded_assistant_pb2_grpc.EmbeddedAssistantStub(channel)

    input_path = Path(args.input_audio_file)
    output_path = Path(args.output_audio_file)
    text_output_path = Path(args.text_output_file)
    transcript = ''
    answer = ''

    def gen_assist_requests():
        dialog_state_in = embedded_assistant_pb2.DialogStateIn(
            language_code=args.lang,
            conversation_state=b'',
        )
        config = embedded_assistant_pb2.AssistConfig(
            audio_in_config=embedded_assistant_pb2.AudioInConfig(
                encoding='LINEAR16',
                sample_rate_hertz=16000,
            ),
            audio_out_config=embedded_assistant_pb2.AudioOutConfig(
                encoding='LINEAR16',
                sample_rate_hertz=16000,
                volume_percentage=100,
            ),
            dialog_state_in=dialog_state_in,
            device_config=embedded_assistant_pb2.DeviceConfig(
                device_id=args.device_id,
                device_model_id=args.device_model_id,
            ),
        )
        yield embedded_assistant_pb2.AssistRequest(config=config)
        with input_path.open('rb') as input_handle:
            while True:
                data = input_handle.read(args.block_size)
                if not data:
                    break
                yield embedded_assistant_pb2.AssistRequest(audio_in=data)

    with output_path.open('wb') as audio_handle:
        for response in assistant.Assist(gen_assist_requests(), args.grpc_deadline):
            if response.speech_results:
                transcript = ' '.join(result.transcript for result in response.speech_results)
            if response.audio_out.audio_data:
                audio_handle.write(response.audio_out.audio_data)
            if response.dialog_state_out.supplemental_display_text:
                answer = response.dialog_state_out.supplemental_display_text

    text_output_path.parent.mkdir(parents=True, exist_ok=True)
    lines = [transcript.strip()]
    if answer.strip():
        lines.append(answer.strip())
    text_output_path.write_text('\n'.join(filter(None, lines)), encoding='utf-8')


if __name__ == '__main__':
    main()
