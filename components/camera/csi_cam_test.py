#!/usr/bin/env python3
import argparse
import time
import cv2


def gstreamer_pipeline(
    capture_width: int,
    capture_height: int,
    display_width: int,
    display_height: int,
    framerate: int,
    flip_method: int,
) -> str:
    return (
        'nvarguscamerasrc ! '
        f'video/x-raw(memory:NVMM), width=(int){capture_width}, height=(int){capture_height}, '
        f'format=(string)NV12, framerate=(fraction){framerate}/1 ! '
        f'nvvidconv flip-method={flip_method} ! '
        f'video/x-raw, width=(int){display_width}, height=(int){display_height}, format=(string)BGRx ! '
        'videoconvert ! video/x-raw, format=(string)BGR ! appsink'
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Jetson CSI camera smoke test.')
    parser.add_argument('--capture-width', type=int, default=1280)
    parser.add_argument('--capture-height', type=int, default=720)
    parser.add_argument('--display-width', type=int, default=1280)
    parser.add_argument('--display-height', type=int, default=720)
    parser.add_argument('--framerate', type=int, default=30)
    parser.add_argument('--flip-method', type=int, default=0)
    parser.add_argument('--frames', type=int, default=0, help='Stop after N frames. Use 0 to run until interrupted.')
    parser.add_argument('--headless', action='store_true', help='Capture frames without opening a window.')
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    pipeline = gstreamer_pipeline(
        capture_width=args.capture_width,
        capture_height=args.capture_height,
        display_width=args.display_width,
        display_height=args.display_height,
        framerate=args.framerate,
        flip_method=args.flip_method,
    )

    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        print('Error: unable to open the CSI camera pipeline.')
        return 1

    frame_count = 0
    start_time = time.time()

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print('Error: failed to read a frame from the camera.')
                return 1

            frame_count += 1
            if not args.headless:
                cv2.imshow('Jetson CSI Camera', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            if args.frames and frame_count >= args.frames:
                break
    finally:
        cap.release()
        if not args.headless:
            cv2.destroyAllWindows()

    elapsed = max(time.time() - start_time, 1e-6)
    print(f'Captured {frame_count} frame(s) in {elapsed:.2f}s ({frame_count / elapsed:.2f} FPS).')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
