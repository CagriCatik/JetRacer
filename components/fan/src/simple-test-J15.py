#!/usr/bin/env python3
import argparse
import itertools
import time

import Jetson.GPIO as GPIO


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Simple board-pin toggle test for Jetson fan experiments.')
    parser.add_argument('--pin', type=int, default=15, help='Physical BOARD pin number to toggle.')
    parser.add_argument('--cycles', type=int, default=10, help='Number of on/off cycles. Use 0 for infinite.')
    parser.add_argument('--on-seconds', type=float, default=2.0)
    parser.add_argument('--off-seconds', type=float, default=2.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    cycle_iter = itertools.count() if args.cycles == 0 else range(args.cycles)

    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(args.pin, GPIO.OUT, initial=GPIO.LOW)

    try:
        for cycle in cycle_iter:
            GPIO.output(args.pin, GPIO.HIGH)
            print(f'Cycle {cycle + 1}: pin {args.pin} HIGH')
            time.sleep(max(args.on_seconds, 0.0))

            GPIO.output(args.pin, GPIO.LOW)
            print(f'Cycle {cycle + 1}: pin {args.pin} LOW')
            time.sleep(max(args.off_seconds, 0.0))
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup(args.pin)

    return 0


if __name__ == '__main__':
    raise SystemExit(main())
