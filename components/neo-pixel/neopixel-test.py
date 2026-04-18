#!/usr/bin/env python3
import argparse
import itertools
import time

import board
import neopixel


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='NeoPixel rainbow-cycle smoke test.')
    parser.add_argument('--pixels', type=int, default=8, help='Number of LEDs in the strip.')
    parser.add_argument('--brightness', type=float, default=0.2)
    parser.add_argument('--delay', type=float, default=0.01, help='Delay between frame updates.')
    parser.add_argument('--iterations', type=int, default=1, help='Number of rainbow passes. Use 0 for infinite.')
    parser.add_argument('--pin', default='D18', help='Blinka pin name, for example D18.')
    return parser.parse_args()


def wheel(position: int) -> tuple[int, int, int]:
    position %= 256
    if position < 85:
        return position * 3, 255 - position * 3, 0
    if position < 170:
        position -= 85
        return 255 - position * 3, 0, position * 3
    position -= 170
    return 0, position * 3, 255 - position * 3


def main() -> int:
    args = parse_args()
    pin = getattr(board, args.pin)
    pixels = neopixel.NeoPixel(pin, args.pixels, brightness=args.brightness, auto_write=False)

    iteration_range = itertools.count() if args.iterations == 0 else range(args.iterations)
    try:
        for _ in iteration_range:
            for offset in range(255):
                for index in range(args.pixels):
                    pixel_index = (index * 256 // max(args.pixels, 1)) + offset
                    pixels[index] = wheel(pixel_index)
                pixels.show()
                time.sleep(max(args.delay, 0.0))
    except KeyboardInterrupt:
        pass
    finally:
        pixels.fill((0, 0, 0))
        pixels.show()

    return 0


if __name__ == '__main__':
    raise SystemExit(main())
