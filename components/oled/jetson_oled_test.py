#!/usr/bin/env python3
import argparse
import time

import board
import busio
from PIL import Image, ImageDraw, ImageFont

import adafruit_ssd1306


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='SSD1306 OLED smoke test.')
    parser.add_argument('--message', default='JetRacer OLED ready')
    parser.add_argument('--width', type=int, default=128)
    parser.add_argument('--height', type=int, default=64)
    parser.add_argument('--duration', type=float, default=5.0)
    parser.add_argument('--reset-pin', default='', help='Optional board pin name such as D4.')
    return parser.parse_args()


def centered_text(draw: ImageDraw.ImageDraw, text: str, width: int, height: int, font: ImageFont.ImageFont) -> tuple[int, int]:
    left, top, right, bottom = draw.textbbox((0, 0), text, font=font)
    text_width = right - left
    text_height = bottom - top
    return (width - text_width) // 2, (height - text_height) // 2


def main() -> int:
    args = parse_args()
    i2c = busio.I2C(board.SCL, board.SDA)

    reset = getattr(board, args.reset_pin) if args.reset_pin else None
    display = adafruit_ssd1306.SSD1306_I2C(args.width, args.height, i2c, reset=reset)

    display.fill(0)
    display.show()

    image = Image.new('1', (display.width, display.height))
    draw = ImageDraw.Draw(image)
    font = ImageFont.load_default()

    draw.rectangle((0, 0, display.width, display.height), outline=0, fill=0)
    x_pos, y_pos = centered_text(draw, args.message, display.width, display.height, font)
    draw.text((x_pos, y_pos), args.message, font=font, fill=255)

    display.image(image)
    display.show()
    time.sleep(max(args.duration, 0.0))

    display.fill(0)
    display.show()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
