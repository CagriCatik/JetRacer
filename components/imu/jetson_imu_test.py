#!/usr/bin/env python3
import argparse
import time

from smbus2 import SMBus

MPU9250_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43

ACCEL_SCALE_2G = 16384.0
GYRO_SCALE_250DPS = 131.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Read raw MPU9250 accelerometer and gyroscope data.')
    parser.add_argument('--bus', type=int, default=1, help='I2C bus number.')
    parser.add_argument('--address', type=lambda value: int(value, 0), default=MPU9250_ADDR, help='I2C address, for example 0x68.')
    parser.add_argument('--rate-hz', type=float, default=5.0, help='Sampling rate in Hz.')
    parser.add_argument('--samples', type=int, default=0, help='Number of samples to read. Use 0 to run until interrupted.')
    return parser.parse_args()


def read_word(bus: SMBus, address: int, register: int) -> int:
    high = bus.read_byte_data(address, register)
    low = bus.read_byte_data(address, register + 1)
    value = (high << 8) | low
    return value - 65536 if value >= 0x8000 else value


def initialize_sensor(bus: SMBus, address: int) -> None:
    bus.write_byte_data(address, PWR_MGMT_1, 0)
    time.sleep(0.1)


def read_accel_gyro(bus: SMBus, address: int) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    accel = (
        read_word(bus, address, ACCEL_XOUT_H) / ACCEL_SCALE_2G,
        read_word(bus, address, ACCEL_XOUT_H + 2) / ACCEL_SCALE_2G,
        read_word(bus, address, ACCEL_XOUT_H + 4) / ACCEL_SCALE_2G,
    )
    gyro = (
        read_word(bus, address, GYRO_XOUT_H) / GYRO_SCALE_250DPS,
        read_word(bus, address, GYRO_XOUT_H + 2) / GYRO_SCALE_250DPS,
        read_word(bus, address, GYRO_XOUT_H + 4) / GYRO_SCALE_250DPS,
    )
    return accel, gyro


def main() -> int:
    args = parse_args()
    period = 1.0 / args.rate_hz if args.rate_hz > 0 else 0.2

    with SMBus(args.bus) as bus:
        initialize_sensor(bus, args.address)

        count = 0
        try:
            while True:
                accel, gyro = read_accel_gyro(bus, args.address)
                count += 1
                print(
                    'accel[g] '
                    f'x={accel[0]: .3f} y={accel[1]: .3f} z={accel[2]: .3f} | '
                    'gyro[dps] '
                    f'x={gyro[0]: .3f} y={gyro[1]: .3f} z={gyro[2]: .3f}'
                )

                if args.samples and count >= args.samples:
                    break
                time.sleep(period)
        except KeyboardInterrupt:
            pass

    return 0


if __name__ == '__main__':
    raise SystemExit(main())
