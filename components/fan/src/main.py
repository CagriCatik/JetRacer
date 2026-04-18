#!/usr/bin/env python3
import argparse
import json
import signal
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path

DEFAULT_CONFIG_PATH = Path('/etc/automagic-fan/config.json')
DEFAULT_TEMP_PATH = Path('/sys/devices/virtual/thermal/thermal_zone0/temp')
DEFAULT_PWM_PATH = Path('/sys/devices/pwm-fan/target_pwm')


@dataclass(frozen=True)
class FanConfig:
    fan_off_temp: float = 35.0
    fan_max_temp: float = 60.0
    update_interval: float = 2.0
    max_perf: int = 0

    @classmethod
    def from_dict(cls, data: dict) -> 'FanConfig':
        config = cls(
            fan_off_temp=float(data.get('FAN_OFF_TEMP', 35.0)),
            fan_max_temp=float(data.get('FAN_MAX_TEMP', 60.0)),
            update_interval=float(data.get('UPDATE_INTERVAL', 2.0)),
            max_perf=int(data.get('MAX_PERF', 0)),
        )
        if config.fan_max_temp <= config.fan_off_temp:
            raise ValueError('FAN_MAX_TEMP must be greater than FAN_OFF_TEMP.')
        if config.update_interval <= 0:
            raise ValueError('UPDATE_INTERVAL must be greater than zero.')
        return config


def load_config(path: Path) -> FanConfig:
    with path.open('r', encoding='utf-8') as handle:
        return FanConfig.from_dict(json.load(handle))


def read_temp(path: Path) -> float:
    return int(path.read_text(encoding='utf-8').strip()) / 1000.0


def compute_pwm(temp_c: float, config: FanConfig) -> int:
    if temp_c <= config.fan_off_temp:
        return 0
    if temp_c >= config.fan_max_temp:
        return 255
    span = config.fan_max_temp - config.fan_off_temp
    return int(255.0 * (temp_c - config.fan_off_temp) / span)


def maybe_enable_max_perf(config: FanConfig) -> None:
    if config.max_perf > 0:
        subprocess.run(['jetson_clocks'], check=False)


class FanController:
    def __init__(self, config: FanConfig, temp_path: Path, pwm_path: Path, print_only: bool) -> None:
        self.config = config
        self.temp_path = temp_path
        self.pwm_path = pwm_path
        self.print_only = print_only
        self.last_pwm = None
        self.running = True

    def stop(self, *_args) -> None:
        self.running = False

    def write_pwm(self, pwm_value: int) -> None:
        if self.print_only:
            return
        self.pwm_path.write_text(f'{pwm_value}\n', encoding='utf-8')

    def step(self) -> tuple[float, int]:
        temp_c = read_temp(self.temp_path)
        pwm_value = compute_pwm(temp_c, self.config)
        if pwm_value != self.last_pwm:
            self.write_pwm(pwm_value)
            self.last_pwm = pwm_value
        return temp_c, pwm_value

    def run(self, once: bool) -> int:
        while self.running:
            temp_c, pwm_value = self.step()
            print(f'temp={temp_c:.1f}C pwm={pwm_value}')
            if once:
                return 0
            time.sleep(self.config.update_interval)
        return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Jetson PWM fan controller.')
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG_PATH)
    parser.add_argument('--temp-path', type=Path, default=DEFAULT_TEMP_PATH)
    parser.add_argument('--pwm-path', type=Path, default=DEFAULT_PWM_PATH)
    parser.add_argument('--once', action='store_true', help='Read temperature and update PWM a single time.')
    parser.add_argument('--print-only', action='store_true', help='Do not write PWM values; print the computed output only.')
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    config = load_config(args.config)
    maybe_enable_max_perf(config)

    controller = FanController(
        config=config,
        temp_path=args.temp_path,
        pwm_path=args.pwm_path,
        print_only=args.print_only,
    )
    signal.signal(signal.SIGINT, controller.stop)
    signal.signal(signal.SIGTERM, controller.stop)
    return controller.run(once=args.once)


if __name__ == '__main__':
    raise SystemExit(main())
