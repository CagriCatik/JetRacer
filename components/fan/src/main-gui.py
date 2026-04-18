#!/usr/bin/env python3
import argparse
import subprocess
import tkinter as tk
from pathlib import Path
from tkinter import messagebox

import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

from main import DEFAULT_CONFIG_PATH, DEFAULT_PWM_PATH, DEFAULT_TEMP_PATH, compute_pwm, load_config, read_temp


class FanControlApp:
    def __init__(self, root: tk.Tk, config_path: Path, temp_path: Path, pwm_path: Path, print_only: bool) -> None:
        self.root = root
        self.root.title('Jetson Fan Control')
        self.config_path = config_path
        self.temp_path = temp_path
        self.pwm_path = pwm_path
        self.print_only = print_only
        self.config = load_config(self.config_path)
        self.samples = []
        self.temps = []
        self.pwms = []
        self._after_id = None

        if self.config.max_perf > 0:
            subprocess.run(['jetson_clocks'], check=False)

        self._build_ui()
        self._refresh()
        self.root.protocol('WM_DELETE_WINDOW', self._close)

    def _build_ui(self) -> None:
        self.status_var = tk.StringVar(value='Starting...')
        self.temp_var = tk.StringVar(value='-')
        self.pwm_var = tk.StringVar(value='-')

        tk.Label(self.root, text='Status').pack(anchor='w', padx=12, pady=(12, 0))
        tk.Label(self.root, textvariable=self.status_var).pack(anchor='w', padx=12)

        tk.Label(self.root, text='Temperature').pack(anchor='w', padx=12, pady=(8, 0))
        tk.Label(self.root, textvariable=self.temp_var).pack(anchor='w', padx=12)

        tk.Label(self.root, text='PWM').pack(anchor='w', padx=12, pady=(8, 0))
        tk.Label(self.root, textvariable=self.pwm_var).pack(anchor='w', padx=12)

        tk.Button(self.root, text='Reload config', command=self._reload_config).pack(anchor='w', padx=12, pady=12)

        figure = plt.Figure(figsize=(7, 5), dpi=100)
        self.temp_axis = figure.add_subplot(211)
        self.pwm_axis = figure.add_subplot(212)
        self.temp_line, = self.temp_axis.plot([], [], color='tab:red')
        self.pwm_line, = self.pwm_axis.plot([], [], color='tab:blue')
        self.temp_axis.set_ylabel('Temp [C]')
        self.pwm_axis.set_ylabel('PWM')
        self.pwm_axis.set_xlabel('Sample')
        figure.tight_layout()

        self.canvas = FigureCanvasTkAgg(figure, master=self.root)
        self.canvas.get_tk_widget().pack(fill='both', expand=True, padx=12, pady=(0, 12))

    def _reload_config(self) -> None:
        try:
            self.config = load_config(self.config_path)
            self.status_var.set(f'Config reloaded from {self.config_path}')
        except Exception as exc:  # noqa: BLE001
            messagebox.showerror('Config error', str(exc))

    def _write_pwm(self, pwm_value: int) -> None:
        if self.print_only:
            return
        self.pwm_path.write_text(f'{pwm_value}\n', encoding='utf-8')

    def _refresh(self) -> None:
        try:
            temp_c = read_temp(self.temp_path)
            pwm_value = compute_pwm(temp_c, self.config)
            self._write_pwm(pwm_value)

            sample_index = len(self.samples)
            self.samples.append(sample_index)
            self.temps.append(temp_c)
            self.pwms.append(pwm_value)

            self.temp_var.set(f'{temp_c:.1f} C')
            self.pwm_var.set(str(pwm_value))
            self.status_var.set('Running')

            self.temp_line.set_data(self.samples, self.temps)
            self.pwm_line.set_data(self.samples, self.pwms)

            self.temp_axis.relim()
            self.temp_axis.autoscale_view()
            self.pwm_axis.relim()
            self.pwm_axis.autoscale_view()
            self.canvas.draw_idle()
        except Exception as exc:  # noqa: BLE001
            self.status_var.set('Error')
            messagebox.showerror('Fan control error', str(exc))
            return

        interval_ms = max(int(self.config.update_interval * 1000), 250)
        self._after_id = self.root.after(interval_ms, self._refresh)

    def _close(self) -> None:
        if self._after_id is not None:
            self.root.after_cancel(self._after_id)
        self.root.destroy()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Jetson fan-control GUI.')
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG_PATH)
    parser.add_argument('--temp-path', type=Path, default=DEFAULT_TEMP_PATH)
    parser.add_argument('--pwm-path', type=Path, default=DEFAULT_PWM_PATH)
    parser.add_argument('--print-only', action='store_true')
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    root = tk.Tk()
    FanControlApp(root, args.config, args.temp_path, args.pwm_path, args.print_only)
    root.mainloop()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
