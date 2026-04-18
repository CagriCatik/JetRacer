# Components Standalone Test Guide

This folder contains hardware-focused diagnostics you can run independently from the ROS 2 stack.

## Subfolders

| Folder | Purpose | Primary test command |
| :--- | :--- | :--- |
| `camera` | CSI camera bringup and FPS smoke test (OpenCV + GStreamer) | `python camera/csi_cam_test.py --frames 120 --headless` |
| `imu` | I2C IMU sampling (accel + gyro) | `python imu/jetson_imu_test.py --samples 20` |
| `neo-pixel` | WS2812 RGB strip/ring animation test | `sudo -E .venv/bin/python neo-pixel/neopixel-test.py --iterations 2` |
| `oled` | SSD1306 I2C OLED text rendering test | `python oled/jetson_oled_test.py --message "OLED OK" --duration 3` |
| `speaker` | ALSA speaker output validation (`speaker-test`) | `python speaker/jetson_speaker_test.py` |
| `fan` | Host-level PWM/systemd fan controller install and validation | `sudo ./fan/jetson-fan-control-install.sh` |

## Standalone Python venv Setup

Run on the Jetson host:

```bash
cd components
sudo apt-get update
sudo apt-get install -y python3-venv python3-pip python3-opencv i2c-tools alsa-utils
python3 -m venv .venv --system-site-packages
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements-standalone.txt
```

Notes:

- `--system-site-packages` allows the venv to reuse Jetson OS packages like `cv2` (`python3-opencv`).
- For GPIO/NeoPixel and some I2C paths, run with `sudo -E .venv/bin/python ...`.
- Avoid running these tests while the full robot stack is active to prevent device contention (`/dev/i2c-*`, `/dev/snd`, camera).

## Standalone Test Commands

From `components/` with venv active (`source .venv/bin/activate`):

```bash
python camera/csi_cam_test.py --frames 120 --headless
python imu/jetson_imu_test.py --samples 20 --rate-hz 5
python oled/jetson_oled_test.py --message "OLED OK" --duration 3
python speaker/jetson_speaker_test.py
sudo -E .venv/bin/python neo-pixel/neopixel-test.py --pixels 8 --iterations 2
```

## Fan Component (Host Service)

The fan controller is host-level infrastructure and not a ROS node.

Install:

```bash
sudo ./fan/jetson-fan-control-install.sh
```

Uninstall:

```bash
sudo ./fan/jetson-fan-control-uninstall.sh
```

Dry-run logic test without writing PWM:

```bash
python fan/src/main.py --once --print-only
```

## Cleanup

```bash
deactivate
```
