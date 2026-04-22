<div align="center">

# JetRacer ROS 2
**Autonomy | Safety | Jetson Nano Optimized**

[![ROS 2](https://img.shields.io/badge/ROS_2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Jetson Host](https://img.shields.io/badge/Jetson_Host-Ubuntu_20.04_Workaround-E95420.svg?logo=ubuntu&logoColor=white)](docs/00_ROS2-Jetson-Nano.md)
[![Platform](https://img.shields.io/badge/Platform-Jetson_Nano-76B900.svg?logo=nvidia&logoColor=white)](https://developer.nvidia.com/embedded-computing)
[![Docker](https://img.shields.io/badge/Container-Docker-informational.svg?logo=docker&logoColor=white)](https://docs.docker.com/)
[![Docker Compose](https://img.shields.io/badge/Orchestration-Docker_Compose-2496ED.svg?logo=docker&logoColor=white)](https://docs.docker.com/compose/)
[![Nav2](https://img.shields.io/badge/Navigation-Nav2-1f6feb.svg)](https://navigation.ros.org/)
[![SLAM](https://img.shields.io/badge/SLAM-slam__toolbox-0A7E8C.svg)](https://github.com/SteveMacenski/slam_toolbox)
[![RViz](https://img.shields.io/badge/Visualization-RViz2-4B8BBE.svg)](https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/RViz-User-Guide/RViz-User-Guide.html)
[![Foxglove](https://img.shields.io/badge/Telemetry-Foxglove_Bridge-FF6B35.svg)](https://foxglove.dev/)
[![System](https://img.shields.io/badge/Architecture-Hardened-success.svg)](#production-hardening-roadmap)
[![Diagnostics](https://img.shields.io/badge/Telemetry-Diagnostic_Ready-informational.svg)](#-active-diagnostics)
[![Sentinel](https://img.shields.io/badge/Interface-Gamepad_Messenger-blueviolet.svg)](#one-button-mission-control)
[![OLED](https://img.shields.io/badge/Display-OLED_Dashboard-blue.svg)](#onboard-telemetry)
[![Platform](https://img.shields.io/badge/Hardware-Jetson_Nano-76B900.svg?logo=nvidia&logoColor=white)](https://developer.nvidia.com/embedded-computing)

*Professional grade ROS 2 Humble transformation for the Waveshare JetRacer platform. Designed for stability, deterministic control, and comprehensive observability.*

</div>

---

## Production Hardening
This repository has undergone a comprehensive architectural audit and hardening process to transition from a prototype to a deployment-ready robotics platform.

### Key Enhancements
*   **Arbitration Integrity:** All navigation and behavior commands are routed through `twist_mux` with strict priority-based overrides (Emergency > Manual > Behavior > Lane > Nav2).
*   **Environmental Portability:** Removed all absolute path dependencies. ML models (YOLO, Vosk) and configurations utilize `ament_index` for dynamic, system-agnostic resolution.
*   **Native Performance:** Critical control bridges (Ackermann conversion) have been ported from Python to **Native C++** to reduce IPC latency and jitter on the Jetson Nano.
*   **Fail-Safe Design:** Implemented a hardware "Heart-Stop" in the serial bridge and tightened safety watchdogs (0.1s) for immediate teleop disconnection handling.

---

## Repository Architecture

| Package | Responsibility | Language/Tech |
| :--- | :--- | :--- |
| **jetracer_bringup** | Launch orchestration & global overrides | Python / YAML |
| **jetracer_hardware** | C++ Serial Bridge & Diagnostic Aggregator | C++ / Asio |
| **jetracer_behavior** | Collision Assurance & Wheel-Slip Monitor | Python / LiDAR |
| **jetracer_navigation**| Nav2 Integration & Ackermann Bridge | C++ / Nav2 |
| **jetracer_lane_following**| High-speed Stanley/PID Controller | Python / OpenCV |
| **jetracer_perception** | YOLO11 Object Detection & Camera | Python / CUDA |

---

## Deployment Guide

### 1. Containerized Setup
Prepare your Jetson Nano using the [Ubuntu 20.04 Workaround](docs/00_ROS2-Jetson-Nano.md).

```bash
# Build and enter the hardened workspace
docker compose up -d --build
docker exec -it jetracer_workspace bash

# Build the C++ components
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# Register the one-button autostart service
bash install_service.sh
```

## Operation Guide

### 1. Manual Driving (Always-on)
The robot boots into **Sentinel Mode**. You can drive manually immediately by holding **L2** (Deadman Switch).
*   **Steering**: Left Stick
*   **Throttle**: Right Stick (Vertical)

### 2. One-Button Mission Control
Trigger full autonomous launches directly from the gamepad:

| Mission | Combo | Description |
| :--- | :--- | :--- |
| **Autonomy** | `SELECT` + `START` | Full Lane/YOLO perception stack. |
| **Mapping** | `SELECT` + `X` | SLAM + Nav2 Grid Mapping. |
| **E-Stop** | `MODE` | Immediate mission teardown and stop. |

### 3. Manual Launching (Advanced)
If you wish to launch specific stacks manually without the sentinel:

```bash
# Core hardware and safety stack only
ros2 launch jetracer_bringup jetracer.launch.py

# Full autonomous behavior suite
ros2 launch jetracer_bringup autonomy.launch.py
```

---

## Active Diagnostics
The system utilizes the ROS 2 Diagnostic stack. Monitor hardware health in real-time:

```bash
ros2 topic echo /diagnostics
```

**Monitored Metrics:**
*   **Serial Status:** Port connectivity and throughput.
*   **Heartbeat Frequency:** Command freshness and safety timing.
*   **Sensor Streaming:** IMU and Wheel Odom update rates.
*   **Onboard Telemetry:** Physical OLED display showing IP, Battery %, and Thermal Pressure.
*   **Thermal Health:** Real-time throttling monitor for each Nano CPU/GPU core.
*   **Wheel Slip:** Real-time divergence check between IMU and Encoders.

---

## Testing & Validation
Verify system health before high-speed deployments using the integrated smoke tests:

```bash
colcon test --packages-select jetracer_bringup
colcon test-result --all
```

---

## Documentation Index
- [System Architecture Audit](docs/01_System_Overview.md)
- [Hardening & Safety Implementation](docs/06_Behavior_and_Arbitration.md)
- [Jetson Nano Optimization](docs/03_Deployment_and_Docker.md)
- [Hardware Diagnostic Specs](docs/02_Hardware_and_Assembly.md)