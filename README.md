# JetRacer ROS 2 Humble Autonomous Stack

[![ROS 2](https://img.shields.io/badge/ROS_2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-E95420.svg?logo=ubuntu&logoColor=white)](https://releases.ubuntu.com/22.04/)
[![Platform](https://img.shields.io/badge/Platform-Jetson_Nano-76B900.svg?logo=nvidia&logoColor=white)](https://developer.nvidia.com/embedded-computing)
[![Docker](https://img.shields.io/badge/Container-Docker-informational.svg?logo=docker&logoColor=white)](https://docs.docker.com/)
[![Docker Compose](https://img.shields.io/badge/Orchestration-Docker_Compose-2496ED.svg?logo=docker&logoColor=white)](https://docs.docker.com/compose/)
[![Nav2](https://img.shields.io/badge/Navigation-Nav2-1f6feb.svg)](https://navigation.ros.org/)
[![SLAM](https://img.shields.io/badge/SLAM-slam__toolbox-0A7E8C.svg)](https://github.com/SteveMacenski/slam_toolbox)
[![Foxglove](https://img.shields.io/badge/Telemetry-Foxglove_Bridge-FF6B35.svg)](https://foxglove.dev/)

Production-focused ROS 2 Humble stack for the Waveshare JetRacer platform, with modular bringup, perception, behavior, navigation, and voice packages.

## Supported Runtime

- ROS distro: ROS 2 Humble
- Policy: ROS 2 only (no ROS1 bridge, no mixed ROS1/ROS2 runtime)
- Target platform: Jetson Nano workflows documented in [docs/03_Deployment_and_Docker.md](docs/03_Deployment_and_Docker.md)

## Key Capabilities

- Safety command arbitration through `twist_mux`
- Lane following with Stanley lateral control + PID longitudinal control
- Nav2-based mapping and navigation (`slam_toolbox` or Cartographer)
- YOLO-based semantic perception and behavior hooks
- Collision assurance from LiDAR data
- Centralized parameter tuning via a single YAML file

## Repository Layout

- `src/jetracer_bringup`: top-level launch orchestration and shared configs
- `src/jetracer_hardware`: hardware interfaces and calibration
- `src/jetracer_localization`: EKF and localization integration
- `src/jetracer_lane_following`: lane detection and control node
- `src/jetracer_navigation`: Nav2 launch/config and steering adapter
- `src/jetracer_perception`: camera perception nodes (YOLO and trackers)
- `src/jetracer_behavior`: behavior and safety nodes
- `src/jetracer_voice`: offline voice stack
- `docs`: architecture and operations documentation

## Quick Start (Docker, Recommended)

From repository root:

```bash
docker compose up -d --build
docker exec -it jetracer_workspace bash
```

Inside container:

```bash
source ~/.bashrc
rosdep update
rosdep install --from-paths src --ignore-src -r -y
build_workspace
source install/setup.bash
```

## Launch Profiles

Base robot bringup:

```bash
ros2 launch jetracer_bringup jetracer.launch.py
```

Full autonomy pipeline (lane + yolo + behavior + collision + foxglove):

```bash
ros2 launch jetracer_bringup autonomy.launch.py
```

Lane-following stack:

```bash
ros2 launch jetracer_bringup lane_following.launch.py
```

Enable motion from lane follower:

```bash
ros2 param set /lane_following start true
```

Navigation against a saved map:

```bash
ros2 launch jetracer_bringup nav.launch.py
```

SLAM + navigation:

```bash
ros2 launch jetracer_bringup slam_nav.launch.py
```

Foxglove Studio websocket default:

- `ws://<JETSON_IP_ADDRESS>:8765`

## Centralized Configuration

Primary runtime config:

- `src/jetracer_bringup/config/main_config.yaml`

This file contains parameters for:

- `lane_following`
- `yolo_detection`
- `semantic_behavior`
- `collision_assurance`
- `cmd_vel_to_steering`
- `foxglove_bridge`
- `multipoint_navigation`
- `voice_commander`

All top-level bringup launch files accept:

- `config_file:=<path-to-yaml>`

Example:

```bash
ros2 launch jetracer_bringup autonomy.launch.py \
  config_file:=/path/to/custom_main_config.yaml
```

## Documentation

- [01 System Overview](docs/01_System_Overview.md)
- [02 Hardware and Assembly](docs/02_Hardware_and_Assembly.md)
- [03 Deployment and Docker](docs/03_Deployment_and_Docker.md)
- [04 Perception Stack](docs/04_Perception_Stack.md)
- [05 Navigation and SLAM](docs/05_Navigation_and_SLAM.md)
- [06 Behavior and Arbitration](docs/06_Behavior_and_Arbitration.md)
- [07 Voice Commander Stack](docs/07_Voice_Commander_Stack.md)
- [08 Central Configuration](docs/08_Central_Configuration.md)

## Setup Scripts

Setup helpers and compatibility wrappers are documented in:

- `setup/README.md`
