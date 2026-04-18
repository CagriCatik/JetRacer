# 08. Central Configuration

The stack uses one editable runtime file for autonomous behavior tuning:

- `src/jetracer_bringup/config/main_config.yaml`

This file is loaded by bringup launches and applied to multiple nodes (lane following, YOLO, semantic behavior, collision assurance, steering conversion, foxglove bridge, and multipoint navigation).

## Why this file exists

- Keep tunables out of source code.
- Avoid drifting values across multiple launch files.
- Make field tuning fast and repeatable.

## Covered Node Sections

`main_config.yaml` includes:

- `lane_following`
- `yolo_detection`
- `semantic_behavior`
- `collision_assurance`
- `cmd_vel_to_steering`
- `foxglove_bridge`
- `multipoint_navigation`
- `voice_commander`

Each block uses the ROS 2 standard structure:

```yaml
node_name:
  ros__parameters:
    param_name: value
```

## How to use it

1. Edit values in `src/jetracer_bringup/config/main_config.yaml`.
2. Launch normally (the bringup launch files now default to this config).
3. Restart the relevant launch or node to apply changes.

## Launch override

All top-level bringup launch files now accept:

- `config_file:=<path-to-yaml>`

Example:

```bash
ros2 launch jetracer_bringup autonomy.launch.py \
  config_file:=/path/to/custom_main_config.yaml
```

## Navigation note

`jetracer_navigation` also ships `config/multipoint_nav.yaml` as a standalone default for direct navigation launches. When launched through `jetracer_bringup`, `main_config.yaml` is passed through so multipoint parameters still come from the centralized file.
