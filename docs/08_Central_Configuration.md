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

## Lane-Following Controller Selection

The lane-following node now supports two lateral controllers selected by:

- `lane_following.ros__parameters.lateral_controller_type`

Valid values:

- `stanley`
- `mpc`

When `stanley` is selected, tuning uses:

- `gain_constant`: Proportional steering gain (higher = more aggressive turn).
- `steering_smoothing`: First-order output smoothing in `[0, 1]`.
- `controller_lookahead_m`: Forward lookahead distance in the BEV vehicle frame.
- `controller_heading_points`: Number of local waypoints used to estimate path heading.

### Detection & Geometry Tuning

The BEV pipeline uses these critical parameters to filter the track environment:

- `luminance_threshold`: Drops dark pixels (0-255). Set to `180.0` for white tape on black ground.
- `roi_top_y`: The horizon cut depth.
- `roi_top_width`: The width of the road trapezoid at the horizon.
- `camera_info_topic`: `CameraInfo` topic used to rectify frames before BEV projection.
- `use_camera_calibration`: Enable or bypass runtime undistortion.
- `fallback_lane_width_px`: Prior lane width used for one-sided fallback.
- `lane_width_ema_alpha`: Smoothing factor for the tracked lane-width estimate.
- `lane_width_min_px` / `lane_width_max_px`: Plausibility bounds for BEV lane width.
- `bev_vehicle_center_x_px`: Pixel column treated as the vehicle center in BEV.
- `bev_vehicle_origin_y_px`: Pixel row treated as the vehicle origin in BEV.
- `bev_lateral_m_per_px`: Lateral BEV scale in meters per pixel.
- `bev_forward_m_per_px`: Forward BEV scale in meters per pixel.
- `speed_curvature_gain`: Curvature-to-speed falloff gain.
- `speed_confidence_floor`: Minimum speed scaling applied under weak tracking.

When `mpc` is selected, tuning uses:

- `mpc_horizon`
- `mpc_dt`
- `mpc_wheelbase`
- `mpc_q_cte`
- `mpc_q_heading`
- `mpc_q_terminal`
- `mpc_r_steer`: Steering action cost.
- `mpc_r_steer_rate`: Change-in-steering cost.
- `mpc_min_speed_ms`: Floor speed for matrix stability.
- `max_steering_rate_radps`: Hard per-step steering-rate clamp applied after the solve.

Command interface outputs:

- `drive_lane`: primary `ackermann_msgs/AckermannDriveStamped` command.
- `cmd_vel_lane`: legacy steering-angle `Twist` kept for the existing `twist_mux` path.

Example:

```yaml
lane_following:
  ros__parameters:
    lateral_controller_type: "mpc"
    mpc_horizon: 8
    mpc_dt: 0.1
    mpc_q_cte: 2.5
    mpc_q_heading: 1.5
    controller_lookahead_m: 0.35
    bev_forward_m_per_px: 0.0030
```

## How to use it

1. Edit values in `src/jetracer_bringup/config/main_config.yaml`.
2. Launch normally (bringup launch files default to this config).
3. Restart the relevant launch or node to apply changes.

## Launch override

All top-level bringup launch files accept:

- `config_file:=<path-to-yaml>`

Example:

```bash
ros2 launch jetracer_bringup autonomy.launch.py \
  config_file:=/path/to/custom_main_config.yaml
```

`jetracer_bringup/lane_following.launch.py` also accepts a direct controller override:

```bash
ros2 launch jetracer_bringup lane_following.launch.py \
  lateral_controller_type:=mpc
```

## Navigation note

`jetracer_navigation` also ships `config/multipoint_nav.yaml` as a standalone default for direct navigation launches. When launched through `jetracer_bringup`, `main_config.yaml` is passed through so multipoint parameters still come from the centralized file.
