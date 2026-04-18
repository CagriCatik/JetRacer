# jetracer_bringup

Top-level ROS 2 launch package for the migrated JetRacer stack.

## Entry points

- `visualize.launch.py`: robot model only
- `jetracer.launch.py`: description, hardware bridge, and EKF localization
- `lidar.launch.py`: LiDAR-only bringup
- `laser_filter.launch.py`: LiDAR plus migrated scan filter
- `csi_camera.launch.py`: CSI camera bringup
- `calibrate_linear.launch.py`: hardware plus odometry calibration tool
- `nav.launch.py`: full navigation stack against a saved map
- `slam.launch.py`: mapping-only bringup
- `slam_nav.launch.py`: mapping plus navigation
- `tts.launch.py`, `asr.launch.py`, `talk.launch.py`: migrated voice entry points

## Run

```bash
ros2 launch jetracer_bringup jetracer.launch.py
```

## Lane-Following Controller Option

`jetracer_bringup/lane_following.launch.py` exposes:

- `lateral_controller_type` (`stanley` or `mpc`)

Example:

```bash
ros2 launch jetracer_bringup lane_following.launch.py \
  lateral_controller_type:=mpc
```

## RViz settings

All major bringup launches now expose ROS 2 launch arguments for RViz:

- `use_rviz` (`true`/`false`)
- `rviz_profile` (`description`, `navigation`, `slam`, `autonomy`)
- `rviz_config` (absolute `.rviz` path override)
- `rviz_fixed_frame` (optional fixed-frame override)
- `use_sim_time` (passed to RViz)

Example:

```bash
ros2 launch jetracer_bringup nav.launch.py \
  use_rviz:=true \
  rviz_profile:=navigation \
  rviz_fixed_frame:=map
```
