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
