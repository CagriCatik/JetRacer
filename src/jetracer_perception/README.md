# jetracer_perception

ROS 2 camera bringup package for JetRacer.

## Migrated content

- ROS 1 `csi_camera.launch` migrated to `launch/csi_camera.launch.py`
- camera calibration file migrated to `config/cam_640x480.yaml`
- ROS 2 camera calibration helper added at `launch/camera_calibration.launch.py`

## Notes

- This launch file assumes the ROS 2 `gscam` package is installed on the Jetson target.
- The calibration launch assumes the ROS 2 `camera_calibration` package is installed.
- The default frame is `camera_link`, which is defined in `jetracer_description`.
- The default pipeline is tailored for Jetson CSI cameras using `nvarguscamerasrc`.

## Calibrate the CSI camera

```bash
ros2 launch jetracer_perception camera_calibration.launch.py \
  board_size:=5x7 \
  square_size_m:=0.03
```

Override `board_size` and `square_size_m` to match your physical checkerboard target.
