# jetracer_perception

ROS 2 camera bringup package for JetRacer.

## Migrated content

- ROS 1 `csi_camera.launch` migrated to `launch/csi_camera.launch.py`
- camera calibration file migrated to `config/cam_640x480.yaml`

## Notes

- This launch file assumes the ROS 2 `gscam` package is installed on the Jetson target.
- The default frame is `camera_link`, which is defined in `jetracer_description`.
- The default pipeline is tailored for Jetson CSI cameras using `nvarguscamerasrc`.
