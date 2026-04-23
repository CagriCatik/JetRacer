# 12. Camera Calibration

The Waveshare JetRacer ROS kit uses an **IMX219-160** wide-angle CSI camera. That lens is wide enough that lane following, AprilTag pose estimation, and image overlays become geometry-sensitive. The ROS 2 lane follower in this workspace now consumes `CameraInfo` and rectifies frames before applying the bird's-eye-view warp, so calibration directly affects runtime behavior.

## Prerequisites

- Install the ROS 2 calibration package on the Jetson target if it is not already present:

```bash
sudo apt install ros-humble-camera-calibration
```

- Use a printed checkerboard with a known square size.
- Know the **inner-corner count** of the checkerboard, not the number of black/white squares.

## Recommended Launch

The repository now provides a top-level ROS 2 launch entry point for calibration:

```bash
ros2 launch jetracer_bringup camera_calibration.launch.py \
  board_size:=5x7 \
  square_size_m:=0.03
```

Arguments:

- `board_size`: checkerboard inner-corner count as `cols x rows`
- `square_size_m`: one checker square edge length in meters
- `image_width`: CSI capture width, default `640`
- `image_height`: CSI capture height, default `480`
- `image_fps`: CSI frame rate, default `20`
- `flip_method`: Jetson `nvvidconv` flip mode, default `0`

If your checkerboard is different, override the launch values. Example for an 8x6 board with 24 mm squares:

```bash
ros2 launch jetracer_bringup camera_calibration.launch.py \
  board_size:=8x6 \
  square_size_m:=0.024
```

## What The Launch Starts

The calibration launch does two things:

1. Starts the CSI camera pipeline with `gscam` on `/csi_cam_0/image_raw` and `/csi_cam_0/camera_info`
2. Starts `camera_calibration/cameracalibrator` against the matching ROS 2 topics and `set_camera_info` service

This is the ROS 2 equivalent of the old ROS 1 manual flow, but wrapped in a single launch file.

## Calibration Procedure

1. Keep the JetRacer stationary and well lit.
2. Hold the checkerboard flat, then move it through the center, left/right edges, upper/lower edges, and all four corners of the image.
3. Vary distance and yaw/pitch so the GUI coverage bars for position, size, and skew fill out.
4. Wait for the `CALIBRATE` button to turn active.
5. Click `CALIBRATE`.
6. Inspect the reprojection result.
7. Click `COMMIT`.

`COMMIT` writes the new intrinsics through the camera driver's `set_camera_info` service. The file that gets updated is whatever `camera_info_url` points at. By default this repository uses `jetracer_perception/config/cam_640x480.yaml`.

## Verify The Result

Restart the camera node or the lane-following stack after calibration.

Check that the camera info is populated:

```bash
ros2 topic echo /csi_cam_0/camera_info --once
```

Launch the lane follower in safe mode:

```bash
ros2 launch jetracer_bringup lane_following.launch.py start:=false
```

Then verify:

- `/lane_following/debug_image/compressed` shows `cal: RECTIFIED`
- the lane overlay is stable near the image edges
- AprilTag and other image-space consumers are using the updated `/csi_cam_0/camera_info`

## Troubleshooting

- If the calibrator window never appears, confirm the target has a graphical session or X11 forwarding.
- If `CALIBRATE` never enables, the checkerboard size is probably wrong.
- If `COMMIT` fails, confirm `/csi_cam_0/set_camera_info` exists.
- If the updated YAML does not persist into runtime, rebuild or relaunch from the same workspace install that the launch files resolve via `FindPackageShare`.
- If the image is mirrored or upside down, adjust `flip_method` in the camera launch before calibrating.
