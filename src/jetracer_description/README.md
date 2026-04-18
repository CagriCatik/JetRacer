# jetracer_description

This package owns the robot model and frame conventions.

## Included now

- `urdf/jetracer.urdf.xacro`: baseline JetRacer robot model
- `launch/view_description.launch.py`: RViz visualization entry point
- `launch/rviz.launch.py`: reusable, profile-based RViz launcher
- `rviz/jetracer.rviz`: default visualization profile
- `rviz/navigation.rviz`, `rviz/slam.rviz`, `rviz/autonomy.rviz`: profile presets

## Frame intent

- `base_footprint`: ground-projected reference frame
- `base_link`: chassis-centered body frame
- `camera_link`: forward-facing CSI camera mount
- `imu_link`: body-mounted IMU frame

The wheel and steering joints are exposed so the description can be exercised with `joint_state_publisher_gui` during bring-up.

## Run

```bash
ros2 launch jetracer_description view_description.launch.py
```

## Configurable RViz (ROS 2 conform)

The RViz launcher supports profile and explicit config overrides:

```bash
ros2 launch jetracer_description rviz.launch.py \
  use_rviz:=true \
  rviz_profile:=navigation \
  rviz_fixed_frame:=map
```

Or provide your own `.rviz` file directly:

```bash
ros2 launch jetracer_description rviz.launch.py \
  rviz_config:=/absolute/path/custom.rviz
```
