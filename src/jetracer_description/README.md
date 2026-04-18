# jetracer_description

This package owns the robot model and frame conventions.

## Included now

- `urdf/jetracer.urdf.xacro`: baseline JetRacer robot model
- `launch/view_description.launch.py`: RViz visualization entry point
- `rviz/jetracer.rviz`: default visualization profile

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