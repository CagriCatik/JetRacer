# ROS 2 Workspace Source Directory

This directory now contains the ROS 2 migration target for the original `jetracer_ros-main` ROS 1 package.

## Packages

- `jetracer_bringup`: top-level launch entry points that replace the ROS 1 XML launches
- `jetracer_description`: robot description, TF frames, RViz config, and sensor mounting geometry
- `jetracer_hardware`: serial bridge, LiDAR launch, scan filtering, and linear calibration
- `jetracer_localization`: `robot_localization` EKF bringup and odometry compatibility tools
- `jetracer_perception`: CSI camera bringup and camera calibration
- `jetracer_navigation`: Nav2, mapping, multipoint patrol, steering adapter, and map tools
- `jetracer_voice`: ROS 2 speech/TTS stack migrated from the ROS 1 scripts
- `jetracer_teleop`: reserved for teleoperation interfaces

## Key launch entry points

```bash
ros2 launch jetracer_bringup jetracer.launch.py
ros2 launch jetracer_bringup nav.launch.py
ros2 launch jetracer_bringup slam.launch.py
ros2 launch jetracer_bringup talk.launch.py
```

## Migration notes

- The ROS 1 `move_base` stack was replaced with Nav2.
- A steering adapter now converts Nav2 yaw-rate commands into the steering-angle convention used by the JetRacer MCU.
- Voice scripts no longer write into the repository tree at runtime.
- `jetracer_ros-main` is kept as a source reference and should not be built as part of the ROS 2 workspace.

## Example build flow

```bash
source /opt/ros/<distro>/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```
