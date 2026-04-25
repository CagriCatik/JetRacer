# 16. Preflight Checklist

Use this checklist before any physical autonomous run. Keep the robot on a stand with the wheels off the ground until the dry-run and command-arbitration checks pass.

## 0. Safety State

- Wheels off ground.
- Battery disconnect or power switch reachable.
- Gamepad connected and deadman behavior verified.
- Area around the robot clear.
- Simulation uses a different `ROS_DOMAIN_ID` than the physical robot.

```bash
export ROS_DOMAIN_ID=0
printenv ROS_DOMAIN_ID
```

If a Gazebo simulation is running on the same network, stop it or move it to another domain:

```bash
export ROS_DOMAIN_ID=42
ros2 daemon stop
ros2 daemon start
```

## 1. Host And Container

On the Jetson host:

```bash
cd ~/JetRacer
git status --short
docker compose up -d --build
docker exec -it jetracer_workspace bash
```

Inside the container:

```bash
cd /workspaces/JetRacer-ROS2
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Run tests when time allows:

```bash
colcon test
colcon test-result --verbose
```

Fast Python-only checks:

```bash
python3 -m compileall -q src
python3 -m pytest -q src/jetracer_lane_following/test src/jetracer_navigation/test
```

## 2. Linux Hardware Visibility

Check USB serial devices:

```bash
ls -l /dev/ttyACM*
dmesg | grep -Ei 'ttyACM|rp2040|rplidar|usb' | tail -n 80
```

Check gamepad:

```bash
ls -l /dev/input/js*
ros2 run joy joy_node --ros-args -p dev:=/dev/input/js0
ros2 topic echo /joy --once
```

Stop the temporary `joy_node` with `Ctrl-C`.

Check I2C devices:

```bash
i2cdetect -y 1
```

Check camera availability:

```bash
v4l2-ctl --list-devices
gst-inspect-1.0 nvarguscamerasrc
```

Check Jetson thermal state:

```bash
for z in /sys/class/thermal/thermal_zone*; do
  printf "%s " "$(cat "$z/type" 2>/dev/null)"
  awk '{printf "%.1f C\n", $1/1000}' "$z/temp" 2>/dev/null
done
```

## 3. Dry-Run Bringup

Start the stack with no motor serial output and no camera/LiDAR dependency:

```bash
ros2 launch jetracer_bringup autonomy.launch.py \
  dry_run:=true \
  start_camera:=false \
  start_lidar:=false \
  start_yolo:=false \
  use_rviz:=false
```

In another shell:

```bash
source /opt/ros/foxy/setup.bash
source /workspaces/JetRacer-ROS2/install/setup.bash
ros2 node list
ros2 topic list
ros2 topic info /cmd_vel
ros2 topic info /cmd_vel_safety
ros2 topic info /cmd_vel_lane
```

Expected nodes include:

```bash
ros2 node info /jetracer_hardware
ros2 node info /twist_mux
ros2 node info /safety_supervisor
ros2 node info /lane_following
```

Stop the dry-run launch with `Ctrl-C`.

## 4. Base Hardware Bringup

Start base hardware only:

```bash
ros2 launch jetracer_bringup jetracer.launch.py use_rviz:=false
```

Verify hardware telemetry:

```bash
ros2 topic echo /battery_state --once
ros2 topic echo /imu --once
ros2 topic echo /odom_raw --once
ros2 topic echo /diagnostics --once
ros2 topic hz /imu
ros2 topic hz /odom_raw
```

Verify TF:

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_link laser_link
ros2 run tf2_ros tf2_echo base_link camera_link
```

Check EKF output:

```bash
ros2 topic echo /odom --once
ros2 topic hz /odom
```

## 5. Command Arbitration

With wheels still off ground, verify the mux output remains controllable:

```bash
ros2 topic echo /cmd_vel
```

Publish a low-speed command briefly:

```bash
ros2 topic pub --once /cmd_vel_teleop geometry_msgs/msg/Twist \
"{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

Publish safety stop:

```bash
ros2 topic pub --once /cmd_vel_safety geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

Confirm command timeout stops output after publishers go silent:

```bash
ros2 topic echo /cmd_vel --once
sleep 2
ros2 topic echo /cmd_vel --once
```

## 6. Camera Pipeline

Start camera only:

```bash
ros2 launch jetracer_perception csi_camera.launch.py
```

Verify topics and frame rate:

```bash
ros2 topic list | grep csi_cam_0
ros2 topic echo /csi_cam_0/camera_info --once
ros2 topic hz /csi_cam_0/image_raw
ros2 run image_transport republish raw compressed \
  --ros-args -r in:=/csi_cam_0/image_raw -r out:=/csi_cam_0/image_raw
ros2 topic hz /csi_cam_0/image_raw/compressed
```

Stop the temporary `republish` command with `Ctrl-C` if the full stack is not running.

## 7. LiDAR Pipeline

Start LiDAR:

```bash
ros2 launch jetracer_hardware lidar.launch.py
```

Verify scan output:

```bash
ros2 topic echo /scan --once
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo base_footprint laser_link
```

Optional filtered scan:

```bash
ros2 launch jetracer_hardware laser_filter.launch.py
ros2 topic echo /filteredscan --once
```

## 8. Perception And Lane Following

Start full autonomy in safe mode, with motors disabled first:

```bash
ros2 launch jetracer_bringup autonomy.launch.py \
  dry_run:=true \
  safe_mode:=true \
  start_yolo:=false \
  use_rviz:=false
```

Check lane follower status:

```bash
ros2 param get /lane_following start
ros2 param get /lane_following max_speed_ms
ros2 topic echo /lane_following/status --once
ros2 topic echo /lane_following/confidence --once
ros2 topic hz /lane_following/control_fps
ros2 topic echo /cmd_vel_lane --once
```

Enable lane following only while the robot is still on a stand:

```bash
ros2 param set /lane_following start true
ros2 topic echo /cmd_vel_lane
```

Disable before putting the robot down:

```bash
ros2 param set /lane_following start false
ros2 topic pub --once /cmd_vel_safety geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

## 9. YOLO Check

Start YOLO only after camera frame rate is stable:

```bash
ros2 launch jetracer_perception yolo_detection.launch.py
```

Verify detections and inference rate:

```bash
ros2 topic echo /perception/yolo_detections --once
ros2 topic hz /perception/yolo_detections
ros2 topic echo /perception/yolo_fps --once
```

If TensorRT fails, switch to PyTorch weights:

```bash
ros2 param set /yolo_detection model_path yolo11n-best.pt
```

If the node rejects model swapping, stop the node and relaunch with the `.pt` model in the parameter file.

## 10. Final Physical Run

Place the robot on the floor only after the checks above pass. Start in safe mode:

```bash
ros2 launch jetracer_bringup autonomy.launch.py \
  safe_mode:=true \
  start_yolo:=false \
  use_rviz:=false
```

Confirm safety and telemetry:

```bash
ros2 topic echo /battery_state --once
ros2 topic echo /diagnostics --once
ros2 topic echo /lane_following/status --once
```

Enable motion:

```bash
ros2 param set /lane_following start true
```

Emergency software halt:

```bash
ros2 param set /lane_following start false
ros2 topic pub --once /cmd_vel_safety geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

Stop all launched processes with `Ctrl-C`, or stop the autostart service:

```bash
sudo systemctl stop jetracer.service
journalctl -u jetracer.service -n 120 --no-pager
```

## 11. Pass Criteria

- `/battery_state` voltage is above the configured minimum.
- `/diagnostics` has no `ERROR` entries.
- `/imu`, `/odom_raw`, `/odom`, `/scan`, and camera topics publish at stable rates.
- `tf2_echo` succeeds for `odom -> base_footprint`, `base_link -> laser_link`, and `base_link -> camera_link`.
- `twist_mux` output stops when command sources go silent.
- Teleop deadman overrides autonomous command sources.
- Lane following remains disabled until explicitly armed.
- First physical run uses `safe_mode:=true`.
