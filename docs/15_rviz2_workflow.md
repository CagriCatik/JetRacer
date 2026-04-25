# RViz2 Workflow Guide — JetRacer Autonomous Stack

## Overview

RViz2 is **never started on the Jetson Nano by default**.  
The Jetson Nano has limited RAM (~4 GB shared) and GPU memory; running RViz2 
locally during autonomous driving would compete with YOLO inference and lane 
detection for GPU/CPU resources.

The recommended workflow is:

```
Jetson Nano  ──ROS 2 DDS──▶  Laptop (Ubuntu / WSL2)
(runs stack)                  (runs rviz2 remotely)
```

---

## Quick Start

### On the Jetson Nano

```bash
# 1. Source workspace
source /opt/ros/foxy/setup.bash
source ~/ros2_ws/install/setup.bash

# 2. Run the health check
bash src/jetracer_bringup/scripts/health_check.sh

# 3. Launch the stack (headless, no RViz)
ros2 launch jetracer_bringup jetracer.launch.py use_rviz:=false

# 4. In a second terminal: start autonomy
ros2 launch jetracer_bringup autonomy.launch.py
```

### On the Laptop (remote RViz2)

```bash
# 1. Source ROS 2 (same distro as Jetson)
source /opt/ros/foxy/setup.bash

# 2. Set ROS_DOMAIN_ID to match the Jetson
export ROS_DOMAIN_ID=0        # must match value on Jetson

# 3. Verify the Jetson is reachable
ros2 node list                # should show /jetracer_hardware, /lane_following, etc.
ros2 topic list               # should show /cmd_vel, /scan, /odom, /lane_following/*

# 4. Launch RViz2 with the autonomy profile
rviz2 -d $(ros2 pkg prefix jetracer_description)/share/jetracer_description/rviz/autonomy.rviz

# OR via launch file:
ros2 launch jetracer_description rviz.launch.py use_rviz:=true rviz_profile:=autonomy
```

---

## ROS 2 Network Setup

### Requirements

| Setting | Value |
|---|---|
| ROS 2 distro | Same on Jetson and laptop (e.g. `foxy`) |
| `ROS_DOMAIN_ID` | Same integer on both machines (default: `0`) |
| Network | Same LAN subnet, or direct Ethernet link |
| Multicast | Enabled on the network switch (most home routers: yes) |
| Firewall | UDP ports unrestricted between the two hosts |

### Verify DDS Discovery

```bash
# On laptop — should list Jetson nodes within 2–3 seconds:
ros2 node list

# If empty, try forcing the Jetson IP:
export ROS_DISCOVERY_SERVER=192.168.1.XXX:11811   # only if using Connext
# OR for CycloneDDS, add a cyclone_dds.xml with peer IP
```

### CycloneDDS Unicast (when multicast fails)

If your switch blocks multicast (common on managed switches), force unicast:

```xml
<!-- ~/cyclone_dds.xml on BOTH machines -->
<CycloneDDS>
  <Domain>
    <Discovery>
      <Peers>
        <Peer address="192.168.1.XXX"/>  <!-- Jetson IP -->
        <Peer address="192.168.1.YYY"/>  <!-- Laptop IP -->
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
```

```bash
export CYCLONEDDS_URI=file://$HOME/cyclone_dds.xml  # on both machines
```

---

## Available RViz2 Profiles

| Profile | Config File | Best For |
|---|---|---|
| `autonomy` | `autonomy.rviz` | Lane following debug (default) |
| `lane_following` | `autonomy.rviz` | Alias for autonomy |
| `description` | `jetracer.rviz` | Robot model + TF only |
| `navigation` | `navigation.rviz` | Nav2 + SLAM map |
| `slam` | `slam.rviz` | SLAM building |

```bash
# Switch profiles:
ros2 launch jetracer_description rviz.launch.py use_rviz:=true rviz_profile:=navigation
```

---

## Topics Visible in autonomy.rviz

| Display | Topic | Type | Default |
|---|---|---|---|
| RobotModel | `/robot_description` | `std_msgs/String` | ✅ On |
| TF | — | TF2 tree | ✅ On |
| LiDAR Scan | `/scan` | `sensor_msgs/LaserScan` | ✅ On |
| Odometry | `/odom` | `nav_msgs/Odometry` | ✅ On |
| Lane Centerline | `/lane_following/path` | `nav_msgs/Path` | ✅ On |
| Waypoints | `/lane_following/path_viz` | `geometry_msgs/PoseArray` | ✅ On |
| Lane Markers | `/lane_following/markers` | `visualization_msgs/MarkerArray` | ✅ On |
| Lane Debug Image | `/lane_following/debug_image/compressed` | `sensor_msgs/CompressedImage` | ❌ Off |
| YOLO Image | `/perception/yolo_debug/compressed` | `sensor_msgs/CompressedImage` | ❌ Off |

> **Image displays are disabled by default** — enabling them adds ~2–8 Mbps 
> of network traffic per topic. Enable only on a wired or fast Wi-Fi connection.

---

## TF Frame Tree

```
odom  ─────────────────────────────────── (EKF → published by ekf_filter_node)
  └─ base_footprint
       └─ base_link
            ├─ camera_link               x=0.115m forward, z=0.045m
            │    └─ camera_link_optical  (rotated −π/2 around Z, −π/2 around X)
            ├─ imu_link                  z=0.020m
            ├─ laser_link               z=0.100m, yaw=π (upside-down RPLidar)
            ├─ front_left_steering_link
            ├─ front_right_steering_link
            └─ jetson_link
```

### Verify TF Tree

```bash
# Quick check:
ros2 run tf2_tools view_frames
# Generates /tmp/frames.pdf showing the complete tree

# Check specific transform:
ros2 run tf2_ros tf2_echo base_link camera_link
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo base_link laser_link
```

---

## Lane Following Markers (lane_following/markers)

Three markers are published per control cycle when `enable_markers: true`:

| ID | Type | Meaning |
|---|---|---|
| 0 | `SPHERE` (gold) | Lookahead / target waypoint |
| 1 | `ARROW` (blue) | Steering command direction (scaled by speed) |
| 2 | `CUBE` (color) | Tracking state: 🟢 GOOD / 🟡 WEAK / 🔴 LOST |

Markers have `lifetime.sec = 1` — they disappear automatically if the node stops.

### Disable Markers on Jetson (save CPU)

```bash
ros2 param set /lane_following enable_markers false
```

Or set in `main_config.yaml`:
```yaml
lane_following:
  ros__parameters:
    enable_markers: false     # Disable on Jetson, enable on remote laptop
```

---

## Viewing Image Topics Remotely

Image topics are compressed (`sensor_msgs/CompressedImage`) and already bandwidth-optimised. To view them without RViz2:

```bash
# rqt_image_view (lightweight):
ros2 run rqt_image_view rqt_image_view /lane_following/debug_image/compressed

# Raw stats:
ros2 topic hz /lane_following/debug_image/compressed    # should be ≤5 Hz
ros2 topic hz /csi_cam_0/image_raw/compressed           # should be ~20 Hz
```

---

## Validation Commands

```bash
# List all topics
ros2 topic list

# Check lane following topics
ros2 topic info /lane_following/path
ros2 topic info /lane_following/markers
ros2 topic echo /lane_following/status --once
ros2 topic hz /lane_following/control_fps

# Check sensor topics
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic hz /csi_cam_0/image_raw/compressed

# TF checks
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo base_link camera_link
ros2 run tf2_ros tf2_echo odom base_footprint

# Launch RViz2 manually
rviz2 -d $(ros2 pkg prefix jetracer_description)/share/jetracer_description/rviz/autonomy.rviz
```

---

## Remaining Limitations

| Limitation | Reason | Workaround |
|---|---|---|
| RViz2 cannot run on Jetson Nano during autonomous driving | RAM/GPU budget | Use remote laptop workflow |
| `camera_info` topic uses `camera_link` frame (not optical) | gscam publishes with `frame_id` from launch arg | Set `frame_id: camera_link` — already done |
| `lane_following/debug_image/compressed` is 320×240 | Processed in 320×240 space | Acceptable; upscale in rqt_image_view if needed |
| No 3D lane boundary visualization | Lane detector works in 2D BEV | `nav_msgs/Path` + `MarkerArray` approximate this |
| Steering joint state not animated in robot model | No joint_state_publisher running (correct for HW) | Accept static model — avoids zero-flooding `/joint_states` |
