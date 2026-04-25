# Runtime Parameter Tuning Guide

The JetRacer ROS 2 stack is designed with **Dynamic Configuration** capability. This allows you to tune everything from PID gains to safety distances while the vehicle is live, without restarting the autonomy engine.

---

## Tuning Tools

Depending on your environment, you can use any of the following tools to adjust the `/ros__parameters` of our nodes.

### 1. Foxglove Studio (Remote Web-based)

**Recommended for the Jetson Nano** to minimize CPU overhead on the main machine.

1. Connect Foxglove to the JetRacer (`ws://<IP>:8765`).
2. Add the **Parameters** panel from the sidebar.
3. Select a node (e.g., `/lane_following` or `/jetracer_hardware`).
4. Adjust values directly; the node updates instantly.

### 2. ROS 2 CLI (Expert/Command Line)

Useful for rapid scripting or when a GUI is not available.

```bash
# Get the current value of a parameter
ros2 param get /lane_following lateral_controller_type

# Set a new value (e.g., switch to MPC)
ros2 param set /lane_following lateral_controller_type "mpc"

# Change speed limits mid-run
ros2 param set /lane_following max_speed_ms 0.5
```

### 3. RQT Reconfigure (Standard GUI)

Use this if you are running on a local desktop with X11 forwarding or if the Jetson is connected to a monitor.

```bash
ros2 run rqt_reconfigure rqt_reconfigure
```

---

## Tunable Parameter Reference

### 1. Lane Following (`/lane_following`)

| Parameter | Default | Tuning Strategy |
| :--- | :--- | :--- |
| `start` | `false` | Set to `true` to begin motion. |
| `max_speed_ms` | `0.35` | Increase for performance, decrease for indoor safety. |
| `kp`, `ki`, `kd` | Variable | Longitudinal PID tuning for smooth acceleration. |
| `lateral_controller_type` | `stanley` | Toggle between `stanley` and `mpc`. |

### 2. Hardware Interface (`/jetracer_hardware`)

| Parameter | Default | Tuning Strategy |
| :--- | :--- | :--- |
| `kp`, `ki` | `350`, `120` | Tune motor responsiveness at the hardware level. |
| `servo_bias` | `0` | Use this to offset mechanical steering misalignment. |
| `command_timeout_sec` | `1.0` | Adjust the "Heart-Stop" window for loss-of-signal. |

### 3. Collision Assurance (`/collision_assurance`)

| Parameter | Default | Tuning Strategy |
| :--- | :--- | :--- |
| `min_distance_m` | `0.45` | Increase if the car has a long stopping distance. |
| `cone_angle_deg` | `30.0` | Narrow or widen the LiDAR protection "frustum". |

---

> [!IMPORTANT]
> **Safety Overrides:** While most parameters are dynamic, **hardware connection settings** (port name, baud rate) and **ML model paths** (YOLO/Vosk) are **Read-Only** after the node starts. Attempting to change these at runtime will be rejected by the node's validation logic to prevent memory crashes or hardware locks.

---

## Tuning Workflow

1. **Bringup:** Launch the `autonomy.launch.py`.
2. **Monitor:** Open Foxglove and watch the `/diagnostics` and `/lane_following/debug_image`.
3. **Adjust:** Change `max_speed_ms` or `kp` until the lane tracking is stable.
4. **Persist:** Once you find the "Golden Settings," copy them back into `src/jetracer_bringup/config/main_config.yaml` to make them permanent.
