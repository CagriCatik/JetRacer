# 11. Operational Guide

This guide provides the primary instructions for daily operation of the JetRacer autonomous platform, including startup, mission control, and safety procedures.

## Quick Startup (Sentinel Mode)

The robot is configured to boot automatically into **Sentinel Mode**. In this state, it is "Ready but Idle"—waiting for your controller to trigger a mission.

1.  **Power On**: Connect the batteries and flip the Jetson Nano power switch.
2.  **Wait for Ready**: Observe the onboard OLED display. When it shows **"MISSION: IDLE"**, the robot is ready.
3.  **Launch**: Use the Gamepad to start a mission (see controls below).

## Mission Controls (Gamepad)

The standard Waveshare Gamepad matches the following layout for mission management:

| Action | Combination | Result |
| :--- | :--- | :--- |
| **Launch Autonomy** | `SELECT` + `START` | Starts Full Autonomy (LiDAR + Camera + YOLO) |
| **Launch Mapping** | `SELECT` + `X` | Starts SLAM and Navigation |
| **Emergency Halt** | `MODE` (Center) | Kills all active missions and stops all motors |

## Teleoperation (Manual Override)

Manual driving is **always active** as a background safety layer. You can override any autonomous mission at any time by simply using the gamepad.

- **Trigger**: Hold the **Deadman Switch (L2)**. 
- **Priority**: High. Pressing L2 will immediately "mute" the Autonomy/SLAM AI and give you direct control of the wheels.
- **Controls (RC Style)**:
    - **Right Stick Vertical**: Forward / Backward Speed
    - **Left Stick Horizontal**: Steering Angle

> [!WARNING]
> **Safety Gating**: If you release the L2 button while in manual mode, the robot will immediately stop, even if an autonomous mission is running.

## Diagnostic Feedback (OLED)

The onboard display provides real-time telemetry to prevent "blind" operation:

- **Mission**: Shows the currently active ROS 2 launch.
- **IP**: Used for SSH or Foxglove connection.
- **BAT**: Total battery voltage. **Halt immediately if < 9.6V.**
- **CPU**: Nano temperature. **Throttling starts at 82°C.**

## Autostart Management

The robot uses a `systemd` service to manage boot logic.

- **Check Service Status**: `sudo systemctl status jetracer.service`
- **View Live Logs**: `journalctl -u jetracer.service -f`
- **Manual Start/Stop**: `sudo systemctl [start|stop|restart] jetracer.service`

---
> [!TIP]
> **Pro-Tip**: If the LiDAR is spinning but the OLED says "IDLE", check the `/sentinel/status` topic to ensure the relay node is communicating properly.
