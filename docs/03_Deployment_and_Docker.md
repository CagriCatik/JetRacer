# 03. Deployment and Docker

This guide defines the supported Docker workflow for running JetRacer on Jetson Nano.

## Supported Configuration

- Host: Jetson Nano with JetPack 4.6.1 (L4T 32.7.1)
- Container base: `dustynv/ros:humble-ros-base-l4t-r32.7.1`
- ROS distro in container: ROS 2 Humble
- Runtime policy: ROS 2 only (no `ros1_bridge`)

## 1. Host Prerequisites (Jetson)

Ensure Docker is installed and NVIDIA container runtime support is available (default on JetPack).

```bash
sudo usermod -aG docker $USER
```

Log out and log back in after running the command above.

## 2. Start the Container

From the repository root:

```bash
cd JetRacer-ROS2
docker compose up -d --build
```

What this gives you:

- The project image is built from the local `Dockerfile`
- Source code is bind-mounted into `/workspaces/JetRacer-ROS2`
- Container runs with host networking and device access for robot hardware

## 3. Build the ROS 2 Workspace

Open a shell inside the running container:

```bash
docker exec -it jetracer_workspace bash
```

Then run:

```bash
source ~/.bashrc
rosdep update
rosdep install --from-paths src --ignore-src -r -y
build_workspace
source install/setup.bash
```

## 4. Run Core Modes

Use separate terminals (each with `docker exec -it jetracer_workspace bash`).

Hardware bringup (must run first):

```bash
ros2 launch jetracer_bringup jetracer.launch.py
```

Full lane-following stack (camera + control):

```bash
ros2 launch jetracer_bringup lane_following.launch.py
```

Enable autonomous lane command output:

```bash
ros2 param set /lane_following start true
```

Disable lane command output:

```bash
ros2 param set /lane_following start false
```

## 5. Foxglove Telemetry

Start autonomy pipeline in the container:

```bash
ros2 launch jetracer_bringup autonomy.launch.py
```

Then from a browser on your PC/tablet:

1. Open [studio.foxglove.dev](https://studio.foxglove.dev/)
2. Select **Foxglove WebSocket**
3. Connect to `ws://<JETSON_IP_ADDRESS>:8765`

## 6. Useful Operations

Stop services:

```bash
docker compose down
```

Rebuild after Dockerfile or dependency changes:

```bash
docker compose up -d --build
```

View container logs:

```bash
docker logs -f jetracer_workspace
```

---

Next: [04. Perception Stack](04_Perception_Stack.md)
