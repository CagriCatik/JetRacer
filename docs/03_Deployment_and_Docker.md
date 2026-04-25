# 3. Deployment and Docker

This guide is the canonical runtime workflow for JetRacer on Jetson Nano.

## Supported Configuration

- Hardware: Jetson Nano
- Host OS baseline: Ubuntu 20.04 workaround image on JetPack 4.6.1 (L4T 32.7.1)
- Container base: `dustynv/ros:humble-ros-base-l4t-r32.7.1`
- ROS distro in container: ROS 2 Humble
- Runtime policy: ROS 2 only (no `ros1_bridge`)
- Native `setup/install_ros2.sh` on Ubuntu 20.04 host: not supported

Before continuing, complete the host baseline in [00_ROS2-Jetson-Nano.md](00_ROS2-Jetson-Nano.md).

## 1. Host Prerequisites (Jetson)

Ensure Docker and NVIDIA container runtime are available:

```bash
docker --version
docker compose version
docker info | grep -i Runtime
```

If Docker is missing, install it from this repository:

```bash
cd setup
./install_docker.sh
```

Ensure your user can run Docker without sudo:

```bash
sudo usermod -aG docker $USER
```

Log out and log back in after changing group membership.

## 2. Start the Container

From repository root:

```bash
docker compose up -d --build
```

What this gives you:

- Image built from local `Dockerfile`
- Source bind-mounted into `/workspaces/JetRacer-ROS2`
- Host networking and device passthrough for robot hardware

## 3. Build the ROS 2 Workspace

Open a shell in the running container:

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
