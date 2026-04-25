# JetRacer ROS 2 Foxy Docker Setup

This Dockerfile creates a ROS 2 Foxy development environment for the JetRacer ROS 2 workspace.

It is based on:

- Ubuntu 20.04
- ROS 2 Foxy
- Python 3
- Colcon
- rosdep
- JetRacer-related ROS 2 packages
- Navigation, SLAM, camera, joystick, and robot-state tools

The container workspace path is:

```bash
/workspaces/JetRacer-ROS2
````

---

## 1. Requirements

Install Docker on the host machine before using this setup.

Check that Docker is available:

```bash
docker --version
```

Optional but recommended on Linux:

```bash
sudo usermod -aG docker "$USER"
```

After running that command, log out and log back in.

---

## 2. Project structure

Place the Dockerfile in the root of the JetRacer ROS 2 project:

```text
JetRacer-ROS2/
├── Dockerfile
├── requirements.txt
├── src/
│   └── ...
└── README.md
```

The `requirements.txt` file is expected by the Dockerfile. If the project does not need extra Python packages, create an empty file:

```bash
touch requirements.txt
```

---

## 3. Build the Docker image

From the project root directory, run:

```bash
docker build -t jetracer-ros2-foxy .
```

This creates a Docker image named:

```text
jetracer-ros2-foxy
```

The build installs:

* ROS 2 Foxy base
* ROS camera and image packages
* Nav2 bringup
* SLAM Toolbox
* RViz2
* robot localization
* joystick support
* RPLidar ROS driver
* GSCam
* Python dependencies from `requirements.txt`

---

## 4. Run the container

For JetRacer hardware access, run the container with device and network access:

```bash
docker run -it --rm \
  --privileged \
  --network host \
  -v /dev:/dev \
  -v "$PWD":/workspaces/JetRacer-ROS2 \
  jetracer-ros2-foxy
```

Explanation:

| Option                                | Purpose                                           |
| ------------------------------------- | ------------------------------------------------- |
| `-it`                                 | Opens an interactive terminal                     |
| `--rm`                                | Removes the container after exit                  |
| `--privileged`                        | Allows access to hardware devices                 |
| `--network host`                      | Uses the host network, useful for ROS 2 discovery |
| `-v /dev:/dev`                        | Gives access to cameras, I2C, serial, USB, etc.   |
| `-v "$PWD":/workspaces/JetRacer-ROS2` | Mounts the current project into the container     |

---

## 5. Build the ROS 2 workspace

Inside the container, the shell automatically starts in:

```bash
/workspaces/JetRacer-ROS2
```

Build the workspace with:

```bash
colcon build --symlink-install
```

Or use the included alias:

```bash
build_workspace
```

After building, source the workspace:

```bash
source install/setup.bash
```

The Dockerfile also adds this to `.bashrc`, so future shells will automatically source it if the workspace has already been built.

---

## 6. Source ROS 2 manually

If needed, source ROS 2 Foxy manually:

```bash
source /opt/ros/foxy/setup.bash
```

Then source the local workspace:

```bash
source /workspaces/JetRacer-ROS2/install/setup.bash
```

---

## 7. Check the ROS 2 installation

Inside the container, verify ROS 2:

```bash
ros2 --help
```

Check the active ROS distribution:

```bash
echo $ROS_DISTRO
```

Expected output:

```text
foxy
```

List installed ROS packages:

```bash
ros2 pkg list
```

---

## 8. Check hardware access

### List video devices

```bash
ls /dev/video*
```

### Test camera devices

```bash
v4l2-ctl --list-devices
```

### Check I2C devices

```bash
i2cdetect -l
```

### Check USB devices

```bash
lsusb
```

If hardware devices are missing, make sure the container was started with:

```bash
--privileged -v /dev:/dev
```

---

## 9. Running ROS 2 commands

Example:

```bash
ros2 topic list
```

Run a ROS 2 node:

```bash
ros2 run <package_name> <node_name>
```

Launch a ROS 2 launch file:

```bash
ros2 launch <package_name> <launch_file.py>
```

Example format:

```bash
ros2 launch jetracer_bringup bringup.launch.py
```

Adjust the package and launch file names according to your workspace.

---

## 10. Using RViz2

RViz2 is installed in the container.

To run RViz2 from Docker, the host must allow GUI forwarding.

On Linux host:

```bash
xhost +local:docker
```

Run the container with X11 support:

```bash
docker run -it --rm \
  --privileged \
  --network host \
  -e DISPLAY="$DISPLAY" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /dev:/dev \
  -v "$PWD":/workspaces/JetRacer-ROS2 \
  jetracer-ros2-foxy
```

Inside the container:

```bash
rviz2
```

After finishing, you can restrict X11 access again:

```bash
xhost -local:docker
```

---

## 11. Using Foxglove Bridge

The Dockerfile tries to install:

```text
ros-foxy-foxglove-bridge
```

This package is optional because it may not always be available in the Foxy apt repository.

Check if it was installed:

```bash
ros2 pkg list | grep foxglove
```

If available, run:

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

Then connect Foxglove Studio to the bridge endpoint.

---

## 12. Clean rebuild

If the Docker cache causes problems, rebuild without cache:

```bash
docker build --no-cache -t jetracer-ros2-foxy .
```

To remove old containers and unused layers:

```bash
docker system prune
```

To remove unused images as well:

```bash
docker system prune -a
```

Use the last command carefully because it removes all unused Docker images.

---

## 13. Common problems

### `requirements.txt` not found

The Dockerfile contains:

```dockerfile
COPY requirements.txt /tmp/requirements.txt
```

So `requirements.txt` must exist next to the Dockerfile.

Fix:

```bash
touch requirements.txt
```

---

### ROS 2 package not found during build

ROS 2 Foxy is old and some packages may no longer be available from all apt mirrors.

The Dockerfile already treats `foxglove-bridge` as optional.

If another package fails, check it manually:

```bash
apt-cache show ros-foxy-package-name
```

Inside a temporary Ubuntu 20.04 container, you can also test package availability after adding the ROS 2 repository.

---

### ROS 2 topics are not visible between host and container

Use host networking:

```bash
--network host
```

Also check that both systems use the same ROS domain ID:

```bash
echo $ROS_DOMAIN_ID
```

Set it if needed:

```bash
export ROS_DOMAIN_ID=0
```

---

### Camera or USB device not visible

Run the container with:

```bash
--privileged -v /dev:/dev
```

Then check:

```bash
ls /dev/video*
lsusb
```

---

### Permission problems in mounted workspace

If files created inside the container are owned by root, fix them on the host:

```bash
sudo chown -R "$USER:$USER" .
```

This happens because the container runs as root by default.

---

## 14. Useful commands

Build image:

```bash
docker build -t jetracer-ros2-foxy .
```

Run container:

```bash
docker run -it --rm \
  --privileged \
  --network host \
  -v /dev:/dev \
  -v "$PWD":/workspaces/JetRacer-ROS2 \
  jetracer-ros2-foxy
```

Build workspace inside container:

```bash
colcon build --symlink-install
```

Source workspace:

```bash
source install/setup.bash
```

List ROS 2 topics:

```bash
ros2 topic list
```

List ROS 2 nodes:

```bash
ros2 node list
```

List ROS 2 packages:

```bash
ros2 pkg list
```

Run RViz2:

```bash
rviz2
```

---

## 15. Notes

This Dockerfile is intended for ROS 2 Foxy on Ubuntu 20.04.

ROS 2 Foxy is an older ROS 2 distribution. It is useful for Jetson and legacy Ubuntu 20.04 systems, but for newer systems consider:

| Ubuntu version | Recommended ROS 2 version |
| -------------- | ------------------------- |
| Ubuntu 20.04   | Foxy                      |
| Ubuntu 22.04   | Humble                    |
| Ubuntu 24.04   | Jazzy                     |

For this JetRacer setup, Foxy is used because the Docker image is based on Ubuntu 20.04.
