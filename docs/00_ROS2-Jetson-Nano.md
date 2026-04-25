# 0. Jetson Nano Ubuntu 20.04 Workaround

This project supports Jetson Nano with a practical workaround:

- Host OS on the Nano: Ubuntu 20.04 community image
- Robot runtime: ROS 2 Foxy inside Docker

This avoids the official-image gap on Jetson Nano while keeping the codebase aligned with the ROS 2 Foxy stack used in this repository.

## Why this workflow exists

- Jetson Nano + JetPack 4.6.1 is the target hardware baseline for this repo.
- `setup/install_ros2.sh` supports native apt install only on Ubuntu 22.04 (Humble) and 24.04 (Jazzy). Foxy is deployed via Docker on the Ubuntu 20.04 workaround image.
- On Nano with Ubuntu 20.04 workaround image, the supported path for this repo is Docker, not native ROS 2 apt install.

## Compatibility Matrix

| Layer | Status in this repository | Notes |
| :--- | :--- | :--- |
| Jetson Nano host OS | Supported via Ubuntu 20.04 workaround image | Community image, not official NVIDIA Ubuntu 20.04 image |
| Native `setup/install_ros2.sh` on Nano | Not supported | Script rejects Ubuntu 20.04 by design |
| Docker workflow on Nano | Supported and recommended | Canonical path for this project |
| ROS runtime | ROS 2 Foxy in container | Ubuntu 20.04 base with ROS 2 Foxy |

## Workaround Image Source

Reference source used for the workaround path:

- [Jetson-Nano-Ubuntu-20-Image](https://github.com/Qengineering/Jetson-Nano-Ubuntu-20-image)
- [Install Ubuntu 20.04 on Jetson Nano](https://qengineering.eu/install-ubuntu-20.04-on-jetson-nano.html)

Follow that source for SD image download and flashing details. Their image has historically used `jetson` as the default password; change credentials immediately after first boot.

## Project-Specific Setup on Jetson Nano

After flashing and first boot:

1. Update OS packages.
2. Expand the filesystem if you used a larger SD card.
3. Verify Docker availability.

```bash
docker --version
docker info | grep -i Runtime
```

If Docker is missing:

```bash
cd setup
./install_docker.sh
```

Then run this repository exactly as documented in deployment docs:

```bash
docker compose up -d --build
docker exec -it jetracer_workspace bash
source ~/.bashrc
rosdep update
rosdep install --from-paths src --ignore-src -r -y
build_workspace
source install/setup.bash
```

## Rules for This Repo on Nano

- Do not run `setup/install_ros2.sh` on Nano Ubuntu 20.04.
- Keep ROS 2 runtime inside the container.
- Use `03_Deployment_and_Docker.md` as the operational runbook after host preparation.

---

Next: [03. Deployment and Docker](03_Deployment_and_Docker.md)
