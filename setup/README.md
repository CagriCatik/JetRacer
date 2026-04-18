# Setup Scripts Documentation

This document describes the `.sh` scripts that currently exist in `setup/` and what each one does.

## Scope

Scripts covered:

- `common.sh`
- `install_ros2.sh`
- `install_docker.sh`
- `install_balena_etcher.sh`
- `install_pi_imager.sh`
- `install_vscode.sh`
- `install_nomachine.sh`
- `install_foxglove.sh`

## Shared Helper: `common.sh`

`common.sh` provides utility functions used by the installer scripts:

- logging helpers: `log`, `warn`, `error`, `fail`
- command checks: `command_exists`
- privilege escalation: `require_root` (re-execs via `sudo -E` if needed)
- Ubuntu detection: `detect_ubuntu` (reads `/etc/os-release`, exports `UBUNTU_VERSION_ID` and `UBUNTU_CODENAME`)
- user/home detection for sudo flows: `target_user`, `target_home`
- idempotent line append helper: `append_if_missing`
- apt keyrings directory helper: `ensure_apt_keyrings_dir`

## Installers

### `install_ros2.sh`

Installs native ROS 2 from apt on supported Ubuntu versions.

Supported Ubuntu versions:

- `22.04` -> installs ROS 2 `humble`
- `24.04` -> installs ROS 2 `jazzy`

Not supported:

- `20.04` -> script exits by design
- Jetson Nano workaround host flow in this repository (use Docker runtime instead)

Arguments:

- `--desktop` -> install `ros-$distro-desktop` (default is `ros-base`)
- `--skip-bashrc` -> do not add `source /opt/ros/$distro/setup.bash` to target user `.bashrc`

Actions:

- configures locales and `universe`
- adds ROS 2 apt key/repository
- installs ROS 2 + RMW packages + common dev tools (`colcon`, `rosdep`, `vcstool`, etc.)
- runs `rosdep init` (if needed) and `rosdep update` as target user
- creates workspace directory: `~/jetracer_ws/src` for target user

Example:

```bash
cd setup
./install_ros2.sh --desktop
```

Jetson Nano workaround note:

- If you are using the Ubuntu 20.04 Nano workaround documented in `../docs/00_ROS2-Jetson-Nano.md`, do not run this script.
- For that flow, install Docker (`install_docker.sh`) and run the robot stack in container (`../docs/03_Deployment_and_Docker.md`).

### `install_docker.sh`

Installs Docker Engine from Docker's official Ubuntu apt repository.

Actions:

- installs apt prerequisites (`ca-certificates`, `curl`, `gnupg`)
- adds Docker GPG key and apt source
- installs:
  - `docker-ce`
  - `docker-ce-cli`
  - `containerd.io`
  - `docker-buildx-plugin`
  - `docker-compose-plugin`
- enables and starts Docker service
- adds the target user to `docker` group

Example:

```bash
cd setup
./install_docker.sh
```

### `install_balena_etcher.sh`

Installs Balena Etcher from the vendor repository.

Constraints:

- script is intended for `amd64` hosts
- exits with error on non-`amd64`

Actions:

- installs `curl` and `gnupg`
- adds Balena Etcher key and apt source
- installs `balena-etcher-electron`

Example:

```bash
cd setup
./install_balena_etcher.sh
```

### `install_pi_imager.sh`

Installs Raspberry Pi Imager from apt if available.

Actions:

- runs `apt-get update`
- checks `apt-cache show rpi-imager`
- installs `rpi-imager` when available
- fails with a clear message if package is not present in current apt sources

Example:

```bash
cd setup
./install_pi_imager.sh
```

### `install_vscode.sh`

Installs Visual Studio Code by downloading a `.deb` from the official update endpoint.

Arguments:

- `--version <value>` (default: `latest`)

Architecture mapping:

- `amd64` -> VS Code `x64`
- `arm64` -> VS Code `arm64`
- other architectures are rejected

Actions:

- downloads installer to a temporary `.deb`
- installs with `apt-get install -y <deb>`

Example:

```bash
cd setup
./install_vscode.sh --version latest
```

### `install_nomachine.sh`

Installs NoMachine from a `.zip` or `.deb`.

Arguments:

- `--package <path>`: package path override

Default package path in script:

- `setup/Nomachine_7.10.1_1_arm64.zip` (preferred when present)
- fallback: `remote/Nomachine_7.10.1_1_arm64.zip`

Behavior:

- installs `unzip`
- if package is `.zip`, extracts and finds `nomachine_*.deb`
- if package is `.deb`, installs directly
- enables and starts `nxserver` (best effort)

Example with explicit package path:

```bash
cd setup
./install_nomachine.sh --package ./Nomachine_7.10.1_1_arm64.zip
```

### `install_foxglove.sh`

Installs Foxglove Studio from a locally downloaded `.deb`.

Behavior:

- searches current directory for `foxglove-studio-*.deb`
- picks the newest match
- installs via apt as root
- prints update hint: `sudo apt update && sudo apt install foxglove-studio`

Optional argument:

- `--package <path>` to install a specific local `.deb` file

Example:

```bash
cd setup
./install_foxglove.sh
```

## Duplicate Wrappers

Legacy duplicate wrapper scripts were removed to keep one authoritative script per installation task.

## Recommended Usage Order

Host workstation:

1. `install_balena_etcher.sh` (or `install_pi_imager.sh`)
2. `install_docker.sh` (only if this machine is your Docker host)
3. `install_vscode.sh`

Jetson target (Ubuntu 20.04 workaround path):

1. `install_docker.sh` (if Docker is not already available)
2. `install_nomachine.sh` (optional)
3. Follow `../docs/03_Deployment_and_Docker.md` for runtime

Native ROS target (Ubuntu 22.04 or 24.04 only):

1. `install_ros2.sh`
2. `install_nomachine.sh` (optional)
3. `install_foxglove.sh` (optional, local desktop use)

## Notes

- Most installers are Ubuntu-only and require internet access.
- Scripts using `common.sh` auto-escalate to sudo when needed.
- After Docker installation, logout/login may be required for group changes to apply.
