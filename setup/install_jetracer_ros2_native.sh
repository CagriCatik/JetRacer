#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=./common.sh
source "$SCRIPT_DIR/common.sh"

original_args=("$@")
install_variant='ros-base'
skip_bashrc=0
install_jetracer_packages=1

while [[ $# -gt 0 ]]; do
  case "$1" in
    --desktop)
      install_variant='desktop'
      shift
      ;;
    --ros-base)
      install_variant='ros-base'
      shift
      ;;
    --skip-bashrc)
      skip_bashrc=1
      shift
      ;;
    --no-jetracer-packages)
      install_jetracer_packages=0
      shift
      ;;
    *)
      fail "Unknown argument: $1"
      ;;
  esac
done

require_root "${original_args[@]}"
detect_ubuntu

case "$UBUNTU_VERSION_ID" in
  20.04)
    ros_distro='foxy'
    ;;
  22.04)
    ros_distro='humble'
    ;;
  24.04)
    ros_distro='jazzy'
    ;;
  *)
    fail "Native ROS 2 apt installation is not supported by this script on Ubuntu $UBUNTU_VERSION_ID. Use Ubuntu 20.04 for Foxy, 22.04 for Humble, or 24.04 for Jazzy."
    ;;
esac

log "Installing ROS 2 $ros_distro ($install_variant) on Ubuntu $UBUNTU_VERSION_ID."

export DEBIAN_FRONTEND=noninteractive

apt-get update
apt-get install -y \
  ca-certificates \
  curl \
  gnupg2 \
  locales \
  lsb-release \
  software-properties-common

locale-gen en_US en_US.UTF-8
update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

add-apt-repository universe -y || true

ensure_apt_keyrings_dir

curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  | gpg --dearmor -o /etc/apt/keyrings/ros-archive-keyring.gpg

chmod a+r /etc/apt/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $UBUNTU_CODENAME main" \
  > /etc/apt/sources.list.d/ros2.list

apt-get update

base_packages=(
  "ros-$ros_distro-$install_variant"
  "ros-$ros_distro-rmw-cyclonedds-cpp"
  "ros-$ros_distro-rmw-fastrtps-cpp"
  python3-argcomplete
  python3-colcon-common-extensions
  python3-pip
  python3-rosdep
  python3-vcstool
)

apt-get install -y "${base_packages[@]}"

if [[ "$install_jetracer_packages" -eq 1 ]]; then
  log "Installing JetRacer ROS 2 packages for $ros_distro where available."

  required_jetracer_packages=(
    "ros-$ros_distro-cv-bridge"
    "ros-$ros_distro-diagnostic-updater"
    "ros-$ros_distro-image-transport"
    "ros-$ros_distro-camera-info-manager"
    "ros-$ros_distro-joy"
    "ros-$ros_distro-ackermann-msgs"
    "ros-$ros_distro-xacro"
    "ros-$ros_distro-robot-state-publisher"
    "ros-$ros_distro-joint-state-publisher"
    "ros-$ros_distro-rviz2"
    "ros-$ros_distro-slam-toolbox"
    "ros-$ros_distro-nav2-bringup"
    "ros-$ros_distro-robot-localization"
    "ros-$ros_distro-vision-msgs"
  )

  optional_jetracer_packages=(
    "ros-$ros_distro-joint-state-publisher-gui"
    "ros-$ros_distro-twist-mux"
    "ros-$ros_distro-rplidar-ros"
    "ros-$ros_distro-gscam"
    "ros-$ros_distro-foxglove-bridge"
  )

  apt-get install -y "${required_jetracer_packages[@]}"

  for pkg in "${optional_jetracer_packages[@]}"; do
    if apt-cache show "$pkg" >/dev/null 2>&1; then
      log "Installing optional package: $pkg"
      apt-get install -y "$pkg"
    else
      log "Skipping unavailable optional package: $pkg"
    fi
  done
fi

rosdep init 2>/dev/null || true
sudo -u "$(target_user)" rosdep update --rosdistro "$ros_distro" || true

workspace_dir="$(target_home)/jetracer_ws/src"
install -d -m 0755 "$workspace_dir"
chown -R "$(target_user):$(target_user)" "$(target_home)/jetracer_ws"

if [[ "$skip_bashrc" -eq 0 ]]; then
  bashrc="$(target_home)/.bashrc"
  append_if_missing "source /opt/ros/$ros_distro/setup.bash" "$bashrc"
fi

log "ROS 2 installation completed."
log "ROS distro: $ros_distro"
log "Workspace directory prepared at $workspace_dir"
log "Next step:"
log "  source /opt/ros/$ros_distro/setup.bash"
log "  cd $(target_home)/jetracer_ws"
log "  colcon build --symlink-install"