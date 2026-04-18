#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=./common.sh
source "$SCRIPT_DIR/common.sh"

original_args=("$@")
install_variant='ros-base'
skip_bashrc=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --desktop)
      install_variant='desktop'
      shift
      ;;
    --skip-bashrc)
      skip_bashrc=1
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
  22.04)
    ros_distro='humble'
    ;;
  24.04)
    ros_distro='jazzy'
    ;;
  *)
    fail "Native ROS 2 apt installation is not supported by this script on Ubuntu $UBUNTU_VERSION_ID. Use a supported Ubuntu release for native ROS 2, or document a constrained Jetson-specific workaround separately."
    ;;
esac

log "Installing ROS 2 $ros_distro ($install_variant) on Ubuntu $UBUNTU_VERSION_ID."

export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y ca-certificates curl gnupg2 locales lsb-release software-properties-common
locale-gen en_US en_US.UTF-8
update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8
add-apt-repository universe -y || true

ensure_apt_keyrings_dir
curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | gpg --dearmor -o /etc/apt/keyrings/ros-archive-keyring.gpg
chmod a+r /etc/apt/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $UBUNTU_CODENAME main" > /etc/apt/sources.list.d/ros2.list

apt-get update
apt-get install -y \
  "ros-$ros_distro-$install_variant" \
  "ros-$ros_distro-rmw-cyclonedds-cpp" \
  "ros-$ros_distro-rmw-fastrtps-cpp" \
  python3-argcomplete \
  python3-colcon-common-extensions \
  python3-pip \
  python3-rosdep \
  python3-vcstool

rosdep init 2>/dev/null || true
sudo -u "$(target_user)" rosdep update

workspace_dir="$(target_home)/jetracer_ws/src"
install -d -m 0755 "$workspace_dir"

if [[ $skip_bashrc -eq 0 ]]; then
  bashrc="$(target_home)/.bashrc"
  append_if_missing "source /opt/ros/$ros_distro/setup.bash" "$bashrc"
fi

log "ROS 2 installation completed."
log "Workspace directory prepared at $workspace_dir"
log "Next step: source /opt/ros/$ros_distro/setup.bash and build packages under ~/jetracer_ws/src"
