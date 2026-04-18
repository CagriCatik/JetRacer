#!/usr/bin/env bash
set -euo pipefail

log() {
  printf '[INFO] %s\n' "$*"
}

warn() {
  printf '[WARN] %s\n' "$*" >&2
}

error() {
  printf '[ERROR] %s\n' "$*" >&2
}

fail() {
  error "$*"
  exit 1
}

command_exists() {
  command -v "$1" >/dev/null 2>&1
}

require_root() {
  if [[ ${EUID:-$(id -u)} -ne 0 ]]; then
    exec sudo -E "$0" "$@"
  fi
}

detect_ubuntu() {
  [[ -r /etc/os-release ]] || fail 'Cannot read /etc/os-release.'
  # shellcheck disable=SC1091
  . /etc/os-release
  [[ ${ID:-} == 'ubuntu' ]] || fail 'These setup scripts only support Ubuntu.'
  UBUNTU_VERSION_ID=${VERSION_ID:-}
  UBUNTU_CODENAME=${VERSION_CODENAME:-${UBUNTU_CODENAME:-}}
  export UBUNTU_VERSION_ID UBUNTU_CODENAME
}

target_user() {
  printf '%s\n' "${SUDO_USER:-$USER}"
}

target_home() {
  local user
  user=$(target_user)
  getent passwd "$user" | cut -d: -f6
}

append_if_missing() {
  local line file
  line=$1
  file=$2
  touch "$file"
  grep -Fqx "$line" "$file" || printf '%s\n' "$line" >> "$file"
}

ensure_apt_keyrings_dir() {
  install -d -m 0755 /etc/apt/keyrings
}
