#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
# shellcheck source=./common.sh
source "$SCRIPT_DIR/common.sh"

original_args=("$@")
local_package="$SCRIPT_DIR/Nomachine_7.10.1_1_arm64.zip"
repo_package="$REPO_ROOT/remote/Nomachine_7.10.1_1_arm64.zip"
if [[ -f "$local_package" ]]; then
  package_path="$local_package"
else
  package_path="$repo_package"
fi

while [[ $# -gt 0 ]]; do
  case "$1" in
    --package)
      [[ $# -ge 2 ]] || fail '--package requires a path.'
      package_path=$2
      shift 2
      ;;
    *)
      fail "Unknown argument: $1"
      ;;
  esac
done

require_root "${original_args[@]}"
detect_ubuntu
[[ -f "$package_path" ]] || fail "Package not found: $package_path"

log "Installing NoMachine from package: $package_path"

export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y unzip

tmp_dir=$(mktemp -d)
trap 'rm -rf "$tmp_dir"' EXIT

case "$package_path" in
  *.zip)
    unzip -q "$package_path" -d "$tmp_dir"
    deb_path=$(find "$tmp_dir" -maxdepth 2 -type f -name 'nomachine_*.deb' | head -n 1 || true)
    [[ -n ${deb_path:-} ]] || fail 'No NoMachine .deb package found inside the zip archive.'
    ;;
  *.deb)
    deb_path=$package_path
    ;;
  *)
    fail 'Expected a .zip or .deb package for NoMachine installation.'
    ;;
esac

apt-get install -y "$deb_path"
systemctl enable --now nxserver || true

log 'NoMachine installation completed.'
