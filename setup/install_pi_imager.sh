#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=./common.sh
source "$SCRIPT_DIR/common.sh"

require_root "$@"
detect_ubuntu

log 'Installing Raspberry Pi Imager if it is available through apt.'

export DEBIAN_FRONTEND=noninteractive
apt-get update

if apt-cache show rpi-imager >/dev/null 2>&1; then
  apt-get install -y rpi-imager
  log 'Raspberry Pi Imager installation completed.'
else
  fail 'The rpi-imager package is not available in the current apt sources. Install the official AppImage manually from raspberrypi.com/software if needed.'
fi
