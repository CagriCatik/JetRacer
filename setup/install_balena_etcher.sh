#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=./common.sh
source "$SCRIPT_DIR/common.sh"

require_root "$@"
detect_ubuntu

if [[ $(dpkg --print-architecture) != 'amd64' ]]; then
  fail 'Balena Etcher is usually installed on an amd64 Linux workstation. For other architectures, install it manually from the vendor release artifacts.'
fi

log 'Installing Balena Etcher from the vendor apt repository.'

export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y curl gnupg

ensure_apt_keyrings_dir
curl -fsSL https://dl.cloudsmith.io/public/balena/etcher/gpg.key | gpg --dearmor -o /etc/apt/keyrings/balena-etcher-archive-keyring.gpg
chmod a+r /etc/apt/keyrings/balena-etcher-archive-keyring.gpg

echo "deb [signed-by=/etc/apt/keyrings/balena-etcher-archive-keyring.gpg] https://dl.cloudsmith.io/public/balena/etcher/deb/ubuntu $UBUNTU_CODENAME main" > /etc/apt/sources.list.d/balena-etcher.list

apt-get update
apt-get install -y balena-etcher-electron

log 'Balena Etcher installation completed.'
