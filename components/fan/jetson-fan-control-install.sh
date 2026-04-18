#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
INSTALL_DIR='/usr/local/lib/automagic-fan'
CONFIG_DIR='/etc/automagic-fan'
SERVICE_PATH='/etc/systemd/system/automagic-fan.service'

if [[ ${EUID:-$(id -u)} -ne 0 ]]; then
  exec sudo -E "$0" "$@"
fi

[[ -f "$SCRIPT_DIR/src/main.py" ]] || { echo 'main.py not found.' >&2; exit 1; }
[[ -f "$SCRIPT_DIR/configs/automagic-fan.service" ]] || { echo 'Service file not found.' >&2; exit 1; }
[[ -f "$SCRIPT_DIR/configs/config_standard.json" ]] || { echo 'Default config not found.' >&2; exit 1; }

apt-get update
apt-get install -y python3

install -d -m 0755 "$INSTALL_DIR" "$CONFIG_DIR"
install -m 0755 "$SCRIPT_DIR/src/main.py" "$INSTALL_DIR/main.py"
install -m 0644 "$SCRIPT_DIR/configs/automagic-fan.service" "$SERVICE_PATH"

if [[ ! -f "$CONFIG_DIR/config.json" ]]; then
  install -m 0644 "$SCRIPT_DIR/configs/config_standard.json" "$CONFIG_DIR/config.json"
fi

systemctl daemon-reload
systemctl enable --now automagic-fan.service

echo 'automagic-fan installed successfully.'
