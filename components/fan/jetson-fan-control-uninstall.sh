#!/usr/bin/env bash
set -euo pipefail

INSTALL_DIR='/usr/local/lib/automagic-fan'
CONFIG_DIR='/etc/automagic-fan'
SERVICE_PATH='/etc/systemd/system/automagic-fan.service'

if [[ ${EUID:-$(id -u)} -ne 0 ]]; then
  exec sudo -E "$0" "$@"
fi

systemctl disable --now automagic-fan.service 2>/dev/null || true
rm -f "$SERVICE_PATH"
rm -rf "$INSTALL_DIR" "$CONFIG_DIR"
systemctl daemon-reload

echo 'automagic-fan removed.'
