#!/bin/bash

# JetRacer Service Installer
# Installs the jetracer.service to /etc/systemd/system/
# Automatically detects the JetRacer repository location.

# Determine the repository root
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

BOOT_SCRIPT="${REPO_ROOT}/jetracer_boot.sh"
SERVICE_USER="${SUDO_USER:-$USER}"
TMP_SERVICE="$(mktemp)"

echo "--- JetRacer Service Installation ---"

# 1. Ensure scripts are executable
chmod +x "$BOOT_SCRIPT"
echo "Made boot script executable."

# 2. Generate a systemd unit for this user and checkout path.
cat > "$TMP_SERVICE" <<EOF
[Unit]
Description=JetRacer ROS 2 Sentinel Service
After=network.target network-online.target
Wants=network-online.target

[Service]
Type=simple
User=${SERVICE_USER}
WorkingDirectory=${REPO_ROOT}
ExecStart=/bin/bash ${BOOT_SCRIPT}
Restart=always
RestartSec=5
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF

# 3. Copy to systemd directory
sudo cp "$TMP_SERVICE" /etc/systemd/system/jetracer.service
rm -f "$TMP_SERVICE"
echo "Installed service file for user ${SERVICE_USER} at /etc/systemd/system/jetracer.service"

# 4. Reload and Enable
sudo systemctl daemon-reload
sudo systemctl enable jetracer.service
echo "Service enabled. It will start automatically at next boot."

echo "You can start it now with: sudo systemctl start jetracer.service"
echo "You can check logs with: journalctl -u jetracer.service -f"
