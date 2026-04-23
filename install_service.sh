#!/bin/bash

# JetRacer Service Installer
# Installs the jetracer.service to /etc/systemd/system/
# Automatically detects the JetRacer repository location.

# Determine the repository root
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

SERVICE_FILE="${REPO_ROOT}/jetracer.service"
BOOT_SCRIPT="${REPO_ROOT}/jetracer_boot.sh"

echo "--- JetRacer Service Installation ---"

# 1. Ensure scripts are executable
chmod +x "$BOOT_SCRIPT"
echo "Made boot script executable."

# 2. Check if service file exists
if [ ! -f "$SERVICE_FILE" ]; then
    echo "ERROR: Service file not found at $SERVICE_FILE"
    exit 1
fi

# 3. Copy to systemd directory
sudo cp "$SERVICE_FILE" /etc/systemd/system/jetracer.service
echo "Copied service file to /etc/systemd/system/"

# 4. Reload and Enable
sudo systemctl daemon-reload
sudo systemctl enable jetracer.service
echo "Service enabled. It will start automatically at next boot."

echo "You can start it now with: sudo systemctl start jetracer.service"
echo "You can check logs with: journalctl -u jetracer.service -f"
