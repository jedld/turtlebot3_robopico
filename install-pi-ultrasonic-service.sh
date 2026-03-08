#!/usr/bin/env bash
set -euo pipefail

SERVICE_SRC="/home/jedld/turtlebot3_robopico/systemd/pi-ultrasonic-dual.service"
SERVICE_DST="/etc/systemd/system/pi-ultrasonic-dual.service"

if [[ ! -f "$SERVICE_SRC" ]]; then
  echo "Missing service file: $SERVICE_SRC"
  exit 1
fi

echo "Installing $SERVICE_DST"
sudo cp "$SERVICE_SRC" "$SERVICE_DST"
sudo systemctl daemon-reload
sudo systemctl enable --now pi-ultrasonic-dual.service

echo "Service status:"
sudo systemctl --no-pager --full status pi-ultrasonic-dual.service || true

echo
echo "Tail logs with: journalctl -u pi-ultrasonic-dual.service -f"
