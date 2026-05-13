#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SERVICE_SRC="$ROOT_DIR/systemd/solarbot-mission.service"
SERVICE_DST="/etc/systemd/system/solarbot-mission.service"

if [[ ! -f "$SERVICE_SRC" ]]; then
  echo "Missing service file: $SERVICE_SRC" >&2
  exit 1
fi

echo "Installing Solar Bot autostart service..."
sudo install -m 0644 "$SERVICE_SRC" "$SERVICE_DST"
sudo systemctl daemon-reload
sudo systemctl enable solarbot-mission.service
sudo systemctl restart solarbot-mission.service

echo
echo "Autostart enabled."
echo "Status: sudo systemctl status solarbot-mission.service --no-pager"
echo "Logs:   journalctl -u solarbot-mission.service -f"
echo "Open:   http://192.168.2.6:5000"
