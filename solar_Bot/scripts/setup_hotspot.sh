#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────────
# SolarBot — One-time hotspot setup  (run with sudo)
#
#   sudo ./scripts/setup_hotspot.sh [SSID] [PASSWORD]
#
#   Defaults:  SSID=SolarBot   PASSWORD=solarbot123   IP=192.168.4.1
#
# What it does:
#   1. Creates a NetworkManager hotspot profile "SolarBot-AP"
#   2. Installs /etc/systemd/system/solarbot-hotspot.service
#   3. Enables the service so it runs at every boot
# ─────────────────────────────────────────────────────────────────────────────

set -euo pipefail

SSID="${1:-SolarBot}"
PASSWORD="${2:-solarbot123}"
HOTSPOT_IP="192.168.4.1"
HOTSPOT_CON="SolarBot-AP"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SERVICE=/etc/systemd/system/solarbot-hotspot.service

if [[ $EUID -ne 0 ]]; then
    echo "ERROR: run with sudo"
    exit 1
fi

echo "╔══════════════════════════════════════════════╗"
echo "  SolarBot Hotspot Setup"
echo "  SSID     : $SSID"
echo "  Password : $PASSWORD"
echo "  Pi IP    : $HOTSPOT_IP  (hotspot mode)"
echo "╚══════════════════════════════════════════════╝"
echo

# ── NetworkManager profile ────────────────────────────────────────────────────

echo "[1/4] Removing old hotspot profile (if any)..."
nmcli connection delete "$HOTSPOT_CON" 2>/dev/null && echo "  Removed." || echo "  None found."

echo "[2/4] Creating hotspot profile..."
nmcli connection add \
    type wifi \
    ifname wlan0 \
    con-name "$HOTSPOT_CON" \
    autoconnect no \
    ssid "$SSID" \
    "802-11-wireless.mode" ap \
    "802-11-wireless.band" bg \
    wifi-sec.key-mgmt wpa-psk \
    wifi-sec.psk "$PASSWORD" \
    ipv4.method shared \
    "ipv4.addresses" "${HOTSPOT_IP}/24" \
    ipv6.method disabled
echo "  Done."

# ── Systemd service ───────────────────────────────────────────────────────────

echo "[3/4] Installing systemd service..."
chmod +x "${SCRIPT_DIR}/auto_hotspot.sh"

cat > "$SERVICE" <<EOF
[Unit]
Description=SolarBot WiFi Manager — hotspot fallback
After=NetworkManager.service
Wants=NetworkManager.service

[Service]
Type=oneshot
ExecStart=${SCRIPT_DIR}/auto_hotspot.sh
RemainAfterExit=yes
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF

echo "[4/4] Enabling service..."
systemctl daemon-reload
systemctl enable solarbot-hotspot.service
echo "  Done."

# ── Summary ───────────────────────────────────────────────────────────────────
echo
echo "╔══════════════════════════════════════════════════════════════╗"
echo "  Setup complete!"
echo
echo "  On NEXT BOOT:"
echo "    • If a known WiFi router is nearby → connects automatically"
echo "    • Otherwise → broadcasts hotspot '$SSID'"
echo
echo "  To connect your phone/laptop in the field:"
echo "    WiFi:     $SSID"
echo "    Password: $PASSWORD"
echo "    Dashboard: http://${HOTSPOT_IP}:8080"
echo
echo "  To test NOW without rebooting:"
echo "    sudo systemctl start solarbot-hotspot.service"
echo "    journalctl -u solarbot-hotspot.service -f"
echo "╚══════════════════════════════════════════════════════════════╝"
