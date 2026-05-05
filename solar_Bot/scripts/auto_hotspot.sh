#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────────
# SolarBot — Auto Hotspot Manager
# Called by systemd at boot.
#   1. Waits up to WIFI_WAIT seconds for NetworkManager to join a known network.
#   2. If no WiFi found, activates the "SolarBot-AP" hotspot profile.
#   3. Writes /tmp/solarbot_net.json for rc_drive.py and web_ui.py to read.
# ─────────────────────────────────────────────────────────────────────────────

HOTSPOT_CON="SolarBot-AP"
WIFI_WAIT=20          # seconds to wait for existing WiFi
IP_FILE=/tmp/solarbot_net.json
LOG=/tmp/solarbot_wifi.log

log() { echo "$(date '+%H:%M:%S') $*" | tee -a "$LOG"; }

# ── Helpers ───────────────────────────────────────────────────────────────────

wifi_state() {
    # Returns NM connectivity state as integer (100 = connected)
    nmcli -t -f GENERAL.STATE dev show wlan0 2>/dev/null \
        | grep -oP '\d+' | head -1 || echo "0"
}

wifi_ip() {
    nmcli -t -f IP4.ADDRESS dev show wlan0 2>/dev/null \
        | head -1 | sed 's/.*://;s|/.*||'
}

active_ssid() {
    nmcli -t -f ACTIVE,SSID dev wifi 2>/dev/null \
        | grep '^yes:' | cut -d: -f2 | head -1 || echo ""
}

write_net() {
    local mode="$1" ssid="$2" ip="$3" pw="${4:-}"
    printf '{"mode":"%s","ssid":"%s","ip":"%s","password":"%s"}\n' \
        "$mode" "$ssid" "$ip" "$pw" > "$IP_FILE"
    log "net → mode=$mode  ssid='$ssid'  ip=$ip"
}

# ── Main ──────────────────────────────────────────────────────────────────────

log "=== SolarBot WiFi manager starting ==="

# Step 1 — wait for any existing connection
log "Waiting ${WIFI_WAIT}s for WiFi..."
for ((i=1; i<=WIFI_WAIT; i++)); do
    if [[ "$(wifi_state)" -ge 100 ]]; then
        SSID=$(active_ssid)
        IP=$(wifi_ip)
        log "Connected to router '$SSID'  IP=$IP"
        write_net "wifi" "$SSID" "$IP" ""
        exit 0
    fi
    sleep 1
done

# Step 2 — no router found, try hotspot
log "No WiFi after ${WIFI_WAIT}s — starting hotspot..."

if ! nmcli connection show "$HOTSPOT_CON" &>/dev/null; then
    log "ERROR: hotspot profile '$HOTSPOT_CON' missing — run setup_hotspot.sh first"
    write_net "none" "" "" ""
    exit 1
fi

# Disconnect wlan0 from any partial connections first
nmcli device disconnect wlan0 2>/dev/null || true
sleep 1

if nmcli connection up "$HOTSPOT_CON"; then
    sleep 3
    IP=$(wifi_ip)
    IP="${IP:-192.168.4.1}"
    PW=$(nmcli -s -g 802-11-wireless-security.psk connection show "$HOTSPOT_CON" 2>/dev/null || echo "solarbot123")
    log "Hotspot up  SSID=SolarBot  IP=$IP"
    write_net "hotspot" "SolarBot" "$IP" "$PW"
else
    log "ERROR: could not bring up hotspot"
    write_net "error" "" "" ""
    exit 1
fi
