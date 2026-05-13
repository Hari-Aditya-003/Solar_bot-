# Solar Bot Tools

Standalone field tools live here so the project root stays focused on the
installable app.

Run commands from `solar_Bot/`:

```bash
python3 tools/check_gps.py --port auto --seconds 10
python3 tools/check_pico_uart.py --port /dev/ttyAMA4
python3 tools/check_motors.py --port /dev/ttyAMA4 --speed 30
python3 tools/check_rp3v2.py
python3 tools/mavlink_bridge.py
python3 tools/rc_drive.py --no-gps
sudo bash tools/fix_uart_permissions.sh
```

`_paths.py` is a small shared helper that lets these scripts import
`mission_planner` and save missions under `paths/` even when run directly.
