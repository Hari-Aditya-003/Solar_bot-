#!/usr/bin/env python3
"""
Connectivity Test — RadioMaster RP3 V2 (ELRS MAVLink mode) on Raspberry Pi 5

The RP3 V2 is configured in ELRS 3.x MAVLink output mode.
It sends RC_CHANNELS_OVERRIDE + RADIO_STATUS over MAVLink at 460800 baud.

Wiring (RPi 5 GPIO 40-pin header):
  RP3 V2 TX  →  Pin 10 (GPIO 15, RX)
  RP3 V2 GND →  Pin 6  (GND)
  RP3 V2 5V  ←  Pin 2  (5V)
  RP3 V2 RX  ←  Pin 8  (GPIO 14, TX)  optional, telemetry back

Tests:
  1. Serial port opens         /dev/ttyAMA0 @ 460800
  2. MAVLink messages arrive
  3. RC_CHANNELS_OVERRIDE received  (sticks readable)
  4. RADIO_STATUS received          (link quality readable)
  5. Live channel snapshot

Usage:
  python3 tools/check_receiver_connection.py
  python3 tools/check_receiver_connection.py --mav /dev/ttyUSB0   # if using USB adapter
"""

import argparse
import os
import sys
import time

MAV_PORT = "/dev/ttyAMA0"    # GPIO Pin 10 (RX) ← RP3 V2 TX
MAV_BAUD = 460_800           # ELRS MAVLink output baud rate

PASS = "\033[92mPASS\033[0m"
FAIL = "\033[91mFAIL\033[0m"
WARN = "\033[93mWARN\033[0m"
INFO = "\033[94mINFO\033[0m"

def step(n, title): print(f"\n[{n}] {title}")
def ok(msg):   print(f"  {PASS}  {msg}")
def err(msg):  print(f"  {FAIL}  {msg}")
def warn(msg): print(f"  {WARN}  {msg}")
def info(msg): print(f"  {INFO}  {msg}")

def bar(us, lo=1000, hi=2000, width=20):
    filled = int((us - lo) / (hi - lo) * width)
    filled = max(0, min(width, filled))
    return "[" + "#" * filled + "-" * (width - filled) + "]"

# ─── Test 1: port exists ──────────────────────────────────────────────────────
def test_port(port, baud):
    step(1, f"Serial port  {port} @ {baud} baud")
    if not os.path.exists(port):
        err(f"{port} not found")
        info("Check: raspi-config > Interface Options > Serial Port → enabled")
        info("Check: /boot/firmware/config.txt has  dtparam=uart0=on")
        return False
    try:
        import serial
        s = serial.Serial(port, baud, timeout=0.1)
        s.close()
        ok("Port opens OK")
        return True
    except PermissionError:
        err("Permission denied — run:  sudo bash tools/fix_uart_permissions.sh")
        return False
    except Exception as e:
        err(str(e))
        return False

# ─── Test 2-4: MAVLink messages ───────────────────────────────────────────────
def test_mavlink(port, baud):
    step(2, f"MAVLink connection  ({baud} baud)")
    try:
        from pymavlink import mavutil
    except ImportError:
        err("pymavlink not installed — run:  pip install pymavlink")
        return None, {}

    try:
        mav = mavutil.mavlink_connection(port, baud=baud, source_system=255)
    except Exception as e:
        err(f"Cannot open: {e}")
        return None, {}

    ok("Connection object created — reading messages (5 s) ...")

    rc_msgs = []
    radio_msgs = []
    deadline = time.time() + 5.0

    while time.time() < deadline:
        msg = mav.recv_msg()
        if msg is None:
            continue
        t = msg.get_type()
        if t == 'RC_CHANNELS_OVERRIDE':
            rc_msgs.append(msg)
        elif t == 'RADIO_STATUS':
            radio_msgs.append(msg)
        if rc_msgs and radio_msgs:
            break   # got both, no need to wait longer

    results = {"rc": rc_msgs, "radio": radio_msgs}
    return mav, results

def test_rc_channels(results):
    step(3, "RC_CHANNELS_OVERRIDE messages (sticks)")
    rc_msgs = results.get("rc", [])
    if not rc_msgs:
        err("No RC_CHANNELS_OVERRIDE messages received")
        info("Check: RP3 V2 LED blinking green = bound to RadioMaster TX")
        info("Check: RadioMaster TX is powered ON")
        return False
    msg = rc_msgs[-1]
    d = msg.to_dict()
    ch = [d.get(f'chan{i}_raw', 0) for i in range(1, 9)]
    ok(f"Got {len(rc_msgs)} RC messages")
    info(f"Steering CH1 = {ch[0]} µs   Throttle CH3 = {ch[2]} µs")
    return True

def test_radio_status(results):
    step(4, "RADIO_STATUS messages (link quality)")
    radio_msgs = results.get("radio", [])
    if not radio_msgs:
        warn("No RADIO_STATUS messages (link stats may come less often)")
        return True   # not fatal
    msg = radio_msgs[-1]
    d = msg.to_dict()
    rssi    = d.get('rssi', '?')
    remrssi = d.get('remrssi', '?')
    noise   = d.get('noise', '?')
    txbuf   = d.get('txbuf', '?')
    ok(f"Got {len(radio_msgs)} RADIO_STATUS messages")
    info(f"RSSI: {rssi}   Remote RSSI: {remrssi}   Noise: {noise}   TX buf: {txbuf}%")
    return True

def print_channel_snapshot(results):
    rc_msgs = results.get("rc", [])
    if not rc_msgs:
        return
    d = rc_msgs[-1].to_dict()
    ch = [d.get(f'chan{i}_raw', 0) for i in range(1, 9)]
    labels = ["Steering", "Pitch   ", "Throttle", "Yaw     ",
              "SA      ", "SB      ", "SC      ", "SD      "]
    print("\n" + "=" * 58)
    print("  CHANNEL SNAPSHOT (last RC frame)")
    print("=" * 58)
    for i, (us, lbl) in enumerate(zip(ch, labels, strict=True)):
        b = bar(us)
        print(f"  CH{i+1:02d}  {lbl}  {b}  {us:4d} µs")

def print_result(results_dict):
    print("\n" + "=" * 58)
    print("  CONNECTIVITY TEST RESULTS")
    print("=" * 58)
    for name, passed in results_dict.items():
        status = PASS if passed else FAIL
        print(f"  {status}  {name}")
    all_ok = all(results_dict.values())
    print()
    if all_ok:
        print("  RESULT: ALL TESTS PASSED")
        print("  RP3 V2 → MAVLink link is working correctly.")
        print("  Next: run   python3 tools/mavlink_bridge.py  for live car control")
    else:
        print("  RESULT: SOME TESTS FAILED — see details above")
    print("=" * 58)

# ─── Main ─────────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mav", default=MAV_PORT,
                    help=f"Serial port (default: {MAV_PORT})")
    ap.add_argument("--baud", type=int, default=MAV_BAUD,
                    help=f"Baud rate (default: {MAV_BAUD})")
    args = ap.parse_args()

    print()
    print("=" * 58)
    print("  RP3 V2 + MAVLink Connectivity Test")
    print("  Raspberry Pi 5  |  ELRS MAVLink mode  |  460800 baud")
    print("=" * 58)

    r = {}

    r["Serial port opens"] = test_port(args.mav, args.baud)
    if not r["Serial port opens"]:
        print_result(r)
        sys.exit(1)

    mav, msg_results = test_mavlink(args.mav, args.baud)
    r["MAVLink messages received"] = bool(msg_results.get("rc") or msg_results.get("radio"))

    if mav:
        r["RC_CHANNELS_OVERRIDE (sticks)"] = test_rc_channels(msg_results)
        r["RADIO_STATUS (link quality)"]   = test_radio_status(msg_results)
        print_channel_snapshot(msg_results)
        mav.close()

    print_result(r)

if __name__ == "__main__":
    main()
