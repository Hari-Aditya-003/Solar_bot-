#!/usr/bin/env python3
"""
MAVLink Car Bridge — RadioMaster RP3 V2 (ELRS MAVLink) → Car control
Hardware: Raspberry Pi 5

The RP3 V2 runs ELRS 3.x in MAVLink output mode.
It sends RC_CHANNELS_OVERRIDE + RADIO_STATUS over MAVLink at 460800 baud.

Wiring (RPi 5 GPIO 40-pin header):
  RP3 V2 TX  →  Pin 10 (GPIO 15, RX)  =  /dev/ttyAMA0
  RP3 V2 GND →  Pin 6  (GND)
  RP3 V2 5V  ←  Pin 2  (5V)
  RP3 V2 RX  ←  Pin 8  (GPIO 14, TX)  optional

Car channel mapping (ArduPilot Rover / direct PWM):
  CH1  →  Steering     (1000=left, 1500=center, 2000=right)
  CH2  →  Pitch aux
  CH3  →  Throttle     (1000=stop, 1500=neutral, 2000=full)
  CH4  →  Yaw aux
  CH5..8 → Switches    (mode, arm, aux)

Modes:
  --forward          Read RC from RP3 V2 → forward to ArduPilot FC via MAVLink
  --print            Read RC from RP3 V2 → print live channels (default, no FC needed)

Requirements:
  pip install pymavlink pyserial

Usage:
  python3 mavlink_bridge.py                       # live display only
  python3 mavlink_bridge.py --forward /dev/ttyUSB0   # forward to FC on USB
  python3 mavlink_bridge.py --forward udp:127.0.0.1:14550  # forward to SITL
"""

import argparse
import os
import signal
import sys
import time
from collections import deque

# ─── Configuration ────────────────────────────────────────────────────────────
RX_PORT   = "/dev/ttyAMA0"   # RP3 V2 MAVLink output → GPIO Pin 10 (RX)
RX_BAUD   = 460_800          # ELRS MAVLink baud rate

FC_BAUD   = 57_600           # Flight controller serial baud (ArduPilot default)
FAILSAFE_THROTTLE = 900      # µs: below arming threshold → car stops
SIGNAL_TIMEOUT    = 0.5      # seconds without RC before failsafe

# ─── Terminal helpers ─────────────────────────────────────────────────────────
def bar(us, lo=1000, hi=2000, width=22):
    filled = int((us - lo) / (hi - lo) * width)
    filled = max(0, min(width, filled))
    return "[" + "#" * filled + "-" * (width - filled) + "]"

def clear():
    print("\033[H\033[J", end="")

# ─── Live display mode (no FC required) ───────────────────────────────────────
def run_display(mav_rx):
    """Read RC from RP3 V2 and display live on terminal."""
    stop = [False]
    def _sig(s, f): stop[0] = True
    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

    print("\n[Live mode] Reading RC channels from RP3 V2  (Ctrl+C to stop)\n")

    channels   = [1500] * 18
    link       = {}
    last_rc    = None
    frame_count = 0
    lq_history  = deque(maxlen=20)
    last_display = 0.0
    failsafe    = True

    while not stop[0]:
        msg = mav_rx.recv_msg()
        if msg is None:
            time.sleep(0.001)
            continue

        t = msg.get_type()
        if t == 'RC_CHANNELS_OVERRIDE':
            d = msg.to_dict()
            for i in range(1, 19):
                v = d.get(f'chan{i}_raw', 0)
                if v > 0:
                    channels[i-1] = v
            last_rc    = time.time()
            frame_count += 1

        elif t == 'RADIO_STATUS':
            d = msg.to_dict()
            link = {
                "rssi"    : d.get('rssi', 0),
                "remrssi" : d.get('remrssi', 0),
                "noise"   : d.get('noise', 0),
                "txbuf"   : d.get('txbuf', 100),
            }
            lq_history.append(link["txbuf"])

        now = time.time()
        failsafe = (last_rc is None) or (now - last_rc > SIGNAL_TIMEOUT)

        if now - last_display >= 0.1:
            last_display = now
            ch = channels

            clear()
            print("=" * 62)
            print("  RadioMaster RP3 V2 — ELRS MAVLink — Live Monitor")
            print(f"  Port: {RX_PORT}  Baud: {RX_BAUD}  Frames: {frame_count}")
            print("=" * 62)

            # Failsafe banner
            if failsafe:
                print("\n  \033[91m*** SIGNAL LOST / FAILSAFE ***\033[0m")
            else:
                sig_age = now - last_rc
                print(f"\n  \033[92mSIGNAL OK\033[0m  (age: {sig_age*1000:.0f} ms)")

            # Main axes
            axes = [(1,"Steering"), (3,"Throttle"), (2,"Pitch"), (4,"Yaw")]
            print(f"\n  {'CH':>3}  {'Name':<10}  {'Bar':^24}  {'µs':>6}  {'%':>7}")
            print("  " + "-" * 56)
            for idx, name in axes:
                us  = ch[idx-1]
                pct = round((us - 1500) / 500 * 100, 1)
                print(f"  {idx:>3}  {name:<10}  {bar(us, width=24)}  {us:6d}  {pct:+6.1f}%")

            # Switches CH5-CH8
            sw_names = {5:"SA", 6:"SB", 7:"SC", 8:"SD"}
            print(f"\n  {'CH':>3}  {'SW':<4}  {'State':<6}  {'µs':>6}")
            print("  " + "-" * 30)
            for idx in range(5, 9):
                us = ch[idx-1]
                if us < 1200:
                    state = "\033[91mLOW \033[0m"
                elif us > 1700:
                    state = "\033[92mHIGH\033[0m"
                else:
                    state = "\033[93mMID \033[0m"
                name = sw_names.get(idx, f"S{idx}")
                print(f"  {idx:>3}  {name:<4}  {state}  {us:6d}")

            # Link stats
            print("\n  LINK QUALITY:")
            if link:
                rssi  = link["rssi"]
                noise = link["noise"]
                txbuf = link["txbuf"]
                avg   = round(sum(lq_history)/len(lq_history), 1) if lq_history else "--"
                print(f"  RSSI: {rssi}   Remote RSSI: {link['remrssi']}   "
                      f"Noise: {noise}   TX buf: {txbuf}%   Avg: {avg}%")
                lq_bar_w  = 20
                lq_filled = int(txbuf / 100 * lq_bar_w)
                lq_bar    = "[" + "#" * lq_filled + "-" * (lq_bar_w - lq_filled) + "]"
                print(f"  LQ   {lq_bar}  {txbuf}%")
            else:
                print("  Waiting for RADIO_STATUS ...")

            print("\n  Ctrl+C to stop")

    signal.signal(signal.SIGINT, signal.SIG_DFL)
    print("\n\nMonitor stopped.")

# ─── Forward mode: RP3 V2 → Flight Controller ─────────────────────────────────
def run_forward(mav_rx, fc_str: str):
    """Read RC from RP3 V2 and forward RC_CHANNELS_OVERRIDE to flight controller."""
    from pymavlink import mavutil

    print(f"\n[Forward mode] RP3 V2 → {fc_str}")
    print("Connecting to flight controller ...")
    try:
        mav_fc = mavutil.mavlink_connection(fc_str, baud=FC_BAUD, source_system=255)
    except Exception as e:
        print(f"  ERROR: {e}")
        sys.exit(1)

    print("  Waiting for FC heartbeat (10 s) ...")
    mav_fc.wait_heartbeat(timeout=10)
    if mav_fc.target_system == 0:
        print("  ERROR: No heartbeat. Check FC connection and baud rate.")
        sys.exit(1)

    sys_id  = mav_fc.target_system
    comp_id = mav_fc.target_component
    print(f"  FC heartbeat OK — sysid={sys_id}  compid={comp_id}")
    print("\nForwarding RC at 50 Hz  (Ctrl+C to stop)\n")

    stop = [False]
    def _sig(s, f): stop[0] = True
    signal.signal(signal.SIGINT, _sig)

    channels   = [1500] * 18
    last_rc    = None
    tx_count   = 0
    interval   = 1.0 / 50  # 50 Hz send rate
    last_send  = 0.0
    failsafe   = True
    last_print = 0.0

    while not stop[0]:
        msg = mav_rx.recv_msg()
        if msg is not None and msg.get_type() == 'RC_CHANNELS_OVERRIDE':
            d = msg.to_dict()
            for i in range(1, 19):
                v = d.get(f'chan{i}_raw', 0)
                if v > 0:
                    channels[i-1] = v
            last_rc = time.time()

        now      = time.time()
        failsafe = (last_rc is None) or (now - last_rc > SIGNAL_TIMEOUT)

        if now - last_send >= interval:
            last_send = now
            if failsafe:
                ch_out = [1500] * 18
                ch_out[2] = FAILSAFE_THROTTLE
            else:
                ch_out = list(channels[:18])
                while len(ch_out) < 18:
                    ch_out.append(0)

            try:
                mav_fc.mav.rc_channels_override_send(
                    sys_id, comp_id, *ch_out[:18])
                tx_count += 1
            except Exception as e:
                print(f"\nSend error: {e}")

        if now - last_print >= 0.5:
            last_print = now
            ch = channels
            status = "\033[91mFAILSAFE\033[0m" if failsafe else "\033[92mOK\033[0m"
            print(f"\r  Status: {status}  Steer: {ch[0]:4d}µs  "
                  f"Thr: {ch[2]:4d}µs  Sent: {tx_count} msgs   ",
                  end="", flush=True)

    print("\n\nShutting down — sending failsafe ...")
    mav_fc.mav.rc_channels_override_send(
        sys_id, comp_id,
        1500, 1500, FAILSAFE_THROTTLE, 1500, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
    time.sleep(0.1)
    mav_fc.close()
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    print(f"Done. Forwarded {tx_count} RC override messages.")

# ─── Entry point ──────────────────────────────────────────────────────────────
def main():
    global FC_BAUD

    ap = argparse.ArgumentParser(
        description="RP3 V2 MAVLink RC reader and bridge for car control")
    ap.add_argument("--rx-port", default=RX_PORT,
                    help=f"RP3 V2 serial port (default: {RX_PORT})")
    ap.add_argument("--rx-baud", type=int, default=RX_BAUD,
                    help=f"RP3 V2 baud rate (default: {RX_BAUD})")
    ap.add_argument("--forward", metavar="FC",
                    help="Forward RC to flight controller. Examples:\n"
                         "  /dev/ttyUSB0   serial FC\n"
                         "  udp:IP:14550   UDP (SITL)\n"
                         "  tcp:IP:5760    TCP")
    ap.add_argument("--fc-baud", type=int, default=FC_BAUD,
                    help=f"FC serial baud rate (default: {FC_BAUD})")
    args = ap.parse_args()

    FC_BAUD = args.fc_baud

    print("=" * 62)
    print("  MAVLink Car Bridge — RP3 V2 ELRS MAVLink → Car Control")
    print("=" * 62)

    print(f"\n[1] Connecting to RP3 V2  {args.rx_port} @ {args.rx_baud} baud ...")

    if not os.path.exists(args.rx_port):
        print(f"  ERROR: {args.rx_port} not found")
        print("  Check: sudo bash fix_uart_permissions.sh")
        sys.exit(1)

    try:
        from pymavlink import mavutil
        mav_rx = mavutil.mavlink_connection(
            args.rx_port, baud=args.rx_baud, source_system=255)
        print(f"  OK  Connected to {args.rx_port}")
    except PermissionError:
        print("  ERROR: Permission denied — run:  sudo bash fix_uart_permissions.sh")
        sys.exit(1)
    except Exception as e:
        print(f"  ERROR: {e}")
        sys.exit(1)

    # Verify we get data
    print("[2] Verifying RC signal (3 s) ...")
    deadline = time.time() + 3.0
    got_rc = False
    while time.time() < deadline:
        msg = mav_rx.recv_msg()
        if msg and msg.get_type() == 'RC_CHANNELS_OVERRIDE':
            got_rc = True
            break

    if not got_rc:
        print("  WARNING: No RC_CHANNELS_OVERRIDE in 3 s")
        print("  Check: RadioMaster TX is ON, RP3 V2 LED is blinking green (bound)")
    else:
        print("  OK  RC signal confirmed")

    if args.forward:
        run_forward(mav_rx, args.forward)
    else:
        run_display(mav_rx)

    mav_rx.close()

if __name__ == "__main__":
    main()
