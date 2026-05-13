#!/usr/bin/env python3
"""
Motor Driver Test — Solar Bot
Tests L298N + Pico 2W via UART.

Usage:
    python3 tools/check_motors.py
    python3 tools/check_motors.py --port /dev/ttyAMA4
    python3 tools/check_motors.py --speed 40
"""

import argparse
import sys
import time

import serial

PICO_PORT = "/dev/ttyAMA4"
PICO_BAUD = 115_200

def send(ser, cmd):
    ser.write((cmd + "\n").encode())

def read_status(ser, timeout=1.0):
    """Read one STATUS line from Pico. Returns parsed dict or None."""
    deadline = time.time() + timeout
    buf = b""
    while time.time() < deadline:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                txt = line.decode("ascii", "replace").strip()
                if txt.startswith("STATUS"):
                    p = txt.split()
                    if len(p) >= 9:
                        return {
                            "steer":    int(p[1]),
                            "throttle": int(p[2]),
                            "yaw":      float(p[3]),
                            "bat_mv":   int(p[5]),
                            "roll":     float(p[7]),
                            "pitch":    float(p[8]),
                            "raw":      txt,
                        }
    return None

def check(ser, label):
    """Read STATUS and print result."""
    st = read_status(ser)
    if st:
        bat = st["bat_mv"] / 1000
        print(f"    {label:<26} bat={bat:.2f}V  yaw={st['yaw']:.1f}°  "
              f"roll={st['roll']:.1f}°  pitch={st['pitch']:.1f}°")
        if bat < 9.0:
            print(f"    \033[91m⚠  Battery low ({bat:.2f}V) — motors may not spin!\033[0m")
        elif bat < 11.0:
            print(f"    \033[93m⚠  Battery {bat:.2f}V — getting low\033[0m")
        return st
    else:
        print(f"    {label:<26} \033[91mNo STATUS received — Pico not responding\033[0m")
        return None

def run_test(port, speed):
    print()
    print("=" * 58)
    print("  Solar Bot — Motor Driver Test")
    print(f"  Port: {port}   Speed: {speed}%")
    print("=" * 58)

    try:
        ser = serial.Serial(port, PICO_BAUD, timeout=0.1)
    except Exception as e:
        print(f"\n  \033[91mERROR: Cannot open {port}: {e}\033[0m")
        print("  Is Pico connected and main.py running?")
        print("  Try: mpremote connect /dev/ttyACM0 reset")
        sys.exit(1)

    time.sleep(0.5)   # let Pico settle

    print("\n[0] Pico comms check ...")
    send(ser, "STOP")
    st = check(ser, "STOP sent")
    if not st:
        print("\n  Pico is connected but not sending STATUS.")
        print("  Try resetting it: mpremote connect /dev/ttyACM0 reset\n")
        ser.close()
        sys.exit(1)

    print("\n  \033[92mPico is responding. Starting motor tests...\033[0m")
    print("  \033[93m⚠  Make sure the robot is lifted off the ground!\033[0m\n")
    input("  Press ENTER to begin, or Ctrl+C to cancel... ")

    tests = [
        # (label,             steer,     throttle,  dur_s)
        ("STOP (baseline)",       0,          0,       1.0),
        ("FORWARD slow",          0,    speed//2,     2.0),
        ("FORWARD full",          0,      speed,      2.0),
        ("REVERSE full",          0,     -speed,      2.0),
        ("STOP",                  0,          0,       0.5),
        ("STEER LEFT  (fwd)",  -speed,    speed,      2.0),
        ("STEER RIGHT (fwd)",   speed,    speed,      2.0),
        ("SPIN LEFT  (on spot)", -speed,      0,      2.0),
        ("SPIN RIGHT (on spot)",  speed,      0,      2.0),
        ("STOP (final)",          0,          0,       1.0),
    ]

    all_ok = True
    for label, steer, throttle, dur in tests:
        cmd = f"MOVE {steer} {throttle}" if (steer != 0 or throttle != 0) else "STOP"
        send(ser, cmd)
        print(f"\n[→] {label}")
        print(f"    Sending: {cmd}")
        time.sleep(dur)
        st = check(ser, "Status")
        if st is None:
            all_ok = False

    send(ser, "STOP")
    ser.close()

    print()
    print("=" * 58)
    if all_ok:
        print("  \033[92m✓ All steps completed — check motors moved as expected\033[0m")
    else:
        print("  \033[91m✗ Some STATUS reads failed — check Pico connection\033[0m")
    print()
    print("  If motors did NOT spin:")
    print("   1. Check 12V battery is connected to L298N 12V terminal")
    print("   2. Check ENA/ENB jumpers are REMOVED from L298N")
    print("   3. Check IN1/IN2/IN3/IN4 wires from Pico GP0/1/3/6")
    print("   4. Check ENA=GP2, ENB=GP7 wires to L298N")
    print("=" * 58)
    print()

if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Motor driver test")
    ap.add_argument("--port",  default=PICO_PORT)
    ap.add_argument("--speed", type=int, default=50,
                    help="Test speed 0-100% (default 50)")
    args = ap.parse_args()
    try:
        run_test(args.port, args.speed)
    except KeyboardInterrupt:
        print("\n\n  Aborted — sending STOP")
        try:
            s = serial.Serial(args.port, PICO_BAUD, timeout=1)
            s.write(b"STOP\n")
            s.close()
        except Exception:
            pass
