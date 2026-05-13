#!/usr/bin/env python3
"""Check Pico UART telemetry without running the motors."""

from __future__ import annotations

import argparse
import sys
import time

import serial

PICO_PORT = "/dev/ttyAMA4"
PICO_BAUD = 115_200


def read_status(ser: serial.Serial, timeout: float) -> dict[str, float | int | str] | None:
    deadline = time.time() + timeout
    buf = b""
    while time.time() < deadline:
        chunk = ser.read(ser.in_waiting or 1)
        if not chunk:
            continue
        buf += chunk
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            text = line.decode("ascii", "replace").strip()
            if not text.startswith("STATUS"):
                continue
            parts = text.split()
            if len(parts) < 9:
                return {"raw": text}
            return {
                "steer": int(parts[1]),
                "throttle": int(parts[2]),
                "yaw": float(parts[3]),
                "speed_cms": float(parts[4]),
                "battery_mv": int(parts[5]),
                "wp_reached": int(parts[6]),
                "roll": float(parts[7]),
                "pitch": float(parts[8]),
                "raw": text,
            }
    return None


def main() -> int:
    parser = argparse.ArgumentParser(description="Check Pico UART STATUS telemetry")
    parser.add_argument("--port", default=PICO_PORT)
    parser.add_argument("--baud", type=int, default=PICO_BAUD)
    parser.add_argument("--seconds", type=float, default=5.0)
    args = parser.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.2)
    except Exception as exc:
        print(f"ERROR: cannot open {args.port}: {exc}")
        return 2

    with ser:
        time.sleep(0.3)
        ser.write(b"STOP\n")
        status = read_status(ser, args.seconds)

    if status is None:
        print(f"No STATUS received from Pico on {args.port} @ {args.baud}.")
        print("Check Pico power, UART wiring, firmware, and shared ground.")
        return 1

    print("Pico UART OK")
    print(f"Raw: {status['raw']}")
    if "battery_mv" in status:
        print(
            "Parsed: "
            f"steer={status['steer']} throttle={status['throttle']} "
            f"yaw={status['yaw']} battery={status['battery_mv'] / 1000:.2f}V "
            f"roll={status['roll']} pitch={status['pitch']}"
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
