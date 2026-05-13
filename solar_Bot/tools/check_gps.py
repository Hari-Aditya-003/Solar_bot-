#!/usr/bin/env python3
"""Quick GPS connectivity check for USB/GPIO NMEA receivers."""

from __future__ import annotations

import argparse
import time

import serial

import _paths  # noqa: F401
from mission_planner.gps_reader import _HAS_PYNMEA2, GPSReader, resolve_gps_port


def main() -> int:
    parser = argparse.ArgumentParser(description="Check SmartElex/u-blox GPS NMEA output")
    parser.add_argument("--port", default="auto", help="Serial port, or 'auto'")
    parser.add_argument("--baud", type=int, default=9600)
    parser.add_argument("--seconds", type=float, default=10.0)
    args = parser.parse_args()

    port = resolve_gps_port(args.port)
    if port is None:
        print("GPS serial device not found.")
        print("Expected one of: /dev/ttyUSB*, /dev/ttyACM*, or /dev/ttyAMA2")
        print("Check USB data cable, GPS power LED, and `lsusb`.")
        return 2

    print(f"Opening GPS on {port} @ {args.baud} baud")
    reader = GPSReader(port=port, baud=args.baud)
    raw_count = 0
    deadline = time.time() + args.seconds

    with serial.Serial(port, args.baud, timeout=0.5) as ser:
        buf = b""
        while time.time() < deadline:
            chunk = ser.read(ser.in_waiting or 1)
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line_b, buf = buf.split(b"\n", 1)
                line = line_b.decode("ascii", errors="replace").strip()
                if not line.startswith("$"):
                    continue
                raw_count += 1
                if raw_count <= 8:
                    print(line)
                if line.startswith(("$GNGGA", "$GPGGA", "$GNRMC", "$GPRMC")):
                    if _HAS_PYNMEA2:
                        reader._parse_pynmea2(line)
                    else:
                        reader._parse_manual(line)
                    fix = reader.get_fix()
                    if fix.has_fix:
                        print(
                            f"FIX OK: lat={fix.lat:.8f} lon={fix.lon:.8f} "
                            f"sats={fix.satellites} hdop={fix.hdop:.2f}"
                        )
                        return 0

    fix = reader.get_fix()
    print(f"Read {raw_count} NMEA sentences.")
    if raw_count == 0:
        print("No NMEA data received. Try baud 38400 or check USB adapter wiring.")
        return 3
    print(
        "GPS is talking but no fix yet. Move antenna outdoors with sky view; "
        f"sats={fix.satellites}, fix_quality={fix.fix_quality}, hdop={fix.hdop:.2f}"
    )
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
