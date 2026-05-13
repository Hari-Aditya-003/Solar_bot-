#!/usr/bin/env python3
"""
UART Scanner — finds which /dev/ttyAMA* is receiving data from RP3 V2
Run with transmitter ON and RP3 V2 powered and bound.

Usage:  python3 test_uart_scan.py
        sudo python3 test_uart_scan.py   (if permission errors)
"""
import glob
import time

import serial

BAUD    = 420000
TIMEOUT = 1.5   # seconds per port


def main():
    candidates = sorted(glob.glob("/dev/ttyAMA*") + glob.glob("/dev/ttyUSB*"))

    print("=" * 54)
    print("  UART Scanner — looking for RP3 V2 CRSF data")
    print(f"  Baud: {BAUD}   Timeout per port: {TIMEOUT}s")
    print("=" * 54)
    print(f"  Ports to scan: {candidates}\n")

    found = []
    for port in candidates:
        print(f"  Scanning {port} ...", end=" ", flush=True)
        try:
            ser = serial.Serial(port, BAUD, timeout=TIMEOUT)
            ser.reset_input_buffer()
            deadline = time.time() + TIMEOUT
            total = 0
            while time.time() < deadline:
                chunk = ser.read(ser.in_waiting or 1)
                total += len(chunk)
                if total >= 10:
                    break
            ser.close()
            if total >= 10:
                print(f"DATA FOUND!  ({total} bytes)  <-- connect RP3 V2 TX here")
                found.append(port)
            elif total > 0:
                print(f"partial ({total} bytes) — possibly noise")
            else:
                print("no data")
        except PermissionError:
            print("permission denied (try sudo)")
        except serial.SerialException as e:
            print(f"error: {e}")

    print()
    if found:
        print(f"  RESULT: Data detected on: {found}")
        print(f"  Update SERIAL_PORT in test_rp3v2.py to: '{found[0]}'")
    else:
        print("  RESULT: No data on any port.")
        print("  Check:")
        print("    [ ] RP3 V2 LED is blinking green (bound)")
        print("    [ ] Transmitter is powered ON")
        print("    [ ] JST SH cable is fully inserted in RPi 5 connector")
        print("    [ ] RP3 V2 TX wire is going to JST Pin 1 (RX)")
        print("    [ ] RP3 V2 has 5V power (from GPIO header Pin 2 or 4)")


if __name__ == "__main__":
    main()
