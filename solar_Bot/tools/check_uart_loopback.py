#!/usr/bin/env python3
"""
UART Loopback Test — Raspberry Pi 5 JST SH connector
Verifies that /dev/ttyAMA0 TX and RX are working.

Before running: bridge JST Pin 1 (RX) to JST Pin 3 (TX) with a short wire.
  JST Pin 1 <--wire--> JST Pin 3

Run: python3 tools/check_uart_loopback.py
"""
import os
import sys
import time

import serial

PORT  = "/dev/ttyAMA0"
BAUD  = 115200          # use safe baud for loopback


def main():
    print("=" * 50)
    print("  UART Loopback Test  —  /dev/ttyAMA0")
    print("  Bridge JST Pin 1 (RX) <--> JST Pin 3 (TX)")
    print("=" * 50)

    if not os.path.exists(PORT):
        print(f"ERROR: {PORT} not found")
        sys.exit(1)

    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
    except PermissionError:
        print("ERROR: Permission denied. Run:  sudo python3 tools/check_uart_loopback.py")
        sys.exit(1)

    test_bytes = b"\xAA\x55\x01\x02\x03\xDE\xAD\xBE\xEF"

    print(f"\nPort    : {PORT} @ {BAUD} baud")
    print(f"Sending : {test_bytes.hex(' ').upper()}")

    ser.reset_input_buffer()
    ser.write(test_bytes)
    time.sleep(0.1)
    received = ser.read(len(test_bytes))

    print(f"Received: {received.hex(' ').upper() if received else '(nothing)'}")
    print()

    if received == test_bytes:
        print("RESULT: PASS — Loopback OK, UART TX and RX are working")
        print()
        print("Next step: remove the loopback wire and connect RP3 V2:")
        print("  JST Pin 1 (RX)  <--  RP3 V2 TX")
        print("  JST Pin 2 (GND) <--  RP3 V2 GND")
    elif len(received) == 0:
        print("RESULT: FAIL — Nothing received back")
        print("  Check: loopback wire is between JST Pin 1 and Pin 3")
        print("  Check: JST SH cable is properly inserted")
        print("  Check: /boot/firmware/config.txt has  dtparam=uart0=on")
    else:
        print("RESULT: FAIL — Got wrong data back")
        print(f"  Expected: {test_bytes.hex(' ').upper()}")
        print(f"  Got     : {received.hex(' ').upper()}")
        print("  Partial match — check wiring or try lower baud rate")

    ser.close()


if __name__ == "__main__":
    main()
