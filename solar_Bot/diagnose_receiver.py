#!/usr/bin/env python3
"""
Receiver Protocol Diagnostic
Dumps raw bytes and scans baud rates to identify what the RP3 V2 is outputting.

Usage:  python3 diagnose_receiver.py
"""
import serial
import time
import sys
import os

PORT = "/dev/ttyAMA0"

# Baud rates to try (CRSF=420000, ELRS-alt=400000, MAVLink=115200, SBUS=100000)
BAUD_RATES = [420_000, 400_000, 115_200, 100_000, 57_600]

CRSF_SYNC_BYTES = {0xC8, 0xEE, 0xEA, 0xEC, 0xE4, 0x00}

def crc8_dvbs2(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc

def hex_dump(data: bytes, cols=16):
    """Pretty hex dump with ASCII sidebar."""
    for i in range(0, len(data), cols):
        chunk = data[i:i+cols]
        hex_part  = " ".join(f"{b:02X}" for b in chunk)
        ascii_part = "".join(chr(b) if 32 <= b < 127 else "." for b in chunk)
        print(f"  {i:04X}  {hex_part:<{cols*3}}  {ascii_part}")

def count_crsf_candidates(data: bytes) -> int:
    """Count bytes that look like CRSF sync bytes."""
    return sum(1 for b in data if b in CRSF_SYNC_BYTES)

def try_crsf_parse(data: bytes) -> tuple[int, int]:
    """Returns (good_frames, bad_frames) from a raw byte buffer."""
    buf = bytearray(data)
    good = bad = 0
    while len(buf) >= 4:
        if buf[0] not in CRSF_SYNC_BYTES:
            buf.pop(0)
            continue
        fl = buf[1]
        if fl < 2 or fl > 64:
            buf.pop(0)
            continue
        total = 2 + fl
        if len(buf) < total:
            break
        fb = bytes(buf[:total])
        if crc8_dvbs2(fb[2:-1]) == fb[-1]:
            good += 1
        else:
            bad += 1
        del buf[:total]
    return good, bad

def identify_protocol(data: bytes, baud: int) -> str:
    """Heuristic: what protocol does this data look like?"""
    if len(data) < 4:
        return "too few bytes"

    good, bad = try_crsf_parse(data)
    if good > 0:
        return f"CRSF  (good={good}, bad={bad})"

    # SBUS: starts with 0x0F, 25 bytes, ends with 0x00, at 100000 baud inverted
    sbus_starts = sum(1 for i in range(len(data)-1) if data[i] == 0x0F)
    if baud == 100_000 and sbus_starts > 2:
        return f"Possible SBUS (0x0F start byte found {sbus_starts}x)"

    # DSM / SRXL2: starts with 0xA2 or 0xA5
    dsm_starts = sum(1 for b in data[:20] if b in (0xA2, 0xA5, 0x12))
    if dsm_starts > 1:
        return "Possible DSM/SRXL2"

    # MAVLink: starts with 0xFE (v1) or 0xFD (v2)
    mav_starts = sum(1 for b in data if b in (0xFE, 0xFD))
    if mav_starts > 2:
        return f"Possible MAVLink (0xFE/0xFD found {mav_starts}x)"

    sync_count = count_crsf_candidates(data)
    if sync_count > len(data) * 0.05:
        return f"CRSF-like sync bytes present ({sync_count}) but CRC fails — baud mismatch?"

    return f"Unknown  (bad CRC={bad}, sync_hits={sync_count})"

def scan_baud(baud: int, duration=1.5) -> bytes | None:
    print(f"  Trying {baud:>7} baud ... ", end="", flush=True)
    try:
        ser = serial.Serial(PORT, baud, timeout=0.2)
    except Exception as e:
        print(f"error: {e}")
        return None

    ser.reset_input_buffer()
    buf = b""
    deadline = time.time() + duration
    while time.time() < deadline:
        chunk = ser.read(ser.in_waiting or 1)
        buf += chunk
    ser.close()

    if len(buf) == 0:
        print("no data")
        return None

    proto = identify_protocol(buf, baud)
    print(f"{len(buf):5d} bytes  →  {proto}")
    return buf

# ─── Main ────────────────────────────────────────────────────────────────────
def main():
    print("=" * 60)
    print("  Receiver Protocol Diagnostic")
    print(f"  Port: {PORT}")
    print("=" * 60)

    if not os.path.exists(PORT):
        print(f"\nERROR: {PORT} not found. Run fix_uart_permissions.sh first.")
        sys.exit(1)

    print("\n[1] Baud rate scan (1.5 s per rate):")
    print("    Make sure RadioMaster TX is ON and RP3 V2 LED is blinking green\n")

    results = {}
    for baud in BAUD_RATES:
        data = scan_baud(baud)
        if data:
            results[baud] = data

    # ─── Deep dive on 420000 ─────────────────────────────────────────────────
    print("\n[2] Raw hex dump at 420000 baud (first 128 bytes):")
    try:
        ser = serial.Serial(PORT, 420_000, timeout=0.5)
        ser.reset_input_buffer()
        time.sleep(0.1)
        raw = b""
        deadline = time.time() + 2.0
        while len(raw) < 128 and time.time() < deadline:
            raw += ser.read(ser.in_waiting or 1)
        ser.close()
        if raw:
            hex_dump(raw[:128])
            print(f"\n  First 8 bytes: {' '.join(f'{b:02X}' for b in raw[:8])}")
            print(f"  Sync byte matches in first 32: "
                  f"{sum(1 for b in raw[:32] if b in CRSF_SYNC_BYTES)}")
        else:
            print("  No data received at 420000 baud")
    except Exception as e:
        print(f"  Error: {e}")

    # ─── Recommendation ──────────────────────────────────────────────────────
    print("\n[3] Diagnosis:")

    best_baud = None
    for baud in BAUD_RATES:
        if baud in results:
            g, b = try_crsf_parse(results[baud])
            if g > 0:
                best_baud = baud
                print(f"  CRSF frames found at {baud} baud — update CRSF_BAUD to {baud}")
                break

    if best_baud is None:
        data_420 = results.get(420_000, b"")
        good, bad = try_crsf_parse(data_420)
        print(f"  No baud rate yielded valid CRSF frames.")
        print()
        if bad > 0 and good == 0:
            print("  CRC errors at 420000 suggest a baud rate mismatch.")
            print("  Check:")
            print("    [ ] RadioMaster TX is powered ON (not just in standby)")
            print("    [ ] RP3 V2 LED: blinking green = bound, solid/fast = NOT bound")
            print("    [ ] In EdgeTX/OpenTX, ELRS is set to 'CRSF' output mode")
            print("    [ ] RP3 V2 is NOT in WiFi/update mode (hold button to exit)")
            print()
            print("  If RP3 V2 LED is NOT blinking green (not bound):")
            print("    → Bind it first via the RadioMaster transmitter ELRS menu")
            print()
            print("  If you changed ELRS packet rate in LUA script:")
            print("    → Some packet rates require CRSF baud 400000 not 420000")
        else:
            print("  Possible causes:")
            print("    [ ] Receiver is in SBUS output mode — check jumper/config")
            print("    [ ] Receiver not bound — LED should blink green when bound")
            print("    [ ] Wrong serial port — run: python3 test_uart_scan.py")


if __name__ == "__main__":
    main()
