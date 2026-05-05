#!/usr/bin/env python3
"""
RadioMaster RP3 V2 Receiver Test Script
Hardware: Raspberry Pi 5  <-->  RP3 V2 (ELRS)

Wiring — RPi 5 JST SH connector (3-pin, board bottom edge):
  JST Pin 1 (UART RX) <--  RP3 V2 TX
  JST Pin 2 (GND)     <--  RP3 V2 GND
  JST Pin 3 (UART TX) -->  RP3 V2 RX   (optional, for telemetry)
  5V power:  GPIO header Pin 2 or Pin 4 -->  RP3 V2 5V

RPi 5 GPIO header (Pin 10 RX) = /dev/ttyAMA0
If permission denied, run once:  sudo bash fix_uart_permissions.sh

Protocol auto-detected: CRSF (420000 baud) or MAVLink (115200 baud)
"""

import serial
import time
import sys
import os
import signal
from collections import deque
SERIAL_PORT   = "/dev/ttyAMA0"   # GPIO Pin 10 (RX) on RPi 5 40-pin header

# ─── Configuration ────────────────────────────────────────────────────────────
# RPi 5: GPIO header Pin 10 (RX) → ttyAMA0
BAUD_RATE     = 420000           # CRSF default; script auto-retries at 115200 for MAVLink
TIMEOUT_SEC   = 3.0              # read timeout
TEST_DURATION = 30               # seconds to run live monitor (0 = run forever)

# CRSF frame types
CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16
CRSF_FRAMETYPE_LINK_STATISTICS    = 0x14
CRSF_FRAMETYPE_LINK_STATISTICS_RX = 0x1C
CRSF_FRAMETYPE_BATTERY_SENSOR     = 0x08

# CRSF device address bytes that start a valid frame
CRSF_SYNC_BYTES = {0xC8, 0xEE, 0xEA, 0xEC, 0xE4, 0x00}

# Channel value range from CRSF
CRSF_CHANNEL_MIN    = 172
CRSF_CHANNEL_CENTER = 992
CRSF_CHANNEL_MAX    = 1811

# Map CRSF 172-1811 → microseconds 1000-2000
def crsf_to_us(val):
    return int(1000 + (val - CRSF_CHANNEL_MIN) / (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN) * 1000)

# Map CRSF 172-1811 → percentage -100 to +100
def crsf_to_pct(val):
    return round((val - CRSF_CHANNEL_CENTER) / (CRSF_CHANNEL_MAX - CRSF_CHANNEL_CENTER) * 100, 1)

# Switch logic: 3-position or 2-position
def switch_label(val):
    us = crsf_to_us(val)
    if us < 1200:
        return "LOW  "
    elif us > 1700:
        return "HIGH "
    else:
        return "MID  "

# ─── CRC-8 DVB-S2 ─────────────────────────────────────────────────────────────
def crc8_dvbs2(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0xD5) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc

# ─── CRSF RC Channel Unpack (16 channels × 11 bits = 176 bits = 22 bytes) ────
def unpack_rc_channels(payload: bytes):
    if len(payload) < 22:
        return None
    channels = [0] * 16
    bit_offset = 0
    for i in range(16):
        byte_offset = bit_offset >> 3
        bit_in_byte = bit_offset & 7
        raw = (payload[byte_offset]
               | (payload[byte_offset + 1] << 8)
               | (payload[byte_offset + 2] << 16))
        channels[i] = (raw >> bit_in_byte) & 0x7FF
        bit_offset += 11
    return channels

# ─── Link Statistics ──────────────────────────────────────────────────────────
def unpack_link_stats(payload: bytes):
    if len(payload) < 10:
        return None
    return {
        "rssi_ant1"    : -payload[0],        # dBm
        "rssi_ant2"    : -payload[1],        # dBm
        "link_quality" : payload[2],         # 0-100 %
        "snr"          : payload[3] - 128 if payload[3] > 127 else payload[3],
        "active_ant"   : payload[4],
        "rf_mode"      : payload[5],
        "tx_power_mw"  : payload[6],
        "dl_rssi"      : -payload[7],
        "dl_lq"        : payload[8],
        "dl_snr"       : payload[9] - 128 if payload[9] > 127 else payload[9],
    }

# ─── CRSF Frame Parser ────────────────────────────────────────────────────────
class CRSFParser:
    def __init__(self):
        self.buf          = bytearray()
        self.frames_ok    = 0
        self.frames_bad   = 0
        self.channels     = [CRSF_CHANNEL_CENTER] * 16
        self.link_stats   = {}
        self.last_rc_time = None

    def feed(self, data: bytes):
        """Feed raw bytes; returns list of parsed frame dicts."""
        self.buf.extend(data)
        frames = []
        while len(self.buf) >= 4:
            # Search for a valid sync byte
            if self.buf[0] not in CRSF_SYNC_BYTES:
                self.buf.pop(0)
                continue
            frame_len = self.buf[1]
            # Sanity check: CRSF max payload is ~64 bytes
            if frame_len < 2 or frame_len > 64:
                self.buf.pop(0)
                continue
            # Full frame = sync(1) + len(1) + payload(frame_len)
            total = 2 + frame_len
            if len(self.buf) < total:
                break   # wait for more bytes
            frame_bytes = bytes(self.buf[:total])
            # CRC covers type + payload (not sync/len/crc itself)
            crc_data    = frame_bytes[2:-1]   # type + payload
            expected_crc = frame_bytes[-1]
            calc_crc    = crc8_dvbs2(crc_data)
            if calc_crc != expected_crc:
                self.frames_bad += 1
                self.buf.pop(0)
                continue
            # Good frame
            self.frames_ok += 1
            ftype   = frame_bytes[2]
            payload = frame_bytes[3:-1]
            del self.buf[:total]
            frame = {"type": ftype, "payload": payload}
            # Decode known types
            if ftype == CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
                ch = unpack_rc_channels(payload)
                if ch:
                    self.channels     = ch
                    self.last_rc_time = time.time()
                    frame["channels"] = ch
            elif ftype in (CRSF_FRAMETYPE_LINK_STATISTICS,
                           CRSF_FRAMETYPE_LINK_STATISTICS_RX):
                ls = unpack_link_stats(payload)
                if ls:
                    self.link_stats   = ls
                    frame["link"]     = ls
            frames.append(frame)
        return frames

# ─── Terminal helpers ─────────────────────────────────────────────────────────
def bar(val, minv=CRSF_CHANNEL_MIN, maxv=CRSF_CHANNEL_MAX, width=20):
    filled = int((val - minv) / (maxv - minv) * width)
    filled = max(0, min(width, filled))
    return "[" + "#" * filled + "-" * (width - filled) + "]"

def clear():
    print("\033[H\033[J", end="")

def throttle_warning(us):
    if us < 1050:
        return " <<< DISARMED / MIN"
    elif us > 1900:
        return " <<< HIGH THROTTLE!"
    return ""

# ─── Test 1: Port open ────────────────────────────────────────────────────────
def test_connectivity():
    print("=" * 60)
    print("  RadioMaster RP3 V2  --  Connectivity Test")
    print("  Raspberry Pi 5  |  CRSF @ 420000 baud")
    print("=" * 60)
    print(f"\n[1] Opening {SERIAL_PORT} ({BAUD_RATE} baud 8N1) ...")

    if not os.path.exists(SERIAL_PORT):
        print(f"  ERROR: {SERIAL_PORT} not found.")
        print("  Check: raspi-config > Interface Options > Serial Port")
        print("         enable_uart=1 in /boot/firmware/config.txt")
        sys.exit(1)

    try:
        ser = serial.Serial(
            port       = SERIAL_PORT,
            baudrate   = BAUD_RATE,
            bytesize   = serial.EIGHTBITS,
            parity     = serial.PARITY_NONE,
            stopbits   = serial.STOPBITS_ONE,
            timeout    = TIMEOUT_SEC,
        )
        print(f"  OK  Port opened: {ser.name}")
        print(f"      Baud: {ser.baudrate}  Bytesize: {ser.bytesize}  "
              f"Parity: {ser.parity}  Stopbits: {ser.stopbits}")
        return ser
    except PermissionError:
        print(f"  ERROR: Permission denied on {SERIAL_PORT}")
        print("  Fix (run once, then try again):")
        print(f"    sudo chown root:dialout {SERIAL_PORT}")
        print(f"    sudo chmod 660 {SERIAL_PORT}")
        print("  Or run with: sudo python3 test_rp3v2.py")
        sys.exit(1)
    except serial.SerialException as e:
        print(f"  ERROR: {e}")
        print("  Hint: sudo usermod -aG dialout $USER  then re-login")
        sys.exit(1)

# ─── Test 2: Raw bytes check ──────────────────────────────────────────────────
def test_raw_bytes(ser):
    print("\n[2] Checking raw bytes from receiver (2 seconds) ...")
    ser.reset_input_buffer()
    time.sleep(0.1)
    deadline = time.time() + 2.0
    total = 0
    while time.time() < deadline:
        chunk = ser.read(ser.in_waiting or 1)
        total += len(chunk)
    if total == 0:
        print("  WARNING: No bytes received!")
        print()
        print("  Wiring check (JST SH connector on RPi 5 board edge):")
        print("    JST Pin 1 (UART RX)  <--  RP3 V2 TX  (white/signal wire)")
        print("    JST Pin 2 (GND)      <--  RP3 V2 GND")
        print("    JST Pin 3 (UART TX)  -->  RP3 V2 RX  (optional / telemetry)")
        print("    5V power for RP3 V2: GPIO header Pin 2 or Pin 4  (NOT JST SH)")
        print()
        print("  Other checks:")
        print("    [ ] RP3 V2 LED blinking green? (bound)  solid/off = not bound")
        print("    [ ] Transmitter is powered ON and within range")
        print("    [ ] TX→RX not swapped (most common mistake)")
        print()
        print("  Loopback test (to verify UART hardware):")
        print("    Short JST Pin 1 to JST Pin 3 with a wire,")
        print("    then run:  python3 test_loopback.py")
        return False
    print(f"  OK  Received {total} bytes in 2 s  (~{total//2} bytes/s)")
    return True

# ─── Test 3: CRSF frame sync ─────────────────────────────────────────────────
def test_frame_sync(ser, parser):
    print("\n[3] Waiting for valid CRSF frame (up to 5 seconds) ...")
    ser.reset_input_buffer()
    deadline = time.time() + 5.0
    while time.time() < deadline:
        raw = ser.read(ser.in_waiting or 64)
        if raw:
            frames = parser.feed(raw)
            if frames:
                rc_frames = [f for f in frames if f["type"] == CRSF_FRAMETYPE_RC_CHANNELS_PACKED]
                if rc_frames:
                    print(f"  OK  Got RC_CHANNELS frame! "
                          f"(good={parser.frames_ok}, bad={parser.frames_bad})")
                    return True
    print(f"  FAIL: No RC_CHANNELS frame in 5 s "
          f"(good={parser.frames_ok}, bad={parser.frames_bad})")
    if parser.frames_bad > 0:
        print("  Hint: CRC errors detected — check baud rate (should be 420000)")
        print("        or wrong wire (TX→TX instead of TX→RX)")
    elif parser.frames_ok == 0:
        print("  Hint: Receiver may not be bound or transmitter is off")
    return False

# ─── Live monitor ─────────────────────────────────────────────────────────────
def live_monitor(ser, parser, duration=TEST_DURATION):
    lq_history = deque(maxlen=50)
    stop        = [False]

    def _sig(s, f):
        stop[0] = True
    signal.signal(signal.SIGINT, _sig)

    print("\n[4] Live channel monitor — press Ctrl+C to stop")
    if duration > 0:
        print(f"    (auto-stop after {duration} s)\n")
    time.sleep(0.5)

    deadline     = time.time() + duration if duration > 0 else float("inf")
    last_display = 0
    frame_count  = 0

    while time.time() < deadline and not stop[0]:
        raw = ser.read(ser.in_waiting or 128)
        if raw:
            frames = parser.feed(raw)
            for f in frames:
                if f["type"] == CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
                    frame_count += 1
                if "link" in f:
                    lq_history.append(f["link"]["link_quality"])

        now = time.time()
        if now - last_display >= 0.1:   # refresh ~10 Hz
            last_display = now
            ch = parser.channels
            ls = parser.link_stats
            avg_lq = round(sum(lq_history) / len(lq_history), 1) if lq_history else "--"

            clear()
            print("=" * 62)
            print("  RadioMaster RP3 V2 — Live Signal Monitor")
            print(f"  Port: {SERIAL_PORT}   Frames: {frame_count}   "
                  f"Bad CRC: {parser.frames_bad}")
            print("=" * 62)

            # ── Throttle (CH3) ────────────────────────────────────────────
            thr_raw = ch[2]
            thr_us  = crsf_to_us(thr_raw)
            thr_pct = crsf_to_pct(thr_raw)
            print(f"\n  THROTTLE  CH3  {bar(thr_raw, width=30)}  "
                  f"{thr_us:4d} µs  {thr_pct:+6.1f}%{throttle_warning(thr_us)}")

            # ── Collective/Pitch axes (CH1 Roll, CH2 Pitch, CH4 Yaw) ─────
            print(f"\n  {'CH':>3}  {'Name':<10}  {'Bar':^22}  {'µs':>6}  {'%':>7}")
            print("  " + "-" * 56)
            axes = [(1,"Roll"), (2,"Pitch"), (4,"Yaw")]
            for idx, name in axes:
                raw = ch[idx - 1]
                us  = crsf_to_us(raw)
                pct = crsf_to_pct(raw)
                print(f"  {idx:>3}  {name:<10}  {bar(raw, width=22)}  {us:6d}  {pct:+6.1f}%")

            # ── Switches (CH5-CH12) ───────────────────────────────────────
            print(f"\n  {'CH':>3}  {'Switch':<8}  {'State':<6}  {'µs':>6}  {''}")
            print("  " + "-" * 40)
            sw_names = {5:"SA", 6:"SB", 7:"SC", 8:"SD",
                        9:"SE", 10:"SF", 11:"SG", 12:"SH"}
            for idx in range(5, 13):
                raw  = ch[idx - 1]
                us   = crsf_to_us(raw)
                lbl  = switch_label(raw)
                name = sw_names.get(idx, f"S{idx}")
                state_color = ""
                if lbl.strip() == "HIGH":
                    state_color = "\033[92m"  # green
                elif lbl.strip() == "LOW":
                    state_color = "\033[91m"  # red
                else:
                    state_color = "\033[93m"  # yellow
                print(f"  {idx:>3}  {name:<8}  "
                      f"{state_color}{lbl}\033[0m  {us:6d} µs")

            # ── All 16 raw values ─────────────────────────────────────────
            raw_line = "  RAW: " + "  ".join(
                f"CH{i+1}:{ch[i]:4d}" for i in range(16))
            # Split into two lines of 8
            print("\n  RAW CRSF values (172=min  992=ctr  1811=max):")
            print("  " + "  ".join(f"CH{i+1:02d}:{ch[i]:4d}" for i in range(8)))
            print("  " + "  ".join(f"CH{i+1:02d}:{ch[i]:4d}" for i in range(8, 16)))

            # ── Link quality ──────────────────────────────────────────────
            print("\n  LINK STATISTICS:")
            if ls:
                rssi = ls.get("rssi_ant1", "--")
                lq   = ls.get("link_quality", "--")
                snr  = ls.get("snr", "--")
                print(f"  RSSI: {rssi} dBm   LQ: {lq}%   SNR: {snr} dB   "
                      f"Avg LQ: {avg_lq}%")
                if isinstance(lq, int):
                    lq_bar = "[" + "#" * (lq // 5) + "-" * (20 - lq // 5) + "]"
                    print(f"  LQ   {lq_bar}  {lq}%")
            else:
                print(f"  Waiting for link statistics...  Avg LQ: {avg_lq}%")

            # ── Status ────────────────────────────────────────────────────
            if parser.last_rc_time:
                age = time.time() - parser.last_rc_time
                if age > 0.5:
                    print(f"\n  *** SIGNAL LOST — last frame {age:.1f}s ago ***")
                else:
                    print(f"\n  Status: SIGNAL OK  ({frame_count} frames received)")
            else:
                print(f"\n  Status: Waiting for RC frames...")

            remaining = deadline - time.time()
            if duration > 0:
                print(f"  Auto-stop in: {remaining:.0f}s   Ctrl+C to stop early")
            else:
                print(f"  Ctrl+C to stop")

    signal.signal(signal.SIGINT, signal.SIG_DFL)
    print("\n\nMonitor stopped.")
    return frame_count

# ─── Test Summary ─────────────────────────────────────────────────────────────
def print_summary(parser, frame_count):
    ch = parser.channels
    ls = parser.link_stats

    print("\n" + "=" * 60)
    print("  TEST SUMMARY")
    print("=" * 60)
    print(f"  Total RC frames  : {frame_count}")
    print(f"  CRC errors       : {parser.frames_bad}")
    print(f"  Good frames      : {parser.frames_ok}")

    print("\n  Final channel snapshot:")
    print(f"  Throttle CH3 : {crsf_to_us(ch[2])} µs  "
          f"({crsf_to_pct(ch[2]):+.1f}%)")

    any_active = False
    for idx in range(5, 13):
        raw = ch[idx - 1]
        us  = crsf_to_us(raw)
        if abs(us - 1500) > 100:
            any_active = True
        print(f"  Switch  CH{idx:<2}  : {us} µs  [{switch_label(raw).strip()}]")

    print()
    if frame_count == 0:
        print("  RESULT: FAIL — No RC frames received")
        print("          Check wiring, binding, and transmitter power")
    elif parser.frames_bad > parser.frames_ok:
        print("  RESULT: FAIL — Too many CRC errors")
        print("          Check baud rate (420000) and TX/RX wiring")
    else:
        print("  RESULT: PASS — Receiver is working correctly")

    if ls:
        print(f"\n  Link Quality  : {ls.get('link_quality', '--')}%")
        print(f"  RSSI          : {ls.get('rssi_ant1', '--')} dBm")
        print(f"  SNR           : {ls.get('snr', '--')} dB")
    print("=" * 60)

# ─── Main ─────────────────────────────────────────────────────────────────────
def main():
    print()
    ser    = test_connectivity()
    parser = CRSFParser()

    has_bytes = test_raw_bytes(ser)
    if not has_bytes:
        print("\nAborting — no data on serial port.")
        ser.close()
        sys.exit(1)

    has_frames = test_frame_sync(ser, parser)
    if not has_frames:
        print("\nAborting — could not sync CRSF frames.")
        ser.close()
        sys.exit(1)

    print("\nAll basic checks passed.  Starting live monitor...\n")
    time.sleep(1)

    frame_count = live_monitor(ser, parser, duration=TEST_DURATION)
    print_summary(parser, frame_count)

    ser.close()
    print(f"\nPort {SERIAL_PORT} closed.\n")

if __name__ == "__main__":
    main()
