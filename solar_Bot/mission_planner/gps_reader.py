"""NMEA reader for the SmartElex / u-blox NEO-M9N GNSS module.

Runs as a daemon thread.  ``get_fix()`` always returns the most recent
:class:`GPSFix` snapshot.  Auto-reconnects on serial errors.
"""

from __future__ import annotations

import logging
import os
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path

import serial  # type: ignore

try:
    import pynmea2  # type: ignore
    _HAS_PYNMEA2 = True
except ImportError:  # pragma: no cover - optional dep
    _HAS_PYNMEA2 = False

log = logging.getLogger(__name__)

GPS_AUTO_PORT = "auto"
GPS_CANDIDATE_PORTS = (
    "/dev/ttyUSB0",
    "/dev/ttyUSB1",
    "/dev/ttyACM0",
    "/dev/ttyACM1",
    "/dev/ttyAMA2",
)


@dataclass
class GPSFix:
    lat: float = 0.0
    lon: float = 0.0
    alt_m: float = 0.0
    speed_ms: float = 0.0
    heading: float = 0.0
    satellites: int = 0
    hdop: float = 99.9
    fix_quality: int = 0
    timestamp: float = field(default_factory=time.time)

    @property
    def has_fix(self) -> bool:
        return self.fix_quality > 0 and self.lat != 0.0

    @property
    def accuracy_m(self) -> float:
        return self.hdop * 2.5

    def to_dict(self) -> dict:
        return {
            "lat": round(self.lat, 8),
            "lon": round(self.lon, 8),
            "alt_m": round(self.alt_m, 1),
            "speed_ms": round(self.speed_ms, 2),
            "heading": round(self.heading, 1),
            "satellites": self.satellites,
            "hdop": round(self.hdop, 2),
            "fix_quality": self.fix_quality,
            "accuracy_m": round(self.accuracy_m, 1),
            "has_fix": self.has_fix,
            "timestamp": self.timestamp,
        }


class GPSReader(threading.Thread):
    """Background NMEA reader."""

    def __init__(
        self,
        port: str = "/dev/ttyAMA2",
        baud: int = 9600,
        on_fix: Callable[[GPSFix], None] | None = None,
    ) -> None:
        super().__init__(daemon=True, name="GPSReader")
        self.port = port
        self.baud = baud
        self.on_fix = on_fix
        self._fix = GPSFix()
        self._lock = threading.Lock()
        self._stop_evt = threading.Event()
        self.connected = False
        self.current_port = ""
        self.last_sentence_t = 0.0
        if not _HAS_PYNMEA2:
            log.warning("pynmea2 not installed; using built-in parser")

    def get_fix(self) -> GPSFix:
        with self._lock:
            f = self._fix
            return GPSFix(
                lat=f.lat, lon=f.lon, alt_m=f.alt_m,
                speed_ms=f.speed_ms, heading=f.heading,
                satellites=f.satellites, hdop=f.hdop,
                fix_quality=f.fix_quality, timestamp=f.timestamp,
            )

    def stop(self) -> None:
        self._stop_evt.set()

    # ── Serial loop ───────────────────────────────────────────────────────

    def run(self) -> None:
        while not self._stop_evt.is_set():
            port = resolve_gps_port(self.port)
            if port is None:
                self.connected = False
                log.warning("GPS device not found for port=%s — retrying in 3 s", self.port)
                time.sleep(3)
                continue
            try:
                with serial.Serial(port, self.baud, timeout=1.0) as ser:
                    log.info("GPS connected: %s @ %d", port, self.baud)
                    self.connected = True
                    self.current_port = port
                    self._read_loop(ser)
            except serial.SerialException as exc:
                self.connected = False
                log.warning("GPS error: %s — retrying in 3 s", exc)
                time.sleep(3)
            except PermissionError:
                self.connected = False
                log.error("Permission denied on %s", self.port)
                time.sleep(5)

    def _read_loop(self, ser: serial.Serial) -> None:
        buf = b""
        while not self._stop_evt.is_set():
            try:
                chunk = ser.read(ser.in_waiting or 1)
            except serial.SerialException:
                break
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line_b, buf = buf.split(b"\n", 1)
                sentence = line_b.decode("ascii", errors="replace").strip()
                if sentence.startswith("$"):
                    self.last_sentence_t = time.time()
                    if _HAS_PYNMEA2:
                        self._parse_pynmea2(sentence)
                    else:
                        self._parse_manual(sentence)
        self.connected = False

    # ── Parsers ───────────────────────────────────────────────────────────

    def _parse_pynmea2(self, sentence: str) -> None:
        try:
            msg = pynmea2.parse(sentence)
        except Exception:
            return
        with self._lock:
            if isinstance(msg, pynmea2.types.talker.GGA):
                if msg.latitude and msg.longitude:
                    self._fix.lat = float(msg.latitude)
                    self._fix.lon = float(msg.longitude)
                    self._fix.alt_m = float(msg.altitude or 0.0)
                    self._fix.satellites = int(msg.num_sats or 0)
                    self._fix.hdop = float(msg.horizontal_dil or 99.9)
                    self._fix.fix_quality = int(msg.gps_qual or 0)
                    self._fix.timestamp = time.time()
            elif isinstance(msg, pynmea2.types.talker.RMC):
                if msg.status == "A":
                    self._fix.speed_ms = float(msg.spd_over_grnd or 0) * 0.51444
                    if msg.true_course is not None:
                        self._fix.heading = float(msg.true_course)
        if self._fix.has_fix and self.on_fix:
            self.on_fix(self.get_fix())

    def _parse_manual(self, sentence: str) -> None:
        parts = sentence.split(",")
        try:
            talker = parts[0]
            if talker in ("$GPGGA", "$GNGGA", "$GLGGA"):
                if len(parts) < 10 or parts[2] == "":
                    return
                lat = _nmea_to_deg(parts[2], parts[3])
                lon = _nmea_to_deg(parts[4], parts[5])
                with self._lock:
                    self._fix.lat = lat
                    self._fix.lon = lon
                    self._fix.fix_quality = int(parts[6])
                    self._fix.satellites = int(parts[7])
                    self._fix.hdop = float(parts[8])
                    self._fix.alt_m = float(parts[9])
                    self._fix.timestamp = time.time()
                if self._fix.has_fix and self.on_fix:
                    self.on_fix(self.get_fix())
            elif talker in ("$GPRMC", "$GNRMC"):
                if len(parts) < 9 or parts[2] != "A":
                    return
                with self._lock:
                    self._fix.speed_ms = float(parts[7]) * 0.51444
                    if parts[8]:
                        self._fix.heading = float(parts[8])
        except (ValueError, IndexError):
            log.debug("Bad NMEA: %s", sentence)


def _nmea_to_deg(value: str, direction: str) -> float:
    if not value:
        return 0.0
    dot = value.index(".")
    degrees = float(value[: dot - 2])
    minutes = float(value[dot - 2:])
    dec = degrees + minutes / 60.0
    if direction in ("S", "W"):
        dec = -dec
    return dec


def resolve_gps_port(port: str | None = GPS_AUTO_PORT) -> str | None:
    """Resolve ``auto`` to the first likely USB/GPIO GPS serial port."""
    if port and port != GPS_AUTO_PORT:
        return port if os.path.exists(port) else None

    by_id = Path("/dev/serial/by-id")
    if by_id.exists():
        for link in sorted(by_id.iterdir()):
            try:
                target = str(link.resolve())
            except OSError:
                continue
            if Path(target).exists() and (
                target.startswith("/dev/ttyUSB")
                or target.startswith("/dev/ttyACM")
            ):
                return target

    for candidate in GPS_CANDIDATE_PORTS:
        if os.path.exists(candidate):
            return candidate
    return None
