"""Bidirectional link between the Pi and the Pico 2 W motor controller.

Transports
----------
``uart`` Serial (recommended): /dev/ttyAMA4 @ 115200, newline-terminated ASCII.

``udp``  WiFi: Pi creates a hotspot and sends/receives UDP datagrams.

Wire protocol (RPi → Pico)
--------------------------
::

    STOP
    MOVE <steer -100..100> <throttle 0..100>
    WAYPOINT <lat> <lon>
    MISSION_START | MISSION_PAUSE | MISSION_ABORT

Wire protocol (Pico → RPi, 10 Hz)
---------------------------------
::

    STATUS <lat> <lon> <yaw> <speed_cms> <battery_mv> <wp_reached> <roll> <pitch>
"""

from __future__ import annotations

import logging
import queue
import socket
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Optional

import serial  # type: ignore

log = logging.getLogger(__name__)


# ─── Status dataclass ─────────────────────────────────────────────────────────


@dataclass
class RobotStatus:
    lat: float = 0.0
    lon: float = 0.0
    heading: float = 0.0          # gyro-integrated yaw (relative)
    speed_cms: float = 0.0
    battery_mv: int = 0
    wp_reached: bool = False
    connected: bool = False
    last_seen: float = field(default_factory=time.time)
    roll: float = 0.0
    pitch: float = 0.0
    battery_full_mv: int = 12_600
    battery_empty_mv: int = 9_000

    @property
    def battery_pct(self) -> int:
        if self.battery_mv == 0:
            return 0
        span = max(1, self.battery_full_mv - self.battery_empty_mv)
        pct = (self.battery_mv - self.battery_empty_mv) / span * 100.0
        return max(0, min(100, int(pct)))

    @property
    def tilt_ok(self) -> bool:
        return abs(self.roll) < 30.0 and abs(self.pitch) < 30.0

    def to_dict(self) -> dict:
        return {
            "lat": self.lat,
            "lon": self.lon,
            "heading": self.heading,
            "speed_cms": round(self.speed_cms, 1),
            "battery_mv": self.battery_mv,
            "battery_pct": self.battery_pct,
            "wp_reached": self.wp_reached,
            "connected": self.connected,
            "roll": round(self.roll, 1),
            "pitch": round(self.pitch, 1),
            "tilt_ok": self.tilt_ok,
            "last_seen": self.last_seen,
        }


# ─── Bridge ────────────────────────────────────────────────────────────────────


class RobotBridge(threading.Thread):
    """Daemon thread maintaining the link to the Pico."""

    def __init__(
        self,
        transport: str = "uart",
        port: str = "/dev/ttyAMA4",
        baud: int = 115_200,
        udp_host: str = "192.168.4.1",
        udp_port: int = 5005,
        on_status: Optional[Callable[[RobotStatus], None]] = None,
        battery_full_mv: int = 12_600,
        battery_empty_mv: int = 9_000,
    ) -> None:
        super().__init__(daemon=True, name="RobotBridge")
        self.transport = transport
        self.port = port
        self.baud = baud
        self.udp_host = udp_host
        self.udp_port = udp_port
        self.on_status = on_status

        self._status = RobotStatus(
            battery_full_mv=battery_full_mv,
            battery_empty_mv=battery_empty_mv,
        )
        self._lock = threading.Lock()
        self._cmd_q: queue.Queue[str] = queue.Queue()
        self._stop_evt = threading.Event()

    # ── Public API ────────────────────────────────────────────────────────

    def send(self, cmd: str) -> None:
        self._cmd_q.put(cmd.strip() + "\n")

    def stop_robot(self) -> None:
        self.send("STOP")

    def move(self, steer: int, throttle: int) -> None:
        steer = max(-100, min(100, int(steer)))
        throttle = max(0, min(100, int(throttle)))
        self.send(f"MOVE {steer} {throttle}")

    def set_waypoint(self, lat: float, lon: float) -> None:
        self.send(f"WAYPOINT {lat:.8f} {lon:.8f}")

    def mission_start(self) -> None:
        self.send("MISSION_START")

    def mission_pause(self) -> None:
        self.send("MISSION_PAUSE")

    def mission_abort(self) -> None:
        self.send("MISSION_ABORT")
        self.send("STOP")

    def get_status(self) -> RobotStatus:
        with self._lock:
            s = self._status
            return RobotStatus(
                lat=s.lat, lon=s.lon, heading=s.heading,
                speed_cms=s.speed_cms, battery_mv=s.battery_mv,
                wp_reached=s.wp_reached, connected=s.connected,
                last_seen=s.last_seen, roll=s.roll, pitch=s.pitch,
                battery_full_mv=s.battery_full_mv,
                battery_empty_mv=s.battery_empty_mv,
            )

    def stop(self) -> None:
        self._stop_evt.set()

    # ── Thread entry ──────────────────────────────────────────────────────

    def run(self) -> None:
        if self.transport == "uart":
            self._run_uart()
        else:
            self._run_udp()

    # ── UART ──────────────────────────────────────────────────────────────

    def _run_uart(self) -> None:
        while not self._stop_evt.is_set():
            try:
                with serial.Serial(self.port, self.baud, timeout=0.05) as ser:
                    log.info("UART connected: %s @ %d", self.port, self.baud)
                    with self._lock:
                        self._status.connected = True
                    self._uart_loop(ser)
            except serial.SerialException as exc:
                with self._lock:
                    self._status.connected = False
                log.warning("UART error: %s — retrying in 3 s", exc)
                time.sleep(3)

    def _uart_loop(self, ser: serial.Serial) -> None:
        buf = b""
        while not self._stop_evt.is_set():
            while not self._cmd_q.empty():
                try:
                    ser.write(self._cmd_q.get_nowait().encode())
                except Exception:  # pragma: no cover
                    log.exception("UART write failed")
            chunk = ser.read(ser.in_waiting or 1)
            if chunk:
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    self._parse_status(line.decode("ascii", errors="replace").strip())
        with self._lock:
            self._status.connected = False

    # ── UDP ───────────────────────────────────────────────────────────────

    def _run_udp(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(0.5)
        try:
            sock.bind(("0.0.0.0", self.udp_port + 1))
        except OSError as exc:
            log.error("UDP bind failed: %s", exc)
            return
        log.info("UDP ready — Pico at %s:%d", self.udp_host, self.udp_port)
        while not self._stop_evt.is_set():
            while not self._cmd_q.empty():
                try:
                    sock.sendto(self._cmd_q.get_nowait().encode(),
                                (self.udp_host, self.udp_port))
                except Exception:  # pragma: no cover
                    log.exception("UDP send failed")
            try:
                data, _ = sock.recvfrom(256)
                self._parse_status(data.decode("ascii", errors="replace").strip())
            except socket.timeout:
                with self._lock:
                    if time.time() - self._status.last_seen > 3.0:
                        self._status.connected = False
        sock.close()

    # ── Status parser ─────────────────────────────────────────────────────

    def _parse_status(self, line: str) -> None:
        if not line.startswith("STATUS"):
            return
        parts = line.split()
        if len(parts) < 7:
            return
        try:
            with self._lock:
                self._status.lat = float(parts[1])
                self._status.lon = float(parts[2])
                self._status.heading = float(parts[3])
                self._status.speed_cms = float(parts[4])
                self._status.battery_mv = int(parts[5])
                self._status.wp_reached = parts[6] == "1"
                if len(parts) > 7:
                    self._status.roll = float(parts[7])
                if len(parts) > 8:
                    self._status.pitch = float(parts[8])
                self._status.last_seen = time.time()
                self._status.connected = True
            if self.on_status:
                self.on_status(self.get_status())
        except (ValueError, IndexError):
            log.debug("Malformed STATUS: %s", line)
