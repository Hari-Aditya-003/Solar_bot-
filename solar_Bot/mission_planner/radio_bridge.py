"""Always-on MAVLink radio reader for the RP3 V2 receiver."""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass, field

log = logging.getLogger(__name__)

try:
    from pymavlink import mavutil  # type: ignore
except ImportError:  # pragma: no cover - optional dependency
    mavutil = None

RC_TIMEOUT_S = 0.5
RC_DEADBAND_US = 40
RC_ACTIVE_US = 60


def _map_axis(us: int, *, center: int = 1500, invert: bool = False) -> int:
    delta = us - center
    if abs(delta) <= RC_DEADBAND_US:
        return 0
    pct = round(delta / 500 * 100)
    if invert:
        pct *= -1
    return max(-100, min(100, pct))


@dataclass
class RadioStatus:
    connected: bool = False
    last_seen: float = 0.0
    channels: list[int] = field(default_factory=lambda: [1500] * 8)
    rssi: int = 0
    remrssi: int = 0
    noise: int = 0
    txbuf: int = 0
    steer_channel: int = 4
    throttle_channel: int = 2
    steer_invert: bool = False
    throttle_invert: bool = False
    switch_high_us: int = 1700

    def channel_us(self, channel: int, default: int = 1500) -> int:
        if 1 <= channel <= len(self.channels):
            return int(self.channels[channel - 1])
        return default

    @property
    def signal_ok(self) -> bool:
        return self.connected and (time.time() - self.last_seen) <= RC_TIMEOUT_S

    @property
    def steer_pct(self) -> int:
        return _map_axis(
            self.channel_us(self.steer_channel),
            invert=self.steer_invert,
        )

    @property
    def throttle_pct(self) -> int:
        return _map_axis(
            self.channel_us(self.throttle_channel),
            invert=self.throttle_invert,
        )

    def switch_high(self, channel: int) -> bool:
        return self.channel_us(channel) >= self.switch_high_us

    @property
    def control_active(self) -> bool:
        return self.signal_ok and (
            abs(self.steer_pct) >= round(RC_ACTIVE_US / 500 * 100)
            or abs(self.throttle_pct) >= round(RC_ACTIVE_US / 500 * 100)
        )

    def to_dict(self) -> dict:
        return {
            "connected": self.connected,
            "signal_ok": self.signal_ok,
            "last_seen": self.last_seen,
            "channels": list(self.channels),
            "steer_pct": self.steer_pct,
            "throttle_pct": self.throttle_pct,
            "control_active": self.control_active,
            "rssi": self.rssi,
            "remrssi": self.remrssi,
            "noise": self.noise,
            "txbuf": self.txbuf,
            "switches": {
                f"ch{i}": self.channel_us(i) for i in range(5, 9)
            },
        }


class RadioBridge(threading.Thread):
    """Background reader for RP3 V2 MAVLink RC data."""

    def __init__(
        self,
        port: str = "/dev/ttyAMA0",
        baud: int = 460_800,
        *,
        steer_channel: int = 4,
        throttle_channel: int = 2,
        steer_invert: bool = False,
        throttle_invert: bool = False,
        switch_high_us: int = 1700,
    ) -> None:
        super().__init__(daemon=True, name="RadioBridge")
        self.port = port
        self.baud = baud
        self._lock = threading.Lock()
        self._stop_evt = threading.Event()
        self._status = RadioStatus(
            steer_channel=steer_channel,
            throttle_channel=throttle_channel,
            steer_invert=steer_invert,
            throttle_invert=throttle_invert,
            switch_high_us=switch_high_us,
        )

    def get_status(self) -> RadioStatus:
        with self._lock:
            s = self._status
            return RadioStatus(
                connected=s.connected,
                last_seen=s.last_seen,
                channels=list(s.channels),
                rssi=s.rssi,
                remrssi=s.remrssi,
                noise=s.noise,
                txbuf=s.txbuf,
                steer_channel=s.steer_channel,
                throttle_channel=s.throttle_channel,
                steer_invert=s.steer_invert,
                throttle_invert=s.throttle_invert,
                switch_high_us=s.switch_high_us,
            )

    def stop(self) -> None:
        self._stop_evt.set()

    def run(self) -> None:
        if mavutil is None:
            log.warning("pymavlink not installed; radio receiver disabled")
            return

        while not self._stop_evt.is_set():
            try:
                mav = mavutil.mavlink_connection(
                    self.port,
                    baud=self.baud,
                    source_system=255,
                )
                log.info("Radio connected: %s @ %d", self.port, self.baud)
                self._read_loop(mav)
            except Exception as exc:  # pragma: no cover - serial failures are environment-specific
                log.warning("Radio link error: %s — retrying in 3 s", exc)
                with self._lock:
                    self._status.connected = False
                time.sleep(3)

    def _read_loop(self, mav) -> None:
        while not self._stop_evt.is_set():
            msg = mav.recv_msg()
            now = time.time()
            if msg is None:
                with self._lock:
                    if self._status.connected and now - self._status.last_seen > RC_TIMEOUT_S:
                        self._status.connected = False
                time.sleep(0.01)
                continue

            msg_type = msg.get_type()
            if msg_type == "RC_CHANNELS_OVERRIDE":
                raw = msg.to_dict()
                channels = []
                for i in range(1, 9):
                    value = int(raw.get(f"chan{i}_raw", 1500) or 1500)
                    channels.append(value)
                with self._lock:
                    self._status.channels = channels
                    self._status.last_seen = now
                    self._status.connected = True
            elif msg_type == "RADIO_STATUS":
                raw = msg.to_dict()
                with self._lock:
                    self._status.rssi = int(raw.get("rssi", 0) or 0)
                    self._status.remrssi = int(raw.get("remrssi", 0) or 0)
                    self._status.noise = int(raw.get("noise", 0) or 0)
                    self._status.txbuf = int(raw.get("txbuf", 0) or 0)
