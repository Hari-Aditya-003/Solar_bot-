"""Sensor fusion — heading + position estimator.

Fuses GPS, IMU yaw and (optionally) wheel-encoder odometry into a single
``Pose`` object that the navigation loop can consume in either GPS or
non-GPS mode.

Strategy
--------
* **Heading**: complementary blend of GPS course-over-ground (when speed
  > GPS_HEADING_MIN_SPEED) and the gyro-integrated yaw reported by the
  Pico.  When GPS is unavailable we trust the gyro 100 % but accumulate
  an estimate of drift to display in the UI.

* **Position (GPS mode)**: pass the GPS fix straight through.

* **Position (non-GPS mode)**: dead-reckon from the last known anchor by
  integrating ``speed_ms`` along the fused heading, ``dx = v * sin(θ) dt``,
  ``dy = v * cos(θ) dt``.  This is "good enough" for one or two
  panel-rows; for anything longer the user must drop GPS anchors
  manually.

This module is **stateful** — the ``Pose`` you read with
:py:meth:`Estimator.pose` is the latest fused estimate.  Call
:py:meth:`Estimator.update_*` from your sensor read loops.
"""

from __future__ import annotations

import logging
import math
import threading
import time
from dataclasses import dataclass, field

from .config import NavigationConfig
from .geo import LatLon, M_PER_DEG_LAT

log = logging.getLogger(__name__)

GPS_HEADING_MIN_SPEED_MS: float = 0.30


@dataclass(frozen=True)
class Pose:
    """Best-effort estimate of where the robot is and which way it points."""

    lat: float = 0.0
    lon: float = 0.0
    heading_deg: float = 0.0       # 0 = North, clockwise
    speed_ms: float = 0.0
    has_position: bool = False
    source: str = "none"           # "gps" | "dead_reckon" | "none"
    timestamp: float = field(default_factory=time.time)

    def as_latlon(self) -> LatLon:
        return LatLon(self.lat, self.lon)

    def to_dict(self) -> dict:
        return {
            "lat": round(self.lat, 8),
            "lon": round(self.lon, 8),
            "heading": round(self.heading_deg, 1),
            "speed_ms": round(self.speed_ms, 2),
            "has_position": self.has_position,
            "source": self.source,
            "timestamp": self.timestamp,
        }


class Estimator:
    """Fuses GPS + IMU into a single :class:`Pose`."""

    def __init__(self, cfg: NavigationConfig) -> None:
        self.cfg = cfg
        self._lock = threading.Lock()
        self._pose = Pose()
        self._last_dr_time = time.time()
        self._yaw_offset = 0.0           # gyro yaw - true heading (calibration)
        self._anchor: LatLon | None = None  # last known good GPS

    # ── Sensor inputs ─────────────────────────────────────────────────────

    def update_gps(self, lat: float, lon: float, course_deg: float,
                   speed_ms: float, has_fix: bool) -> None:
        if not has_fix or (lat == 0.0 and lon == 0.0):
            return
        with self._lock:
            old_heading = self._pose.heading_deg
            new_heading = old_heading
            if speed_ms > GPS_HEADING_MIN_SPEED_MS:
                a = self.cfg.heading_filter_alpha
                new_heading = _lerp_angle(old_heading, course_deg, 1.0 - a)
            self._pose = Pose(
                lat=lat,
                lon=lon,
                heading_deg=new_heading,
                speed_ms=speed_ms,
                has_position=True,
                source="gps",
            )
            self._anchor = LatLon(lat, lon)
            self._last_dr_time = time.time()

    def update_imu(self, yaw_deg: float, speed_cms: float = 0.0) -> None:
        """Update from IMU.  ``yaw_deg`` is the gyro-integrated heading
        from the Pico (relative, drifts over time).

        We don't trust gyro yaw as a *true* heading unless GPS hasn't fixed
        anything yet.  When GPS is alive, the GPS-driven heading already
        in ``self._pose`` keeps refreshing it; when GPS dies, we fall back
        to dead reckoning starting from the last anchor.
        """
        speed_ms = speed_cms / 100.0
        with self._lock:
            now = time.time()
            dt = max(0.0, now - self._last_dr_time)
            self._last_dr_time = now

            # Heading: trust gyro if no recent GPS, otherwise just store
            # the latest gyro reading offset for diagnostics.
            if self._pose.source != "gps" or now - self._pose.timestamp > 1.0:
                heading = (yaw_deg - self._yaw_offset) % 360.0
            else:
                heading = self._pose.heading_deg

            # Position: dead-reckon if no GPS
            lat = self._pose.lat
            lon = self._pose.lon
            has_pos = self._pose.has_position
            source = self._pose.source

            if (now - self._pose.timestamp > 1.0 or source != "gps") and self._anchor:
                dx = speed_ms * math.sin(math.radians(heading)) * dt
                dy = speed_ms * math.cos(math.radians(heading)) * dt
                lat = lat + dy / M_PER_DEG_LAT
                lon = lon + dx / (M_PER_DEG_LAT *
                                  math.cos(math.radians(lat or self._anchor.lat)))
                source = "dead_reckon"
                has_pos = True

            self._pose = Pose(
                lat=lat, lon=lon,
                heading_deg=heading,
                speed_ms=speed_ms,
                has_position=has_pos,
                source=source,
            )

    def calibrate_yaw_to(self, true_heading_deg: float, gyro_yaw_deg: float) -> None:
        """Reset the gyro→true-north offset (e.g. after a known turn)."""
        with self._lock:
            self._yaw_offset = (gyro_yaw_deg - true_heading_deg) % 360.0
            log.info("Yaw calibrated: offset=%.1f°", self._yaw_offset)

    def set_anchor(self, lat: float, lon: float) -> None:
        """Force a position anchor (used to start dead-reckoning manually)."""
        with self._lock:
            self._anchor = LatLon(lat, lon)
            self._pose = Pose(
                lat=lat, lon=lon,
                heading_deg=self._pose.heading_deg,
                speed_ms=self._pose.speed_ms,
                has_position=True,
                source="dead_reckon",
            )
            self._last_dr_time = time.time()

    # ── Output ────────────────────────────────────────────────────────────

    def pose(self) -> Pose:
        with self._lock:
            return self._pose


# ─── Helpers ──────────────────────────────────────────────────────────────────


def _lerp_angle(a: float, b: float, t: float) -> float:
    """Linear interpolation between two angles, taking the short way around."""
    diff = ((b - a + 540.0) % 360.0) - 180.0
    return (a + diff * t) % 360.0
