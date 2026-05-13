"""Safety subsystem.

Centralises every check that can stop the robot before it hits something or
goes out of bounds.  All checks are pure functions of the current state, so
they are easy to unit-test.

Hard stops (return :class:`SafetyVerdict.STOP`):
    * Inside ``geofence_buffer`` of polygon edge → STOP
    * Tilt above ``tilt_limit_deg``                → STOP
    * No STATUS heard from Pico for > comms_timeout → STOP
    * E-stop button pressed                         → STOP
    * Battery < ``battery_critical_pct``            → STOP

Soft warnings (emit a warning event but keep moving):
    * Battery < ``battery_low_pct``
    * GPS fix older than ``gps_timeout_s`` (only in GPS mode)
"""

from __future__ import annotations

import enum
import logging
import threading
import time
from dataclasses import dataclass

from .config import SafetyConfig
from .geo import LatLon, Point, latlon_to_xy, point_in_polygon, shrink_polygon

log = logging.getLogger(__name__)


class SafetyVerdict(enum.Enum):
    OK = "ok"
    WARN = "warn"
    STOP = "stop"


@dataclass(frozen=True)
class SafetyReport:
    verdict: SafetyVerdict
    reasons: tuple[str, ...] = ()

    def to_dict(self) -> dict:
        return {"verdict": self.verdict.value, "reasons": list(self.reasons)}


class SafetyMonitor:
    """Thread-safe holder of the current safety state."""

    def __init__(self, cfg: SafetyConfig) -> None:
        self.cfg = cfg
        self._lock = threading.Lock()
        self._estop = False
        self._geofence_local: list[Point] = []
        self._geofence_ref: LatLon | None = None
        self._last_status_t = 0.0
        self._last_gps_t = 0.0

    # ── E-stop ────────────────────────────────────────────────────────────

    def trigger_estop(self, source: str = "user") -> None:
        with self._lock:
            self._estop = True
        log.warning("E-STOP triggered (%s)", source)

    def clear_estop(self) -> None:
        with self._lock:
            self._estop = False
        log.info("E-stop cleared")

    @property
    def estop_active(self) -> bool:
        with self._lock:
            return self._estop

    # ── Geofence ──────────────────────────────────────────────────────────

    def set_geofence(
        self,
        boundary: list[LatLon],
        *,
        buffer_m: float | None = None,
    ) -> None:
        if len(boundary) < 3:
            with self._lock:
                self._geofence_local = []
                self._geofence_ref = None
            return
        ref = boundary[0]
        local = [latlon_to_xy(p, ref) for p in boundary]
        effective_buffer = (
            self.cfg.geofence_buffer_m
            if buffer_m is None
            else buffer_m
        )
        shrunk = shrink_polygon(local, effective_buffer)
        with self._lock:
            self._geofence_local = shrunk
            self._geofence_ref = ref
        log.info("Geofence set: %d vertices, %.2f m buffer",
                 len(shrunk), effective_buffer)

    def inside_fence(self, p: LatLon) -> bool:
        with self._lock:
            poly = list(self._geofence_local)
            ref = self._geofence_ref
        if not poly or ref is None:
            return True  # fence not configured → no constraint
        return point_in_polygon(latlon_to_xy(p, ref), poly)

    # ── Heartbeats ────────────────────────────────────────────────────────

    def mark_status_received(self) -> None:
        self._last_status_t = time.time()

    def mark_gps_fix(self) -> None:
        self._last_gps_t = time.time()

    # ── Evaluation ────────────────────────────────────────────────────────

    def evaluate(
        self,
        *,
        position: LatLon | None,
        tilt_deg: float,
        battery_pct: int,
        gps_mode: bool,
    ) -> SafetyReport:
        reasons: list[str] = []
        verdict = SafetyVerdict.OK

        if self.estop_active:
            return SafetyReport(SafetyVerdict.STOP, ("E-stop engaged",))

        if abs(tilt_deg) > self.cfg.tilt_limit_deg:
            reasons.append(f"Tilt {tilt_deg:.1f}° > limit")
            verdict = SafetyVerdict.STOP

        if (
            not self.cfg.ignore_battery_for_testing
            and battery_pct <= self.cfg.battery_critical_pct
        ):
            reasons.append(f"Battery critical ({battery_pct}%)")
            verdict = SafetyVerdict.STOP

        now = time.time()
        if self._last_status_t and now - self._last_status_t > self.cfg.comms_timeout_s:
            reasons.append(
                f"No Pico STATUS for {now - self._last_status_t:.1f}s")
            verdict = SafetyVerdict.STOP

        if position is not None and not self.inside_fence(position):
            reasons.append("Outside geofence")
            verdict = SafetyVerdict.STOP

        if verdict != SafetyVerdict.STOP:
            if (
                not self.cfg.ignore_battery_for_testing
                and battery_pct <= self.cfg.battery_low_pct
            ):
                reasons.append(f"Battery low ({battery_pct}%)")
                verdict = SafetyVerdict.WARN
            if (
                gps_mode
                and self._last_gps_t
                and now - self._last_gps_t > self.cfg.gps_timeout_s
            ):
                reasons.append(
                    f"GPS fix stale ({now - self._last_gps_t:.1f}s)")
                verdict = SafetyVerdict.WARN

        return SafetyReport(verdict, tuple(reasons))
