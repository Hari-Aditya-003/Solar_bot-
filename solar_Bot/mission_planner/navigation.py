"""Dual-mode waypoint follower.

GPS mode  — needs the :class:`GPSReader` and the Pico for steering.
NON-GPS   — needs only the IMU/encoders coming back via the Pico STATUS
            stream.  Lawnmower lanes are fixed metric distances, executed
            as: drive straight (heading-locked) → encoder distance reached
            → in-place turn ±180° → shift sideways one lane → repeat.

Both modes share the same:

* ``Mission`` object (boundary + waypoints + status flags),
* PID controller for heading,
* :class:`SafetyMonitor` veto (geofence, tilt, comms, e-stop),
* event emitter (set ``mission_event_cb`` to broadcast to Socket.IO).

Threading model: a single ``run()`` thread, ticking at
``cfg.navigation.nav_rate_hz``.  Subscribers register a callback rather
than poking the internal state.
"""

from __future__ import annotations

import enum
import logging
import math
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Optional

from .config import Config
from .geo import LatLon, bearing_deg, haversine_m, heading_error_deg
from .robot_bridge import RobotBridge, RobotStatus
from .safety import SafetyMonitor, SafetyVerdict
from .sensor_fusion import Estimator, Pose

log = logging.getLogger(__name__)


# ─── Public types ─────────────────────────────────────────────────────────────


class NavMode(enum.Enum):
    GPS = "gps"
    NON_GPS = "non_gps"


class MissionState(enum.Enum):
    IDLE = "idle"
    RUNNING = "running"
    PAUSED = "paused"
    COMPLETE = "complete"
    ABORTED = "aborted"


@dataclass
class Mission:
    name: str = ""
    boundary: list[LatLon] = field(default_factory=list)
    waypoints: list[LatLon] = field(default_factory=list)
    state: MissionState = MissionState.IDLE
    current_wp: int = 0
    started_at: float = 0.0
    distance_done_m: float = 0.0

    def progress_pct(self) -> float:
        if not self.waypoints:
            return 0.0
        return min(100.0, (self.current_wp / max(1, len(self.waypoints) - 1)) * 100.0)

    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "boundary": [{"lat": p.lat, "lon": p.lon} for p in self.boundary],
            "waypoints": [
                {"seq": i, "lat": w.lat, "lon": w.lon}
                for i, w in enumerate(self.waypoints)
            ],
            "state": self.state.value,
            "current_wp": self.current_wp,
            "wp_total": len(self.waypoints),
            "progress_pct": round(self.progress_pct(), 1),
            "distance_done_m": round(self.distance_done_m, 1),
        }


# ─── Helpers ──────────────────────────────────────────────────────────────────


@dataclass
class _PIDState:
    integral: float = 0.0
    last_error: float = 0.0
    last_t: float = 0.0


# ─── Navigator ────────────────────────────────────────────────────────────────


class Navigator:
    """Runs the navigation loop in a background daemon thread."""

    def __init__(
        self,
        cfg: Config,
        robot: Optional[RobotBridge],
        estimator: Estimator,
        safety: SafetyMonitor,
        on_event: Callable[[str, dict], None] | None = None,
    ) -> None:
        self.cfg = cfg
        self.robot = robot
        self.estimator = estimator
        self.safety = safety
        self.on_event = on_event or (lambda _e, _p: None)

        self.mode: NavMode = NavMode.GPS
        self.mission = Mission()
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._pid = _PIDState()
        self._lane_start: Pose | None = None
        self._target_speed_pct: int = cfg.navigation.target_speed_pct
        self._thread: threading.Thread | None = None

    # ── Lifecycle ─────────────────────────────────────────────────────────

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True,
                                        name="Navigator")
        self._thread.start()

    def shutdown(self) -> None:
        self._stop.set()
        if self.robot:
            self.robot.stop_robot()

    # ── Mission API ───────────────────────────────────────────────────────

    def set_mission(self, name: str, boundary: list[LatLon],
                    waypoints: list[LatLon]) -> None:
        with self._lock:
            self.mission = Mission(
                name=name,
                boundary=list(boundary),
                waypoints=list(waypoints),
                state=MissionState.IDLE,
                current_wp=0,
            )
        self.safety.set_geofence(boundary)
        log.info("Mission loaded: %s (%d waypoints)", name, len(waypoints))

    def set_mode(self, mode: NavMode) -> None:
        with self._lock:
            self.mode = mode
        log.info("Nav mode: %s", mode.value)

    def set_speed_pct(self, pct: int) -> None:
        with self._lock:
            self._target_speed_pct = max(0, min(100, int(pct)))

    def start_mission(self) -> bool:
        with self._lock:
            if not self.mission.waypoints:
                return False
            self.mission.state = MissionState.RUNNING
            self.mission.current_wp = 0
            self.mission.started_at = time.time()
            self.mission.distance_done_m = 0.0
            self._reset_pid()
            self._lane_start = self.estimator.pose()
        if self.robot:
            self.robot.mission_start()
        self._emit("mission_started")
        return True

    def pause_mission(self) -> None:
        with self._lock:
            if self.mission.state == MissionState.RUNNING:
                self.mission.state = MissionState.PAUSED
        if self.robot:
            self.robot.mission_pause()
        self._emit("mission_paused")

    def resume_mission(self) -> None:
        with self._lock:
            if self.mission.state == MissionState.PAUSED:
                self.mission.state = MissionState.RUNNING
                self._reset_pid()
        self._emit("mission_resumed")

    def abort_mission(self) -> None:
        with self._lock:
            self.mission.state = MissionState.ABORTED
            self.mission.current_wp = 0
        if self.robot:
            self.robot.mission_abort()
        self._emit("mission_aborted")

    # ── State snapshot ────────────────────────────────────────────────────

    def snapshot(self) -> dict:
        with self._lock:
            mission_dict = self.mission.to_dict()
            mode = self.mode.value
            target = self._target_speed_pct
        return {
            "mission": mission_dict,
            "mode": mode,
            "target_speed_pct": target,
        }

    # ── Internal: main loop ───────────────────────────────────────────────

    def _run(self) -> None:
        interval = 1.0 / max(1, self.cfg.navigation.nav_rate_hz)
        log.info("Navigator running at %d Hz", self.cfg.navigation.nav_rate_hz)
        while not self._stop.is_set():
            time.sleep(interval)
            try:
                self._tick()
            except Exception:  # pragma: no cover - resilience
                log.exception("Nav tick failed")

    def _tick(self) -> None:
        with self._lock:
            state = self.mission.state
            wps = list(self.mission.waypoints)
            idx = self.mission.current_wp
            mode = self.mode

        if state != MissionState.RUNNING or not wps or idx >= len(wps):
            return

        pose = self.estimator.pose()
        robot_status = self.robot.get_status() if self.robot else RobotStatus()
        battery_pct = robot_status.battery_pct

        # Safety veto first.
        report = self.safety.evaluate(
            position=pose.as_latlon() if pose.has_position else None,
            tilt_deg=max(abs(robot_status.roll), abs(robot_status.pitch)),
            battery_pct=battery_pct,
            gps_mode=mode is NavMode.GPS,
        )
        if report.verdict is SafetyVerdict.STOP:
            self._halt_and_pause(report.reasons)
            return
        if report.verdict is SafetyVerdict.WARN:
            self._emit("safety_warn", {"reasons": list(report.reasons)})

        # Branch on mode.
        if mode is NavMode.GPS:
            self._tick_gps(wps, idx, pose)
        else:
            self._tick_non_gps(wps, idx, pose, robot_status)

    # ── GPS waypoint follower (PID) ───────────────────────────────────────

    def _tick_gps(self, wps: list[LatLon], idx: int, pose: Pose) -> None:
        if not pose.has_position:
            self._send_move(0, 0)
            return

        target = wps[idx]
        here = pose.as_latlon()
        dist = haversine_m(here, target)

        if dist < self.cfg.navigation.accept_radius_m:
            self._on_waypoint_reached(idx, len(wps))
            return

        desired = bearing_deg(here, target)
        err = heading_error_deg(desired, pose.heading_deg)
        steer = self._pid_step(err)

        # Reduce throttle while turning hard.
        turn_factor = max(0.4, 1.0 - abs(steer) / 200.0)
        throttle = int(self._target_speed_pct * turn_factor)

        self._send_move(steer, throttle)
        self._emit("nav_telemetry", {
            "wp": idx,
            "total": len(wps),
            "dist_m": round(dist, 2),
            "bearing": round(desired, 1),
            "heading": round(pose.heading_deg, 1),
            "heading_err": round(err, 1),
            "steer": steer,
            "throttle": throttle,
            "source": pose.source,
        })

    # ── Non-GPS lane follower ─────────────────────────────────────────────

    def _tick_non_gps(self, wps: list[LatLon], idx: int, pose: Pose,
                      robot_status: RobotStatus) -> None:
        """Use IMU heading + encoder distance to walk the lawnmower path.

        We trust the *generated* lane geometry (its bearing and length) and
        execute it as: lock to lane bearing → drive forward until the encoder
        has covered the lane length → snap to next waypoint.  The estimator
        feeds us a dead-reckoned position so the UI can still show progress.
        """
        if idx >= len(wps) - 1:
            self._on_waypoint_reached(idx, len(wps))
            return

        a, b = wps[idx], wps[idx + 1]
        lane_bearing = bearing_deg(a, b)
        lane_length = haversine_m(a, b)

        # Distance covered along this lane (use estimator if it has a fix,
        # otherwise integrate speed since lane started).
        if pose.has_position:
            covered = haversine_m(a, pose.as_latlon())
        else:
            elapsed = time.time() - (self.mission.started_at or time.time())
            covered = pose.speed_ms * elapsed

        remaining = lane_length - covered
        if remaining <= self.cfg.navigation.end_of_lane_distance_m:
            log.debug("Lane %d→%d complete (%.2f m)", idx, idx + 1, lane_length)
            self._on_waypoint_reached(idx, len(wps))
            return

        err = heading_error_deg(lane_bearing, pose.heading_deg)
        # Cap correction so we don't wildly steer when heading is briefly off.
        err = max(-self.cfg.navigation.drift_correct_max_deg,
                  min(self.cfg.navigation.drift_correct_max_deg, err))
        steer = self._pid_step(err)
        turn_factor = max(0.5, 1.0 - abs(steer) / 200.0)
        throttle = int(self._target_speed_pct * turn_factor)

        self._send_move(steer, throttle)
        self._emit("nav_telemetry", {
            "wp": idx,
            "total": len(wps),
            "dist_m": round(remaining, 2),
            "bearing": round(lane_bearing, 1),
            "heading": round(pose.heading_deg, 1),
            "heading_err": round(err, 1),
            "steer": steer,
            "throttle": throttle,
            "source": pose.source,
            "lane_length_m": round(lane_length, 2),
            "lane_covered_m": round(covered, 2),
        })

    # ── Shared helpers ────────────────────────────────────────────────────

    def _on_waypoint_reached(self, idx: int, total: int) -> None:
        with self._lock:
            self.mission.current_wp += 1
            new_idx = self.mission.current_wp
            done = new_idx >= total
            if done:
                self.mission.state = MissionState.COMPLETE
                self.mission.current_wp = 0
            else:
                self._reset_pid()
                self._lane_start = self.estimator.pose()
        self._send_move(0, 0)
        if done:
            self._emit("mission_complete", {"total": total})
        else:
            self._emit("waypoint_reached", {"wp": new_idx, "total": total})

    def _halt_and_pause(self, reasons: tuple[str, ...]) -> None:
        with self._lock:
            self.mission.state = MissionState.PAUSED
        self._send_move(0, 0)
        log.warning("Safety STOP: %s", "; ".join(reasons))
        self._emit("safety_stop", {"reasons": list(reasons)})

    def _send_move(self, steer: int, throttle: int) -> None:
        if self.robot:
            self.robot.move(steer, throttle)

    def _reset_pid(self) -> None:
        self._pid = _PIDState(last_t=time.time())

    def _pid_step(self, err: float) -> int:
        nav = self.cfg.navigation
        now = time.time()
        dt = max(1e-3, now - self._pid.last_t) if self._pid.last_t else 1.0
        self._pid.integral = max(-50.0, min(50.0, self._pid.integral + err * dt))
        derivative = (err - self._pid.last_error) / dt
        self._pid.last_error = err
        self._pid.last_t = now
        steer = int(nav.kp_steer * err + nav.ki_steer * self._pid.integral
                    + nav.kd_steer * derivative)
        return max(-100, min(100, steer))

    def _emit(self, event: str, payload: dict | None = None) -> None:
        try:
            self.on_event(event, payload or {})
        except Exception:  # pragma: no cover
            log.exception("Event callback failed for %s", event)
