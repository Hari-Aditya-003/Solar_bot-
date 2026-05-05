"""Configuration loader for the Solar Panel Cleaning Robot.

Reads ``config.yaml`` (next to this file) into an immutable, type-checked
dataclass.  Falls back to safe defaults if the file is missing.

Override at runtime with ``--config /path/to/config.yaml``.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import Any

try:
    import yaml  # type: ignore
    _HAS_YAML = True
except ImportError:  # pragma: no cover - optional dep
    _HAS_YAML = False

log = logging.getLogger(__name__)

# ─── Sub-configurations (frozen dataclasses) ──────────────────────────────────


@dataclass(frozen=True)
class HardwareConfig:
    """Serial / UART / GPIO settings."""

    gps_port: str = "/dev/ttyAMA2"
    gps_baud: int = 9600
    pico_port: str = "/dev/ttyAMA4"
    pico_baud: int = 115200
    pico_transport: str = "uart"  # "uart" | "udp"
    pico_udp_host: str = "192.168.4.1"
    pico_udp_port: int = 5005


@dataclass(frozen=True)
class RobotConfig:
    """Physical parameters of the robot used for kinematics."""

    wheel_diameter_cm: float = 6.5
    wheel_circumference_cm: float = 20.42
    track_width_cm: float = 18.0          # left↔right wheel distance
    max_speed_cms: float = 68.0           # ≈ 200 RPM × π × 6.5 cm
    cleaning_width_m: float = 0.40        # brush span
    battery_full_mv: int = 12_600         # 3S LiPo
    battery_empty_mv: int = 9_000


@dataclass(frozen=True)
class NavigationConfig:
    """Tunable parameters for the GPS / non-GPS waypoint follower."""

    target_speed_pct: int = 45            # cruise throttle (0-100)
    accept_radius_m: float = 0.8          # waypoint reached threshold
    kp_steer: float = 1.4                 # P term (heading error → steer)
    ki_steer: float = 0.05                # I term
    kd_steer: float = 0.30                # D term
    nav_rate_hz: int = 10                 # nav loop rate
    heading_filter_alpha: float = 0.85    # complementary filter weight
    drift_correct_max_deg: float = 25.0   # cap on instantaneous correction
    end_of_lane_distance_m: float = 0.20  # non-GPS: extra past last WP before turn


@dataclass(frozen=True)
class PlannerConfig:
    """Defaults shown in the UI sliders."""

    row_spacing_m: float = 0.50           # default lane spacing
    overlap_pct: float = 10.0             # 0-50 %
    sweep_angle_deg: float | None = None  # None = auto (longest edge)
    return_to_home: bool = True
    min_polygon_points: int = 3


@dataclass(frozen=True)
class SafetyConfig:
    """Hard limits enforced by safety.py."""

    geofence_buffer_m: float = 0.50       # extra inside the polygon
    tilt_limit_deg: float = 30.0
    battery_low_pct: int = 15
    battery_critical_pct: int = 5
    comms_timeout_s: float = 2.0          # last STATUS heard from Pico
    gps_timeout_s: float = 3.0            # last fix age
    obstacle_distance_cm: float = 25.0    # if ultrasonic added later


@dataclass(frozen=True)
class ServerConfig:
    host: str = "0.0.0.0"
    port: int = 5000
    socketio_ping: int = 25
    log_level: str = "INFO"


@dataclass(frozen=True)
class Config:
    """Top-level configuration."""

    hardware: HardwareConfig = field(default_factory=HardwareConfig)
    robot: RobotConfig = field(default_factory=RobotConfig)
    navigation: NavigationConfig = field(default_factory=NavigationConfig)
    planner: PlannerConfig = field(default_factory=PlannerConfig)
    safety: SafetyConfig = field(default_factory=SafetyConfig)
    server: ServerConfig = field(default_factory=ServerConfig)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


# ─── YAML loader ──────────────────────────────────────────────────────────────


def _build(section_cls: type, raw: dict[str, Any] | None) -> Any:
    """Construct a frozen dataclass from a (possibly partial) dict."""
    if not raw:
        return section_cls()
    valid = {f for f in section_cls.__dataclass_fields__}
    return section_cls(**{k: v for k, v in raw.items() if k in valid})


def load_config(path: str | Path | None = None) -> Config:
    """Load configuration from YAML; return defaults if the file is absent."""
    if path is None:
        path = Path(__file__).parent / "config.yaml"
    path = Path(path)

    if not path.exists():
        log.info("config.yaml not found at %s — using defaults", path)
        return Config()

    if not _HAS_YAML:
        log.warning("PyYAML not installed; using defaults. pip install pyyaml")
        return Config()

    try:
        raw = yaml.safe_load(path.read_text()) or {}
    except yaml.YAMLError as exc:
        log.error("Bad YAML in %s: %s — using defaults", path, exc)
        return Config()

    return Config(
        hardware=_build(HardwareConfig, raw.get("hardware")),
        robot=_build(RobotConfig, raw.get("robot")),
        navigation=_build(NavigationConfig, raw.get("navigation")),
        planner=_build(PlannerConfig, raw.get("planner")),
        safety=_build(SafetyConfig, raw.get("safety")),
        server=_build(ServerConfig, raw.get("server")),
    )
