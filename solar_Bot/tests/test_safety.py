"""SafetyMonitor unit tests."""

from __future__ import annotations

import time

import pytest

from mission_planner.config import SafetyConfig
from mission_planner.geo import LatLon
from mission_planner.safety import SafetyMonitor, SafetyVerdict


SQUARE = [
    LatLon(0.0, 0.0),
    LatLon(0.0001, 0.0),
    LatLon(0.0001, 0.0001),
    LatLon(0.0, 0.0001),
]


@pytest.mark.unit
def test_estop_short_circuits_everything() -> None:
    sm = SafetyMonitor(SafetyConfig())
    sm.trigger_estop("test")
    rep = sm.evaluate(position=None, tilt_deg=0, battery_pct=100, gps_mode=True)
    assert rep.verdict is SafetyVerdict.STOP
    assert "E-stop" in rep.reasons[0]


@pytest.mark.unit
def test_geofence_violation_stops() -> None:
    cfg = SafetyConfig(geofence_buffer_m=0.0, comms_timeout_s=999)
    sm = SafetyMonitor(cfg)
    sm.set_geofence(SQUARE)
    sm.mark_status_received()
    outside = LatLon(1.0, 1.0)
    rep = sm.evaluate(position=outside, tilt_deg=0,
                      battery_pct=100, gps_mode=True)
    assert rep.verdict is SafetyVerdict.STOP


@pytest.mark.unit
def test_tilt_limit_stops() -> None:
    sm = SafetyMonitor(SafetyConfig(comms_timeout_s=999))
    sm.mark_status_received()
    rep = sm.evaluate(position=None, tilt_deg=45,
                      battery_pct=100, gps_mode=True)
    assert rep.verdict is SafetyVerdict.STOP
    assert any("Tilt" in r for r in rep.reasons)


@pytest.mark.unit
def test_battery_low_warns() -> None:
    sm = SafetyMonitor(SafetyConfig(comms_timeout_s=999, battery_low_pct=20,
                                    battery_critical_pct=5))
    sm.mark_status_received()
    rep = sm.evaluate(position=None, tilt_deg=0,
                      battery_pct=10, gps_mode=True)
    assert rep.verdict is SafetyVerdict.WARN


@pytest.mark.unit
def test_battery_critical_stops() -> None:
    sm = SafetyMonitor(SafetyConfig(comms_timeout_s=999, battery_critical_pct=10))
    sm.mark_status_received()
    rep = sm.evaluate(position=None, tilt_deg=0,
                      battery_pct=5, gps_mode=True)
    assert rep.verdict is SafetyVerdict.STOP


@pytest.mark.unit
def test_clear_estop_recovers() -> None:
    sm = SafetyMonitor(SafetyConfig(comms_timeout_s=999))
    sm.mark_status_received()
    sm.trigger_estop("u")
    sm.clear_estop()
    rep = sm.evaluate(position=None, tilt_deg=0,
                      battery_pct=100, gps_mode=True)
    assert rep.verdict is SafetyVerdict.OK
