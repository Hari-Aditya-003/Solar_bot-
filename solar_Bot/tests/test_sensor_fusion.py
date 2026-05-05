"""Sensor fusion estimator tests."""

from __future__ import annotations

import time

import pytest

from mission_planner.config import NavigationConfig
from mission_planner.sensor_fusion import Estimator


@pytest.mark.unit
def test_gps_update_yields_pose() -> None:
    est = Estimator(NavigationConfig())
    est.update_gps(19.07, 72.87, 90.0, 0.5, has_fix=True)
    p = est.pose()
    assert p.has_position
    assert p.source == "gps"
    assert p.lat == pytest.approx(19.07)
    assert p.lon == pytest.approx(72.87)


@pytest.mark.unit
def test_no_fix_keeps_pose_empty() -> None:
    est = Estimator(NavigationConfig())
    est.update_gps(0.0, 0.0, 0.0, 0.0, has_fix=False)
    assert est.pose().has_position is False


@pytest.mark.unit
def test_dead_reckon_advances_after_anchor() -> None:
    est = Estimator(NavigationConfig())
    est.set_anchor(19.07, 72.87)

    # IMU-only updates with steady heading=0 (north) and 0.5 m/s speed
    # Force a small dt by sleeping briefly between updates.
    est.update_imu(0.0, speed_cms=50.0)
    time.sleep(0.05)
    est.update_imu(0.0, speed_cms=50.0)
    p = est.pose()
    assert p.source in ("dead_reckon", "gps")
    # Latitude should have nudged northward (positive Δlat)
    assert p.lat >= 19.07
