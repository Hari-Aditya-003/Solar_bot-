"""Radio bridge helper tests."""

from __future__ import annotations

import time

import pytest

from mission_planner.radio_bridge import RC_TIMEOUT_S, RadioStatus, _map_axis


@pytest.mark.unit
def test_map_axis_handles_deadband_and_direction() -> None:
    assert _map_axis(1500) == 0
    assert _map_axis(1530) == 0
    assert _map_axis(1600) > 0
    assert _map_axis(1400) < 0


@pytest.mark.unit
def test_radio_status_detects_active_control() -> None:
    status = RadioStatus(
        connected=True,
        last_seen=time.time(),
        channels=[1500, 1300, 1500, 1700, 1000, 1000, 1000, 1000],
    )
    assert status.signal_ok is True
    assert status.control_active is True
    assert status.steer_pct > 0
    assert status.throttle_pct < 0


@pytest.mark.unit
def test_radio_status_switch_high_uses_channel_threshold() -> None:
    status = RadioStatus(
        connected=True,
        last_seen=time.time(),
        channels=[1500, 1500, 1500, 1500, 1000, 1800, 1500, 900],
    )
    assert status.switch_high(6) is True
    assert status.switch_high(7) is False


@pytest.mark.unit
def test_radio_status_times_out() -> None:
    status = RadioStatus(
        connected=True,
        last_seen=time.time() - (RC_TIMEOUT_S + 0.1),
    )
    assert status.signal_ok is False
    assert status.control_active is False
