from __future__ import annotations

import pytest

from mission_planner import gps_reader
from mission_planner.gps_reader import GPSReader


@pytest.mark.unit
def test_resolve_gps_port_returns_existing_explicit_port(monkeypatch) -> None:
    monkeypatch.setattr(gps_reader.os.path, "exists", lambda p: p == "/dev/ttyUSB7")

    assert gps_reader.resolve_gps_port("/dev/ttyUSB7") == "/dev/ttyUSB7"


@pytest.mark.unit
def test_resolve_gps_port_returns_none_for_missing_explicit_port(monkeypatch) -> None:
    monkeypatch.setattr(gps_reader.os.path, "exists", lambda _p: False)

    assert gps_reader.resolve_gps_port("/dev/ttyUSB7") is None


@pytest.mark.unit
def test_resolve_gps_port_auto_prefers_usb_candidate(monkeypatch) -> None:
    monkeypatch.setattr(gps_reader.Path, "exists", lambda self: False)
    monkeypatch.setattr(gps_reader.os.path, "exists", lambda p: p == "/dev/ttyUSB0")

    assert gps_reader.resolve_gps_port("auto") == "/dev/ttyUSB0"


@pytest.mark.unit
def test_gps_reader_tracks_sentence_arrival() -> None:
    reader = GPSReader(port="auto")

    reader._parse_manual("$GNGGA,123519,1904.5600,N,07252.6600,E,1,08,0.9,10.0,M,0.0,M,,*00")

    assert reader.get_fix().has_fix is True
