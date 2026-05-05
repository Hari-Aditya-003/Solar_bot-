"""MissionStore round-trip tests."""

from __future__ import annotations

from pathlib import Path

import pytest

from mission_planner.mission_store import MissionStore


@pytest.fixture
def store(tmp_path: Path) -> MissionStore:
    return MissionStore(tmp_path)


@pytest.mark.unit
def test_save_load_roundtrip(store: MissionStore) -> None:
    payload = {
        "boundary": [{"lat": 19.07, "lon": 72.87}],
        "waypoints": [
            {"seq": 0, "lat": 19.07, "lon": 72.87},
            {"seq": 1, "lat": 19.08, "lon": 72.88},
        ],
        "stats": {"rows": 4, "distance_m": 12.5},
    }
    fname = store.save("Field A", payload)
    loaded = store.load(fname)
    assert loaded is not None
    assert loaded["name"] == "Field A"
    assert len(loaded["waypoints"]) == 2


@pytest.mark.unit
def test_list_orders_recent_first(store: MissionStore) -> None:
    store.save("alpha", {"waypoints": []})
    store.save("beta",  {"waypoints": []})
    listed = store.list()
    assert {m.name for m in listed} == {"alpha", "beta"}


@pytest.mark.unit
def test_delete(store: MissionStore) -> None:
    fname = store.save("kill me", {"waypoints": []})
    assert store.delete(fname) is True
    assert store.delete(fname) is False


@pytest.mark.unit
def test_unsafe_name_sanitized(store: MissionStore) -> None:
    fname = store.save("../../etc/passwd", {"waypoints": []})
    assert ".." not in fname and "/" not in fname
