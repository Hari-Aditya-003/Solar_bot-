from __future__ import annotations

import pytest

from mission_planner.boundary_capture import BoundaryCapture
from mission_planner.geo import LatLon


@pytest.mark.unit
def test_capture_adds_points_and_marks_planned() -> None:
    capture = BoundaryCapture()

    assert capture.add_point(LatLon(19.0, 72.0), source="web") == 1
    assert capture.add_point(LatLon(19.0, 72.00001), source="web") == 2
    assert capture.recording is True
    assert capture.planned is False

    capture.mark_planned(source="radio")

    assert capture.recording is False
    assert capture.planned is True
    assert capture.last_source == "radio"


@pytest.mark.unit
def test_capture_rejects_duplicate_press_without_motion() -> None:
    capture = BoundaryCapture()
    capture.add_point(LatLon(19.0, 72.0), source="web")

    with pytest.raises(ValueError):
        capture.add_point(LatLon(19.0, 72.0), source="web")


@pytest.mark.unit
def test_capture_starts_new_boundary_after_planned_shape() -> None:
    capture = BoundaryCapture(
        points=[LatLon(19.0, 72.0), LatLon(19.0, 72.00002)],
        recording=False,
        planned=True,
        last_source="web",
    )

    count = capture.add_point(LatLon(19.00001, 72.00003), source="radio")

    assert count == 1
    assert capture.point_count == 1
    assert capture.recording is True
    assert capture.planned is False
    assert capture.last_source == "radio"
