"""Geometry / geodesy tests."""

from __future__ import annotations

import math

import pytest

from mission_planner.geo import (
    LatLon,
    Point,
    bearing_deg,
    haversine_m,
    heading_error_deg,
    latlon_to_xy,
    point_in_polygon,
    polygon_area_m2,
    rotate,
    shrink_polygon,
    xy_to_latlon,
)


@pytest.mark.unit
def test_haversine_known_distance() -> None:
    # 1° of latitude ≈ 111 km
    a = LatLon(0.0, 0.0)
    b = LatLon(1.0, 0.0)
    assert haversine_m(a, b) == pytest.approx(111_195, rel=1e-3)


@pytest.mark.unit
def test_bearing_cardinal_directions() -> None:
    assert bearing_deg(LatLon(0, 0), LatLon(1, 0)) == pytest.approx(0.0, abs=0.5)
    assert bearing_deg(LatLon(0, 0), LatLon(0, 1)) == pytest.approx(90.0, abs=0.5)
    assert bearing_deg(LatLon(0, 0), LatLon(-1, 0)) == pytest.approx(180.0, abs=0.5)
    assert bearing_deg(LatLon(0, 0), LatLon(0, -1)) == pytest.approx(270.0, abs=0.5)


@pytest.mark.unit
def test_heading_error_wraps() -> None:
    assert heading_error_deg(10.0, 350.0) == pytest.approx(20.0)
    assert heading_error_deg(350.0, 10.0) == pytest.approx(-20.0)
    # 180° is the antipodal case; ±180 are both valid representations.
    assert abs(heading_error_deg(180.0, 0.0)) == pytest.approx(180.0)


@pytest.mark.unit
def test_latlon_xy_roundtrip() -> None:
    ref = LatLon(19.0760, 72.8777)
    p = LatLon(19.0762, 72.8779)
    xy = latlon_to_xy(p, ref)
    back = xy_to_latlon(xy, ref)
    assert back.lat == pytest.approx(p.lat, abs=1e-7)
    assert back.lon == pytest.approx(p.lon, abs=1e-7)


@pytest.mark.unit
def test_rotate_preserves_length() -> None:
    p = Point(3.0, 4.0)
    r = rotate(p, math.radians(30))
    assert math.hypot(r.x, r.y) == pytest.approx(5.0)


@pytest.mark.unit
def test_polygon_area_unit_square() -> None:
    sq = [Point(0, 0), Point(1, 0), Point(1, 1), Point(0, 1)]
    assert polygon_area_m2(sq) == pytest.approx(1.0)


@pytest.mark.unit
def test_point_in_polygon() -> None:
    sq = [Point(0, 0), Point(2, 0), Point(2, 2), Point(0, 2)]
    assert point_in_polygon(Point(1, 1), sq) is True
    assert point_in_polygon(Point(3, 3), sq) is False


@pytest.mark.unit
def test_shrink_polygon_smaller_area() -> None:
    sq = [Point(0, 0), Point(10, 0), Point(10, 10), Point(0, 10)]
    shrunk = shrink_polygon(sq, 0.5)
    assert polygon_area_m2(shrunk) < polygon_area_m2(sq)
