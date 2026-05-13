"""Path planner coverage tests."""

from __future__ import annotations

import pytest

from mission_planner.geo import LatLon, latlon_to_xy, point_in_polygon, shrink_polygon
from mission_planner.path_planner import (
    PlanRequest,
    path_distance_m,
    plan_boundary_route,
    plan_coverage,
)

SQUARE_5M = [
    LatLon(19.07600, 72.87770),  # NW corner-ish
    LatLon(19.07604, 72.87770),
    LatLon(19.07604, 72.87775),
    LatLon(19.07600, 72.87775),
]


@pytest.mark.unit
def test_plan_returns_waypoints_for_simple_polygon() -> None:
    plan = plan_coverage(PlanRequest(boundary=tuple(SQUARE_5M),
                                     cleaning_width_m=0.5,
                                     overlap_pct=0.0,
                                     return_to_home=False))
    assert plan.rows >= 4
    assert len(plan.waypoints) >= 8
    assert plan.distance_m > 0
    assert plan.area_m2 > 0


@pytest.mark.unit
def test_overlap_increases_distance() -> None:
    a = plan_coverage(PlanRequest(boundary=tuple(SQUARE_5M),
                                  cleaning_width_m=0.5, overlap_pct=0.0))
    b = plan_coverage(PlanRequest(boundary=tuple(SQUARE_5M),
                                  cleaning_width_m=0.5, overlap_pct=40.0))
    # More overlap = closer rows = more rows = more distance covered.
    assert b.distance_m >= a.distance_m


@pytest.mark.unit
def test_waypoints_inside_boundary() -> None:
    """Waypoints should sit inside the polygon (within a small edge tolerance).

    The boustrophedon planner places the start/end of each row exactly on the
    boundary edges where the scan-line meets the polygon — those points are
    "on the edge", which a strict ray-cast may classify either way.  We test
    instead that every waypoint is within ``edge_tolerance_m`` of being inside
    the un-shrunk polygon.
    """
    from mission_planner.geo import shrink_polygon

    plan = plan_coverage(PlanRequest(boundary=tuple(SQUARE_5M),
                                     cleaning_width_m=0.4,
                                     overlap_pct=0.0,
                                     return_to_home=False))
    ref = SQUARE_5M[0]
    poly_local = [latlon_to_xy(p, ref) for p in SQUARE_5M]
    # Slightly *expand* the polygon so edge-coincident points read as inside.
    poly_inflated = shrink_polygon(poly_local, -0.05)
    for w in plan.waypoints:
        assert point_in_polygon(latlon_to_xy(w, ref), poly_inflated), (
            f"Waypoint {w} fell outside boundary")


@pytest.mark.unit
def test_edge_buffer_keeps_waypoints_inside_geofence_margin() -> None:
    plan = plan_coverage(PlanRequest(boundary=tuple(SQUARE_5M),
                                     cleaning_width_m=0.4,
                                     overlap_pct=0.0,
                                     return_to_home=False,
                                     edge_buffer_m=0.55))
    ref = SQUARE_5M[0]
    geofence_local = shrink_polygon(
        [latlon_to_xy(p, ref) for p in SQUARE_5M],
        0.5,
    )
    for w in plan.waypoints:
        assert point_in_polygon(latlon_to_xy(w, ref), geofence_local), (
            f"Waypoint {w} fell outside buffered geofence")


@pytest.mark.unit
def test_too_few_points_returns_empty() -> None:
    plan = plan_coverage(PlanRequest(
        boundary=(LatLon(0, 0), LatLon(0, 1))))
    assert plan.waypoints == ()


@pytest.mark.unit
def test_return_to_home_appends_first() -> None:
    plan = plan_coverage(PlanRequest(boundary=tuple(SQUARE_5M),
                                     cleaning_width_m=0.5,
                                     return_to_home=True))
    assert plan.waypoints[0] == plan.waypoints[-1]


@pytest.mark.unit
def test_distance_helper_matches_plan() -> None:
    plan = plan_coverage(PlanRequest(boundary=tuple(SQUARE_5M),
                                     cleaning_width_m=0.5))
    assert path_distance_m(list(plan.waypoints)) == pytest.approx(
        plan.distance_m, abs=0.01)


@pytest.mark.unit
def test_boundary_route_reuses_corners_as_waypoints() -> None:
    plan = plan_boundary_route(SQUARE_5M, return_to_home=True)
    assert list(plan.waypoints[:-1]) == SQUARE_5M
    assert plan.waypoints[0] == plan.waypoints[-1]
    assert plan.rows == len(plan.waypoints) - 1
    assert plan.distance_m > 0
