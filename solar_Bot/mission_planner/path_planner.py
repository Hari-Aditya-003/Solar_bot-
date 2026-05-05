"""Boustrophedon (lawnmower) coverage-path planner.

Given a GPS polygon boundary, the cleaning width and an overlap percentage,
this module produces an ordered list of waypoints that fully covers the
interior with parallel passes alternating in direction (the snake / Z pattern
used in agricultural and floor-cleaning robots).

Algorithm:
    1. Reproject the polygon to local metres around its centroid.
    2. Rotate the frame so passes are horizontal (auto = longest edge,
       or user-supplied ``angle_deg``).
    3. Step horizontal scan-lines at ``effective_spacing_m``
       (= cleaning_width × (1 - overlap_pct/100)).
    4. Clip each line to the polygon and append endpoints in alternating
       order to form the snake.
    5. Rotate back, reproject to GPS, optionally append return-to-home.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field
from typing import Sequence

from .geo import (
    LatLon,
    Point,
    bearing_deg,
    haversine_m,
    horizontal_intersections,
    latlon_centroid,
    latlon_to_xy,
    longest_edge_angle,
    polygon_area_m2,
    rotate,
    xy_to_latlon,
)

log = logging.getLogger(__name__)


# ─── Public dataclasses ───────────────────────────────────────────────────────


@dataclass(frozen=True)
class CoveragePlan:
    """Result of a single planning run."""

    waypoints: tuple[LatLon, ...] = ()
    rows: int = 0
    distance_m: float = 0.0
    area_m2: float = 0.0
    sweep_angle_deg: float = 0.0
    effective_spacing_m: float = 0.0
    estimated_time_s: float = 0.0

    def to_dict(self) -> dict:
        return {
            "waypoints": [
                {"seq": i, "lat": w.lat, "lon": w.lon}
                for i, w in enumerate(self.waypoints)
            ],
            "rows": self.rows,
            "distance_m": round(self.distance_m, 1),
            "area_m2": round(self.area_m2, 1),
            "sweep_angle_deg": round(self.sweep_angle_deg, 1),
            "effective_spacing_m": round(self.effective_spacing_m, 3),
            "estimated_time_s": round(self.estimated_time_s),
        }


@dataclass(frozen=True)
class PlanRequest:
    boundary: tuple[LatLon, ...]
    cleaning_width_m: float = 0.40
    overlap_pct: float = 10.0
    sweep_angle_deg: float | None = None
    return_to_home: bool = True
    robot_speed_ms: float = 0.30


# ─── Core planner ─────────────────────────────────────────────────────────────


def plan_coverage(req: PlanRequest) -> CoveragePlan:
    """Generate a boustrophedon coverage plan for ``req.boundary``."""
    if len(req.boundary) < 3:
        log.warning("plan_coverage called with %d points (need >=3)", len(req.boundary))
        return CoveragePlan()

    spacing = max(0.05, req.cleaning_width_m * (1.0 - req.overlap_pct / 100.0))

    ref = latlon_centroid(req.boundary)
    local: list[Point] = [latlon_to_xy(p, ref) for p in req.boundary]

    sweep_rad = (
        math.radians(req.sweep_angle_deg)
        if req.sweep_angle_deg is not None
        else longest_edge_angle(local)
    )

    rotated = [rotate(p, -sweep_rad) for p in local]
    ys = [p.y for p in rotated]
    y_min, y_max = min(ys), max(ys)

    if y_max - y_min < spacing:
        log.warning("Polygon too small for one full pass — returning centre point only")
        return CoveragePlan(waypoints=(ref,), rows=0, distance_m=0.0,
                            area_m2=polygon_area_m2(local),
                            sweep_angle_deg=math.degrees(sweep_rad),
                            effective_spacing_m=spacing)

    waypoints_rot: list[Point] = []
    rows = 0
    y = y_min + spacing / 2.0
    while y <= y_max:
        xs = horizontal_intersections(rotated, y)
        if len(xs) >= 2:
            x_start, x_end = xs[0], xs[-1]
            if rows % 2 == 0:
                waypoints_rot.append(Point(x_start, y))
                waypoints_rot.append(Point(x_end, y))
            else:
                waypoints_rot.append(Point(x_end, y))
                waypoints_rot.append(Point(x_start, y))
            rows += 1
        y += spacing

    if not waypoints_rot:
        return CoveragePlan()

    waypoints_xy = [rotate(p, sweep_rad) for p in waypoints_rot]
    waypoints_gps: list[LatLon] = [xy_to_latlon(p, ref) for p in waypoints_xy]

    if req.return_to_home and len(waypoints_gps) > 1:
        waypoints_gps.append(waypoints_gps[0])

    distance = path_distance_m(waypoints_gps)
    speed = max(req.robot_speed_ms, 0.01)

    return CoveragePlan(
        waypoints=tuple(waypoints_gps),
        rows=rows,
        distance_m=distance,
        area_m2=polygon_area_m2(local),
        sweep_angle_deg=math.degrees(sweep_rad),
        effective_spacing_m=spacing,
        estimated_time_s=distance / speed,
    )


# ─── Helpers used elsewhere ───────────────────────────────────────────────────


def path_distance_m(waypoints: Sequence[LatLon]) -> float:
    """Total path length in metres."""
    return sum(haversine_m(waypoints[i], waypoints[i + 1])
               for i in range(len(waypoints) - 1))


def lane_segments(waypoints: Sequence[LatLon]) -> list[tuple[LatLon, LatLon, float]]:
    """Return ``(start, end, bearing_deg)`` triples — used by the non-GPS
    follower so it knows how long each straight lane is and where to turn."""
    out: list[tuple[LatLon, LatLon, float]] = []
    for i in range(len(waypoints) - 1):
        a, b = waypoints[i], waypoints[i + 1]
        out.append((a, b, bearing_deg(a, b)))
    return out


# ─── Backwards-compatibility shims (used by old app.py) ───────────────────────


def generate_grid_path(
    boundary_gps: list[tuple[float, float]],
    row_spacing_m: float = 0.8,
    angle_deg: float | None = None,
    return_to_home: bool = True,
) -> list[tuple[float, float]]:
    """Legacy function retained so the old app.py still imports cleanly."""
    plan = plan_coverage(PlanRequest(
        boundary=tuple(LatLon(lat, lon) for lat, lon in boundary_gps),
        cleaning_width_m=row_spacing_m,
        overlap_pct=0.0,
        sweep_angle_deg=angle_deg,
        return_to_home=return_to_home,
    ))
    return [(w.lat, w.lon) for w in plan.waypoints]


def path_stats(waypoints_gps: list[tuple[float, float]],
               robot_speed_ms: float = 0.3) -> dict:
    """Legacy function: distance/time summary."""
    if len(waypoints_gps) < 2:
        return {"distance_m": 0, "time_s": 0, "waypoints": len(waypoints_gps)}
    pts = [LatLon(lat, lon) for lat, lon in waypoints_gps]
    total = path_distance_m(pts)
    return {
        "distance_m": round(total, 1),
        "time_s": round(total / max(robot_speed_ms, 0.01)),
        "waypoints": len(waypoints_gps),
    }


def bearing_to(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    return bearing_deg(LatLon(lat1, lon1), LatLon(lat2, lon2))


def distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    return haversine_m(LatLon(lat1, lon1), LatLon(lat2, lon2))
