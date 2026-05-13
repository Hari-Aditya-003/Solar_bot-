"""Geodesy and 2-D geometry helpers.

All coordinate conversions use an equirectangular projection centred on a
reference point.  This is accurate to a few cm over panel-cleaning sized fields
(tens of metres) — far better than the 2-3 m of GPS noise we expect.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from typing import NamedTuple

# ─── Constants ────────────────────────────────────────────────────────────────

EARTH_RADIUS_M: float = 6_371_000.0
M_PER_DEG_LAT: float = 111_320.0


# ─── Value types ──────────────────────────────────────────────────────────────


class LatLon(NamedTuple):
    lat: float
    lon: float


class Point(NamedTuple):
    """Local 2-D Cartesian point in metres."""

    x: float
    y: float


# ─── Coordinate conversions ───────────────────────────────────────────────────


def latlon_to_xy(p: LatLon, ref: LatLon) -> Point:
    """Project a GPS point to local metres relative to ``ref``."""
    x = (p.lon - ref.lon) * math.cos(math.radians(ref.lat)) * M_PER_DEG_LAT
    y = (p.lat - ref.lat) * M_PER_DEG_LAT
    return Point(x, y)


def xy_to_latlon(p: Point, ref: LatLon) -> LatLon:
    """Inverse of :func:`latlon_to_xy`."""
    lat = ref.lat + p.y / M_PER_DEG_LAT
    lon = ref.lon + p.x / (M_PER_DEG_LAT * math.cos(math.radians(ref.lat)))
    return LatLon(lat, lon)


def rotate(p: Point, angle_rad: float) -> Point:
    """Rotate ``p`` about the origin by ``angle_rad`` (radians)."""
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    return Point(p.x * c - p.y * s, p.x * s + p.y * c)


# ─── Distance and bearing ─────────────────────────────────────────────────────


def haversine_m(a: LatLon, b: LatLon) -> float:
    """Great-circle distance between two GPS points, in metres."""
    phi1 = math.radians(a.lat)
    phi2 = math.radians(b.lat)
    dphi = math.radians(b.lat - a.lat)
    dlam = math.radians(b.lon - a.lon)
    h = (
        math.sin(dphi / 2) ** 2
        + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2
    )
    return EARTH_RADIUS_M * 2.0 * math.atan2(math.sqrt(h), math.sqrt(1 - h))


def bearing_deg(a: LatLon, b: LatLon) -> float:
    """True bearing (degrees, 0 = N, clockwise) from point ``a`` to ``b``."""
    phi1 = math.radians(a.lat)
    phi2 = math.radians(b.lat)
    dlam = math.radians(b.lon - a.lon)
    x = math.sin(dlam) * math.cos(phi2)
    y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlam)
    return (math.degrees(math.atan2(x, y)) + 360.0) % 360.0


def heading_error_deg(desired_deg: float, actual_deg: float) -> float:
    """Signed shortest angular difference, in (-180, 180]."""
    return ((desired_deg - actual_deg + 540.0) % 360.0) - 180.0


# ─── Polygon utilities ────────────────────────────────────────────────────────


def centroid(pts: Sequence[Point]) -> Point:
    n = len(pts)
    if n == 0:
        return Point(0.0, 0.0)
    return Point(sum(p.x for p in pts) / n, sum(p.y for p in pts) / n)


def latlon_centroid(pts: Sequence[LatLon]) -> LatLon:
    n = len(pts)
    if n == 0:
        return LatLon(0.0, 0.0)
    return LatLon(sum(p.lat for p in pts) / n, sum(p.lon for p in pts) / n)


def longest_edge_angle(pts: Sequence[Point]) -> float:
    """Return the angle (radians) of the longest edge of a closed polygon."""
    best_len = 0.0
    best_ang = 0.0
    n = len(pts)
    for i in range(n):
        a, b = pts[i], pts[(i + 1) % n]
        dx, dy = b.x - a.x, b.y - a.y
        length = math.hypot(dx, dy)
        if length > best_len:
            best_len = length
            best_ang = math.atan2(dy, dx)
    return best_ang


def point_in_polygon(p: Point, poly: Sequence[Point]) -> bool:
    """Ray-cast test.  ``poly`` is closed implicitly."""
    inside = False
    n = len(poly)
    if n < 3:
        return False
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if (yi > p.y) != (yj > p.y) and p.x < (xj - xi) * (p.y - yi) / ((yj - yi) or 1e-12) + xi:
            inside = not inside
        j = i
    return inside


def polygon_area_m2(pts: Sequence[Point]) -> float:
    """Signed area via the shoelace formula (absolute value returned)."""
    n = len(pts)
    if n < 3:
        return 0.0
    s = 0.0
    for i in range(n):
        x1, y1 = pts[i]
        x2, y2 = pts[(i + 1) % n]
        s += x1 * y2 - x2 * y1
    return abs(s) * 0.5


def shrink_polygon(pts: Sequence[Point], buffer_m: float) -> list[Point]:
    """Inset (or expand, if ``buffer_m`` < 0) a convex-ish polygon.

    Quick-and-dirty: scale each vertex relative to the centroid.  Adequate for
    a nearly-rectangular solar array used as a geofence buffer.
    """
    if len(pts) < 3:
        return list(pts)
    c = centroid(pts)
    out: list[Point] = []
    for p in pts:
        dx, dy = p.x - c.x, p.y - c.y
        d = math.hypot(dx, dy)
        if d <= max(buffer_m, 0.0):
            out.append(c)
            continue
        scale = (d - buffer_m) / d
        out.append(Point(c.x + dx * scale, c.y + dy * scale))
    return out


# ─── Scan-line intersection (used by path planner) ────────────────────────────


def horizontal_intersections(poly: Sequence[Point], y: float) -> list[float]:
    """Return sorted X values where the horizontal line ``y`` crosses ``poly``."""
    xs: list[float] = []
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 <= y < y2) or (y2 <= y < y1):
            t = (y - y1) / ((y2 - y1) or 1e-12)
            xs.append(x1 + t * (x2 - x1))
    xs.sort()
    return xs
