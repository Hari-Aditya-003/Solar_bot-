"""Drive-and-save boundary capture state.

Lets the operator drive the robot around the field and drop boundary points
from either the radio or the web UI.  The captured polygon can then be fed
into the normal coverage planner.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field

from .geo import LatLon, haversine_m


@dataclass
class BoundaryCapture:
    points: list[LatLon] = field(default_factory=list)
    recording: bool = False
    planned: bool = False
    last_source: str = "none"
    updated_at: float = field(default_factory=time.time)

    @property
    def point_count(self) -> int:
        return len(self.points)

    def clear(self) -> None:
        self.points.clear()
        self.recording = False
        self.planned = False
        self.last_source = "none"
        self.updated_at = time.time()

    def add_point(
        self,
        point: LatLon,
        *,
        source: str,
        min_separation_m: float = 0.25,
    ) -> int:
        if self.planned:
            self.clear()
        if self.points and haversine_m(self.points[-1], point) < min_separation_m:
            raise ValueError("Move a little farther before saving the next point")
        self.points.append(point)
        self.recording = True
        self.planned = False
        self.last_source = source
        self.updated_at = time.time()
        return len(self.points)

    def mark_planned(self, *, source: str) -> None:
        self.recording = False
        self.planned = True
        self.last_source = source
        self.updated_at = time.time()

    def to_dict(self) -> dict:
        return {
            "point_count": self.point_count,
            "recording": self.recording,
            "planned": self.planned,
            "last_source": self.last_source,
            "updated_at": self.updated_at,
            "points": [{"lat": p.lat, "lon": p.lon} for p in self.points],
        }
