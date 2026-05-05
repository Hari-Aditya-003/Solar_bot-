"""JSON-on-disk persistence for missions.

A *mission* is a named bundle of:

* polygon ``boundary`` (list of {lat, lon})
* generated ``waypoints`` (list of {seq, lat, lon})
* the planner ``params`` used to generate it
* ``stats`` (rows / distance / area)
* ``saved_at`` UNIX timestamp

Missions live under ``mission_planner/missions/`` as ``<safe_name>.json``.
"""

from __future__ import annotations

import json
import logging
import re
import time
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Any

log = logging.getLogger(__name__)

DEFAULT_DIR = Path(__file__).parent / "missions"
_SAFE_NAME = re.compile(r"[^A-Za-z0-9 _\-]")


@dataclass(frozen=True)
class MissionMeta:
    filename: str
    name: str
    saved_at: float
    waypoint_count: int
    distance_m: float
    rows: int

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


class MissionStore:
    """File-backed store with safe naming and atomic writes."""

    def __init__(self, directory: Path | str = DEFAULT_DIR) -> None:
        self.dir = Path(directory)
        self.dir.mkdir(parents=True, exist_ok=True)

    # ── Save ──────────────────────────────────────────────────────────────

    def save(self, name: str, payload: dict[str, Any]) -> str:
        safe = _safe_name(name)
        if not safe:
            safe = f"mission_{int(time.time())}"
        payload = {**payload, "name": name, "saved_at": time.time()}
        path = self.dir / f"{safe}.json"
        tmp = path.with_suffix(".json.tmp")
        tmp.write_text(json.dumps(payload, indent=2))
        tmp.replace(path)
        log.info("Mission saved: %s", path.name)
        return path.name

    # ── List ──────────────────────────────────────────────────────────────

    def list(self) -> list[MissionMeta]:
        out: list[MissionMeta] = []
        for f in sorted(self.dir.glob("*.json")):
            try:
                d = json.loads(f.read_text())
                stats = d.get("stats", {})
                out.append(MissionMeta(
                    filename=f.name,
                    name=d.get("name", f.stem),
                    saved_at=float(d.get("saved_at", f.stat().st_mtime)),
                    waypoint_count=len(d.get("waypoints", [])),
                    distance_m=float(stats.get("distance_m", 0.0)),
                    rows=int(stats.get("rows", 0)),
                ))
            except (json.JSONDecodeError, OSError, ValueError) as exc:
                log.warning("Skipping corrupt mission %s: %s", f.name, exc)
        out.sort(key=lambda m: m.saved_at, reverse=True)
        return out

    # ── Load / delete ─────────────────────────────────────────────────────

    def load(self, filename: str) -> dict[str, Any] | None:
        path = self.dir / Path(filename).name  # strip any path traversal
        if not path.exists():
            return None
        try:
            return json.loads(path.read_text())
        except json.JSONDecodeError as exc:
            log.error("Failed to load mission %s: %s", filename, exc)
            return None

    def delete(self, filename: str) -> bool:
        path = self.dir / Path(filename).name
        if path.exists():
            path.unlink()
            log.info("Mission deleted: %s", filename)
            return True
        return False


def _safe_name(name: str) -> str:
    return _SAFE_NAME.sub("", (name or "").strip()).strip(" _-")[:60]
