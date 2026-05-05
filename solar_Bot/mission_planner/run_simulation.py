"""Standalone simulator — drive the planner without GPS or a Pico.

Spins up the Flask server with hardware disabled, then nudges a fake robot
along a sample boundary so you can see the UI light up end-to-end.

Run:
    python3 -m mission_planner.run_simulation
    open http://localhost:5000
"""

from __future__ import annotations

import logging
import math
import threading
import time

from .app import create_app
from .config import load_config
from .geo import LatLon, M_PER_DEG_LAT
from .navigation import NavMode

# Sample plot — a small rectangle in Mumbai (≈ 8 m × 5 m).
SAMPLE_BOUNDARY = [
    LatLon(19.07600, 72.87770),
    LatLon(19.07604, 72.87770),
    LatLon(19.07604, 72.87778),
    LatLon(19.07600, 72.87778),
]


def _drive_fake_robot(estimator) -> None:
    """Walk a virtual robot at 30 cm/s along a snake."""
    lat, lon = SAMPLE_BOUNDARY[0]
    heading = 90.0
    speed_ms = 0.30
    while True:
        time.sleep(0.1)
        # Simple sine-shaped trajectory (just for visual feedback).
        dx = speed_ms * 0.1 * math.sin(math.radians(heading))
        dy = speed_ms * 0.1 * math.cos(math.radians(heading))
        lat += dy / M_PER_DEG_LAT
        lon += dx / (M_PER_DEG_LAT * math.cos(math.radians(lat)))
        heading = (heading + 1.5) % 360.0
        estimator.update_gps(lat, lon, heading, speed_ms, has_fix=True)


def main() -> None:
    cfg = load_config()
    logging.basicConfig(level="INFO",
                        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s")
    app, sio, nav = create_app(cfg, with_gps=False, with_robot=False)

    nav.set_mission("Sample plot",
                    list(SAMPLE_BOUNDARY),
                    list(SAMPLE_BOUNDARY))  # placeholder waypoints
    nav.set_mode(NavMode.GPS)

    # Hand the navigator a fake estimator that moves on its own.
    threading.Thread(target=_drive_fake_robot, args=(nav.estimator,),
                     daemon=True, name="FakeRobot").start()

    print("Simulator running — open http://localhost:5000")
    sio.run(app, host=cfg.server.host, port=cfg.server.port,
            debug=False, use_reloader=False, allow_unsafe_werkzeug=True)


if __name__ == "__main__":  # pragma: no cover
    main()
