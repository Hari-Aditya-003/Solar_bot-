# Solar Panel Cleaning Robot — v2

Mission planner + autopilot for a solar-panel cleaning robot built around a
Raspberry Pi 5 + Pico 2 W stack, with **GPS** and **non-GPS** modes, a
**QGC-style web UI** (Leaflet + draw + Socket.IO), and a clean modular Python
codebase.

## Architecture

```
mission_planner/
  config.py          frozen-dataclass settings (config.yaml loader)
  geo.py             projection / haversine / polygon utilities
  path_planner.py    boustrophedon coverage path generator
  gps_reader.py      NEO-M9N NMEA reader (background thread)
  sensor_fusion.py   IMU + odometry dead-reckoning, GPS heading filter
  robot_bridge.py    UART/UDP link to the Pico (motor + IMU)
  navigation.py      dual-mode PID waypoint follower
  safety.py          geofence + tilt + comms + battery + e-stop
  mission_store.py   mission JSON persistence
  app.py             Flask + Socket.IO entry point
  run_simulation.py  no-hardware demo
  templates/index.html   QGC-style UI
  static/css/style.css   dark theme
  static/js/app.js       Leaflet map, sliders, telemetry

pico/main.py         MicroPython firmware (L298N + MPU-6050)
tests/               pytest suite
config.yaml          all tunables live here
```

## Quick start

```bash
# 1. install deps (on the Pi or your laptop)
pip install -r requirements.txt

# 2. simulate without hardware
python3 -m mission_planner.run_simulation
# open http://localhost:5000

# 3. run on the Pi
python3 -m mission_planner.app
# add --no-gps or --no-robot for partial setups
```

## What was fixed in v2

* **UI was broken** (`templates/index.docx` instead of an HTML template) →
  full QGC-style map UI with draw, sliders, telemetry HUD, mode toggle.
* **No non-GPS mode** → `sensor_fusion.py` (complementary filter +
  dead-reckoning) and `navigation._tick_non_gps` lane follower.
* **No safety layer** → `safety.py` with geofence (point-in-polygon),
  tilt limit, comms watchdog, battery cut-off, e-stop.
* **Mixed concerns in `app.py`** → each capability is now its own module.
* `print()` everywhere → standard `logging` throughout.
* Loose dicts and tuples → frozen dataclasses, type annotations, `NamedTuple`.
* `path_planner` only had row-spacing → adds `cleaning_width` + `overlap %`.
* Magic numbers → `config.yaml` with `Config` dataclass.
* No tests → 25+ pytest tests for geo, path planner, safety,
  sensor fusion and mission store.

## Hardware

* Raspberry Pi 5 → mission planner, web UI, GPS reader.
* Raspberry Pi Pico 2 W → motor driver (L298N), IMU (MPU-6050), STATUS uplink.
* SmartElex / u-blox NEO-M9N GNSS on `/dev/ttyAMA2 @ 9600`.
* Pico ↔ Pi UART on `/dev/ttyAMA4 @ 115200`.
* Pi 12 V LiPo with 100 k / 33 k divider on Pico GP26 ADC.

See `WIRING.md.docx` (preserved from v1) for full pinouts.

## Running tests

```bash
pip install -e ".[dev]"
pytest -v
```

## Web UI keyboard shortcuts

| Key      | Action            |
|----------|-------------------|
| **G**    | Start mission     |
| **P**    | Pause mission     |
| **Space**| E-stop            |
