# Solar Bot

[![GitHub stars](https://img.shields.io/github/stars/Hari-Aditya-003/Solar_bot-?style=flat-square)](https://github.com/Hari-Aditya-003/Solar_bot-/stargazers)
[![GitHub forks](https://img.shields.io/github/forks/Hari-Aditya-003/Solar_bot-?style=flat-square)](https://github.com/Hari-Aditya-003/Solar_bot-/forks)
[![Default branch](https://img.shields.io/badge/branch-main-2563eb?style=flat-square)](https://github.com/Hari-Aditya-003/Solar_bot-/tree/main)

A solar-panel cleaning robot project with a Raspberry Pi mission planner, a Pico motor controller, GPS and non-GPS navigation, and a browser-based control interface.

This repository contains the application in **`solar_Bot/`**. Start with simulation, then follow the hardware and deployment guides before connecting a robot.

## Capabilities

- Draw a field boundary and generate a coverage path.
- Switch between GPS-assisted and non-GPS navigation.
- View robot position and telemetry through Flask and Socket.IO.
- Connect a Raspberry Pi 5 to a Pico 2 W over UART or UDP.
- Configure geofence, tilt, communications, and battery limits, and trigger emergency stop.
- Save missions and use diagnostic tools during hardware bring-up.

## Quick start: simulation

Requires Python 3.10 or newer. Run these commands from a terminal:

```bash
git clone https://github.com/Hari-Aditya-003/Solar_bot-.git
cd Solar_bot-
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r solar_Bot/requirements.txt
cd solar_Bot
python -m mission_planner.run_simulation
```

Open [localhost:5000](http://localhost:5000). Simulation provides synthetic position updates and needs no GPS receiver or motor controller. The default configuration listens on all interfaces; use an isolated operator network with access limited to authorized devices.

## Project layout

| Path | Purpose |
| --- | --- |
| [solar_Bot/mission_planner](solar_Bot/mission_planner) | Web server, navigation, planning, bridges, and safety logic |
| [solar_Bot/pico/main.py](solar_Bot/pico/main.py) | MicroPython motor-controller firmware |
| [solar_Bot/mission_planner/config.yaml](solar_Bot/mission_planner/config.yaml) | Runtime configuration |
| [solar_Bot/tools](solar_Bot/tools) | Hardware diagnostics |
| [solar_Bot/tests](solar_Bot/tests) | Python test suite |
| [solar_Bot/docs](solar_Bot/docs) | Wiring, references, and screenshots |
| [solar_Bot/systemd](solar_Bot/systemd) | Raspberry Pi service template |

## Documentation

- [Application and architecture guide](solar_Bot/README.md)
- [Documentation index](solar_Bot/docs/README.md)
- [Wiring and verification](solar_Bot/docs/WIRING.md)
- [Raspberry Pi deployment, service setup, and rollback](docs/DEPLOYMENT.md)
- [Diagnostic tools](solar_Bot/tools/README.md)
- [Contributing](CONTRIBUTING.md)

## Development checks

From the repository root, with the virtual environment activated:

```bash
python -m pip install -e './solar_Bot[dev]'
cd solar_Bot
python -m pytest
```

A passing software suite does not verify physical wiring, motor direction, braking, or sensor calibration. Perform the wiring guide's checks with the drive wheels raised before operating on panels.

## Deployment status

The repository provides simulation, Pico firmware, and a Raspberry Pi systemd template. Hardware installation is site-specific: the checked-in service contains the original user's paths and must be adapted before installation. The current control server is intended for an isolated robot network and has no user-login boundary. Restrict port 5000 to authorized operator devices at the firewall before exposing controls.

## Stars, forks, and branches

Star the repository if it is useful, fork it to experiment, and submit focused pull requests against `main`. Browse the [current branches](https://github.com/Hari-Aditya-003/Solar_bot-/branches). The badges above show live stars and forks; branch count is not a measure of project quality.
