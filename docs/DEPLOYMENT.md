# Solar Bot: Raspberry Pi deployment

This guide runs the existing application as a systemd service on the robot's Raspberry Pi. The laptop simulator is documented in the [root README](../README.md).

## Deployment prerequisites

- Raspberry Pi OS with Python 3.10 or newer, Git, and venv support.
- Wiring and UARTs configured using [WIRING.md](../solar_Bot/docs/WIRING.md).
- Pico 2 W flashed with the MicroPython firmware from `solar_Bot/pico/main.py`.
- Correct serial-port names and access for the service account.
- An isolated operator network and firewall rules allowing port 5000 only from authorized operator devices. The Flask/Socket.IO server has no user-authentication boundary and currently binds to `0.0.0.0:5000`.

Before network access, configure an allowlist for the authorized operator addresses and deny other clients at the Pi's firewall. Verify both allowed and denied access. Until that is in place, bind to `127.0.0.1` and keep controls local. Do not expose this control service directly to the public internet. Use simulation first and verify physical stops and motor direction before powering a robot on panels.

## 1. Install source and dependencies

The commands below use a checkout in your home directory:

```bash
sudo apt-get update
sudo apt-get install -y git python3-venv
git clone https://github.com/Hari-Aditya-003/Solar_bot-.git "$HOME/Solar_bot-"
cd "$HOME/Solar_bot-"
python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -r solar_Bot/requirements.txt
git rev-parse HEAD
```

Record the commit SHA and installed dependency versions with your field log. On systems where the serial device is owned by `dialout`, add the service account to that group and start a fresh login session:

```bash
sudo usermod -aG dialout "$(id -un)"
```

This command assumes the service will run as the current login account. If `User=` names another account, add that actual service account to the device group or include the group in its `SupplementaryGroups=` setting. Confirm the actual device permissions; distribution-specific group names can differ.

## 2. Configure and verify manually

Review `solar_Bot/mission_planner/config.yaml`: GPS, radio, and Pico ports/baud rates; UART or UDP transport; wheel geometry; cleaning width; navigation; safety limits; and server binding.

From the checkout:

```bash
source .venv/bin/activate
cd solar_Bot
python -m pytest
python -m mission_planner.app --no-gps --no-robot --host 127.0.0.1
```

Use the standalone simulator for synthetic telemetry. The application supports `--config PATH`, `--no-gps`, `--no-robot`, `--host`, and `--port`. Disabling the GPS/Pico link does not necessarily disable every other serial peripheral; read logs for radio-port access failures.

Stop the manual process before starting systemd. Connect to the configured robot hardware only after the wiring checks pass. The `--ignore-battery` option is an indoor-test override and must not be part of the normal field service.

## 3. Adapt the service file

The checked-in [service template](../solar_Bot/systemd/solarbot-mission.service) hard-codes the original user's `/home/solar_bot/Desktop/Solar_bot-` checkout. The helper script copies that template unchanged and immediately restarts it. Adapt a local copy before installation:

```bash
cp systemd/solarbot-mission.service /tmp/solarbot-mission.service
nano /tmp/solarbot-mission.service
```

These commands assume the current directory is `solar_Bot/`. Replace the following fields with actual values:

| Field | Required value |
| --- | --- |
| `User` / `Group` | Your robot service account and primary group |
| `WorkingDirectory` | Absolute path to the checkout's `solar_Bot` directory |
| `ExecStart` | Absolute path to the checkout's `.venv/bin/python`, followed by `-m mission_planner.app` |
| `SupplementaryGroups` | Only hardware groups that exist and are needed on this Pi |

If using a configuration outside the checkout, append `--config /absolute/path/config.yaml` to `ExecStart`. Preserve `PYTHONUNBUFFERED=1` for readable logs. Verify that the service account can traverse the checkout, execute its virtual environment, and write the configured mission/path directories; avoid world-writable permissions.

## 4. Install and inspect

```bash
sudo systemd-analyze verify /tmp/solarbot-mission.service
sudo install -m 0644 /tmp/solarbot-mission.service /etc/systemd/system/solarbot-mission.service
sudo systemctl daemon-reload
sudo systemctl enable solarbot-mission.service
sudo systemctl restart solarbot-mission.service
sudo systemctl status solarbot-mission.service --no-pager
journalctl -u solarbot-mission.service -n 100 --no-pager
```

Open `http://<pi-address>:5000` from the robot network. Do not assume the helper's printed `192.168.2.6` address matches your Pi. Hotspot setup scripts change network configuration; inspect them before using them on an existing network.

## Operational acceptance

Verify telemetry, GPS fix where required, Pico communications, boundary loading, path planning, pause, emergency stop, tilt limit, communications timeout, and battery reporting. Start with raised drive wheels, then controlled low-speed movement in a clear area. Software tests do not establish field readiness.

The default mission store and saved paths live under the application tree; back them up together with your active configuration before upgrading. Use the hardware diagnostics listed in [tools/README.md](../solar_Bot/tools/README.md).

## Update and rollback

1. Stop the service and place the robot in a physically safe state.
2. Back up configuration, saved mission/path files, and the previous commit SHA.
3. Fetch and check out the reviewed commit; reinstall dependencies if required.
4. Run tests from `solar_Bot/`, then restart and inspect logs.
5. If acceptance fails, stop the service, restore the previous source/dependency environment and compatible configuration/data, and repeat checks before operating.

```bash
sudo systemctl stop solarbot-mission.service
sudo systemctl start solarbot-mission.service
journalctl -u solarbot-mission.service -f
```

The first two commands show the stop/start controls; perform the intervening update or recovery work before starting again.
