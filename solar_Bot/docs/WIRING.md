# Solar Bot Wiring Guide

Updated to match the current project code and runtime configuration.

Hardware stack:

- Raspberry Pi 5
- Raspberry Pi Pico 2 W
- RadioMaster RP3 V2 receiver
- SmartElex u-blox GNSS GPS over USB
- MPU-6050 IMU
- L298N dual motor driver
- 12 V / 14.8 V motor battery, depending on your pack

## 1. Active Ports Used By The Code

| Device | Raspberry Pi port | Baud | Code/config |
|---|---:|---:|---|
| USB GPS | `/dev/ttyACM0` usually, auto-detected | 9600 | `mission_planner/gps_reader.py`, `config.yaml gps_port: auto` |
| RadioMaster RP3 V2 | `/dev/ttyAMA0` | 460800 | `mission_planner/radio_bridge.py` |
| Pico 2 W motor controller | `/dev/ttyAMA4` | 115200 | `mission_planner/robot_bridge.py` |

The web app auto-start service runs:

```bash
/home/solar_bot/Desktop/Solar_bot-/.venv/bin/python -m mission_planner.app
```

Open the UI at:

```text
http://192.168.2.6:5000
```

## 2. USB GPS To Raspberry Pi 5

Current working connection:

| GPS module | Raspberry Pi 5 |
|---|---|
| USB data/power cable | Pi USB port |

Expected Linux device:

```text
/dev/ttyACM0
```

Useful checks:

```bash
lsusb
ls -l /dev/ttyACM0 /dev/serial/by-id/*
../.venv/bin/python tools/check_gps.py --port /dev/ttyACM0 --seconds 10
```

Expected result:

```text
FIX OK
```

Notes:

- Use a real USB data cable, not a charge-only cable.
- GPS LEDs only prove power/status. Linux must also show `/dev/ttyACM0`.
- The app config uses `gps_port: auto`, so USB GPS is preferred automatically.

Optional UART GPS wiring is not the current active setup. If used later, wire GPS TX to a Pi UART RX and update `mission_planner/config.yaml`.

## 3. RadioMaster RP3 V2 To Raspberry Pi 5

The mission planner expects MAVLink RC messages from the receiver:

```text
/dev/ttyAMA0 @ 460800
```

Wiring:

| RadioMaster RP3 V2 | Raspberry Pi 5 | Notes |
|---|---|---|
| TX | Pin 10, GPIO15 RX | Required, receiver data to Pi |
| RX | Pin 8, GPIO14 TX | Optional telemetry back to receiver |
| 5V | Pin 2 or Pin 4, 5V | Receiver power |
| GND | Pin 6, GND | Common ground |

Test:

```bash
../.venv/bin/python tools/check_rp3v2.py
```

Mission planner mapping:

| Radio channel | Function |
|---:|---|
| CH4 | Steering, left gimbal horizontal |
| CH2 | Throttle, right gimbal vertical |
| CH6 | Save boundary point |
| CH7 | Generate/start route |
| CH8 | Abort/reset safety action |

Important:

- The app uses MAVLink at `460800`.
- If the receiver is configured for CRSF only, raw bytes may appear but the app will still show radio offline.
- The transmitter and receiver must be bound and outputting valid RC frames.

## 4. Raspberry Pi 5 To Pico 2 W UART

Current Pico firmware uses:

```python
UART(0, baudrate=115200, tx=Pin(12), rx=Pin(13))
```

Wiring:

| Pico 2 W | Raspberry Pi 5 | Notes |
|---|---|---|
| Pin 16, GP12 UART0 TX | Pin 33, GPIO13 RX | Pico STATUS data to Pi |
| Pin 17, GP13 UART0 RX | Pin 32, GPIO12 TX | Pi MOVE/STOP commands to Pico |
| GND | Any Pi GND, for example Pin 39 | Required common ground |

Pi UART device:

```text
/dev/ttyAMA4 @ 115200
```

Required Pi config:

```text
dtoverlay=uart4-pi5
```

Add it to:

```text
/boot/firmware/config.txt
```

Then reboot.

Test Pico link:

```bash
../.venv/bin/python tools/check_pico_uart.py --port /dev/ttyAMA4
```

The app expects Pico telemetry lines like:

```text
STATUS 0 0 <yaw> <speed_cms> <battery_mv> <wp_reached> <roll> <pitch>
```

If Pico is offline in the UI, battery and IMU will also show offline/0 because both come through this UART telemetry.

## 5. MPU-6050 IMU To Pico 2 W

The current firmware uses Pico I2C0:

```python
I2C(0, sda=Pin(8), scl=Pin(9), freq=400000)
```

Wiring:

| MPU-6050 | Pico 2 W | Notes |
|---|---|---|
| VCC | Pin 36, 3V3 OUT | Use 3.3 V |
| GND | Pin 38, GND | Common ground |
| SDA | Pin 11, GP8 | I2C0 SDA |
| SCL | Pin 12, GP9 | I2C0 SCL |
| AD0 | GND | I2C address `0x68` |
| INT | Pin 14, GP10 | Optional, not required by current code |
| XDA / XCL | Not connected | Not used |

The firmware checks for MPU-6050 at address:

```text
0x68
```

The UI displays:

- IMU yaw
- Roll
- Pitch
- Tilt state

But only after the Pico sends valid `STATUS` lines to the Pi.

## 6. Pico 2 W To L298N Motor Driver

The current firmware pins are:

```python
Left:  IN1=GP0, IN2=GP1, ENA=GP2 PWM
Right: IN3=GP3, IN4=GP6, ENB=GP7 PWM
```

Wiring:

| Pico 2 W | L298N | Function |
|---|---|---|
| GP0, Pin 1 | IN1 | Left motor direction A |
| GP1, Pin 2 | IN2 | Left motor direction B |
| GP2, Pin 4 | ENA | Left motor PWM speed |
| GP3, Pin 5 | IN3 | Right motor direction A |
| GP6, Pin 9 | IN4 | Right motor direction B |
| GP7, Pin 10 | ENB | Right motor PWM speed |
| GND | GND | Common ground |

Power wiring:

| Power | L298N / Pico | Notes |
|---|---|---|
| Battery positive | L298N `12V` / `VIN` motor terminal | Motor power |
| Battery negative | L298N GND | Also tied to Pico GND |
| L298N 5V out | Pico VSYS | Only if the L298N 5V regulator is enabled and stable |
| L298N Motor A | Left motor wires | Swap wires if direction is reversed |
| L298N Motor B | Right motor wires | Swap wires if direction is reversed |

Important:

- For software PWM speed control, remove ENA/ENB jumpers and wire ENA to GP2, ENB to GP7.
- If ENA/ENB jumpers are installed, motors may run full speed and code speed control will not work correctly.
- All grounds must be connected together: Pi, Pico, L298N, receiver, and sensors.

Motor test:

```bash
../.venv/bin/python tools/check_motors.py --port /dev/ttyAMA4 --speed 30
```

## 7. Battery Voltage Divider To Pico

Firmware ADC:

```python
ADC(26)
```

Voltage divider:

```text
Battery +  -> R1 100k -> GP26 ADC0
                         |
                       R2 33k
                         |
Battery - / GND ---------+
```

Wiring:

| Divider node | Pico |
|---|---|
| R1/R2 midpoint | GP26, ADC0 |
| Battery negative | Pico GND |

Notes:

- The divider scales battery voltage down for the Pico 3.3 V ADC.
- Do not connect battery positive directly to GP26.
- The UI battery percentage is 0 if Pico telemetry is missing.

## 8. Power And Ground Rules

Required common ground:

```text
Raspberry Pi GND
Pico GND
L298N GND
Radio receiver GND
GPS GND through USB
Battery negative
```

Recommended power layout:

- Raspberry Pi 5: official/strong USB-C supply.
- Motors: separate battery into L298N motor input.
- Pico: USB during testing, or stable 5 V into VSYS.
- Radio receiver: Pi 5 V pin.
- GPS: Pi USB port.

Do not power motors from the Raspberry Pi 5.

If the Pi shows undervoltage warnings, USB GPS/radio/Pico links may disconnect.

## 9. Raspberry Pi 5 Setup

Enable UART4 for Pico:

```bash
sudo nano /boot/firmware/config.txt
```

Add:

```text
dtoverlay=uart4-pi5
```

Then:

```bash
sudo reboot
```

Fix UART permissions:

```bash
cd /home/solar_bot/Desktop/Solar_bot-/solar_Bot
sudo bash tools/fix_uart_permissions.sh
sudo usermod -aG dialout solar_bot
```

Log out/reboot after group changes.

## 10. Autostart Service

The project now auto-starts with:

```text
solarbot-mission.service
```

Commands:

```bash
sudo systemctl status solarbot-mission.service --no-pager
sudo systemctl restart solarbot-mission.service
sudo systemctl stop solarbot-mission.service
journalctl -u solarbot-mission.service -f
```

If the service is running, do not also run `python -m mission_planner.app` manually, because port 5000 will already be in use.

## 11. Quick Verification Commands

GPS:

```bash
../.venv/bin/python tools/check_gps.py --port /dev/ttyACM0 --seconds 10
```

Pico:

```bash
../.venv/bin/python tools/check_pico_uart.py --port /dev/ttyAMA4
```

Motors:

```bash
../.venv/bin/python tools/check_motors.py --port /dev/ttyAMA4 --speed 30
```

Radio:

```bash
../.venv/bin/python tools/check_rp3v2.py
```

Web app health:

```bash
curl http://127.0.0.1:5000/api/health
```

## 12. Expected UI Meaning

| UI item | Meaning |
|---|---|
| GPS good | USB GPS is connected and has fix |
| Pico offline | No valid `STATUS` lines from Pico on `/dev/ttyAMA4` |
| Radio offline | No valid MAVLink RC frames from `/dev/ttyAMA0` |
| Battery 0% | Pico telemetry missing or battery ADC reads 0 |
| IMU offline | Pico telemetry missing, or MPU-6050 not found |
| Green trail while idle | Should not happen after the GPS drift UI fix |

## 13. Full System Summary

```text
USB GPS        -> Raspberry Pi 5 USB        -> /dev/ttyACM0 @ 9600
RP3 V2 TX      -> Pi GPIO15 RX, Pin 10      -> /dev/ttyAMA0 @ 460800
Pi GPIO12 TX   -> Pico GP13 RX, Pin 17      -> /dev/ttyAMA4 @ 115200
Pico GP12 TX   -> Pi GPIO13 RX, Pin 33      -> /dev/ttyAMA4 @ 115200
MPU-6050       -> Pico GP8/GP9 I2C0         -> 0x68
Pico GP0/1/2   -> L298N left motor control
Pico GP3/6/7   -> L298N right motor control
Battery divider-> Pico GP26 ADC0
L298N          -> Left and right DC motors
```
