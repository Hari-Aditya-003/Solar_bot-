"""
Solar Panel Cleaning Robot — Pico 2 W Motor Controller
MicroPython firmware for Raspberry Pi Pico 2 W

Receives ASCII commands from Raspberry Pi 5 over UART1 and drives
two DC motors via an L298N dual H-bridge driver.
Reads MPU-6050 over I2C0 for tilt protection and heading estimation.

─────────────────────────────────────────────────────────────────────────────
Wiring — UART to RPi 5
─────────────────────────────────────────────────────────────────────────────
  Pico Pin 16 (GP12 UART0 TX)  →  RPi5 Pin 33 (GPIO13 RX)
  Pico Pin 17 (GP13 UART0 RX)  ←  RPi5 Pin 32 (GPIO12 TX)
  Common GND

  Enable on RPi 5: dtoverlay=uart4-pi5 in /boot/firmware/config.txt
  Run: python3 tools/rc_drive.py --pico /dev/ttyAMA4

─────────────────────────────────────────────────────────────────────────────
Wiring — MPU-6050 IMU  (I2C0)
─────────────────────────────────────────────────────────────────────────────
  MPU-6050 VCC  →  Pico Pin 36  (3V3 OUT)
  MPU-6050 GND  →  Pico Pin 38  (GND)
  MPU-6050 SDA  →  Pico GP8    Pin 11   (I2C0 SDA)
  MPU-6050 SCL  →  Pico GP9    Pin 12   (I2C0 SCL)
  MPU-6050 AD0  →  GND                  (I2C address = 0x68)
  MPU-6050 INT  →  Pico GP10   Pin 14   (optional interrupt)

─────────────────────────────────────────────────────────────────────────────
Wiring — L298N Motor Driver
─────────────────────────────────────────────────────────────────────────────
  Left  motor:  IN1=GP0  IN2=GP1  ENA=GP2(PWM)
  Right motor:  IN3=GP3  IN4=GP6  ENB=GP7(PWM)
  Motor power:  12V battery → L298N 12V terminal
  Logic power:  L298N 5V out → Pico VSYS
  GND:          all grounds tied together

─────────────────────────────────────────────────────────────────────────────
Battery monitor
─────────────────────────────────────────────────────────────────────────────
  Battery+ → R1(100kΩ) → GP26(ADC0)
                       ↓
                    R2(33kΩ)
                       ↓
                      GND

─────────────────────────────────────────────────────────────────────────────
Command protocol  (UART 115200 8N1, newline-terminated)
─────────────────────────────────────────────────────────────────────────────
  STOP
  MOVE <steer -100..100> <throttle 0..100>
  MISSION_START | MISSION_PAUSE | MISSION_ABORT

Reply (100 ms interval):
  STATUS 0 0 <yaw> <speed_cms> <battery_mv> <wp_reached> <roll> <pitch>
  yaw   = gyro-integrated heading (degrees, relative, drifts over time)
  roll  = tilt left/right  (degrees, +right)
  pitch = tilt front/back  (degrees, +nose-up)
"""

import asyncio
import math
import sys
import time

from machine import ADC, I2C, PWM, UART, Pin

# ─── Motor driver pins ────────────────────────────────────────────────────────

_L_IN1 = Pin(0, Pin.OUT)
_L_IN2 = Pin(1, Pin.OUT)
_L_EN  = PWM(Pin(2))

_R_IN1 = Pin(3, Pin.OUT)
_R_IN2 = Pin(6, Pin.OUT)
_R_EN  = PWM(Pin(7))

_L_EN.freq(1000)
_R_EN.freq(1000)

# ─── Battery ADC ──────────────────────────────────────────────────────────────

_BATT_ADC     = ADC(26)
_BATT_DIVIDER = (100 + 33) / 33   # voltage divider inverse

# ─── UART (to RPi 5) ──────────────────────────────────────────────────────────

_uart = UART(0, baudrate=115200, tx=Pin(12), rx=Pin(13), rxbuf=512)

# ─── MPU-6050 I2C ─────────────────────────────────────────────────────────────

_MPU_ADDR = 0x68          # AD0 pin tied to GND
_i2c      = I2C(0, sda=Pin(8), scl=Pin(9), freq=400_000)

# MPU-6050 register addresses
_REG_PWR_MGMT_1  = 0x6B
_REG_ACCEL_XOUT  = 0x3B   # 6 bytes: AX_H AX_L AY_H AY_L AZ_H AZ_L
_REG_GYRO_XOUT   = 0x43   # 6 bytes: GX_H GX_L GY_H GY_L GZ_H GZ_L
_REG_WHO_AM_I    = 0x75

# Scale factors (default ranges)
_ACCEL_SCALE = 16384.0    # ±2g  → LSB/g
_GYRO_SCALE  =   131.0    # ±250°/s → LSB/(°/s)

# Complementary filter coefficient (0.98 = trust gyro 98%, accel 2%)
_ALPHA = 0.98

# Tilt safety limit — stop motors if robot tilts beyond this (degrees)
TILT_LIMIT_DEG = 30.0

# ─── Robot physical constants (200RPM TT motor, 65mm wheel) ──────────────────
_WHEEL_CIRC_CM = 20.4    # π × 6.5 cm
_MAX_SPEED_CMS = 68.0    # 200 RPM / 60 × 20.4 cm


def _mpu_init() -> bool:
    """Wake up MPU-6050 and verify it responds. Returns True on success."""
    try:
        devs = _i2c.scan()
        if _MPU_ADDR not in devs:
            print(f"[MPU] Not found on I2C bus. Scanned: {devs}")
            return False
        # Clear sleep bit in PWR_MGMT_1
        _i2c.writeto_mem(_MPU_ADDR, _REG_PWR_MGMT_1, b'\x00')
        time.sleep_ms(100)
        who = _i2c.readfrom_mem(_MPU_ADDR, _REG_WHO_AM_I, 1)[0]
        if who != 0x68:
            print(f"[MPU] Unexpected WHO_AM_I: 0x{who:02X}")
            return False
        print("[MPU] MPU-6050 initialised OK")
        return True
    except Exception as e:
        print(f"[MPU] Init error: {e}")
        return False


def _read_signed16(data: bytes, offset: int) -> int:
    val = (data[offset] << 8) | data[offset + 1]
    return val - 65536 if val > 32767 else val


def _mpu_read_raw():
    """Return (ax, ay, az, gx, gy, gz) in physical units (g and °/s)."""
    try:
        ab = _i2c.readfrom_mem(_MPU_ADDR, _REG_ACCEL_XOUT, 6)
        gb = _i2c.readfrom_mem(_MPU_ADDR, _REG_GYRO_XOUT,  6)
        ax = _read_signed16(ab, 0) / _ACCEL_SCALE
        ay = _read_signed16(ab, 2) / _ACCEL_SCALE
        az = _read_signed16(ab, 4) / _ACCEL_SCALE
        gx = _read_signed16(gb, 0) / _GYRO_SCALE
        gy = _read_signed16(gb, 2) / _GYRO_SCALE
        gz = _read_signed16(gb, 4) / _GYRO_SCALE
        return ax, ay, az, gx, gy, gz
    except Exception:
        return 0.0, 0.0, 1.0, 0.0, 0.0, 0.0


# ─── Complementary filter state ───────────────────────────────────────────────

_roll  = 0.0   # degrees  +right
_pitch = 0.0   # degrees  +nose-up
_yaw   = 0.0   # degrees  gyro-integrated (relative, drifts)
_last_imu_time = time.ticks_ms()
_mpu_ok = False


def _update_imu():
    """Read MPU-6050 and run complementary filter. Updates _roll, _pitch, _yaw."""
    global _roll, _pitch, _yaw, _last_imu_time

    now = time.ticks_ms()
    dt  = time.ticks_diff(now, _last_imu_time) / 1000.0
    _last_imu_time = now

    if dt <= 0 or dt > 0.5:
        return

    ax, ay, az, gx, gy, gz = _mpu_read_raw()

    # Accelerometer angle (absolute but noisy)
    accel_roll  = math.atan2(ay, az)       * 57.2958
    accel_pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az)) * 57.2958

    # Complementary filter: blend gyro integration with accelerometer correction
    _roll  = _ALPHA * (_roll  + gx * dt) + (1 - _ALPHA) * accel_roll
    _pitch = _ALPHA * (_pitch + gy * dt) + (1 - _ALPHA) * accel_pitch

    # Yaw from gyro only (no magnetometer — will drift slowly)
    _yaw = (_yaw + gz * dt) % 360.0


# ─── Shared drive state ───────────────────────────────────────────────────────

_steer    = 0
_throttle = 0
_running  = False
_stopped  = True


# ─── Motor control ────────────────────────────────────────────────────────────

def _set_motor(in1, in2, en, speed: int):
    speed = max(-100, min(100, speed))
    duty  = int(abs(speed) / 100 * 65535)
    if speed > 0:
        in1.value(1)
        in2.value(0)
    elif speed < 0:
        in1.value(0)
        in2.value(1)
    else:
        in1.value(0)
        in2.value(0)
    en.duty_u16(duty)


def tank_drive(steer: int, throttle: int):
    left  = throttle - steer
    right = throttle + steer
    peak  = max(abs(left), abs(right), 100)
    _set_motor(_L_IN1, _L_IN2, _L_EN, int(left  / peak * 100))
    _set_motor(_R_IN1, _R_IN2, _R_EN, int(right / peak * 100))


def stop_motors():
    _set_motor(_L_IN1, _L_IN2, _L_EN, 0)
    _set_motor(_R_IN1, _R_IN2, _R_EN, 0)


def read_battery_mv() -> int:
    raw = _BATT_ADC.read_u16()
    return int((raw / 65535) * 3.3 * _BATT_DIVIDER * 1000)


# ─── Command handler ──────────────────────────────────────────────────────────

def handle_command(line: str):
    global _steer, _throttle, _running, _stopped

    parts = line.split()
    if not parts:
        return
    cmd = parts[0].upper()

    if cmd == "STOP":
        _steer = _throttle = 0
        _running = False
        _stopped = True
        stop_motors()

    elif cmd == "MOVE" and len(parts) == 3:
        try:
            s = int(parts[1])
            t = int(parts[2])
            _steer = s
            _throttle = t
            _stopped = False
            tank_drive(s, t)
        except ValueError:
            pass

    elif cmd == "MISSION_START":
        _running = True
        _stopped = False

    elif cmd == "MISSION_PAUSE":
        _running = False
        _stopped = True
        stop_motors()

    elif cmd == "MISSION_ABORT":
        _running = False
        _stopped = True
        _steer = _throttle = 0
        stop_motors()


# ─── Async tasks ──────────────────────────────────────────────────────────────

async def task_imu():
    """Update IMU at ~50 Hz and apply tilt safety cut-off."""
    global _stopped
    while True:
        if _mpu_ok:
            _update_imu()
            # Safety: stop if robot tilts dangerously on panel edge
            if (abs(_roll) > TILT_LIMIT_DEG or abs(_pitch) > TILT_LIMIT_DEG):
                if not _stopped:
                    stop_motors()
                    _stopped = True
        await asyncio.sleep(0.02)   # 50 Hz


async def task_read_commands():
    """Read newline-terminated commands from UART."""
    buf = b""
    while True:
        n = _uart.any()
        if n:
            chunk = _uart.read(n)
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                text = line.decode("ascii", "ignore").strip()
                if text and not text.startswith("STATUS"):
                    handle_command(text)
        await asyncio.sleep(0.005)


async def task_send_status():
    """Send STATUS at 10 Hz to both UART1 (GPIO) and USB stdout."""
    while True:
        bat       = read_battery_mv()
        speed_cms = abs(_throttle) / 100.0 * _MAX_SPEED_CMS
        msg = (f"STATUS {_steer} {_throttle} {_yaw:.1f} {speed_cms:.1f} {bat} 0 "
               f"{_roll:.1f} {_pitch:.1f}\n")
        _uart.write(msg.encode())
        try:
            sys.stdout.write(msg)
        except Exception:
            pass
        await asyncio.sleep(0.1)


async def task_watchdog():
    """Stop motors if no MOVE command received for > 1 s."""
    global _stopped
    last_move_t = time.ticks_ms()
    prev_thr    = _throttle

    while True:
        if _throttle != prev_thr and _throttle != 0:
            last_move_t = time.ticks_ms()
        prev_thr = _throttle

        if time.ticks_diff(time.ticks_ms(), last_move_t) > 1000 and not _stopped:
            stop_motors()
            _stopped = True

        await asyncio.sleep(0.25)


# ─── Main ─────────────────────────────────────────────────────────────────────

async def main():
    global _mpu_ok
    stop_motors()
    _mpu_ok = _mpu_init()
    print("Solar Bot Pico 2W — ready  (IMU:", "OK" if _mpu_ok else "NOT FOUND", ")")
    await asyncio.gather(
        task_imu(),
        task_read_commands(),
        task_send_status(),
        task_watchdog(),
    )

asyncio.run(main())
