#!/usr/bin/env python3
"""
Solar Bot RC Drive + Boundary Mission Planner
RadioMaster RP3 V2 → Pico 2W Motor Control  —  Raspberry Pi 5

Connections:
  RC receiver  : /dev/ttyAMA0  @ 460800 (RadioMaster ELRS MAVLink)
  Pico UART    : /dev/ttyAMA4  @ 115200
  GPS (USB)    : /dev/ttyUSB0  @ 9600

Channel mapping:
  CH2  Right stick U/D   : throttle  1000=rev 1500=stop 2000=fwd
  CH4  Left  stick L/R   : steering  1000=left 1500=ctr 2000=right
  CH5 (SA) HIGH          : ARMED
  CH6 (SB) LOW→HIGH edge : mark boundary corner (GPS fix required)
  CH7 (SC) LOW→HIGH      : close boundary + generate path  (≥3 corners)
                           second flip → start autonomous mission (if armed)
           HIGH→LOW      : pause autonomous mission
  CH8 (SD) LOW→HIGH      : ABORT and reset everything

Boundary workflow:
  1. Drive robot to each corner of solar panel area
  2. Flip SB at every corner to mark it  (GPS fix required)
  3. When all corners marked, flip SC  → lawnmower path auto-generated
  4. ARM (SA↑), position robot, flip SC again  → autonomous sweep starts
  5. Flip SC↓ to pause;  SD to abort/reset

Path files: paths/mission_YYYYMMDD_HHMMSS.json

Usage:
  python3 rc_drive.py                         # full system
  python3 rc_drive.py --no-gps               # skip GPS, use dead-reckoning odometry
  python3 rc_drive.py --spacing 0.4          # 40 cm sweep line spacing
  python3 rc_drive.py --pico /dev/ttyAMA4
"""

import argparse
import json
import math
import signal
import sys
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path

# ── Constants ─────────────────────────────────────────────────────────────────

RC_PORT   = "/dev/ttyAMA0"
RC_BAUD   = 460_800
PICO_PORT = "/dev/ttyAMA4"
PICO_BAUD = 115_200
GPS_PORT  = "/dev/ttyUSB0"
GPS_BAUD  = 9_600

DEADZONE         = 60
FAILSAFE_S       = 0.5
SEND_HZ          = 20
PATH_DIR         = Path(__file__).parent / "paths"

# ── Robot physical specs — 200RPM TT motor, 65mm wheel, 12×12cm chassis ──────
WHEEL_DIAM_M     = 0.065   # 65 mm plastic TT wheel
WHEELBASE_M      = 0.10    # ~10 cm between left/right wheels
MAX_SPEED_MS     = 0.68    # π × 0.065 × 200/60  (at 100% throttle)

# ── Autonomous navigation tuning ──────────────────────────────────────────────
ARRIVAL_RADIUS_M = 0.30   # arrive within 30 cm of waypoint (was 0.5)
NAV_THROTTLE     = 30     # 30% → ~0.20 m/s — controllable on panels (was 40)
NAV_STEER_GAIN   = 0.8    # heading error (deg) → steer % — snappier (was 0.6)
MIN_GPS_SPEED_MS = 0.15   # use GPS course above this speed (was 0.2)

sys.path.insert(0, str(Path(__file__).parent / "mission_planner"))

# ── Helpers ───────────────────────────────────────────────────────────────────

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def us_to_pct(us, centre=1500, span=500):
    delta = us - centre
    if abs(delta) < DEADZONE:
        return 0
    sign = 1 if delta > 0 else -1
    return int(clamp(sign * (abs(delta) - DEADZONE) / (span - DEADZONE) * 100, -100, 100))

def sw_state(us):
    return "HIGH" if us > 1700 else ("LOW" if us < 1300 else "MID")

def bar(us, width=22, lo=1000, hi=2000):
    filled = int(clamp((us - lo) / (hi - lo) * width, 0, width))
    return "[" + "#" * filled + "-" * (width - filled) + "]"

def clr():
    print("\033[H\033[J", end="", flush=True)

# ── Geometry helpers ──────────────────────────────────────────────────────────

_R = 6_371_000.0  # Earth radius, metres

def haversine_m(lat1, lon1, lat2, lon2):
    """Distance in metres between two WGS-84 points."""
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (math.sin(dlat / 2) ** 2 +
         math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) *
         math.sin(dlon / 2) ** 2)
    return _R * 2 * math.asin(math.sqrt(a))

def bearing_deg(lat1, lon1, lat2, lon2):
    """Initial bearing in degrees (0=N, 90=E) from point 1 to point 2."""
    dlon = math.radians(lon2 - lon1)
    x = math.sin(dlon) * math.cos(math.radians(lat2))
    y = (math.cos(math.radians(lat1)) * math.sin(math.radians(lat2)) -
         math.sin(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.cos(dlon))
    return (math.degrees(math.atan2(x, y)) + 360) % 360

def _ll_to_en(lat, lon, olat, olon):
    """Lat/lon → local East/North (metres) relative to origin."""
    north = math.radians(lat - olat) * _R
    east  = math.radians(lon - olon) * _R * math.cos(math.radians(olat))
    return east, north

def _en_to_ll(east, north, olat, olon):
    """Local East/North → lat/lon."""
    lat = olat + math.degrees(north / _R)
    lon = olon + math.degrees(east / (_R * math.cos(math.radians(olat))))
    return lat, lon

# ── Lawnmower path generator ──────────────────────────────────────────────────

def generate_lawnmower(corners_ll, spacing_m=0.5, overshoot_m=1.0):
    """
    Build a boustrophedon (back-and-forth) sweep path inside a GPS polygon.

    corners_ll  : [(lat, lon), …]  — at least 3 points, clockwise or CCW
    spacing_m   : distance between adjacent sweep lines in metres
    overshoot_m : extra metres added past each polygon edge on sweep lines
    Returns     : [(lat, lon), …]  — ordered waypoints
    """
    if len(corners_ll) < 3:
        return []

    olat = sum(p[0] for p in corners_ll) / len(corners_ll)
    olon = sum(p[1] for p in corners_ll) / len(corners_ll)
    poly = [_ll_to_en(la, lo, olat, olon) for la, lo in corners_ll]
    n    = len(poly)

    best_angle, best_len = 0.0, 0.0
    for i in range(n):
        dx = poly[(i + 1) % n][0] - poly[i][0]
        dy = poly[(i + 1) % n][1] - poly[i][1]
        le = math.hypot(dx, dy)
        if le > best_len:
            best_len   = le
            best_angle = math.atan2(dy, dx)

    ca, sa = math.cos(-best_angle), math.sin(-best_angle)
    rot = [(x * ca - y * sa, x * sa + y * ca) for x, y in poly]

    min_y = min(p[1] for p in rot)
    max_y = max(p[1] for p in rot)

    wps_rot, row = [], 0
    y = min_y + spacing_m / 2
    while y <= max_y:
        xs = []
        for i in range(n):
            x1, y1 = rot[i]
            x2, y2 = rot[(i + 1) % n]
            if (y1 <= y < y2) or (y2 <= y < y1):
                t = (y - y1) / (y2 - y1)
                xs.append(x1 + t * (x2 - x1))
        if len(xs) >= 2:
            xs.sort()
            left  = xs[0]  - overshoot_m
            right = xs[-1] + overshoot_m
            if row % 2 == 0:
                wps_rot += [(left, y), (right, y)]
            else:
                wps_rot += [(right, y), (left, y)]
            row += 1
        y += spacing_m

    ca2, sa2 = math.cos(best_angle), math.sin(best_angle)
    result = []
    for rx, ry in wps_rot:
        xb = rx * ca2 - ry * sa2
        yb = rx * sa2 + ry * ca2
        result.append(_en_to_ll(xb, yb, olat, olon))
    return result

def generate_lawnmower_xy(corners_xy, spacing_m=0.12, overshoot_m=0.5):
    """
    Boustrophedon sweep in local X/Y metres (no GPS needed).
    corners_xy  : [(x, y), …]  — at least 3 points in metres
    spacing_m   : distance between sweep lines
    overshoot_m : extra metres added past each polygon edge on sweep lines
    Returns     : [(x, y), …]  waypoints in metres
    """
    if len(corners_xy) < 3:
        return []
    poly = list(corners_xy)
    n    = len(poly)

    best_angle, best_len = 0.0, 0.0
    for i in range(n):
        dx = poly[(i + 1) % n][0] - poly[i][0]
        dy = poly[(i + 1) % n][1] - poly[i][1]
        le = math.hypot(dx, dy)
        if le > best_len:
            best_len   = le
            best_angle = math.atan2(dy, dx)

    ca, sa = math.cos(-best_angle), math.sin(-best_angle)
    rot = [(x * ca - y * sa, x * sa + y * ca) for x, y in poly]

    min_y = min(p[1] for p in rot)
    max_y = max(p[1] for p in rot)

    wps_rot, row = [], 0
    y = min_y + spacing_m / 2
    while y <= max_y:
        xs = []
        for i in range(n):
            x1, y1 = rot[i]
            x2, y2 = rot[(i + 1) % n]
            if (y1 <= y < y2) or (y2 <= y < y1):
                t = (y - y1) / (y2 - y1)
                xs.append(x1 + t * (x2 - x1))
        if len(xs) >= 2:
            xs.sort()
            left  = xs[0]  - overshoot_m
            right = xs[-1] + overshoot_m
            if row % 2 == 0:
                wps_rot += [(left, y), (right, y)]
            else:
                wps_rot += [(right, y), (left, y)]
            row += 1
        y += spacing_m

    ca2, sa2 = math.cos(best_angle), math.sin(best_angle)
    return [(rx * ca2 - ry * sa2, rx * sa2 + ry * ca2) for rx, ry in wps_rot]

# ── Dead-reckoning odometer ───────────────────────────────────────────────────

class Odometer:
    """
    Estimates robot position from throttle% (speed) and IMU yaw.
    Updated every time a STATUS line arrives from the Pico (~10 Hz).

    Coordinate frame:
      Origin (0, 0) = robot position at startup (or last reset)
      X = East  (robot's initial right)
      Y = North (robot's initial forward)
      Angles follow compass: 0° = North, 90° = East  (same as IMU yaw)

    Heading source priority:
      1. IMU yaw from Pico STATUS — used when it is actively changing (> 0.3° delta)
      2. Steer-integrated estimate — used when IMU yaw is stuck (Pico not connected
         or IMU not calibrated); uses differential-drive kinematics with WHEELBASE_M.
    """

    def __init__(self):
        self._x        = 0.0
        self._y        = 0.0
        self._yaw      = 0.0       # tracked heading (degrees)
        self._dist     = 0.0
        self._lock     = threading.Lock()
        self._last_t   = time.time()
        self._imu_prev = None      # previous IMU yaw; None until first update

    @property
    def x(self):
        with self._lock:
            return self._x

    @property
    def y(self):
        with self._lock:
            return self._y

    @property
    def dist(self):
        with self._lock:
            return self._dist

    @property
    def position(self):
        with self._lock:
            return (self._x, self._y)

    def update(self, throttle_pct: int, yaw_deg: float, steer_pct: int = 0):
        """Call each time a STATUS is received."""
        now = time.time()
        dt  = now - self._last_t
        self._last_t = now
        if dt <= 0 or dt > 0.5:
            return

        speed_ms = abs(throttle_pct) / 100.0 * MAX_SPEED_MS

        # Use live IMU yaw when Pico is reporting a changing heading
        imu_live = (self._imu_prev is not None and
                    abs(yaw_deg - self._imu_prev) > 0.3)
        self._imu_prev = yaw_deg

        if imu_live:
            self._yaw = yaw_deg
        elif steer_pct != 0:
            # IMU not live — integrate heading from steer command.
            # Differential-drive model: omega = 2*v*steer / wheelbase
            # Use a small minimum speed so pivoting in place still shows turns.
            spd = max(speed_ms, 0.05 * MAX_SPEED_MS)
            omega_deg = (steer_pct / 100.0) * 2.0 * spd / WHEELBASE_M * (180.0 / math.pi)
            self._yaw = (self._yaw + omega_deg * dt) % 360

        if speed_ms < 0.01:
            return

        sign    = 1 if throttle_pct >= 0 else -1
        yaw_rad = math.radians(self._yaw)
        dx = sign * speed_ms * math.sin(yaw_rad) * dt
        dy = sign * speed_ms * math.cos(yaw_rad) * dt
        with self._lock:
            self._x    += dx
            self._y    += dy
            self._dist += math.hypot(dx, dy)

    def reset(self):
        with self._lock:
            self._x = self._y = self._dist = 0.0
        self._yaw      = 0.0
        self._imu_prev = None
        self._last_t   = time.time()

# ── Boundary recorder ─────────────────────────────────────────────────────────

class BoundaryRecorder:
    """Accumulates GPS corners that define the cleaning area polygon."""

    def __init__(self):
        self._corners = []
        self._lock    = threading.Lock()

    @property
    def count(self):
        with self._lock:
            return len(self._corners)

    @property
    def corners(self):
        with self._lock:
            return list(self._corners)

    def add(self, a, b, odo_mode=False):
        """Add corner. In GPS mode: (lat, lon). In odo mode: (x_m, y_m)."""
        with self._lock:
            self._corners.append((a, b))
            n = len(self._corners)
        if odo_mode:
            print(f"[BOUNDARY] Corner {n}: x={a:.2f}m  y={b:.2f}m")
        else:
            print(f"[BOUNDARY] Corner {n}: {a:.7f}, {b:.7f}")
        return n

    def load(self, corners):
        """Bulk-load corners from a saved file."""
        with self._lock:
            self._corners = list(corners)

    def reset(self):
        with self._lock:
            self._corners = []
        print("[BOUNDARY] Reset")

# ── Mission runner ────────────────────────────────────────────────────────────

class MissionRunner:
    """
    Follows waypoints autonomously — GPS mode (lat/lon) or odometry mode (x/y metres).
    compute() is called from the Pico thread at SEND_HZ.
    """

    def __init__(self, waypoints, odo_mode=False):
        self._wps      = waypoints
        self._idx      = 0
        self._running  = False
        self._done     = False
        self._odo_mode = odo_mode   # True = use dead reckoning, False = use GPS
        self._lock     = threading.Lock()

    @property
    def running(self):
        with self._lock:
            return self._running

    @property
    def done(self):
        with self._lock:
            return self._done

    @property
    def current_wp(self):
        with self._lock:
            return self._idx

    @property
    def total(self):
        return len(self._wps)

    @property
    def waypoints(self):
        return list(self._wps)

    def start(self):
        with self._lock:
            self._running = True
            self._done    = False

    def pause(self):
        with self._lock:
            self._running = False

    def abort(self):
        with self._lock:
            self._running = False
            self._idx     = 0
            self._done    = False

    def compute(self, fix, yaw_deg, odo_xy=None):
        """Return (steer -100..100, throttle 0..100) toward the current waypoint."""
        with self._lock:
            if not self._running or self._done:
                return 0, 0
            if self._idx >= len(self._wps):
                self._done    = True
                self._running = False
                return 0, 0
            wp = self._wps[self._idx]

        if self._odo_mode:
            # ── Dead-reckoning navigation ─────────────────────────────────────
            rx, ry = odo_xy if odo_xy else (0.0, 0.0)
            wx, wy = wp
            dist   = math.hypot(wx - rx, wy - ry)
            if dist < ARRIVAL_RADIUS_M:
                with self._lock:
                    self._idx += 1
                    if self._idx >= len(self._wps):
                        self._done    = True
                        self._running = False
                        return 0, 0
                    wp = self._wps[self._idx]
                wx, wy = wp
            # Bearing: atan2(dx, dy) → compass degrees (0=N 90=E)
            target = (math.degrees(math.atan2(wx - rx, wy - ry)) + 360) % 360

        else:
            # ── GPS navigation ────────────────────────────────────────────────
            if not fix or not fix.has_fix:
                return 0, NAV_THROTTLE
            wp_lat, wp_lon = wp
            dist = haversine_m(fix.lat, fix.lon, wp_lat, wp_lon)
            if dist < ARRIVAL_RADIUS_M:
                with self._lock:
                    self._idx += 1
                    if self._idx >= len(self._wps):
                        self._done    = True
                        self._running = False
                        return 0, 0
                    wp = self._wps[self._idx]
                wp_lat, wp_lon = wp
            heading_gps = (fix.heading
                           if fix.speed_ms >= MIN_GPS_SPEED_MS and fix.heading >= 0
                           else yaw_deg)
            target  = bearing_deg(fix.lat, fix.lon, wp_lat, wp_lon)
            yaw_deg = heading_gps

        err = (target - yaw_deg + 360) % 360
        if err > 180:
            err -= 360  # −180..+180

        steer = int(clamp(err * NAV_STEER_GAIN, -100, 100))
        return steer, NAV_THROTTLE

# ── Shared state ──────────────────────────────────────────────────────────────

_state = {
    "channels":     [1500] * 18,
    "frames":       0,
    "last_rc":      None,
    "failsafe":     True,
    "armed":        False,
    "steer":        0,
    "throttle":     0,
    "gps_on":       False,
    "gps_fix":      None,
    "gps_ok":       False,
    "pico_ok":      False,
    "pico_seen":    False,
    "pico_status":  "",
    "link":         {},
    # Mission state machine
    "mission_mode": "IDLE",   # IDLE | RECORDING | PLANNED | RUNNING | DONE
    "wp_index":     0,
    # Dead-reckoning position (metres from start, updated each Pico STATUS)
    "odo_x":        0.0,
    "odo_y":        0.0,
    "odo_dist":     0.0,
    # Web UI control flags (set by web_ui.py, cleared here)
    "web_stop":     False,
    "web_pause":    False,
    # Web joystick manual drive
    "web_manual":   False,
    "web_throttle": 0,
    "web_steer":    0,
    "web_manual_t": 0.0,
}

_stop     = threading.Event()
_boundary = BoundaryRecorder()
_mission  = None   # MissionRunner — set when path is planned
_odo      = Odometer()   # dead-reckoning position tracker

# ── GPS ───────────────────────────────────────────────────────────────────────

_gps_reader = None

def _start_gps(port, baud):
    global _gps_reader
    if _gps_reader:
        return
    try:
        from gps_reader import GPSReader
        _gps_reader = GPSReader(port=port, baud=baud)
        _gps_reader.start()
        _state["gps_on"] = True
        print(f"[GPS] Started on {port} @ {baud}")
    except Exception as e:
        print(f"[GPS] Failed to start: {e}")

def _stop_gps():
    global _gps_reader
    if not _gps_reader:
        return
    _gps_reader.stop()
    _gps_reader = None
    _state["gps_on"]  = False
    _state["gps_ok"]  = False
    _state["gps_fix"] = None

def _gps_thread(port, baud):
    while not _stop.is_set():
        if _gps_reader:
            fix = _gps_reader.get_fix()
            _state["gps_fix"] = fix
            _state["gps_ok"]  = getattr(_gps_reader, "connected", False) and fix.has_fix
        time.sleep(0.2)

# ── Pico UART thread ──────────────────────────────────────────────────────────

def _parse_status(status_str):
    """Parse 'STATUS steer throttle yaw speed_cms bat_mv wp_r roll pitch'.
    Returns (throttle_pct, yaw_deg) or (0, 0.0) on failure."""
    try:
        p = status_str.split()
        if len(p) >= 4:
            return int(p[2]), float(p[3])
    except Exception:
        pass
    return 0, 0.0

def _pico_thread(port, baud):
    import serial
    interval = 1.0 / SEND_HZ
    while not _stop.is_set():
        try:
            with serial.Serial(port, baud, timeout=0.05) as ser:
                _state["pico_ok"]   = True
                _state["pico_seen"] = True
                buf       = b""
                last_send = 0.0
                while not _stop.is_set():
                    now = time.time()
                    if now - last_send >= interval:
                        last_send = now
                        # Web joystick: fresh command + armed + no emergency stop
                        _wm_age = now - _state.get("web_manual_t", 0)
                        _wm_ok  = (_state.get("web_manual") and _wm_age < 0.5
                                   and _state["armed"] and not _state.get("web_stop"))
                        if _state.get("web_stop") or (not _wm_ok and
                                (_state["failsafe"] or not _state["armed"])):
                            ser.write(b"STOP\n")
                        elif _state["mission_mode"] == "RUNNING" and _mission is not None:
                            thr, yaw = _parse_status(_state["pico_status"])
                            s, t = _mission.compute(_state["gps_fix"], yaw,
                                                    odo_xy=_odo.position)
                            _state["steer"]    = s
                            _state["throttle"] = t
                            _state["wp_index"] = _mission.current_wp
                            if _mission.done:
                                _state["mission_mode"] = "DONE"
                            ser.write(f"MOVE {s} {t}\n".encode())
                        elif _wm_ok:
                            # Web joystick override: Y=throttle, X=steer
                            s = clamp(int(_state.get("web_steer",    0)), -100, 100)
                            t = clamp(int(_state.get("web_throttle", 0)), -100, 100)
                            _state["steer"]    = s
                            _state["throttle"] = t
                            ser.write(f"MOVE {s} {t}\n".encode())
                        else:
                            s = _state["steer"]
                            t = _state["throttle"]
                            ser.write(f"MOVE {s} {t}\n".encode())
                    chunk = ser.read(ser.in_waiting or 1)
                    if chunk:
                        buf += chunk
                        while b"\n" in buf:
                            line, buf = buf.split(b"\n", 1)
                            txt = line.decode("ascii", "replace").strip()
                            if txt.startswith("STATUS"):
                                _state["pico_status"] = txt
                                thr, yaw = _parse_status(txt)
                                _odo.update(thr, yaw, steer_pct=_state.get("steer", 0))
                                _state["odo_x"]    = _odo.x
                                _state["odo_y"]    = _odo.y
                                _state["odo_dist"] = _odo.dist
        except Exception:
            _state["pico_ok"] = False
            if not _stop.is_set():
                time.sleep(2)

# ── Mission save / load ───────────────────────────────────────────────────────

def _save_mission(waypoints, corners, odo_mode=False):
    PATH_DIR.mkdir(parents=True, exist_ok=True)
    ts   = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = PATH_DIR / f"mission_{ts}.json"
    if odo_mode:
        data = {
            "saved_at":  datetime.now().isoformat(),
            "mode":      "odometry",
            "boundary":  [{"x": c[0], "y": c[1]} for c in corners],
            "waypoints": [{"x": c[0], "y": c[1]} for c in waypoints],
        }
    else:
        data = {
            "saved_at":  datetime.now().isoformat(),
            "boundary":  [{"lat": c[0], "lon": c[1]} for c in corners],
            "waypoints": [{"lat": c[0], "lon": c[1]} for c in waypoints],
        }
    path.write_text(json.dumps(data, indent=2))
    print(f"[MISSION] Saved → {path.name}")
    return path

def _autosave_boundary(corners, odo_mode=False):
    """Write current boundary corners to boundary_current.json after every SB press."""
    if not corners:
        return
    PATH_DIR.mkdir(parents=True, exist_ok=True)
    p = PATH_DIR / "boundary_current.json"
    if odo_mode:
        raw = [{"x": c[0], "y": c[1]} for c in corners]
        p.write_text(json.dumps({"saved_at": datetime.now().isoformat(),
                                 "mode": "odometry", "corners": raw}, indent=2))
    else:
        raw = [{"lat": c[0], "lon": c[1]} for c in corners]
        p.write_text(json.dumps({"saved_at": datetime.now().isoformat(),
                                 "corners": raw}, indent=2))

def _load_mission_file(path):
    """
    Load a saved mission or boundary JSON.
    Returns (corners, waypoints, odo_mode).
    odo_mode=True means coordinates are local (x, y) metres, not (lat, lon).
    waypoints may be [] if only a boundary was saved.
    """
    data = json.loads(Path(path).read_text())
    odo  = (data.get("mode") == "odometry")
    def _pt(p):
        return (p["x"], p["y"]) if odo else (p["lat"], p["lon"])
    if "waypoints" in data:
        corners   = [_pt(p) for p in data.get("boundary",  [])]
        waypoints = [_pt(p) for p in data.get("waypoints", [])]
        return corners, waypoints, odo
    if "corners" in data:
        corners = [_pt(p) for p in data["corners"]]
        return corners, [], odo
    return [], [], False

def list_missions():
    """Print all saved missions and exit. Called when --list-missions is used."""
    files = sorted(PATH_DIR.glob("mission_*.json"), reverse=True)
    bfile = PATH_DIR / "boundary_current.json"
    print(f"\nSaved missions in {PATH_DIR}/\n")
    if bfile.exists():
        try:
            d  = json.loads(bfile.read_text())
            ts = d.get("saved_at", "")[:19]
            n  = len(d.get("corners", []))
            print(f"  [boundary_current.json]  {ts}  — {n} corners  (use --resume-boundary)")
        except Exception:
            pass
    if not files:
        print("  No completed missions found.")
    for f in files:
        try:
            d  = json.loads(f.read_text())
            ts = d.get("saved_at", "")[:19]
            bc = len(d.get("boundary",  []))
            wc = len(d.get("waypoints", []))
            print(f"  {f.name}  {ts}  — {bc} corners  {wc} waypoints")
        except Exception:
            print(f"  {f.name}  (unreadable)")
    print("\nUsage:")
    print("  python3 rc_drive.py --resume                  # load newest mission")
    print("  python3 rc_drive.py --mission paths/<file>    # load specific file")
    print("  python3 rc_drive.py --resume-boundary         # reload corners only\n")

# ── RC main loop ──────────────────────────────────────────────────────────────

def run(args):
    global _mission

    # ── Load saved mission / boundary if requested ────────────────────────────
    _loaded_from = None
    _load_path   = None

    if args.list_missions:
        list_missions()
        return

    if getattr(args, "mission", None):
        _load_path = args.mission
    elif getattr(args, "resume", False):
        files = sorted(PATH_DIR.glob("mission_*.json"))
        if files:
            _load_path = str(files[-1])
        else:
            print("[MISSION] No saved missions found — starting fresh")
    elif getattr(args, "resume_boundary", False):
        bp = PATH_DIR / "boundary_current.json"
        if bp.exists():
            _load_path = str(bp)
        else:
            print("[MISSION] No boundary_current.json found — start recording corners with SB")
    else:
        # Auto-load the last saved mission so the bot always remembers the field
        files = sorted(PATH_DIR.glob("mission_*.json"))
        if files:
            _load_path = str(files[-1])
            print(f"[MISSION] Auto-resuming last saved mission: {Path(_load_path).name}")

    if _load_path:
        try:
            corners, waypoints, load_odo = _load_mission_file(_load_path)
            _loaded_from = Path(_load_path).name
            if corners:
                _boundary.load(corners)
                ctype = "odo" if load_odo else "GPS"
                print(f"[MISSION] Loaded '{_loaded_from}' ({ctype}): {len(corners)} corner(s)")
            if waypoints:
                _mission = MissionRunner(waypoints, odo_mode=load_odo)
                _state["mission_mode"] = "PLANNED"
                print(f"[MISSION] Path ready: {len(waypoints)} waypoints — ARM + flip SC to start")
            elif corners:
                _state["mission_mode"] = "RECORDING"
                print("[MISSION] Boundary loaded — flip SC when done adding corners")
        except Exception as e:
            print(f"[MISSION] Failed to load '{_load_path}': {e}")

    # Store loaded filename in args so _draw() can display it
    args._loaded_from = _loaded_from

    print("=" * 62)
    print("  Solar Bot RC Drive + Mission Planner")
    print("=" * 62)

    print(f"[1] RC receiver  {args.rc} @ {RC_BAUD} ...")
    try:
        from pymavlink import mavutil
        mav = mavutil.mavlink_connection(args.rc, baud=RC_BAUD, source_system=255)
    except Exception as e:
        print(f"    ERROR: {e}")
        sys.exit(1)
    print("    OK")

    print(f"[2] Pico UART    {args.pico} @ {PICO_BAUD} ...")
    threading.Thread(target=_pico_thread, args=(args.pico, PICO_BAUD),
                     daemon=True).start()
    print("    OK (connecting...)")

    if not args.no_gps:
        print(f"[3] GPS          {args.gps}  (auto-starts on first SB corner mark)")
        threading.Thread(target=_gps_thread, args=(args.gps, args.gps_baud),
                         daemon=True).start()
    else:
        print("[3] GPS          DISABLED  — using dead-reckoning odometry")

    if not args.no_web:
        print(f"[4] Web UI       port {args.web_port} ...")
        import web_ui
        web_ui.start(_state, _boundary, lambda: _mission, _stop, port=args.web_port)
        # Print the actual access URL immediately so any device on the network knows where to connect
        try:
            _ni = web_ui.get_net_info()
            _url = _ni.get("url") or f"http://localhost:{args.web_port}"
            if _ni.get("mode") == "hotspot":
                print("\n  ┌─ HOTSPOT MODE ─────────────────────────────────────┐")
                print(f"  │  SSID    : \033[97m{_ni.get('ssid', 'SolarBot')}\033[0m")
                print(f"  │  Password: \033[97m{_ni.get('password', 'solarbot123')}\033[0m")
                print(f"  │  Open    : \033[96m{_url}\033[0m")
                print("  └────────────────────────────────────────────────────┘\n")
            elif _ni.get("mode") == "wifi":
                ssid = _ni.get('ssid', '')
                print("\n  ┌─ WIFI MODE ─────────────────────────────────────────┐")
                print(f"  │  Router  : \033[97m{ssid}\033[0m")
                print(f"  │  Open    : \033[96m{_url}\033[0m")
                print("  └────────────────────────────────────────────────────┘\n")
            else:
                print(f"    Browse → \033[96m{_url}\033[0m")
        except Exception:
            print(f"    Browse → http://localhost:{args.web_port}")
    else:
        print("[4] Web UI       DISABLED")

    stop_flag = [False]
    def _sig(s, f):
        stop_flag[0] = True
    signal.signal(signal.SIGINT,  _sig)
    signal.signal(signal.SIGTERM, _sig)

    lq_hist      = deque(maxlen=20)
    last_display = 0.0
    prev_sb = prev_sc = prev_sd = False

    nav_mode = "dead-reckoning (odo)" if args.no_gps else "GPS"
    print(f"\nReady [{nav_mode}] — SA↑=arm  SB=mark corner  SC=plan→run  SC↓=pause  SD=abort")
    print(f"Sweep spacing: {args.spacing} m\n")

    while not stop_flag[0]:
        try:
            msg = mav.recv_msg()
        except Exception as _e:
            print(f"\n[RC] Serial error: {_e} — reconnecting in 2s ...")
            time.sleep(2)
            try:
                from pymavlink import mavutil as _mu
                mav = _mu.mavlink_connection(args.rc, baud=RC_BAUD, source_system=255)
                print("[RC] Reconnected.")
            except Exception as _e2:
                print(f"[RC] Reconnect failed: {_e2} — retrying ...")
            continue
        if msg:
            t = msg.get_type()

            if t == "RC_CHANNELS_OVERRIDE":
                d  = msg.to_dict()
                ch = _state["channels"]
                for i in range(1, 19):
                    v = d.get(f"chan{i}_raw", 0)
                    if v > 0:
                        ch[i - 1] = v
                _state["last_rc"] = time.time()
                _state["frames"] += 1

                ch2 = ch[1]
                ch4 = ch[3]
                ch5 = ch[4]
                ch6 = ch[5]
                ch7 = ch[6]
                ch8 = ch[7]

                _state["armed"] = (ch5 > 1700)
                mode = _state["mission_mode"]

                # Manual steer/throttle only when not autonomously running
                if mode != "RUNNING":
                    _state["steer"]    = us_to_pct(ch4)
                    _state["throttle"] = us_to_pct(ch2)

                sb_now = (ch6 > 1700)
                sc_now = (ch7 > 1700)
                sd_now = (ch8 > 1700)

                # ── SB LOW→HIGH: mark boundary corner ────────────────────────
                if sb_now and not prev_sb:
                    if args.no_gps:
                        # Dead-reckoning mode: record current odometry position
                        if mode in ("IDLE", "RECORDING"):
                            if mode == "IDLE":
                                _state["mission_mode"] = "RECORDING"
                                mode = "RECORDING"
                            px, py = _odo.position
                            n = _boundary.add(px, py, odo_mode=True)
                            _autosave_boundary(_boundary.corners, odo_mode=True)
                            print(f"[BOUNDARY] Corner {n}: x={px:.2f}m  y={py:.2f}m  "
                                  f"(dist={_odo.dist:.2f}m)")
                    elif mode in ("IDLE", "RECORDING"):
                        _start_gps(args.gps, args.gps_baud)
                        fix = _state["gps_fix"]
                        if fix and fix.has_fix:
                            if mode == "IDLE":
                                _state["mission_mode"] = "RECORDING"
                                mode = "RECORDING"
                            _boundary.add(fix.lat, fix.lon)
                            _autosave_boundary(_boundary.corners)
                        else:
                            if mode == "IDLE":
                                _state["mission_mode"] = "RECORDING"
                            print("[BOUNDARY] No GPS fix yet — move to open sky and flip SB again")

                # ── SC LOW→HIGH: plan path OR start mission ───────────────────
                if sc_now and not prev_sc:
                    if mode == "RECORDING":
                        cnt = _boundary.count
                        if cnt >= 3:
                            corners = _boundary.corners
                            if args.no_gps:
                                wps = generate_lawnmower_xy(corners, args.spacing, args.overshoot)
                                if wps:
                                    _mission = MissionRunner(wps, odo_mode=True)
                                    _state["mission_mode"] = "PLANNED"
                                    _save_mission(wps, corners, odo_mode=True)
                                    print(f"[MISSION] Planned (odo): {len(wps)} waypoints, "
                                          f"{cnt} corners, spacing {args.spacing}m, overshoot {args.overshoot}m")
                                else:
                                    print("[MISSION] Path generation failed — check corners")
                            else:
                                wps = generate_lawnmower(corners, args.spacing, args.overshoot)
                                if wps:
                                    _mission = MissionRunner(wps)
                                    _state["mission_mode"] = "PLANNED"
                                    _save_mission(wps, corners)
                                    print(f"[MISSION] Planned: {len(wps)} waypoints, "
                                          f"{cnt} boundary corners, spacing {args.spacing}m, overshoot {args.overshoot}m")
                                else:
                                    print("[MISSION] Path generation failed — check corners")
                        else:
                            print(f"[MISSION] Need ≥3 corners (have {cnt}) — mark more with SB")

                    elif mode in ("PLANNED", "DONE"):
                        if _state["armed"]:
                            if _mission:
                                _mission.abort()
                                _mission.start()
                            _state["mission_mode"] = "RUNNING"
                            _state["wp_index"]     = 0
                            print("[MISSION] Autonomous sweep started")
                        else:
                            print("[MISSION] ARM first (SA↑) then flip SC to start")

                # ── SC HIGH→LOW: pause mission ────────────────────────────────
                if not sc_now and prev_sc:
                    if mode == "RUNNING" and _mission:
                        _mission.pause()
                        _state["mission_mode"] = "PLANNED"
                        _state["steer"]        = 0
                        _state["throttle"]     = 0
                        print("[MISSION] Paused")

                # ── SD LOW→HIGH: abort and full reset ─────────────────────────
                if sd_now and not prev_sd:
                    if _mission:
                        _mission.abort()
                    _boundary.reset()
                    _mission = None
                    _state["mission_mode"] = "IDLE"
                    _state["wp_index"]     = 0
                    _state["steer"]        = 0
                    _state["throttle"]     = 0
                    if args.no_gps:
                        _odo.reset()
                        print("[MISSION] Aborted and reset (odometer zeroed)")
                    else:
                        print("[MISSION] Aborted and reset")

                prev_sb, prev_sc, prev_sd = sb_now, sc_now, sd_now

            elif t == "RADIO_STATUS":
                d = msg.to_dict()
                _state["link"] = {
                    "rssi":    d.get("rssi", 0),
                    "remrssi": d.get("remrssi", 0),
                    "lq":      d.get("txbuf", 100),
                }
                lq_hist.append(_state["link"]["lq"])

        lr = _state["last_rc"]
        _state["failsafe"] = (lr is None) or (time.time() - lr > FAILSAFE_S)

        # ── Handle web UI control flags ───────────────────────────────────────
        mode = _state["mission_mode"]

        if _state.get("web_stop"):
            _state["web_stop"] = False
            if _mission:
                _mission.pause()
            _state["mission_mode"] = "PLANNED" if _mission else "IDLE"
            _state["steer"] = _state["throttle"] = 0
            print("[WEB] Emergency stop")

        if _state.get("web_pause") and mode == "RUNNING":
            _state["web_pause"] = False
            if _mission:
                _mission.pause()
            _state["mission_mode"] = "PLANNED"
            _state["steer"] = _state["throttle"] = 0
            print("[WEB] Mission paused")

        if _state.get("web_arm"):
            _state["web_arm"] = False
            _state["armed"] = True
            print("[WEB] Armed")

        if _state.get("web_disarm"):
            _state["web_disarm"] = False
            _state["armed"] = False
            _state["steer"] = _state["throttle"] = 0
            print("[WEB] Disarmed")

        if _state.get("web_mark_corner"):
            _state["web_mark_corner"] = False
            mode = _state["mission_mode"]
            if mode in ("IDLE", "RECORDING"):
                if mode == "IDLE":
                    _state["mission_mode"] = "RECORDING"
                if args.no_gps:
                    px, py = _odo.position
                    n = _boundary.add(px, py, odo_mode=True)
                    _autosave_boundary(_boundary.corners, odo_mode=True)
                    print(f"[WEB] Corner {n}: x={px:.2f}m y={py:.2f}m")
                else:
                    fix = _state["gps_fix"]
                    if fix and fix.has_fix:
                        _boundary.add(fix.lat, fix.lon)
                        _autosave_boundary(_boundary.corners)
                        print(f"[WEB] Corner {_boundary.count}: {fix.lat:.7f},{fix.lon:.7f}")
                    else:
                        print("[WEB] Mark corner: no GPS fix yet")

        if _state.get("web_plan"):
            _state["web_plan"] = False
            mode = _state["mission_mode"]
            spacing   = float(_state.get("web_row_spacing", 0.8))
            overshoot = float(_state.get("web_overshoot", 1.0))
            if mode == "RECORDING":
                cnt = _boundary.count
                if cnt >= 3:
                    corners = _boundary.corners
                    if args.no_gps:
                        wps = generate_lawnmower_xy(corners, spacing, overshoot)
                    else:
                        wps = generate_lawnmower(corners, spacing, overshoot)
                    if wps:
                        _mission = MissionRunner(wps, odo_mode=args.no_gps)
                        _state["mission_mode"] = "PLANNED"
                        _save_mission(wps, corners, odo_mode=args.no_gps)
                        print(f"[WEB] Path planned: {len(wps)} waypoints, spacing={spacing}m, overshoot={overshoot}m")
                    else:
                        print("[WEB] Path generation failed")
                else:
                    print(f"[WEB] Need ≥3 corners (have {cnt})")
            elif mode in ("PLANNED", "DONE"):
                # Second plan press = start mission
                if _state["armed"] and _mission:
                    _mission.abort()
                    _mission.start()
                    _state["mission_mode"] = "RUNNING"
                    _state["wp_index"] = 0
                    print("[WEB] Mission started")
                else:
                    print("[WEB] ARM first, then Plan/Start again to run")

        if _state.get("web_resume"):
            _state["web_resume"] = False
            mode = _state["mission_mode"]
            if mode in ("PLANNED", "DONE") and _state["armed"] and _mission:
                _mission.abort()
                _mission.start()
                _state["mission_mode"] = "RUNNING"
                _state["wp_index"] = 0
                print("[WEB] Mission resumed/started")
            elif not _state["armed"]:
                print("[WEB] Resume: ARM first")

        if _state.get("web_abort"):
            _state["web_abort"] = False
            if _mission:
                _mission.abort()
                _mission = None
            _boundary.reset()
            _state["mission_mode"] = "IDLE"
            _state["wp_index"]     = 0
            _state["steer"]        = 0
            _state["throttle"]     = 0
            print("[WEB] Aborted and reset")

        if _state.get("web_load_mission"):
            load_path = _state.pop("web_load_mission")
            try:
                corners, waypoints, load_odo = _load_mission_file(load_path)
                if corners:
                    _boundary.load(corners)
                if waypoints:
                    _mission = MissionRunner(waypoints, odo_mode=load_odo)
                    _state["mission_mode"] = "PLANNED"
                    print(f"[WEB] Loaded mission: {Path(load_path).name} — {len(waypoints)} waypoints")
                elif corners:
                    _state["mission_mode"] = "RECORDING"
                    print(f"[WEB] Loaded boundary: {len(corners)} corners")
            except Exception as e:
                print(f"[WEB] Load mission failed: {e}")

        now = time.time()
        if now - last_display >= 0.1:
            last_display = now
            _draw(args, lq_hist)

        time.sleep(0.002)

    # ── Shutdown ──────────────────────────────────────────────────────────────
    _stop.set()
    print("\n\n[Shutdown] Stopping motors ...")
    try:
        import serial
        with serial.Serial(args.pico, PICO_BAUD, timeout=1) as s:
            s.write(b"STOP\n")
    except Exception:
        pass
    if _mission and _mission.running:
        _mission.pause()
    _stop_gps()
    print("Done.")

# ── Display ───────────────────────────────────────────────────────────────────

def _draw(args, lq_hist):
    ch        = _state["channels"]
    steer     = _state["steer"]
    throttle  = _state["throttle"]
    armed     = _state["armed"]
    failsafe  = _state["failsafe"]
    pico_ok   = _state["pico_ok"]
    pico_seen = _state["pico_seen"]
    pico_st   = _state["pico_status"]
    link      = _state["link"]
    gps_ok    = _state["gps_ok"]
    fix       = _state["gps_fix"]
    mode      = _state["mission_mode"]
    wp_idx    = _state["wp_index"]

    # Read network info once per draw
    try:
        import web_ui as _wu
        _net = _wu.get_net_info()
    except Exception:
        _net = {"mode": "unknown", "ssid": "", "ip": "", "url": ""}

    clr()
    print("=" * 64)
    print("  Solar Bot — RC Drive + Mission Planner")
    print(f"  RC:{args.rc}   Pico:{args.pico}")
    if not args.no_web:
        _url = _net.get("url") or f"http://{_net.get('ip','?')}:{args.web_port}"
        if _net.get("mode") == "hotspot":
            print(f"  \033[93m[HOTSPOT]\033[0m SSID:\033[97m {_net.get('ssid','SolarBot')}\033[0m"
                  f"  pw:\033[97m {_net.get('password','solarbot123')}\033[0m"
                  f"  → \033[96m{_url}\033[0m")
        elif _net.get("mode") == "wifi":
            ssid_part = f'"{_net["ssid"]}" ' if _net.get("ssid") else ""
            print(f"  \033[92m[WIFI]\033[0m Router {ssid_part}→ \033[96m{_url}\033[0m")
        else:
            print(f"  \033[90m[WEB]\033[0m Dashboard → \033[96m{_url}\033[0m")
    print("=" * 64)

    # Arm / failsafe banner
    if failsafe:
        print("\n  \033[91m*** RC SIGNAL LOST — MOTORS STOPPED ***\033[0m")
    elif not armed:
        print("\n  \033[93m  DISARMED  (flip SA↑ to arm)\033[0m")
    else:
        print("\n  \033[92m  ARMED — motors active\033[0m")

    # RC channels
    print(f"\n  {'CH':<4} {'Name':<10} {'Bar':^24} {'µs':>6}  {'%':>5}")
    print("  " + "─" * 56)
    print(f"  CH4  {'Steer':<10} {bar(ch[3])} {ch[3]:6d}µs  {steer:+5d}%")
    print(f"  CH2  {'Throttle':<10} {bar(ch[1])} {ch[1]:6d}µs  {throttle:+5d}%")

    # Switches
    labels = {5: "SA arm", 6: "SB corner", 7: "SC plan/run", 8: "SD abort"}
    print(f"\n  {'Switch':<14} {'µs':>6}  State")
    print("  " + "─" * 32)
    for i in range(5, 9):
        us = ch[i - 1]
        st = sw_state(us)
        c  = "\033[92m" if st == "HIGH" else ("\033[91m" if st == "LOW" else "\033[93m")
        print(f"  {labels[i]:<14} {us:6d}µs  {c}{st}\033[0m")

    # Motor output
    print("\n  MOTORS: ", end="")
    if failsafe or not armed:
        print("\033[91mSTOP\033[0m")
    else:
        left  = throttle - steer
        right = throttle + steer
        peak  = max(abs(left), abs(right), 100)
        lp    = int(left  / peak * 100)
        rp    = int(right / peak * 100)
        def dir_label(v):
            return "FWD" if v > 0 else ("REV" if v < 0 else "STP")
        print(f"\033[92mLeft {lp:+4d}% {dir_label(lp)}   Right {rp:+4d}% {dir_label(rp)}\033[0m")

    # GPS / odometry position
    ox = _state["odo_x"]
    oy = _state["odo_y"]
    od = _state["odo_dist"]
    if args.no_gps:
        print(f"\n  ODO:  \033[96mx={ox:+.2f}m  y={oy:+.2f}m  dist={od:.2f}m\033[0m"
              f"  (dead reckoning — flip SB to mark corners)")
    elif gps_ok and fix:
        print(f"\n  GPS:  \033[92mFIX\033[0m  {fix.lat:.7f}, {fix.lon:.7f}"
              f"  sats:{fix.satellites}  spd:{fix.speed_ms:.1f}m/s")
    else:
        print("\n  GPS:  \033[93mno fix\033[0m  — flip SB to start boundary recording when ready")

    # ── Mission panel ─────────────────────────────────────────────────────────
    print()
    bc = _boundary.count

    # Show which file this session loaded from
    _lf = getattr(args, "_loaded_from", None)
    if _lf:
        print(f"  \033[90mLoaded: {_lf}\033[0m")

    if mode == "IDLE":
        print("  MISSION: \033[90mIDLE\033[0m")
        print("           Drive to each corner and flip SB to mark it")
        saved = sorted(PATH_DIR.glob("mission_*.json"))
        if saved:
            print(f"           \033[90m(tip: --resume to reload last mission: {saved[-1].name})\033[0m")

    elif mode == "RECORDING":
        mode_tag = "odo" if args.no_gps else "GPS"
        print(f"  MISSION: \033[93mRECORDING BOUNDARY\033[0m [{mode_tag}] — {bc} corner(s) marked"
              f"  \033[90m(auto-saved)\033[0m")
        if bc >= 3:
            print("           Flip SC to close boundary and generate lawnmower path")
        else:
            print(f"           Mark {3 - bc} more corner(s) then flip SC")

    elif mode == "PLANNED":
        total = _mission.total if _mission else 0
        print(f"  MISSION: \033[96mPATH READY\033[0m — {bc} boundary corners  →  {total} waypoints")
        print(f"           Spacing: {args.spacing}m    ARM then flip SC to start sweep")

    elif mode == "RUNNING":
        total = _mission.total if _mission else 0
        pct   = int(wp_idx / max(total, 1) * 100)
        bar_w = 30
        filled_b = int(pct / 100 * bar_w)
        prog_bar = "[" + "█" * filled_b + "░" * (bar_w - filled_b) + "]"
        print(f"  MISSION: \033[92mRUNNING\033[0m  WP {wp_idx + 1}/{total}  {prog_bar} {pct}%")
        m = _mission
        if m and wp_idx < m.total:
            wp = m._wps[wp_idx]
            if args.no_gps:
                wx, wy = wp
                dist = math.hypot(wx - ox, wy - oy)
                brg  = (math.degrees(math.atan2(wx - ox, wy - oy)) + 360) % 360
                print(f"           → WP{wp_idx + 1}: {dist:.2f}m away  heading {brg:.0f}°"
                      f"  (odo x={ox:+.2f} y={oy:+.2f})")
            elif fix and fix.has_fix:
                wp_lat, wp_lon = wp
                dist = haversine_m(fix.lat, fix.lon, wp_lat, wp_lon)
                brg  = bearing_deg(fix.lat, fix.lon, wp_lat, wp_lon)
                print(f"           → WP{wp_idx + 1}: {dist:.1f}m away  bearing {brg:.0f}°")
        print("           Flip SC↓ to pause  |  SD to abort")

    elif mode == "DONE":
        total = _mission.total if _mission else 0
        print(f"  MISSION: \033[92m✓ COMPLETE\033[0m  All {total} waypoints finished!")
        print("           Flip SC to re-run  |  SD to reset boundary")

    # Pico status
    if pico_ok:
        pico_col, pico_lbl = "\033[92m", "OK"
    elif pico_seen:
        pico_col, pico_lbl = "\033[91m", "disconnected"
    else:
        pico_col, pico_lbl = "\033[93m", "connecting..."

    bat_str = ""
    if pico_st:
        parts = pico_st.split()
        if len(parts) >= 6:
            try:
                bat_str = f"  bat:{int(parts[5]) / 1000:.2f}V"
            except Exception:
                pass

    print(f"\n  PICO: {pico_col}{pico_lbl}\033[0m{bat_str}  {pico_st}")

    if link:
        avg = round(sum(lq_hist) / len(lq_hist), 1) if lq_hist else "--"
        print(f"  RC:   RSSI {link['rssi']}  LQ {link['lq']}%  avg {avg}%"
              f"  frames {_state['frames']}")

    print(f"\n  SA↑=arm  SB=corner  SC=plan/start  SC↓=pause  SD=abort"
          f"  spacing={args.spacing}m")

# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description="Solar Bot RC Drive + Mission Planner")
    ap.add_argument("--rc",       default=RC_PORT,   help=f"RC port    (default {RC_PORT})")
    ap.add_argument("--pico",     default=PICO_PORT, help=f"Pico UART  (default {PICO_PORT})")
    ap.add_argument("--gps",      default=GPS_PORT,  help=f"GPS port   (default {GPS_PORT})")
    ap.add_argument("--gps-baud", type=int, default=GPS_BAUD)
    ap.add_argument("--no-gps",   action="store_true",
                    help="Disable GPS; use dead-reckoning (IMU yaw + throttle speed) for navigation")
    ap.add_argument("--spacing",  type=float, default=0.12,
                    help="Lawnmower sweep line spacing in metres (default: 0.12 = robot width)")
    ap.add_argument("--overshoot", type=float, default=0.5,
                    help="Extra metres past polygon edge on each sweep line (default: 0.5)")
    ap.add_argument("--no-web",   action="store_true", help="Disable web dashboard")
    ap.add_argument("--web-port", type=int,   default=8080,
                    help="Web dashboard port (default: 8080)")
    # ── Saved mission / path loading ─────────────────────────────────────────
    ap.add_argument("--resume",          action="store_true",
                    help="Load the most recent saved mission (goes straight to PLANNED)")
    ap.add_argument("--resume-boundary", action="store_true",
                    help="Reload boundary corners from boundary_current.json (continues recording)")
    ap.add_argument("--mission",         default=None, metavar="FILE",
                    help="Load a specific mission JSON file  e.g. paths/mission_20241201_143022.json")
    ap.add_argument("--list-missions",   action="store_true",
                    help="List all saved missions and exit")
    args = ap.parse_args()
    PATH_DIR.mkdir(parents=True, exist_ok=True)
    run(args)

if __name__ == "__main__":
    main()
