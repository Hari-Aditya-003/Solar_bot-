"""
Solar Bot Web Dashboard — Flask SSE server
Browse to http://<raspberry-pi-ip>:8080 from any device on the same WiFi.

Start from rc_drive.py:
    import web_ui
    web_ui.start(_state, _boundary, lambda: _mission, _stop, port=8080)
"""

import json
import math
import threading
import time

# References set by start()
_state_ref     = None
_boundary_ref  = None
_mission_fn    = None   # callable → current MissionRunner or None
_stop_event    = None
_web_port      = 8080

_NET_FILE = "/tmp/solarbot_net.json"

# ── Network info ──────────────────────────────────────────────────────────────

def get_net_info():
    """
    Read /tmp/solarbot_net.json written by auto_hotspot.sh.
    Falls back to a basic IP scan if the file is missing.
    Returns dict: {mode, ssid, ip, password, url}
    """
    try:
        with open(_NET_FILE) as f:
            d = json.load(f)
        ip = d.get("ip", "")
        d["url"] = f"http://{ip}:{_web_port}" if ip else ""
        return d
    except Exception:
        pass

    # Fallback: grab first non-loopback IPv4
    try:
        import socket
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return {"mode": "wifi", "ssid": "", "ip": ip,
                "password": "", "url": f"http://{ip}:{_web_port}"}
    except Exception:
        return {"mode": "unknown", "ssid": "", "ip": "", "password": "", "url": ""}

_R = 6_371_000.0

# ── Geometry (duplicated so web_ui has no circular import) ────────────────────

def _haversine(lat1, lon1, lat2, lon2):
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (math.sin(dlat / 2) ** 2 +
         math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) *
         math.sin(dlon / 2) ** 2)
    return _R * 2 * math.asin(math.sqrt(a))

def _bearing(lat1, lon1, lat2, lon2):
    dlon = math.radians(lon2 - lon1)
    x = math.sin(dlon) * math.cos(math.radians(lat2))
    y = (math.cos(math.radians(lat1)) * math.sin(math.radians(lat2)) -
         math.sin(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.cos(dlon))
    return (math.degrees(math.atan2(x, y)) + 360) % 360

# ── State snapshot ────────────────────────────────────────────────────────────

def _snapshot():
    s   = _state_ref
    bd  = _boundary_ref
    m   = _mission_fn() if _mission_fn else None
    fix = s.get("gps_fix")

    # Odometry mode: GPS is disabled, dead reckoning is used instead
    odo_mode = not s.get("gps_on", False)

    yaw = roll = pitch = bat_v = 0.0
    pico_st = s.get("pico_status", "")
    try:
        parts = pico_st.split()
        if len(parts) >= 6:
            bat_v = int(parts[5]) / 1000
        if len(parts) >= 4:
            yaw = float(parts[3])
        if len(parts) >= 9:
            roll  = float(parts[7])
            pitch = float(parts[8])
    except Exception:
        pass

    link    = s.get("link", {})
    wp_idx  = s.get("wp_index", 0)
    wp_dist = wp_brg = None

    odo_x = s.get("odo_x", 0.0)
    odo_y = s.get("odo_y", 0.0)
    odo_dist = s.get("odo_dist", 0.0)

    if odo_mode:
        # Dead-reckoning: pass robot position as canvas coords (a=y/north, b=x/east)
        robot_a = odo_y
        robot_b = odo_x
        # Compute wp_dist/brg from odo position
        if m:
            wps = m.waypoints
            if wp_idx < len(wps):
                wx, wy = wps[wp_idx]
                wp_dist = round(math.hypot(wx - odo_x, wy - odo_y), 2)
                wp_brg  = round((math.degrees(math.atan2(wx - odo_x, wy - odo_y)) + 360) % 360, 1)
        # Send boundary/waypoints as [y, x] so canvas treats y as vertical (north-up)
        def _bnd_pair(c):  return [c[1], c[0]]   # (x,y) → [y, x]
        def _wp_pair(w):   return [w[1], w[0]]
    else:
        robot_a = fix.lat if fix and fix.has_fix else None
        robot_b = fix.lon if fix and fix.has_fix else None
        if m and fix and getattr(fix, "has_fix", False):
            wps = m.waypoints
            if wp_idx < len(wps):
                wp_dist = round(_haversine(fix.lat, fix.lon, *wps[wp_idx]), 2)
                wp_brg  = round(_bearing(fix.lat, fix.lon, *wps[wp_idx]), 1)
        def _bnd_pair(c):  return list(c)
        def _wp_pair(w):   return list(w)

    return {
        "armed":          s.get("armed", False),
        "failsafe":       s.get("failsafe", True),
        "steer":          s.get("steer", 0),
        "throttle":       s.get("throttle", 0),
        "pico_ok":        s.get("pico_ok", False),
        "pico_seen":      s.get("pico_seen", False),
        "battery_v":      round(bat_v, 2),
        "yaw_deg":        round(yaw, 1),
        "roll":           round(roll, 1),
        "pitch":          round(pitch, 1),
        "gps_ok":         s.get("gps_ok", False),
        "robot_lat":      robot_a,
        "robot_lon":      robot_b,
        "gps_sats":       fix.satellites if fix else 0,
        "gps_speed":      round(fix.speed_ms, 2) if fix else 0,
        "odo_mode":       odo_mode,
        "odo_x":          round(odo_x, 3),
        "odo_y":          round(odo_y, 3),
        "odo_dist":       round(odo_dist, 2),
        "rc_rssi":        link.get("rssi", 0),
        "rc_lq":          link.get("lq", 0),
        "rc_frames":      s.get("frames", 0),
        "mission_mode":   s.get("mission_mode", "IDLE"),
        "boundary_count": bd.count if bd else 0,
        "boundary":       [_bnd_pair(c) for c in (bd.corners if bd else [])],
        "waypoints":      [_wp_pair(w) for w in (m.waypoints if m else [])],
        "wp_index":       wp_idx,
        "wp_total":       m.total if m else 0,
        "wp_dist":        wp_dist,
        "wp_bearing":     wp_brg,
        "net":            get_net_info(),
    }

# ── HTML page (embedded) ──────────────────────────────────────────────────────

_HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Solar Bot Monitor</title>
<style>
:root{--bg:#0d1117;--card:#161b27;--border:#21273a;--text:#d4dff0;--dim:#7a8faa;
 --green:#22c55e;--yellow:#eab308;--red:#ef4444;--blue:#3b82f6;--cyan:#06b6d4;--orange:#f97316}
*{box-sizing:border-box;margin:0;padding:0}
body{background:var(--bg);color:var(--text);font-family:'Segoe UI',system-ui,sans-serif;
 font-size:14px;height:100vh;display:flex;flex-direction:column;overflow:hidden}
header{background:var(--card);border-bottom:1px solid var(--border);padding:9px 16px;
 display:flex;justify-content:space-between;align-items:center;flex-shrink:0}
header h1{font-size:1rem;color:var(--cyan);letter-spacing:.5px;font-weight:600}
.hbadges{display:flex;gap:8px;align-items:center}
.badge{font-size:.72rem;padding:3px 10px;border-radius:12px;font-weight:700;letter-spacing:.5px}
.net-bar{padding:6px 16px;font-size:.82rem;display:flex;gap:16px;align-items:center;
 flex-shrink:0;border-bottom:1px solid var(--border)}
.net-bar.hotspot{background:#1a1200;border-bottom-color:#78350f}
.net-bar.wifi{background:#0a1a0a;border-bottom-color:#14532d}
.net-tag{font-weight:700;padding:2px 9px;border-radius:10px;font-size:.72rem}
.net-bar.hotspot .net-tag{background:#78350f;color:#fef3c7}
.net-bar.wifi .net-tag{background:#14532d;color:#bbf7d0}
.qr-hint{margin-left:auto;color:var(--dim);font-size:.78rem}
.main{display:grid;grid-template-columns:370px 1fr;gap:8px;padding:8px;flex:1;min-height:0}
.left{display:flex;flex-direction:column;gap:7px;overflow-y:auto;scrollbar-width:thin}
.left::-webkit-scrollbar{width:4px}
.left::-webkit-scrollbar-thumb{background:var(--border);border-radius:2px}
.card{background:var(--card);border:1px solid var(--border);border-radius:8px;padding:11px 14px}
.card h2{font-size:.72rem;text-transform:uppercase;color:var(--dim);letter-spacing:1.3px;
 margin-bottom:9px;font-weight:700;border-bottom:1px solid var(--border);padding-bottom:5px}
.row{display:flex;justify-content:space-between;align-items:center;padding:4px 0;
 border-bottom:1px solid rgba(33,39,58,0.7);font-size:.87rem;min-height:26px;gap:8px}
.row:last-child{border:none}
.lbl{color:var(--dim);flex-shrink:0}
.val{font-weight:600;text-align:right}
.ok{color:var(--green)}.warn{color:var(--yellow)}.err{color:var(--red)}.info{color:var(--cyan)}
/* Compass widget */
.hdg-widget{display:flex;align-items:center;gap:14px;margin-bottom:8px}
.hdg-right{flex:1}
.hdg-deg{font-size:2.6rem;font-weight:700;color:var(--cyan);line-height:1}
.hdg-dir{font-size:.9rem;color:var(--dim);margin-top:3px;letter-spacing:1.5px;font-weight:600}
.hdg-sub{font-size:.78rem;color:var(--dim);margin-top:7px;line-height:1.6}
/* Tilt bars */
.tilt-row{display:flex;align-items:center;gap:8px;margin-top:5px}
.tilt-lbl{font-size:.72rem;color:var(--dim);width:34px;text-align:right;font-weight:600}
/* Mission steps */
.steps{display:flex;flex-direction:column;gap:4px;margin-top:7px}
.step{display:flex;gap:8px;font-size:.8rem;padding:6px 8px;border-radius:5px;
 background:rgba(255,255,255,0.02);border:1px solid var(--border);align-items:flex-start;
 line-height:1.4}
.step-active{border-color:var(--green)!important;background:rgba(34,197,94,0.08)!important}
.step-done{opacity:.4}
.step-num{font-weight:700;color:var(--cyan);min-width:18px;flex-shrink:0;font-size:.85rem}
.sw{display:inline-block;background:#1e293b;border:1px solid #334155;border-radius:4px;
 padding:1px 7px;font-size:.75rem;font-weight:700;color:var(--yellow);white-space:nowrap}
/* Switch table */
.sw-table{width:100%;font-size:.8rem;border-collapse:collapse;margin-top:9px}
.sw-table td{padding:5px 7px;border-bottom:1px solid var(--border)}
.sw-table tr:last-child td{border:none}
.sw-name{color:var(--yellow);font-weight:700;white-space:nowrap;font-size:.85rem}
.sw-low{color:var(--dim)}.sw-high{color:var(--green);font-weight:600}
/* Progress */
.pbar-bg{height:7px;background:var(--border);border-radius:4px;margin:6px 0;overflow:hidden}
.pbar-fill{height:100%;background:var(--green);border-radius:4px;transition:width .4s}
/* Buttons */
.btns{display:flex;gap:6px;margin-top:9px}
.btn{flex:1;padding:9px 4px;border:none;border-radius:6px;font-size:.85rem;
 font-weight:700;cursor:pointer;letter-spacing:.3px;transition:opacity .15s}
.btn:hover{opacity:.85}
.btn-stop{background:#dc2626;color:#fff}
.btn-pause{background:#b45309;color:#fff}
/* Guide toggle */
.gtoggle{width:100%;background:none;border:1px solid var(--border);color:var(--dim);
 border-radius:5px;padding:5px 10px;font-size:.78rem;cursor:pointer;margin-top:8px;text-align:left}
.gbody{display:none;margin-top:6px}
.gbody.open{display:block}
/* Web Controls */
.ctrl-grid{display:grid;grid-template-columns:1fr 1fr;gap:6px;margin-top:7px}
.ctrl-btn{width:100%;padding:9px 6px;border:none;border-radius:5px;font-size:.8rem;
 font-weight:700;cursor:pointer;transition:opacity .15s;letter-spacing:.3px;line-height:1.3}
.ctrl-btn:hover{opacity:.8}.ctrl-btn:active{opacity:.6}
.ctrl-arm{background:#14532d;color:#86efac}
.ctrl-disarm{background:#7f1d1d;color:#fca5a5}
.ctrl-mark{background:#1e3a5f;color:#93c5fd}
.ctrl-plan{background:#422006;color:#fdba74}
.ctrl-run{background:#064e3b;color:#6ee7b7}
.ctrl-abort{background:#7f1d1d;color:#fca5a5}
.opt-row{display:flex;justify-content:space-between;align-items:center;
 padding:5px 0;font-size:.82rem}
.opt-label{color:var(--dim)}.opt-val{color:var(--cyan);font-weight:700}
.divider{height:1px;background:var(--border);margin:8px 0}
select.opt-sel{background:#0d1117;color:var(--text);border:1px solid var(--border);
 border-radius:4px;font-size:.8rem;padding:3px 6px;cursor:pointer}
input[type=range]{accent-color:#06b6d4;cursor:pointer;width:100%;margin:4px 0 6px}
/* Virtual joystick */
.joy-wrap{display:flex;flex-direction:column;align-items:center;gap:6px;margin-top:4px}
.joy-pad{width:160px;height:160px;border-radius:50%;background:#0a0f17;
 border:2px solid var(--border);position:relative;cursor:grab;
 touch-action:none;user-select:none;flex-shrink:0}
.joy-pad:active{cursor:grabbing}
.joy-ring{position:absolute;inset:22px;border-radius:50%;border:1px dashed rgba(255,255,255,0.07);pointer-events:none}
.joy-cr-h{position:absolute;top:50%;left:18px;right:18px;height:1px;background:rgba(255,255,255,0.06);transform:translateY(-50%);pointer-events:none}
.joy-cr-v{position:absolute;left:50%;top:18px;bottom:18px;width:1px;background:rgba(255,255,255,0.06);transform:translateX(-50%);pointer-events:none}
.joy-knob{position:absolute;width:54px;height:54px;border-radius:50%;
 background:radial-gradient(circle at 38% 32%,#4a5568,#1a2030);
 border:2px solid #334155;top:50%;left:50%;transform:translate(-50%,-50%);
 box-shadow:0 3px 10px rgba(0,0,0,0.7);pointer-events:none;transition:border-color .1s,box-shadow .1s}
.joy-knob.joy-active{border-color:var(--cyan);box-shadow:0 0 14px rgba(6,182,212,0.5)}
.joy-readout{display:flex;justify-content:center;gap:22px;font-size:.85rem}
.joy-field{color:var(--dim)}.joy-field b{color:var(--cyan)}
.joy-hint{font-size:.72rem;color:var(--dim);text-align:center}
kbd{background:#1a2030;border:1px solid #334155;border-radius:3px;
 padding:0 4px;font-size:.7rem;color:var(--text)}
/* Map save buttons */
.btn-mapsave{background:var(--bg0);border:1px solid var(--border);border-radius:5px;
 color:var(--dim);font-size:.72rem;padding:3px 9px;cursor:pointer;white-space:nowrap}
.btn-mapsave:hover{border-color:var(--accent);color:var(--text)}
/* Map */
.right-col{display:flex;flex-direction:column;min-height:0}
.map-card{flex:1;display:flex;flex-direction:column;min-height:0}
.map-top{display:flex;justify-content:space-between;align-items:flex-start;flex-wrap:wrap;gap:6px;margin-bottom:5px}
.map-hud{display:flex;gap:14px}
.hud-item{display:flex;flex-direction:column;align-items:center;min-width:52px}
.hud-val{font-size:1.2rem;font-weight:700;color:var(--cyan);line-height:1}
.hud-lbl{color:var(--dim);font-size:.65rem;margin-top:3px;letter-spacing:.6px;font-weight:600}
.legend{display:flex;gap:10px;flex-wrap:wrap;font-size:.72rem;color:var(--dim);margin-bottom:5px}
.li{display:flex;align-items:center;gap:4px}
.ld{width:14px;height:4px;border-radius:2px;flex-shrink:0}
.map-wrap{flex:1;position:relative;min-height:180px}
#map{position:absolute;inset:0;width:100%;height:100%}
#map.draw-active{cursor:crosshair!important}
/* Survey panel — floats over map, right side */
.sv-panel{position:absolute;top:10px;right:10px;width:192px;z-index:20;
 background:rgba(10,14,22,0.93);border:1px solid var(--border);border-radius:8px;
 padding:10px 12px;backdrop-filter:blur(6px);display:none}
.sv-title{font-size:.7rem;font-weight:700;color:var(--dim);letter-spacing:1.2px;
 text-transform:uppercase;border-bottom:1px solid var(--border);padding-bottom:5px;margin-bottom:8px}
.sv-lbl{font-size:.74rem;color:var(--dim);margin-bottom:2px;display:block}
.sv-row{margin-bottom:8px}
.sv-inline{display:flex;align-items:center;gap:6px}
.sv-val{color:var(--cyan);font-weight:700;font-size:.8rem;min-width:38px;text-align:right}
.sv-stat{font-size:.78rem;color:var(--text-dim,#8b949e);margin-bottom:3px}
.sv-stat b{color:var(--cyan)}
.sv-div{height:1px;background:var(--border);margin:7px 0}
.sv-btn{width:100%;padding:8px 6px;border:none;border-radius:5px;font-size:.8rem;
 font-weight:700;cursor:pointer;margin-bottom:5px;letter-spacing:.3px}
.sv-gen{background:#14532d;color:#86efac}.sv-gen:hover{background:#166534}
.sv-start{background:var(--green);color:#000}.sv-start:hover{background:#16a34a}
.sv-clr{background:#4a1010;color:#fca5a5;margin-bottom:0}.sv-clr:hover{background:#7f1d1d}
.ms-panel{position:absolute;top:10px;left:10px;width:260px;z-index:25;background:#0d1520;
 border:1px solid var(--accent);border-radius:8px;padding:10px;display:none}
.ms-title{font-size:.75rem;font-weight:700;color:var(--accent);letter-spacing:.06em;
 margin-bottom:6px;display:flex;justify-content:space-between;align-items:center}
.ms-list{max-height:220px;overflow-y:auto;display:flex;flex-direction:column;gap:4px}
.ms-item{background:#111c2b;border:1px solid var(--border);border-radius:5px;
 padding:6px 8px;cursor:pointer;font-size:.76rem;color:var(--text)}
.ms-item:hover{border-color:var(--accent);background:#162030}
.ms-item b{display:block;color:var(--text);margin-bottom:2px;word-break:break-all}
.ms-item span{color:var(--dim);font-size:.71rem}
.ms-close{background:none;border:none;color:var(--dim);cursor:pointer;font-size:1rem;padding:0 2px}
.status-bar{background:var(--card);border-top:1px solid var(--border);padding:5px 14px;
 font-size:.76rem;color:var(--dim);display:flex;gap:18px;flex-shrink:0;flex-wrap:wrap;
 align-items:center}
@media(max-width:740px){.main{grid-template-columns:1fr;overflow-y:auto}
 .right-col{min-height:60vh}.hdg-deg{font-size:1.8rem}}
</style>
</head>
<body>
<header>
 <h1>&#9728; Solar Panel Cleaning Robot &mdash; Live Monitor</h1>
 <div class="hbadges">
  <span id="mode-badge" class="badge" style="background:#1e293b">IDLE</span>
  <span id="conn-badge" class="badge" style="background:#374151">OFFLINE</span>
 </div>
</header>
<div id="net-bar" class="net-bar">
 <span id="net-tag" class="net-tag">NETWORK</span>
 <span id="net-info">Detecting&hellip;</span>
 <span class="qr-hint" id="net-url"></span>
</div>

<div class="main">
 <!-- ═══ LEFT PANEL ═══ -->
 <div class="left">

  <!-- System -->
  <div class="card">
   <h2>System Status</h2>
   <div class="row"><span class="lbl">Arm state</span><span class="val" id="s-armed">—</span></div>
   <div class="row"><span class="lbl">RC link</span><span class="val" id="s-rc">—</span></div>
   <div class="row"><span class="lbl">Battery</span><span class="val" id="s-bat">—</span></div>
   <div class="row"><span class="lbl">Pico MCU</span><span class="val" id="s-pico">—</span></div>
   <div class="row"><span class="lbl">Motors L / R</span><span class="val" id="s-motors">—</span></div>
  </div>

  <!-- Heading + Tilt -->
  <div class="card">
   <h2>Heading &amp; Orientation</h2>
   <div class="hdg-widget">
    <canvas id="compass" width="96" height="96" style="display:block;flex-shrink:0;border-radius:50%;width:96px;height:96px"></canvas>
    <div class="hdg-right">
     <div class="hdg-deg" id="hdg-deg">0&deg;</div>
     <div class="hdg-dir" id="hdg-dir">NORTH</div>
     <div class="hdg-sub">
      Roll &nbsp;<span id="hdg-roll" style="color:var(--cyan)">0.0&deg;</span><br>
      Pitch <span id="hdg-pitch" style="color:var(--cyan)">0.0&deg;</span>
     </div>
    </div>
   </div>
   <div class="tilt-row">
    <span class="tilt-lbl">ROLL</span>
    <canvas id="tilt-roll" style="display:block;flex:1;height:20px;border-radius:4px"></canvas>
   </div>
   <div class="tilt-row" style="margin-top:3px">
    <span class="tilt-lbl">PITCH</span>
    <canvas id="tilt-pitch" style="display:block;flex:1;height:20px;border-radius:4px"></canvas>
   </div>
  </div>

  <!-- Position -->
  <div class="card">
   <h2>Position</h2>
   <div class="row"><span class="lbl">Nav mode</span><span class="val" id="g-fix">—</span></div>
   <div class="row"><span class="lbl">Location</span>
    <span class="val" id="g-pos" style="font-size:.8rem;text-align:right">—</span></div>
   <div class="row"><span class="lbl" id="g-sats-lbl">Satellites</span><span class="val" id="g-sats">—</span></div>
   <div class="row"><span class="lbl">Speed</span><span class="val" id="g-spd">—</span></div>
  </div>

  <!-- Mission + guide -->
  <div class="card">
   <h2>Mission Control</h2>
   <div class="row"><span class="lbl">State</span><span class="val" id="m-mode">IDLE</span></div>
   <div class="row"><span class="lbl">Boundary</span><span class="val" id="m-bc">0 corners</span></div>
   <div class="row"><span class="lbl">Progress</span><span class="val" id="m-prog">—</span></div>
   <div class="pbar-bg"><div class="pbar-fill" id="m-bar" style="width:0%"></div></div>
   <div class="row"><span class="lbl">Next waypoint</span><span class="val" id="m-dist">—</span></div>
   <div class="row"><span class="lbl">Sweep pass</span><span class="val" id="m-sweep">—</span></div>
   <div class="btns">
    <button class="btn btn-stop"  onclick="cmd('stop')">&#9940; STOP</button>
    <button class="btn btn-pause" onclick="cmd('pause')">&#9646;&#9646; Pause</button>
   </div>

   <button class="gtoggle" onclick="toggleGuide(this)">&#9660; Switch Workflow Guide</button>
   <div class="gbody" id="gbody">
    <div class="steps">
     <div class="step" id="gs0"><span class="step-num">1</span>
      Drive robot to first corner of the solar panel area</div>
     <div class="step" id="gs1"><span class="step-num">2</span>
      Flip <span class="sw">SB &uarr;</span> to mark corner &mdash; repeat for &ge;3 corners</div>
     <div class="step" id="gs2"><span class="step-num">3</span>
      Flip <span class="sw">SC &uarr;</span> once &rarr; lawnmower path generated</div>
     <div class="step" id="gs3"><span class="step-num">4</span>
      <span class="sw">SA &uarr;</span> ARM, then <span class="sw">SC &uarr;</span> again &rarr; sweep starts</div>
     <div class="step" id="gs4"><span class="step-num">5</span>
      <span class="sw">SC &darr;</span> pause &nbsp;|&nbsp; <span class="sw">SD &uarr;</span> abort &amp; reset</div>
    </div>
    <table class="sw-table">
     <tr><td class="sw-name">SA</td><td class="sw-low">&#8595; DISARMED &mdash; motors locked</td>
      <td class="sw-high">&#8593; ARMED</td></tr>
     <tr><td class="sw-name">SB</td><td class="sw-low">&#8595; idle</td>
      <td class="sw-high">&#8593; Mark corner (edge)</td></tr>
     <tr><td class="sw-name">SC</td><td class="sw-low">&#8595; Pause mission</td>
      <td class="sw-high">&#8593; Plan / Start (edge)</td></tr>
     <tr><td class="sw-name">SD</td><td class="sw-low">&#8595; idle</td>
      <td class="sw-high">&#8593; ABORT + RESET (edge)</td></tr>
    </table>
   </div>
  </div>

  <!-- Web Controls — Switch Alternatives -->
  <div class="card">
   <h2>Web Controls — Switch Alternatives</h2>
   <div style="font-size:.78rem;color:var(--dim);margin-bottom:8px">
    Replaces SA / SB / SC / SD physical switches
   </div>
   <div class="ctrl-grid">
    <button class="ctrl-btn ctrl-arm"    onclick="webCmd('arm')">SA &#8593; &nbsp;ARM</button>
    <button class="ctrl-btn ctrl-disarm" onclick="webCmd('disarm')">SA &#8595; &nbsp;DISARM</button>
    <button class="ctrl-btn ctrl-mark"   onclick="webCmd('mark_corner')">SB &#8593; &nbsp;Mark Corner</button>
    <button class="ctrl-btn ctrl-plan"   onclick="webCmd('plan')">SC &#8593; &nbsp;Plan / Start</button>
    <button class="ctrl-btn ctrl-run"    onclick="webCmd('resume')">SC &#8594; &nbsp;Resume</button>
    <button class="ctrl-btn ctrl-abort"  onclick="webCmd('abort')">SD &#8593; &nbsp;Abort + Reset</button>
   </div>
   <div class="divider"></div>
   <div class="opt-row">
    <span class="opt-label">Row Spacing</span>
    <span class="opt-val" id="rs-val">0.8 m</span>
   </div>
   <input type="range" id="row-spacing" min="0.3" max="3.0" step="0.1" value="0.8"
    oninput="el('rs-val').textContent=parseFloat(this.value).toFixed(1)+' m'; const s=el('sv-spacing'); if(s){s.value=this.value;el('sv-spacing-v').textContent=parseFloat(this.value).toFixed(1)+'m';}"
    >
   <div class="opt-row">
    <span class="opt-label">Sweep Angle</span>
    <select class="opt-sel" id="sweep-angle">
     <option value="auto">Auto (longest edge)</option>
     <option value="0">0&#176; &#8212; North&#8211;South</option>
     <option value="90">90&#176; &#8212; East&#8211;West</option>
     <option value="45">45&#176;</option>
    </select>
   </div>
  </div>

  <!-- Manual Drive — Web Joystick -->
  <div class="card">
   <h2>Manual Drive &mdash; Web Joystick</h2>
   <div class="joy-hint" style="margin-bottom:6px">ARM first &middot; drag to drive &middot; release = stop</div>
   <div class="joy-wrap">
    <div class="joy-pad" id="joy-pad"
     onmousedown="joyDown(event)" ontouchstart="joyDown(event)">
     <div class="joy-ring"></div>
     <div class="joy-cr-h"></div>
     <div class="joy-cr-v"></div>
     <div class="joy-knob" id="joy-knob"></div>
    </div>
    <div class="joy-readout">
     <span class="joy-field">THR <b id="joy-thr">0%</b></span>
     <span class="joy-field">STR <b id="joy-str">CTR</b></span>
    </div>
   </div>
   <div class="joy-hint" style="margin-top:5px">
    <kbd>&uarr;</kbd><kbd>&darr;</kbd> throttle &nbsp;
    <kbd>&larr;</kbd><kbd>&rarr;</kbd> steer &nbsp;
    <kbd>Space</kbd> stop
   </div>
  </div>

 </div><!-- /left -->

 <!-- ═══ MAP AREA ═══ -->
 <div class="right-col">
  <div class="card map-card">
   <div class="map-top">
    <div style="display:flex;align-items:center;gap:8px;flex-wrap:wrap">
     <h2 style="margin:0">Coverage Map</h2>
     <button class="btn-mapsave" id="draw-btn" onclick="toggleDrawMode()" title="Click on map to draw boundary corners · Right-click=undo · Double-click=done">&#9998; Draw</button>
     <button class="btn-mapsave" onclick="clearBoundary()" title="Remove all boundary corners">&#10005; Clear</button>
     <button class="btn-mapsave" onclick="saveMapPNG()" title="Download canvas as PNG">&#11015; PNG</button>
     <button class="btn-mapsave" onclick="saveField()" title="Save boundary+waypoints JSON to Pi">&#128190; Field</button>
     <button class="btn-mapsave" onclick="showMissions()" title="Load a previously saved mission">&#128193; History</button>
    </div>
    <div class="map-hud">
     <div class="hud-item">
      <div class="hud-val" id="hud-hdg">0&deg;</div>
      <div class="hud-lbl">HEADING</div>
     </div>
     <div class="hud-item">
      <div class="hud-val" id="hud-spd">0.00</div>
      <div class="hud-lbl">M/S</div>
     </div>
     <div class="hud-item">
      <div class="hud-val" id="hud-thr">0%</div>
      <div class="hud-lbl">THROTTLE</div>
     </div>
     <div class="hud-item">
      <div class="hud-val" id="hud-steer">CTR</div>
      <div class="hud-lbl">STEER</div>
     </div>
    </div>
   </div>
   <div class="legend">
    <div class="li"><div class="ld" style="background:#22c55e"></div>Cleaned</div>
    <div class="li"><div class="ld" style="background:#facc15;height:6px"></div>Active pass</div>
    <div class="li"><div class="ld" style="background:#2563eb"></div>Planned</div>
    <div class="li"><div class="ld" style="background:rgba(59,100,180,0.25);border:1px dashed #3b4a6a"></div>Grid base</div>
    <div class="li"><div class="ld" style="background:#22c55e;border-radius:50%;width:10px;height:10px"></div>Start</div>
    <div class="li"><div class="ld" style="background:#475569;width:10px;height:10px;border-radius:50%"></div>Corner</div>
    <div class="li">
     <svg width="14" height="14" viewBox="-7 -7 14 14" style="flex-shrink:0">
      <polygon points="0,-6 5,5 0,2 -5,5" fill="#ef4444" stroke="#fff" stroke-width="1.2"/>
     </svg>Robot
    </div>
    <div class="li"><div class="ld" style="background:rgba(6,182,212,0.45)"></div>Trail</div>
   </div>
   <div class="map-wrap">
    <canvas id="map"></canvas>
    <!-- Mission history panel (floats over map, top-left) -->
    <div id="ms-panel" class="ms-panel">
     <div class="ms-title">
      <span>&#128193; Saved Missions</span>
      <button class="ms-close" onclick="el('ms-panel').style.display='none'" title="Close">&#10005;</button>
     </div>
     <div class="ms-list" id="ms-list"><span style="color:var(--dim);font-size:.76rem">Loading...</span></div>
    </div>
    <!-- Survey settings panel (floats over map) -->
    <div id="sv-panel" class="sv-panel">
     <div class="sv-title">&#128208; Survey Settings</div>
     <div class="sv-row">
      <span class="sv-lbl">Row Spacing</span>
      <div class="sv-inline">
       <input type="range" id="sv-spacing" min="0.3" max="5" step="0.1" value="0.8" style="flex:1"
        oninput="el('sv-spacing-v').textContent=parseFloat(this.value).toFixed(1)+'m'">
       <span class="sv-val" id="sv-spacing-v">0.8m</span>
      </div>
     </div>
     <div class="sv-row">
      <span class="sv-lbl">Sweep Angle</span>
      <select id="sv-angle" style="width:100%;background:#0a0f17;color:var(--text);
       border:1px solid var(--border);border-radius:4px;font-size:.78rem;padding:3px 5px">
       <option value="auto">Auto (longest edge)</option>
       <option value="0">0&#176; — North–South</option>
       <option value="90">90&#176; — East–West</option>
       <option value="45">45&#176;</option>
       <option value="135">135&#176;</option>
      </select>
     </div>
     <div class="sv-row">
      <span class="sv-lbl">Overshoot (turnaround)</span>
      <div class="sv-inline">
       <input type="range" id="sv-overshoot" min="0" max="5" step="0.5" value="1.0" style="flex:1"
        oninput="el('sv-overshoot-v').textContent=parseFloat(this.value).toFixed(1)+'m'">
       <span class="sv-val" id="sv-overshoot-v">1.0m</span>
      </div>
     </div>
     <div class="sv-div"></div>
     <div class="sv-stat">Corners: <b id="sv-corners">0</b></div>
     <div class="sv-stat">Passes: <b id="sv-passes">—</b></div>
     <div class="sv-stat">Distance: <b id="sv-dist">—</b></div>
     <div class="sv-stat">Est. time: <b id="sv-time">—</b></div>
     <div class="sv-div"></div>
     <button class="sv-btn sv-gen" onclick="surveyGenerate()">&#10003; Generate Path</button>
     <button class="sv-btn sv-start" onclick="webCmd('resume')" id="sv-start-btn" style="display:none">&#9654; Start Mission</button>
     <button class="sv-btn sv-clr" onclick="clearBoundary()">&#10005; Clear Field</button>
    </div>
   </div>
  </div>
 </div>
</div><!-- /main -->

<div class="status-bar">
 <span id="ts">—</span>
 <span id="st-frames">RC: —</span>
 <span>Dashboard: <b style="color:var(--cyan)" id="st-ip">—</b></span>
 <span style="margin-left:auto">SB=corner &nbsp; SC=plan/run &nbsp; SD=abort</span>
</div>

<script>
"use strict";
let D = null;
const trail = [];
const TRAIL_MAX = 100;
// Draw-mode state
let _drawMode    = false;
let _mapView     = null;   // {minA,maxA,minB,maxB,M,W,H} — updated every frame
let _drawHover   = null;   // [canvasPx, canvasPy] of mouse, or null
let _dragCorner  = null;   // {idx, hasMoved, origPx:[x,y]} while dragging a corner
let _dragPos     = null;   // [canvasPx, canvasPy] current drag position
let _didDrag     = false;  // true after a completed corner drag, suppresses next click

// ─── SSE ──────────────────────────────────────────────────────────────────────
const src = new EventSource("/stream");
src.onopen  = () => badge("conn-badge","LIVE","#16a34a");
src.onerror = () => badge("conn-badge","OFFLINE","#991b1b");
src.onmessage = e => { D = JSON.parse(e.data); update(D); };

function el(id){ return document.getElementById(id); }
function badge(id,txt,bg){ const e=el(id); e.textContent=txt; e.style.background=bg; }
function cv(id,text,cls){
  const e=el(id); if(!e) return;
  e.textContent=text;
  if(cls!==undefined) e.className="val "+cls;
}

// ─── Compass widget ────────────────────────────────────────────────────────────
function drawCompass(yawDeg) {
  const canvas = el("compass"); if (!canvas) return;
  const ctx = canvas.getContext("2d");
  const W = canvas.width, H = canvas.height;
  const cx = W/2, cy = H/2, r = Math.min(W,H)/2 - 2;
  ctx.clearRect(0,0,W,H);

  // Background
  ctx.beginPath(); ctx.arc(cx,cy,r,0,Math.PI*2);
  ctx.fillStyle="#0d1117"; ctx.fill();
  ctx.strokeStyle="#21273a"; ctx.lineWidth=2; ctx.stroke();

  // Tick marks
  for(let deg=0; deg<360; deg+=10){
    const a=(deg-90)*Math.PI/180;
    const big=(deg%90===0), med=(deg%45===0&&!big);
    const r0 = big ? r-11 : med ? r-7 : r-4;
    ctx.beginPath();
    ctx.moveTo(cx+Math.cos(a)*r0, cy+Math.sin(a)*r0);
    ctx.lineTo(cx+Math.cos(a)*(r-1), cy+Math.sin(a)*(r-1));
    ctx.strokeStyle = big ? "#475569" : med ? "#2d3748" : "#1e293b";
    ctx.lineWidth   = big ? 2 : 1;
    ctx.stroke();
  }

  // Cardinal labels N E S W
  ["N","E","S","W"].forEach((d,i)=>{
    const a=(i*90-90)*Math.PI/180, lr=r-9;
    ctx.fillStyle = d==="N" ? "#ef4444" : "#94a3b8";
    ctx.font = `bold ${d==="N"?14:11}px monospace`;
    ctx.textAlign="center"; ctx.textBaseline="middle";
    ctx.fillText(d, cx+Math.cos(a)*lr, cy+Math.sin(a)*lr);
  });

  // Heading arc (cyan fill behind needle)
  const yR = yawDeg*Math.PI/180;
  const arcW = Math.PI/8;
  ctx.beginPath();
  ctx.moveTo(cx,cy);
  ctx.arc(cx,cy,r-14, yR-Math.PI/2-arcW, yR-Math.PI/2+arcW);
  ctx.closePath();
  ctx.fillStyle="rgba(6,182,212,0.18)"; ctx.fill();

  // Needle
  ctx.save(); ctx.translate(cx,cy); ctx.rotate(yawDeg*Math.PI/180);
  const nL=r-8;
  // Forward (cyan)
  ctx.beginPath();
  ctx.moveTo(0,-nL); ctx.lineTo(5,2); ctx.lineTo(-5,2); ctx.closePath();
  ctx.fillStyle="#06b6d4"; ctx.fill();
  // Back (dim)
  ctx.beginPath();
  ctx.moveTo(0,nL*0.35); ctx.lineTo(4,2); ctx.lineTo(-4,2); ctx.closePath();
  ctx.fillStyle="#334155"; ctx.fill();
  ctx.restore();

  // Center hub
  ctx.beginPath(); ctx.arc(cx,cy,4,0,Math.PI*2);
  ctx.fillStyle="#fff"; ctx.fill();
  ctx.beginPath(); ctx.arc(cx,cy,2,0,Math.PI*2);
  ctx.fillStyle="#06b6d4"; ctx.fill();
}

// ─── Tilt bar ──────────────────────────────────────────────────────────────────
function drawTiltBar(canvasId, val, maxVal) {
  const canvas = el(canvasId); if (!canvas) return;
  const W = canvas.offsetWidth || 200, H = 20;
  canvas.width=W; canvas.height=H;
  const ctx = canvas.getContext("2d");
  const cx=W/2, pct=Math.max(-1,Math.min(1,val/maxVal));
  const bColor = Math.abs(pct)>0.8?"#ef4444":Math.abs(pct)>0.5?"#eab308":"#22c55e";

  ctx.fillStyle="#0d1117"; ctx.fillRect(0,0,W,H);
  // Track
  ctx.fillStyle="#1e293b";
  ctx.beginPath(); ctx.roundRect(6, H/2-3, W-12, 6, 3); ctx.fill();
  // Danger zones
  ctx.fillStyle="rgba(239,68,68,0.15)";
  ctx.fillRect(6, H/2-3, (W-12)*0.2, 6);
  ctx.fillRect(W-6-(W-12)*0.2, H/2-3, (W-12)*0.2, 6);
  // Centre mark
  ctx.fillStyle="#374151";
  ctx.fillRect(cx-1, H/2-6, 2, 12);
  // Bubble
  const bx = cx + pct*(W/2-14);
  ctx.beginPath(); ctx.arc(bx, H/2, 8, 0, Math.PI*2);
  ctx.fillStyle=bColor; ctx.fill();
  ctx.strokeStyle="rgba(255,255,255,0.6)"; ctx.lineWidth=1; ctx.stroke();
  // Value label
  ctx.fillStyle="#94a3b8"; ctx.font="8px monospace";
  ctx.textAlign="right"; ctx.textBaseline="middle";
  ctx.fillText(val.toFixed(1)+"°", W-2, H/2);
}

// ─── Cardinal direction ────────────────────────────────────────────────────────
function cardDir(deg){
  const d=((deg%360)+360)%360;
  return ["N","NE","E","SE","S","SW","W","NW","N"][Math.round(d/45)];
}

// ─── UI update ─────────────────────────────────────────────────────────────────
function update(d) {
  // Status
  if(d.failsafe)       cv("s-armed","FAILSAFE","err");
  else if(d.armed)     cv("s-armed","ARMED","ok");
  else                 cv("s-armed","DISARMED","warn");

  cv("s-rc", d.rc_frames>0
    ? `RSSI ${d.rc_rssi}  LQ ${d.rc_lq}%`
    : "NO SIGNAL",
    d.rc_frames>0 ? (d.rc_lq>70?"ok":"warn") : "err");

  const bv=d.battery_v||0;
  cv("s-bat", bv?bv.toFixed(2)+" V":"—", bv>11?"ok":bv>10?"warn":"err");
  cv("s-pico", d.pico_ok?"CONNECTED":d.pico_seen?"LOST":"SEARCHING",
     d.pico_ok?"ok":"err");

  if(d.failsafe||!d.armed){ cv("s-motors","STOP","err"); }
  else {
    const l=d.throttle-d.steer, r=d.throttle+d.steer;
    const p=Math.max(Math.abs(l),Math.abs(r),100);
    const lp=Math.round(l/p*100), rp=Math.round(r/p*100);
    cv("s-motors",`L ${lp>=0?"+":""}${lp}%  R ${rp>=0?"+":""}${rp}%`,"info");
  }

  // Heading widget
  const yaw=d.yaw_deg||0;
  const yawNorm=Math.round(((yaw%360)+360)%360);
  el("hdg-deg").textContent = yawNorm+"°";
  el("hdg-dir").textContent = cardDir(yaw);
  el("hdg-roll").textContent  = d.roll.toFixed(1)+"°";
  el("hdg-pitch").textContent = d.pitch.toFixed(1)+"°";
  drawCompass(yaw);
  drawTiltBar("tilt-roll",  d.roll,  30);
  drawTiltBar("tilt-pitch", d.pitch, 30);

  // HUD
  const spd = d.odo_mode
    ? (Math.abs(d.throttle)/100*0.68).toFixed(2)
    : (d.gps_speed||0).toFixed(2);
  el("hud-hdg").textContent   = yawNorm+"° "+cardDir(yaw);
  el("hud-spd").textContent   = spd;
  el("hud-thr").textContent   = (d.throttle>=0?"+":"")+d.throttle+"%";
  el("hud-steer").textContent = d.steer>0?"R"+d.steer:d.steer<0?"L"+Math.abs(d.steer):"CTR";

  // Position
  if(d.odo_mode){
    cv("g-fix","DEAD RECKONING","info");
    cv("g-pos",`x=${d.odo_x.toFixed(2)} m  y=${d.odo_y.toFixed(2)} m`,"info");
    const sl=el("g-sats-lbl"); if(sl) sl.textContent="Total dist";
    cv("g-sats", d.odo_dist.toFixed(2)+" m");
    cv("g-spd", spd+" m/s (est)");
  } else {
    cv("g-fix", d.gps_ok?"GPS FIX":"NO GPS FIX", d.gps_ok?"ok":"warn");
    cv("g-pos", d.robot_lat!=null
      ? `${d.robot_lat.toFixed(6)},\n${d.robot_lon.toFixed(6)}` : "—");
    const sl=el("g-sats-lbl"); if(sl) sl.textContent="Satellites";
    cv("g-sats", d.gps_sats+" sats");
    cv("g-spd", (d.gps_speed||0).toFixed(2)+" m/s");
  }

  // Mission
  const mc={IDLE:"#1e293b",RECORDING:"#78350f",PLANNED:"#1e3a5f",RUNNING:"#14532d",DONE:"#14532d"};
  badge("mode-badge", d.mission_mode, mc[d.mission_mode]||"#1e293b");
  const mcls={IDLE:"",RECORDING:"warn",PLANNED:"info",RUNNING:"ok",DONE:"ok"};
  cv("m-mode", d.mission_mode, mcls[d.mission_mode]||"");
  cv("m-bc", `${d.boundary_count} corner${d.boundary_count!==1?"s":""}`);
  const total=d.wp_total||0, idx=d.wp_index||0;
  const pct=total>0?Math.round(idx/total*100):0;
  cv("m-prog", total?`WP ${idx}/${total}  (${pct}%)`:"—", d.mission_mode==="RUNNING"?"ok":"");
  el("m-bar").style.width=pct+"%";
  if(d.wp_dist!=null)
    cv("m-dist",`${d.wp_dist.toFixed(2)} m  ·  ${(d.wp_bearing||0).toFixed(0)}° ${cardDir(d.wp_bearing||0)}`);
  else cv("m-dist","—");
  const sw=Math.floor(idx/2), ts2=Math.floor(total/2);
  cv("m-sweep", total?`Pass ${sw+1} / ${ts2}`:"—");

  // Highlight active step
  const stepMap={IDLE:0,RECORDING:1,PLANNED:2,RUNNING:3,DONE:4};
  const active=stepMap[d.mission_mode]??0;
  for(let i=0;i<=4;i++){
    const s=el("gs"+i); if(!s) continue;
    s.className="step"+(i===active?" step-active":i<active?" step-done":"");
  }

  // Network bar
  const net=d.net||{};
  const nb=el("net-bar");
  if(net.mode==="hotspot"){
    nb.className="net-bar hotspot";
    el("net-tag").textContent="HOTSPOT";
    el("net-info").innerHTML=`Pi broadcasting <b>${net.ssid||"SolarBot"}</b>`
      +` &middot; Password: <b>${net.password||"solarbot123"}</b>`
      +` &middot; Open <b>${net.url}</b>`;
    el("net-url").textContent="No router — hotspot mode";
  } else if(net.mode==="wifi"){
    nb.className="net-bar wifi";
    el("net-tag").textContent="WIFI";
    el("net-info").innerHTML=`Router${net.ssid?` "${net.ssid}"`:""}`;
    el("net-url").innerHTML=`<a href="${net.url}" style="color:var(--cyan)">${net.url}</a>`;
  } else {
    nb.className="net-bar";
    el("net-tag").textContent="NETWORK";
    el("net-info").textContent="Detecting...";
    el("net-url").textContent="";
  }

  el("ts").textContent=new Date().toLocaleTimeString();
  el("st-frames").textContent=`RC: ${d.rc_frames} frames`;
  el("st-ip").textContent=net.url||location.href;

  // Update trail
  if(d.robot_lat!=null){
    trail.push({a:d.robot_lat, b:d.robot_lon});
    if(trail.length>TRAIL_MAX) trail.shift();
  }

  // Survey panel — auto-show when field data exists, refresh stats
  const hasSurveyData=(d.boundary||[]).length>0||(d.waypoints||[]).length>0;
  if(hasSurveyData && !_drawMode) _svPanelVisible(true);
  if(hasSurveyData) _svPanelUpdate();
  // Show/hide "Start Mission" based on mission state
  const ssb=el("sv-start-btn");
  if(ssb) ssb.style.display=(d.mission_mode==="PLANNED"||(d.waypoints||[]).length>0)?"block":"none";
}

// ─── Guide toggle ──────────────────────────────────────────────────────────────
function toggleGuide(btn){
  const b=el("gbody");
  b.classList.toggle("open");
  btn.textContent=b.classList.contains("open")?"▲ Switch Workflow Guide":"▼ Switch Workflow Guide";
}

// ─── Robot arrow on map ────────────────────────────────────────────────────────
function drawRobot(ctx, rx, ry, yawDeg, sz) {
  const a = yawDeg * Math.PI / 180;

  // Pulse ring (small, tight)
  const pt=(Date.now()/900)%1;
  ctx.beginPath(); ctx.arc(rx, ry, sz+2+pt*7, 0, Math.PI*2);
  ctx.strokeStyle=`rgba(239,68,68,${0.6-pt*0.6})`; ctx.lineWidth=1.5; ctx.stroke();

  // Arrow body
  ctx.save();
  ctx.translate(rx, ry);
  ctx.rotate(a);
  ctx.beginPath();
  ctx.moveTo(0, -sz);
  ctx.lineTo(sz*0.62, sz*0.65);
  ctx.lineTo(0, sz*0.22);
  ctx.lineTo(-sz*0.62, sz*0.65);
  ctx.closePath();
  ctx.fillStyle="#ef4444"; ctx.fill();
  ctx.strokeStyle="#fff"; ctx.lineWidth=1.5; ctx.stroke();

  // Front headlight dot
  ctx.beginPath(); ctx.arc(0, -sz*0.52, 2.5, 0, Math.PI*2);
  ctx.fillStyle="#fef3c7"; ctx.fill();

  ctx.restore();

  // Heading label (small badge just ahead of the nose)
  const normYaw=Math.round(((yawDeg%360)+360)%360);
  const lx = rx + Math.sin(a)*(sz+16);
  const ly = ry - Math.cos(a)*(sz+16);
  ctx.fillStyle="rgba(0,0,0,0.55)";
  ctx.beginPath(); ctx.roundRect(lx-22, ly-8, 44, 16, 4); ctx.fill();
  ctx.fillStyle="#e2e8f0";
  ctx.font="bold 10px monospace";
  ctx.textAlign="center"; ctx.textBaseline="middle";
  ctx.fillText(`${normYaw}° ${cardDir(yawDeg)}`, lx, ly);
}

// ─── Compass rose on map ───────────────────────────────────────────────────────
function drawCompassRose(ctx, cx, cy, r) {
  // Dark background circle
  ctx.beginPath(); ctx.arc(cx,cy,r+5,0,Math.PI*2);
  ctx.fillStyle="rgba(13,17,23,0.75)"; ctx.fill();
  ctx.strokeStyle="#21273a"; ctx.lineWidth=1; ctx.stroke();
  // Tick lines
  [0,90,180,270].forEach(deg=>{
    const a=(deg-90)*Math.PI/180;
    ctx.beginPath();
    ctx.moveTo(cx+Math.cos(a)*(r-5), cy+Math.sin(a)*(r-5));
    ctx.lineTo(cx+Math.cos(a)*(r+2), cy+Math.sin(a)*(r+2));
    ctx.strokeStyle=deg===0?"#ef4444":"#334155"; ctx.lineWidth=1.5; ctx.stroke();
  });
  // Cardinal labels
  ["N","E","S","W"].forEach((d,i)=>{
    const a=(i*90-90)*Math.PI/180;
    ctx.fillStyle = d==="N" ? "#ef4444" : "#94a3b8";
    ctx.font=`bold ${d==="N"?13:11}px monospace`;
    ctx.textAlign="center"; ctx.textBaseline="middle";
    ctx.fillText(d, cx+Math.cos(a)*(r-8), cy+Math.sin(a)*(r-8));
  });
  ctx.beginPath(); ctx.arc(cx,cy,3,0,Math.PI*2);
  ctx.fillStyle="#475569"; ctx.fill();
}

// ─── Map ───────────────────────────────────────────────────────────────────────
function drawMap(d) {
  const canvas=el("map"); if(!canvas) return;
  const wrap=canvas.parentElement;
  // Use offsetWidth/Height — reliable even before full layout, avoids 0-size frames
  const W=wrap.offsetWidth||wrap.clientWidth||600;
  const H=wrap.offsetHeight||wrap.clientHeight||400;
  if(canvas.width!==W) canvas.width=W;
  if(canvas.height!==H) canvas.height=H;
  const ctx=canvas.getContext("2d");

  ctx.fillStyle="#0d1117"; ctx.fillRect(0,0,W,H);

  const bnd=d.boundary||[], wps=d.waypoints||[], wpIdx=d.wp_index||0;
  const hasRobot=d.robot_lat!=null;

  if(bnd.length<2 && wps.length===0 && !hasRobot){
    // Set a default _mapView so draw-mode clicks register even on an empty canvas.
    // Odo mode: 20 m square centred at origin. GPS mode: use last fix or a tiny box.
    const _defPad = d.odo_mode ? 10 : 0.0001;
    const _defA   = d.odo_mode ? 0 : 0;
    const _defB   = d.odo_mode ? 0 : 0;
    _mapView = { minA:_defA-_defPad, maxA:_defA+_defPad,
                 minB:_defB-_defPad, maxB:_defB+_defPad, M:36, W, H };
    // Idle grid lines
    ctx.strokeStyle="#151e2e"; ctx.lineWidth=1;
    for(let i=0;i<=12;i++){
      ctx.beginPath(); ctx.moveTo(i*(W/12),0); ctx.lineTo(i*(W/12),H); ctx.stroke();
    }
    for(let i=0;i<=8;i++){
      ctx.beginPath(); ctx.moveTo(0,i*(H/8)); ctx.lineTo(W,i*(H/8)); ctx.stroke();
    }
    if(_drawMode){
      // In draw mode: crosshair prompt instead of idle text
      ctx.fillStyle="rgba(0,0,0,0.65)"; ctx.fillRect(0,0,W,28);
      ctx.fillStyle="#facc15"; ctx.font="bold 11px 'Segoe UI',sans-serif";
      ctx.textAlign="center"; ctx.textBaseline="middle";
      ctx.fillText("✏  DRAW BOUNDARY  —  Click anywhere to place first corner", W/2, 14);
      if(_drawHover){
        ctx.beginPath(); ctx.arc(_drawHover[0],_drawHover[1],7,0,Math.PI*2);
        ctx.fillStyle="rgba(250,204,21,0.75)"; ctx.fill();
        ctx.strokeStyle="#fff"; ctx.lineWidth=1.5; ctx.stroke();
      }
    } else {
      ctx.fillStyle="#7a8faa"; ctx.font="bold 14px 'Segoe UI',sans-serif";
      ctx.textAlign="center"; ctx.textBaseline="middle";
      ctx.fillText("No field defined yet", W/2, H/2-22);
      ctx.font="12px 'Segoe UI',sans-serif"; ctx.fillStyle="#4a5f7a";
      ctx.fillText("▶  Click  ✏ Draw  above, then click the map to outline the field", W/2, H/2+2);
      ctx.fillText("Or: drive to each corner → Web Controls → Mark Corner", W/2, H/2+22);
    }
    return;
  }

  // ── Fixed view — locked to field boundary + waypoints only.
  // Robot position and trail do NOT affect the view extent, so the grid stays
  // completely still while the robot icon moves within it (drone-GCS style).
  const fieldPts=[...bnd,...wps];
  let viewPts;
  if(fieldPts.length>0){
    viewPts=[...fieldPts];
  } else {
    // No field defined yet — fall back to robot + trail so something shows
    viewPts=hasRobot?[[d.robot_lat,d.robot_lon]]:[];
    trail.forEach(p=>viewPts.push([p.a,p.b]));
  }
  if(viewPts.length===1){
    const[a,b]=viewPts[0];
    const pad=d.odo_mode?3.0:0.00008;
    viewPts.push([a-pad,b-pad],[a+pad,b+pad]);
  }
  if(viewPts.length===0) return;

  let minA=Infinity,maxA=-Infinity,minB=Infinity,maxB=-Infinity;
  for(const[a,b] of viewPts){
    minA=Math.min(minA,a); maxA=Math.max(maxA,a);
    minB=Math.min(minB,b); maxB=Math.max(maxB,b);
  }
  const pf=0.18;
  const dA=(maxA-minA)||(d.odo_mode?2:0.0001);
  const dB=(maxB-minB)||(d.odo_mode?2:0.0001);
  minA-=dA*pf; maxA+=dA*pf; minB-=dB*pf; maxB+=dB*pf;

  const M=36;
  function xy(a,b){
    return [
      M+(b-minB)/(maxB-minB)*(W-2*M),
      M+(1-(a-minA)/(maxA-minA))*(H-2*M)
    ];
  }
  // Expose view so click handler can convert canvas px → coord
  _mapView={minA,maxA,minB,maxB,M,W,H};

  // ── Background dot grid ─────────────────────────────────────────────────────
  ctx.fillStyle="#1a2535";
  for(let i=0;i<=10;i++) for(let j=0;j<=8;j++){
    const gx=M+i*(W-2*M)/10, gy=M+j*(H-2*M)/8;
    ctx.beginPath(); ctx.arc(gx,gy,1.2,0,Math.PI*2); ctx.fill();
  }

  // ── Field boundary fill ─────────────────────────────────────────────────────
  if(bnd.length>=3){
    ctx.beginPath();
    const[x0,y0]=xy(bnd[0][0],bnd[0][1]); ctx.moveTo(x0,y0);
    for(let i=1;i<bnd.length;i++){ const[xi,yi]=xy(bnd[i][0],bnd[i][1]); ctx.lineTo(xi,yi); }
    ctx.closePath();
    ctx.fillStyle="rgba(28,42,74,0.55)"; ctx.fill();
    ctx.setLineDash([6,5]); ctx.strokeStyle="#3b4a6a"; ctx.lineWidth=1.5; ctx.stroke();
    ctx.setLineDash([]);
  }

  const sweepCount=Math.floor(wps.length/2);

  // ── Sweep grid base — all row lines drawn light in background ───────────────
  if(sweepCount>=1){
    for(let s=0;s<sweepCount;s++){
      const[x1,y1]=xy(wps[s*2][0],wps[s*2][1]);
      const[x2,y2]=xy(wps[s*2+1][0],wps[s*2+1][1]);
      ctx.beginPath(); ctx.moveTo(x1,y1); ctx.lineTo(x2,y2);
      ctx.setLineDash([3,7]);
      ctx.strokeStyle="rgba(59,100,180,0.22)";
      ctx.lineWidth=3; ctx.stroke();
    }
    ctx.setLineDash([]);
    // Row-to-row transition connectors
    for(let s=0;s<sweepCount-1;s++){
      const ei=s*2+1, si=(s+1)*2;
      if(ei<wps.length && si<wps.length){
        const[ex,ey]=xy(wps[ei][0],wps[ei][1]);
        const[sx2,sy2]=xy(wps[si][0],wps[si][1]);
        ctx.beginPath(); ctx.moveTo(ex,ey); ctx.lineTo(sx2,sy2);
        ctx.setLineDash([3,8]);
        ctx.strokeStyle="rgba(70,90,130,0.45)";
        ctx.lineWidth=1; ctx.stroke();
      }
    }
    ctx.setLineDash([]);
  }

  // ── Colored sweep passes (progress overlay) ─────────────────────────────────
  for(let s=0;s<sweepCount;s++){
    const i1=s*2, i2=s*2+1;
    const[x1,y1]=xy(wps[i1][0],wps[i1][1]);
    const[x2,y2]=xy(wps[i2][0],wps[i2][1]);
    const cur=Math.floor(wpIdx/2);
    let col,lw;
    if(i2<wpIdx){col="#22c55e";lw=5;}
    else if(s===cur){
      const t=(Date.now()/600)%1;
      col=`rgb(250,${Math.round(175+80*Math.sin(t*Math.PI*2))},20)`;lw=6;
    } else{col="#2563eb";lw=2.5;}
    ctx.beginPath(); ctx.moveTo(x1,y1); ctx.lineTo(x2,y2);
    ctx.strokeStyle=col; ctx.lineWidth=lw; ctx.stroke();
    for(const[px,py] of[[x1,y1],[x2,y2]]){
      ctx.beginPath(); ctx.arc(px,py,lw>3?4:3,0,Math.PI*2);
      ctx.fillStyle=col; ctx.fill();
    }
  }

  // ── Row number labels ───────────────────────────────────────────────────────
  if(sweepCount>0 && sweepCount<=30){
    const cur=Math.floor(wpIdx/2);
    for(let s=0;s<sweepCount;s++){
      const[x1,y1]=xy(wps[s*2][0],wps[s*2][1]);
      ctx.font="8px monospace"; ctx.textBaseline="middle";
      ctx.fillStyle = s<cur?"rgba(34,197,94,0.5)":s===cur?"#facc15":"rgba(59,130,246,0.45)";
      ctx.textAlign="right";
      ctx.fillText("R"+(s+1), x1-7, y1);
    }
  }

  // ── Boundary corners (larger + draggable-aware) ─────────────────────────────
  bnd.forEach(([a,b],i)=>{
    // If this corner is being dragged, show it at drag position
    let cx,cy;
    if(_dragCorner && _dragCorner.hasMoved && _dragCorner.idx===i && _dragPos){
      [cx,cy]=_dragPos;
    } else {
      [cx,cy]=xy(a,b);
    }
    const isDragging=_drawMode&&_dragCorner&&_dragCorner.idx===i;
    const isHovered=_drawMode&&!_dragCorner&&_drawHover&&Math.hypot(_drawHover[0]-cx,_drawHover[1]-cy)<18;
    ctx.beginPath(); ctx.arc(cx,cy,isDragging||isHovered?9:7,0,Math.PI*2);
    ctx.fillStyle=isDragging?"#eab308":isHovered?"#475569":"#1e293b"; ctx.fill();
    ctx.strokeStyle=isDragging?"#fff":isHovered?"#94a3b8":"#64748b";
    ctx.lineWidth=isDragging?2.5:1.5; ctx.stroke();
    ctx.fillStyle="#e2e8f0"; ctx.font="bold 10px monospace";
    ctx.textAlign="left"; ctx.textBaseline="alphabetic";
    ctx.fillText(i+1, cx+11, cy-3);
    if(_drawMode) {
      // Small drag-handle cross
      ctx.strokeStyle="rgba(255,255,255,0.4)"; ctx.lineWidth=1;
      ctx.beginPath(); ctx.moveTo(cx-4,cy); ctx.lineTo(cx+4,cy);
      ctx.moveTo(cx,cy-4); ctx.lineTo(cx,cy+4); ctx.stroke();
    }
  });

  // ── Target WP ring ──────────────────────────────────────────────────────────
  if(wpIdx<wps.length && d.mission_mode==="RUNNING"){
    const[tx,ty]=xy(wps[wpIdx][0],wps[wpIdx][1]);
    ctx.beginPath(); ctx.arc(tx,ty,13,0,Math.PI*2);
    ctx.strokeStyle="#facc15"; ctx.lineWidth=2;
    ctx.setLineDash([3,3]); ctx.stroke(); ctx.setLineDash([]);
  }

  // ── Start path indicator (pulsing ring + direction arrow) ───────────────────
  if(wps.length>=2 && d.mission_mode!=="RUNNING"){
    const[sx,sy]=xy(wps[0][0],wps[0][1]);
    const[sx2,sy2]=xy(wps[1][0],wps[1][1]);

    // Pulsing ring animation
    const pt=(Date.now()/750)%1;
    ctx.beginPath(); ctx.arc(sx,sy, 9+pt*20, 0, Math.PI*2);
    ctx.strokeStyle=`rgba(34,197,94,${0.85-pt*0.85})`; ctx.lineWidth=2.5; ctx.stroke();

    // Solid inner dot
    ctx.beginPath(); ctx.arc(sx,sy,5,0,Math.PI*2);
    ctx.fillStyle="#22c55e"; ctx.fill();
    ctx.strokeStyle="#fff"; ctx.lineWidth=1.5; ctx.stroke();

    // Direction arrow toward first waypoint end
    const ang=Math.atan2(sy2-sy, sx2-sx);
    const aLen=28;
    const ax=sx+Math.cos(ang)*aLen, ay=sy+Math.sin(ang)*aLen;
    ctx.beginPath();
    ctx.moveTo(sx+Math.cos(ang)*7, sy+Math.sin(ang)*7);
    ctx.lineTo(ax,ay);
    ctx.strokeStyle="#22c55e"; ctx.lineWidth=2.5; ctx.stroke();
    // Arrowhead
    const hL=9, hA=Math.PI/5;
    ctx.beginPath();
    ctx.moveTo(ax,ay);
    ctx.lineTo(ax-Math.cos(ang-hA)*hL, ay-Math.sin(ang-hA)*hL);
    ctx.moveTo(ax,ay);
    ctx.lineTo(ax-Math.cos(ang+hA)*hL, ay-Math.sin(ang+hA)*hL);
    ctx.strokeStyle="#22c55e"; ctx.lineWidth=2.5; ctx.stroke();

    // "START" label
    ctx.fillStyle="#22c55e"; ctx.font="bold 10px monospace";
    ctx.textAlign="center"; ctx.textBaseline="bottom";
    ctx.fillText("START", sx, sy-13);
  }

  // ── Robot trail ─────────────────────────────────────────────────────────────
  if(trail.length>1){
    ctx.beginPath();
    const[ta,tb]=[trail[0].a,trail[0].b];
    const[tx0,ty0]=xy(ta,tb); ctx.moveTo(tx0,ty0);
    for(let i=1;i<trail.length;i++){
      const[tx,ty]=xy(trail[i].a,trail[i].b); ctx.lineTo(tx,ty);
    }
    ctx.strokeStyle="rgba(6,182,212,0.35)"; ctx.lineWidth=2; ctx.stroke();
    ctx.beginPath(); ctx.arc(tx0,ty0,3,0,Math.PI*2);
    ctx.fillStyle="rgba(6,182,212,0.25)"; ctx.fill();
  }

  // ── Robot ───────────────────────────────────────────────────────────────────
  if(hasRobot){
    const[rx,ry]=xy(d.robot_lat,d.robot_lon);
    if(d.odo_mode){
      ctx.fillStyle="#64748b"; ctx.font="9px monospace";
      ctx.textAlign="left"; ctx.textBaseline="alphabetic";
      ctx.fillText(`x=${d.odo_x.toFixed(2)}m  y=${d.odo_y.toFixed(2)}m`, rx+22, ry+14);
    }
    drawRobot(ctx, rx, ry, d.yaw_deg||0, 14);
  }

  // ── Scale bar ───────────────────────────────────────────────────────────────
  {
    let px1m;
    if(d.odo_mode){
      px1m=(W-2*M)/Math.max(dB,0.1);
    } else {
      const ppd=(W-2*M)/Math.max(dB,1e-9);
      const mpd=111320*Math.cos(((minA+maxA)/2)*Math.PI/180);
      px1m=ppd/mpd;
    }
    const barM=d.odo_mode?1:5;
    const pxBar=Math.max(12,Math.round(px1m*barM));
    const barY=H-M/2+2;
    // End ticks
    ctx.beginPath(); ctx.moveTo(M,barY-4); ctx.lineTo(M,barY+4); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(M+pxBar,barY-4); ctx.lineTo(M+pxBar,barY+4); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(M,barY); ctx.lineTo(M+pxBar,barY);
    ctx.strokeStyle="#64748b"; ctx.lineWidth=2; ctx.stroke();
    ctx.fillStyle="#94a3b8"; ctx.font="bold 10px monospace";
    ctx.textAlign="center"; ctx.textBaseline="alphabetic";
    ctx.fillText(`${barM} m`, M+pxBar/2, barY-7);
  }

  // ── Draw-mode overlay ───────────────────────────────────────────────────────
  if(_drawMode){
    // Ghost line from last boundary corner to mouse
    if(bnd.length>0 && _drawHover){
      const[lx,ly]=xy(bnd[bnd.length-1][0],bnd[bnd.length-1][1]);
      ctx.beginPath(); ctx.moveTo(lx,ly); ctx.lineTo(_drawHover[0],_drawHover[1]);
      ctx.setLineDash([5,4]); ctx.strokeStyle="rgba(250,204,21,0.65)";
      ctx.lineWidth=1.5; ctx.stroke(); ctx.setLineDash([]);
    }
    // Ghost closing line (polygon preview)
    if(bnd.length>=2 && _drawHover){
      const[fx,fy]=xy(bnd[0][0],bnd[0][1]);
      ctx.beginPath(); ctx.moveTo(_drawHover[0],_drawHover[1]); ctx.lineTo(fx,fy);
      ctx.setLineDash([2,6]); ctx.strokeStyle="rgba(250,204,21,0.25)";
      ctx.lineWidth=1; ctx.stroke(); ctx.setLineDash([]);
    }
    // Ghost dot at mouse cursor
    if(_drawHover){
      ctx.beginPath(); ctx.arc(_drawHover[0],_drawHover[1],7,0,Math.PI*2);
      ctx.fillStyle="rgba(250,204,21,0.75)"; ctx.fill();
      ctx.strokeStyle="#fff"; ctx.lineWidth=1.5; ctx.stroke();
    }
    // Instruction banner
    ctx.fillStyle="rgba(0,0,0,0.65)";
    ctx.fillRect(0,0,W,28);
    ctx.fillStyle="#facc15"; ctx.font="bold 11px 'Segoe UI',sans-serif";
    ctx.textAlign="center"; ctx.textBaseline="middle";
    const cnt=bnd.length;
    const hint=cnt===0?"Click to place first corner"
      :cnt<3?`${cnt} corner${cnt>1?"s":""} — keep clicking (need 3+)`
      :`${cnt} corners — double-click or ✓ Done to finish`;
    ctx.fillText(`✏  DRAW BOUNDARY  —  ${hint}  ·  Right-click = undo`, W/2, 14);
  }

  // ── Compass rose ────────────────────────────────────────────────────────────
  drawCompassRose(ctx, W-M-18, M+18, 20);
}

// ─── Animation loop ────────────────────────────────────────────────────────────
(function loop(){ if(D) drawMap(D); requestAnimationFrame(loop); })();

// ─── Toast notification ────────────────────────────────────────────────────────
function toast(msg, type="info"){
  const d=document.createElement("div");
  const c=type==="ok"?"#22c55e":type==="err"?"#ef4444":"#06b6d4";
  d.style.cssText="position:fixed;bottom:52px;right:16px;background:#1e293b;color:#e2e8f0;"
    +"padding:8px 14px;border-radius:6px;font-size:.75rem;z-index:9999;"
    +"border-left:3px solid "+c+";box-shadow:0 2px 8px rgba(0,0,0,.6);"
    +"animation:fadein .2s ease";
  d.textContent=msg;
  document.body.appendChild(d);
  setTimeout(()=>d.remove(),3000);
}

// ─── Web controls (switch alternatives) ───────────────────────────────────────
async function webCmd(action){
  const extra={};
  if(action==="plan"){
    extra.row_spacing=parseFloat(el("row-spacing")?.value||0.8);
    const sa=el("sweep-angle");
    extra.angle=sa?sa.value:"auto";
  }
  try{
    const r=await fetch("/api/control",{
      method:"POST",
      headers:{"Content-Type":"application/json"},
      body:JSON.stringify({cmd:action,...extra})
    });
    const j=await r.json();
    if(!j.ok) toast(j.msg||"Command failed","err");
    else toast(j.msg||action+" sent","ok");
  }catch(e){ toast("Network error: "+e,"err"); }
}

// ─── Virtual joystick ──────────────────────────────────────────────────────────
let _joyActive=false, _joyLastSend=0;
const _keys=new Set();

function joyDown(e){
  _joyActive=true;
  el("joy-knob").classList.add("joy-active");
  _joyMove(e); e.preventDefault();
}
function _joyMove(e){
  if(!_joyActive) return;
  e.preventDefault();
  const pad=el("joy-pad"), rect=pad.getBoundingClientRect();
  const cx=rect.left+rect.width/2, cy=rect.top+rect.height/2;
  const pt=e.touches?e.touches[0]:e;
  const dx=pt.clientX-cx, dy=pt.clientY-cy;
  const maxR=rect.width/2-20;
  const dist=Math.hypot(dx,dy);
  const sc=dist>maxR?maxR/dist:1;
  const kx=dx*sc, ky=dy*sc;
  el("joy-knob").style.transform=`translate(calc(-50% + ${kx}px),calc(-50% + ${ky}px))`;
  const thr=Math.round(-ky/maxR*100), str=Math.round(kx/maxR*100);
  _joyDisplay(thr,str); _joySend(thr,str);
}
function _joyUp(){
  if(!_joyActive) return;
  _joyActive=false;
  el("joy-knob").classList.remove("joy-active");
  el("joy-knob").style.transform="translate(-50%,-50%)";
  _joyDisplay(0,0); _joySend(0,0);
}
function _joyDisplay(thr,str){
  el("joy-thr").textContent=(thr>=0?"+":"")+thr+"%";
  el("joy-str").textContent=str===0?"CTR":(str>0?"R+":"L")+Math.abs(str)+"%";
}
function _joySend(throttle,steer){
  const now=Date.now(); if(now-_joyLastSend<80) return;
  _joyLastSend=now;
  fetch("/api/control",{method:"POST",headers:{"Content-Type":"application/json"},
    body:JSON.stringify({cmd:"manual_drive",throttle,steer})}).catch(()=>{});
}
// Global mouse/touch listeners for joystick
document.addEventListener("mousemove", e=>_joyMove(e));
document.addEventListener("mouseup",   ()=>_joyUp());
document.addEventListener("touchmove", e=>_joyMove(e),{passive:false});
document.addEventListener("touchend",  ()=>_joyUp());
document.addEventListener("touchcancel",()=>_joyUp());

// Keyboard drive
document.addEventListener("keydown",e=>{
  if([" ","ArrowUp","ArrowDown","ArrowLeft","ArrowRight"].includes(e.key)) e.preventDefault();
  _keys.add(e.key); _keyJoy();
});
document.addEventListener("keyup",e=>{ _keys.delete(e.key); _keyJoy(); });
function _keyJoy(){
  if(_joyActive) return; // joystick takes priority
  let thr=0,str=0;
  if(_keys.has(" ")){ thr=0; str=0; }
  else{
    if(_keys.has("ArrowUp"))    thr=65;
    if(_keys.has("ArrowDown"))  thr=-65;
    if(_keys.has("ArrowRight")) str=55;
    if(_keys.has("ArrowLeft"))  str=-55;
  }
  _joyDisplay(thr,str); _joySend(thr,str);
}

// ─── Map save ──────────────────────────────────────────────────────────────────
function saveMapPNG(){
  const canvas=el("map");
  const ts=new Date().toISOString().slice(0,19).replace(/[T:]/g,"-");
  const a=document.createElement("a");
  a.download="solarbot-map-"+ts+".png";
  a.href=canvas.toDataURL("image/png");
  a.click();
  toast("Map saved as PNG","ok");
}
async function saveField(){
  if(!D){ toast("No data yet","err"); return; }
  try{
    const r=await fetch("/api/save_field",{method:"POST",
      headers:{"Content-Type":"application/json"},
      body:JSON.stringify({boundary:D.boundary||[],waypoints:D.waypoints||[]})});
    const j=await r.json();
    toast(j.ok?"Saved: "+j.filename:"Save failed: "+(j.msg||"?"),j.ok?"ok":"err");
  }catch(e){ toast("Network error","err"); }
}

// ─── Mission history panel ────────────────────────────────────────────────────
async function showMissions(){
  const panel=el("ms-panel");
  panel.style.display="block";
  const list=el("ms-list");
  list.innerHTML='<span style="color:var(--dim);font-size:.76rem">Loading...</span>';
  try{
    const r=await fetch("/api/missions");
    const j=await r.json();
    if(!j.ok||!j.missions.length){
      list.innerHTML='<span style="color:var(--dim);font-size:.76rem">No saved missions found</span>';
      return;
    }
    list.innerHTML="";
    for(const m of j.missions){
      const div=document.createElement("div");
      div.className="ms-item";
      div.innerHTML=`<b>${m.name}</b><span>${m.saved_at} &nbsp;&#183;&nbsp; ${m.corners} corners &nbsp;&#183;&nbsp; ${m.waypoints} wps &nbsp;&#183;&nbsp; ${m.mode}</span>`;
      div.onclick=()=>loadMission(m.name);
      list.appendChild(div);
    }
  }catch(e){ list.innerHTML='<span style="color:#f87171;font-size:.76rem">Error fetching missions</span>'; }
}
async function loadMission(name){
  try{
    const r=await fetch("/api/load_mission",{method:"POST",
      headers:{"Content-Type":"application/json"},
      body:JSON.stringify({name})});
    const j=await r.json();
    toast(j.ok?"Loaded: "+name:"Load failed: "+(j.msg||"?"),j.ok?"ok":"err");
    if(j.ok) el("ms-panel").style.display="none";
  }catch(e){ toast("Network error","err"); }
}

// ─── Coordinate helpers ────────────────────────────────────────────────────────
function _canvasToCoord(px, py){
  if(!_mapView) return null;
  const {minA,maxA,minB,maxB,M,W,H}=_mapView;
  return [minA+(1-(py-M)/(H-2*M))*(maxA-minA), minB+(px-M)/(W-2*M)*(maxB-minB)];
}
function _coordToCanvas(a, b){
  if(!_mapView) return [0,0];
  const {minA,maxA,minB,maxB,M,W,H}=_mapView;
  return [M+(b-minB)/(maxB-minB)*(W-2*M), M+(1-(a-minA)/(maxA-minA))*(H-2*M)];
}

// ─── Survey panel helpers ──────────────────────────────────────────────────────
function _svPanelVisible(show){
  const p=el("sv-panel"); if(p) p.style.display=show?"block":"none";
}
function _svPanelUpdate(){
  const bnd=D?.boundary||[], wps=D?.waypoints||[];
  el("sv-corners").textContent=bnd.length;
  const passes=Math.floor(wps.length/2);
  el("sv-passes").textContent=passes||"—";
  if(wps.length>=2){
    // Simple total path distance from waypoints
    let dist=0;
    const R=6371000;
    for(let i=0;i<wps.length-1;i++){
      const [a1,b1]=wps[i],[a2,b2]=wps[i+1];
      if(D?.odo_mode){
        dist+=Math.hypot(b2-b1,a2-a1);
      } else {
        const dlat=(a2-a1)*Math.PI/180, dlon=(b2-b1)*Math.PI/180;
        const s=Math.sin(dlat/2)**2+Math.cos(a1*Math.PI/180)*Math.cos(a2*Math.PI/180)*Math.sin(dlon/2)**2;
        dist+=R*2*Math.asin(Math.sqrt(s));
      }
    }
    el("sv-dist").textContent=dist>1000?(dist/1000).toFixed(2)+"km":dist.toFixed(1)+"m";
    const secs=dist/0.3;
    el("sv-time").textContent=secs>60?Math.floor(secs/60)+"m "+Math.round(secs%60)+"s":Math.round(secs)+"s";
    el("sv-start-btn").style.display="block";
  } else {
    el("sv-dist").textContent="—"; el("sv-time").textContent="—";
    el("sv-start-btn").style.display="none";
  }
}

// ─── Draw mode toggle ──────────────────────────────────────────────────────────
function toggleDrawMode(){
  _drawMode=!_drawMode;
  const btn=el("draw-btn"), canvas=el("map");
  if(_drawMode){
    btn.style.cssText="background:#eab308;color:#000;border-color:#eab308";
    btn.textContent="✓ Done";
    canvas.classList.add("draw-active");
    _svPanelVisible(true);
    toast("Draw mode ON — click map to place corners  ·  drag corners to adjust  ·  right-click = undo  ·  double-click = finish","ok");
  } else {
    btn.style.cssText=""; btn.textContent="✏ Draw";
    canvas.classList.remove("draw-active");
    _drawHover=null; _dragCorner=null; _dragPos=null;
    // Keep survey panel visible if boundary exists
    _svPanelVisible((D?.boundary||[]).length>0||(D?.waypoints||[]).length>0);
  }
}

async function clearBoundary(){
  try{
    const r=await fetch("/api/clear_boundary",{method:"POST"});
    const j=await r.json();
    if(j.ok){ toast("Field cleared","ok"); _svPanelVisible(false); }
    else toast("Clear failed","err");
  }catch(e){ toast("Network error","err"); }
}

async function surveyGenerate(){
  const spacing=parseFloat(el("sv-spacing")?.value||0.8);
  const angle=el("sv-angle")?.value||"auto";
  const overshoot=parseFloat(el("sv-overshoot")?.value||1.0);
  // keep sidebar row-spacing slider in sync
  const rsEl=el("row-spacing"); if(rsEl){ rsEl.value=spacing; el("row-spacing-v").textContent=spacing.toFixed(1)+"m"; }
  try{
    const r=await fetch("/api/control",{method:"POST",
      headers:{"Content-Type":"application/json"},
      body:JSON.stringify({cmd:"plan",row_spacing:spacing,angle,overshoot})});
    const j=await r.json();
    if(j.ok) toast("Path generated — "+j.msg,"ok");
    else      toast("Generate failed: "+(j.msg||"?"),"err");
  }catch(e){ toast("Network error","err"); }
}

// ─── Canvas draw/drag event listeners ─────────────────────────────────────────
(function _setupDrawCanvas(){
  const canvas=el("map");
  if(!canvas){ console.warn("Canvas #map not found"); return; }

  function _px(e){
    const rect=canvas.getBoundingClientRect();
    const t=e.touches?e.touches[0]:e;
    return [(t.clientX-rect.left)*(canvas.width/rect.width),
            (t.clientY-rect.top) *(canvas.height/rect.height)];
  }
  function _nearCorner(px,py){
    const bnd=D?.boundary||[];
    for(let i=0;i<bnd.length;i++){
      const[cx,cy]=_coordToCanvas(bnd[i][0],bnd[i][1]);
      if(Math.hypot(px-cx,py-cy)<18) return i;
    }
    return -1;
  }

  // mousedown: detect corner drag start
  canvas.addEventListener("mousedown", e=>{
    if(!_drawMode) return;
    _didDrag=false;
    const[px,py]=_px(e);
    const idx=_nearCorner(px,py);
    if(idx>=0){ _dragCorner={idx,hasMoved:false,origPx:[px,py]}; e.preventDefault(); }
  });

  // mouseup: finish drag or let click handler add corner
  canvas.addEventListener("mouseup", async e=>{
    if(!_drawMode) return;
    if(_dragCorner && _dragCorner.hasMoved && _dragPos){
      _didDrag=true; // suppress the click event that fires right after
      const coord=_canvasToCoord(_dragPos[0],_dragPos[1]);
      if(coord){
        try{
          await fetch("/api/update_corner",{method:"POST",
            headers:{"Content-Type":"application/json"},
            body:JSON.stringify({index:_dragCorner.idx,a:coord[0],b:coord[1],odo_mode:D?.odo_mode||false})});
        }catch(ex){}
      }
    }
    _dragCorner=null; _dragPos=null;
  });

  // click: add new corner (only if not a drag)
  canvas.addEventListener("click", async e=>{
    if(!_drawMode) return;
    if(_didDrag){ _didDrag=false; return; } // suppress click after corner drag
    const[px,py]=_px(e);
    if(_nearCorner(px,py)>=0) return; // clicked existing corner, not adding
    const coord=_canvasToCoord(px,py); if(!coord) return;
    try{
      const r=await fetch("/api/add_corner",{method:"POST",
        headers:{"Content-Type":"application/json"},
        body:JSON.stringify({a:coord[0],b:coord[1],odo_mode:D?.odo_mode||false})});
      const j=await r.json();
      if(j.ok){ toast(`Corner ${j.count} placed`,"ok"); _svPanelVisible(true); }
      else toast("Add failed","err");
    }catch(ex){ toast("Network error","err"); }
  });

  // double-click: finish polygon
  canvas.addEventListener("dblclick", e=>{
    if(!_drawMode) return; e.preventDefault();
    const cnt=(D?.boundary||[]).length;
    if(cnt>=3){ toggleDrawMode(); toast(`${cnt}-corner field ready — click Generate Path`,"ok"); }
    else        toast(`Need ≥3 corners (have ${cnt})`,"err");
  });

  // right-click: undo
  canvas.addEventListener("contextmenu", async e=>{
    e.preventDefault(); if(!_drawMode) return;
    try{
      const r=await fetch("/api/undo_corner",{method:"POST"});
      const j=await r.json();
      toast(j.count>0?`Undid — ${j.count} left`:"All corners removed","ok");
    }catch(ex){ toast("Network error","err"); }
  });

  // mousemove: ghost dot + corner drag
  canvas.addEventListener("mousemove", e=>{
    if(!_drawMode){ _drawHover=null; return; }
    const[px,py]=_px(e);
    _drawHover=[px,py];
    if(_dragCorner){
      const moved=Math.hypot(px-_dragCorner.origPx[0],py-_dragCorner.origPx[1]);
      if(moved>5) _dragCorner.hasMoved=true;
      if(_dragCorner.hasMoved) _dragPos=[px,py];
    }
  });
  canvas.addEventListener("mouseleave",()=>{ _drawHover=null; if(!_dragCorner) _dragPos=null; });
})();

// ─── Legacy stop/pause buttons ─────────────────────────────────────────────────
async function cmd(action){
  try{
    const r=await fetch("/api/control",{
      method:"POST",
      headers:{"Content-Type":"application/json"},
      body:JSON.stringify({cmd:action})
    });
    const j=await r.json();
    if(!j.ok) toast(j.msg||"Command failed","err");
    else toast(j.msg||action+" sent","ok");
  }catch(e){ toast("Network error: "+e,"err"); }
}
</script>
</body>
</html>
"""

# ── Flask app ─────────────────────────────────────────────────────────────────

def _run_flask(host, port):
    try:
        from flask import Flask, Response, request
    except ImportError:
        print("[WEB] Flask not found — run:  pip3 install flask")
        return

    import logging
    logging.getLogger("werkzeug").setLevel(logging.ERROR)

    app = Flask(__name__)

    @app.route("/")
    def index():
        return _HTML, 200, {"Content-Type": "text/html"}

    @app.route("/stream")
    def stream():
        def gen():
            while not _stop_event.is_set():
                try:
                    payload = json.dumps(_snapshot())
                    yield f"data: {payload}\n\n"
                except Exception as exc:
                    yield f"data: {json.dumps({'error': str(exc)})}\n\n"
                time.sleep(0.2)
        return Response(gen(), mimetype="text/event-stream",
                        headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"})

    @app.route("/api/state")
    def api_state():
        from flask import jsonify as fj
        return fj(_snapshot())

    @app.route("/api/control", methods=["POST"])
    def api_control():
        from flask import jsonify as fj
        data = request.get_json(silent=True) or {}
        action = data.get("cmd", "")

        if action == "stop":
            _state_ref["web_stop"] = True
            return fj({"ok": True, "msg": "Emergency stop sent"})
        elif action == "pause":
            _state_ref["web_pause"] = True
            return fj({"ok": True, "msg": "Mission paused"})
        elif action == "resume":
            _state_ref["web_pause"] = False
            _state_ref["web_resume"] = True
            return fj({"ok": True, "msg": "Mission resumed"})
        elif action == "arm":
            _state_ref["web_arm"] = True
            return fj({"ok": True, "msg": "ARM sent — motors will engage"})
        elif action == "disarm":
            _state_ref["web_disarm"] = True
            return fj({"ok": True, "msg": "DISARM sent — motors locked"})
        elif action == "mark_corner":
            _state_ref["web_mark_corner"] = True
            return fj({"ok": True, "msg": "Corner marked"})
        elif action == "plan":
            spacing   = float(data.get("row_spacing", 0.8))
            angle     = data.get("angle", "auto")
            overshoot = float(data.get("overshoot", 1.0))
            _state_ref["web_plan"]         = True
            _state_ref["web_row_spacing"]  = max(0.1, min(10.0, spacing))
            _state_ref["web_angle"]        = angle
            _state_ref["web_overshoot"]    = max(0.0, min(10.0, overshoot))
            return fj({"ok": True, "msg": f"Plan/Start sent  (spacing={spacing:.1f}m, angle={angle}, overshoot={overshoot:.1f}m)"})
        elif action == "abort":
            _state_ref["web_abort"] = True
            return fj({"ok": True, "msg": "Abort + Reset sent"})
        elif action == "manual_drive":
            thr = max(-100, min(100, int(data.get("throttle", 0))))
            str_ = max(-100, min(100, int(data.get("steer",    0))))
            _state_ref["web_throttle"] = thr
            _state_ref["web_steer"]    = str_
            _state_ref["web_manual_t"] = time.time()
            _state_ref["web_manual"]   = True
            return fj({"ok": True})
        elif action == "manual_stop":
            _state_ref["web_manual"]   = False
            _state_ref["web_throttle"] = 0
            _state_ref["web_steer"]    = 0
            return fj({"ok": True})
        else:
            return fj({"ok": False, "msg": f"Unknown command: {action}"})

    @app.route("/api/save_field", methods=["POST"])
    def api_save_field():
        from datetime import datetime
        from pathlib import Path

        from flask import jsonify as fj
        data = request.get_json(silent=True) or {}
        boundary  = data.get("boundary",  [])
        waypoints = data.get("waypoints", [])
        ts   = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = Path(__file__).parent / "paths" / f"field_{ts}.json"
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            with open(path, "w") as f:
                json.dump({"boundary": boundary, "waypoints": waypoints,
                           "saved": ts, "corners": len(boundary),
                           "waypoints_n": len(waypoints)}, f, indent=2)
            return fj({"ok": True, "filename": path.name})
        except Exception as exc:
            return fj({"ok": False, "msg": str(exc)})

    @app.route("/api/add_corner", methods=["POST"])
    def api_add_corner():
        from flask import jsonify as fj
        data   = request.get_json(silent=True) or {}
        a      = float(data.get("a", 0))
        b      = float(data.get("b", 0))
        odo    = bool(data.get("odo_mode", False))
        # Transition IDLE → RECORDING automatically
        if _state_ref and _state_ref.get("mission_mode") == "IDLE":
            _state_ref["mission_mode"] = "RECORDING"
        if odo:
            # Canvas packs odo as (a=y_m, b=x_m); BoundaryRecorder.add wants (x, y)
            n = _boundary_ref.add(b, a, odo_mode=True)
        else:
            n = _boundary_ref.add(a, b, odo_mode=False)
        return fj({"ok": True, "count": n})

    @app.route("/api/undo_corner", methods=["POST"])
    def api_undo_corner():
        from flask import jsonify as fj
        with _boundary_ref._lock:
            if _boundary_ref._corners:
                _boundary_ref._corners.pop()
        count = _boundary_ref.count
        if count == 0 and _state_ref:
            _state_ref["mission_mode"] = "IDLE"
        return fj({"ok": True, "count": count})

    @app.route("/api/clear_boundary", methods=["POST"])
    def api_clear_boundary():
        from flask import jsonify as fj
        _boundary_ref.reset()
        if _state_ref:
            _state_ref["mission_mode"] = "IDLE"
            _state_ref["wp_index"]     = 0
        return fj({"ok": True})

    @app.route("/api/update_corner", methods=["POST"])
    def api_update_corner():
        from flask import jsonify as fj
        data  = request.get_json(silent=True) or {}
        idx   = int(data.get("index", -1))
        a     = float(data.get("a", 0))
        b     = float(data.get("b", 0))
        odo   = bool(data.get("odo_mode", False))
        with _boundary_ref._lock:
            if 0 <= idx < len(_boundary_ref._corners):
                # odo canvas sends (a=y_m, b=x_m); store as (x, y)
                _boundary_ref._corners[idx] = (b, a) if odo else (a, b)
                return fj({"ok": True, "index": idx})
        return fj({"ok": False, "msg": "index out of range"})

    @app.route("/api/missions", methods=["GET"])
    def api_missions():
        import json as _json
        from pathlib import Path as _P

        from flask import jsonify as fj
        path_dir = _P("paths")
        files = sorted(path_dir.glob("mission_*.json"), reverse=True)
        result = []
        for f in files:
            try:
                d   = _json.loads(f.read_text())
                bc  = len(d.get("boundary",  []))
                wc  = len(d.get("waypoints", []))
                mode= d.get("mode", "gps")
                ts  = d.get("saved_at", "")[:19].replace("T", " ")
                result.append({"name": f.name, "saved_at": ts,
                                "corners": bc, "waypoints": wc, "mode": mode})
            except Exception:
                pass
        return fj({"ok": True, "missions": result})

    @app.route("/api/load_mission", methods=["POST"])
    def api_load_mission():
        from pathlib import Path as _P

        from flask import jsonify as fj
        data = request.get_json(silent=True) or {}
        name = data.get("name", "")
        if not name:
            return fj({"ok": False, "msg": "no filename given"})
        path = _P("paths") / name
        if not path.exists():
            return fj({"ok": False, "msg": "file not found"})
        _state_ref["web_load_mission"] = str(path)
        return fj({"ok": True, "msg": f"Loading {name} ..."})

    print(f"[WEB] Dashboard → http://0.0.0.0:{port}  (browse from any device on WiFi)")
    app.run(host=host, port=port, threaded=True, use_reloader=False)


def start(state, boundary, mission_fn, stop_event, host="0.0.0.0", port=8080):
    """
    Start the web dashboard in a background daemon thread.

    state       : _state dict from rc_drive.py
    boundary    : BoundaryRecorder instance
    mission_fn  : callable returning current MissionRunner (or None)
    stop_event  : threading.Event — server exits when set
    """
    global _state_ref, _boundary_ref, _mission_fn, _stop_event, _web_port
    _state_ref    = state
    _boundary_ref = boundary
    _mission_fn   = mission_fn
    _stop_event   = stop_event
    _web_port     = port

    t = threading.Thread(target=_run_flask, args=(host, port), daemon=True)
    t.start()
    return t
