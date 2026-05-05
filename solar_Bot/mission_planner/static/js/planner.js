/* ═══════════════════════════════════════════════════════════════════════════
   Solar Panel Cleaning Robot — Mission Planner  (planner.js)
   Leaflet map + real-time GPS + boundary drawing + path generation
   ═══════════════════════════════════════════════════════════════════════════ */

"use strict";

// ─── Leaflet map init ────────────────────────────────────────────────────────

const map = L.map("map", {
  center: [20.5937, 78.9629],   // India centre — overridden when GPS arrives
  zoom: 18,
  zoomControl: true,
});

const tileOSM = L.tileLayer(
  "https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png",
  { attribution: "© OpenStreetMap", maxZoom: 22 }
);

const tileSat = L.tileLayer(
  "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
  { attribution: "© Esri World Imagery", maxZoom: 22 }
);

tileOSM.addTo(map);
let usingSatellite = false;

// ─── Map layers ───────────────────────────────────────────────────────────────

let gpsMarker        = null;
let gpsCircle        = null;
let boundaryLayer    = null;
let boundaryMarkers  = [];
let pathLayer        = null;
let wpMarkers        = [];
let currentWpMarker  = null;
let homeMarker       = null;

// ─── App state ────────────────────────────────────────────────────────────────

const state = {
  gpsFixed:      false,
  gpsLocked:     false,
  boundaryMode:  false,
  corners:       [],            // [{lat, lon}, ...]
  waypoints:     [],            // [{seq, lat, lon}, ...]
  missionStatus: "idle",
  currentWp:     0,
  manualSpeed:   50,
  centeredOnGps: false,
};

// ─── Socket.IO ────────────────────────────────────────────────────────────────

const socket = io();

socket.on("connect",        ()  => toast("Connected to server", "success"));
socket.on("disconnect",     ()  => toast("Disconnected from server", "error"));
socket.on("mission_loaded", (d) => onMissionLoaded(d));
socket.on("gps",            (d) => onGPS(d));
socket.on("robot",          (d) => onRobotStatus(d));
socket.on("nav_telemetry",  (d) => onNavTelemetry(d));
socket.on("mission_event",  (d) => onMissionEvent(d));

// ─── GPS handler ──────────────────────────────────────────────────────────────

function onGPS(d) {
  state.gpsLocked = d.has_fix;

  // Status panel
  setEl("gps-lat",    d.lat  ? d.lat.toFixed(7)  : "--");
  setEl("gps-lon",    d.lon  ? d.lon.toFixed(7)  : "--");
  setEl("gps-sats",   d.satellites);
  setEl("gps-hdop",   d.hdop.toFixed(1));
  setEl("gps-acc",    d.accuracy_m.toFixed(1) + " m");
  setEl("gps-speed",  (d.speed_ms * 3.6).toFixed(1) + " km/h");
  setEl("gps-alt",    d.alt_m.toFixed(1) + " m");

  const dotEl = document.getElementById("gps-dot");
  if (!dotEl) return;
  dotEl.className = "dot " + (d.fix_quality >= 4 ? "dot-green"
                            : d.fix_quality >= 1 ? "dot-yellow"
                            : "dot-red");
  setEl("gps-fix-text", d.fix_quality === 4 ? "RTK Fixed"
                       : d.fix_quality === 5 ? "RTK Float"
                       : d.fix_quality >= 1  ? "GPS Fix"
                       : "No Fix");

  if (!d.has_fix) return;

  const latlng = L.latLng(d.lat, d.lon);

  // GPS accuracy ring
  if (!gpsCircle) {
    gpsCircle = L.circle(latlng, {
      radius:      d.accuracy_m,
      color:       "#58a6ff",
      weight:      1,
      fillOpacity: 0.08,
      interactive: false,
    }).addTo(map);
  } else {
    gpsCircle.setLatLng(latlng).setRadius(d.accuracy_m);
  }

  // Robot position marker
  if (!gpsMarker) {
    gpsMarker = L.circleMarker(latlng, {
      radius:      8,
      color:       "#fff",
      weight:      2,
      fillColor:   "#58a6ff",
      fillOpacity: 1,
    }).addTo(map).bindPopup("Robot position");
  } else {
    gpsMarker.setLatLng(latlng);
  }

  // Auto-centre map on first fix
  if (!state.centeredOnGps) {
    map.setView(latlng, 20);
    state.centeredOnGps = true;
  }
}

// ─── Robot telemetry handler ──────────────────────────────────────────────────

function onRobotStatus(d) {
  const dotEl = document.getElementById("robot-dot");
  if (dotEl) dotEl.className = "dot " + (d.connected ? "dot-green" : "dot-red");

  setEl("robot-conn",    d.connected ? "Connected" : "Disconnected");
  setEl("robot-heading", d.heading.toFixed(1) + "°");
  setEl("robot-speed",   (d.speed_cms / 100).toFixed(2) + " m/s");
  setEl("robot-bat-txt", d.battery_pct + "%  (" + d.battery_mv + " mV)");

  const fill = document.getElementById("robot-bat-fill");
  if (fill) {
    fill.style.width = d.battery_pct + "%";
    fill.className   = "battery-fill " +
      (d.battery_pct > 60 ? "battery-high"
     : d.battery_pct > 25 ? "battery-mid"
     : "battery-low");
  }

  // ── IMU display ───────────────────────────────────────────────────────────
  const roll  = d.roll  ?? 0;
  const pitch = d.pitch ?? 0;

  const rollColor  = Math.abs(roll)  > 20 ? "red" : Math.abs(roll)  > 10 ? "yellow" : "green";
  const pitchColor = Math.abs(pitch) > 20 ? "red" : Math.abs(pitch) > 10 ? "yellow" : "green";

  const rollEl  = document.getElementById("imu-roll");
  const pitchEl = document.getElementById("imu-pitch");
  if (rollEl)  { rollEl.textContent  = roll.toFixed(1)  + "°";  rollEl.className  = "v " + rollColor; }
  if (pitchEl) { pitchEl.textContent = pitch.toFixed(1) + "°"; pitchEl.className = "v " + pitchColor; }

  const tiltEl = document.getElementById("imu-tilt-ok");
  if (tiltEl) {
    tiltEl.textContent = d.tilt_ok ? "SAFE" : "⚠ TILT ALERT";
    tiltEl.className   = "v " + (d.tilt_ok ? "green" : "red");
  }

  drawTiltIndicator(roll, pitch);
}

// ─── Tilt indicator canvas ────────────────────────────────────────────────────

function drawTiltIndicator(roll, pitch) {
  const canvas = document.getElementById("tilt-canvas");
  if (!canvas) return;
  const ctx = canvas.getContext("2d");
  const W = canvas.width, H = canvas.height;
  const cx = W / 2, cy = H / 2;

  ctx.clearRect(0, 0, W, H);

  // Background
  ctx.fillStyle = "#21262d";
  ctx.fillRect(0, 0, W, H);

  // Horizon line (rotated by roll)
  const rad   = roll * Math.PI / 180;
  const len   = W * 0.45;
  const dy    = Math.sin(rad) * len;
  const dx    = Math.cos(rad) * len;
  // Pitch offset: 1° = 1px
  const yOff  = pitch;

  ctx.strokeStyle = Math.abs(roll) > 20 || Math.abs(pitch) > 20 ? "#f85149" : "#3fb950";
  ctx.lineWidth   = 2;
  ctx.beginPath();
  ctx.moveTo(cx - dx, cy + dy + yOff);
  ctx.lineTo(cx + dx, cy - dy + yOff);
  ctx.stroke();

  // Centre cross
  ctx.strokeStyle = "#8b949e";
  ctx.lineWidth   = 1;
  ctx.beginPath();
  ctx.moveTo(cx - 8, cy); ctx.lineTo(cx + 8, cy);
  ctx.moveTo(cx, cy - 8); ctx.lineTo(cx, cy + 8);
  ctx.stroke();

  // Roll text (left)
  ctx.fillStyle = "#8b949e";
  ctx.font      = "10px monospace";
  ctx.fillText("R:" + roll.toFixed(1) + "°",  4, H - 4);
  ctx.fillText("P:" + pitch.toFixed(1) + "°", W / 2 + 4, H - 4);
}

// ─── Navigation telemetry ─────────────────────────────────────────────────────

function onNavTelemetry(d) {
  state.currentWp = d.wp;
  updateProgress(d.wp, d.total);
  setEl("nav-wp",   (d.wp + 1) + " / " + d.total);
  setEl("nav-dist", d.dist_m.toFixed(1) + " m");
  setEl("nav-bear", d.bearing.toFixed(0) + "°");

  // Highlight current waypoint on map
  highlightWaypoint(d.wp);
}

// ─── Mission event handler ────────────────────────────────────────────────────

function onMissionEvent(d) {
  if (d.event === "wp_reached") {
    state.currentWp = d.wp;
    updateProgress(d.wp, d.total);
    highlightWaypoint(d.wp);
  } else if (d.event === "complete") {
    state.missionStatus = "complete";
    updateMissionStatusUI("complete");
    toast("Mission complete! Robot has returned to start.", "success");
  }
}

// ─── Mission loaded handler ───────────────────────────────────────────────────

function onMissionLoaded(d) {
  state.corners       = d.boundary  || [];
  state.waypoints     = d.waypoints || [];
  state.missionStatus = d.status    || "idle";

  redrawBoundary();
  redrawPath();
  updateMissionStatusUI(state.missionStatus);
}

// ─── Boundary drawing ─────────────────────────────────────────────────────────

function enterBoundaryMode() {
  state.boundaryMode = true;
  state.corners      = [];
  clearBoundary();
  document.getElementById("map").style.cursor = "crosshair";
  document.getElementById("boundary-hint").style.display = "block";
  document.getElementById("btn-add-boundary").classList.add("active");
}

function exitBoundaryMode() {
  state.boundaryMode = false;
  document.getElementById("map").style.cursor = "";
  document.getElementById("boundary-hint").style.display = "none";
  document.getElementById("btn-add-boundary").classList.remove("active");
}

map.on("click", function (e) {
  if (!state.boundaryMode) return;
  state.corners.push({ lat: e.latlng.lat, lon: e.latlng.lng });
  redrawBoundary();
  updateCornerList();
});

function clearBoundary() {
  if (boundaryLayer) { map.removeLayer(boundaryLayer); boundaryLayer = null; }
  boundaryMarkers.forEach(m => map.removeLayer(m));
  boundaryMarkers = [];
}

function redrawBoundary() {
  clearBoundary();
  if (state.corners.length < 1) return;

  const latlngs = state.corners.map(c => [c.lat, c.lon]);

  // Labels A, B, C, D, E…
  const LABELS = "ABCDEFGHIJKLMNOPQRSTUVWXYZ";
  state.corners.forEach((c, i) => {
    const icon = L.divIcon({
      html:      `<div style="
        background:#e3873a;color:#000;font-weight:700;font-size:11px;
        width:20px;height:20px;border-radius:50%;display:flex;
        align-items:center;justify-content:center;
        border:2px solid #fff;box-shadow:0 1px 4px #000;">${LABELS[i] || i}</div>`,
      iconSize:  [20, 20],
      iconAnchor:[10, 10],
      className: "",
    });
    const m = L.marker([c.lat, c.lon], { icon, interactive: false }).addTo(map);
    boundaryMarkers.push(m);
  });

  if (state.corners.length >= 2) {
    const closed = [...latlngs];
    if (state.corners.length >= 3) closed.push(latlngs[0]);
    boundaryLayer = L.polyline(closed, {
      color:     "#e3873a",
      weight:    2,
      dashArray: "6 4",
      interactive: false,
    }).addTo(map);
  }
}

function undoLastCorner() {
  if (state.corners.length === 0) return;
  state.corners.pop();
  redrawBoundary();
  updateCornerList();
}

function clearAllBoundary() {
  state.corners = [];
  clearBoundary();
  clearPath();
  updateCornerList();
  exitBoundaryMode();
}

function updateCornerList() {
  const el = document.getElementById("corner-list");
  if (!el) return;
  const LABELS = "ABCDEFGHIJKLMNOPQRSTUVWXYZ";
  el.innerHTML = state.corners.map((c, i) =>
    `<div class="corner-item">
      <span class="corner-label">${LABELS[i] || i}</span>
      <span>${c.lat.toFixed(6)}, ${c.lon.toFixed(6)}</span>
    </div>`
  ).join("") || '<span style="color:var(--text-dim)">No points yet</span>';

  const btnClose = document.getElementById("btn-close-poly");
  if (btnClose) btnClose.disabled = state.corners.length < 3;
}

function closeBoundary() {
  if (state.corners.length < 3) { toast("Need at least 3 corner points", "warning"); return; }
  exitBoundaryMode();
  redrawBoundary();
  toast(`Boundary set with ${state.corners.length} corners`, "success");
}

// ─── Path generation ──────────────────────────────────────────────────────────

async function generatePath() {
  if (state.corners.length < 3) {
    toast("Draw a boundary first (need ≥ 3 corners)", "warning"); return;
  }

  const spacing  = parseFloat(document.getElementById("row-spacing").value) || 0.8;
  const angleEl  = document.getElementById("sweep-angle");
  const angle    = angleEl.value === "auto" ? null : parseFloat(angleEl.value);
  const rth      = document.getElementById("return-home").checked;

  const res = await apiPost("/api/generate_path", {
    boundary:       state.corners,
    row_spacing_m:  spacing,
    angle_deg:      angle,
    return_to_home: rth,
  });

  if (res.error) { toast("Path error: " + res.error, "error"); return; }

  state.waypoints = res.waypoints;
  redrawPath();

  const s = res.stats;
  const statsEl = document.getElementById("path-stats");
  if (statsEl) {
    statsEl.style.display = "block";
    statsEl.innerHTML =
      `<b>${s.waypoints}</b> waypoints &nbsp;·&nbsp; ` +
      `<b>${s.distance_m} m</b> total &nbsp;·&nbsp; ` +
      `~<b>${Math.ceil(s.time_s / 60)} min</b> est.`;
  }

  toast(`Generated ${res.waypoints.length} waypoints`, "success");
}

function clearPath() {
  if (pathLayer) { map.removeLayer(pathLayer); pathLayer = null; }
  wpMarkers.forEach(m => map.removeLayer(m));
  wpMarkers = [];
  if (currentWpMarker) { map.removeLayer(currentWpMarker); currentWpMarker = null; }
  if (homeMarker)      { map.removeLayer(homeMarker);      homeMarker      = null; }
  const statsEl = document.getElementById("path-stats");
  if (statsEl) statsEl.style.display = "none";
}

function redrawPath() {
  clearPath();
  const wps = state.waypoints;
  if (!wps || wps.length < 2) return;

  const latlngs = wps.map(w => [w.lat, w.lon]);

  pathLayer = L.polyline(latlngs, {
    color:   "#39c5cf",
    weight:  1.5,
    opacity: 0.8,
    interactive: false,
  }).addTo(map);

  // Small waypoint dots
  wps.forEach((w, i) => {
    const m = L.circleMarker([w.lat, w.lon], {
      radius:      3,
      color:       "#39c5cf",
      weight:      1,
      fillColor:   "#39c5cf",
      fillOpacity: 0.7,
      interactive: false,
    }).addTo(map);
    wpMarkers.push(m);
  });

  // Home marker (first WP)
  const homeIcon = L.divIcon({
    html: `<div style="
      background:#3fb950;color:#000;font-size:12px;font-weight:700;
      width:22px;height:22px;border-radius:50%;display:flex;
      align-items:center;justify-content:center;
      border:2px solid #fff;box-shadow:0 1px 4px #000;">H</div>`,
    iconSize: [22, 22], iconAnchor: [11, 11], className: "",
  });
  homeMarker = L.marker(latlngs[0], { icon: homeIcon }).addTo(map)
    .bindPopup("Start / Home");

  map.fitBounds(pathLayer.getBounds(), { padding: [30, 30] });
}

function highlightWaypoint(idx) {
  if (currentWpMarker) { map.removeLayer(currentWpMarker); currentWpMarker = null; }
  if (!state.waypoints[idx]) return;
  const w = state.waypoints[idx];
  currentWpMarker = L.circleMarker([w.lat, w.lon], {
    radius:      7,
    color:       "#d29922",
    weight:      2,
    fillColor:   "#d29922",
    fillOpacity: 0.5,
    interactive: false,
  }).addTo(map);
}

// ─── Mission save / load ──────────────────────────────────────────────────────

async function saveMission() {
  const name    = document.getElementById("mission-name").value.trim() || "mission";
  const spacing = parseFloat(document.getElementById("row-spacing").value) || 0.8;
  const res = await apiPost("/api/mission/save", { name, row_spacing_m: spacing });
  if (res.ok) {
    toast(`Saved as "${res.filename}"`, "success");
    loadMissionList();
  } else {
    toast("Save failed: " + (res.error || "unknown"), "error");
  }
}

async function loadMissionList() {
  const list = await apiFetch("/api/mission/list");
  const sel  = document.getElementById("mission-select");
  if (!sel || !Array.isArray(list)) return;
  sel.innerHTML = '<option value="">— select saved mission —</option>';
  list.forEach(m => {
    const dt  = m.saved_at ? new Date(m.saved_at * 1000).toLocaleDateString() : "";
    const opt = document.createElement("option");
    opt.value       = m.filename;
    opt.textContent = `${m.name}  (${m.wps} wps, ${dt})`;
    sel.appendChild(opt);
  });
}

async function loadSelectedMission() {
  const sel      = document.getElementById("mission-select");
  const filename = sel ? sel.value : "";
  if (!filename) return;
  const data = await apiFetch(`/api/mission/load/${filename}`);
  if (data.error) { toast("Load failed: " + data.error, "error"); return; }
  onMissionLoaded(data);
  toast(`Loaded: ${data.name}`, "success");
}

async function deleteSelectedMission() {
  const sel      = document.getElementById("mission-select");
  const filename = sel ? sel.value : "";
  if (!filename) return;
  if (!confirm(`Delete ${filename}?`)) return;
  await apiFetch(`/api/mission/delete/${filename}`, { method: "DELETE" });
  toast("Mission deleted", "success");
  loadMissionList();
}

// ─── Execution control ────────────────────────────────────────────────────────

async function execStart() {
  const res = await apiPost("/api/execute/start", {});
  if (res.error) { toast(res.error, "error"); return; }
  state.missionStatus = "running";
  updateMissionStatusUI("running");
  toast("Mission started", "success");
}

async function execPause() {
  await apiPost("/api/execute/pause", {});
  state.missionStatus = "paused";
  updateMissionStatusUI("paused");
  toast("Mission paused", "warning");
}

async function execResume() {
  await apiPost("/api/execute/resume", {});
  state.missionStatus = "running";
  updateMissionStatusUI("running");
}

async function execAbort() {
  await apiPost("/api/execute/abort", {});
  state.missionStatus = "idle";
  updateMissionStatusUI("idle");
  toast("Mission aborted", "error");
}

function updateMissionStatusUI(status) {
  const badge = document.getElementById("exec-status-badge");
  if (badge) {
    const labels = { idle:"IDLE", running:"RUNNING", paused:"PAUSED", complete:"COMPLETE" };
    const classes = { idle:"badge-idle", running:"badge-running",
                      paused:"badge-paused", complete:"badge-complete" };
    badge.textContent = labels[status] || status.toUpperCase();
    badge.className   = "status-badge " + (classes[status] || "badge-idle");
  }

  const btnStart  = document.getElementById("btn-exec-start");
  const btnPause  = document.getElementById("btn-exec-pause");
  const btnResume = document.getElementById("btn-exec-resume");
  const btnAbort  = document.getElementById("btn-exec-abort");

  if (btnStart)  btnStart.disabled  = status === "running";
  if (btnPause)  btnPause.disabled  = status !== "running";
  if (btnResume) btnResume.disabled = status !== "paused";
  if (btnAbort)  btnAbort.disabled  = status === "idle";
}

function updateProgress(wp, total) {
  const fill = document.getElementById("exec-progress-fill");
  const pct  = total > 0 ? Math.round((wp / total) * 100) : 0;
  if (fill) fill.style.width = pct + "%";
  setEl("exec-progress-pct", pct + "%");
  setEl("nav-wp", (wp + 1) + " / " + (total || "--"));
}

// ─── Manual control ───────────────────────────────────────────────────────────

function manualMove(steer, fwd) {
  const speed = state.manualSpeed;
  apiPost("/api/manual", { steer: steer * speed / 100, throttle: fwd * speed / 100 });
}

function manualStop() {
  apiPost("/api/manual/stop", {});
}

// Touch/mouse hold for d-pad buttons
function holdDpad(steer, fwd) {
  manualMove(steer, fwd);
}

let _dpadInterval = null;

function dpadDown(steer, fwd) {
  manualMove(steer, fwd);
  _dpadInterval = setInterval(() => manualMove(steer, fwd), 120);
}

function dpadUp() {
  clearInterval(_dpadInterval);
  manualStop();
}

// Keyboard manual control
document.addEventListener("keydown", e => {
  if (e.target.tagName === "INPUT" || e.target.tagName === "SELECT") return;
  switch (e.code) {
    case "ArrowUp":    case "KeyW": manualMove(0, 1);   break;
    case "ArrowDown":  case "KeyS": manualMove(0, -1);  break;
    case "ArrowLeft":  case "KeyA": manualMove(-1, 0.5);break;
    case "ArrowRight": case "KeyD": manualMove(1, 0.5); break;
    case "Space":                   manualStop();        break;
  }
});
document.addEventListener("keyup", e => {
  if (["ArrowUp","ArrowDown","ArrowLeft","ArrowRight","KeyW","KeyS","KeyA","KeyD"].includes(e.code)) {
    manualStop();
  }
});

// ─── Map toolbar ─────────────────────────────────────────────────────────────

document.getElementById("btn-add-boundary")?.addEventListener("click", () => {
  if (state.boundaryMode) { exitBoundaryMode(); }
  else                    { enterBoundaryMode(); }
});

document.getElementById("btn-layer-toggle")?.addEventListener("click", () => {
  if (usingSatellite) { map.removeLayer(tileSat); tileOSM.addTo(map); }
  else               { map.removeLayer(tileOSM); tileSat.addTo(map); }
  usingSatellite = !usingSatellite;
});

document.getElementById("btn-centre-gps")?.addEventListener("click", () => {
  if (gpsMarker) map.setView(gpsMarker.getLatLng(), 20);
});

document.getElementById("btn-fit-path")?.addEventListener("click", () => {
  if (pathLayer)    map.fitBounds(pathLayer.getBounds(),    { padding: [30,30] });
  else if (boundaryLayer) map.fitBounds(boundaryLayer.getBounds(), { padding: [40,40] });
});

// ─── Row spacing live preview ──────────────────────────────────────────────────

document.getElementById("row-spacing")?.addEventListener("input", function() {
  setEl("row-spacing-val", parseFloat(this.value).toFixed(1) + " m");
});

// ─── Panel collapse ────────────────────────────────────────────────────────────

document.querySelectorAll(".panel-header").forEach(hdr => {
  hdr.addEventListener("click", () => {
    hdr.closest(".panel").classList.toggle("collapsed");
  });
});

// ─── API helpers ──────────────────────────────────────────────────────────────

async function apiFetch(url, opts = {}) {
  try {
    const res = await fetch(url, opts);
    return await res.json();
  } catch (e) {
    toast("Network error: " + e.message, "error");
    return { error: e.message };
  }
}

async function apiPost(url, body) {
  return apiFetch(url, {
    method:  "POST",
    headers: { "Content-Type": "application/json" },
    body:    JSON.stringify(body),
  });
}

// ─── Toast ────────────────────────────────────────────────────────────────────

function toast(msg, type = "info") {
  const el  = document.createElement("div");
  el.className   = "toast " + type;
  el.textContent = msg;
  document.getElementById("toast-container").appendChild(el);
  setTimeout(() => el.remove(), 3500);
}

// ─── DOM helper ──────────────────────────────────────────────────────────────

function setEl(id, val) {
  const el = document.getElementById(id);
  if (el) el.textContent = val;
}

// ─── Init ─────────────────────────────────────────────────────────────────────

loadMissionList();
updateMissionStatusUI("idle");
updateCornerList();
