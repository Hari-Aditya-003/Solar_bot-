/* Solar Bot Mission Planner — front-end controller
   ================================================ */
'use strict';

(() => {

const $ = (id) => document.getElementById(id);

// ─── State ────────────────────────────────────────────────────────────────

const state = {
  boundary:  [],          // [{lat, lon}]
  waypoints: [],          // [{seq, lat, lon}]
  mode:      'gps',       // 'gps' | 'non_gps'
  mission:   { state: 'idle', current_wp: 0, wp_total: 0 },
  robot:     { speed_cms: 0, battery_pct: null },
  radio:     { connected: false, signal_ok: false, steer_pct: 0, throttle_pct: 0 },
  gps:       { accuracy_m: null, speed_ms: 0 },
  pose:      { has_position: false, source: 'none', heading: 0 },
  capture:   { point_count: 0, recording: false, planned: false, last_source: 'none', points: [] },
  plannerSettings: { path_mode: 'coverage' },
  followRobot: true,
};

// ─── Map setup ────────────────────────────────────────────────────────────

const map = L.map('map', { zoomControl: true, preferCanvas: true })
            .setView([19.0760, 72.8777], 18);

const tileEsri = L.tileLayer(
  'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
  { maxZoom: 19, attribution: 'Imagery © Esri' });
const tileOSM  = L.tileLayer(
  'https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',
  { maxZoom: 22, maxNativeZoom: 19, attribution: '© OpenStreetMap' });
tileOSM.addTo(map);
L.control.layers({ Satellite: tileEsri, Streets: tileOSM }, {}, { position: 'topright' }).addTo(map);

let satelliteTileErrors = 0;
tileEsri.on('tileerror', () => {
  satelliteTileErrors += 1;
  if (satelliteTileErrors >= 3 && map.hasLayer(tileEsri)) {
    map.removeLayer(tileEsri);
    tileOSM.addTo(map);
    log('map', 'satellite unavailable here; switched to streets', 'warn');
  }
});

// Drawing layer + toolbar
const drawnItems = new L.FeatureGroup().addTo(map);
const drawCtl = new L.Control.Draw({
  position: 'topleft',
  draw: {
    polygon: {
      allowIntersection: false,
      showArea: true,
      shapeOptions: { color: '#6ec1ff', weight: 2, fillOpacity: 0.06 },
    },
    polyline:   false,
    rectangle:  { shapeOptions: { color: '#6ec1ff' } },
    circle:     false,
    marker:     false,
    circlemarker: false,
  },
  edit: { featureGroup: drawnItems, remove: true },
});
map.addControl(drawCtl);

// Path / waypoint / robot layers
const pathLayer = L.layerGroup().addTo(map);
let robotMarker = null;
const breadcrumbs = L.polyline([], { color: '#36d399', weight: 2, opacity: 0.7 }).addTo(map);
let activeSegment = null;
let lastDrawnPose = null;

// ─── Drawing handlers ─────────────────────────────────────────────────────

map.on(L.Draw.Event.CREATED, (e) => {
  drawnItems.clearLayers();
  drawnItems.addLayer(e.layer);
  syncBoundaryFromMap();
});

map.on('draw:edited',  syncBoundaryFromMap);
map.on('draw:deleted', () => { state.boundary = []; updateKVs(); });

function syncBoundaryFromMap() {
  state.boundary = [];
  drawnItems.eachLayer((layer) => {
    if (typeof layer.getLatLngs === 'function') {
      const ring = layer.getLatLngs();
      const coords = (Array.isArray(ring[0]) ? ring[0] : ring).map(
        ll => ({ lat: ll.lat, lon: ll.lng }));
      state.boundary = coords;
    } else if (typeof layer.getLatLng === 'function') {
      const ll = layer.getLatLng();
      state.boundary.push({ lat: ll.lat, lon: ll.lng });
    }
  });
  updateKVs();
}

function renderBoundaryGeometry(boundary, { fit = false } = {}) {
  state.boundary = Array.isArray(boundary) ? boundary.map(p => ({ lat: p.lat, lon: p.lon })) : [];
  drawnItems.clearLayers();

  if (state.boundary.length >= 3) {
    drawnItems.addLayer(L.polygon(
      state.boundary.map(p => [p.lat, p.lon]),
      { color: '#6ec1ff', weight: 2, fillOpacity: 0.06 },
    ));
  } else if (state.boundary.length === 2) {
    drawnItems.addLayer(L.polyline(
      state.boundary.map(p => [p.lat, p.lon]),
      { color: '#6ec1ff', weight: 3, opacity: 0.95 },
    ));
  } else if (state.boundary.length === 1) {
    const p = state.boundary[0];
    drawnItems.addLayer(L.circleMarker([p.lat, p.lon], {
      radius: 5,
      color: '#6ec1ff',
      fillColor: '#6ec1ff',
      fillOpacity: 1,
    }));
  }

  if (fit && drawnItems.getLayers().length) {
    map.fitBounds(drawnItems.getBounds().pad(0.2));
  }
  updateKVs();
}

function applyPlannerSync(data = {}) {
  if (data.planner_settings) {
    state.plannerSettings = data.planner_settings;
    syncPathModeUI();
  }
  if (Array.isArray(data.boundary)) {
    renderBoundaryGeometry(data.boundary);
  }
  if (Array.isArray(data.waypoints) && data.waypoints.length) {
    if (data.plan_stats) {
      renderPlan({
        waypoints: data.waypoints,
        rows: data.plan_stats.rows ?? 0,
        distance_m: data.plan_stats.distance_m ?? 0,
        effective_spacing_m: data.plan_stats.effective_spacing_m ?? 0,
        estimated_time_s: data.plan_stats.estimated_time_s ?? 0,
      });
    } else {
      state.waypoints = data.waypoints;
      renderWaypoints(data.waypoints);
    }
  } else if (Array.isArray(data.waypoints) && !data.waypoints.length) {
    pathLayer.clearLayers();
    state.waypoints = [];
    clearActiveSegment();
    $('kv-rows').textContent = '—';
    $('kv-dist').textContent = '— m';
    $('kv-spacing').textContent = '— m';
    $('kv-eta').textContent = '—';
  }
  if (data.capture) {
    state.capture = data.capture;
    syncCaptureUI();
  }
  if (data.saved_filename) {
    refreshMissions();
  }
}

function syncCaptureUI() {
  $('kv-drive-points').textContent = state.capture.point_count ?? 0;
  let mode = 'idle';
  if (state.capture.recording) mode = 'recording';
  else if (state.capture.planned) mode = 'planned';
  $('kv-capture').textContent = mode;
}

function currentPathMode() {
  return state.plannerSettings?.path_mode === 'boundary' ? 'boundary' : 'coverage';
}

function syncPathModeUI() {
  const mode = currentPathMode();
  document.querySelectorAll('[data-path-mode]').forEach((btn) => {
    btn.classList.toggle('active', btn.dataset.pathMode === mode);
  });
  $('plan-mode-hint').textContent = mode === 'boundary'
    ? 'Save the driven boundary itself as the route to replay later.'
    : 'Generate lawnmower lanes inside the boundary.';
  $('btn-generate').textContent = mode === 'boundary' ? 'Save boundary route' : 'Generate path';
  $('btn-close-loop').textContent = mode === 'boundary' ? 'Save Route' : 'Close + Generate';
  $('kv-rows-label').textContent = mode === 'boundary' ? 'Segments' : 'Lanes';
  $('lbl-rth').textContent = mode === 'boundary'
    ? 'Close route back to the first point'
    : 'Return to home after coverage';
}

syncPathModeUI();

// ─── Slider wiring ────────────────────────────────────────────────────────

const sliders = {
  width:   { el: $('r-width'),   lbl: $('lbl-width'),   fmt: v => `${(+v).toFixed(2)} m` },
  overlap: { el: $('r-overlap'), lbl: $('lbl-overlap'), fmt: v => `${v} %` },
  angle:   { el: $('r-angle'),   lbl: $('lbl-angle'),   fmt: v => `${v}°` },
  speed:   { el: $('r-speed'),   lbl: $('lbl-speed'),   fmt: v => `${v} %` },
};
let angleAuto = true;
let plannerSettingsTimer = null;

function pushPlannerSettings() {
  clearTimeout(plannerSettingsTimer);
  plannerSettingsTimer = setTimeout(() => {
    postJSON('/api/planner/settings', {
      cleaning_width_m: +$('r-width').value,
      overlap_pct: +$('r-overlap').value,
      sweep_angle_deg: angleAuto ? null : +$('r-angle').value,
      return_to_home: $('cb-rth').checked,
      path_mode: currentPathMode(),
    });
  }, 120);
}

for (const [key, s] of Object.entries(sliders)) {
  s.el.addEventListener('input', () => {
    if (key === 'angle') angleAuto = false;
    s.lbl.textContent = s.fmt(s.el.value);
    if (key === 'speed') postJSON('/api/speed', { pct: +s.el.value });
    else pushPlannerSettings();
  });
}
$('btn-angle-auto').addEventListener('click', () => {
  angleAuto = true;
  $('lbl-angle').textContent = 'auto';
  pushPlannerSettings();
});

$('cb-rth').addEventListener('change', pushPlannerSettings);
document.querySelectorAll('[data-path-mode]').forEach((btn) => {
  btn.addEventListener('click', () => {
    state.plannerSettings.path_mode = btn.dataset.pathMode;
    syncPathModeUI();
    pushPlannerSettings();
  });
});

// ─── Mission planning ─────────────────────────────────────────────────────

$('btn-clear').addEventListener('click', () => {
  drawnItems.clearLayers();
  state.boundary = [];
  pathLayer.clearLayers();
  state.waypoints = [];
  breadcrumbs.setLatLngs([]);
  clearActiveSegment();
  updateKVs();
});

$('btn-mark-point').addEventListener('click', async () => {
  const r = await postJSON('/api/boundary/mark', {});
  if (!r.ok) return toast(r.message || r.error || 'Point save failed', 'bad');
  toast(r.message || 'Boundary point saved', 'good');
});

$('btn-close-loop').addEventListener('click', async () => {
  const r = await postJSON('/api/boundary/plan_or_start', {});
  if (!r.ok) return toast(r.message || r.error || 'Boundary action failed', 'bad');
  toast(r.message || 'Boundary action done', 'good');
});

$('btn-reset-drive').addEventListener('click', async () => {
  const r = await postJSON('/api/boundary/reset', {});
  if (!r.ok) return toast(r.message || r.error || 'Reset failed', 'bad');
  renderBoundaryGeometry([]);
  toast(r.message || 'Boundary reset', 'good');
});

$('btn-recenter').addEventListener('click', () => {
  if (drawnItems.getLayers().length) map.fitBounds(drawnItems.getBounds().pad(0.3));
});

$('btn-generate').addEventListener('click', async () => {
  if (state.boundary.length < 3) return toast('Draw a polygon first', 'warn');
  const body = {
    boundary:         state.boundary,
    cleaning_width_m: +$('r-width').value,
    overlap_pct:      +$('r-overlap').value,
    sweep_angle_deg:  angleAuto ? null : +$('r-angle').value,
    return_to_home:   $('cb-rth').checked,
    path_mode:        currentPathMode(),
  };
  try {
    const r = await postJSON('/api/plan', body);
    if (r.error) return toast(r.error, 'bad');
    renderPlan(r.plan);
    toast(
      currentPathMode() === 'boundary'
        ? `Saved route with ${r.plan.rows} segments · ${r.plan.distance_m} m`
        : `Generated ${r.plan.rows} lanes · ${r.plan.distance_m} m`,
      'good',
    );
  } catch (err) {
    toast(`Plan failed: ${err}`, 'bad');
  }
});

function renderPlan(plan) {
  state.waypoints = plan.waypoints;
  breadcrumbs.setLatLngs([]);
  renderWaypoints(plan.waypoints);

  $('kv-rows').textContent    = plan.rows ?? '—';
  $('kv-dist').textContent    = Number.isFinite(plan.distance_m) ? `${plan.distance_m} m` : '— m';
  $('kv-spacing').textContent = currentPathMode() === 'boundary'
    ? 'route'
    : (plan.effective_spacing_m ? `${plan.effective_spacing_m} m` : '— m');
  $('kv-eta').textContent     = plan.estimated_time_s ? secsToHMS(plan.estimated_time_s) : '—';
}

function renderWaypoints(waypoints) {
  pathLayer.clearLayers();
  clearActiveSegment();

  const latlngs = waypoints.map(w => [w.lat, w.lon]);
  if (!latlngs.length) return;

  L.polyline(latlngs, { color: '#ffb347', weight: 3, opacity: 0.95 }).addTo(pathLayer);

  if (currentPathMode() === 'coverage') {
    // Cleaning passes (highlighted) — every other segment
    for (let i = 0; i + 1 < latlngs.length; i += 2) {
      L.polyline([latlngs[i], latlngs[i + 1]],
        { color: '#ffce6a', weight: 4, opacity: 1 }).addTo(pathLayer);
    }
  }

  L.circleMarker(latlngs[0], { radius: 6, color: '#36d399',
    fillColor: '#36d399', fillOpacity: 1 }).bindTooltip('Start').addTo(pathLayer);
  L.circleMarker(latlngs[latlngs.length - 1],
    { radius: 6, color: '#f87171', fillColor: '#f87171',
      fillOpacity: 1 }).bindTooltip('End').addTo(pathLayer);

  map.fitBounds(L.latLngBounds(latlngs).pad(0.15));
  updateActiveSegment(0);
}

function updateKVs() {
  $('kv-verts').textContent = state.boundary.length;
  $('kv-area').textContent  = state.boundary.length >= 3
    ? `${approxArea(state.boundary).toFixed(1)} m²` : '— m²';
  syncCaptureUI();
}

function approxArea(pts) {
  if (pts.length < 3) return 0;
  const lat0 = pts[0].lat * Math.PI / 180;
  const m = 111320;
  const xy = pts.map(p => [
    (p.lon - pts[0].lon) * Math.cos(lat0) * m,
    (p.lat - pts[0].lat) * m,
  ]);
  let s = 0;
  for (let i = 0; i < xy.length; i++) {
    const [x1, y1] = xy[i];
    const [x2, y2] = xy[(i + 1) % xy.length];
    s += x1 * y2 - x2 * y1;
  }
  return Math.abs(s) / 2;
}

function secsToHMS(s) {
  s = Math.round(s);
  const h = Math.floor(s / 3600);
  const m = Math.floor((s % 3600) / 60);
  const r = s % 60;
  return h ? `${h}h ${m}m` : (m ? `${m}m ${r}s` : `${r}s`);
}

// ─── Mission save / load ──────────────────────────────────────────────────

$('btn-save').addEventListener('click', async () => {
  if (!state.waypoints.length) return toast('Generate a path first', 'warn');
  const name = $('mission-name').value.trim() || `mission_${Date.now()}`;
  const r = await postJSON('/api/missions', {
    name,
    params: {
      cleaning_width_m: +$('r-width').value,
      overlap_pct: +$('r-overlap').value,
      sweep_angle_deg: angleAuto ? null : +$('r-angle').value,
      path_mode: currentPathMode(),
    },
    stats: {
      rows: +$('kv-rows').textContent || 0,
      distance_m: parseFloat($('kv-dist').textContent) || 0,
    },
  });
  if (r.ok) { toast(`Saved “${name}”`, 'good'); refreshMissions(); }
});

async function refreshMissions() {
  const list = await fetch('/api/missions').then(r => r.json());
  const ul = $('mission-list');
  ul.innerHTML = '';
  if (!list.length) { ul.innerHTML = '<li class="empty">No saved missions yet.</li>'; return; }
  for (const m of list) {
    const li = document.createElement('li');
    li.innerHTML = `
      <div>
        <div>${escapeHtml(m.name)}</div>
        <div class="meta">${m.waypoint_count} wp · ${Math.round(m.distance_m)} m</div>
      </div>
      <button class="x" title="Delete">✕</button>`;
    li.addEventListener('click', async (ev) => {
      if (ev.target.classList.contains('x')) return;
      const data = await fetch(`/api/missions/${encodeURIComponent(m.filename)}`)
                        .then(r => r.json());
      if (data.boundary) {
        renderBoundaryGeometry(data.boundary, { fit: true });
      }
      if (data.waypoints) {
        renderPlan({
          waypoints: data.waypoints,
          rows: data.stats?.rows || 0,
          distance_m: data.stats?.distance_m || 0,
          effective_spacing_m: '',
          estimated_time_s: 0,
        });
      }
      $('mission-name').value = data.name || '';
      updateKVs();
      toast(`Loaded "${data.name}"`, 'good');
    });
    li.querySelector('.x').addEventListener('click', async (ev) => {
      ev.stopPropagation();
      await fetch(`/api/missions/${encodeURIComponent(m.filename)}`,
                  { method: 'DELETE' });
      refreshMissions();
    });
    ul.appendChild(li);
  }
}

// ─── Run mission ──────────────────────────────────────────────────────────

$('btn-start').addEventListener('click',  () => runAction('/api/run/start', {}, 'Mission started'));
$('btn-pause').addEventListener('click',  () => runAction('/api/run/pause', {}, 'Mission paused'));
$('btn-resume').addEventListener('click', () => runAction('/api/run/resume', {}, 'Mission resumed'));
$('btn-abort').addEventListener('click',  () => runAction('/api/run/abort', {}, 'Mission aborted', 'bad'));

$('btn-estop').addEventListener('click', async () => {
  const r = await postJSON('/api/estop', {});
  if (r.error) {
    toast(r.error, 'bad');
    return;
  }
  $('chip-estop').hidden = false;
  toast('E-stop engaged', 'bad');
});

$('btn-follow').addEventListener('click', () => {
  state.followRobot = !state.followRobot;
  $('btn-follow').classList.toggle('active', state.followRobot);
  $('btn-follow').textContent = state.followRobot ? 'Follow robot' : 'Free pan';
  if (state.followRobot && state.pose?.has_position) {
    map.panTo([state.pose.lat, state.pose.lon], { animate: true, duration: 0.5 });
  }
});

// Mode toggle
document.querySelectorAll('.seg').forEach(btn => {
  btn.addEventListener('click', async () => {
    const nextMode = btn.dataset.mode;
    const prevMode = state.mode;
    setModeUI(btn.dataset.mode);
    const r = await postJSON('/api/mode', { mode: state.mode });
    if (r.error) {
      setModeUI(prevMode);
      toast(r.error, 'bad');
      return;
    }
    toast(`Mode: ${nextMode === 'gps' ? 'GPS' : 'No GPS'}`, 'good');
  });
});

// ─── Manual drive (D-pad) ─────────────────────────────────────────────────

document.querySelectorAll('.dpad-btn').forEach(b => {
  let timer = null;
  const send = () => postJSON('/api/manual', {
    steer:    +b.dataset.steer,
    throttle: +b.dataset.thr,
  });
  const press = () => { send(); timer = setInterval(send, 200); };
  const release = () => {
    if (timer) clearInterval(timer);
    timer = null;
    fetch('/api/manual/stop', { method: 'POST' });
  };
  b.addEventListener('mousedown',  press);
  b.addEventListener('mouseup',    release);
  b.addEventListener('mouseleave', release);
  b.addEventListener('touchstart', (e) => { e.preventDefault(); press(); });
  b.addEventListener('touchend',   release);
});

// Keyboard shortcuts
document.addEventListener('keydown', (e) => {
  if (e.target.tagName === 'INPUT') return;
  switch (e.key.toLowerCase()) {
    case 'g': postJSON('/api/run/start', {}); break;
    case 'p': postJSON('/api/run/pause', {}); break;
    case ' ': $('btn-estop').click(); e.preventDefault(); break;
  }
});

// ─── Socket.IO live updates ───────────────────────────────────────────────

const sock = io();

sock.on('connect',    () => log('socket', 'connected', 'good'));
sock.on('disconnect', () => log('socket', 'disconnected', 'bad'));

sock.on('hello', (msg) => {
  log('hello', `server ${msg.version}`, 'good');
  if (msg.capture) {
    state.capture = msg.capture;
    syncCaptureUI();
  }
});

sock.on('planner_sync', (data) => {
  applyPlannerSync(data);
});
sock.on('ui_message', (msg) => {
  if (!msg?.text) return;
  toast(msg.text, msg.level || '');
  log('radio', msg.text, msg.level || '');
});
sock.on('boundary_point_saved', (e) =>
  log('boundary', `point ${e.count} saved`, 'good'));
sock.on('boundary_reset', () =>
  log('boundary', 'reset', 'warn'));
sock.on('plan_generated', (e) =>
  log(
    'planner',
    `${e.rows} ${e.path_mode === 'boundary' ? 'segments' : 'lanes'} · ${e.distance_m} m`,
    'good',
  ));

sock.on('gps', (g) => {
  state.gps = g;
  updateGpsQuality(g);
});

function updateGpsQuality(g) {
  state.gps = g;
  const sat = gpsSatelliteQuality(g.satellites, g.has_fix);
  const hdop = gpsHdopQuality(g.hdop);
  const acc = gpsAccuracyQuality(g.accuracy_m);
  const overall = gpsOverallQuality(sat, hdop, acc, g.has_fix);
  const accText = Number.isFinite(Number(g.accuracy_m)) ? g.accuracy_m : '—';

  $('lbl-gps').textContent = g.has_fix
    ? `${g.satellites}sat · ±${accText}m · ${overall.short}`
    : (g.satellites ? `${g.satellites} sat` : 'no fix');
  $('chip-gps').classList.toggle('ok', overall.level === 'good');
  $('chip-gps').classList.toggle('warn', overall.level === 'warn');
  $('chip-gps').classList.toggle('bad', overall.level === 'bad');
  $('hud-sats').textContent = g.satellites;
  $('hud-hdop').textContent = g.hdop ?? '—';
  if ($('hud-accuracy')) $('hud-accuracy').textContent = accText === '—' ? '—' : `±${accText}`;
  setMetric('hud-gps-quality', overall.short, overall.level);
  setMetric('t-gps-sats', `${g.satellites ?? 0}`, sat.level);
  setMetric('t-gps-sat-quality', sat.label, sat.level);
  setMetric('t-gps-hdop-quality', `${fmtMetric(g.hdop)} · ${hdop.label}`, hdop.level);
  setMetric('t-gps-accuracy', `±${fmtMetric(g.accuracy_m)} m · ${acc.label}`, acc.level);
}

function gpsSatelliteQuality(sats, hasFix) {
  const n = Number(sats) || 0;
  if (!hasFix || n < 4) return { label: 'no fix', short: 'no fix', level: 'bad' };
  if (n < 6) return { label: 'minimum fix', short: 'min', level: 'warn' };
  if (n < 8) return { label: 'slow test', short: 'test', level: 'warn' };
  if (n < 12) return { label: 'outdoor good', short: 'good', level: 'good' };
  return { label: 'very good', short: 'v.good', level: 'good' };
}

function gpsHdopQuality(hdop) {
  const n = Number(hdop);
  if (!Number.isFinite(n)) return { label: 'no data', short: 'no data', level: 'bad' };
  if (n < 1.0) return { label: 'very good', short: 'v.good', level: 'good' };
  if (n < 2.0) return { label: 'usable', short: 'usable', level: 'good' };
  return { label: 'not good', short: 'bad', level: 'bad' };
}

function gpsAccuracyQuality(accM) {
  const n = Number(accM);
  if (!Number.isFinite(n)) return { label: 'no data', short: 'no data', level: 'bad' };
  if (n < 3.0) return { label: 'good', short: 'good', level: 'good' };
  if (n <= 5.0) return { label: 'okay, may drift', short: 'okay', level: 'warn' };
  return { label: 'not good', short: 'bad', level: 'bad' };
}

function gpsOverallQuality(sat, hdop, acc, hasFix) {
  if (!hasFix || [sat, hdop, acc].some(q => q.level === 'bad')) {
    return { short: 'bad', level: 'bad' };
  }
  if ([sat, acc].some(q => q.level === 'warn')) {
    return { short: 'okay', level: 'warn' };
  }
  if (sat.label === 'very good' && hdop.label === 'very good' && acc.label === 'good') {
    return { short: 'v.good', level: 'good' };
  }
  return { short: 'good', level: 'good' };
}

function setMetric(id, text, level) {
  const el = $(id);
  if (!el) return;
  el.textContent = text;
  el.classList.remove('metric-good', 'metric-warn', 'metric-bad');
  el.classList.add(`metric-${level}`);
}

function fmtMetric(value) {
  const n = Number(value);
  if (!Number.isFinite(n)) return '—';
  return n < 10 ? n.toFixed(2).replace(/0$/, '').replace(/\.$/, '') : n.toFixed(0);
}

sock.on('robot', (r) => {
  state.robot = r;
  $('lbl-link').textContent = r.connected ? 'online' : 'offline';
  $('chip-link').classList.toggle('ok',  r.connected);
  $('chip-link').classList.toggle('bad', !r.connected);
  const battPct = Number.isFinite(r.battery_pct) ? r.battery_pct : null;
  $('lbl-batt').textContent = battPct === null ? '—' : `${battPct}%`;
  $('chip-batt').classList.toggle('ok',   battPct !== null && battPct >= 30);
  $('chip-batt').classList.toggle('warn', battPct !== null && battPct < 30 && battPct >= 15);
  $('chip-batt').classList.toggle('bad',  battPct !== null && battPct < 15);
  const tilt = Math.max(Math.abs(r.roll || 0), Math.abs(r.pitch || 0));
  $('lbl-tilt').textContent = `${tilt.toFixed(1)}°`;
  $('chip-tilt').classList.toggle('ok',  tilt < 15);
  $('chip-tilt').classList.toggle('warn', tilt >= 15 && tilt < 25);
  $('chip-tilt').classList.toggle('bad',  tilt >= 25);
  $('hud-speed').textContent = (r.speed_cms ?? 0).toFixed(1);
  $('hud-heading').textContent = `${(r.heading ?? 0).toFixed(0)}°`;
  $('hud-motion').textContent = (r.speed_cms ?? 0) > 3 ? 'moving' : 'idle';
  $('t-imu-link').textContent = r.connected ? 'live' : 'offline';
  $('t-imu-yaw').textContent = `${(r.heading ?? 0).toFixed(1)}°`;
  $('t-roll').textContent = `${(r.roll ?? 0).toFixed(1)}°`;
  $('t-pitch').textContent = `${(r.pitch ?? 0).toFixed(1)}°`;
  syncRobotMarker();
});

sock.on('radio', (r) => {
  state.radio = r;
  $('lbl-radio').textContent = r.signal_ok ? 'active' : (r.connected ? 'linked' : 'offline');
  $('chip-radio').classList.toggle('ok', !!r.signal_ok);
  $('chip-radio').classList.toggle('warn', !r.signal_ok && !!r.connected);
  $('chip-radio').classList.toggle('bad', !r.connected);
  $('t-rc-steer').textContent = `${r.steer_pct ?? 0}%`;
  $('t-rc-throttle').textContent = `${r.throttle_pct ?? 0}%`;
  $('t-rc-link').textContent = r.signal_ok ? 'live' : (r.connected ? 'idle' : 'offline');
  $('t-rc-mode').textContent = r.control_active ? 'driving' : 'idle';
});

sock.on('pose', (p) => {
  state.pose = p;
  $('t-src').textContent = p.source;
  $('t-lat').textContent = p.has_position ? p.lat.toFixed(6) : '—';
  $('t-lon').textContent = p.has_position ? p.lon.toFixed(6) : '—';
  $('hud-pose').textContent = p.source;
  if (p.has_position) updateRobotMarker(p);
});

sock.on('mission_state', (m) => {
  state.mission = m;
  $('prg-state').textContent = m.state;
  $('prg-wp').textContent    = `${m.current_wp}/${m.wp_total}`;
  $('bar-fill').style.width  = `${m.progress_pct}%`;
  updateActiveSegment(m.current_wp);
});

sock.on('nav_telemetry', (t) => {
  $('t-wp').textContent       = `${t.wp}/${t.total}`;
  $('t-dist').textContent     = `${t.dist_m} m`;
  $('t-bearing').textContent  = `${t.bearing}°`;
  $('t-heading').textContent  = `${t.heading}°`;
  $('t-steer').textContent    = t.steer;
  $('t-throttle').textContent = t.throttle;
  updateActiveSegment(t.wp);
});

sock.on('waypoint_reached', (e) =>
  log('waypoint', `→ ${e.wp}/${e.total}`, 'good'));
sock.on('mission_complete', () => {
  log('mission', 'complete!', 'good');
  toast('Mission complete', 'good');
});
sock.on('mission_started',  () => {
  breadcrumbs.setLatLngs([]);
  log('mission', 'started', 'good');
  toast('Mission started');
});
sock.on('mission_paused',   () => log('mission', 'paused',   'warn'));
sock.on('mission_resumed',  () => log('mission', 'resumed',  'good'));
sock.on('mission_aborted',  () => { log('mission', 'aborted', 'bad'); toast('Mission aborted', 'bad'); });
sock.on('safety_warn', (e)  => log('safety', e.reasons.join('; '), 'warn'));
sock.on('safety_stop', (e)  => {
  log('safety', e.reasons.join('; '), 'bad');
  toast('Safety stop: ' + e.reasons.join('; '), 'bad');
});

// ─── Robot marker ─────────────────────────────────────────────────────────

function updateRobotMarker(pose) {
  const ll = [pose.lat, pose.lon];
  const driftThresholdM = Math.max(2.5, Number(state.gps?.accuracy_m || 0) * 1.5);
  const movedM = lastDrawnPose ? distanceMeters(lastDrawnPose, pose) : Infinity;
  const robotLinked = state.robot.connected === true;
  const robotMoving = (state.robot.speed_cms ?? 0) > 3;
  const gpsMoving = (state.gps?.speed_ms ?? pose.speed_ms ?? 0) > 0.2;
  const missionRunning = state.mission.state === 'running';
  const realMotion = robotMoving || (missionRunning && robotLinked && gpsMoving);

  if (!robotMarker) {
    robotMarker = L.marker(ll, {
      icon: L.divIcon({
        className: 'robot-icon',
        html: '<div class="robot-marker"><div class="robot-arrow"></div><div class="robot-body"></div></div>',
        iconSize: [34, 34],
        iconAnchor: [17, 17],
      }),
    }).addTo(map);
    map.setView(ll, 19);
    lastDrawnPose = pose;
  } else {
    if (!missionRunning && !realMotion) {
      breadcrumbs.setLatLngs([]);
      syncRobotMarker();
      return;
    }
    if (pose.source === 'gps' && !realMotion && movedM < driftThresholdM) {
      syncRobotMarker();
      return;
    }
    robotMarker.setLatLng(ll);
    lastDrawnPose = pose;
  }
  syncRobotMarker();
  if (realMotion && (missionRunning || movedM >= driftThresholdM)) {
    breadcrumbs.addLatLng(ll);
  }
  if (state.followRobot && missionRunning) {
    map.panTo(ll, { animate: true, duration: 0.35 });
  }
}

function syncRobotMarker() {
  const markerEl = robotMarker?.getElement()?.querySelector('.robot-marker');
  if (!markerEl) return;
  markerEl.style.transform = `rotate(${state.pose.heading ?? 0}deg)`;
  markerEl.classList.toggle('moving', (state.robot.speed_cms ?? 0) > 3);
}

function updateActiveSegment(wpIndex) {
  if (!state.waypoints.length || wpIndex >= state.waypoints.length - 1) {
    clearActiveSegment();
    return;
  }
  const a = state.waypoints[wpIndex];
  const b = state.waypoints[wpIndex + 1];
  clearActiveSegment();
  activeSegment = L.polyline(
    [[a.lat, a.lon], [b.lat, b.lon]],
    { color: '#36d399', weight: 6, opacity: 0.95 },
  ).addTo(pathLayer);
}

function clearActiveSegment() {
  if (!activeSegment) return;
  pathLayer.removeLayer(activeSegment);
  activeSegment = null;
}

function setModeUI(mode) {
  state.mode = mode;
  document.querySelectorAll('.seg').forEach((b) => {
    b.classList.toggle('active', b.dataset.mode === mode);
  });
  $('lbl-mode').textContent = mode === 'gps' ? 'GPS' : 'NO-GPS';
}

function distanceMeters(a, b) {
  const lat1 = a.lat * Math.PI / 180;
  const lat2 = b.lat * Math.PI / 180;
  const dLat = (b.lat - a.lat) * Math.PI / 180;
  const dLon = (b.lon - a.lon) * Math.PI / 180;
  const h = Math.sin(dLat / 2) ** 2
    + Math.cos(lat1) * Math.cos(lat2) * Math.sin(dLon / 2) ** 2;
  return 6371000 * 2 * Math.atan2(Math.sqrt(h), Math.sqrt(1 - h));
}

// ─── Helpers ──────────────────────────────────────────────────────────────

async function postJSON(url, body) {
  const r = await fetch(url, {
    method:  'POST',
    headers: { 'Content-Type': 'application/json' },
    body:    JSON.stringify(body),
  });
  return r.json().catch(() => ({}));
}

async function runAction(url, body, okMsg = '', level = 'good') {
  const r = await postJSON(url, body);
  if (r.error || r.ok === false) {
    const msg = r.error || r.message || 'Request failed';
    toast(msg, 'bad');
    log('ui', `${url} failed: ${msg}`, 'bad');
    return r;
  }
  if (okMsg) {
    toast(okMsg, level);
  }
  return r;
}

function log(ev, text, level = '') {
  const ul = $('event-log');
  const li = document.createElement('li');
  const t  = new Date().toLocaleTimeString();
  li.innerHTML = `<span class="t">${t}</span><span class="ev ${level}">${ev}</span><span>${escapeHtml(text)}</span>`;
  ul.prepend(li);
  while (ul.children.length > 80) ul.lastChild.remove();
}

function toast(msg, level = '') {
  const host = $('toast-host');
  const t = document.createElement('div');
  t.className = `toast ${level}`;
  t.textContent = msg;
  host.appendChild(t);
  setTimeout(() => t.style.opacity = '0', 2500);
  setTimeout(() => t.remove(), 2900);
}

function escapeHtml(s) {
  return String(s ?? '').replace(/[&<>"']/g, (c) =>
    ({ '&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;' }[c]));
}

// ─── Initial fetch ────────────────────────────────────────────────────────

(async () => {
  refreshMissions();
  try {
    const s = await fetch('/api/state').then(r => r.json());
  if (s?.mode) {
      setModeUI(s.mode);
    }
    if (s?.planner_settings) {
      state.plannerSettings = s.planner_settings;
      syncPathModeUI();
      $('r-width').value = s.planner_settings.cleaning_width_m ?? s.config?.cleaning_width_m ?? $('r-width').value;
      $('lbl-width').textContent = `${(+$('r-width').value).toFixed(2)} m`;
      $('r-overlap').value = s.planner_settings.overlap_pct ?? s.config?.overlap_pct ?? $('r-overlap').value;
      $('lbl-overlap').textContent = `${$('r-overlap').value} %`;
      $('cb-rth').checked = s.planner_settings.return_to_home ?? $('cb-rth').checked;
      if (s.planner_settings.sweep_angle_deg == null) {
        angleAuto = true;
        $('lbl-angle').textContent = 'auto';
      } else {
        angleAuto = false;
        $('r-angle').value = s.planner_settings.sweep_angle_deg;
        $('lbl-angle').textContent = `${s.planner_settings.sweep_angle_deg}°`;
      }
    } else if (s?.config?.cleaning_width_m) {
      $('r-width').value   = s.config.cleaning_width_m;
      $('lbl-width').textContent = `${(+s.config.cleaning_width_m).toFixed(2)} m`;
      $('r-overlap').value = s.config.overlap_pct;
      $('lbl-overlap').textContent = `${s.config.overlap_pct} %`;
      syncPathModeUI();
    }
    if (typeof s?.target_speed_pct === 'number') {
      $('r-speed').value = s.target_speed_pct;
      $('lbl-speed').textContent = `${s.target_speed_pct} %`;
    }
    if (s?.gps) {
      updateGpsQuality(s.gps);
    }
    if (Array.isArray(s?.capture?.points) && s.capture.points.length) {
      state.capture = s.capture;
      renderBoundaryGeometry(s.capture.points);
    } else if (Array.isArray(s?.mission?.boundary) && s.mission.boundary.length) {
      renderBoundaryGeometry(s.mission.boundary);
    }
    if (Array.isArray(s?.mission?.waypoints) && s.mission.waypoints.length) {
      renderPlan({
        waypoints: s.mission.waypoints,
        rows: s.plan_stats?.rows ?? 0,
        distance_m: s.plan_stats?.distance_m ?? 0,
        effective_spacing_m: s.plan_stats?.effective_spacing_m ?? 0,
        estimated_time_s: s.plan_stats?.estimated_time_s ?? 0,
      });
    }
    if (s?.pose?.has_position) {
      state.pose = s.pose;
      updateRobotMarker(s.pose);
    }
    if (s?.robot) {
      state.robot = s.robot;
      $('hud-motion').textContent = (s.robot.speed_cms ?? 0) > 3 ? 'moving' : 'idle';
      $('hud-pose').textContent = s.pose?.source || 'none';
      $('t-imu-link').textContent = s.robot.connected ? 'live' : 'offline';
      $('t-imu-yaw').textContent = `${(s.robot.heading ?? 0).toFixed(1)}°`;
      $('t-roll').textContent = `${(s.robot.roll ?? 0).toFixed(1)}°`;
      $('t-pitch').textContent = `${(s.robot.pitch ?? 0).toFixed(1)}°`;
      syncRobotMarker();
    }
    if (s?.radio) {
      state.radio = s.radio;
      $('lbl-radio').textContent = s.radio.signal_ok ? 'active' : (s.radio.connected ? 'linked' : 'offline');
      $('chip-radio').classList.toggle('ok', !!s.radio.signal_ok);
      $('chip-radio').classList.toggle('warn', !s.radio.signal_ok && !!s.radio.connected);
      $('chip-radio').classList.toggle('bad', !s.radio.connected);
      $('t-rc-steer').textContent = `${s.radio.steer_pct ?? 0}%`;
      $('t-rc-throttle').textContent = `${s.radio.throttle_pct ?? 0}%`;
      $('t-rc-link').textContent = s.radio.signal_ok ? 'live' : (s.radio.connected ? 'idle' : 'offline');
      $('t-rc-mode').textContent = s.radio.control_active ? 'driving' : 'idle';
    }
    if (s?.capture) {
      state.capture = s.capture;
      syncCaptureUI();
    }
  } catch {}
})();

})();
