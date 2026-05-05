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
};

// ─── Map setup ────────────────────────────────────────────────────────────

const map = L.map('map', { zoomControl: true, preferCanvas: true })
            .setView([19.0760, 72.8777], 18);

const tileEsri = L.tileLayer(
  'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
  { maxZoom: 22, attribution: 'Imagery © Esri' });
const tileOSM  = L.tileLayer(
  'https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',
  { maxZoom: 19, attribution: '© OpenStreetMap' });
tileEsri.addTo(map);
L.control.layers({ Satellite: tileEsri, Streets: tileOSM }, {}, { position: 'topright' }).addTo(map);

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
    const ring = layer.getLatLngs();
    const coords = (Array.isArray(ring[0]) ? ring[0] : ring).map(
      ll => ({ lat: ll.lat, lon: ll.lng }));
    state.boundary = coords;
  });
  updateKVs();
}

// ─── Slider wiring ────────────────────────────────────────────────────────

const sliders = {
  width:   { el: $('r-width'),   lbl: $('lbl-width'),   fmt: v => `${(+v).toFixed(2)} m` },
  overlap: { el: $('r-overlap'), lbl: $('lbl-overlap'), fmt: v => `${v} %` },
  angle:   { el: $('r-angle'),   lbl: $('lbl-angle'),   fmt: v => `${v}°` },
  speed:   { el: $('r-speed'),   lbl: $('lbl-speed'),   fmt: v => `${v} %` },
};
let angleAuto = true;

for (const [key, s] of Object.entries(sliders)) {
  s.el.addEventListener('input', () => {
    if (key === 'angle') angleAuto = false;
    s.lbl.textContent = s.fmt(s.el.value);
    if (key === 'speed') postJSON('/api/speed', { pct: +s.el.value });
  });
}
$('btn-angle-auto').addEventListener('click', () => {
  angleAuto = true;
  $('lbl-angle').textContent = 'auto';
});

$('cb-rth').addEventListener('change', () => {});

// ─── Mission planning ─────────────────────────────────────────────────────

$('btn-clear').addEventListener('click', () => {
  drawnItems.clearLayers();
  state.boundary = [];
  pathLayer.clearLayers();
  state.waypoints = [];
  updateKVs();
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
  };
  try {
    const r = await postJSON('/api/plan', body);
    if (r.error) return toast(r.error, 'bad');
    renderPlan(r.plan);
    toast(`Generated ${r.plan.rows} lanes · ${r.plan.distance_m} m`, 'good');
  } catch (err) {
    toast(`Plan failed: ${err}`, 'bad');
  }
});

function renderPlan(plan) {
  state.waypoints = plan.waypoints;
  pathLayer.clearLayers();

  const latlngs = plan.waypoints.map(w => [w.lat, w.lon]);
  L.polyline(latlngs, { color: '#ffb347', weight: 3, opacity: 0.95 }).addTo(pathLayer);

  // Cleaning passes (highlighted) — every other segment
  for (let i = 0; i + 1 < latlngs.length; i += 2) {
    L.polyline([latlngs[i], latlngs[i + 1]],
      { color: '#ffce6a', weight: 4, opacity: 1 }).addTo(pathLayer);
  }
  // Numbered start/end
  if (latlngs.length) {
    L.circleMarker(latlngs[0], { radius: 6, color: '#36d399',
      fillColor: '#36d399', fillOpacity: 1 }).bindTooltip('Start').addTo(pathLayer);
    L.circleMarker(latlngs[latlngs.length - 1],
      { radius: 6, color: '#f87171', fillColor: '#f87171',
        fillOpacity: 1 }).bindTooltip('End').addTo(pathLayer);
  }

  $('kv-rows').textContent    = plan.rows;
  $('kv-dist').textContent    = `${plan.distance_m} m`;
  $('kv-spacing').textContent = `${plan.effective_spacing_m} m`;
  $('kv-eta').textContent     = secsToHMS(plan.estimated_time_s);
}

function updateKVs() {
  $('kv-verts').textContent = state.boundary.length;
  $('kv-area').textContent  = state.boundary.length >= 3
    ? `${approxArea(state.boundary).toFixed(1)} m²` : '— m²';
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
        state.boundary = data.boundary.map(p => ({ lat: p.lat, lon: p.lon }));
        drawnItems.clearLayers();
        drawnItems.addLayer(L.polygon(
          state.boundary.map(p => [p.lat, p.lon]),
          { color: '#6ec1ff', weight: 2, fillOpacity: 0.06 }));
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
      if (drawnItems.getLayers().length) map.fitBounds(drawnItems.getBounds().pad(0.2));
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

$('btn-start').addEventListener('click',  () => postJSON('/api/run/start',  {}));
$('btn-pause').addEventListener('click',  () => postJSON('/api/run/pause',  {}));
$('btn-resume').addEventListener('click', () => postJSON('/api/run/resume', {}));
$('btn-abort').addEventListener('click',  () => postJSON('/api/run/abort',  {}));

$('btn-estop').addEventListener('click', () => {
  postJSON('/api/estop', {});
  $('chip-estop').hidden = false;
});

// Mode toggle
document.querySelectorAll('.seg').forEach(btn => {
  btn.addEventListener('click', () => {
    document.querySelectorAll('.seg').forEach(b => b.classList.remove('active'));
    btn.classList.add('active');
    state.mode = btn.dataset.mode;
    $('lbl-mode').textContent = state.mode === 'gps' ? 'GPS' : 'NO-GPS';
    postJSON('/api/mode', { mode: state.mode });
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

sock.on('hello', (msg) => log('hello', `server ${msg.version}`, 'good'));

sock.on('gps', (g) => {
  $('lbl-gps').textContent = g.has_fix
    ? `${g.satellites}sat · ±${g.accuracy_m}m`
    : (g.satellites ? `${g.satellites} sat` : 'no fix');
  $('chip-gps').classList.toggle('ok', g.has_fix);
  $('chip-gps').classList.toggle('warn', !g.has_fix && g.satellites > 0);
  $('chip-gps').classList.toggle('bad', !g.has_fix && !g.satellites);
  $('hud-sats').textContent = g.satellites;
  $('hud-hdop').textContent = g.hdop ?? '—';
});

sock.on('robot', (r) => {
  $('lbl-link').textContent = r.connected ? 'online' : 'offline';
  $('chip-link').classList.toggle('ok',  r.connected);
  $('chip-link').classList.toggle('bad', !r.connected);
  $('lbl-batt').textContent = r.battery_pct ? `${r.battery_pct}%` : '—';
  $('chip-batt').classList.toggle('ok',   r.battery_pct >= 30);
  $('chip-batt').classList.toggle('warn', r.battery_pct < 30 && r.battery_pct >= 15);
  $('chip-batt').classList.toggle('bad',  r.battery_pct < 15);
  const tilt = Math.max(Math.abs(r.roll || 0), Math.abs(r.pitch || 0));
  $('lbl-tilt').textContent = `${tilt.toFixed(1)}°`;
  $('chip-tilt').classList.toggle('ok',  tilt < 15);
  $('chip-tilt').classList.toggle('warn', tilt >= 15 && tilt < 25);
  $('chip-tilt').classList.toggle('bad',  tilt >= 25);
  $('hud-speed').textContent = (r.speed_cms ?? 0).toFixed(1);
  $('hud-heading').textContent = `${(r.heading ?? 0).toFixed(0)}°`;
});

sock.on('pose', (p) => {
  $('t-src').textContent = p.source;
  $('t-lat').textContent = p.has_position ? p.lat.toFixed(6) : '—';
  $('t-lon').textContent = p.has_position ? p.lon.toFixed(6) : '—';
  if (p.has_position) updateRobotMarker(p.lat, p.lon, p.heading);
});

sock.on('mission_state', (m) => {
  state.mission = m;
  $('prg-state').textContent = m.state;
  $('prg-wp').textContent    = `${m.current_wp}/${m.wp_total}`;
  $('bar-fill').style.width  = `${m.progress_pct}%`;
});

sock.on('nav_telemetry', (t) => {
  $('t-wp').textContent       = `${t.wp}/${t.total}`;
  $('t-dist').textContent     = `${t.dist_m} m`;
  $('t-bearing').textContent  = `${t.bearing}°`;
  $('t-heading').textContent  = `${t.heading}°`;
  $('t-steer').textContent    = t.steer;
  $('t-throttle').textContent = t.throttle;
});

sock.on('waypoint_reached', (e) =>
  log('waypoint', `→ ${e.wp}/${e.total}`, 'good'));
sock.on('mission_complete', () => {
  log('mission', 'complete!', 'good');
  toast('Mission complete', 'good');
});
sock.on('mission_started',  () => { log('mission', 'started', 'good');  toast('Mission started'); });
sock.on('mission_paused',   () => log('mission', 'paused',   'warn'));
sock.on('mission_resumed',  () => log('mission', 'resumed',  'good'));
sock.on('mission_aborted',  () => { log('mission', 'aborted', 'bad'); toast('Mission aborted', 'bad'); });
sock.on('safety_warn', (e)  => log('safety', e.reasons.join('; '), 'warn'));
sock.on('safety_stop', (e)  => {
  log('safety', e.reasons.join('; '), 'bad');
  toast('Safety stop: ' + e.reasons.join('; '), 'bad');
});

// ─── Robot marker ─────────────────────────────────────────────────────────

function updateRobotMarker(lat, lon, heading) {
  const ll = [lat, lon];
  if (!robotMarker) {
    robotMarker = L.marker(ll, {
      icon: L.divIcon({ className: '', html: '<div class="robot-dot"></div>',
                        iconSize: [14, 14] }),
    }).addTo(map);
    map.setView(ll, 19);
  } else {
    robotMarker.setLatLng(ll);
  }
  breadcrumbs.addLatLng(ll);
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
    if (s?.config?.cleaning_width_m) {
      $('r-width').value   = s.config.cleaning_width_m;
      $('lbl-width').textContent = `${(+s.config.cleaning_width_m).toFixed(2)} m`;
      $('r-overlap').value = s.config.overlap_pct;
      $('lbl-overlap').textContent = `${s.config.overlap_pct} %`;
    }
    if (s.target_speed_pct) {
      $('r-speed').value = s.target_speed_pct;
      $('lbl-speed').textContent = `${s.target_speed_pct} %`;
    }
  } catch {}
})();

})();
