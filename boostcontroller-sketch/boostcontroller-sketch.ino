/*
 * ESP32 PID Pressure Controller — Normally-Open Solenoid Valve controlling wastegate
 *
 * Sensor:   0–3.3 V = -1 to +4 bar (raw), then divided by 1.1 for adjustment
 *           → effective display range -1 to ~3.6 bar
 * ADC:      GPIO1
 * MOSFET:   GPIO10  (PWM, max. ~20 kHz(is @ ~20-30hz), 10-bit)
 *           0% duty  = valve fully open  (pressure falls)   → 100% OPEN display
 *           100% duty = valve fully closed (pressure rises) →   0% OPEN display
 *
 * WEB:     http://192.168.4.1
 */

#include <WiFi.h>
#include <WebServer.h>

// ─── WiFi AP credentials ──────────────────────────────────────────────────────
const char* AP_SSID     = "BoostConfig";

// ─── Pin definitions ──────────────────────────────────────────────────────────
#define PIN_ADC   1
#define PIN_PWM   10

// ─── PWM config ───────────────────────────────────────────────────────────────
#define PWM_FREQ  28      // Hz
#define PWM_RES   10      // 10-bit → 0–1023

// ─── Sensor scaling ───────────────────────────────────────────────────────────
// 0 V → -1 bar,  3.3 V → +4 bar  (span = 5 bar over 3.3 V)
// Then divided by 1.1 for physical calibration adjustment
#define PRESSURE_AT_0V    -1.0f   // bar when ADC reads 0 V
#define PRESSURE_AT_3V3   +4.0f   // bar when ADC reads 3.3 V
#define PRESSURE_ADJUST    1.1f   // divisor for calibration

inline float voltsToPressure(float v) {
  float raw = PRESSURE_AT_0V +
              (v / 3.3f) * (PRESSURE_AT_3V3 - PRESSURE_AT_0V);
  return raw / PRESSURE_ADJUST;
}

// ─── PID parameters ───────────────────────────────────────────────────────────
//
//  Tuning direction:
//    Too slow to reach setpoint → raise Kp
//    Oscillates around setpoint → lower Kp, then raise Kd
//    Steady-state offset remains → raise Ki
//    Noisy / chattery valve     → lower Kd
//
float Kp       =  50.0f;
float Ki       =   0.5f;
float Kd       =   0.0f;
float setpoint =   0.95f;   // bar (within -1 … ~3.6 bar)
float outMin   =  -0.0f;    // PWM counts
float outMax   =   511.0f;

// ─── PID state ────────────────────────────────────────────────────────────────
float  integral      = 0.0f;
float  lastInput_bar = 0.0f;
float  pidOutput     = 0.0f;   // 0–1023 (pre-inversion)
float  pressure_bar  = 0.0f;
float  adcVolts      = 0.0f;
unsigned long lastPIDTime = 0;
#define PID_INTERVAL_MS 10     // 100 Hz

// ─── Pressure history for graph (60 s × 2 Hz = 120 samples) ──────────────────
#define HIST_LEN  120
float  pressHist[HIST_LEN];
float  spHist[HIST_LEN];
int    histIdx   = 0;
bool   histFull  = false;
unsigned long lastHistTime = 0;
#define HIST_INTERVAL_MS 500   // store one sample every 500 ms → 60 s window

// ─── Web server ───────────────────────────────────────────────────────────────
WebServer server(80);

// ─── PID compute ─────────────────────────────────────────────────────────────
void computePID() {
  unsigned long now = millis();
  float dt = (now - lastPIDTime) / 1000.0f;
  if (dt < (PID_INTERVAL_MS / 1000.0f)) return;
  lastPIDTime = now;

  // Read ADC — 8-sample average
  int raw = 0;
  for (int i = 0; i < 8; i++) raw += analogRead(PIN_ADC);
  raw /= 8;
  adcVolts     = raw * (3.3f / 4095.0f);
  pressure_bar = voltsToPressure(adcVolts);

  
  // Derivative on measurement (avoids setpoint-change kick)
  float dInput = (pressure_bar - lastInput_bar) / dt;
  float error = setpoint - pressure_bar;

  // Integral with anti-windup
 float pidUnsat = Kp * error + integral - Kd * dInput;
    if (pidUnsat > outMin && pidUnsat < outMax) {
        
        integral += Ki * error * dt;

}


integral  = constrain(integral, -400.0f, 400.0f);
pidOutput = constrain(pidUnsat, outMin, outMax);


  // ── push PWM signal  ───────────────────────────────────────
  uint32_t pwmVal = (uint32_t)pidOutput;

  // safety, if boost is over 0.12 bar over target, valve open.

  if((pressure_bar - setpoint) > 0.12f) {
    ledcWrite(PIN_PWM, 0);
  } else {
    ledcWrite(PIN_PWM, pwmVal);
  }
  
  lastInput_bar = pressure_bar;
  
  // ── History log ──────────────────────────────────────────────────────────
  if ((now - lastHistTime) >= HIST_INTERVAL_MS) {
    lastHistTime = now;
    pressHist[histIdx] = pressure_bar;
    spHist[histIdx]    = setpoint;
    histIdx = (histIdx + 1) % HIST_LEN;
    if (histIdx == 0) histFull = true;
  }

  if (Serial) {
    // dutyOpen: 0% = closed, 100% = open (inverted from pwmVal)
    float valveOpenPct = (outMax > 0) ? (1.0f - pidOutput / outMax) * 100.0f : 0.0f;
    Serial.printf("[PID] Target=%.3fbar  BoostNow=%.3fbar  err=%.4f  PWM(0-1023)=%u  OpenPct=%.1f%%\n",
                  setpoint, pressure_bar, error, pwmVal, valveOpenPct);
  }

  if (pressure_bar < -0.89) {
    Serial.printf(" Sensor Disconnect? ");
  }

}

// ─── History JSON ─────────────────────────────────────────────────────────────
// Returns arrays ordered oldest→newest
void handleHistory() {
  int count = histFull ? HIST_LEN : histIdx;
  int start = histFull ? histIdx  : 0;

  String json = "{\"pressure\":[";
  for (int i = 0; i < count; i++) {
    int idx = (start + i) % HIST_LEN;
    if (i) json += ',';
    json += String(pressHist[idx], 3);
  }
  json += "],\"setpoint\":[";
  for (int i = 0; i < count; i++) {
    int idx = (start + i) % HIST_LEN;
    if (i) json += ',';
    json += String(spHist[idx], 3);
  }
  json += "]}";
  server.send(200, "application/json", json);
}

// ─── HTML page ────────────────────────────────────────────────────────────────
String buildPage(String message = "") {
  // Duty cycle expressed as % OPEN (100% = valve wide open, 0% = fully closed)
  uint32_t pwmVal = (uint32_t)(outMax - constrain(pidOutput, outMin, outMax));
  float dutyOpenPct = (1.0f - (float)pwmVal / outMax) * 100.0f;

  // Gauge fill: map -1…+2 bar (3 bar span) to 0–100%
  float gaugePct = ((pressure_bar - (-1.0f)) / 3.0f) * 100.0f;
  gaugePct = constrain(gaugePct, 0.0f, 100.0f);

  String html = R"rawhtml(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Pressure PID</title>
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; }
  :root {
    --bg:      #0f1117;
    --surface: #1a1d27;
    --border:  #2e3147;
    --accent:  #4f8ef7;
    --accent2: #7c5cfc;
    --text:    #e8eaf0;
    --muted:   #6b7280;
    --success: #34d399;
    --warn:    #fbbf24;
    --danger:  #ef4444;
    --radius:  12px;
  }
  body {
    background: var(--bg); color: var(--text);
    font-family: 'Segoe UI', system-ui, sans-serif;
    min-height: 100vh; display: flex;
    align-items: flex-start; justify-content: center; padding: 24px;
  }
  .card {
    background: var(--surface); border: 1px solid var(--border);
    border-radius: var(--radius); width: 100%; max-width: 500px;
    padding: 32px 28px;
  }
  .header { display: flex; align-items: center; gap: 12px; margin-bottom: 8px; }
  .dot {
    width: 10px; height: 10px; border-radius: 50%;
    background: var(--success); box-shadow: 0 0 8px var(--success); flex-shrink: 0;
  }
  h1 { font-size: 1.1rem; font-weight: 600; }
  .subtitle { font-size: 0.75rem; color: var(--muted); margin-top: 2px; }

  /* view toggle */
  .view-toggle {
    display: flex; gap: 8px; margin: 16px 0 0;
  }
  .btn-view {
    flex: 1; padding: 9px 0;
    border-radius: 8px; font-size: 0.82rem; font-weight: 600;
    cursor: pointer; border: 1px solid var(--border);
    transition: background 0.15s, color 0.15s;
  }
  .btn-view.active {
    background: linear-gradient(135deg, var(--accent), var(--accent2));
    border-color: transparent; color: #fff;
  }
  .btn-view:not(.active) { background: var(--bg); color: var(--muted); }

  .status-bar {
    display: flex; gap: 8px; margin: 16px 0 20px;
    padding: 14px; background: var(--bg);
    border: 1px solid var(--border); border-radius: 8px;
  }
  .stat { flex: 1; text-align: center; }
  .stat-val { font-size: 1.1rem; font-weight: 700; color: var(--accent); }
  .stat-val.warn   { color: var(--warn); }
  .stat-val.danger { color: var(--danger); }
  .stat-lbl { color: var(--muted); font-size: 0.68rem; margin-top: 3px; }

  /* ── Gauge ─── */
  .gauge-wrap { margin: 0 0 20px; }
  .gauge-track {
    height: 8px; background: var(--border); border-radius: 99px;
    position: relative; overflow: visible;
  }
  .gauge-fill {
    height: 100%; border-radius: 99px;
    background: linear-gradient(90deg, #6b7280 0%, #34d399 20%, #4f8ef7 67%, #ef4444 100%);
    transition: width 0.4s ease;
  }
  /* zero-bar marker at 33.3% (1 bar into 3-bar span) */
  .gauge-zero {
    position: absolute; top: -4px; bottom: -4px;
    left: 33.33%; width: 2px;
    background: var(--muted); border-radius: 1px;
  }
  .gauge-labels {
    display: flex; justify-content: space-between;
    font-size: 0.65rem; color: var(--muted); margin-top: 5px;
    position: relative;
  }
  .gauge-labels span:nth-child(2) {
    position: absolute; left: 33.33%; transform: translateX(-50%);
  }

  /* ── Graph view ─── */
  #graph-panel { display: none; margin-bottom: 20px; }
  #graph-panel canvas {
    width: 100%; height: 180px; border-radius: 8px;
    background: var(--bg); border: 1px solid var(--border);
  }
  .graph-legend {
    display: flex; gap: 16px; justify-content: center;
    font-size: 0.72rem; color: var(--muted); margin-top: 8px;
  }
  .leg-dot { display: inline-block; width: 10px; height: 10px;
    border-radius: 50%; margin-right: 4px; vertical-align: middle; }

  .section-label {
    font-size: 0.7rem; font-weight: 700; letter-spacing: 0.1em;
    text-transform: uppercase; color: var(--muted);
    margin: 20px 0 12px; display: flex; align-items: center; gap: 8px;
  }
  .section-label::after { content:''; flex:1; height:1px; background:var(--border); }

  .field { margin-bottom: 14px; }
  label {
    display: flex; align-items: baseline; gap: 8px;
    margin-bottom: 6px; font-size: 0.82rem; font-weight: 600;
    letter-spacing: 0.06em; text-transform: uppercase; color: var(--muted);
  }
  .badge {
    font-size: 0.7rem; padding: 1px 7px; border-radius: 99px;
    font-weight: 500; text-transform: none; letter-spacing: 0; color: #fff;
    background: linear-gradient(135deg, var(--accent), var(--accent2));
  }
  .badge.sp  { background: linear-gradient(135deg, #f97316, #ef4444); }
  .badge.lim { background: linear-gradient(135deg, #6b7280, #4b5563); }
  input[type="number"] {
    width: 100%; background: var(--bg); border: 1px solid var(--border);
    border-radius: 8px; color: var(--text); font-size: 1rem;
    padding: 10px 14px; outline: none; transition: border-color 0.15s;
  }
  input[type="number"]:focus {
    border-color: var(--accent);
    box-shadow: 0 0 0 3px rgba(79,142,247,0.15);
  }
  .row { display: flex; gap: 10px; }
  .row .field { flex: 1; }
  button.btn-apply {
    width: 100%; padding: 12px;
    background: linear-gradient(135deg, var(--accent), var(--accent2));
    border: none; border-radius: 8px; color: #fff;
    font-size: 0.95rem; font-weight: 600; cursor: pointer;
    transition: opacity 0.15s, transform 0.1s; margin-top: 6px;
  }
  button.btn-apply:hover  { opacity: 0.9; }
  button.btn-apply:active { transform: scale(0.98); }
  .btn-reset {
    width: 100%; background: #1f2230; border: 1px solid var(--border);
    border-radius: 8px; color: var(--muted); font-size: 0.82rem;
    padding: 9px; margin-top: 8px; cursor: pointer;
    transition: color 0.15s;
  }
  .btn-reset:hover { color: var(--text); }
  .toast {
    margin-top: 14px; padding: 10px 14px; border-radius: 8px;
    font-size: 0.85rem; text-align: center;
    background: rgba(52,211,153,0.12); border: 1px solid rgba(52,211,153,0.3);
    color: var(--success);
  }
  .note { font-size: 0.72rem; color: var(--muted); margin-top: 10px; text-align: center; }
</style>
</head>
<body>
<div class="card">
  <div class="header">
    <div class="dot"></div>
    <div>
      <h1>Pressure PID</h1>
      <div class="subtitle">ESP32 &nbsp;·&nbsp; Normally-open valve &nbsp;·&nbsp; −1 to +4 bar</div>
    </div>
  </div>

  <!-- View toggle -->
  <div class="view-toggle">
    <button class="btn-view active" id="btn-dash" onclick="switchView('dash')">Dashboard</button>
    <button class="btn-view"        id="btn-graph" onclick="switchView('graph')">Graph (60 s)</button>
  </div>

  <!-- Live status bar (always visible) -->
  <div class="status-bar">
    <div class="stat">
      <div class="stat-val" id="lv-pressure">—</div>
      <div class="stat-lbl">Pressure (bar)</div>
    </div>
    <div class="stat">
      <div class="stat-val" id="lv-sp">—</div>
      <div class="stat-lbl">Setpoint (bar)</div>
    </div>
    <div class="stat">
      <div class="stat-val" id="lv-err">—</div>
      <div class="stat-lbl">Error (bar)</div>
    </div>
    <div class="stat">
      <div class="stat-val" id="lv-duty">—</div>
      <div class="stat-lbl">Valve open %</div>
    </div>
  </div>

  <!-- Gauge (-1 to +2 bar display range) -->
  <div class="gauge-wrap">
    <div class="gauge-track">
      <div class="gauge-fill" id="gauge" style="width:0%"></div>
      <div class="gauge-zero"></div>
    </div>
    <div class="gauge-labels">
      <span>−1 bar</span>
      <span>0 bar</span>
      <span style="margin-left:auto">+2 bar</span>
    </div>
  </div>

  <!-- Graph panel (hidden by default) -->
  <div id="graph-panel">
    <canvas id="graph-canvas" width="440" height="180"></canvas>
    <div class="graph-legend">
      <span><span class="leg-dot" style="background:#4f8ef7"></span>Pressure</span>
      <span><span class="leg-dot" style="background:#f97316"></span>Setpoint</span>
    </div>
  </div>

  <!-- Settings form (dashboard view) -->
  <div id="dash-panel">
    <form method="POST" action="/set">
      <div class="section-label">Setpoint</div>
      <div class="field">
        <label>Setpoint <span class="badge sp">bar</span></label>
        <input type="number" step="0.01" min="-1" max="4" name="setpoint" value=")rawhtml";
  html += String(setpoint, 3);
  html += R"rawhtml(">
      </div>

      <div class="section-label">PID Gains</div>
      <div class="row">
        <div class="field">
          <label>Kp <span class="badge">prop</span></label>
          <input type="number" step="any" name="Kp" value=")rawhtml";
  html += String(Kp, 4);
  html += R"rawhtml(">
        </div>
        <div class="field">
          <label>Ki <span class="badge">integ</span></label>
          <input type="number" step="any" name="Ki" value=")rawhtml";
  html += String(Ki, 4);
  html += R"rawhtml(">
        </div>
      </div>
      <div class="field">
        <label>Kd <span class="badge">deriv</span></label>
        <input type="number" step="any" name="Kd" value=")rawhtml";
  html += String(Kd, 4);
  html += R"rawhtml(">
      </div>

      <div class="section-label">Output Clamp <span class="badge lim">PWM counts 0–1023</span></div>
      <div class="row">
        <div class="field">
          <label>Min</label>
          <input type="number" step="1" min="0" max="1023" name="outMin" value=")rawhtml";
  html += String((int)outMin);
  html += R"rawhtml(">
        </div>
        <div class="field">
          <label>Max</label>
          <input type="number" step="1" min="0" max="1023" name="outMax" value=")rawhtml";
  html += String((int)outMax);
  html += R"rawhtml(">
        </div>
      </div>

      <button type="submit" class="btn-apply">Apply</button>
      <button type="submit" formaction="/reset" class="btn-reset">Reset Integral</button>
    </form>
    <p class="note">Normally-open valve: 100% open = no PWM · 0% open = full PWM</p>
  </div>
  )rawhtml";

  if (message.length() > 0)
    html += "<div class=\"toast\">" + message + "</div>";

  html += R"rawhtml(
</div><!-- .card -->

<script>
// ── View switching ────────────────────────────────────────────────────────────
let currentView = 'dash';
function switchView(v) {
  currentView = v;
  document.getElementById('dash-panel').style.display  = (v === 'dash')  ? '' : 'none';
  document.getElementById('graph-panel').style.display = (v === 'graph') ? 'block' : 'none';
  document.getElementById('btn-dash').classList.toggle('active',  v === 'dash');
  document.getElementById('btn-graph').classList.toggle('active', v === 'graph');
  if (v === 'graph') drawGraph();
}

// ── Canvas graph ──────────────────────────────────────────────────────────────
let graphData = { pressure: [], setpoint: [] };

function drawGraph() {
  const canvas = document.getElementById('graph-canvas');
  const ctx    = canvas.getContext('2d');
  const W = canvas.width, H = canvas.height;
  const PAD = { top: 12, right: 10, bottom: 24, left: 38 };
  const pArr = graphData.pressure;
  const sArr = graphData.setpoint;
  const n    = pArr.length;

  ctx.clearRect(0, 0, W, H);

  // Background
  ctx.fillStyle = '#0f1117';
  ctx.fillRect(0, 0, W, H);

  // Y range: -1 to +2 bar
  const yMin = -1.0, yMax = 2.0;
  const ySpan = yMax - yMin;
  const plotW = W - PAD.left - PAD.right;
  const plotH = H - PAD.top  - PAD.bottom;

  function px(i) { return PAD.left + (n > 1 ? (i / (n - 1)) * plotW : plotW / 2); }
  function py(v) { return PAD.top  + (1 - (v - yMin) / ySpan) * plotH; }

  // Grid lines at -1, 0, 0.5, 1, 1.5, 2
  const gridY = [-1, -0.5, 0, 0.5, 1.0, 1.5, 2.0];
  gridY.forEach(g => {
    const y = py(g);
    ctx.beginPath();
    ctx.strokeStyle = g === 0 ? '#3a3d55' : '#232537';
    ctx.lineWidth   = g === 0 ? 1.5 : 1;
    ctx.setLineDash(g === 0 ? [] : [4, 4]);
    ctx.moveTo(PAD.left, y); ctx.lineTo(PAD.left + plotW, y);
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.fillStyle = '#6b7280'; ctx.font = '10px system-ui';
    ctx.textAlign = 'right';
    ctx.fillText(g.toFixed(1), PAD.left - 4, y + 3.5);
  });

  // X axis label
  ctx.fillStyle = '#6b7280'; ctx.font = '9px system-ui'; ctx.textAlign = 'center';
  ctx.fillText('−60 s', PAD.left, H - 4);
  ctx.fillText('now',   PAD.left + plotW, H - 4);

  if (n < 2) return;

  // Setpoint line (orange dashed)
  ctx.beginPath(); ctx.strokeStyle = '#f97316'; ctx.lineWidth = 1.5; ctx.setLineDash([6, 4]);
  sArr.forEach((v, i) => i === 0 ? ctx.moveTo(px(i), py(v)) : ctx.lineTo(px(i), py(v)));
  ctx.stroke(); ctx.setLineDash([]);

  // Pressure line (blue solid)
  ctx.beginPath(); ctx.strokeStyle = '#4f8ef7'; ctx.lineWidth = 2;
  pArr.forEach((v, i) => i === 0 ? ctx.moveTo(px(i), py(v)) : ctx.lineTo(px(i), py(v)));
  ctx.stroke();
}

// ── Live values refresh ───────────────────────────────────────────────────────
function refresh() {
  fetch('/values').then(r => r.json()).then(d => {
    // Pressure: show with sign
    const pEl = document.getElementById('lv-pressure');
    pEl.textContent = (d.pressure >= 0 ? '+' : '') + d.pressure.toFixed(3);
    pEl.className   = 'stat-val' + (d.pressure < 0 ? ' danger' : '');

    document.getElementById('lv-sp').textContent = (d.setpoint >= 0 ? '+' : '') + d.setpoint.toFixed(3);

    const err   = d.setpoint - d.pressure;
    const errEl = document.getElementById('lv-err');
    errEl.textContent = (err >= 0 ? '+' : '') + err.toFixed(3);
    errEl.className   = 'stat-val' + (Math.abs(err) > 0.15 ? ' warn' : '');

    // Valve open %: 100% = wide open, 0% = fully closed
    document.getElementById('lv-duty').textContent = d.valveOpen.toFixed(1) + '%';

    // Gauge: map -1…+2 bar to 0–100% (3 bar span)
    const gaugePct = Math.min(100, Math.max(0, ((d.pressure - (-1.0)) / 3.0) * 100));
    document.getElementById('gauge').style.width = gaugePct.toFixed(1) + '%';
  }).catch(() => {});

  if (currentView === 'graph') {
    fetch('/history').then(r => r.json()).then(d => {
      graphData = d;
      drawGraph();
    }).catch(() => {});
  }
}
setInterval(refresh, 400);
refresh();
</script>
</body>
</html>)rawhtml";

  return html;
}

// ─── Routes ───────────────────────────────────────────────────────────────────
void handleRoot() { server.send(200, "text/html", buildPage()); }

void handleSet() {
  if (server.hasArg("Kp"))       Kp       = server.arg("Kp").toFloat();
  if (server.hasArg("Ki"))       Ki       = server.arg("Ki").toFloat();
  if (server.hasArg("Kd"))       Kd       = server.arg("Kd").toFloat();
  if (server.hasArg("setpoint")) setpoint = server.arg("setpoint").toFloat();
  if (server.hasArg("outMin"))   outMin   = server.arg("outMin").toFloat();
  if (server.hasArg("outMax"))   outMax   = server.arg("outMax").toFloat();

  setpoint = constrain(setpoint, -1.0f, 2.0f);
  outMin   = constrain(outMin,    0.0f, 1023.0f);
  outMax   = constrain(outMax,    0.0f, 1023.0f);
  if (outMin > outMax) outMin = outMax;

  Serial.printf("[Config] Kp=%.3f Ki=%.3f Kd=%.3f SP=%.3fbar out=[%.0f,%.0f]\n",
                Kp, Ki, Kd, setpoint, outMin, outMax);

  server.send(200, "text/html",
    buildPage("✓ Applied — SP=" + String(setpoint, 2) + " bar"
              "  Kp=" + String(Kp, 2) +
              "  Ki=" + String(Ki, 2) +
              "  Kd=" + String(Kd, 2)));
}

void handleReset() {
  integral      = 0.0f;
  lastInput_bar = pressure_bar;
  Serial.println("[PID] Integral reset.");
  server.send(200, "text/html", buildPage("✓ Integral reset."));
}

void handleValues() {
  float valveOpen = (outMax > 0) ? (1.0f - constrain(pidOutput, outMin, outMax) / outMax) * 100.0f : 0.0f;  
    
    // 100% = fully open = wastegate pressure. / 0% = fully closed = max turbo pressure

  String json =
    "{\"pressure\":"  + String(pressure_bar, 4) +
    ",\"setpoint\":"  + String(setpoint,      4) +
    ",\"output\":"    + String(pidOutput,      2) +
    ",\"valveOpen\":" + String(valveOpen,      2) +
    ",\"Kp\":"        + String(Kp, 4) +
    ",\"Ki\":"        + String(Ki, 4) +
    ",\"Kd\":"        + String(Kd, 4) +
    ",\"outMin\":"    + String(outMin, 0) +
    ",\"outMax\":"    + String(outMax, 0) + "}";
  server.send(200, "application/json", json);
}

// ─── Setup ────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(3000);
  while (!Serial && millis() < 5000);

  // Zero-fill history
  memset(pressHist, 0, sizeof(pressHist));
  memset(spHist,    0, sizeof(spHist));

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);   // 0–3.3 V range

  ledcAttach(PIN_PWM, PWM_FREQ, PWM_RES);
  ledcWrite(PIN_PWM, 0);            // valve fully open at boot (safe default)

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID);

  Serial.println("\n=== ESP32 Pressure PID ===");
  Serial.printf("Sensor:   0–3.3V = -1 to +4 bar (raw), ÷ 1.1 adjustment\n");
  Serial.printf("Valve:    Normally open, GPIO%d\n", PIN_PWM);
  Serial.print ("URL:      http://"); Serial.println(WiFi.softAPIP());
  Serial.printf("PID init: Kp=%.1f Ki=%.1f Kd=%.1f SP=%.2fbar\n", Kp, Ki, Kd, setpoint);

  server.on("/",        HTTP_GET,  handleRoot);
  server.on("/set",     HTTP_POST, handleSet);
  server.on("/reset",   HTTP_POST, handleReset);
  server.on("/values",  HTTP_GET,  handleValues);
  server.on("/history", HTTP_GET,  handleHistory);

  server.begin();
  Serial.println("Web server started.\n");

  lastPIDTime  = millis();
  lastHistTime = millis();
}

// ─── Loop ─────────────────────────────────────────────────────────────────────
void loop() {
  server.handleClient();
  computePID();
}
