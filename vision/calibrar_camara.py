#!/usr/bin/env python3
"""
Calibrador y Ajuste Web de Camara para Raspberry Pi 3B (WRO RoboMission 2026).

Permite ajustar en tiempo real la exposicion, ganancia, balance de blancos,
brillo, contraste y saturacion de la camara USB para evitar imagenes oscuras
y garantizar una deteccion precisa de colores en pista.

Uso:
    python3 calibrar_camara.py
    -> abre http://<ip-de-la-pi>:8082 desde el navegador
"""

import argparse
import json
import os
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np

from vision_core import Camara, GestorExclusividadCamara, cargar_config, guardar_config

PAGINA_HTML = r"""<!DOCTYPE html>
<html lang="es">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Calibrador de Camara - WRO 2026</title>
<style>
:root {
  --bg: #101216;
  --card: #181b22;
  --border: #282c37;
  --text: #e6edf3;
  --muted: #8b949e;
  --primary: #2f81f7;
  --primary-hover: #388bfd;
  --accent: #238636;
  --accent-hover: #2ea043;
  --warn: #d29922;
  --danger: #f85149;
}
* { box-sizing: border-box; margin: 0; padding: 0; }
body {
  background: var(--bg);
  color: var(--text);
  font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
  padding: 14px;
}
header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  flex-wrap: wrap;
  gap: 10px;
  margin-bottom: 14px;
  padding-bottom: 10px;
  border-bottom: 1px solid var(--border);
}
h2 { font-size: 1.25rem; font-weight: 600; }
.badge {
  background: #1f242c;
  padding: 4px 10px;
  border-radius: 12px;
  font-size: 12px;
  color: var(--muted);
  border: 1px solid var(--border);
}
.grid {
  display: grid;
  grid-template-columns: minmax(320px, 480px) 1fr;
  gap: 16px;
  align-items: start;
}
@media (max-width: 860px) {
  .grid { grid-template-columns: 1fr; }
}
.card {
  background: var(--card);
  border: 1px solid var(--border);
  border-radius: 8px;
  padding: 14px;
}
.video-box {
  display: flex;
  flex-direction: column;
  gap: 8px;
}
.video-wrap {
  position: relative;
  background: #000;
  border-radius: 6px;
  overflow: hidden;
  border: 1px solid var(--border);
  display: flex;
  align-items: center;
  justify-content: center;
}
.video-wrap img {
  width: 100%;
  height: auto;
  display: block;
  image-rendering: pixelated;
}
.telemetria {
  display: grid;
  grid-template-columns: repeat(3, 1fr);
  gap: 8px;
  text-align: center;
}
.stat-pill {
  background: #11141a;
  border: 1px solid var(--border);
  padding: 6px;
  border-radius: 6px;
}
.stat-val { font-size: 1.1rem; font-weight: 700; color: #58a6ff; font-variant-numeric: tabular-nums; }
.stat-lbl { font-size: 11px; color: var(--muted); text-transform: uppercase; margin-top: 2px; }

.section-title {
  font-size: 13px;
  text-transform: uppercase;
  letter-spacing: 0.5px;
  color: var(--muted);
  margin: 12px 0 8px 0;
  display: flex;
  align-items: center;
  gap: 6px;
}
.section-title:first-child { margin-top: 0; }

.control-row {
  margin-bottom: 10px;
}
.control-header {
  display: flex;
  justify-content: space-between;
  font-size: 13px;
  margin-bottom: 4px;
}
.control-val {
  font-weight: 600;
  color: #79c0ff;
  font-variant-numeric: tabular-nums;
}
input[type=range] {
  width: 100%;
  height: 6px;
  background: #252a34;
  border-radius: 3px;
  outline: none;
  -webkit-appearance: none;
  cursor: pointer;
}
input[type=range]::-webkit-slider-thumb {
  -webkit-appearance: none;
  width: 16px;
  height: 16px;
  border-radius: 50%;
  background: var(--primary);
  cursor: pointer;
}
input[type=range]:disabled {
  opacity: 0.35;
  cursor: not-allowed;
}

.toggle-group {
  display: flex;
  gap: 8px;
  align-items: center;
  margin-bottom: 10px;
}
.btn {
  padding: 7px 12px;
  border: 1px solid var(--border);
  border-radius: 6px;
  background: #21262d;
  color: var(--text);
  font-size: 13px;
  cursor: pointer;
  display: inline-flex;
  align-items: center;
  gap: 6px;
  transition: 0.15s ease;
}
.btn:hover { background: #30363d; }
.btn-primary { background: var(--primary); border-color: var(--primary); color: #fff; }
.btn-primary:hover { background: var(--primary-hover); }
.btn-accent { background: var(--accent); border-color: var(--accent); color: #fff; }
.btn-accent:hover { background: var(--accent-hover); }
.btn-warn { background: var(--warn); border-color: var(--warn); color: #000; font-weight: 600; }
.btn-sm { padding: 4px 8px; font-size: 12px; }

.presets-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(130px, 1fr));
  gap: 6px;
  margin-bottom: 12px;
}
.actions-bar {
  display: flex;
  gap: 8px;
  flex-wrap: wrap;
  margin-top: 14px;
  padding-top: 12px;
  border-top: 1px solid var(--border);
}
.toast {
  position: fixed;
  bottom: 20px;
  right: 20px;
  background: #238636;
  color: #fff;
  padding: 10px 18px;
  border-radius: 6px;
  box-shadow: 0 4px 12px rgba(0,0,0,0.4);
  font-size: 13px;
  opacity: 0;
  transform: translateY(10px);
  transition: 0.25s ease;
  pointer-events: none;
  z-index: 100;
}
.toast.show { opacity: 1; transform: translateY(0); }
.toast.warn { background: var(--warn); color: #000; }
.toast.err { background: var(--danger); color: #fff; }

.switch {
  position: relative;
  display: inline-block;
  width: 38px;
  height: 20px;
}
.switch input { opacity: 0; width: 0; height: 0; }
.slider-round {
  position: absolute;
  cursor: pointer;
  top: 0; left: 0; right: 0; bottom: 0;
  background-color: #2b313c;
  transition: .2s;
  border-radius: 20px;
}
.slider-round:before {
  position: absolute;
  content: "";
  height: 14px;
  width: 14px;
  left: 3px;
  bottom: 3px;
  background-color: white;
  transition: .2s;
  border-radius: 50%;
}
input:checked + .slider-round { background-color: var(--primary); }
input:checked + .slider-round:before { transform: translateX(18px); }

select {
  padding: 6px 10px;
  background: #1f242c;
  color: #eee;
  border: 1px solid var(--border);
  border-radius: 6px;
  font-size: 13px;
}
</style>
</head>
<body>

<header>
  <div>
    <h2>Ajuste de Camara &mdash; WRO 2026</h2>
    <div style="font-size: 12px; color: var(--muted); margin-top: 2px;">Optimizado para Raspberry Pi 3B &bull; Control V4L2 en tiempo real</div>
  </div>
  <div style="display: flex; gap: 8px; align-items: center;">
    <a href="http://" id="link-color" class="btn btn-sm" target="_blank" style="text-decoration: none;">🎨 Calibrar Color (HSV)</a>
    <span class="badge" id="badge-status">Conectado</span>
  </div>
</header>

<div class="grid">
  <!-- Columna Video & Telemetria -->
  <div class="video-box">
    <div class="card" style="padding: 8px;">
      <div class="video-wrap">
        <img id="stream" src="/video" alt="Video en vivo">
      </div>
    </div>

    <div class="telemetria">
      <div class="stat-pill">
        <div class="stat-val" id="stat-fps">--</div>
        <div class="stat-lbl">FPS Reales</div>
      </div>
      <div class="stat-pill">
        <div class="stat-val" id="stat-luma">--</div>
        <div class="stat-lbl">Luminancia (0-255)</div>
      </div>
      <div class="stat-pill">
        <div class="stat-val" id="stat-eval">--</div>
        <div class="stat-lbl">Exposicion</div>
      </div>
    </div>

    <!-- Presets y Ajuste Automatico Inteligente -->
    <div class="card">
      <div class="section-title">⚡ Ajuste Inteligente & Presets</div>
      <div style="display: flex; gap: 8px; margin-bottom: 10px;">
        <button class="btn btn-accent" style="flex: 1;" onclick="autoCalibrar()" id="btn-autocal">
          🪄 Auto-Medir y Fijar
        </button>
        <button class="btn" onclick="toggleAutoContinuo()" id="btn-autocont">
          🔄 Auto Continuo
        </button>
      </div>
      <div class="presets-grid">
        <button class="btn btn-sm" onclick="aplicarPreset('wro_optimo')">🎯 WRO Estandar</button>
        <button class="btn btn-sm" onclick="aplicarPreset('luz_tenue')">🌙 Pista Tenue</button>
        <button class="btn btn-sm" onclick="aplicarPreset('luz_fuerte')">☀️ Luz Intensa</button>
        <button class="btn btn-sm" onclick="aplicarPreset('fabrica')">⚙️ Fabrica</button>
      </div>
    </div>
  </div>

  <!-- Columna Controles Manuales -->
  <div class="card">
    <div class="section-title">📷 Exposicion & Ganancia</div>
    
    <div class="toggle-group" style="justify-content: space-between;">
      <span style="font-size: 13px;">Auto Exposicion de Hardware</span>
      <label class="switch">
        <input type="checkbox" id="chk-auto-exp" onchange="cambiarToggle('auto_exposicion', this.checked)">
        <span class="slider-round"></span>
      </label>
    </div>

    <div class="control-row" id="row-exp">
      <div class="control-header">
        <span>Tiempo de Exposicion (Shutter)</span>
        <span class="control-val" id="val-exp">--</span>
      </div>
      <input type="range" id="rng-exp" min="10" max="2000" step="10" oninput="cambiarSlider('exposicion_absoluta', this.value, 'val-exp')">
    </div>

    <div class="control-row">
      <div class="control-header">
        <span>Ganancia Sensor (Gain / ISO)</span>
        <span class="control-val" id="val-gain">--</span>
      </div>
      <input type="range" id="rng-gain" min="0" max="63" step="1" oninput="cambiarSlider('ganancia', this.value, 'val-gain')">
    </div>

    <div class="section-title">🌡️ Color & Balance de Blancos</div>

    <div class="toggle-group" style="justify-content: space-between;">
      <span style="font-size: 13px;">Auto Balance de Blancos</span>
      <label class="switch">
        <input type="checkbox" id="chk-auto-wb" onchange="cambiarToggle('auto_balance_blancos', this.checked)">
        <span class="slider-round"></span>
      </label>
    </div>

    <div class="control-row" id="row-wb">
      <div class="control-header">
        <span>Temperatura de Color (Kelvin)</span>
        <span class="control-val" id="val-wb">--</span>
      </div>
      <input type="range" id="rng-wb" min="2800" max="6500" step="50" oninput="cambiarSlider('temperatura_color', this.value, 'val-wb', 'K')">
    </div>

    <div class="section-title">🎨 Procesamiento de Imagen</div>

    <div class="control-row">
      <div class="control-header">
        <span>Brillo</span>
        <span class="control-val" id="val-bri">--</span>
      </div>
      <input type="range" id="rng-bri" min="-64" max="64" step="1" oninput="cambiarSlider('brillo', this.value, 'val-bri')">
    </div>

    <div class="control-row">
      <div class="control-header">
        <span>Contraste</span>
        <span class="control-val" id="val-con">--</span>
      </div>
      <input type="range" id="rng-con" min="0" max="63" step="1" oninput="cambiarSlider('contraste', this.value, 'val-con')">
    </div>

    <div class="control-row">
      <div class="control-header">
        <span>Saturacion</span>
        <span class="control-val" id="val-sat">--</span>
      </div>
      <input type="range" id="rng-sat" min="0" max="128" step="1" oninput="cambiarSlider('saturacion', this.value, 'val-sat')">
    </div>

    <div class="control-row">
      <div class="control-header">
        <span>Gamma</span>
        <span class="control-val" id="val-gam">--</span>
      </div>
      <input type="range" id="rng-gam" min="50" max="300" step="5" oninput="cambiarSlider('gamma', this.value, 'val-gam')">
    </div>

    <div class="control-row">
      <div class="control-header">
        <span>Nitidez (Sharpness)</span>
        <span class="control-val" id="val-shp">--</span>
      </div>
      <input type="range" id="rng-shp" min="0" max="31" step="1" oninput="cambiarSlider('sharpness', this.value, 'val-shp')">
    </div>

    <div class="section-title">⚙️ Configuracion Avanzada</div>
    <div style="display: grid; grid-template-columns: 1fr 1fr; gap: 10px; margin-bottom: 8px;">
      <div>
        <div class="control-header"><span>Anti-parpadeo Red</span></div>
        <select id="sel-freq" onchange="cambiarSelect('frecuencia_red_hz', this.value)" style="width: 100%;">
          <option value="60">60 Hz (America)</option>
          <option value="50">50 Hz (Europa/Asia)</option>
          <option value="0">Desactivado</option>
        </select>
      </div>
      <div style="display: flex; flex-direction: column; justify-content: flex-end;">
        <div class="toggle-group" style="margin: 0; justify-content: space-between;">
          <span style="font-size: 13px;">Giro 180°</span>
          <label class="switch">
            <input type="checkbox" id="chk-flip" onchange="cambiarToggle('voltear_180', this.checked)">
            <span class="slider-round"></span>
          </label>
        </div>
      </div>
    </div>

    <!-- Barra de acciones -->
    <div class="actions-bar">
      <button class="btn btn-primary" style="flex: 1;" onclick="guardarConfig()">
        💾 Guardar en config.json
      </button>
      <button class="btn" onclick="cargarEstado()">
        🔄 Recargar
      </button>
    </div>
  </div>
</div>

<div id="toast" class="toast">Guardado correctamente</div>

<script>
let cfg = {};
let timerDebounce = {};

function showToast(msg, tipo = '') {
  const t = document.getElementById('toast');
  t.textContent = msg;
  t.className = 'toast show ' + tipo;
  setTimeout(() => { t.className = 'toast'; }, 2800);
}

// Configurar enlace hacia el calibrador de color
document.getElementById('link-color').href = 'http://' + window.location.hostname + ':8081';

async function cargarEstado() {
  try {
    const res = await fetch('/estado');
    const data = await res.json();
    cfg = data.camara;
    actualizarUI();
    showToast('Configuracion cargada');
  } catch(e) {
    showToast('Error al conectar con la Pi', 'err');
  }
}

function actualizarUI() {
  document.getElementById('chk-auto-exp').checked = !!cfg.auto_exposicion;
  document.getElementById('rng-exp').disabled = !!cfg.auto_exposicion;
  document.getElementById('rng-exp').value = cfg.exposicion_absoluta || 500;
  document.getElementById('val-exp').textContent = cfg.exposicion_absoluta || 500;

  document.getElementById('rng-gain').value = cfg.ganancia ?? 0;
  document.getElementById('val-gain').textContent = cfg.ganancia ?? 0;

  document.getElementById('chk-auto-wb').checked = !!cfg.auto_balance_blancos;
  document.getElementById('rng-wb').disabled = !!cfg.auto_balance_blancos;
  document.getElementById('rng-wb').value = cfg.temperatura_color || 4500;
  document.getElementById('val-wb').textContent = (cfg.temperatura_color || 4500) + ' K';

  document.getElementById('rng-bri').value = cfg.brillo ?? 8;
  document.getElementById('val-bri').textContent = cfg.brillo ?? 8;

  document.getElementById('rng-con').value = cfg.contraste ?? 33;
  document.getElementById('val-con').textContent = cfg.contraste ?? 33;

  document.getElementById('rng-sat').value = cfg.saturacion ?? 64;
  document.getElementById('val-sat').textContent = cfg.saturacion ?? 64;

  document.getElementById('rng-gam').value = cfg.gamma ?? 100;
  document.getElementById('val-gam').textContent = cfg.gamma ?? 100;

  document.getElementById('rng-shp').value = cfg.sharpness ?? 2;
  document.getElementById('val-shp').textContent = cfg.sharpness ?? 2;

  document.getElementById('sel-freq').value = String(cfg.frecuencia_red_hz ?? 60);
  document.getElementById('chk-flip').checked = !!cfg.voltear_180;
}

function cambiarSlider(clave, val, idVal, sufijo = '') {
  const numVal = parseInt(val);
  cfg[clave] = numVal;
  document.getElementById(idVal).textContent = numVal + (sufijo ? ' ' + sufijo : '');

  clearTimeout(timerDebounce[clave]);
  timerDebounce[clave] = setTimeout(() => {
    enviarControl(clave, numVal);
  }, 40);
}

function cambiarToggle(clave, val) {
  cfg[clave] = val;
  if (clave === 'auto_exposicion') {
    document.getElementById('rng-exp').disabled = val;
  }
  if (clave === 'auto_balance_blancos') {
    document.getElementById('rng-wb').disabled = val;
  }
  enviarControl(clave, val);
}

function cambiarSelect(clave, val) {
  const numVal = parseInt(val);
  cfg[clave] = numVal;
  enviarControl(clave, numVal);
}

async function enviarControl(clave, valor) {
  try {
    await fetch('/set_control', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ clave: clave, valor: valor })
    });
  } catch(e) {
    console.error('Error al enviar control:', e);
  }
}

async function autoCalibrar() {
  const btn = document.getElementById('btn-autocal');
  btn.disabled = true;
  btn.textContent = '⏳ Calibrando luz...';
  showToast('Midiendo luz de la pista...', 'warn');

  try {
    const res = await fetch('/auto_ajustar', { method: 'POST' });
    const data = await res.json();
    if (data.ok) {
      cfg = data.camara;
      actualizarUI();
      showToast('Luz calibrada y fijada con exito: Exp=' + cfg.exposicion_absoluta + ', WB=' + cfg.temperatura_color + 'K');
    } else {
      showToast('Error en calibracion automatica', 'err');
    }
  } catch(e) {
    showToast('Fallo en la peticion', 'err');
  } finally {
    btn.disabled = false;
    btn.textContent = '🪄 Auto-Medir y Fijar';
  }
}

async function toggleAutoContinuo() {
  const nuevo = !cfg.auto_exposicion;
  cfg.auto_exposicion = nuevo;
  cfg.auto_balance_blancos = nuevo;
  actualizarUI();
  await enviarControl('auto_exposicion', nuevo);
  await enviarControl('auto_balance_blancos', nuevo);
  showToast(nuevo ? 'Auto continuo activado' : 'Modo manual fijado');
}

async function aplicarPreset(nombre) {
  try {
    const res = await fetch('/preset?nombre=' + nombre, { method: 'POST' });
    const data = await res.json();
    if (data.ok) {
      cfg = data.camara;
      actualizarUI();
      showToast('Preset "' + nombre + '" aplicado');
    }
  } catch(e) {
    showToast('Error al aplicar preset', 'err');
  }
}

async function guardarConfig() {
  try {
    const res = await fetch('/guardar', { method: 'POST' });
    const txt = await res.text();
    showToast('💾 ' + txt);
  } catch(e) {
    showToast('Error al guardar en disco', 'err');
  }
}

async function actualizarTelemetria() {
  try {
    const res = await fetch('/telemetria');
    const d = await res.json();
    document.getElementById('stat-fps').textContent = d.fps.toFixed(1);
    document.getElementById('stat-luma').textContent = d.luma.toFixed(0);

    const elEval = document.getElementById('stat-eval');
    if (d.luma < 30) {
      elEval.textContent = 'Oscura ⚠️';
      elEval.style.color = '#f85149';
    } else if (d.luma > 180) {
      elEval.textContent = 'Saturada ⚠️';
      elEval.style.color = '#d29922';
    } else {
      elEval.textContent = 'Optima ✅';
      elEval.style.color = '#3fb950';
    }
  } catch(e) {}
}

cargarEstado();
setInterval(actualizarTelemetria, 400);
</script>
</body>
</html>
"""

class EstadoServidor:
    def __init__(self, cfg):
        self.cfg = cfg
        self.lock = threading.Lock()
        self.cam = None
        self.frame = None
        self.fps = 0.0
        self.luma = 0.0
        self.corriendo = True


def bucle_captura(estado):
    cam = Camara(estado.cfg["camara"]).abrir()
    estado.cam = cam
    primer_frame = cam.leer(timeout=1.0)
    if primer_frame is not None:
        with estado.lock:
            estado.frame = primer_frame
            estado.luma = float(np.mean(cv2.cvtColor(primer_frame, cv2.COLOR_BGR2GRAY)))
            estado.fps = 20.0

    t_ant = time.time()
    fps = 20.0

    try:
        while estado.corriendo:
            frame = cam.leer(solo_nuevos=True, timeout=0.5)
            if frame is None:
                continue

            ahora = time.time()
            fps = 0.85 * fps + 0.15 / max(ahora - t_ant, 1e-6)
            t_ant = ahora

            # Medicion de luminancia media (escala de grises)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            luma = float(np.mean(gray))

            with estado.lock:
                estado.frame = frame
                estado.fps = fps
                estado.luma = luma
    finally:
        cam.cerrar()


def _jpeg(img, calidad=65):
    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), calidad])
    return buf.tobytes() if ok else None


def crear_handler(estado, ruta_config):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _resp_json(self, obj, code=200):
            b = json.dumps(obj).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(b)))
            self.end_headers()
            self.wfile.write(b)

        def _resp_txt(self, txt, code=200):
            b = txt.encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "text/plain; charset=utf-8")
            self.send_header("Content-Length", str(len(b)))
            self.end_headers()
            self.wfile.write(b)

        def do_GET(self):
            u = urlparse(self.path)

            if u.path == "/":
                b = PAGINA_HTML.encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(b)))
                self.end_headers()
                self.wfile.write(b)
                return

            if u.path == "/video":
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                try:
                    while estado.corriendo:
                        with estado.lock:
                            f = estado.frame
                        if f is None:
                            time.sleep(0.04)
                            continue
                        j = _jpeg(f)
                        if j:
                            self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(j))
                            self.wfile.write(j)
                            self.wfile.write(b"\r\n")
                        time.sleep(0.06)  # ~16 FPS al navegador
                except (BrokenPipeError, ConnectionResetError):
                    pass
                return

            if u.path == "/estado":
                with estado.lock:
                    datos = {
                        "camara": estado.cfg["camara"],
                        "fps": estado.fps,
                        "luma": estado.luma
                    }
                return self._resp_json(datos)

            if u.path == "/telemetria":
                with estado.lock:
                    datos = {
                        "fps": estado.fps,
                        "luma": estado.luma
                    }
                return self._resp_json(datos)

            self.send_response(404)
            self.end_headers()

        def do_POST(self):
            u = urlparse(self.path)

            if u.path == "/set_control":
                length = int(self.headers.get("Content-Length", 0))
                body = self.rfile.read(length)
                try:
                    req = json.loads(body.decode("utf-8"))
                    clave = req.get("clave")
                    valor = req.get("valor")
                    with estado.lock:
                        estado.cfg["camara"][clave] = valor
                        if estado.cam:
                            estado.cam.aplicar_control(clave, valor)
                    return self._resp_json({"ok": True})
                except Exception as e:
                    return self._resp_json({"ok": False, "error": str(e)}, 400)

            if u.path == "/auto_ajustar":
                # Rutina inteligente de auto-calibracion en lazo cerrado
                try:
                    with estado.lock:
                        if estado.cam:
                            estado.cam.aplicar_control("auto_exposicion", False)
                            estado.cam.aplicar_control("auto_balance_blancos", False)

                    target_luma = 115.0
                    exp = 400
                    gain = 5
                    with estado.lock:
                        if estado.cam:
                            estado.cam.aplicar_control("exposicion_absoluta", exp)
                            estado.cam.aplicar_control("ganancia", gain)

                    time.sleep(0.2)

                    # Bucle de convergencia hacia luminancia optima (110 - 125)
                    for step in range(10):
                        time.sleep(0.12)
                        with estado.lock:
                            current_luma = estado.luma
                            f = estado.frame

                        if abs(current_luma - target_luma) <= 8:
                            break

                        ratio = target_luma / max(current_luma, 5.0)
                        if current_luma < target_luma:
                            if exp < 800:
                                exp = int(min(exp * ratio, 800))
                            else:
                                gain = int(min(gain + (ratio - 1.0) * 12 + 3, 40))
                                if gain >= 35:
                                    exp = int(min(exp * 1.2, 1800))
                        else:
                            if gain > 0:
                                gain = max(0, int(gain - 5))
                            else:
                                exp = int(max(exp * ratio, 50))

                        with estado.lock:
                            if estado.cam:
                                estado.cam.aplicar_control("exposicion_absoluta", exp)
                                estado.cam.aplicar_control("ganancia", gain)

                    # Auto balance de color basado en canales BGR
                    temp_color = 4500
                    with estado.lock:
                        if estado.frame is not None:
                            b_mean = float(np.mean(estado.frame[:, :, 0]))
                            r_mean = float(np.mean(estado.frame[:, :, 2]))
                            if b_mean > r_mean + 15:
                                temp_color = 5200
                            elif r_mean > b_mean + 15:
                                temp_color = 3800

                        c = estado.cfg["camara"]
                        c["auto_exposicion"] = False
                        c["exposicion_absoluta"] = int(exp)
                        c["auto_balance_blancos"] = False
                        c["temperatura_color"] = int(temp_color)
                        c["ganancia"] = int(gain)
                        if estado.cam:
                            estado.cam.aplicar_control("temperatura_color", temp_color)

                    return self._resp_json({"ok": True, "camara": estado.cfg["camara"]})
                except Exception as e:
                    return self._resp_json({"ok": False, "error": str(e)}, 500)

            if u.path == "/preset":
                q = parse_qs(u.query)
                nombre = q.get("nombre", [""])[0]
                presets = {
                    "wro_optimo": {
                        "auto_exposicion": False,
                        "exposicion_absoluta": 750,
                        "ganancia": 25,
                        "auto_balance_blancos": False,
                        "temperatura_color": 4500,
                        "brillo": 12,
                        "contraste": 33,
                        "saturacion": 68,
                        "gamma": 100,
                        "sharpness": 2
                    },
                    "luz_tenue": {
                        "auto_exposicion": False,
                        "exposicion_absoluta": 1100,
                        "ganancia": 35,
                        "auto_balance_blancos": False,
                        "temperatura_color": 4500,
                        "brillo": 15,
                        "contraste": 35,
                        "saturacion": 70,
                        "gamma": 110,
                        "sharpness": 3
                    },
                    "luz_fuerte": {
                        "auto_exposicion": False,
                        "exposicion_absoluta": 250,
                        "ganancia": 0,
                        "auto_balance_blancos": False,
                        "temperatura_color": 4500,
                        "brillo": 0,
                        "contraste": 30,
                        "saturacion": 60,
                        "gamma": 90,
                        "sharpness": 2
                    },
                    "fabrica": {
                        "auto_exposicion": True,
                        "exposicion_absoluta": 313,
                        "ganancia": 0,
                        "auto_balance_blancos": True,
                        "temperatura_color": 4500,
                        "brillo": 8,
                        "contraste": 33,
                        "saturacion": 56,
                        "gamma": 100,
                        "sharpness": 2
                    }
                }
                p = presets.get(nombre)
                if p:
                    with estado.lock:
                        estado.cfg["camara"].update(p)
                        if estado.cam:
                            estado.cam.actualizar_cfg(estado.cfg["camara"])
                    return self._resp_json({"ok": True, "camara": estado.cfg["camara"]})
                return self._resp_json({"ok": False, "error": "Preset desconocido"}, 400)

            if u.path == "/guardar":
                with estado.lock:
                    guardar_config(estado.cfg, ruta_config)
                return self._resp_txt("Guardado correctamente en config.json")

            self.send_response(404)
            self.end_headers()

    return Handler


def main():
    import signal
    parser = argparse.ArgumentParser(description="Calibrador web de camara para WRO")
    parser.add_argument("--config", default=None, help="Ruta al archivo config.json")
    parser.add_argument("--puerto", type=int, default=8082, help="Puerto HTTP (defecto: 8082)")
    args = parser.parse_args()

    cfg = cargar_config(args.config)
    estado = EstadoServidor(cfg)

    gestor = GestorExclusividadCamara(nombre_script="calibrar_camara.py", gestionar_servicio=True)
    gestor.adquirir()

    hilo_captura = threading.Thread(target=bucle_captura, args=(estado,), daemon=True)
    hilo_captura.start()

    handler_class = crear_handler(estado, args.config)
    servidor = ThreadingHTTPServer(("0.0.0.0", args.puerto), handler_class)

    def signal_handler(sig, frame):
        print("\nDeteniendo servidor...", flush=True)
        estado.corriendo = False
        try:
            servidor.server_close()
        except Exception:
            pass
        gestor.liberar()
        os._exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    print("==========================================================")
    print("🎥 Calibrador de Camara WRO iniciado")
    print("👉 Abre en tu navegador: http://<ip-de-la-pi>:%d" % args.puerto)
    print("==========================================================", flush=True)

    try:
        servidor.serve_forever()
    except Exception:
        pass
    finally:
        estado.corriendo = False
        gestor.liberar()
        os._exit(0)


if __name__ == "__main__":
    main()
