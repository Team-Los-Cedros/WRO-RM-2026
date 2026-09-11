#!/usr/bin/env python3
"""
Calibrador web exclusivo para las Torres de WRO 2026.

    python3 calibrar_torres.py
    -> abre http://<ip-de-la-pi>:8084 desde tu PC

Permite calibrar de forma independiente:
  1. Torre a Recolectar (Cuerpo amarillo + Corona blanca abierta)
  2. Torre de Destino (Cuerpo amarillo + Tapa negra cerrada)
  3. Geometria (cx_garra, tabla_distancia, ROI)

Modifica exclusivamente la seccion "torres" de config.json, protegiendo
los rangos de artefactos y destinos del museo.
"""

import argparse
import copy
import json
import os
from pathlib import Path
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np

from vision_core import (Camara, Detector, GestorExclusividadCamara,
                         cargar_config, dibujar_overlay, guardar_config)


def _suprimir_warnings_libjpeg():
    try:
        devnull_fd = os.open(os.devnull, os.O_WRONLY)
        os.dup2(devnull_fd, 2)
        os.close(devnull_fd)
    except OSError:
        pass


def _asegurar_seccion_torres(cfg):
    torres = cfg.setdefault("torres", {})
    torres.setdefault("cx_garra", cfg.get("geometria", {}).get("cx_garra", 320))
    torres.setdefault("ex_tolerancia_px", 10)

    rec = torres.setdefault("recolectar", {})
    rec.setdefault("amarillo", [[18, 205, 25, 29, 255, 255]])
    rec.setdefault("blanco_corona", [[0, 0, 80, 179, 115, 255]])
    rec.setdefault("roi_y", [0.08, 0.95])
    rec.setdefault("area_min_px", 300)
    rec.setdefault("relacion_aspecto_max", 3.0)
    rec.setdefault("tabla_distancia", [
        [320, 65], [260, 110], [215, 170], [175, 250], [140, 360]
    ])

    dest = torres.setdefault("destino", {})
    dest.setdefault("amarillo", [[18, 205, 25, 29, 255, 255]])
    dest.setdefault("negro_cabeza", [[0, 0, 0, 179, 255, 60]])
    dest.setdefault("roi_y", [0.05, 0.90])
    dest.setdefault("area_min_px", 250)
    dest.setdefault("relacion_aspecto_max", 3.0)
    dest.setdefault("tabla_distancia", [
        [320, 65], [260, 110], [215, 170], [175, 250], [140, 360]
    ])


estado = {
    "cfg": None,
    "config_path": None,
    "tipo_torre": "recolectar",  # "recolectar" o "destino"
    "capa_mascara": "comb",      # "amarillo", "cabeza", "comb"
    "detector": None,
    "lock": threading.Lock(),
    "det": None,
    "roi_y0": 0,
    "fps": 0.0,
    "jpeg_video": None,
    "jpeg_mask": None,
}


PAGINA = """<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>Calibrar Torres WRO 2026</title>
<style>
body{background:#111827;color:#f3f4f6;font-family:system-ui,-apple-system,sans-serif;margin:16px}
.header{display:flex;justify-content:space-between;align-items:center;margin-bottom:12px;border-bottom:1px solid #374151;padding-bottom:8px}
.tabs{display:flex;gap:8px;margin-bottom:12px}
.tab{padding:8px 16px;border-radius:6px;background:#1f2937;color:#9ca3af;cursor:pointer;font-weight:600;border:1px solid #374151}
.tab.active{background:#2563eb;color:#fff;border-color:#3b82f6}
.fila{display:flex;gap:16px;flex-wrap:wrap;align-items:flex-start}
.col-video{display:flex;gap:12px}
img{width:min(44vw,400px);border:1px solid #374151;border-radius:6px;image-rendering:pixelated;background:#000}
.panel{background:#1f2937;border:1px solid #374151;border-radius:6px;padding:12px;min-width:320px}
.grupo-hsv{background:#111827;border-radius:4px;padding:10px;margin-bottom:10px}
.grupo-hsv h4{margin:0 0 6px 0;font-size:13px;color:#60a5fa}
label{display:block;margin:4px 0;font-size:12px;color:#d1d5db}
input[type=range]{width:180px;vertical-align:middle}
.val{display:inline-block;width:32px;text-align:right;font-variant-numeric:tabular-nums;color:#93c5fd;font-family:monospace}
button{padding:8px 14px;margin-right:8px;border:0;border-radius:6px;background:#2563eb;color:#fff;font-size:13px;font-weight:600;cursor:pointer}
button:hover{background:#1d4ed8}
button.sec{background:#374151;color:#d1d5db}
button.sec:hover{background:#4b5563}
#info{font-family:ui-monospace,monospace;background:#030712;padding:8px;border-radius:4px;margin-top:8px;font-size:12px;color:#4ade80}
.badge{padding:2px 6px;border-radius:4px;font-size:11px;font-weight:bold;margin-left:6px}
.links a{color:#60a5fa;text-decoration:none;font-size:12px;margin-left:8px;background:#1f2937;padding:4px 8px;border-radius:4px;border:1px solid #374151}
</style></head><body>

<div class="header">
  <h3 style="margin:0">🏰 Calibrador de Torres &mdash; WRO 2026</h3>
  <div class="links">
    <a href="http://" + window.location.hostname + ":8080" target="_blank">Stream Robot (8080)</a>
    <a href="http://" + window.location.hostname + ":8081" target="_blank">Artefactos (8081)</a>
    <a href="http://" + window.location.hostname + ":8082" target="_blank">Camara/Luz (8082)</a>
    <a href="http://" + window.location.hostname + ":8083" target="_blank">Destinos (8083)</a>
  </div>
</div>

<div class="tabs">
  <div class="tab" id="tab_rec" onclick="seleccionarTorre('recolectar')">🟡⚪ Torre a Recolectar (Amarillo + Corona Blanca)</div>
  <div class="tab" id="tab_dest" onclick="seleccionarTorre('destino')">🟡⚫ Torre Destino (Amarillo + Tapa Negra)</div>
</div>

<div class="fila">
  <div class="col-video">
    <div>
      <div style="font-size:12px;margin-bottom:4px;color:#9ca3af">Vista Camara + Deteccion</div>
      <img src="/video" id="img_video">
    </div>
    <div>
      <div style="font-size:12px;margin-bottom:4px;color:#9ca3af">
        Mascara: 
        <select id="sel_mascara" onchange="cambiarCapaMascara()" style="background:#111827;color:#fff;border:1px solid #374151;border-radius:4px;padding:2px 4px;font-size:11px">
          <option value="comb">Combinada (Cuerpo + Cabeza)</option>
          <option value="amarillo">Solo Cuerpo Amarillo</option>
          <option value="cabeza">Solo Corona / Cabeza</option>
        </select>
      </div>
      <img src="/mask" id="img_mask">
    </div>
  </div>

  <div class="panel">
    <div class="grupo-hsv">
      <h4>1. CUERPO AMARILLO (Base de la Torre)</h4>
      <div id="sliders_amarillo"></div>
    </div>

    <div class="grupo-hsv">
      <h4 id="titulo_cabeza">2. CORONA / CABEZA SUPERIOR</h4>
      <div id="sliders_cabeza"></div>
    </div>

    <div class="grupo-hsv">
      <h4>3. GEOMETRIA Y CENTRADO</h4>
      <label>Centro Garra (cx): <input type="range" id="cx_garra" min="150" max="490" oninput="cambiarGeometria()"><span class="val" id="val_cx">320</span> px</label>
      <label>Area Minima: <input type="range" id="area_min" min="100" max="1500" step="50" oninput="cambiarGeometria()"><span class="val" id="val_area">300</span> px</label>
      <label>ROI Y Min: <input type="range" id="roi_min" min="0" max="50" oninput="cambiarGeometria()"><span class="val" id="val_roimin">8</span>%</label>
      <label>ROI Y Max: <input type="range" id="roi_max" min="50" max="100" oninput="cambiarGeometria()"><span class="val" id="val_roimax">95</span>%</label>
    </div>

    <button onclick="guardar()">💾 Guardar en config.json</button>
    <button class="sec" onclick="cargar()">🔄 Recargar</button>
    <div id="info">Conectando...</div>
  </div>
</div>

<script>
let cfgTorres = {};
let tipoActual = 'recolectar';
const N_HSV = ['H min', 'S min', 'V min', 'H max', 'S max', 'V max'];
const LIMITES = [179, 255, 255, 179, 255, 255];

async function cargar() {
  let r = await fetch('/cfg');
  cfgTorres = await r.json();
  tipoActual = cfgTorres.tipo || 'recolectar';
  actualizarUI();
}

function actualizarUI() {
  document.getElementById('tab_rec').className = 'tab' + (tipoActual === 'recolectar' ? ' active' : '');
  document.getElementById('tab_dest').className = 'tab' + (tipoActual === 'destino' ? ' active' : '');
  
  let sub = cfgTorres[tipoActual] || {};
  let tituloCabeza = document.getElementById('titulo_cabeza');
  if (tipoActual === 'recolectar') {
    tituloCabeza.innerText = '2. CORONA BLANCA (Almena superior)';
  } else {
    tituloCabeza.innerText = '2. TAPA NEGRA (Cabeza cuadrada de destino)';
  }

  renderSliders('sliders_amarillo', sub.amarillo ? sub.amarillo[0] : [20,140,60,48,255,255], 'amarillo');

  let claveCabeza = tipoActual === 'recolectar' ? 'blanco_corona' : 'negro_cabeza';
  let defaultCabeza = tipoActual === 'recolectar' ? [0,0,90,179,130,255] : [0,0,0,179,255,80];
  renderSliders('sliders_cabeza', sub[claveCabeza] ? sub[claveCabeza][0] : defaultCabeza, 'cabeza');

  let cx = cfgTorres.cx_garra || 320;
  document.getElementById('cx_garra').value = cx;
  document.getElementById('val_cx').innerText = cx;

  let area = sub.area_min_px || 300;
  document.getElementById('area_min').value = area;
  document.getElementById('val_area').innerText = area;

  let roi = sub.roi_y || [0.08, 0.95];
  document.getElementById('roi_min').value = Math.round(roi[0] * 100);
  document.getElementById('val_roimin').innerText = Math.round(roi[0] * 100);
  document.getElementById('roi_max').value = Math.round(roi[1] * 100);
  document.getElementById('val_roimax').innerText = Math.round(roi[1] * 100);
}

function renderSliders(divId, valores, grupo) {
  let h = '';
  for (let i = 0; i < 6; i++) {
    h += `<label>${N_HSV[i]}: <input type="range" min="0" max="${LIMITES[i]}" value="${valores[i]}" ` +
         `oninput="cambiarHSV('${grupo}', ${i}, this.value)"> <span class="val" id="val_${grupo}_${i}">${valores[i]}</span></label>`;
  }
  document.getElementById(divId).innerHTML = h;
}

function cambiarHSV(grupo, idx, val) {
  val = parseInt(val);
  document.getElementById(`val_${grupo}_${idx}`).innerText = val;
  let sub = cfgTorres[tipoActual];
  let clave = grupo === 'amarillo' ? 'amarillo' : (tipoActual === 'recolectar' ? 'blanco_corona' : 'negro_cabeza');
  if (!sub[clave]) sub[clave] = [[0,0,0,179,255,255]];
  sub[clave][0][idx] = val;
  enviarCambios();
}

function cambiarGeometria() {
  let cx = parseInt(document.getElementById('cx_garra').value);
  let area = parseInt(document.getElementById('area_min').value);
  let rmin = parseInt(document.getElementById('roi_min').value) / 100.0;
  let rmax = parseInt(document.getElementById('roi_max').value) / 100.0;

  document.getElementById('val_cx').innerText = cx;
  document.getElementById('val_area').innerText = area;
  document.getElementById('val_roimin').innerText = Math.round(rmin * 100);
  document.getElementById('val_roimax').innerText = Math.round(rmax * 100);

  cfgTorres.cx_garra = cx;
  cfgTorres[tipoActual].area_min_px = area;
  cfgTorres[tipoActual].roi_y = [rmin, rmax];
  enviarCambios();
}

async function seleccionarTorre(tipo) {
  tipoActual = tipo;
  await fetch('/tipo?t=' + tipo);
  actualizarUI();
}

async function cambiarCapaMascara() {
  let capa = document.getElementById('sel_mascara').value;
  await fetch('/capa?c=' + capa);
}

let timeoutEnvio = null;
function enviarCambios() {
  if (timeoutEnvio) clearTimeout(timeoutEnvio);
  timeoutEnvio = setTimeout(async () => {
    await fetch('/set_cfg', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify(cfgTorres)
    });
  }, 120);
}

async function guardar() {
  let r = await fetch('/guardar', {method: 'POST'});
  let res = await r.json();
  alert(res.msg || 'Guardado');
}

async function updateInfo() {
  try {
    let r = await fetch('/info');
    let d = await r.json();
    let l = document.getElementById('info');
    let enc = d.encontrado ? '<span style="color:#4ade80">SI</span>' : '<span style="color:#f87171">NO</span>';
    l.innerHTML = `[${d.fps} fps] Torre: <b>${d.tipo.toUpperCase()}</b> | Encontrada: ${enc} | ` +
                  `Error X: <b>${d.ex > 0 ? '+' : ''}${d.ex} px</b> | Distancia: <b>${d.dist_mm} mm</b> | Base Y: ${d.ey} | Confianza: ${d.confianza}%`;
  } catch(e) {}
  setTimeout(updateInfo, 250);
}

cargar().then(updateInfo);
</script></body></html>
"""


def _hilo_captura(cam, detector):
    fps_calc = 0.0
    t_prev = time.time()
    n_frames = 0

    while True:
        frame = cam.leer(solo_nuevos=True, timeout=0.5)
        if frame is None:
            time.sleep(0.02)
            continue

        with estado["lock"]:
            tipo = estado["tipo_torre"]
            capa = estado["capa_mascara"]
            cx_garra = estado["cfg"].get("torres", {}).get("cx_garra", detector.cx_garra)

        if tipo == "recolectar":
            det, mask_comb, roi_y0 = detector.detectar_torre_recolectar(frame)
            etiqueta = "TORRE_REC"
        else:
            det, mask_comb, roi_y0 = detector.detectar_torre_destino(frame)
            etiqueta = "TORRE_DEST"

        n_frames += 1
        ahora = time.time()
        if ahora - t_prev >= 1.0:
            fps_calc = n_frames / (ahora - t_prev)
            t_prev = ahora
            n_frames = 0

        mask_vis = mask_comb
        if capa != "comb":
            alto, ancho = frame.shape[:2]
            sub = estado["cfg"].get("torres", {}).get(tipo, {})
            y0 = int(alto * sub.get("roi_y", [0.08, 0.95])[0])
            y1 = int(alto * sub.get("roi_y", [0.08, 0.95])[1])
            roi_hsv = cv2.cvtColor(frame[y0:y1], cv2.COLOR_BGR2HSV)
            if capa == "amarillo":
                rangos = sub.get("amarillo", [[22, 140, 60, 48, 255, 255]])
            else:
                clave = "blanco_corona" if tipo == "recolectar" else "negro_cabeza"
                rangos = sub.get(clave, [[0, 0, 90, 179, 130, 255]])
            mask_vis = np.zeros(roi_hsv.shape[:2], dtype=np.uint8)
            for r in rangos:
                mask_vis |= cv2.inRange(roi_hsv, np.array(r[0:3], dtype=np.uint8), np.array(r[3:6], dtype=np.uint8))

        overlay = dibujar_overlay(frame.copy(), det, etiqueta, fps_calc, roi_y0, cx_garra)

        ok_v, buf_v = cv2.imencode(".jpg", overlay, [int(cv2.IMWRITE_JPEG_QUALITY), 65])
        ok_m, buf_m = cv2.imencode(".jpg", mask_vis, [int(cv2.IMWRITE_JPEG_QUALITY), 50])

        with estado["lock"]:
            estado["det"] = det
            estado["fps"] = round(fps_calc, 1)
            estado["roi_y0"] = roi_y0
            if ok_v:
                estado["jpeg_video"] = buf_v.tobytes()
            if ok_m:
                estado["jpeg_mask"] = buf_m.tobytes()


def arrancar_servidor(puerto=8084):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_GET(self):
            parsed = urlparse(self.path)
            path = parsed.path

            if path == "/video":
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                try:
                    while True:
                        with estado["lock"]:
                            jpeg = estado["jpeg_video"]
                        if jpeg is None:
                            time.sleep(0.04)
                            continue
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                         b"Content-Length: %d\r\n\r\n" % len(jpeg))
                        self.wfile.write(jpeg)
                        self.wfile.write(b"\r\n")
                        time.sleep(0.05)
                except (BrokenPipeError, ConnectionResetError):
                    pass

            elif path == "/mask":
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                try:
                    while True:
                        with estado["lock"]:
                            jpeg = estado["jpeg_mask"]
                        if jpeg is None:
                            time.sleep(0.04)
                            continue
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                         b"Content-Length: %d\r\n\r\n" % len(jpeg))
                        self.wfile.write(jpeg)
                        self.wfile.write(b"\r\n")
                        time.sleep(0.05)
                except (BrokenPipeError, ConnectionResetError):
                    pass

            elif path == "/cfg":
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                with estado["lock"]:
                    data = copy.deepcopy(estado["cfg"].get("torres", {}))
                    data["tipo"] = estado["tipo_torre"]
                self.wfile.write(json.dumps(data).encode("utf-8"))

            elif path == "/tipo":
                qs = parse_qs(parsed.query)
                t = qs.get("t", ["recolectar"])[0]
                if t in ("recolectar", "destino"):
                    with estado["lock"]:
                        estado["tipo_torre"] = t
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(b'{"status":"ok"}')

            elif path == "/capa":
                qs = parse_qs(parsed.query)
                c = qs.get("c", ["comb"])[0]
                if c in ("comb", "amarillo", "cabeza"):
                    with estado["lock"]:
                        estado["capa_mascara"] = c
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(b'{"status":"ok"}')

            elif path == "/info":
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                with estado["lock"]:
                    d = estado["det"]
                    resp = {
                        "tipo": estado["tipo_torre"],
                        "fps": estado["fps"],
                        "encontrado": d.encontrado if d else False,
                        "ex": d.ex if d else 0,
                        "ey": d.ey if d else 0,
                        "dist_mm": d.dist_mm if d else -1,
                        "area": d.area if d else 0,
                        "confianza": d.confianza if d else 0
                    }
                self.wfile.write(json.dumps(resp).encode("utf-8"))

            else:
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.end_headers()
                self.wfile.write(PAGINA.encode("utf-8"))

        def do_POST(self):
            parsed = urlparse(self.path)
            if parsed.path == "/set_cfg":
                length = int(self.headers.get("Content-Length", 0))
                body = self.rfile.read(length)
                try:
                    data = json.loads(body.decode("utf-8"))
                    with estado["lock"]:
                        estado["cfg"]["torres"].update(data)
                        estado["detector"].cfg["torres"] = estado["cfg"]["torres"]
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.end_headers()
                    self.wfile.write(b'{"status":"ok"}')
                except Exception as e:
                    self.send_response(400)
                    self.end_headers()
                    self.wfile.write(str(e).encode("utf-8"))

            elif parsed.path == "/guardar":
                with estado["lock"]:
                    try:
                        with open(estado["config_path"], "r", encoding="utf-8") as f:
                            completo = json.load(f)
                        completo["torres"] = copy.deepcopy(estado["cfg"]["torres"])
                        guardar_config(completo, estado["config_path"])
                        resp = {"status": "ok", "msg": "Configuracion de torres guardada en config.json"}
                    except Exception as e:
                        resp = {"status": "error", "msg": f"Error al guardar: {e}"}
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(json.dumps(resp).encode("utf-8"))

    srv = ThreadingHTTPServer(("0.0.0.0", puerto), Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv


def main():
    ap = argparse.ArgumentParser(description="Calibrador web exclusivo para Torres WRO 2026")
    ap.add_argument("--config", default=None, help="Ruta a config.json")
    ap.add_argument("--puerto", type=int, default=8084, help="Puerto HTTP (defecto 8084)")
    args = ap.parse_args()

    _suprimir_warnings_libjpeg()

    ruta_cfg = args.config or str(Path(__file__).resolve().parent / "config.json")
    cfg = cargar_config(ruta_cfg)
    _asegurar_seccion_torres(cfg)

    detector = Detector(cfg)

    estado["cfg"] = cfg
    estado["config_path"] = ruta_cfg
    estado["detector"] = detector

    gestor = GestorExclusividadCamara(nombre_script="calibrar_torres.py", gestionar_servicio=True)
    gestor.adquirir()

    cam = Camara(cfg["camara"]).abrir()
    print(f"[calibrar_torres] Camara abierta {cam.ancho}x{cam.alto}")

    t_captura = threading.Thread(target=_hilo_captura, args=(cam, detector), daemon=True)
    t_captura.start()

    arrancar_servidor(args.puerto)
    print(f"[calibrar_torres] Servidor de calibracion activo en http://<ip-de-la-pi>:{args.puerto}")
    print("[calibrar_torres] Presiona Ctrl+C para salir.")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[calibrar_torres] Cerrando...")
    finally:
        cam.cerrar()
        gestor.liberar()


if __name__ == "__main__":
    main()
