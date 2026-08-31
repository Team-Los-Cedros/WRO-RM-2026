#!/usr/bin/env python3
"""
Calibrador de color por navegador. La Pi no necesita monitor.

    python3 calibrar_web.py
    -> abre http://<ip-de-la-pi>:8081 desde tu PC

Muestra a la vez la imagen real y la mascara del color, con sliders HSV.
Cuando la mascara marque SOLO el objeto (y en toda la pista, no solo en un
punto), pulsa "Guardar en config.json".

Tambien sirve para calibrar la geometria:
  - cx_garra: pon un objeto justo delante de la garra y ajusta hasta que la
    linea amarilla pase por su centro.
  - tabla_distancia: coloca el objeto a distancias medidas con regla (60, 100,
    150, 220, 300, 420 mm desde la garra) y anota el valor "ey" que aparece.
    Esos pares [ey, mm] son la tabla.
"""

import argparse
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import cv2

from vision_core import (Camara, Detector, cargar_config, colores_disponibles,
                         dibujar_overlay, guardar_config)

estado = {
    "cfg": None,
    "color": None,
    "detector": None,
    "lock": threading.Lock(),
    "frame": None,
    "det": None,
    "roi_y0": 0,
}

PAGINA = """<html><head><meta charset="utf-8"><title>Calibrar WRO</title>
<style>
body{background:#15171c;color:#e8e8e8;font-family:system-ui,sans-serif;margin:12px}
.fila{display:flex;gap:12px;flex-wrap:wrap;align-items:flex-start}
img{width:min(46vw,420px);border:1px solid #333;image-rendering:pixelated;background:#000}
label{display:block;margin:6px 0;font-size:13px}
input[type=range]{width:260px;vertical-align:middle}
.val{display:inline-block;width:38px;text-align:right;font-variant-numeric:tabular-nums}
button{padding:8px 14px;margin-right:8px;border:0;border-radius:6px;background:#3a6df0;color:#fff;font-size:14px;cursor:pointer}
button.sec{background:#444}
select{padding:6px;background:#222;color:#eee;border:1px solid #444;border-radius:4px}
#info{font-family:ui-monospace,monospace;background:#0c0e12;padding:8px;border-radius:6px;margin-top:8px;font-size:13px}
h4{margin:14px 0 4px}
</style></head><body>
<h3>Calibracion de color &mdash; WRO RoboMission
  <a href="javascript:void(0)" onclick="window.open('http://' + window.location.hostname + ':8082', '_blank')"
     style="font-size:13px; font-weight:normal; margin-left:14px; color:#58a6ff; text-decoration:none; background:#21262d; padding:4px 10px; border-radius:4px; border:1px solid #38404d;">
     📷 Ajustar Camara (Exposicion / Luz)
  </a>
</h3>
<div class="fila">
  <div><div>camara</div><img src="/video"></div>
  <div><div>mascara</div><img src="/mask"></div>
  <div>
    <label>Color: <select id="color" onchange="cambiarColor()"></select></label>
    <div id="sliders"></div>
    <h4>&nbsp;</h4>
    <button onclick="guardar()">Guardar en config.json</button>
    <button class="sec" onclick="cargar()">Recargar</button>
    <div id="info">...</div>
  </div>
</div>
<script>
let rangos = [];
const N = ['H min','S min','V min','H max','S max','V max'];
const MAX = [179,255,255,179,255,255];

async function cargar(){
  const r = await (await fetch('/estado')).json();
  const sel = document.getElementById('color');
  if(sel.options.length === 0){
    r.colores.forEach(c => sel.add(new Option(c,c)));
  }
  sel.value = r.color;
  rangos = r.rangos;
  pintarSliders();
}

function pintarSliders(){
  const cont = document.getElementById('sliders');
  cont.innerHTML = '';
  rangos.forEach((rango,i) => {
    const t = document.createElement('h4');
    t.textContent = 'Rango ' + (i+1) + (rangos.length>1 ? ' (el rojo usa 2)' : '');
    cont.appendChild(t);
    rango.forEach((v,j) => {
      const lab = document.createElement('label');
      lab.innerHTML = N[j] + ' <input type="range" min="0" max="' + MAX[j] +
        '" value="' + v + '" oninput="cambiar(' + i + ',' + j + ',this.value)">' +
        '<span class="val" id="v' + i + '_' + j + '">' + v + '</span>';
      cont.appendChild(lab);
    });
  });
}

async function cambiar(i,j,v){
  rangos[i][j] = parseInt(v);
  document.getElementById('v'+i+'_'+j).textContent = v;
  await fetch('/set?rangos=' + encodeURIComponent(JSON.stringify(rangos)));
}

async function cambiarColor(){
  await fetch('/set?color=' + document.getElementById('color').value);
  cargar();
}

async function guardar(){
  const r = await (await fetch('/guardar')).text();
  alert(r);
}

async function tick(){
  try{
    const r = await (await fetch('/info')).json();
    document.getElementById('info').textContent =
      'encontrado=' + r.encontrado + '  ex=' + r.ex + '  ey=' + r.ey +
      '  area=' + r.area + '  dist=' + r.dist + 'mm  ' + r.fps.toFixed(1) + 'fps';
  }catch(e){}
}
cargar(); setInterval(tick, 300);
</script></body></html>"""


def bucle_captura(cfg, args):
    cam = Camara(cfg["camara"]).abrir()
    t_ant = time.time()
    fps = 0.0
    try:
        while True:
            frame = cam.leer(solo_nuevos=True, timeout=0.5)
            if frame is None:
                continue
            ahora = time.time()
            fps = 0.85 * fps + 0.15 / max(ahora - t_ant, 1e-6)
            t_ant = ahora

            with estado["lock"]:
                det_obj = estado["detector"]
                color = estado["color"]
            d, mask, roi_y0 = det_obj.detectar(frame, color)

            with estado["lock"]:
                estado["frame"] = frame
                estado["mask"] = mask
                estado["det"] = d
                estado["roi_y0"] = roi_y0
                estado["fps"] = fps
    finally:
        cam.cerrar()


def _jpeg(img, calidad=60):
    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), calidad])
    return buf.tobytes() if ok else None


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _txt(self, cuerpo, tipo="text/plain"):
        b = cuerpo.encode("utf-8") if isinstance(cuerpo, str) else cuerpo
        self.send_response(200)
        self.send_header("Content-Type", tipo + "; charset=utf-8")
        self.send_header("Content-Length", str(len(b)))
        self.end_headers()
        self.wfile.write(b)

    def _stream(self, hacer_imagen):
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=f")
        self.end_headers()
        try:
            while True:
                img = hacer_imagen()
                if img is None:
                    time.sleep(0.05)
                    continue
                j = _jpeg(img)
                self.wfile.write(b"--f\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(j))
                self.wfile.write(j)
                self.wfile.write(b"\r\n")
                time.sleep(0.1)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def do_GET(self):
        u = urlparse(self.path)
        q = parse_qs(u.query)

        if u.path == "/":
            return self._txt(PAGINA, "text/html")

        if u.path == "/video":
            def img():
                with estado["lock"]:
                    f = estado.get("frame")
                    d = estado.get("det")
                    y0 = estado.get("roi_y0", 0)
                    cxg = estado["detector"].cx_garra
                if f is None or d is None:
                    return None
                return dibujar_overlay(f.copy(), d, estado["color"],
                                       estado.get("fps", 0), y0, cxg)
            return self._stream(img)

        if u.path == "/mask":
            def img():
                with estado["lock"]:
                    m = estado.get("mask")
                return None if m is None else cv2.cvtColor(m, cv2.COLOR_GRAY2BGR)
            return self._stream(img)

        if u.path == "/estado":
            with estado["lock"]:
                cfg = estado["cfg"]
                color = estado["color"]
                datos = {"color": color,
                         "colores": colores_disponibles(cfg),
                         "rangos": cfg["colores"][color]}
            return self._txt(json.dumps(datos), "application/json")

        if u.path == "/info":
            with estado["lock"]:
                d = estado.get("det")
                fps = estado.get("fps", 0)
            if d is None:
                return self._txt('{"encontrado":0,"ex":0,"ey":0,"area":0,"dist":-1,"fps":0}',
                                 "application/json")
            return self._txt(json.dumps({"encontrado": int(d.encontrado), "ex": d.ex,
                                         "ey": d.ey, "area": d.area,
                                         "dist": d.dist_mm, "fps": fps}),
                             "application/json")

        if u.path == "/set":
            with estado["lock"]:
                cfg = estado["cfg"]
                if "color" in q:
                    c = q["color"][0].upper()
                    if c in colores_disponibles(cfg):
                        estado["color"] = c
                if "rangos" in q:
                    cfg["colores"][estado["color"]] = json.loads(q["rangos"][0])
                    estado["detector"].recargar_color(estado["color"])
            return self._txt("ok")

        if u.path == "/guardar":
            with estado["lock"]:
                guardar_config(estado["cfg"])
            return self._txt("Guardado en config.json")

        self.send_response(404)
        self.end_headers()


def main():
    import signal
    import os
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=None)
    ap.add_argument("--puerto", type=int, default=8081)
    args = ap.parse_args()

    cfg = cargar_config(args.config)
    estado["cfg"] = cfg
    estado["color"] = cfg["deteccion"].get("color_inicial", colores_disponibles(cfg)[0])
    estado["detector"] = Detector(cfg)

    threading.Thread(target=bucle_captura, args=(cfg, args), daemon=True).start()
    srv = ThreadingHTTPServer(("0.0.0.0", args.puerto), Handler)

    def signal_handler(sig, frame):
        print("\nDeteniendo servidor...", flush=True)
        try:
            srv.server_close()
        except Exception:
            pass
        os._exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    print("Calibrador en http://<ip-de-la-pi>:%d" % args.puerto, flush=True)
    try:
        srv.serve_forever()
    except Exception:
        pass
    finally:
        os._exit(0)


if __name__ == "__main__":
    main()
