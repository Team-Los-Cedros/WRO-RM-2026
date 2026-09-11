#!/usr/bin/env python3
"""Calibrador web exclusivo para los cuadros planos del museo.

    python3 calibrar_destinos.py
    -> abre http://<ip-de-la-pi>:8083 desde tu PC

Este calibrador modifica ``colores_destino`` en config.json, no los rangos
``colores`` de los artefactos 3D. La vista de mascara muestra el HSV activo y
la vista de camara ejecuta el detector real de destino, incluida la validacion
de forma y marco blanco.
"""

import argparse
import copy
import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np

from vision_core import (Camara, Detector, GestorExclusividadCamara,
                         cargar_config, colores_disponibles, dibujar_overlay,
                         guardar_config)


def _suprimir_warnings_libjpeg():
    """Silencia avisos de frames MJPG truncados que OpenCV puede recuperar."""
    try:
        devnull_fd = os.open(os.devnull, os.O_WRONLY)
        os.dup2(devnull_fd, 2)
        os.close(devnull_fd)
    except OSError:
        pass


def _asegurar_colores_destino(cfg):
    """Crea rangos de destino a partir de artefactos en configs antiguos."""
    destino = cfg.setdefault("colores_destino", {})
    for color in colores_disponibles(cfg):
        if color not in destino:
            destino[color] = copy.deepcopy(cfg["colores"][color])


def _colores_destino(cfg):
    destino = cfg.get("colores_destino", {})
    orden = colores_disponibles(cfg)
    return [color for color in orden if color in destino]


def _validar_rangos(valor):
    """Normaliza una lista de 1..4 rangos HSV de OpenCV."""
    if not isinstance(valor, list) or not 1 <= len(valor) <= 4:
        raise ValueError("se requieren entre 1 y 4 rangos HSV")

    limites = (179, 255, 255, 179, 255, 255)
    resultado = []
    for rango in valor:
        if not isinstance(rango, list) or len(rango) != 6:
            raise ValueError("cada rango debe contener 6 valores")
        normalizado = []
        for indice, dato in enumerate(rango):
            if isinstance(dato, bool):
                raise ValueError("los valores HSV deben ser enteros")
            numero = int(dato)
            if numero < 0 or numero > limites[indice]:
                raise ValueError("valor HSV fuera de rango")
            normalizado.append(numero)
        if any(normalizado[i] > normalizado[i + 3] for i in range(3)):
            raise ValueError("un minimo HSV no puede superar a su maximo")
        resultado.append(normalizado)
    return resultado


def _validar_roi(valor):
    """Normaliza la ROI vertical [inicio, fin] expresada como fracciones."""
    if not isinstance(valor, list) or len(valor) != 2:
        raise ValueError("la ROI debe contener inicio y fin")
    inicio, fin = (float(valor[0]), float(valor[1]))
    if not 0.0 <= inicio < fin <= 1.0:
        raise ValueError("la ROI debe cumplir 0 <= inicio < fin <= 1")
    if fin - inicio < 0.05:
        raise ValueError("la ROI debe cubrir al menos 5% de la imagen")
    return [round(inicio, 3), round(fin, 3)]


estado = {
    "cfg": None,
    "config_path": None,
    "color": None,
    "detector": None,
    "lock": threading.Lock(),
    "det": None,
    "diagnostico": None,
    "fps": 0.0,
    "jpeg_video": None,
    "jpeg_mask": None,
    "detener": threading.Event(),
}


PAGINA = r"""<!doctype html>
<html lang="es"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Calibrar cuadros de destino WRO</title>
<style>
body{background:#15171c;color:#e8e8e8;font-family:system-ui,sans-serif;margin:12px}
.cabecera{display:flex;gap:12px;align-items:center;flex-wrap:wrap}
.fila{display:flex;gap:12px;flex-wrap:wrap;align-items:flex-start}
.panel{background:#1d2027;border:1px solid #30343e;border-radius:8px;padding:10px}
.visor{width:min(46vw,480px);min-width:300px}
img{width:100%;border:1px solid #333;image-rendering:pixelated;background:#000}
label{display:block;margin:6px 0;font-size:13px}
input[type=range]{width:260px;vertical-align:middle}
.val{display:inline-block;width:38px;text-align:right;font-variant-numeric:tabular-nums}
button{padding:8px 13px;margin:4px 5px 4px 0;border:0;border-radius:6px;background:#3a6df0;color:#fff;font-size:14px;cursor:pointer}
button.sec{background:#444} button.danger{background:#85404b}
select{padding:6px;background:#222;color:#eee;border:1px solid #444;border-radius:4px}
#info,#json{font-family:ui-monospace,monospace;background:#0c0e12;padding:8px;border-radius:6px;margin-top:8px;font-size:13px;white-space:pre-wrap}
#mensaje{min-height:22px;color:#8bd49c;margin-top:6px}.error{color:#ff8f8f!important}
h3{margin:8px 0} h4{margin:14px 0 4px}.nota{max-width:980px;color:#c7cad1;font-size:13px}
a{color:#58a6ff}
</style></head><body>
<div class="cabecera"><h3>Calibracion HSV de cuadros de destino</h3>
<a href="javascript:void(0)" onclick="window.open('http://'+location.hostname+':8082','_blank')">Ajustar camara</a></div>
<p class="nota">Edita solamente <code>colores_destino</code>. La mascara debe cubrir la tinta del cuadro sin pintar la pista. El rectangulo verde solo aparece cuando el candidato tambien pasa forma, relleno y marco blanco.</p>
<div class="fila">
  <div class="panel visor"><div>Camara + detector DESTINO</div><img src="/video"></div>
  <div class="panel visor"><div>Mascara HSV de destino</div><img src="/mask"></div>
  <div class="panel">
    <label>Color: <select id="color" onchange="cambiarColor()"></select></label>
    <div id="sliders"></div>
    <h4>ROI vertical del detector</h4>
    <label>Inicio <input id="roi0" type="range" min="0" max="95" value="4" oninput="cambiarRoi(0,this.value)"><span class="val" id="r0">4%</span></label>
    <label>Fin <input id="roi1" type="range" min="5" max="100" value="80" oninput="cambiarRoi(1,this.value)"><span class="val" id="r1">80%</span></label>
    <button class="sec" onclick="agregarRango()">+ rango</button>
    <button class="danger" onclick="quitarRango()">- ultimo rango</button><br>
    <button onclick="guardar()">Guardar colores_destino</button>
    <button class="sec" onclick="recargarDisco()">Descartar y recargar</button>
    <div id="mensaje"></div>
    <div id="info">Esperando imagen...</div>
    <div id="json"></div>
  </div>
</div>
<script>
let rangos=[], roi=[0.04,0.80];
const N=['H min','S min','V min','H max','S max','V max'];
const MAX=[179,255,255,179,255,255];

function mensaje(texto,error=false){
  const e=document.getElementById('mensaje'); e.textContent=texto;
  e.className=error?'error':'';
}

async function peticion(url){
  const respuesta=await fetch(url);
  const texto=await respuesta.text();
  if(!respuesta.ok) throw new Error(texto||('HTTP '+respuesta.status));
  return texto;
}

async function cargar(){
  try{
    const r=JSON.parse(await peticion('/estado'));
    const sel=document.getElementById('color');
    sel.innerHTML=''; r.colores.forEach(c=>sel.add(new Option(c,c)));
    sel.value=r.color; rangos=r.rangos; roi=r.roi_y;
    pintarSliders(); pintarRoi(); mensaje('');
  }catch(e){mensaje(e.message,true)}
}

function pintarSliders(){
  const cont=document.getElementById('sliders'); cont.innerHTML='';
  rangos.forEach((rango,i)=>{
    const t=document.createElement('h4'); t.textContent='Rango '+(i+1); cont.appendChild(t);
    rango.forEach((v,j)=>{
      const lab=document.createElement('label');
      lab.innerHTML=N[j]+' <input id="s'+i+'_'+j+'" type="range" min="0" max="'+MAX[j]+'" value="'+v+'" oninput="cambiar('+i+','+j+',this.value)">'+
        '<span class="val" id="v'+i+'_'+j+'">'+v+'</span>';
      cont.appendChild(lab);
    });
  });
  mostrarJson();
}

let envioPendiente=null;
function cambiar(i,j,v){
  v=parseInt(v); rangos[i][j]=v;
  const par=j<3?j+3:j-3;
  if(j<3 && v>rangos[i][par]) rangos[i][par]=v;
  if(j>=3 && v<rangos[i][par]) rangos[i][par]=v;
  document.getElementById('v'+i+'_'+j).textContent=rangos[i][j];
  document.getElementById('v'+i+'_'+par).textContent=rangos[i][par];
  document.getElementById('s'+i+'_'+par).value=rangos[i][par];
  mostrarJson();
  clearTimeout(envioPendiente);
  envioPendiente=setTimeout(enviarRangos,45);
}

async function enviarRangos(){
  try{await peticion('/set?rangos='+encodeURIComponent(JSON.stringify(rangos)));}
  catch(e){mensaje(e.message,true)}
}

function pintarRoi(){
  document.getElementById('roi0').value=Math.round(roi[0]*100);
  document.getElementById('roi1').value=Math.round(roi[1]*100);
  document.getElementById('r0').textContent=Math.round(roi[0]*100)+'%';
  document.getElementById('r1').textContent=Math.round(roi[1]*100)+'%';
  mostrarJson();
}

let envioRoiPendiente=null;
function cambiarRoi(indice,valor){
  valor=parseInt(valor)/100;
  if(indice===0) valor=Math.min(valor,roi[1]-0.05);
  else valor=Math.max(valor,roi[0]+0.05);
  roi[indice]=Math.round(valor*100)/100;
  pintarRoi();
  clearTimeout(envioRoiPendiente);
  envioRoiPendiente=setTimeout(enviarRoi,45);
}

async function enviarRoi(){
  try{await peticion('/set?roi='+encodeURIComponent(JSON.stringify(roi)));}
  catch(e){mensaje(e.message,true)}
}

async function cambiarColor(){
  try{await peticion('/set?color='+encodeURIComponent(document.getElementById('color').value)); await cargar();}
  catch(e){mensaje(e.message,true)}
}

function agregarRango(){
  if(rangos.length>=4) return mensaje('Maximo 4 rangos.',true);
  rangos.push([0,0,0,179,255,255]); pintarSliders(); enviarRangos();
}

function quitarRango(){
  if(rangos.length<=1) return mensaje('Debe quedar al menos un rango.',true);
  rangos.pop(); pintarSliders(); enviarRangos();
}

function mostrarJson(){
  const c=document.getElementById('color').value||'?';
  document.getElementById('json').textContent='"'+c+'": '+JSON.stringify(rangos)+
    '\n"roi_y": '+JSON.stringify(roi);
}

async function guardar(){
  try{mensaje(await peticion('/guardar'));}catch(e){mensaje(e.message,true)}
}

async function recargarDisco(){
  try{mensaje(await peticion('/recargar')); await cargar();}catch(e){mensaje(e.message,true)}
}

async function tick(){
  try{
    const r=JSON.parse(await peticion('/info'));
    const d=r.diagnostico||{};
    const detalle=d.area>0 ?
      '\ncontorno: area='+d.area+' aspecto='+d.aspecto.toFixed(2)+
      ' relleno='+(100*d.relleno).toFixed(1)+'% marco='+(100*d.marco_blanco).toFixed(1)+
      '% vertices='+d.vertices+' rechazo='+(d.rechazo||'ninguno') : '';
    document.getElementById('info').textContent=
      'valido='+r.encontrado+'  ex='+r.ex+'  ey='+r.ey+'  area='+r.area+
      '  caja='+r.w+'x'+r.h+'  dist='+r.dist+'mm  q='+r.confianza+
      '  '+r.fps.toFixed(1)+'fps'+detalle;
  }catch(e){}
}
cargar(); setInterval(tick,300);
</script></body></html>"""


def _jpeg(img, calidad=65):
    ok, buf = cv2.imencode(".jpg", img,
                           [int(cv2.IMWRITE_JPEG_QUALITY), calidad])
    return buf.tobytes() if ok else None


def _diagnosticar_candidato(frame, mask, roi_y0, detector):
    """Describe el mayor contorno HSV, incluso si el detector lo rechaza."""
    cfg = detector.destino_cfg
    contornos = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                 cv2.CHAIN_APPROX_SIMPLE)[-2]
    if not contornos:
        return None

    contorno = max(contornos, key=cv2.contourArea)
    area = float(cv2.contourArea(contorno))
    x, y, w, h = cv2.boundingRect(contorno)
    if w <= 0 or h <= 0:
        return None

    aspecto = w / float(h)
    relleno = area / float(w * h)
    alto, ancho = frame.shape[:2]
    roi_alto = mask.shape[0]
    margen = max(6, int(max(w, h) * cfg.get("margen_marco_frac", 0.45)))
    rx0, ry0 = max(0, x - margen), max(0, y - margen)
    rx1, ry1 = min(ancho, x + w + margen), min(roi_alto, y + h + margen)
    anillo = np.ones((ry1 - ry0, rx1 - rx0), dtype=np.uint8)
    anillo[y - ry0:y + h - ry0, x - rx0:x + w - rx0] = 0
    hsv = cv2.cvtColor(frame[roi_y0:roi_y0 + roi_alto], cv2.COLOR_BGR2HSV)
    hsv_anillo = hsv[ry0:ry1, rx0:rx1]
    blanco = ((hsv_anillo[:, :, 1] <= cfg.get("blanco_s_max", 70)) &
              (hsv_anillo[:, :, 2] >= cfg.get("blanco_v_min", 120)) &
              (anillo > 0))
    marco_blanco = np.count_nonzero(blanco) / float(
        max(int(np.count_nonzero(anillo)), 1))
    perimetro = cv2.arcLength(contorno, True)
    vertices = len(cv2.approxPolyDP(contorno, 0.04 * perimetro, True))

    rechazo = ""
    if area < cfg.get("area_min_px", 220):
        rechazo = "area"
    elif y <= 1 or y + h >= roi_alto - 1:
        rechazo = "borde_roi"
    elif not (cfg.get("relacion_aspecto_min", 0.45) <= aspecto <=
              cfg.get("relacion_aspecto_max", 2.2)):
        rechazo = "aspecto"
    elif relleno < cfg.get("relleno_min", 0.58):
        rechazo = "relleno"
    elif marco_blanco < cfg.get("marco_blanco_min", 0.12):
        rechazo = "marco_blanco"

    return {
        "area": int(area), "w": w, "h": h,
        "aspecto": round(aspecto, 4), "relleno": round(relleno, 4),
        "marco_blanco": round(marco_blanco, 4), "vertices": vertices,
        "rechazo": rechazo,
    }


def bucle_captura(cfg):
    cam = Camara(cfg["camara"]).abrir()
    t_ant = time.time()
    fps = 0.0
    try:
        while not estado["detener"].is_set():
            frame = cam.leer(solo_nuevos=True, timeout=0.5)
            if frame is None:
                continue
            ahora = time.time()
            fps = 0.85 * fps + 0.15 / max(ahora - t_ant, 1e-6)
            t_ant = ahora

            with estado["lock"]:
                detector = estado["detector"]
                color = estado["color"]
            det, mask, roi_y0 = detector.detectar_destino(frame, color)
            diagnostico = _diagnosticar_candidato(
                frame, mask, roi_y0, detector)

            alto, ancho = frame.shape[:2]
            mask_completa = np.zeros((alto, ancho), dtype=np.uint8)
            fin = min(roi_y0 + mask.shape[0], alto)
            mask_completa[roi_y0:fin] = mask[:fin - roi_y0]

            vis = dibujar_overlay(frame.copy(), det, "DESTINO/" + color,
                                  fps, roi_y0, detector.cx_garra)
            roi_y = detector.destino_cfg.get("roi_y", [0.04, 0.70])
            roi_y1 = min(alto - 1, int(alto * roi_y[1]))
            cv2.line(vis, (0, roi_y1), (ancho - 1, roi_y1),
                     (120, 120, 120), 1)

            j_video = _jpeg(vis)
            j_mask = _jpeg(cv2.cvtColor(mask_completa, cv2.COLOR_GRAY2BGR))
            with estado["lock"]:
                estado["det"] = det
                estado["diagnostico"] = diagnostico
                estado["fps"] = fps
                estado["jpeg_video"] = j_video
                estado["jpeg_mask"] = j_mask
    finally:
        cam.cerrar()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def _responder(self, cuerpo, tipo="text/plain", codigo=200):
        datos = cuerpo.encode("utf-8") if isinstance(cuerpo, str) else cuerpo
        self.send_response(codigo)
        self.send_header("Content-Type", tipo + "; charset=utf-8")
        self.send_header("Content-Length", str(len(datos)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(datos)

    def _error(self, mensaje, codigo=400):
        self._responder(mensaje, codigo=codigo)

    def _stream_jpeg(self, clave):
        self.send_response(200)
        self.send_header("Content-Type",
                         "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        try:
            while not estado["detener"].is_set():
                with estado["lock"]:
                    jpeg = estado.get(clave)
                if jpeg is None:
                    time.sleep(0.05)
                    continue
                self.wfile.write(
                    b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n"
                    % len(jpeg))
                self.wfile.write(jpeg)
                self.wfile.write(b"\r\n")
                time.sleep(0.05)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass

    def do_GET(self):
        url = urlparse(self.path)
        query = parse_qs(url.query)

        if url.path == "/":
            return self._responder(PAGINA, "text/html")
        if url.path == "/video":
            return self._stream_jpeg("jpeg_video")
        if url.path == "/mask":
            return self._stream_jpeg("jpeg_mask")

        if url.path == "/estado":
            with estado["lock"]:
                cfg = estado["cfg"]
                color = estado["color"]
                datos = {
                    "color": color,
                    "colores": _colores_destino(cfg),
                    "rangos": cfg["colores_destino"][color],
                    "roi_y": cfg["destino"].get("roi_y", [0.04, 0.70]),
                }
            return self._responder(json.dumps(datos), "application/json")

        if url.path == "/info":
            with estado["lock"]:
                det = estado.get("det")
                diagnostico = estado.get("diagnostico")
                fps = estado.get("fps", 0.0)
            datos = {"encontrado": 0, "ex": 0, "ey": 0, "area": 0,
                     "w": 0, "h": 0, "dist": -1, "confianza": 0,
                     "fps": fps}
            if det is not None:
                datos.update({"encontrado": int(det.encontrado),
                              "ex": det.ex, "ey": det.ey,
                              "area": det.area, "w": det.w, "h": det.h,
                              "dist": det.dist_mm,
                              "confianza": det.confianza})
            datos["diagnostico"] = diagnostico
            return self._responder(json.dumps(datos), "application/json")

        if url.path == "/set":
            try:
                with estado["lock"]:
                    cfg = estado["cfg"]
                    if "color" in query:
                        color = query["color"][0].upper()
                        if color not in _colores_destino(cfg):
                            raise ValueError("color de destino desconocido")
                        estado["color"] = color
                    if "rangos" in query:
                        rangos = _validar_rangos(
                            json.loads(query["rangos"][0]))
                        cfg["colores_destino"][estado["color"]] = rangos
                    if "roi" in query:
                        cfg["destino"]["roi_y"] = _validar_roi(
                            json.loads(query["roi"][0]))
                return self._responder("ok")
            except (ValueError, TypeError, json.JSONDecodeError) as exc:
                return self._error(str(exc))

        if url.path == "/guardar":
            with estado["lock"]:
                guardar_config(estado["cfg"], estado["config_path"])
                ruta = estado["config_path"] or "config.json"
            return self._responder(
                "Rangos y ROI de destino guardados en " + ruta)

        if url.path == "/recargar":
            try:
                with estado["lock"]:
                    cfg = cargar_config(estado["config_path"])
                    _asegurar_colores_destino(cfg)
                    colores = _colores_destino(cfg)
                    if not colores:
                        raise ValueError("no hay colores de destino configurados")
                    color = estado["color"]
                    estado["cfg"] = cfg
                    estado["detector"] = Detector(cfg)
                    estado["color"] = color if color in colores else colores[0]
                return self._responder("Configuracion recargada desde disco")
            except (OSError, ValueError, json.JSONDecodeError) as exc:
                return self._error(str(exc), 500)

        self.send_response(404)
        self.end_headers()


def main():
    import signal

    parser = argparse.ArgumentParser(
        description="Calibrador HSV de cuadros planos de destino")
    parser.add_argument("--config", default=None)
    parser.add_argument("--puerto", type=int, default=8083)
    args = parser.parse_args()

    cfg = cargar_config(args.config)
    _asegurar_colores_destino(cfg)
    colores = _colores_destino(cfg)
    if not colores:
        raise RuntimeError("No hay colores de destino para calibrar")

    estado["cfg"] = cfg
    estado["config_path"] = args.config
    estado["color"] = colores[0]
    estado["detector"] = Detector(cfg)
    estado["detener"].clear()

    gestor = GestorExclusividadCamara(
        nombre_script="calibrar_destinos.py", gestionar_servicio=True)
    gestor.adquirir()
    _suprimir_warnings_libjpeg()

    hilo = threading.Thread(target=bucle_captura, args=(cfg,), daemon=True)
    hilo.start()
    servidor = ThreadingHTTPServer(("0.0.0.0", args.puerto), Handler)

    def interrumpir(_sig, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, interrumpir)
    signal.signal(signal.SIGTERM, interrumpir)
    print("Calibrador de destinos en http://<ip-de-la-pi>:%d" % args.puerto,
          flush=True)
    print("Edita colores_destino; colores de artefactos no se modifica.",
          flush=True)
    try:
        servidor.serve_forever()
    except KeyboardInterrupt:
        print("\nDeteniendo calibrador...", flush=True)
    finally:
        estado["detener"].set()
        servidor.server_close()
        hilo.join(timeout=1.0)
        gestor.liberar()


if __name__ == "__main__":
    main()
