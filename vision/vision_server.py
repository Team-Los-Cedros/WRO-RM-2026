#!/usr/bin/env python3
"""
Servidor de vision: reconoce o sigue un artefacto y le dice a la MegaPi hacia
donde moverse.

PROTOCOLO (texto ASCII, lineas terminadas en \n, 115200 baudios)

  Pi -> MegaPi   (una linea por frame, a la frecuencia de serial.hz_envio)
      T <found> <color> <ex> <ey> <area> <dist> <fps> <confianza>
      ejemplo: T 1 ROJO -34 187 950 145 24 88
               T 0 NINGUNO 0 0 0 -1 24 0

      found : 1 si se ve el objeto, 0 si no
      ex    : error horizontal en pixeles respecto al centro de la garra.
              NEGATIVO = el objeto esta a la IZQUIERDA -> el robot gira a la izquierda
              POSITIVO = esta a la DERECHA             -> el robot gira a la derecha
      ey    : fila del borde inferior del objeto (mayor = mas cerca)
      area  : area del contorno en pixeles
      dist  : distancia estimada en mm segun la tabla de calibracion (-1 si no aplica)

  MegaPi -> Pi
      C <COLOR>   sigue ese color. C AUTO reconoce el artefacto centrado.
      X           escanea LOS CINCO colores en el frame actual y responde
                    X ROJO 1 -34 145 VERDE 0 0 -1 NEGRO 1 88 260 ...
                  Sirve para saber cual de los cinco colores NO salio en la
                  ronda y dejar su hueco vacio en la zona de descarga.
      S 0|1       apaga o enciende el envio continuo
      P           ping -> la Pi responde  K <version> <colores...>
      # <texto>   cualquier linea que empiece por '#' se trata como log de la
                  MegaPi y solo se imprime; asi puedes depurar el Arduino por
                  el mismo cable sin romper el protocolo.

Uso:
    python3 vision_server.py                  # produccion (con serial)
    python3 vision_server.py --sin-serial     # sin la MegaPi conectada
    python3 vision_server.py --web            # + video en http://<ip>:8080
"""

import argparse
from collections import deque
import datetime
import glob
import json
import os
from pathlib import Path
import sys
import threading
import time

import cv2

from vision_core import (Camara, ContadorFPS, Deteccion, Detector,
                         GestorExclusividadCamara, cargar_config,
                         colores_disponibles, dibujar_overlay)

VERSION = "3.0"


# ---------------------------------------------------------------------------
# Enlace serial con la MegaPi
# ---------------------------------------------------------------------------

class EnlaceSerial:
    def __init__(self, cfg_serial, log):
        self.cfg = cfg_serial
        self.log = log
        self.ser = None
        self._rx = ""
        self._proximo_intento = 0.0
        self._conectado_antes = False

    def _resolver_puerto(self):
        puerto = self.cfg.get("puerto", "/dev/ttyUSB0")
        if os.path.exists(puerto):
            return puerto
        patron = self.cfg.get("patron_by_id")
        candidatos = sorted(glob.glob(patron)) if patron else []
        return candidatos[0] if candidatos else puerto

    def abrir(self):
        import serial  # pyserial

        puerto = self._resolver_puerto()
        baud = self.cfg.get("baudios", 115200)

        if self.cfg.get("evitar_reset_dtr"):
            # Sin togglear DTR la MegaPi NO se reinicia al abrir el puerto.
            # Util si el Arduino ya esta a mitad de una rutina.
            self.ser = serial.Serial()
            self.ser.port = puerto
            self.ser.baudrate = baud
            self.ser.timeout = 0
            self.ser.dtr = False
            self.ser.rts = False
            self.ser.open()
        else:
            self.ser = serial.Serial(puerto, baud, timeout=0)
            espera = self.cfg.get("espera_reset_s", 2.5)
            # Abrir el puerto USB resetea el ATmega2560. Hay que esperar a que
            # arranque o los primeros mensajes se pierden.
            self.log("esperando %.1fs el reinicio de la MegaPi..." % espera)
            time.sleep(espera)
            self.ser.reset_input_buffer()

        self.log("serial abierto en %s @ %d" % (puerto, baud))
        self._conectado_antes = True
        self._proximo_intento = 0.0
        return self

    @property
    def conectado(self):
        return self.ser is not None and self.ser.is_open

    def _marcar_desconectado(self, error):
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self._rx = ""
        self._proximo_intento = time.monotonic() + self.cfg.get("reintento_s", 1.0)
        self.log("serial desconectado: %s" % error)

    def asegurar_conectado(self):
        if self.conectado:
            return True
        if time.monotonic() < self._proximo_intento:
            return False
        try:
            self.abrir()
            return True
        except Exception as e:
            self._proximo_intento = time.monotonic() + self.cfg.get("reintento_s", 1.0)
            return False

    def enviar(self, linea):
        if self.ser is None:
            return
        try:
            self.ser.write((linea + "\n").encode("ascii", "ignore"))
        except Exception as e:
            self._marcar_desconectado(e)

    def leer_lineas(self):
        """Lectura no bloqueante; devuelve las lineas completas recibidas."""
        if self.ser is None:
            return []
        try:
            n = self.ser.in_waiting
            if n:
                self._rx += self.ser.read(n).decode("ascii", "ignore")
        except Exception as e:
            self._marcar_desconectado(e)
            return []

        if "\n" not in self._rx:
            return []
        partes = self._rx.split("\n")
        self._rx = partes[-1]
        return [p.strip() for p in partes[:-1] if p.strip()]

    def cerrar(self):
        if self.ser is not None:
            self.ser.close()
            self.ser = None


# ---------------------------------------------------------------------------
# Buffer de video y registro de telemetria
# ---------------------------------------------------------------------------

class BufferFrame:
    def __init__(self):
        self.lock = threading.Lock()
        self.jpeg = None

    def set(self, frame):
        ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
        if ok:
            with self.lock:
                self.jpeg = buf.tobytes()

    def get(self):
        with self.lock:
            return self.jpeg


class RegistroTelemetria:
    """Almacena registros de telemetria en memoria y archivo JSONL para depuracion."""
    def __init__(self, cfg_telemetria=None, max_memoria=3000):
        self.lock = threading.Lock()
        self.items = deque(maxlen=max_memoria)
        self.cfg = cfg_telemetria or {}
        self.habilitado = self.cfg.get("habilitado", True)
        self.archivo_log = None
        self.archivo_path = None

        if self.habilitado and self.cfg.get("guardar_archivo", True):
            try:
                dir_logs = Path(__file__).parent / self.cfg.get("directorio_logs", "logs")
                dir_logs.mkdir(parents=True, exist_ok=True)
                ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                self.archivo_path = dir_logs / f"telemetria_{ts}.jsonl"
                self.archivo_log = open(self.archivo_path, "a", encoding="utf-8")
            except Exception as e:
                print(f"[telemetria] Error al inicializar archivo de logs: {e}", flush=True)

    def registrar(self, tipo, datos):
        if not self.habilitado:
            return
        ahora = time.time()
        registro = {
            "t_epoch": round(ahora, 4),
            "t_iso": datetime.datetime.now(datetime.timezone.utc).isoformat(),
            "tipo": tipo,
            **datos
        }
        with self.lock:
            self.items.append(registro)

        if self.archivo_log:
            try:
                self.archivo_log.write(json.dumps(registro, ensure_ascii=False) + "\n")
                self.archivo_log.flush()
            except Exception:
                pass

    def obtener_todos(self):
        with self.lock:
            return list(self.items)

    def limpiar(self):
        with self.lock:
            self.items.clear()

    def cerrar(self):
        if self.archivo_log:
            try:
                self.archivo_log.close()
            except Exception:
                pass


def arrancar_web(buffer_frame, telemetria, estado_global, puerto=8080):
    from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

    PAGINA = (b"<!DOCTYPE html><html><head><meta charset='utf-8'><title>WRO Vision & Telemetria</title>"
              b"<style>body{background:#18181b;color:#f4f4f5;font-family:ui-monospace,monospace;margin:0;padding:15px;text-align:center}"
              b".container{max-width:800px;margin:0 auto}"
              b"img{width:100%;max-width:640px;height:auto;border:2px solid #3f3f46;border-radius:6px;image-rendering:pixelated}"
              b".panel{background:#27272a;border-radius:6px;padding:12px;margin-top:10px;text-align:left;font-size:13px}"
              b".tag{display:inline-block;padding:2px 8px;border-radius:4px;background:#3f3f46;font-weight:bold;margin-right:6px}"
              b"#logs{max-height:180px;overflow-y:auto;background:#09090b;padding:8px;border-radius:4px;margin-top:8px;font-size:12px}"
              b"</style></head><body><div class='container'>"
              b"<h2>WRO RoboMission 2026 - Vision & Telemetria</h2>"
              b"<img src='/stream'>"
              b"<div class='panel'>"
              b"<div><span class='tag'>STREAM</span> <a href='/stream' target='_blank' style='color:#38bdf8'>/stream</a> | "
              b"<span class='tag'>TELEMETRIA</span> <a href='/telemetria' target='_blank' style='color:#4ade80'>/telemetria (JSON)</a> | "
              b"<span class='tag'>ESTADO</span> <a href='/estado' target='_blank' style='color:#fbbf24'>/estado</a></div>"
              b"<div id='logs'>Cargando telemetria en vivo...</div>"
              b"</div></div>"
              b"<script>"
              b"async function updateLogs(){try{let r=await fetch('/telemetria');let d=await r.json();let l=document.getElementById('logs');"
              b"if(d.items){l.innerHTML=d.items.slice(-12).reverse().map(e=>`[${e.t_iso.split('T')[1].slice(0,8)}] <b>${e.tipo.toUpperCase()}</b>: `+JSON.stringify(e)).join('<br>');}"
              b"}catch(e){}setTimeout(updateLogs, 800);}updateLogs();"
              b"</script></body></html>")

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_GET(self):
            if self.path == "/stream":
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                try:
                    while True:
                        jpeg = buffer_frame.get()
                        if jpeg is None:
                            time.sleep(0.04)
                            continue
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                         b"Content-Length: %d\r\n\r\n" % len(jpeg))
                        self.wfile.write(jpeg)
                        self.wfile.write(b"\r\n")
                        time.sleep(0.05)  # ~20 fps
                except (BrokenPipeError, ConnectionResetError):
                    pass

            elif self.path == "/telemetria":
                self.send_response(200)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                items = telemetria.obtener_todos()
                resp = json.dumps({"total": len(items), "items": items}, ensure_ascii=False)
                self.wfile.write(resp.encode("utf-8"))

            elif self.path == "/telemetria/reset":
                telemetria.limpiar()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                self.wfile.write(b'{"status":"ok","message":"telemetria limpiada"}')

            elif self.path == "/estado":
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                resp = json.dumps(estado_global, ensure_ascii=False)
                self.wfile.write(resp.encode("utf-8"))

            else:
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.end_headers()
                self.wfile.write(PAGINA)

    srv = ThreadingHTTPServer(("0.0.0.0", puerto), Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv


# ---------------------------------------------------------------------------
# Bucle principal
# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=None)
    ap.add_argument("--sin-serial", action="store_true",
                    help="no abre el puerto serial; imprime las tramas en pantalla")
    ap.add_argument("--web", action="store_true",
                    help="publica el video con overlay en http://<ip>:8080")
    ap.add_argument("--puerto-web", type=int, default=None)
    ap.add_argument("--color", default=None, help="color inicial a buscar")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()

    def log(msg):
        print("[vision] %s" % msg, flush=True)

    cfg = cargar_config(args.config)
    disponibles = colores_disponibles(cfg)
    color = (args.color or cfg["deteccion"].get("color_inicial", disponibles[0])).upper()
    modo = cfg["deteccion"].get("modo_inicial", "PAUSA").upper()
    if color not in disponibles and color != "AUTO":
        log("color '%s' no definido; usando %s" % (color, disponibles[0]))
        color = disponibles[0]
    if modo not in ("PAUSA", "ARTEFACTO", "DESTINO", "FILA"):
        modo = "PAUSA"

    detector = Detector(cfg)
    telemetria = RegistroTelemetria(cfg.get("telemetria", {}))
    
    gestor = GestorExclusividadCamara(nombre_script="vision_server.py", gestionar_servicio=False)
    gestor.adquirir()
    
    cam = Camara(cfg["camara"]).abrir()
    log("camara %dx%d abierta, buscando %s" % (cam.ancho, cam.alto, color))

    enlace = None
    if cfg["serial"].get("habilitado", True) and not args.sin_serial:
        enlace = EnlaceSerial(cfg["serial"], log)
        if not enlace.asegurar_conectado():
            log("AVISO: MegaPi ausente; se reconectara automaticamente")

    cfg_web = cfg.get("web", {})
    habilitar_web = args.web or cfg_web.get("habilitado", True)
    puerto_web = args.puerto_web or cfg_web.get("puerto", 8080)

    estado_global = {
        "version": VERSION,
        "modo": modo,
        "color_actual": color,
        "fps": 0,
        "serial_conectado": enlace is not None and enlace.conectado,
        "enviando": True
    }

    buffer_frame = None
    if habilitar_web:
        buffer_frame = BufferFrame()
        arrancar_web(buffer_frame, telemetria, estado_global, puerto_web)
        log("servidor web y telemetria en http://<ip-de-la-pi>:%d" % puerto_web)

    fps = ContadorFPS()
    hz_envio = cfg["serial"].get("hz_envio", 20)
    periodo_envio = 0.0 if hz_envio <= 0 else 0.9 / hz_envio
    ultimo_envio = 0.0
    enviando = True

    try:
        while True:
            if enlace:
                enlace.asegurar_conectado()
                estado_global["serial_conectado"] = enlace.conectado

            frame = cam.leer(solo_nuevos=True, timeout=0.5)
            det = Deteccion()
            color_detectado = "NINGUNO"
            roi_y0 = 0
            if frame is None:
                f = fps.fps
            elif modo == "ARTEFACTO" and color == "AUTO":
                det, color_detectado, _mask, roi_y0 = detector.detectar_auto(frame)
                f = fps.tick()
            elif modo == "ARTEFACTO":
                det, _mask, roi_y0 = detector.detectar(frame, color)
                color_detectado = color
                f = fps.tick()
            elif modo == "DESTINO" and color in disponibles:
                det, _mask, roi_y0 = detector.detectar_destino(frame, color)
                color_detectado = color
                f = fps.tick()
            else:
                f = fps.tick()

            estado_global["fps"] = int(f)
            estado_global["modo"] = modo
            estado_global["color_actual"] = color
            estado_global["camara"] = cam.estado()

            ahora = time.time()
            if enviando and (ahora - ultimo_envio) >= periodo_envio:
                ultimo_envio = ahora
                trama = "T %d %s %d %d %d %d %d %d" % (
                    1 if det.encontrado else 0, color_detectado,
                    det.ex, det.ey, det.area, det.dist_mm, int(f),
                    det.confianza)
                if enlace:
                    enlace.enviar(trama)
                if args.verbose or enlace is None:
                    print(trama, flush=True)

                # Registrar telemetria de vision
                telemetria.registrar("vision", {
                    "found": 1 if det.encontrado else 0,
                    "modo": modo,
                    "color": color_detectado,
                    "ex": det.ex,
                    "ey": det.ey,
                    "area": det.area,
                    "dist_mm": det.dist_mm,
                    "fps": int(f),
                    "confianza": det.confianza
                })

            # Comandos y logs que llegan de la MegaPi
            if enlace:
                for linea in enlace.leer_lineas():
                    if linea.startswith("#"):
                        msg_mega = linea[1:].strip()
                        log("MegaPi: %s" % msg_mega)
                        telemetria.registrar("megapi", {"mensaje": msg_mega})
                        continue
                    
                    partes = linea.split()
                    cmd = partes[0].upper()
                    telemetria.registrar("comando", {"comando": cmd, "linea": linea})

                    if cmd == "C" and len(partes) >= 2:
                        nuevo = partes[1].upper()
                        if nuevo in disponibles or nuevo == "AUTO":
                            color = nuevo
                            modo = "ARTEFACTO"
                            log("modo -> ARTEFACTO %s" % color)
                            enlace.enviar("K COLOR %s" % color)
                        else:
                            enlace.enviar("E COLOR")
                    elif cmd == "M" and len(partes) >= 2:
                        nuevo_modo = partes[1].upper()
                        nuevo_color = partes[2].upper() if len(partes) >= 3 else color
                        valido = nuevo_modo in ("PAUSA", "FILA") or (
                            nuevo_modo in ("ARTEFACTO", "DESTINO") and
                            nuevo_color in disponibles + (["AUTO"] if nuevo_modo == "ARTEFACTO" else []))
                        if valido:
                            modo = nuevo_modo
                            color = nuevo_color
                            log("modo -> %s %s" % (modo, color))
                            enlace.enviar("K MODO %s %s" % (modo, color))
                        else:
                            enlace.enviar("E MODO")
                    elif cmd == "F":
                        fila = detector.detectar_fila_artefactos(frame) if frame is not None else []
                        enlace.enviar("F %d%s" % (
                            len(fila), "" if not fila else " " +
                            " ".join(d.color for d in fila)))
                    elif cmd == "X":
                        partes_resp = []
                        for c in disponibles:
                            dc, _, _ = detector.detectar(frame, c)
                            partes_resp.append("%s %d %d %d" % (
                                c, 1 if dc.encontrado else 0, dc.ex, dc.dist_mm))
                        enlace.enviar("X " + " ".join(partes_resp))
                    elif cmd == "S" and len(partes) >= 2:
                        enviando = partes[1] != "0"
                        estado_global["enviando"] = enviando
                        enlace.enviar("K STREAM %d" % (1 if enviando else 0))
                    elif cmd == "P":
                        enlace.enviar("K %s MODOS=PAUSA,ARTEFACTO,FILA,DESTINO COLORES=AUTO,%s" %
                                      (VERSION, ",".join(disponibles)))

            if buffer_frame is not None and frame is not None:
                etiqueta = "%s/%s" % (modo, color_detectado if color == "AUTO" else color)
                buffer_frame.set(dibujar_overlay(frame.copy(), det, etiqueta, f,
                                                 roi_y0, detector.cx_garra))

    except KeyboardInterrupt:
        log("detenido por el usuario")
    finally:
        telemetria.cerrar()
        cam.cerrar()
        gestor.liberar()
        if enlace:
            enlace.cerrar()


if __name__ == "__main__":
    sys.exit(main())
