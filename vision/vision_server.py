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
import sys
import threading
import time

import cv2

from vision_core import (Camara, ContadorFPS, Detector, GestorExclusividadCamara, cargar_config,
                         colores_disponibles, dibujar_overlay)

VERSION = "2.0"


# ---------------------------------------------------------------------------
# Enlace serial con la MegaPi
# ---------------------------------------------------------------------------

class EnlaceSerial:
    def __init__(self, cfg_serial, log):
        self.cfg = cfg_serial
        self.log = log
        self.ser = None
        self._rx = ""

    def abrir(self):
        import serial  # pyserial

        puerto = self.cfg["puerto"]
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
        return self

    def enviar(self, linea):
        if self.ser is None:
            return
        try:
            self.ser.write((linea + "\n").encode("ascii", "ignore"))
        except Exception as e:
            self.log("error al escribir en serial: %s" % e)

    def leer_lineas(self):
        """Lectura no bloqueante; devuelve las lineas completas recibidas."""
        if self.ser is None:
            return []
        try:
            n = self.ser.in_waiting
            if n:
                self._rx += self.ser.read(n).decode("ascii", "ignore")
        except Exception as e:
            self.log("error al leer serial: %s" % e)
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
# Servidor MJPEG opcional (solo depuracion / calibracion en pista)
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


def arrancar_web(buffer_frame, puerto=8080):
    from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

    PAGINA = (b"<html><head><title>WRO vision</title>"
              b"<style>body{background:#111;color:#eee;font-family:sans-serif;text-align:center}"
              b"img{width:min(90vw,640px);image-rendering:pixelated;border:1px solid #444}</style>"
              b"</head><body><h3>Vision WRO</h3><img src='/stream'></body></html>")

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_GET(self):
            if self.path == "/stream":
                self.send_response(200)
                self.send_header("Content-Type",
                                 "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                try:
                    while True:
                        jpeg = buffer_frame.get()
                        if jpeg is None:
                            time.sleep(0.05)
                            continue
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                         b"Content-Length: %d\r\n\r\n" % len(jpeg))
                        self.wfile.write(jpeg)
                        self.wfile.write(b"\r\n")
                        time.sleep(0.08)  # ~12 fps al navegador, suficiente
                except (BrokenPipeError, ConnectionResetError):
                    pass
            else:
                self.send_response(200)
                self.send_header("Content-Type", "text/html")
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
    ap.add_argument("--puerto-web", type=int, default=8080)
    ap.add_argument("--color", default=None, help="color inicial a buscar")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()

    def log(msg):
        print("[vision] %s" % msg, flush=True)

    cfg = cargar_config(args.config)
    disponibles = colores_disponibles(cfg)
    color = (args.color or cfg["deteccion"].get("color_inicial", disponibles[0])).upper()
    if color not in disponibles and color != "AUTO":
        log("color '%s' no definido; usando %s" % (color, disponibles[0]))
        color = disponibles[0]

    detector = Detector(cfg)
    
    gestor = GestorExclusividadCamara(nombre_script="vision_server.py", gestionar_servicio=False)
    gestor.adquirir()
    
    cam = Camara(cfg["camara"]).abrir()
    log("camara %dx%d abierta, buscando %s" % (cam.ancho, cam.alto, color))

    enlace = None
    if cfg["serial"].get("habilitado", True) and not args.sin_serial:
        try:
            enlace = EnlaceSerial(cfg["serial"], log).abrir()
        except Exception as e:
            log("AVISO: no se pudo abrir el serial (%s). Sigo sin la MegaPi." % e)

    buffer_frame = None
    if args.web:
        buffer_frame = BufferFrame()
        arrancar_web(buffer_frame, args.puerto_web)
        log("video de depuracion en http://<ip-de-la-pi>:%d" % args.puerto_web)

    fps = ContadorFPS()
    hz_envio = cfg["serial"].get("hz_envio", 20)
    # El 0.9 evita un aliasing feo: si la camara entrega a 20.8 fps (48 ms) y
    # el periodo de envio es exactamente 50 ms, un frame de cada dos llega
    # "demasiado pronto" y se descarta, quedando en 10 tramas/s. Con hz_envio=0
    # se envia en cada frame.
    periodo_envio = 0.0 if hz_envio <= 0 else 0.9 / hz_envio
    ultimo_envio = 0.0
    enviando = True

    try:
        while True:
            # solo_nuevos: el bucle corre al ritmo de la camara en vez de
            # reprocesar la misma imagen y quemar CPU para nada.
            frame = cam.leer(solo_nuevos=True, timeout=0.5)
            if frame is None:
                log("sin frames nuevos de la camara")
                continue

            if color == "AUTO":
                det, color_detectado, _mask, roi_y0 = detector.detectar_auto(frame)
            else:
                det, _mask, roi_y0 = detector.detectar(frame, color)
                # En modo fijo se conserva el color solicitado incluso cuando
                # found=0. La MegaPi usa ese campo para validar el cambio de
                # modo; NINGUNO queda reservado para AUTO sin candidato.
                color_detectado = color
            f = fps.tick()

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

            # Comandos que llegan de la MegaPi
            if enlace:
                for linea in enlace.leer_lineas():
                    if linea.startswith("#"):
                        log("MegaPi: %s" % linea[1:].strip())
                        continue
                    partes = linea.split()
                    cmd = partes[0].upper()
                    if cmd == "C" and len(partes) >= 2:
                        nuevo = partes[1].upper()
                        if nuevo in disponibles or nuevo == "AUTO":
                            color = nuevo
                            log("color -> %s" % color)
                            enlace.enviar("K COLOR %s" % color)
                        else:
                            enlace.enviar("E COLOR")
                    elif cmd == "X":
                        # Escaneo de los cinco colores sobre el frame actual.
                        # Cuesta unos 20 ms; se usa una vez al inicio de la
                        # ronda, no en el lazo de control.
                        partes_resp = []
                        for c in disponibles:
                            dc, _, _ = detector.detectar(frame, c)
                            partes_resp.append("%s %d %d %d" % (
                                c, 1 if dc.encontrado else 0, dc.ex, dc.dist_mm))
                        enlace.enviar("X " + " ".join(partes_resp))
                    elif cmd == "S" and len(partes) >= 2:
                        enviando = partes[1] != "0"
                        enlace.enviar("K STREAM %d" % (1 if enviando else 0))
                    elif cmd == "P":
                        enlace.enviar("K %s AUTO,%s" % (VERSION, ",".join(disponibles)))

            if buffer_frame is not None:
                color_overlay = color_detectado if color == "AUTO" else color
                buffer_frame.set(dibujar_overlay(frame.copy(), det, color_overlay, f,
                                                 roi_y0, detector.cx_garra))

    except KeyboardInterrupt:
        log("detenido por el usuario")
    finally:
        cam.cerrar()
        gestor.liberar()
        if enlace:
            enlace.cerrar()


if __name__ == "__main__":
    sys.exit(main())
