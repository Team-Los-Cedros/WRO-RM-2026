#!/usr/bin/env python3
"""
Nucleo de vision para el robot WRO RoboMission 2026.

Pensado para una Raspberry Pi 3B (ARMv7, 1 GB): resolucion baja, MJPG,
una sola conversion a HSV por frame y una sola mascara (solo el color activo).
Con 320x240 en una Pi 3B esto corre entre 25 y 30 FPS usando ~35% de un nucleo.

Este modulo no habla serial ni HTTP; solo captura y detecta.
"""

import json
import os
import subprocess
import threading
import time

import cv2
import numpy as np

# OpenCV en la Pi 3B: mas hilos no ayuda en operaciones tan pequenas y
# compite con el proceso de captura. Dos es el mejor compromiso medido.
cv2.setNumThreads(2)

RUTA_CONFIG_POR_DEFECTO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config.json")


# ---------------------------------------------------------------------------
# Configuracion
# ---------------------------------------------------------------------------

def cargar_config(ruta=None):
    ruta = ruta or RUTA_CONFIG_POR_DEFECTO
    with open(ruta, "r", encoding="utf-8") as f:
        return json.load(f)


def guardar_config(cfg, ruta=None):
    """Guarda de forma atomica para no corromper el archivo si se corta la luz."""
    ruta = ruta or RUTA_CONFIG_POR_DEFECTO
    tmp = ruta + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(cfg, f, indent=2, ensure_ascii=False)
    os.replace(tmp, ruta)


def colores_disponibles(cfg):
    return [k for k in cfg["colores"] if not k.startswith("_")]


# ---------------------------------------------------------------------------
# Camara
# ---------------------------------------------------------------------------

class Camara:
    """Webcam USB via V4L2, con hilo de captura.

    Dos cosas medidas en la Pi 3B con la camara USB 32e6:9221 de este robot:

    1) CAP_PROP_BUFFERSIZE = 1 PARTE EL FRAMERATE A LA MITAD (20 -> 10 fps).
       Con un solo buffer encolado, mientras el programa procesa el frame N el
       driver no tiene donde escribir el N+1 y lo descarta. Hay que dejar
       varios buffers y quitar la latencia por otro lado (el hilo de abajo).

    2) El hilo lector consume siempre el ultimo frame disponible, asi que la
       latencia queda en un frame aunque el bucle de control se ralentice, y
       el bucle principal nunca se bloquea esperando a la camara mientras
       atiende el serial.

    Ademas fija exposicion y balance de blancos con v4l2-ctl: si la camara los
    deja en automatico, el HSV del objeto cambia solo con moverse por la pista
    y los rangos calibrados dejan de servir. Es la causa numero uno de que una
    deteccion por color "funcione en casa y falle en la competencia".
    """

    def __init__(self, cfg_cam):
        self.cfg = cfg_cam
        self.indice = cfg_cam.get("indice", 0)
        self.ancho = cfg_cam.get("ancho", 320)
        self.alto = cfg_cam.get("alto", 240)
        self.voltear = cfg_cam.get("voltear_180", False)
        self.cap = None
        self._hilo = None
        self._corriendo = False
        self._lock = threading.Lock()
        self._frame = None
        self._seq = 0
        self._ultimo_seq_entregado = -1

    def abrir(self):
        self.cap = cv2.VideoCapture(self.indice, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(
                "No se pudo abrir la camara /dev/video%s. "
                "Revisa con: v4l2-ctl --list-devices" % self.indice
            )

        fourcc = self.cfg.get("fourcc", "MJPG")
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.ancho)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.alto)
        self.cap.set(cv2.CAP_PROP_FPS, self.cfg.get("fps", 30))
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, self.cfg.get("buffers", 3))

        self._aplicar_controles_v4l2()

        # Descartar los primeros frames: suelen salir negros o mal expuestos.
        for _ in range(5):
            self.cap.read()

        self._corriendo = True
        self._hilo = threading.Thread(target=self._bucle_captura, daemon=True)
        self._hilo.start()

        # Esperar al primer frame para que quien nos llame no reciba None.
        t0 = time.time()
        while self._frame is None and time.time() - t0 < 5.0:
            time.sleep(0.02)
        if self._frame is None:
            raise RuntimeError("La camara no entrego ningun frame en 5 s")
        return self

    def _bucle_captura(self):
        fallos = 0
        while self._corriendo:
            ok, frame = self.cap.read()
            if not ok:
                fallos += 1
                if fallos > 100:
                    time.sleep(0.1)
                continue
            fallos = 0
            if self.voltear:
                frame = cv2.rotate(frame, cv2.ROTATE_180)
            with self._lock:
                self._frame = frame
                self._seq += 1

    def _aplicar_controles_v4l2(self):
        c = self.cfg
        dev = "/dev/video%s" % self.indice
        controles = []

        if c.get("auto_exposicion") is False:
            # 1 = manual, 3 = aperture priority (automatico) en UVC
            controles.append("auto_exposure=1")
            if c.get("exposicion_absoluta") is not None:
                controles.append("exposure_time_absolute=%d" % c["exposicion_absoluta"])
        if c.get("auto_balance_blancos") is False:
            controles.append("white_balance_automatic=0")
            if c.get("temperatura_color") is not None:
                controles.append("white_balance_temperature=%d" % c["temperatura_color"])
        for nombre_cfg, nombre_v4l2 in (("brillo", "brightness"),
                                        ("contraste", "contrast"),
                                        ("saturacion", "saturation"),
                                        ("ganancia", "gain")):
            if c.get(nombre_cfg) is not None:
                controles.append("%s=%d" % (nombre_v4l2, c[nombre_cfg]))

        # 50 o 60 Hz segun la red electrica del pais. Mal puesto, las luces
        # LED/fluorescentes de la sede meten bandas que mueven el HSV.
        if c.get("frecuencia_red_hz"):
            controles.append("power_line_frequency=%d"
                             % (2 if int(c["frecuencia_red_hz"]) == 60 else 1))

        if not controles:
            return

        # Los nombres de los controles cambian entre versiones de kernel/UVC
        # (auto_exposure vs exposure_auto). Se aplican uno por uno y se ignoran
        # los que la camara no soporte, en vez de fallar en bloque.
        alias = {
            "auto_exposure": "exposure_auto",
            "exposure_time_absolute": "exposure_absolute",
            "white_balance_automatic": "white_balance_temperature_auto",
        }
        for ctrl in controles:
            if not self._v4l2(dev, ctrl):
                clave, _, valor = ctrl.partition("=")
                if clave in alias:
                    # exposure_auto usa 1=manual igual que auto_exposure
                    self._v4l2(dev, "%s=%s" % (alias[clave], valor))

    @staticmethod
    def _v4l2(dev, ctrl):
        try:
            r = subprocess.run(["v4l2-ctl", "-d", dev, "-c", ctrl],
                               capture_output=True, timeout=3)
            return r.returncode == 0
        except (FileNotFoundError, subprocess.TimeoutExpired):
            return False

    def leer(self, solo_nuevos=False, timeout=0.5):
        """Devuelve el frame mas reciente.

        Con solo_nuevos=True espera a que haya un frame que no se haya
        entregado antes; asi el bucle de control corre al ritmo de la camara
        en vez de reprocesar la misma imagen.
        """
        limite = time.time() + timeout
        while True:
            with self._lock:
                if self._frame is not None and (
                        not solo_nuevos or self._seq != self._ultimo_seq_entregado):
                    self._ultimo_seq_entregado = self._seq
                    return self._frame
            if not solo_nuevos or time.time() > limite:
                return None
            time.sleep(0.002)

    def cerrar(self):
        self._corriendo = False
        if self._hilo is not None:
            self._hilo.join(timeout=1.0)
            self._hilo = None
        if self.cap is not None:
            self.cap.release()
            self.cap = None


# ---------------------------------------------------------------------------
# Deteccion
# ---------------------------------------------------------------------------

class Deteccion:
    __slots__ = ("encontrado", "cx", "cy", "y_base", "area", "w", "h", "dist_mm", "ex", "ey")

    def __init__(self):
        self.encontrado = False
        self.cx = 0
        self.cy = 0
        self.y_base = 0
        self.area = 0
        self.w = 0
        self.h = 0
        self.dist_mm = -1
        self.ex = 0
        self.ey = 0


def _rangos_a_arrays(rangos):
    out = []
    for r in rangos:
        bajo = np.array(r[0:3], dtype=np.uint8)
        alto = np.array(r[3:6], dtype=np.uint8)
        out.append((bajo, alto))
    return out


class Detector:
    def __init__(self, cfg):
        self.cfg = cfg
        det = cfg["deteccion"]
        self.area_min = det.get("area_min_px", 120)
        self.aspecto_max = det.get("relacion_aspecto_max", 4.0)
        self.alto_min = det.get("alto_min_px", 6)
        self.reglas = {k: v for k, v in cfg.get("reglas_color", {}).items()
                       if not k.startswith("_")}
        k = det.get("kernel_morfologia", 3)
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))

        self.roi_y = cfg["camara"].get("roi_y", [0.0, 1.0])
        self.cx_garra = cfg["geometria"]["cx_garra"]

        tabla = sorted(cfg["geometria"]["tabla_distancia"], key=lambda p: p[0])
        self._tab_y = np.array([p[0] for p in tabla], dtype=np.float32)
        self._tab_d = np.array([p[1] for p in tabla], dtype=np.float32)

        self._cache_rangos = {}

    def rangos(self, color):
        if color not in self._cache_rangos:
            self._cache_rangos[color] = _rangos_a_arrays(self.cfg["colores"][color])
        return self._cache_rangos[color]

    def recargar_color(self, color):
        self._cache_rangos.pop(color, None)

    def mascara(self, frame, color):
        """Devuelve (mascara, y0) donde y0 es el offset vertical de la ROI."""
        alto = frame.shape[0]
        y0 = int(alto * self.roi_y[0])
        y1 = int(alto * self.roi_y[1])
        roi = frame[y0:y1]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        rangos = self.rangos(color)
        mask = cv2.inRange(hsv, rangos[0][0], rangos[0][1])
        for bajo, alto_r in rangos[1:]:
            mask |= cv2.inRange(hsv, bajo, alto_r)

        # OPEN quita el ruido de pixeles sueltos; CLOSE cierra los huecos que
        # deja el brillo especular sobre los cubos de plastico.
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
        return mask, y0

    def detectar(self, frame, color):
        d = Deteccion()
        mask, y0 = self.mascara(frame, color)

        # El negro comparte HSV con las sombras y con las lineas de la pista,
        # asi que se le pueden exigir umbrales propios via "reglas_color".
        r = self.reglas.get(color, {})
        area_min = r.get("area_min_px", self.area_min)
        alto_min = r.get("alto_min_px", self.alto_min)
        aspecto_max = r.get("relacion_aspecto_max", self.aspecto_max)

        contornos = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        mejor = None
        mejor_area = 0
        for c in contornos:
            area = cv2.contourArea(c)
            if area < area_min or area <= mejor_area:
                continue
            x, y, w, h = cv2.boundingRect(c)
            if w == 0 or h < alto_min:
                continue
            aspecto = max(w / float(h), h / float(w))
            if aspecto > aspecto_max:
                continue  # tiras largas = linea del suelo o reflejo, no un objeto
            mejor = (x, y, w, h, area)
            mejor_area = area

        if mejor is None:
            return d, mask, y0

        x, y, w, h, area = mejor
        d.encontrado = True
        d.cx = x + w // 2
        d.cy = y + h // 2 + y0
        d.y_base = y + h + y0          # borde inferior = punto de contacto con el suelo
        d.area = int(area)
        d.w, d.h = w, h
        d.ex = d.cx - self.cx_garra
        d.ey = d.y_base
        d.dist_mm = self.distancia_mm(d.y_base)
        return d, mask, y0

    def distancia_mm(self, y_base):
        """Interpola la tabla de calibracion. Fuera de rango hace clamp."""
        if len(self._tab_y) < 2:
            return -1
        return int(np.interp(float(y_base), self._tab_y, self._tab_d))


# ---------------------------------------------------------------------------
# Overlay de depuracion
# ---------------------------------------------------------------------------

def dibujar_overlay(frame, det, color, fps, roi_y0, cx_garra):
    alto, ancho = frame.shape[:2]
    cv2.line(frame, (cx_garra, 0), (cx_garra, alto), (0, 255, 255), 1)
    cv2.line(frame, (0, roi_y0), (ancho, roi_y0), (120, 120, 120), 1)

    if det.encontrado:
        x = det.cx - det.w // 2
        y = det.y_base - det.h
        cv2.rectangle(frame, (x, y), (x + det.w, det.y_base), (0, 255, 0), 2)
        cv2.circle(frame, (det.cx, det.y_base), 4, (0, 0, 255), -1)
        txt = "ex=%+d d=%dmm a=%d" % (det.ex, det.dist_mm, det.area)
    else:
        txt = "sin objeto"

    cv2.putText(frame, "%s %s" % (color, txt), (4, 14),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(frame, "%s %s" % (color, txt), (4, 14),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(frame, "%.1f fps" % fps, (ancho - 62, alto - 6),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)
    return frame


class ContadorFPS:
    def __init__(self, ventana=20):
        self.ventana = ventana
        self.t = time.time()
        self.n = 0
        self.fps = 0.0

    def tick(self):
        self.n += 1
        if self.n >= self.ventana:
            ahora = time.time()
            self.fps = self.n / max(ahora - self.t, 1e-6)
            self.t = ahora
            self.n = 0
        return self.fps
