#!/usr/bin/env python3
"""
Nucleo de vision para el robot WRO RoboMission 2026.

Pensado para una Raspberry Pi 3B (ARMv7, 1 GB): resolucion baja, MJPG,
una sola conversion a HSV por frame y una sola mascara (solo el color activo).
Con 320x240 en una Pi 3B esto corre entre 25 y 30 FPS usando ~35% de un nucleo.

Este modulo no habla serial ni HTTP; solo captura y detecta.
"""

import json
import glob
import os
import subprocess
import sys
import threading
import time
import atexit

try:
    import fcntl
except ImportError:
    fcntl = None

import cv2
import numpy as np

# OpenCV en la Pi 3B: mas hilos no ayuda en operaciones tan pequenas y
# compite con el proceso de captura. Dos es el mejor compromiso medido.
cv2.setNumThreads(2)

RUTA_CONFIG_POR_DEFECTO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config.json")


def resolver_dispositivo_video(cfg_cam):
    """Resuelve una camara por ruta persistente y conserva fallback legado."""
    dispositivo = cfg_cam.get("dispositivo")
    if dispositivo and os.path.exists(dispositivo):
        return os.path.realpath(dispositivo)

    patron = cfg_cam.get("patron_by_id")
    if patron:
        candidatos = sorted(glob.glob(patron))
        if candidatos:
            return os.path.realpath(candidatos[0])

    if dispositivo:
        return dispositivo

    indice = cfg_cam.get("indice", 0)
    if isinstance(indice, str) and indice.startswith("/dev/"):
        return indice
    return "/dev/video%d" % int(indice)


# ---------------------------------------------------------------------------
# Bloqueo y Exclusividad
# ---------------------------------------------------------------------------

class GestorExclusividadCamara:
    """
    Evita que dos scripts abran /dev/video0 a la vez y corrompan el streaming (Corrupt JPEG data).
    Tambien puede detener 'wro-vision' (si esta corriendo) y restaurarlo al salir.
    """
    _instancia_activa = None

    def __init__(self, nombre_script="script", gestionar_servicio=False):
        self.nombre_script = nombre_script
        self.gestionar_servicio = gestionar_servicio
        self.lockfile = "/tmp/wro_camara.lock"
        self.fd_lock = None
        self.servicio_detenido = False

    def __enter__(self):
        self.adquirir()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.liberar()

    def adquirir(self):
        if self.gestionar_servicio and self._servicio_activo():
            print("[GestorCamara] El servicio 'wro-vision' esta usando la camara.", flush=True)
            print("[GestorCamara] Deteniendo temporalmente el servicio...", flush=True)
            subprocess.run(["sudo", "systemctl", "stop", "wro-vision"], timeout=5)
            self.servicio_detenido = True
            time.sleep(0.3)

        if fcntl:
            self.fd_lock = os.open(self.lockfile, os.O_RDWR | os.O_CREAT, 0o666)
            try:
                fcntl.flock(self.fd_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                info = json.dumps({"pid": os.getpid(), "script": self.nombre_script})
                os.ftruncate(self.fd_lock, 0)
                os.write(self.fd_lock, info.encode("utf-8"))
                os.fsync(self.fd_lock)
            except (IOError, OSError):
                # Ya esta bloqueado por otro proceso
                try:
                    os.lseek(self.fd_lock, 0, os.SEEK_SET)
                    datos = os.read(self.fd_lock, 1024).decode("utf-8")
                    info = json.loads(datos)
                    pid = info.get("pid", "?")
                    script = info.get("script", "desconocido")
                except Exception:
                    pid = "?"
                    script = "desconocido"
                
                print("\n" + "="*70)
                print(f" ERROR: La camara ya esta en uso por otro proceso (PID: {pid}, Script: {script})")
                print(" No se permite ejecucion simultanea para evitar corromper el video.")
                print(" Cierra primero el otro script o pestaña de calibracion.")
                print("="*70 + "\n", flush=True)
                sys.exit(1)

        GestorExclusividadCamara._instancia_activa = self
        atexit.register(self.liberar)

    def liberar(self):
        if self.fd_lock is not None:
            if fcntl:
                try:
                    fcntl.flock(self.fd_lock, fcntl.LOCK_UN)
                except OSError:
                    pass
            try:
                os.close(self.fd_lock)
            except OSError:
                pass
            self.fd_lock = None
            try:
                os.unlink(self.lockfile)
            except OSError:
                pass

        if self.servicio_detenido:
            print("[GestorCamara] Restaurando servicio 'wro-vision'...", flush=True)
            subprocess.run(["sudo", "systemctl", "start", "wro-vision"], timeout=5)
            self.servicio_detenido = False
            time.sleep(0.2)
            
        GestorExclusividadCamara._instancia_activa = None

    @staticmethod
    def _servicio_activo():
        try:
            r = subprocess.run(["systemctl", "is-active", "wro-vision"],
                               capture_output=True, text=True, timeout=2)
            return r.stdout.strip() == "active"
        except Exception:
            return False

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
        self.dispositivo = resolver_dispositivo_video(cfg_cam)
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
        self._estado = "CERRADA"
        self._ultimo_error = ""
        self._reconexiones = 0

    def _crear_captura(self):
        self.dispositivo = resolver_dispositivo_video(self.cfg)
        cap = cv2.VideoCapture(self.dispositivo, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            raise RuntimeError("No se pudo abrir la camara %s" % self.dispositivo)

        fourcc = self.cfg.get("fourcc", "MJPG")
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.ancho)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.alto)
        cap.set(cv2.CAP_PROP_FPS, self.cfg.get("fps", 30))
        cap.set(cv2.CAP_PROP_BUFFERSIZE, self.cfg.get("buffers", 3))
        return cap

    def abrir(self):
        self.cap = self._crear_captura()
        self._estado = "ABRIENDO"

        # Primero se estabiliza el stream. Varias camaras UVC ignoran o
        # reinician controles si se aplican antes del primer frame.
        for _ in range(3):
            self.cap.read()

        self._aplicar_controles_v4l2(self.dispositivo)

        self._corriendo = True
        self._hilo = threading.Thread(target=self._bucle_captura, daemon=True)
        self._hilo.start()

        # Esperar al primer frame para que quien nos llame no reciba None.
        t0 = time.time()
        while self._frame is None and time.time() - t0 < 5.0:
            time.sleep(0.02)
        if self._frame is None:
            raise RuntimeError("La camara no entrego ningun frame en 5 s")
        self._estado = "CONECTADA"
        return self

    def _bucle_captura(self):
        fallos = 0
        try:
            while self._corriendo:
                if self.cap is None or not self.cap.isOpened():
                    self._estado = "RECONECTANDO"
                    try:
                        self.cap = self._crear_captura()
                        ok = False
                        for _ in range(3):
                            ok, frame = self.cap.read()
                            if ok:
                                break
                        if not ok:
                            raise RuntimeError("sin frames tras reabrir")
                        self._aplicar_controles_v4l2(self.dispositivo)
                        self._reconexiones += 1
                        self._ultimo_error = ""
                        self._estado = "CONECTADA"
                        fallos = 0
                    except Exception as e:
                        self._ultimo_error = str(e)
                        if self.cap is not None:
                            self.cap.release()
                            self.cap = None
                        time.sleep(self.cfg.get("reintento_s", 1.0))
                        continue
                ok, frame = self.cap.read()
                if not self._corriendo:
                    break
                if not ok:
                    fallos += 1
                    if fallos >= self.cfg.get("fallos_para_reabrir", 8):
                        self._ultimo_error = "la camara dejo de entregar frames"
                        self._estado = "RECONECTANDO"
                        self.cap.release()
                        self.cap = None
                        with self._lock:
                            self._frame = None
                    continue
                fallos = 0
                self._estado = "CONECTADA"
                if self.voltear:
                    frame = cv2.rotate(frame, cv2.ROTATE_180)
                with self._lock:
                    self._frame = frame
                    self._seq += 1
        except Exception:
            # Al cerrar VideoCapture, algunos drivers lanzan una excepcion en
            # el hilo lector. El cierre normal no debe tumbar el proceso.
            pass
        finally:
            if self.cap is not None:
                try:
                    self.cap.release()
                except Exception:
                    pass
                self.cap = None

    def _aplicar_controles_v4l2(self, dev=None):
        c = self.cfg
        dev = dev or self.dispositivo
        controles = []

        if c.get("auto_exposicion") is True:
            controles.append("auto_exposure=3")
        elif c.get("auto_exposicion") is False:
            # 1 = manual, 3 = aperture priority (automatico) en UVC
            controles.append("auto_exposure=1")
            if c.get("exposicion_absoluta") is not None:
                controles.append("exposure_time_absolute=%d" % c["exposicion_absoluta"])
        if c.get("auto_balance_blancos") is True:
            controles.append("white_balance_automatic=1")
        elif c.get("auto_balance_blancos") is False:
            controles.append("white_balance_automatic=0")
            if c.get("temperatura_color") is not None:
                controles.append("white_balance_temperature=%d" % c["temperatura_color"])

        for nombre_cfg, nombre_v4l2 in (("brillo", "brightness"),
                                        ("contraste", "contrast"),
                                        ("saturacion", "saturation"),
                                        ("ganancia", "gain"),
                                        ("gamma", "gamma"),
                                        ("nitidez", "sharpness"),
                                        ("sharpness", "sharpness"),
                                        ("compensacion_contraluz", "backlight_compensation")):
            if c.get(nombre_cfg) is not None:
                controles.append("%s=%d" % (nombre_v4l2, int(c[nombre_cfg])))

        # 50 o 60 Hz segun la red electrica del pais. Mal puesto, las luces
        # LED/fluorescentes de la sede meten bandas que mueven el HSV.
        if c.get("frecuencia_red_hz") is not None:
            hz = int(c["frecuencia_red_hz"])
            if hz in (0, 50, 60):
                controles.append("power_line_frequency=%d"
                                 % (2 if hz == 60 else (1 if hz == 50 else 0)))

        if not controles:
            return

        for ctrl in controles:
            self._aplicar_ctrl_individual(dev, ctrl)

    @staticmethod
    def _aplicar_ctrl_individual(dev, ctrl):
        """Aplica un control y prueba el alias usado por kernels antiguos."""
        alias = {
            "auto_exposure": "exposure_auto",
            "exposure_time_absolute": "exposure_absolute",
            "white_balance_automatic": "white_balance_temperature_auto",
        }
        if not Camara._v4l2(dev, ctrl):
            clave, _, valor = ctrl.partition("=")
            if clave in alias:
                Camara._v4l2(dev, "%s=%s" % (alias[clave], valor))

    def aplicar_control(self, clave_cfg, valor):
        """Aplica un control de camara en caliente (usado por calibradores)."""
        self.cfg[clave_cfg] = valor
        if clave_cfg == "voltear_180":
            self.voltear = bool(valor)
            return True

        dev = self.dispositivo
        if clave_cfg == "auto_exposicion":
            self._aplicar_ctrl_individual(dev, "auto_exposure=%d" % (3 if valor else 1))
            if not valor and self.cfg.get("exposicion_absoluta") is not None:
                self._aplicar_ctrl_individual(
                    dev, "exposure_time_absolute=%d" % self.cfg["exposicion_absoluta"])
            return True
        if clave_cfg == "exposicion_absoluta":
            if not self.cfg.get("auto_exposicion", False):
                self._aplicar_ctrl_individual(dev, "auto_exposure=1")
                self._aplicar_ctrl_individual(
                    dev, "exposure_time_absolute=%d" % int(valor))
            return True
        if clave_cfg == "auto_balance_blancos":
            self._aplicar_ctrl_individual(
                dev, "white_balance_automatic=%d" % (1 if valor else 0))
            if not valor and self.cfg.get("temperatura_color") is not None:
                self._aplicar_ctrl_individual(
                    dev, "white_balance_temperature=%d" % self.cfg["temperatura_color"])
            return True
        if clave_cfg == "temperatura_color":
            if not self.cfg.get("auto_balance_blancos", False):
                self._aplicar_ctrl_individual(dev, "white_balance_automatic=0")
                self._aplicar_ctrl_individual(
                    dev, "white_balance_temperature=%d" % int(valor))
            return True

        mapeo = {
            "brillo": "brightness",
            "contraste": "contrast",
            "saturacion": "saturation",
            "ganancia": "gain",
            "gamma": "gamma",
            "nitidez": "sharpness",
            "sharpness": "sharpness",
            "compensacion_contraluz": "backlight_compensation",
        }
        if clave_cfg in mapeo:
            self._aplicar_ctrl_individual(
                dev, "%s=%d" % (mapeo[clave_cfg], int(valor)))
            return True
        if clave_cfg == "frecuencia_red_hz":
            hz = int(valor)
            v = 2 if hz == 60 else (1 if hz == 50 else 0)
            self._aplicar_ctrl_individual(dev, "power_line_frequency=%d" % v)
            return True
        return False

    def actualizar_cfg(self, nueva_cfg):
        for clave, valor in nueva_cfg.items():
            self.aplicar_control(clave, valor)

    @staticmethod
    def leer_controles_v4l2(dev="/dev/video0"):
        """Lee los valores efectivos de los controles soportados por V4L2."""
        try:
            r = subprocess.run(["v4l2-ctl", "-d", dev, "-l"],
                               capture_output=True, text=True, timeout=3)
            if r.returncode != 0:
                return {}
            valores = {}
            for linea in r.stdout.splitlines():
                linea = linea.strip()
                if not linea or ":" not in linea:
                    continue
                nombre, resto = linea.split(":", 1)
                nombre = nombre.split()[0]
                for parte in resto.split():
                    if parte.startswith("value="):
                        try:
                            valores[nombre] = int(parte.split("=", 1)[1])
                        except ValueError:
                            pass
            return valores
        except (FileNotFoundError, subprocess.TimeoutExpired):
            return {}

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
        self._estado = "CERRANDO"
        if self._hilo is not None:
            self._hilo.join(timeout=1.0)
            self._hilo = None
        if self.cap is not None:
            self.cap.release()
            self.cap = None
        self._estado = "CERRADA"

    def estado(self):
        return {
            "estado": self._estado,
            "dispositivo": self.dispositivo,
            "reconexiones": self._reconexiones,
            "ultimo_error": self._ultimo_error,
        }


# ---------------------------------------------------------------------------
# Deteccion
# ---------------------------------------------------------------------------

class Deteccion:
    __slots__ = ("encontrado", "color", "cx", "cy", "y_base", "area",
                 "w", "h", "dist_mm", "ex", "ey", "confianza")

    def __init__(self):
        self.encontrado = False
        self.color = "NINGUNO"
        self.cx = 0
        self.cy = 0
        self.y_base = 0
        self.area = 0
        self.w = 0
        self.h = 0
        self.dist_mm = -1
        self.ex = 0
        self.ey = 0
        self.confianza = 0


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
        self.y_base_max_frac = det.get("y_base_max_frac", 1.0)
        self.auto_ex_max_px = det.get("auto_ex_max_px", 110)
        self.zonas_ignoradas = det.get("zonas_ignoradas", [])
        self.fila_cfg = cfg.get("fila_artefactos", {})
        self.destino_cfg = cfg.get("destino", {})
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

    def _preparar_hsv(self, frame):
        alto = frame.shape[0]
        y0 = int(alto * self.roi_y[0])
        y1 = int(alto * self.roi_y[1])
        roi = frame[y0:y1]
        return cv2.cvtColor(roi, cv2.COLOR_BGR2HSV), y0, y1

    def mascara(self, frame, color, hsv_roi=None, y0=None, y1=None):
        """Devuelve la mascara del color, anulando partes fijas del robot."""
        if hsv_roi is None:
            hsv_roi, y0, y1 = self._preparar_hsv(frame)
        rangos = self.rangos(color)
        mask = cv2.inRange(hsv_roi, rangos[0][0], rangos[0][1])
        for bajo, alto_r in rangos[1:]:
            mask |= cv2.inRange(hsv_roi, bajo, alto_r)

        # Los dedos azules de la garra aparecen siempre abajo. Las zonas se
        # expresan como [x0,y0,x1,y1] normalizadas para sobrevivir a cambios
        # de resolucion y se recortan solo dentro de la ROI activa.
        alto, ancho = frame.shape[:2]
        for zona in self.zonas_ignoradas:
            if len(zona) != 4:
                continue
            x0 = max(0, min(ancho, int(float(zona[0]) * ancho)))
            x1 = max(0, min(ancho, int(float(zona[2]) * ancho)))
            zy0 = max(y0, min(y1, int(float(zona[1]) * alto))) - y0
            zy1 = max(y0, min(y1, int(float(zona[3]) * alto))) - y0
            if x1 > x0 and zy1 > zy0:
                mask[zy0:zy1, x0:x1] = 0

        # OPEN quita el ruido de pixeles sueltos; CLOSE cierra los huecos que
        # deja el brillo especular sobre los cubos de plastico.
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
        return mask, y0

    def candidatos(self, frame, color, hsv_roi=None, y0=None, y1=None,
                   reglas_extra=None):
        """Devuelve todos los contornos plausibles, ordenados por calidad."""
        if hsv_roi is None:
            hsv_roi, y0, y1 = self._preparar_hsv(frame)
        mask, y0 = self.mascara(frame, color, hsv_roi, y0, y1)
        r = dict(self.reglas.get(color, {}))
        if reglas_extra:
            r.update(reglas_extra)
        area_min = r.get("area_min_px", self.area_min)
        area_max = r.get("area_max_px")
        alto_min = r.get("alto_min_px", self.alto_min)
        aspecto_max = r.get("relacion_aspecto_max", self.aspecto_max)
        y_base_max = int(frame.shape[0] * r.get(
            "y_base_max_frac", self.y_base_max_frac))

        contornos = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        candidatos = []
        for c in contornos:
            area = cv2.contourArea(c)
            if area < area_min:
                continue
            if area_max is not None and area > area_max:
                continue
            x, y, w, h = cv2.boundingRect(c)
            if w == 0 or h < alto_min:
                continue
            aspecto = max(w / float(h), h / float(w))
            if aspecto > aspecto_max:
                continue  # tiras largas = linea del suelo o reflejo, no un objeto
            y_base = y + h + y0
            if y_base > y_base_max:
                continue

            if color == "AZUL" and r.get("rechazar_inferior_lateral", False):
                frac_inferior = r.get("inferior_desde_frac", 0.68)
                centro_max = r.get("centro_max_frac", 0.16) * frame.shape[1]
                if y_base >= frame.shape[0] * frac_inferior and \
                        abs((x + w // 2) - self.cx_garra) > centro_max:
                    continue

            d = Deteccion()
            d.encontrado = True
            d.color = color
            d.cx = x + w // 2
            d.cy = y + h // 2 + y0
            d.y_base = y_base
            d.area = int(area)
            d.w, d.h = w, h
            d.ex = d.cx - self.cx_garra
            d.ey = y_base
            d.dist_mm = self.distancia_mm(y_base)

            # Calidad: se prioriza el objeto frente a la garra, luego una
            # silueta compacta y finalmente un area suficientemente grande.
            medio_ancho = max(frame.shape[1] / 2.0, 1.0)
            q_centro = max(0.0, 1.0 - abs(d.ex) / medio_ancho)
            q_forma = max(0.0, 1.0 - (aspecto - 1.0) /
                          max(aspecto_max - 1.0, 0.01))
            q_area = min(1.0, area / max(float(area_min) * 6.0, 1.0))
            d.confianza = int(round(100.0 *
                                    (0.60 * q_centro + 0.25 * q_forma + 0.15 * q_area)))
            candidatos.append(d)

        candidatos.sort(key=lambda d: (d.confianza, -abs(d.ex), d.area), reverse=True)
        return candidatos, mask, y0

    def detectar(self, frame, color, hsv_roi=None, y0=None, y1=None):
        candidatos, mask, y0 = self.candidatos(frame, color, hsv_roi, y0, y1)
        return (candidatos[0] if candidatos else Deteccion()), mask, y0

    def detectar_auto(self, frame):
        """Reconoce el artefacto alineado con la garra.

        Los colores cromaticos tienen prioridad. NEGRO se evalua solo cuando
        no hay uno de ellos centrado, porque los cuatro cuadros de salida y
        las lineas de la pista tambien son negros.
        """
        hsv_roi, y0, y1 = self._preparar_hsv(frame)
        colores = colores_disponibles(self.cfg)
        cromaticos = [c for c in colores if c != "NEGRO"]
        mejores = []
        mascaras = {}
        for color in cromaticos:
            d, mask, _ = self.detectar(frame, color, hsv_roi, y0, y1)
            mascaras[color] = mask
            if d.encontrado and abs(d.ex) <= self.auto_ex_max_px:
                mejores.append(d)

        if mejores:
            mejores.sort(key=lambda d: (abs(d.ex), -d.confianza, -d.area))
            d = mejores[0]
            return d, d.color, mascaras[d.color], y0

        if "NEGRO" in colores:
            d, mask, _ = self.detectar(frame, "NEGRO", hsv_roi, y0, y1)
            if d.encontrado and abs(d.ex) <= self.auto_ex_max_px:
                return d, "NEGRO", mask, y0
            return Deteccion(), "NINGUNO", mask, y0

        vacia = np.zeros(hsv_roi.shape[:2], dtype=np.uint8)
        return Deteccion(), "NINGUNO", vacia, y0

    def detectar_fila_artefactos(self, frame):
        """Detecta la fila completa solo desde la pose central esperada."""
        y_frac = self.fila_cfg.get("roi_y", [0.18, 0.68])
        y_min = int(frame.shape[0] * y_frac[0])
        y_max = int(frame.shape[0] * y_frac[1])
        hsv_roi, y0, y1 = self._preparar_hsv(frame)
        posibles = []

        for color in colores_disponibles(self.cfg):
            reglas_fila = self.fila_cfg.get("reglas_color", {}).get(color, {})
            candidatos, _, _ = self.candidatos(
                frame, color, hsv_roi, y0, y1, reglas_extra=reglas_fila)
            for d in candidatos:
                if y_min <= d.cy <= y_max and d.y_base <= y_max:
                    posibles.append(d)

        # Un detalle amarillo dentro de un artefacto verde no es otro slot.
        # Se agrupan centros cercanos y se conserva la silueta de mayor area.
        posibles.sort(key=lambda d: d.area, reverse=True)
        grupos = []
        separacion = self.fila_cfg.get("separacion_duplicado_frac", 0.08) * frame.shape[1]
        for d in posibles:
            if any(abs(d.cx - g.cx) < separacion and
                   abs(d.cy - g.cy) < separacion for g in grupos):
                continue
            grupos.append(d)

        grupos.sort(key=lambda d: d.cx)
        max_slots = int(self.fila_cfg.get("max_slots", 4))
        if len(grupos) > max_slots:
            grupos = sorted(grupos, key=lambda d: d.area, reverse=True)[:max_slots]
            grupos.sort(key=lambda d: d.cx)
        return grupos

    def detectar_destino(self, frame, color):
        """Detecta el cuadro plano del museo por color, forma y marco blanco."""
        cfg = self.destino_cfg
        alto, ancho = frame.shape[:2]
        roi_y = cfg.get("roi_y", [0.04, 0.70])
        y0 = max(0, int(alto * roi_y[0]))
        y1 = min(alto, int(alto * roi_y[1]))
        hsv = cv2.cvtColor(frame[y0:y1], cv2.COLOR_BGR2HSV)
        rangos_cfg = self.cfg.get("colores_destino", {}).get(color)
        rangos = _rangos_a_arrays(rangos_cfg) if rangos_cfg else self.rangos(color)
        mask = cv2.inRange(hsv, rangos[0][0], rangos[0][1])
        for bajo, alto_r in rangos[1:]:
            mask |= cv2.inRange(hsv, bajo, alto_r)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)

        area_min = cfg.get("area_min_px", 220)
        aspecto_min = cfg.get("relacion_aspecto_min", 0.45)
        aspecto_max = cfg.get("relacion_aspecto_max", 2.2)
        relleno_min = cfg.get("relleno_min", 0.58)
        blanco_min = cfg.get("marco_blanco_min", 0.12)
        candidatos = []

        contornos = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                     cv2.CHAIN_APPROX_SIMPLE)[-2]
        for c in contornos:
            area = cv2.contourArea(c)
            if area < area_min:
                continue
            x, y, w, h = cv2.boundingRect(c)
            if w == 0 or h == 0:
                continue
            # El borde superior suele ser fondo lejano o pared.
            # En el borde inferior, si el área es grande (objeto cercano al depositar),
            # no lo descartamos para no perderlo al aproximar.
            if y <= 1:
                continue
            if (y + h >= (y1 - y0) - 1) and area < area_min * 2.5:
                continue

            aspecto = w / float(h)
            relleno = area / float(w * h)
            if not aspecto_min <= aspecto <= aspecto_max or relleno < relleno_min:
                continue

            margen = max(6, int(max(w, h) * cfg.get("margen_marco_frac", 0.45)))
            rx0, ry0 = max(0, x - margen), max(0, y - margen)
            rx1, ry1 = min(ancho, x + w + margen), min(y1 - y0, y + h + margen)
            anillo = np.ones((ry1 - ry0, rx1 - rx0), dtype=np.uint8)
            anillo[y - ry0:y + h - ry0, x - rx0:x + w - rx0] = 0
            hsv_anillo = hsv[ry0:ry1, rx0:rx1]
            blanco = ((hsv_anillo[:, :, 1] <= cfg.get("blanco_s_max", 70)) &
                      (hsv_anillo[:, :, 2] >= cfg.get("blanco_v_min", 120)) &
                      (anillo > 0))
            total_anillo = max(int(np.count_nonzero(anillo)), 1)
            frac_blanco = np.count_nonzero(blanco) / float(total_anillo)
            if blanco_min > 0.0 and frac_blanco < blanco_min:
                continue

            perimetro = cv2.arcLength(c, True)
            vertices = len(cv2.approxPolyDP(c, 0.04 * perimetro, True))
            q_forma = min(1.0, relleno)
            q_marco = (min(1.0, frac_blanco / max(blanco_min * 2.5, 0.01))
                       if blanco_min > 0.0 else 0.8)
            q_cuatro = 1.0 if 4 <= vertices <= 6 else 0.4
            q_centro = max(0.0, 1.0 - abs((x + w / 2.0) - self.cx_garra) /
                           max(ancho / 2.0, 1.0))

            d = Deteccion()
            d.encontrado = True
            d.color = color
            d.cx = x + w // 2
            d.cy = y + h // 2 + y0
            d.y_base = y + h + y0
            d.area = int(area)
            d.w, d.h = w, h
            d.ex = d.cx - self.cx_garra
            d.ey = d.y_base
            d.dist_mm = self.distancia_destino_mm(d.y_base)
            d.confianza = int(round(100 * (0.30 * q_forma + 0.35 * q_marco +
                                           0.20 * q_cuatro + 0.15 * q_centro)))
            candidatos.append(d)

        if candidatos:
            # El ruido cromatico puede formar cuadritos pequeños con buena
            # forma y cercanos al centro. No deben ganar por confianza a un
            # cuadro de destino mucho mayor que tambien paso todos los filtros.
            area_mayor = max(d.area for d in candidatos)
            fraccion = cfg.get("fraccion_area_maxima_min", 0.30)
            candidatos = [d for d in candidatos
                           if d.area >= area_mayor * fraccion]
            candidatos.sort(
                key=lambda d: (d.confianza, -abs(d.ex), d.area), reverse=True)
        return (candidatos[0] if candidatos else Deteccion()), mask, y0

    def distancia_mm(self, y_base):
        """Interpola la tabla de calibracion. Fuera de rango hace clamp."""
        if len(self._tab_y) < 2:
            return -1
        return int(np.interp(float(y_base), self._tab_y, self._tab_d))

    def distancia_destino_mm(self, y_base):
        tabla = self.destino_cfg.get("tabla_distancia", [])
        if len(tabla) < 2:
            return self.distancia_mm(y_base)
        tabla = sorted(tabla, key=lambda p: p[0])
        ys = np.array([p[0] for p in tabla], dtype=np.float32)
        ds = np.array([p[1] for p in tabla], dtype=np.float32)
        return int(np.interp(float(y_base), ys, ds))


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
        txt = "ex=%+d d=%dmm a=%d q=%d" % (
            det.ex, det.dist_mm, det.area, det.confianza)
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
