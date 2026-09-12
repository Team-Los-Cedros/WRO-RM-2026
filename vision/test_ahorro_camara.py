"""Prueba de integracion del ahorro, sin Raspberry, sin camara y sin MegaPi.

Levanta vision_server.main() de verdad con cv2 y pyserial simulados y comprueba
la secuencia de competencia:

  - se duerme sola cuando no hay nadie en la web ni la MegaPi habla,
  - contesta "P" en menos de 300 ms aun dormida (es el timeout real de
    visionComprobar() en la MegaPi),
  - despierta con ese mismo "P" (el que manda el boton de inicio),
  - despierta si alguien abre la web,
  - /ahorro?on=0 la deja encendida para siempre.

El testigo que se mide es el ritmo de tramas "T" por el serial: despierta manda
unas 20 por segundo, dormida solo 2.
"""
import json
import os
import socket
import sys
import tempfile
import threading
import time
import types
import urllib.request

RUTA_VISION = os.path.dirname(os.path.abspath(__file__))
SCRATCH = tempfile.mkdtemp(prefix="wro_prueba_")
CRLF = chr(13) + chr(10)
PUERTO_WEB = 8099
sys.path.insert(0, RUTA_VISION)

# --------------------------------------------------------------------------
# cv2 simulado
# --------------------------------------------------------------------------
cv2 = types.ModuleType("cv2")


class _Frame:
    shape = (360, 640, 3)

    def copy(self):
        return self


class _Buf:
    def tobytes(self):
        return b"j" * 98304        # 96 KB: llena el socket de un cliente lento


class CapturaFalsa:
    def isOpened(self):
        return True

    def set(self, *a):
        return True

    def read(self):
        time.sleep(0.005)
        return True, _Frame()

    def release(self):
        pass


for nombre, valor in (("CAP_V4L2", 200), ("CAP_PROP_FOURCC", 6),
                      ("CAP_PROP_FRAME_WIDTH", 3), ("CAP_PROP_FRAME_HEIGHT", 4),
                      ("CAP_PROP_FPS", 5), ("CAP_PROP_BUFFERSIZE", 38),
                      ("ROTATE_180", 1), ("IMWRITE_JPEG_QUALITY", 1)):
    setattr(cv2, nombre, valor)
cv2.setNumThreads = lambda n: None
cv2.VideoWriter_fourcc = lambda *a: 0
cv2.rotate = lambda f, c: f
cv2.VideoCapture = lambda dev, api=None: CapturaFalsa()
cv2.imencode = lambda ext, img, params=None: (True, _Buf())
cv2.__getattr__ = lambda nombre: (lambda *a, **k: None)   # dibujos: no-op
sys.modules["cv2"] = cv2

# --------------------------------------------------------------------------
# pyserial simulado: RX es lo que manda la MegaPi, TX lo que responde la Pi
# --------------------------------------------------------------------------
RX = bytearray()
TX = bytearray()
CANDADO = threading.Lock()


class SerialFalso:
    def __init__(self, port=None, baudrate=None, timeout=None):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.dtr = False
        self.rts = False
        self.is_open = port is not None

    def open(self):
        self.is_open = True

    @property
    def in_waiting(self):
        with CANDADO:
            return len(RX)

    def read(self, n):
        with CANDADO:
            datos = bytes(RX[:n])
            del RX[:n]
        return datos

    def write(self, datos):
        with CANDADO:
            TX.extend(datos)
        return len(datos)

    def reset_input_buffer(self):
        with CANDADO:
            del RX[:]

    def close(self):
        self.is_open = False


serial_mod = types.ModuleType("serial")
serial_mod.Serial = SerialFalso
sys.modules["serial"] = serial_mod

# numpy simulado: en PAUSA no se detecta nada, solo hace falta para que
# Detector.__init__ construya sus tablas sin reventar.
try:
    import numpy  # noqa: F401
except ImportError:
    np = types.ModuleType("numpy")
    np.array = lambda x, dtype=None: list(x)
    np.float32 = "float32"
    np.uint8 = "uint8"
    np.__getattr__ = lambda nombre: (lambda *a, **k: None)
    sys.modules["numpy"] = np


def megapi_manda(linea):
    with CANDADO:
        RX.extend((linea + "\n").encode())


def vaciar_tx():
    with CANDADO:
        texto = TX.decode("ascii", "ignore")
        del TX[:]
    return texto


def contar_tramas(segundos):
    """Cuenta tramas T en una ventana. Despierta ~20/s, dormida ~2/s."""
    vaciar_tx()
    time.sleep(segundos)
    return vaciar_tx().count("T ")


# --------------------------------------------------------------------------
# Config de prueba: los tiempos del ahorro se acortan para no esperar 90 s
# --------------------------------------------------------------------------
with open(os.path.join(RUTA_VISION, "config.json"), encoding="utf-8") as f:
    cfg = json.load(f)
cfg["serial"] = {"habilitado": True, "puerto": "PUERTO_FALSO", "baudios": 115200,
                 "reintento_s": 0.5, "evitar_reset_dtr": True, "hz_envio": 20}
cfg["web"] = {"habilitado": True, "puerto": PUERTO_WEB}
cfg["telemetria"] = {"habilitado": True, "guardar_archivo": False}
cfg["ahorro"] = {"habilitado": True, "segundos_sin_web": 1.0,
                 "segundos_sin_megapi": 2.0, "hz_dormida": 2}
RUTA_CFG = os.path.join(SCRATCH, "config_prueba.json")
with open(RUTA_CFG, "w", encoding="utf-8") as f:
    json.dump(cfg, f)

import vision_server  # noqa: E402

fallos = []


def comprobar(cond, texto):
    print(("  OK   " if cond else "  FALLA") + "  " + texto)
    if not cond:
        fallos.append(texto)


def web(ruta):
    with urllib.request.urlopen("http://127.0.0.1:%d%s" % (PUERTO_WEB, ruta),
                                timeout=3) as r:
        return json.loads(r.read().decode("utf-8"))


sys.argv = ["vision_server.py", "--config", RUTA_CFG]
threading.Thread(target=vision_server.main, daemon=True).start()
time.sleep(3.0)          # arranque + que se cumplan los 2 s de silencio

print("1) Se duerme sola cuando no hay nadie")
n = contar_tramas(2.0)
comprobar(n <= 8, "ritmo de espera: %d tramas en 2 s (dormida ~4)" % n)

print("2) Contesta el boton de inicio a tiempo, estando dormida")
vaciar_tx()
t0 = time.time()
megapi_manda("P")
respuesta = ""
while time.time() - t0 < 1.0:
    respuesta += vaciar_tx()
    if "K " in respuesta:
        break
    time.sleep(0.005)
ms = (time.time() - t0) * 1000
comprobar("K " in respuesta, "responde K al ping")
comprobar(ms < 300, "responde en %.0f ms (la MegaPi espera 300)" % ms)

print("3) Ese mismo ping la despierta")
# La ventana se mide justo despues del ping: con segundos_sin_megapi=2 (solo en
# esta prueba) se volveria a dormir enseguida. En el robot son 90 s.
vaciar_tx()
time.sleep(1.0)
n = vaciar_tx().count("T ")
comprobar(n > 12, "vuelve al ritmo normal: %d tramas en 1 s (despierta ~20)" % n)

print("3b) Durante una ronda no se duerme entre comando y comando")
# Se imita la ronda: la MegaPi manda algo de vez en cuando (M PAUSA, M TORRE_REC,
# tramas de log "# ...") con huecos mas cortos que segundos_sin_megapi.
vaciar_tx()
for _ in range(4):
    megapi_manda("# avanzando")
    time.sleep(1.5)
n = vaciar_tx().count("T ")
comprobar(n > 100, "sigue a ritmo normal toda la ronda: %d tramas en 6 s" % n)

print("4) Se vuelve a dormir y la web la despierta")
time.sleep(3.5)
n = contar_tramas(1.5)
comprobar(n <= 6, "dormida otra vez: %d tramas en 1,5 s" % n)
vaciar_tx()
t0 = time.time()
estado = None
while time.time() - t0 < 1.5:      # como la pagina abierta: consulta seguido
    estado = web("/estado")
    time.sleep(0.25)
n = vaciar_tx().count("T ")
comprobar(n > 10, "con la web abierta vuelve al ritmo normal: %d tramas" % n)
comprobar(estado is not None and estado.get("camara", {}).get("estado") == "CONECTADA",
          "la web informa camara CONECTADA para poder ajustar el angulo")

print("5) El interruptor /ahorro?on=0 la deja encendida")
web("/ahorro?on=0")
time.sleep(4.0)                    # sin web ni MegaPi: antes ya estaria dormida
n = contar_tramas(2.0)
comprobar(n > 25, "sigue despierta: %d tramas en 2 s" % n)
estado = web("/estado")
comprobar(estado["ahorro"]["habilitado"] is False, "el estado refleja ahorro apagado")

print("6) Con el video abierto no se duerme, aunque la red se atasque")
# Cliente que pide /stream y deja de leer. El servidor se queda bloqueado en
# wfile.write() y no puede refrescar la marca de actividad: antes la camara se
# dormia en mitad del video y el navegador veia la imagen congelada.
web("/ahorro?on=1")
cli = socket.create_connection(("127.0.0.1", PUERTO_WEB), timeout=5)
cli.sendall(("GET /stream HTTP/1.1" + CRLF + "Host: local" + CRLF + CRLF).encode())
cli.recv(4096)                     # cabeceras y un trozo; a partir de aqui no lee
time.sleep(6.0)                    # supera sin_web (1 s) y sin_megapi (2 s)
estado = web("/estado")
comprobar(estado.get("streams", 0) >= 1,
          "el servidor cuenta el video abierto (streams=%s)" % estado.get("streams"))
cam = estado.get("camara", {}).get("estado")
comprobar(cam == "CONECTADA", "la camara sigue despierta con el video abierto (%s)" % cam)

print("7) Al cerrar el navegador se vuelve a dormir")
cli.close()
time.sleep(6.0)                    # sin consultar la web, que cuenta como actividad
estado = web("/estado")
comprobar(estado.get("streams", 0) == 0, "ya no cuenta videos abiertos")
cam = estado.get("camara", {}).get("estado")
comprobar(cam == "DORMIDA", "se duerme al cerrar el video (%s)" % cam)

print()
print("RESULTADO:", "TODO BIEN" if not fallos else "FALLAN %d: %s" % (len(fallos), fallos))
sys.exit(1 if fallos else 0)
