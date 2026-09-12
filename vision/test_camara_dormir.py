"""Prueba del ahorro de camara sin Raspberry, sin OpenCV y sin camara.

Simula cv2 para comprobar las tres cosas que no pueden fallar en competencia:
  1. dormir() suelta de verdad la camara,
  2. despertar() la vuelve a abrir sola,
  3. si la camara no esta al arrancar, abrir() NO tumba el proceso.
"""
import os
import sys
import time
import types

RUTA = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, RUTA)

# --------------------------------------------------------------------------
# cv2 simulado: solo lo que usa Camara
# --------------------------------------------------------------------------
cv2 = types.ModuleType("cv2")
for nombre, valor in (("CAP_V4L2", 200), ("CAP_PROP_FOURCC", 6),
                      ("CAP_PROP_FRAME_WIDTH", 3), ("CAP_PROP_FRAME_HEIGHT", 4),
                      ("CAP_PROP_FPS", 5), ("CAP_PROP_BUFFERSIZE", 38),
                      ("ROTATE_180", 1), ("IMWRITE_JPEG_QUALITY", 1)):
    setattr(cv2, nombre, valor)
cv2.setNumThreads = lambda n: None
cv2.VideoWriter_fourcc = lambda *a: 0
cv2.rotate = lambda f, c: f

CUENTA = {"abiertas": 0, "liberadas": 0, "falla": False}


class CapturaFalsa:
    def __init__(self, abierta=True):
        self._abierta = abierta
        if abierta:
            CUENTA["abiertas"] += 1

    def isOpened(self):
        return self._abierta

    def set(self, *a):
        return True

    def read(self):
        time.sleep(0.005)
        return (True, "FRAME") if self._abierta else (False, None)

    def release(self):
        if self._abierta:
            CUENTA["liberadas"] += 1
        self._abierta = False


cv2.VideoCapture = lambda dev, api=None: CapturaFalsa(not CUENTA["falla"])
sys.modules["cv2"] = cv2
try:
    import numpy  # noqa: F401
except ImportError:
    sys.modules["numpy"] = types.ModuleType("numpy")

import vision_core  # noqa: E402

CFG = {"indice": 0, "dispositivo": "/dev/video0", "ancho": 640, "alto": 360,
       "fps": 30, "fourcc": "MJPG", "buffers": 3, "reintento_s": 0.2,
       "espera_inicial_s": 1.0}

fallos = []


def comprobar(condicion, texto):
    print(("  OK   " if condicion else "  FALLA") + "  " + texto)
    if not condicion:
        fallos.append(texto)


def esperar(cond, limite=4.0):
    t0 = time.time()
    while time.time() - t0 < limite:
        if cond():
            return True
        time.sleep(0.05)
    return False


print("1) Arranque normal")
cam = vision_core.Camara(CFG)
cam._aplicar_controles_v4l2 = lambda dev=None: None
cam.abrir()
comprobar(cam.estado()["estado"] == "CONECTADA", "queda CONECTADA")
comprobar(cam.leer(timeout=0.5) is not None, "entrega frames")
comprobar(not cam.dormida, "arranca despierta")

print("2) Dormir")
liberadas_antes = CUENTA["liberadas"]
cam.dormir()
comprobar(esperar(lambda: cam.estado()["estado"] == "DORMIDA"), "pasa a DORMIDA")
comprobar(CUENTA["liberadas"] == liberadas_antes + 1, "suelta el dispositivo")
comprobar(cam.leer(solo_nuevos=True, timeout=0.05) is None, "no entrega frames")
recon_antes = cam.estado()["reconexiones"]

print("3) Despertar")
t0 = time.time()
cam.despertar()
ok = esperar(lambda: cam.leer(solo_nuevos=True, timeout=0.05) is not None)
comprobar(ok, "vuelve a entregar frames en %.2f s" % (time.time() - t0))
comprobar(cam.estado()["estado"] == "CONECTADA", "queda CONECTADA otra vez")
comprobar(cam.estado()["reconexiones"] == recon_antes,
          "despertar no cuenta como reconexion/averia")
cam.cerrar()

print("4) Arranque con la camara todavia ausente (el caso del arranque rapido)")
CUENTA["falla"] = True
cam2 = vision_core.Camara(CFG)
cam2._aplicar_controles_v4l2 = lambda dev=None: None
t0 = time.time()
try:
    cam2.abrir()
    comprobar(True, "abrir() no lanza excepcion (el servicio no se reinicia)")
except Exception as e:
    comprobar(False, "abrir() lanzo %r" % e)
comprobar(time.time() - t0 < 8.0, "abrir() vuelve sin colgarse (%.1f s)" % (time.time() - t0))
comprobar(cam2.estado()["estado"] == "RECONECTANDO", "queda RECONECTANDO")

print("5) La camara aparece despues: se engancha sola")
CUENTA["falla"] = False
comprobar(esperar(lambda: cam2.leer(solo_nuevos=True, timeout=0.05) is not None, 6.0),
          "engancha la camara sin reiniciar el servicio")
cam2.cerrar()

print()
print("RESULTADO:", "TODO BIEN" if not fallos else "FALLAN %d: %s" % (len(fallos), fallos))
sys.exit(1 if fallos else 0)
