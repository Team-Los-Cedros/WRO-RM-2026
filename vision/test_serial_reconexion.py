"""Comprueba el arreglo del serial con la MegaPi apagada.

Antes: si el puerto no se podia abrir quedaba un objeto Serial cerrado, el
siguiente enviar() lanzaba "port is not open" y se registraba una desconexion.
Resultado: una linea de log por segundo, para siempre.
"""
import os
import sys
import time
import types

RUTA_VISION = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, RUTA_VISION)

cv2 = types.ModuleType("cv2")
cv2.setNumThreads = lambda n: None
cv2.__getattr__ = lambda n: (lambda *a, **k: None)
sys.modules["cv2"] = cv2
try:
    import numpy  # noqa: F401
except ImportError:
    np = types.ModuleType("numpy")
    np.array = lambda x, dtype=None: list(x)
    np.__getattr__ = lambda n: (lambda *a, **k: None)
    sys.modules["numpy"] = np

ESTADO = {"hay_megapi": False}


class SerialFalso:
    def __init__(self, port=None, baudrate=None, timeout=None):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.dtr = False
        self.rts = False
        self.is_open = False

    def open(self):
        if not ESTADO["hay_megapi"]:
            raise OSError("[Errno 2] could not open port: no such file or directory")
        self.is_open = True

    @property
    def in_waiting(self):
        if not self.is_open:
            raise Exception("Attempting to use a port that is not open")
        return 0

    def write(self, datos):
        if not self.is_open:
            raise Exception("Attempting to use a port that is not open")
        return len(datos)

    def read(self, n):
        return b""

    def reset_input_buffer(self):
        pass

    def close(self):
        self.is_open = False


serial_mod = types.ModuleType("serial")
serial_mod.Serial = SerialFalso
sys.modules["serial"] = serial_mod

import vision_server  # noqa: E402

LINEAS = []
fallos = []


def comprobar(cond, texto):
    print(("  OK   " if cond else "  FALLA") + "  " + texto)
    if not cond:
        fallos.append(texto)


cfg = {"habilitado": True, "puerto": "PUERTO_FALSO", "baudios": 115200,
       "reintento_s": 0.05, "evitar_reset_dtr": True, "hz_envio": 20}
enlace = vision_server.EnlaceSerial(cfg, lambda m: LINEAS.append(m))

print("1) MegaPi apagada: 30 vueltas del bucle como las que da el servidor")
for _ in range(30):
    enlace.asegurar_conectado()
    enlace.enviar("T 0 NINGUNO 0 0 0 -1 20 0")
    enlace.leer_lineas()
    time.sleep(0.02)

desconexiones = [l for l in LINEAS if "desconectado" in l]
esperas = [l for l in LINEAS if "esperando a la MegaPi" in l]
comprobar(enlace.ser is None, "no queda un puerto cerrado colgando")
comprobar(not desconexiones, "no marca desconexiones falsas (antes: 1 por segundo)")
comprobar(len(esperas) == 1, "avisa una sola vez (%d avisos)" % len(esperas))
comprobar(not enlace.conectado, "sigue sabiendo que no esta conectado")

print("2) Se enciende la MegaPi")
ESTADO["hay_megapi"] = True
enlace._proximo_intento = 0.0
comprobar(enlace.asegurar_conectado(), "conecta en cuanto aparece el puerto")
comprobar(any("serial abierto" in l for l in LINEAS), "lo dice en el log")

print("3) Se vuelve a apagar en marcha")
LINEAS.clear()
enlace.ser.is_open = False          # como cuando desaparece el dispositivo
enlace.enviar("T 0 NINGUNO 0 0 0 -1 20 0")
comprobar(len([l for l in LINEAS if "desconectado" in l]) == 1,
          "marca la desconexion real una vez")
LINEAS.clear()
for _ in range(20):
    enlace.asegurar_conectado()
    enlace.enviar("T 0 NINGUNO 0 0 0 -1 20 0")
    time.sleep(0.02)
comprobar(not [l for l in LINEAS if "desconectado" in l],
          "y luego reintenta en silencio")

print()
print("RESULTADO:", "TODO BIEN" if not fallos else "FALLAN %d: %s" % (len(fallos), fallos))
sys.exit(1 if fallos else 0)
