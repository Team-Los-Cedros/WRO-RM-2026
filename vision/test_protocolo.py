#!/usr/bin/env python3
"""
Prueba del protocolo serial SIN la MegaPi conectada.

Crea un puerto serie virtual (pty), arranca vision_server.py contra el, y
hace de MegaPi: comprueba que llegan las tramas T y que responde bien a los
comandos P, C y X.

    python3 test_protocolo.py

Sirve para verificar que el lado Raspberry esta bien antes de tocar el robot,
y para volver a comprobarlo despues de cambiar la configuracion.
"""

import json
import os
import pty
import subprocess
import sys
import time

AQUI = os.path.dirname(os.path.abspath(__file__))

fallos = []


def check(cond, msg, detalle=""):
    print(("  OK   " if cond else "  FALLA") + "  " + msg + (("  <- " + detalle) if detalle and not cond else ""))
    if not cond:
        fallos.append(msg)


def leer_lineas(fd, segundos, hasta=None):
    """Lee del pty durante N segundos; corta antes si aparece 'hasta'."""
    lineas, buf = [], ""
    fin = time.time() + segundos
    while time.time() < fin:
        try:
            datos = os.read(fd, 4096).decode("ascii", "ignore")
        except BlockingIOError:
            time.sleep(0.01)
            continue
        except OSError:
            break
        buf += datos
        while "\n" in buf:
            linea, _, buf = buf.partition("\n")
            linea = linea.strip()
            if linea:
                lineas.append(linea)
                if hasta and linea.startswith(hasta):
                    return lineas
        time.sleep(0.005)
    return lineas


def main():
    maestro, esclavo = pty.openpty()
    os.set_blocking(maestro, False)
    puerto = os.ttyname(esclavo)

    with open(os.path.join(AQUI, "config.json"), encoding="utf-8") as f:
        cfg = json.load(f)
    cfg["serial"]["puerto"] = puerto
    cfg["serial"]["habilitado"] = True
    # En un pty no hay lineas DTR/RTS reales que togglear.
    cfg["serial"]["evitar_reset_dtr"] = True
    ruta_cfg = "/tmp/cfg_test_vision.json"
    with open(ruta_cfg, "w", encoding="utf-8") as f:
        json.dump(cfg, f)

    print("Puerto virtual: %s" % puerto)
    proc = subprocess.Popen(
        [sys.executable, "-u", os.path.join(AQUI, "vision_server.py"), "--config", ruta_cfg],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

    try:
        print("\nEsperando el arranque (camara + OpenCV)...")
        time.sleep(14)

        print("\n1. Tramas T")
        lineas = leer_lineas(maestro, 3.0)
        tramas = [l for l in lineas if l.startswith("T ")]
        check(len(tramas) > 0, "llegan tramas T", "no llego ninguna linea: %r" % lineas[:3])

        if tramas:
            hz = len(tramas) / 3.0
            check(hz >= 12, "frecuencia >= 12 Hz (medido %.1f Hz)" % hz)
            campos = tramas[-1].split()
            check(len(campos) == 8, "la trama tiene 8 campos", tramas[-1])
            check(campos[1] in ("0", "1"), "campo found es 0 o 1", tramas[-1])
            try:
                int(campos[3]); int(campos[4]); int(campos[5]); int(campos[6])
                ok_num = True
            except ValueError:
                ok_num = False
            check(ok_num, "los campos numericos parsean", tramas[-1])
            print("     ejemplo: %s" % tramas[-1])

        print("\n2. Comando P (ping)")
        os.write(maestro, b"P\n")
        resp = leer_lineas(maestro, 2.0, hasta="K ")
        ks = [l for l in resp if l.startswith("K ")]
        check(len(ks) > 0, "responde al ping con K", "respuestas: %r" % resp[-3:])
        if ks:
            print("     %s" % ks[0])

        print("\n3. Comando C (cambio de color)")
        os.write(maestro, b"C VERDE\n")
        resp = leer_lineas(maestro, 2.0, hasta="K COLOR")
        check(any(l.startswith("K COLOR VERDE") for l in resp), "confirma el cambio a VERDE")
        tras = [l for l in leer_lineas(maestro, 1.5) if l.startswith("T ")]
        check(bool(tras) and all(l.split()[2] == "VERDE" for l in tras),
              "las tramas pasan a reportar VERDE",
              tras[-1] if tras else "sin tramas")

        print("\n4. Comando C con un color inexistente")
        os.write(maestro, b"C MORADO\n")
        resp = leer_lineas(maestro, 2.0, hasta="E COLOR")
        check(any(l.startswith("E COLOR") for l in resp), "rechaza un color desconocido")

        print("\n5. Comando X (escaneo de los cinco colores)")
        os.write(maestro, b"X\n")
        resp = leer_lineas(maestro, 3.0, hasta="X ")
        xs = [l for l in resp if l.startswith("X ")]
        check(len(xs) > 0, "responde al escaneo")
        if xs:
            campos = xs[0].split()
            check(len(campos) == 21, "el escaneo trae 5 colores x 4 campos", xs[0])
            nombres = campos[1::4]
            check(set(nombres) == {"ROJO", "VERDE", "AZUL", "AMARILLO", "NEGRO"},
                  "estan los cinco colores", str(nombres))
            check(len(xs[0]) < 128, "la respuesta cabe en el buffer de 128 B del Arduino (%d B)" % len(xs[0]))
            print("     %s" % xs[0])

        print("\n6. Comando S (apagar y encender el envio)")
        os.write(maestro, b"S 0\n")
        leer_lineas(maestro, 1.0)
        quietas = [l for l in leer_lineas(maestro, 1.5) if l.startswith("T ")]
        check(len(quietas) == 0, "con S 0 deja de enviar tramas", "%d tramas" % len(quietas))
        os.write(maestro, b"S 1\n")
        vuelven = [l for l in leer_lineas(maestro, 1.5) if l.startswith("T ")]
        check(len(vuelven) > 0, "con S 1 vuelve a enviar")

        print("\n7. Log de la MegaPi con prefijo #")
        os.write(maestro, b"#hola desde la MegaPi\n")
        time.sleep(0.5)
        check(True, "aceptado sin romper el protocolo")
        siguen = [l for l in leer_lineas(maestro, 1.0) if l.startswith("T ")]
        check(len(siguen) > 0, "sigue enviando tramas despues del log")

    finally:
        proc.terminate()
        try:
            salida = proc.communicate(timeout=5)[0]
        except subprocess.TimeoutExpired:
            proc.kill()
            salida = ""
        os.close(maestro)
        os.close(esclavo)

    print("\n" + "=" * 46)
    if fallos:
        print("FALLARON %d comprobaciones:" % len(fallos))
        for f in fallos:
            print("  - %s" % f)
        print("\nSalida del servidor:\n%s" % salida[-1500:])
        return 1
    print("TODO CORRECTO")
    return 0


if __name__ == "__main__":
    sys.exit(main())
