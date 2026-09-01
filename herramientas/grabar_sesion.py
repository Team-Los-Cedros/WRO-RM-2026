#!/usr/bin/env python3
"""
Orquestador de Grabacion de Sesion Sincronizada (WRO 2026).

Graba simultaneamente:
  1. Camara Externa MSI StarCam 370i (DirectShow / FFmpeg)
  2. Transmision en Vivo Onboard del Robot (Raspberry Pi 3B / MJPEG Stream)
  3. Registro de Telemetria (Vision OpenCV + Eventos MegaPi)

No requiere dependencias externas; utiliza FFmpeg y la libreria estandar de Python.

Uso:
  python herramientas/grabar_sesion.py -d 30                  # Graba 30 segundos
  python herramientas/grabar_sesion.py -n prueba_slot1        # Graba hasta presionar Enter
  python herramientas/grabar_sesion.py --solo-robot -d 15     # Graba solo la camara del robot
  python herramientas/grabar_sesion.py --solo-externa -d 15   # Graba solo la camara MSI
"""

import argparse
import datetime
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
import urllib.request

ROOT_DIR = Path(__file__).resolve().parent.parent
SESIONES_DIR = ROOT_DIR / "sesiones"
DEFAULT_PI_URL = "http://192.168.0.166:8080"
MSI_DEVICE_NAME = "MSI Star Cam 370i"


def detener_proceso_ffmpeg(proc, nombre="FFmpeg"):
    if proc is None:
        return
    try:
        if proc.poll() is None:
            proc.stdin.write(b"q\n")
            proc.stdin.flush()
            proc.wait(timeout=4)
    except Exception:
        try:
            proc.terminate()
            proc.wait(timeout=2)
        except Exception:
            proc.kill()
    print(f"[{nombre}] Grabacion finalizada.")


def grabar_sesion(duracion=None, nombre=None, pi_url=DEFAULT_PI_URL,
                  grabar_externa=True, grabar_robot=True, auto_analizar=True):
    SESIONES_DIR.mkdir(parents=True, exist_ok=True)
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    tag = f"_{nombre}" if nombre else ""
    sesion_path = SESIONES_DIR / f"sesion_{ts}{tag}"
    sesion_path.mkdir(parents=True, exist_ok=True)

    print("=" * 65)
    print(f"  INICIANDO SESION DE PRUEBA WRO: {sesion_path.name}")
    print("=" * 65)

    # 1. Resetear telemetria en la Raspberry Pi si esta activa
    try:
        req = urllib.request.Request(f"{pi_url}/telemetria/reset", method="GET")
        with urllib.request.urlopen(req, timeout=2.0) as resp:
            pass
        print(f"[OK] Telemetria de la Raspberry Pi reseteada para nueva sesion.")
    except Exception:
        print(f"[AVISO] No se pudo contactar {pi_url}/telemetria/reset. Continuando...")

    proc_externa = None
    proc_robot = None
    archivo_robot = sesion_path / "camara_robot.mp4"
    archivo_externa = sesion_path / "camara_externa.mp4"

    t_inicio_epoch = time.time()
    t_inicio_iso = datetime.datetime.now(datetime.timezone.utc).isoformat()

    # 2. Iniciar grabador Robot Stream con FFmpeg
    if grabar_robot:
        cmd_robot = [
            "ffmpeg", "-y",
            "-re",
            "-i", f"{pi_url}/stream",
            "-c:v", "libx264",
            "-preset", "veryfast",
            "-pix_fmt", "yuv420p",
            str(archivo_robot)
        ]
        try:
            print(f"[Robot Stream] Grabando {pi_url}/stream -> {archivo_robot.name}")
            proc_robot = subprocess.Popen(cmd_robot, stdin=subprocess.PIPE,
                                          stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception as e:
            print(f"[Robot Stream] ERROR al arrancar FFmpeg para robot: {e}")
            proc_robot = None

    # 3. Iniciar grabador Camara Externa MSI StarCam con FFmpeg
    if grabar_externa:
        cmd_externa = [
            "ffmpeg", "-y",
            "-f", "dshow",
            "-video_size", "640x480",
            "-framerate", "30",
            "-i", f"video={MSI_DEVICE_NAME}",
            "-c:v", "libx264",
            "-preset", "veryfast",
            "-pix_fmt", "yuv420p",
            str(archivo_externa)
        ]
        try:
            print(f"[MSI StarCam] Grabando 640x480 @ 30fps -> {archivo_externa.name}")
            proc_externa = subprocess.Popen(cmd_externa, stdin=subprocess.PIPE,
                                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception as e:
            print(f"[MSI StarCam] ERROR al arrancar FFmpeg para StarCam: {e}")
            proc_externa = None

    print("-" * 65)
    if duracion:
        print(f"[+] Grabando durante {duracion} segundos...")
        try:
            time.sleep(duracion)
        except KeyboardInterrupt:
            print("\n[!] Interrumpido manualmente.")
    else:
        print("[+] Grabando en vivo... Presiona [ENTER] o Ctrl+C para detener la prueba:")
        try:
            input()
        except KeyboardInterrupt:
            print("\n[!] Interrumpido manualmente.")

    print("[*] Deteniendo grabaciones...")
    detener_proceso_ffmpeg(proc_externa, "MSI StarCam")
    detener_proceso_ffmpeg(proc_robot, "Robot Stream")

    t_fin_epoch = time.time()
    t_fin_iso = datetime.datetime.now(datetime.timezone.utc).isoformat()
    duracion_real = round(t_fin_epoch - t_inicio_epoch, 2)

    # 4. Descargar telemetria completa de la Raspberry Pi
    telemetria_items = []
    try:
        print(f"[+] Descargando telemetria desde {pi_url}/telemetria ...")
        with urllib.request.urlopen(f"{pi_url}/telemetria", timeout=3.0) as resp:
            data = json.loads(resp.read().decode("utf-8"))
            telemetria_items = data.get("items", [])
    except Exception as e:
        print(f"[AVISO] No se pudo obtener telemetria por HTTP: {e}")

    # Guardar telemetria en JSONL
    archivo_telemetria = sesion_path / "telemetria.jsonl"
    with open(archivo_telemetria, "w", encoding="utf-8") as f:
        for item in telemetria_items:
            f.write(json.dumps(item, ensure_ascii=False) + "\n")
    print(f"[OK] Telemetria guardada: {len(telemetria_items)} eventos -> {archivo_telemetria.name}")

    # 5. Guardar metadatos de sesion
    metadatos = {
        "sesion_id": sesion_path.name,
        "nombre": nombre or "sin_nombre",
        "t_inicio_epoch": t_inicio_epoch,
        "t_inicio_iso": t_inicio_iso,
        "t_fin_epoch": t_fin_epoch,
        "t_fin_iso": t_fin_iso,
        "duracion_segundos": duracion_real,
        "archivo_externa": archivo_externa.name if (grabar_externa and archivo_externa.exists()) else None,
        "archivo_robot": archivo_robot.name if (grabar_robot and archivo_robot.exists()) else None,
        "archivo_telemetria": archivo_telemetria.name,
        "total_eventos_telemetria": len(telemetria_items)
    }

    with open(sesion_path / "metadatos.json", "w", encoding="utf-8") as f:
        json.dump(metadatos, f, indent=2, ensure_ascii=False)

    print("=" * 65)
    print(f"  SESION COMPLETADA EXITOSAMENTE: {duracion_real}s")
    print(f"  Carpeta: {sesion_path.resolve()}")
    print("=" * 65)

    # 6. Lanzar analisis automatico si fue solicitado
    if auto_analizar:
        script_analizar = ROOT_DIR / "herramientas" / "analizar_sesion.py"
        if script_analizar.exists():
            print(f"[+] Generando video compuesto y reporte de diagnostico...")
            subprocess.run([sys.executable, str(script_analizar), "--sesion", str(sesion_path)])

    return str(sesion_path)


def main():
    parser = argparse.ArgumentParser(description="Orquestador de Grabacion Multi-Camara y Telemetria WRO")
    parser.add_argument("-d", "--duracion", type=int, default=None, help="Duracion en segundos (opcional)")
    parser.add_argument("-n", "--nombre", type=str, default=None, help="Etiqueta descriptiva para la sesion")
    parser.add_argument("--ip-pi", type=str, default="192.168.0.166", help="IP o host de la Raspberry Pi")
    parser.add_argument("--puerto", type=int, default=8080, help="Puerto del servidor de vision de la Pi")
    parser.add_argument("--solo-robot", action="store_true", help="Grabar solo la camara del robot")
    parser.add_argument("--solo-externa", action="store_true", help="Grabar solo la camara externa MSI")
    parser.add_argument("--no-analisis", action="store_true", help="No generar video side-by-side automaticamente")
    args = parser.parse_args()

    grabar_externa = not args.solo_robot
    grabar_robot = not args.solo_externa
    pi_url = f"http://{args.ip_pi}:{args.puerto}"

    grabar_sesion(
        duracion=args.duracion,
        nombre=args.nombre,
        pi_url=pi_url,
        grabar_externa=grabar_externa,
        grabar_robot=grabar_robot,
        auto_analizar=not args.no_analisis
    )


if __name__ == "__main__":
    main()
