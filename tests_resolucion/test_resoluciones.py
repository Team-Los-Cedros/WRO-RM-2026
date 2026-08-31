#!/usr/bin/env python3
"""
Herramienta de prueba de resoluciones y FOV para Raspberry Pi 3B (WRO 2026).
Este archivo es independiente y NO altera ningun archivo existente del robot.

Uso en la Raspberry Pi:
    python3 test_resoluciones.py

Que hace:
1. Detecta todas las resoluciones MJPG que la camara soporta por hardware.
2. Mide FPS reales y consumo de tiempo de CPU (decodificacion + HSV + contornos) para cada una.
3. Guarda fotos comparativas para verificar el ensanchamiento del campo de vision (FOV).
"""

import os
import subprocess
import time
import cv2
import numpy as np

CARPETA_SALIDA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "capturas_fov")


def obtener_resoluciones_v4l2(dev="/dev/video0"):
    """Consulta v4l2-ctl para extraer las resoluciones soportadas en MJPG."""
    resoluciones = []
    try:
        r = subprocess.run(["v4l2-ctl", "-d", dev, "--list-formats-ext"],
                           capture_output=True, text=True, timeout=5)
        if r.returncode == 0:
            lineas = r.stdout.splitlines()
            en_mjpg = False
            for l in lineas:
                if "MJPG" in l or "Motion-JPEG" in l:
                    en_mjpg = True
                elif "Type:" in l and "MJPG" not in l:
                    en_mjpg = False
                if en_mjpg and "Size: Discrete" in l:
                    partes = l.split()
                    for p in partes:
                        if "x" in p and p.replace("x", "").isdigit():
                            w, h = map(int, p.split("x"))
                            if (w, h) not in resoluciones:
                                resoluciones.append((w, h))
    except Exception as e:
        print(f"Aviso: No se pudo ejecutar v4l2-ctl ({e}). Usando lista de resoluciones estandar.")

    if not resoluciones:
        resoluciones = [
            (320, 240),  # 4:3 actual
            (424, 240),  # 16:9 panoramica baja
            (432, 240),  # 16:9 panoramica alternativa
            (320, 180),  # 16:9 ultraligera
            (640, 360),  # 16:9 panoramica media
            (640, 480),  # 4:3 estandar
        ]
    return resoluciones


def simular_procesamiento_wro(frame):
    """Simula el procesamiento real de vision_core (ROI 70% + HSV + Morph + Contornos)."""
    alto = frame.shape[0]
    y0 = int(alto * 0.30)
    roi = frame[y0:]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    
    # Rango rojo tipico
    mask1 = cv2.inRange(hsv, np.array([0, 120, 70], dtype=np.uint8), np.array([10, 255, 255], dtype=np.uint8))
    mask2 = cv2.inRange(hsv, np.array([168, 120, 70], dtype=np.uint8), np.array([179, 255, 255], dtype=np.uint8))
    mask = mask1 | mask2

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contornos = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    return len(contornos)


def probar_resolucion(w, h, dev_idx=0, num_frames=60):
    cap = cv2.VideoCapture(dev_idx, cv2.CAP_V4L2)
    if not cap.isOpened():
        return None

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 3)

    for _ in range(5):
        cap.read()

    w_real = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h_real = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    tiempos_proc = []
    t_inicio = time.time()
    frames_ok = 0
    frame_guardar = None

    for _ in range(num_frames):
        ok, frame = cap.read()
        if not ok:
            continue
        frames_ok += 1
        
        t_proc_0 = time.time()
        simular_procesamiento_wro(frame)
        t_proc_1 = time.time()
        
        tiempos_proc.append((t_proc_1 - t_proc_0) * 1000.0)
        if frame_guardar is None:
            frame_guardar = frame.copy()

    t_total = time.time() - t_inicio
    cap.release()

    if frames_ok == 0:
        return None

    fps_real = frames_ok / max(t_total, 1e-5)
    t_proc_medio = sum(tiempos_proc) / len(tiempos_proc) if tiempos_proc else 0.0

    return {
        "pedida": f"{w}x{h}",
        "real": f"{w_real}x{h_real}",
        "fps": fps_real,
        "t_proc_ms": t_proc_medio,
        "aspecto": f"{w_real/h_real:.2f}:1",
        "frame": frame_guardar
    }


def main():
    os.makedirs(CARPETA_SALIDA, exist_ok=True)
    print("=" * 75)
    print(" BENCHMARK DE RESOLUCIONES & FOV PARA RASPBERRY PI 3B (WRO 2026)")
    print("=" * 75)
    print("Buscando resoluciones MJPG disponibles en la camara...\n")

    candidatas = obtener_resoluciones_v4l2()
    candidatas_filtradas = [r for r in candidatas if r[0] <= 800]
    
    for r in [(320, 240), (424, 240), (432, 240), (320, 180), (640, 360)]:
        if r not in candidatas_filtradas:
            candidatas_filtradas.append(r)

    candidatas_filtradas.sort(key=lambda x: x[0] * x[1])

    print(f"{'Resolucion Pedida':<18} | {'Resolucion Real':<16} | {'Relacion':<9} | {'FPS Reales':<11} | {'Proc WRO (ms)':<14}")
    print("-" * 75)

    resultados = []
    probadas_reales = set()

    for w, h in candidatas_filtradas:
        res = probar_resolucion(w, h)
        if res:
            if res["real"] not in probadas_reales:
                probadas_reales.add(res["real"])
                if res["frame"] is not None:
                    f = res["frame"].copy()
                    cv2.putText(f, f"{res['real']} ({res['aspecto']}) - {res['fps']:.1f} FPS", 
                                (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    nombre_foto = os.path.join(CARPETA_SALIDA, f"fov_{res['real']}.jpg")
                    cv2.imwrite(nombre_foto, f)

            print(f"{res['pedida']:<18} | {res['real']:<16} | {res['aspecto']:<9} | {res['fps']:<11.1f} | {res['t_proc_ms']:<14.2f}")
            resultados.append(res)
        else:
            print(f"{f'{w}x{h}':<18} | {'NO SOPORTADA':<16} | {'-':<9} | {'-':<11} | {'-':<14}")

    print("-" * 75)
    print(f"\n[OK] Fotos guardadas en: {CARPETA_SALIDA}")
    print("Descarga las fotos para comparar visualmente cual te da mayor angulo de vision horizontal.")


if __name__ == "__main__":
    main()
