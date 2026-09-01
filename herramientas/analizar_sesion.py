#!/usr/bin/env python3
"""
Herramienta de Analisis, Diagnostico y Generacion de Video Side-by-Side (WRO 2026).

Combina la grabacion de la camara externa MSI StarCam con la grabacion del stream
del robot y los logs de telemetria en un video comparativo sincronizado y genera
un reporte diagnostico con capturas de momentos clave.

No requiere dependencias externas; utiliza FFmpeg y la libreria estandar de Python.

Uso:
  python herramientas/analizar_sesion.py --sesion sesiones/sesion_20260901_014500
  python herramientas/analizar_sesion.py --ultima        # Analiza la sesion mas reciente
"""

import argparse
import datetime
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT_DIR = Path(__file__).resolve().parent.parent
SESIONES_DIR = ROOT_DIR / "sesiones"


def cargar_telemetria(archivo_jsonl):
    eventos = []
    if not archivo_jsonl.exists():
        return eventos
    with open(archivo_jsonl, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                try:
                    eventos.append(json.loads(line))
                except Exception:
                    pass
    return eventos


def generar_side_by_side(sesion_dir):
    sesion_dir = Path(sesion_dir)
    print(f"[Analisis] Procesando sesion: {sesion_dir.name}")

    meta_file = sesion_dir / "metadatos.json"
    telemetria_file = sesion_dir / "telemetria.jsonl"
    vid_ext_path = sesion_dir / "camara_externa.mp4"
    vid_rob_path = sesion_dir / "camara_robot.mp4"
    out_video_path = sesion_dir / "comparativa_side_by_side.mp4"
    snap_dir = sesion_dir / "snapshots"
    snap_dir.mkdir(parents=True, exist_ok=True)

    eventos = cargar_telemetria(telemetria_file)
    tiene_ext = vid_ext_path.exists() and vid_ext_path.stat().st_size > 1000
    tiene_rob = vid_rob_path.exists() and vid_rob_path.stat().st_size > 1000

    # 1. Generar video comparativo con FFmpeg
    if tiene_ext and tiene_rob:
        print("[+] Creando video Side-by-Side (Externa + Onboard)...")
        cmd_side = [
            "ffmpeg", "-y",
            "-i", str(vid_ext_path),
            "-i", str(vid_rob_path),
            "-filter_complex",
            "[0:v]scale=480:360,drawtext=text='CAMARA EXTERNA (MSI StarCam)':x=15:y=15:fontsize=16:fontcolor=yellow:box=1:boxcolor=black@0.6[left];"
            "[1:v]scale=480:360,drawtext=text='ROBOT ONBOARD (Vision)':x=15:y=15:fontsize=16:fontcolor=lightgreen:box=1:boxcolor=black@0.6[right];"
            "[left][right]hstack=inputs=2[v]",
            "-map", "[v]",
            "-c:v", "libx264",
            "-preset", "veryfast",
            "-pix_fmt", "yuv420p",
            str(out_video_path)
        ]
        res = subprocess.run(cmd_side, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if res.returncode == 0:
            print(f"[OK] Video Side-by-Side generado: {out_video_path.name}")
        else:
            # Fallback simple sin drawtext si faltan fuentes
            cmd_side_simple = [
                "ffmpeg", "-y",
                "-i", str(vid_ext_path),
                "-i", str(vid_rob_path),
                "-filter_complex", "[0:v]scale=480:360[left];[1:v]scale=480:360[right];[left][right]hstack=inputs=2[v]",
                "-map", "[v]",
                "-c:v", "libx264",
                "-preset", "veryfast",
                "-pix_fmt", "yuv420p",
                str(out_video_path)
            ]
            subprocess.run(cmd_side_simple, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            print(f"[OK] Video Side-by-Side generado (modo directo): {out_video_path.name}")

    # 2. Extraer capturas clave a intervalos
    vid_principal = vid_rob_path if tiene_rob else (vid_ext_path if tiene_ext else None)
    if vid_principal:
        print("[+] Extrayendo snapshots clave de la sesion...")
        cmd_snaps = [
            "ffmpeg", "-y",
            "-i", str(vid_principal),
            "-vf", "fps=0.5",
            "-vframes", "8",
            str(snap_dir / "snap_%03d.jpg")
        ]
        subprocess.run(cmd_snaps, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    # 3. Cargar metadatos
    metadatos = {}
    if meta_file.exists():
        with open(meta_file, "r", encoding="utf-8") as f:
            metadatos = json.load(f)

    # 4. Generar reporte Markdown
    generar_reporte_md(sesion_dir, metadatos, eventos, snap_dir, tiene_ext, tiene_rob, out_video_path.exists())


def generar_reporte_md(sesion_dir, metadatos, eventos, snap_dir, tiene_ext, tiene_rob, tiene_side):
    reporte_path = sesion_dir / "reporte_diagnostico.md"
    ev_megapi = [e for e in eventos if e.get("tipo") == "megapi"]
    ev_vision = [e for e in eventos if e.get("tipo") == "vision"]

    snaps = sorted(list(snap_dir.glob("*.jpg")))

    with open(reporte_path, "w", encoding="utf-8") as f:
        f.write(f"# Reporte de Sesion y Diagnostico: {sesion_dir.name}\n\n")
        f.write(f"- **Fecha:** `{metadatos.get('t_inicio_iso', 'N/A')}`\n")
        f.write(f"- **Duracion:** `{metadatos.get('duracion_segundos', 0)} s`\n")
        f.write(f"- **Total eventos telemetria:** `{len(eventos)}` ({len(ev_vision)} vision, {len(ev_megapi)} MegaPi)\n\n")

        f.write("## Archivos de la Sesion\n\n")
        if tiene_side:
            f.write(f"- **Video Side-by-Side:** [`comparativa_side_by_side.mp4`](file:///{sesion_dir.resolve().as_posix()}/comparativa_side_by_side.mp4)\n")
        if tiene_ext:
            f.write(f"- **Camara Externa (MSI):** [`camara_externa.mp4`](file:///{sesion_dir.resolve().as_posix()}/camara_externa.mp4)\n")
        if tiene_rob:
            f.write(f"- **Camara Robot (Stream):** [`camara_robot.mp4`](file:///{sesion_dir.resolve().as_posix()}/camara_robot.mp4)\n")
        f.write(f"- **Telemetria:** [`telemetria.jsonl`](file:///{sesion_dir.resolve().as_posix()}/telemetria.jsonl)\n\n")

        f.write("## Eventos Registrados de la MegaPi\n\n")
        if ev_megapi:
            f.write("| Timestamp | Mensaje MegaPi |\n|---|---|\n")
            for em in ev_megapi:
                t_str = em.get("t_iso", "").split("T")[-1][:8] if "T" in em.get("t_iso", "") else em.get("t_iso", "")
                f.write(f"| `{t_str}` | **{em.get('mensaje', '')}** |\n")
        else:
            f.write("No se registraron mensajes seriales de la MegaPi durante esta corrida.\n")

        if snaps:
            f.write("\n## Capturas Clave de la Sesion\n\n")
            for sn in snaps:
                f.write(f"### {sn.stem}\n")
                f.write(f"![{sn.stem}](file:///{sn.resolve().as_posix()})\n\n")

    print(f"[OK] Reporte Markdown generado: {reporte_path.name}")


def main():
    parser = argparse.ArgumentParser(description="Generador Side-by-Side y Reporte Diagnostico")
    parser.add_argument("-s", "--sesion", type=str, default=None, help="Ruta de la sesion a analizar")
    parser.add_argument("--ultima", action="store_true", help="Analizar automaticamente la sesion mas reciente")
    args = parser.parse_args()

    if args.ultima or not args.sesion:
        sesiones = sorted([d for d in SESIONES_DIR.iterdir() if d.is_dir() and d.name.startswith("sesion_")])
        if not sesiones:
            print("[!] No se encontraron sesiones en la carpeta sesiones/")
            return
        sesion_dir = sesiones[-1]
    else:
        sesion_dir = Path(args.sesion)

    generar_side_by_side(sesion_dir)


if __name__ == "__main__":
    main()
