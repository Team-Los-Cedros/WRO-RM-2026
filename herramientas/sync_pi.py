#!/usr/bin/env python3
"""
Script de sincronizacion y control para la Raspberry Pi del Robot WRO.

Uso:
    python herramientas/sync_pi.py push       # Sube cambios de vision/ a la Pi y reinicia el servicio
    python herramientas/sync_pi.py pull       # Descarga config.json y logs de la Pi a local
    python herramientas/sync_pi.py restart    # Reinicia wro-vision.service
    python herramientas/sync_pi.py status     # Muestra estado del servicio en la Pi
    python herramientas/sync_pi.py logs       # Muestra logs recientes del servicio
    python herramientas/sync_pi.py test       # Corre test_detector.py en la Pi
"""

import argparse
import subprocess
import sys
from pathlib import Path

HOST_PI = "robot-pi"
REMOTE_REPO = "/home/pi/WRO-RM-2026"
REMOTE_VISION = f"{REMOTE_REPO}/vision"
LOCAL_ROOT = Path(__file__).resolve().parent.parent
LOCAL_VISION = LOCAL_ROOT / "vision"


def run_ssh(cmd, check=True):
    full_cmd = ["ssh", HOST_PI, cmd]
    print(f"[SSH -> {HOST_PI}] {cmd}")
    res = subprocess.run(full_cmd, check=check)
    return res.returncode == 0


def push():
    print(f"[+] Sincronizando {LOCAL_VISION} -> {HOST_PI}:{REMOTE_VISION}/ ...")
    files = [str(f) for f in LOCAL_VISION.glob("*") if f.is_file() or f.is_dir()]
    cmd = ["scp", "-r"] + files + [f"{HOST_PI}:{REMOTE_VISION}/"]
    res = subprocess.run(cmd)
    if res.returncode == 0:
        print("[OK] Archivos sincronizados con la Raspberry Pi.")
        print("[+] Reiniciando servicio wro-vision...")
        run_ssh("sudo systemctl restart wro-vision", check=False)
        run_ssh("sudo systemctl status wro-vision --no-pager", check=False)
    else:
        print(f"[!] Error al copiar archivos a {HOST_PI}")


def pull():
    print(f"[+] Descargando config.json y logs desde {HOST_PI}...")
    subprocess.run(["scp", f"{HOST_PI}:{REMOTE_VISION}/config.json", str(LOCAL_VISION / "config.json")])
    logs_remote = f"{HOST_PI}:{REMOTE_VISION}/logs"
    logs_local = LOCAL_VISION / "logs"
    logs_local.mkdir(parents=True, exist_ok=True)
    subprocess.run(["scp", "-r", f"{logs_remote}/*", str(logs_local)], check=False)
    print("[OK] Sincronizacion completada.")


def status():
    run_ssh("sudo systemctl status wro-vision --no-pager", check=False)
    run_ssh("v4l2-ctl --list-devices ; ls -la /dev/ttyUSB*", check=False)


def logs(n=40):
    run_ssh(f"journalctl -u wro-vision -n {n} --no-pager", check=False)


def restart():
    run_ssh("sudo systemctl restart wro-vision", check=False)
    run_ssh("sudo systemctl status wro-vision --no-pager", check=False)


def test():
    run_ssh(f"cd {REMOTE_VISION} && python3 test_detector.py", check=False)


def main():
    parser = argparse.ArgumentParser(description="Control y sincronizacion con Raspberry Pi WRO")
    parser.add_argument("accion", choices=["push", "pull", "restart", "status", "logs", "test"],
                        help="Accion a realizar")
    parser.add_argument("-n", "--lineas", type=int, default=40, help="Lineas de log a mostrar")
    args = parser.parse_args()

    if args.accion == "push":
        push()
    elif args.accion == "pull":
        pull()
    elif args.accion == "restart":
        restart()
    elif args.accion == "status":
        status()
    elif args.accion == "logs":
        logs(args.lineas)
    elif args.accion == "test":
        test()


if __name__ == "__main__":
    main()
