#!/usr/bin/env python3
"""Evalua detectores sobre un video grabado sin mover el robot."""

import argparse
from pathlib import Path
import sys

import cv2

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "vision"))

from vision_core import Detector, cargar_config  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("video", type=Path)
    ap.add_argument("--config", type=Path, default=ROOT / "vision" / "config.json")
    ap.add_argument("--modo", choices=("FILA", "DESTINO", "ARTEFACTO"), required=True)
    ap.add_argument("--color", default="ROJO")
    ap.add_argument("--desde", type=float, default=0.0)
    ap.add_argument("--hasta", type=float, default=30.0)
    ap.add_argument("--cada", type=float, default=1.0)
    args = ap.parse_args()

    detector = Detector(cargar_config(str(args.config)))
    cap = cv2.VideoCapture(str(args.video))
    if not cap.isOpened():
        raise SystemExit("No se pudo abrir %s" % args.video)

    t = args.desde
    while t <= args.hasta + 1e-6:
        cap.set(cv2.CAP_PROP_POS_MSEC, t * 1000.0)
        ok, frame = cap.read()
        if not ok:
            print("%6.1fs SIN_FRAME" % t)
            t += args.cada
            continue

        if args.modo == "FILA":
            fila = detector.detectar_fila_artefactos(frame)
            resumen = " ".join("%s@%+d/q%d" % (d.color, d.ex, d.confianza)
                               for d in fila)
            print("%6.1fs F %d %s" % (t, len(fila), resumen))
        elif args.modo == "DESTINO":
            d, _, _ = detector.detectar_destino(frame, args.color.upper())
            print("%6.1fs D %d %s ex=%+d ey=%d area=%d q=%d" %
                  (t, int(d.encontrado), args.color.upper(), d.ex, d.ey,
                   d.area, d.confianza))
        else:
            d, _, _ = detector.detectar(frame, args.color.upper())
            print("%6.1fs A %d %s ex=%+d ey=%d area=%d q=%d" %
                  (t, int(d.encontrado), args.color.upper(), d.ex, d.ey,
                   d.area, d.confianza))
        t += args.cada

    cap.release()


if __name__ == "__main__":
    main()
