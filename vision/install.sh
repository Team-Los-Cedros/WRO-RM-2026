#!/usr/bin/env bash
# Instalacion en la Raspberry Pi 3B. Idempotente: se puede volver a correr.
#   bash install.sh
set -euo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "== 1/4 Paquetes del sistema =="
# python3-opencv de los repos viene precompilado para ARMv7. NO uses
# 'pip install opencv-python' en una Pi 3B: intenta compilar y tarda horas.
sudo apt-get update
sudo apt-get install -y python3-opencv python3-serial python3-numpy v4l-utils

echo "== 2/4 Permisos del puerto serie =="
sudo usermod -aG dialout "$USER" || true
echo "   (si es la primera vez, cierra sesion y vuelve a entrar para que aplique)"

echo "== 3/4 Camaras detectadas =="
v4l2-ctl --list-devices || true

echo "== 4/4 Prueba rapida =="
python3 - <<'PY'
import cv2
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
print("camara abierta:", cap.isOpened())
if cap.isOpened():
    ok, f = cap.read()
    print("frame:", ok, None if f is None else f.shape)
cap.release()
PY

cat <<EOF

Listo.

  Calibrar artefactos: python3 $DIR/calibrar_web.py      -> http://\$(hostname -I | awk '{print \$1}'):8081
  Calibrar destinos:  python3 $DIR/calibrar_destinos.py -> http://\$(hostname -I | awk '{print \$1}'):8083
  Probar sin MegaPi:  python3 $DIR/vision_server.py --sin-serial --web
  Autoarranque:       sudo cp $DIR/wro-vision.service /etc/systemd/system/
                      sudo systemctl daemon-reload
                      sudo systemctl enable --now wro-vision
EOF
