# Reglas de Proyecto y Memoria Operativa: WRO-RM-2026

Este archivo contiene las directivas permanentes para el agente de IA al interactuar con el robot y la Raspberry Pi.

---

## 1. Conexión SSH a la Raspberry Pi del Robot

- **Alias SSH configurado:** `robot-pi` (también responde por `clc-wro-rm.local` y `192.168.0.166`).
- **Usuario:** `pi`
- **Autenticación:** Clave SSH dedicada `~/.ssh/id_robot_ed25519` instalada y configurada en `~/.ssh/config`. **No requiere contraseña.**
- **Permisos de root:** `sudo` sin contraseña (`NOPASSWD`) habilitado en `/etc/sudoers.d/010_pi-nopasswd`.

---

## 2. Rutas y Estructura en la Raspberry Pi

- **Directorio raíz del repo:** `/home/pi/WRO-RM-2026`
- **Módulo de visión:** `/home/pi/WRO-RM-2026/vision`
- **Servicio Systemd:** `wro-vision.service` (`/etc/systemd/system/wro-vision.service`)

---

## 3. Dispositivos y Puertos de Hardware

- **Cámara USB:** Enumerada en `/dev/video0` (USB Camera).
- **Controlador MegaPi:** Conectado por USB serial en `/dev/ttyUSB0` (115200 baud).
- **Servidor Web de Visión / Streaming:** Puerto `8080` (`http://clc-wro-rm.local:8080`).
- **Servidor Web de Calibración:** Puerto `8081` (`http://clc-wro-rm.local:8081`).

---

## 4. Comandos Rápidos de Despliegue y Control

El agente debe utilizar directamente estos comandos sin solicitar credenciales:

### Desplegar / Sincronizar cambios:
```bash
scp -r vision/* robot-pi:/home/pi/WRO-RM-2026/vision/
```

### Control del servicio de visión:
```bash
# Reiniciar servicio
ssh robot-pi "sudo systemctl restart wro-vision"

# Ver estado
ssh robot-pi "sudo systemctl status wro-vision --no-pager"

# Ver logs en vivo / recientes
ssh robot-pi "journalctl -u wro-vision -n 50 --no-pager"
```

### Diagnóstico de hardware y pruebas:
```bash
# Verificar dispositivos
ssh robot-pi "v4l2-ctl --list-devices && ls -la /dev/ttyUSB*"

# Ejecutar pruebas unitarias de visión
ssh robot-pi "cd /home/pi/WRO-RM-2026/vision && python3 test_detector.py"
```
