# Reglas de Proyecto y Memoria Operativa: WRO-RM-2026

Este archivo contiene las directivas permanentes para el agente de IA al interactuar con el robot, la Raspberry Pi y las herramientas de depuración.

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
- **Servidor Web de Visión, Streaming y Telemetría:** Puerto `8080` (`http://clc-wro-rm.local:8080` o `http://192.168.0.166:8080`).
  - `/stream`: Video MJPEG con overlay OpenCV.
  - `/telemetria`: JSON con eventos cronológicos de visión y MegaPi.
  - `/estado`: Estado del servidor y serial.
- **Servidor Web de Calibración:** Puerto `8081` (`http://clc-wro-rm.local:8081`).

---

## 4. Comandos de Sincronización y Control (Herramientas PC)

Utilizar directamente las herramientas disponibles en `herramientas/`:

### Sincronizar y desplegar cambios a la Raspberry Pi:
```bash
python herramientas/sync_pi.py push
```

### Descargar configuraciones y logs de la Pi:
```bash
python herramientas/sync_pi.py pull
```

### Grabar sesiones de prueba sincronizadas (MSI StarCam + Stream Robot + Telemetría):
```bash
# Grabar corrida de duración determinada (ej. 30s)
python herramientas/grabar_sesion.py -d 30 -n prueba_slot1

# Grabar corrida completa interactiva
python herramientas/grabar_sesion.py -n corrida_completa
```

### Generar o regenerar análisis Side-by-Side de una sesión:
```bash
python herramientas/analizar_sesion.py --ultima
```

### Diagnóstico de hardware y pruebas manuales en la Pi:
```bash
# Estado del servicio y dispositivos
python herramientas/sync_pi.py status

# Logs en vivo
python herramientas/sync_pi.py logs -n 50

# Pruebas unitarias de detección
python herramientas/sync_pi.py test
```
