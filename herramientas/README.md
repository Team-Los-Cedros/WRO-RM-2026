# Herramientas de Depuración, Grabación y Telemetría WRO 2026

Este módulo contiene las herramientas para la captura coordinada de pruebas, sincronización con la Raspberry Pi y generación de diagnósticos visuales y cuantitativos del robot.

---

## 📋 Resumen de Herramientas

| Script | Descripción |
|---|---|
| [`grabar_sesion.py`](grabar_sesion.py) | Orquestador que graba en paralelo la cámara externa MSI, el stream del robot y descarga la telemetría en JSONL. |
| [`analizar_sesion.py`](analizar_sesion.py) | Genera el video compuesto **Side-by-Side** (Externa + Onboard), extrae capturas de eventos y compila `reporte_diagnostico.md`. |
| [`sync_pi.py`](sync_pi.py) | Sincronizador de código bidireccional y control del servicio de visión en la Raspberry Pi. |

---

## 🎥 1. Grabación de Sesiones de Prueba (`grabar_sesion.py`)

Graba de forma simultánea:
1. **Cámara Externa MSI StarCam 370i:** Vista global/tercera persona de la mesa en 640x480 @ 30fps.
2. **Cámara Onboard del Robot (Raspberry Pi 3B):** Stream MJPEG en 640x360 con overlay de visión OpenCV.
3. **Telemetría Sincronizada:** Eventos de detección ($ex$, $ey$, dist, color, confianza) y trazas seriales `#` de la MegaPi.

### Ejemplos de Uso

```bash
# Grabar durante 30 segundos con nombre descriptivo
python herramientas/grabar_sesion.py -d 30 -n prueba_slot1

# Grabar de forma interactiva (presionar ENTER para detener al terminar la ronda)
python herramientas/grabar_sesion.py -n corrida_completa_01

# Grabar solo el stream del robot (sin cámara externa)
python herramientas/grabar_sesion.py --solo-robot -d 20

# Grabar solo la cámara externa MSI
python herramientas/grabar_sesion.py --solo-externa -d 20

# Grabar sin generar el video compuesto automáticamente
python herramientas/grabar_sesion.py -d 15 --no-analisis
```

---

## 📊 2. Análisis y Video Side-by-Side (`analizar_sesion.py`)

Procesa una sesión previamente grabada para generar un video compuesto lado a lado y un informe de diagnóstico.

```bash
# Analizar la última sesión grabada
python herramientas/analizar_sesion.py --ultima

# Analizar una sesión específica
python herramientas/analizar_sesion.py --sesion sesiones/sesion_20260901_015047_prueba_test
```

### Salida generada en cada sesión:
```
sesiones/sesion_YYYYMMDD_HHMMSS_etiqueta/
├── camara_externa.mp4               # Video de la cámara MSI StarCam
├── camara_robot.mp4                 # Video del stream onboard de la Raspberry Pi
├── comparativa_side_by_side.mp4     # Video compuesto lado a lado
├── telemetria.jsonl                 # Registro de eventos con timestamp epoch e ISO
├── metadatos.json                   # Duración, FPS y fechas de inicio/fin
├── reporte_diagnostico.md           # Informe con tablas y métricas
└── snapshots/                       # Capturas clave en momentos de eventos
    ├── snap_001.jpg
    └── snap_002.jpg
```

---

## 🔄 3. Sincronización y Control de la Raspberry Pi (`sync_pi.py`)

Permite mantener el código sincronizado con la Raspberry Pi del robot (`robot-pi` / `192.168.0.166`) y controlar su servicio systemd sin necesidad de ingresar credenciales.

```bash
# Subir cambios locales de vision/ a la Pi y reiniciar el servicio wro-vision
python herramientas/sync_pi.py push

# Descargar configuraciones (config.json) y logs generados en la Pi a local
python herramientas/sync_pi.py pull

# Reiniciar el servicio de visión en la Pi
python herramientas/sync_pi.py restart

# Ver estado de dispositivos (/dev/video0, /dev/ttyUSB0) y servicio
python herramientas/sync_pi.py status

# Ver últimas 50 líneas de log del servicio en vivo
python herramientas/sync_pi.py logs -n 50

# Ejecutar pruebas unitarias de visión directamente en la Pi
python herramientas/sync_pi.py test
```

---

## 🌐 4. Endpoints Web y Telemetría en Vivo de la Raspberry Pi

Cuando `vision_server.py` se ejecuta en la Raspberry Pi, expone en el puerto `8080`:

- `http://192.168.0.166:8080/` : Dashboard web con video en vivo y log interactivo.
- `http://192.168.0.166:8080/stream` : Feed MJPEG de video con overlay de OpenCV.
- `http://192.168.0.166:8080/telemetria` : Lista completa de eventos cronológicos en JSON.
- `http://192.168.0.166:8080/telemetria/reset` : Limpia el buffer de telemetría para una nueva prueba.
- `http://192.168.0.166:8080/estado` : Estado actual del servidor, FPS y conexión serial.

---

## ⚙️ Requisitos

- **Windows:** Python 3.10+ y `ffmpeg` instalado y agregado al PATH.
- **Red:** Conexión a la red local del robot (`192.168.0.166` o `robot-pi`).
