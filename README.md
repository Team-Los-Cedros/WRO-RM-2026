# WRO RoboMission Junior 2026 - Colegio Los Cedros

Repositorio central del robot autónomo para la WRO RoboMission 2026.

El sistema está compuesto por una controladora **MegaPi** (control motriz, servos, giroscopio y encoders) coordinada vía puerto serie con una **Raspberry Pi 3B** (procesamiento de visión artificial con OpenCV en tiempo real).

---

## 📁 Estructura del Repositorio

```text
WRO-RM-2026/
├── robot_WRO/             # Sketches de Arduino / C++ para la MegaPi
│   ├── prueba_vision/     # Rutina principal coordinada (4 artefactos + museo)
│   ├── prueba_centrales/  # Calibración de secuencias mecánicas de la garra
│   └── ...
├── vision/                # Servidor de visión en Python para Raspberry Pi 3B
│   ├── vision_core.py     # Pipeline de detección HSV, filtros y geometría
│   ├── vision_server.py   # Servidor serial, streaming web y telemetría
│   ├── calibrar_web.py    # Interfaz web para calibración de colores en pista
│   └── config.json        # Configuración de cámara, HSV y distancias
├── herramientas/          # Suite de depuración y grabación sincronizada en PC
│   ├── grabar_sesion.py   # Grabador multi-cámara (MSI StarCam + Stream Robot + Logs)
│   ├── analizar_sesion.py # Generador de video Side-by-Side y reporte diagnóstico
│   ├── sync_pi.py         # Sincronizador de código y control SSH con la Pi
│   └── README.md          # Documentación detallada de las herramientas
├── sesiones/              # Grabaciones de pruebas, videos compuestos y logs
└── AGENTS.md              # Memoria operativa y reglas para agentes de IA
```

---

## 🚀 Inicio Rápido

### 1. Sincronizar y desplegar a la Raspberry Pi
```bash
python herramientas/sync_pi.py push
```

### 2. Monitoreo en vivo
- **Streaming y Telemetría:** `http://192.168.0.166:8080`
- **Calibrador de colores:** `http://192.168.0.166:8081`

### 3. Grabar una prueba del robot
```bash
# Grabar 30 segundos de prueba con video multi-cámara y telemetría
python herramientas/grabar_sesion.py -d 30 -n prueba_slot1
```

Consulta [`herramientas/README.md`](herramientas/README.md) y [`vision/README.md`](vision/README.md) para más detalles.
