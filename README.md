# Robot WRO RoboMission Junior 2026 — Colegio Los Cedros

Robot autónomo en desarrollo para **WRO RoboMission Junior 2026: Heritage
Heroes**, dentro de la temporada *Robots Meet Culture*. El proyecto combina una
controladora **Makeblock MegaPi** para movimiento y mecanismos con una
**Raspberry Pi 3B** para visión artificial, telemetría y diagnóstico.

> **Zona en desarrollo actualmente (1 de septiembre de 2026):** misión 3.3,
> desde la **zona de excavación** hasta el **museo**. El robot identifica,
> recoge y transporta los cuatro artefactos aleatorios hacia los exhibidores de
> su color. Las misiones de visitantes, torres y limpieza del empedrado todavía
> no están integradas en la rutina principal.

## Objetivo de la competencia

El campo Junior 2026 representa una fortaleza histórica. El reto oficial se
divide en cuatro misiones principales:

1. Guiar cuatro visitantes hacia las áreas de su color.
2. Reconstruir las torres rojas y completar las torres amarillas.
3. Llevar los artefactos hallados en la excavación al museo.
4. Retirar diez partículas de suciedad del empedrado.

Las barreras y el loro son objetos de bonificación: deben permanecer sin daños
y en su posición. El reglamento internacional permite escoger qué misiones
resolver y en qué orden; la implementación actual prioriza los artefactos.

### Artefactos que debe recolectar el robot

Existen cinco artefactos posibles, uno de cada color:

| Color | Destino en el museo |
|---|---|
| Rojo | Exhibidor rojo |
| Verde | Exhibidor verde |
| Negro | Exhibidor negro |
| Azul | Exhibidor azul |
| Amarillo | Exhibidor amarillo |

En cada ronda se colocan **solo cuatro de los cinco**, elegidos y ordenados
aleatoriamente sobre los cuatro cuadros negros de la zona de excavación. Por lo
tanto, siempre falta un color y su exhibidor debe quedar vacío. En la pista y en
el código del equipo, los destinos se manejan de izquierda a derecha —desde la
perspectiva de aproximación del robot— como:

```text
ROJO → VERDE → NEGRO → AZUL → AMARILLO
```

Para obtener la puntuación completa de esta misión, cada artefacto debe quedar
vertical y completamente dentro del exhibidor del mismo color. Son 15 puntos
por artefacto, hasta un máximo de 60.

## Estrategia que se está desarrollando

El sketch activo es
[`robot_WRO/prueba_vision/prueba_vision.ino`](robot_WRO/prueba_vision/prueba_vision.ino).
La secuencia prevista es:

1. Iniciar centrado frente a la fila de cuatro artefactos.
2. Visitar primero los dos slots centrales y luego los exteriores
   (`1, 2, 0, 3`).
3. Hacer un desplazamiento en **L**, con giros controlados por giroscopio, para
   quedar alineado con el centro del slot.
4. Solicitar a la Raspberry el reconocimiento automático del color y corregir
   posición, rumbo y distancia con la cámara.
5. Ejecutar una captura mecánica en dos pasos: aislar el artefacto, embocarlo en
   la pala y cerrarlo entre las dos paletas.
6. Volver al eje central, girar 180°, desplazarse al exhibidor correspondiente
   y depositar el artefacto.
7. Retirarse por una zona despejada, regresar a la excavación y repetir hasta
   cuatro veces o hasta alcanzar el límite de seguridad de 116 segundos.

La navegación usa encoders para distancia y giroscopio para rumbo. Los valores
de separación entre slots, recorrido hasta el museo, ángulos de servo y
distancia de precaptura siguen siendo parámetros de calibración física.

### Estado funcional

| Subsistema | Estado actual |
|---|---|
| Comunicación Raspberry–MegaPi | Operativa por USB serial a 115200 baudios |
| Visión de cinco colores | Implementada con OpenCV/HSV a 640×360 |
| Selección automática del artefacto | Implementada con confianza y votación temporal |
| Centrado y aproximación | Implementados; en calibración sobre la pista real |
| Captura con pala y garra | Basada en la rutina mecánica probada de `prueba_centrales` |
| Ruta excavación → museo → excavación | Implementada; en ajuste de repetibilidad |
| Ciclo de cuatro artefactos | Programado; todavía requiere validación física completa |
| Visitantes, torres y empedrado | Pendientes en la rutina principal |

## Arquitectura del robot

```text
Cámara USB
    │ imagen 640×360 MJPG
    ▼
Raspberry Pi 3B ── OpenCV / HSV / telemetría
    │ USB serial: color, error, distancia, confianza
    ▼
Makeblock MegaPi ── encoders + giroscopio + control de trayectoria
    ├── 2 motores de tracción
    ├── 2 servos SG90 para la garra
    └── 1 servo MG996R para la pala/elevador
```

La Raspberry decide **qué se ve y dónde está**. La MegaPi decide **cómo se
mueve el robot y cómo acciona la garra**. Si se pierde visión o navegación, el
sketch detiene la secuencia para evitar continuar a ciegas.

## Hardware

### Componentes Makeblock Ultimate 2.0 utilizados

| Componente | Función en este robot |
|---|---|
| **MegaPi**, basada en ATmega2560 | Control principal de tiempo real y ejecución del sketch Arduino |
| **2 motores DC de 25 mm con encoder** | Tracción diferencial y medición de desplazamiento |
| **Módulos MegaPi Encoder/DC Motor Driver** | Potencia y lectura de los motores conectados a `SLOT1` y `SLOT2` |
| **Me 3-Axis Accelerometer and Gyro Sensor** | Corrección de rumbo y giros de 90°/180° |
| **MegaPi Shield/adapter para RJ25** | Acceso a puertos para sensores y señales de los servos |
| **Ruedas, ejes, soportes, vigas y placas de aluminio Makeblock** | Chasis rígido y estructura modular del mecanismo |

El kit también incluye sensor ultrasónico, seguidor de línea, Bluetooth y otros
módulos, pero no son dependencias de la rutina activa de recolección. El código
conserva dos entradas analógicas para sensores reflectivos TCRT; por ahora la
navegación principal se basa en visión, encoder y giroscopio.

### Componentes añadidos por el equipo

| Componente | Función |
|---|---|
| **Raspberry Pi 3B** | Procesamiento de visión, servidor web, telemetría y enlace serial |
| **Cámara USB UVC** | Imagen frontal de los artefactos y realimentación para el centrado |
| **2 microservos SG90** | Apertura y cierre independiente de las paletas de la garra |
| **1 servo MG996R** | Movimiento de mayor torque para pala, elevación, transporte y depósito |
| **2 sensores reflectivos TCRT** | Hardware auxiliar conectado a `A4` y `A3`, reservado para referencias de pista |
| **Botón físico de inicio** | Arranque controlado de la rutina después de colocar el robot |
| **Alimentación independiente para la Raspberry** | Evita cargar el regulador de la MegaPi; ambas se comunican por USB |

## Software y visión

El servidor de visión corre en Python sobre la Raspberry Pi 3B. Usa OpenCV para
segmentar rojo, verde, negro, azul y amarillo en HSV, filtrar reflejos y zonas
ocupadas por la propia garra, elegir el candidato mejor alineado y estimar la
distancia mediante una tabla calibrada.

La trama principal enviada a la MegaPi es:

```text
T <found> <color> <ex> <ey> <area> <dist_mm> <fps> <confianza>
```

Además del enlace serial, el sistema ofrece:

- streaming MJPEG con overlay en `http://192.168.0.166:8080/stream`;
- estado del servidor en `/estado`;
- telemetría cronológica en `/telemetria`;
- calibración HSV desde el navegador en el puerto `8081`;
- grabación sincronizada de la cámara externa, cámara del robot y eventos;
- servicio `systemd` con autoarranque en la Raspberry.

Los detalles del detector, calibración y protocolo están en
[`vision/README.md`](vision/README.md).

## Estructura del repositorio

```text
WRO-RM-2026/
├── robot_WRO/
│   ├── prueba_vision/       # Rutina activa: excavación y museo
│   ├── prueba_centrales/    # Desarrollo y calibración mecánica
│   └── pruebas/             # Experimentos de movimiento, giros y servos
├── vision/                  # Visión, protocolo serial, calibradores y servicio
├── herramientas/            # Sincronización, grabación y análisis de pruebas
├── test/                    # Pruebas aisladas de hardware
├── tests_resolucion/        # Evaluación de cámara, resolución y campo visual
├── sesiones/                # Salidas locales de pruebas; no se versionan
└── AGENTS.md                # Rutas y procedimientos operativos del proyecto
```

## Flujo de trabajo

### Comprobar la Raspberry y desplegar visión

```bash
python herramientas/sync_pi.py status
python herramientas/sync_pi.py test
python herramientas/sync_pi.py push
```

### Calibrar en la iluminación de la pista

```bash
cd vision
python3 calibrar_camara.py  # controles UVC, puerto 8082
python3 calibrar_web.py     # HSV y geometría, puerto 8081
```

### Grabar y analizar una corrida

```bash
python herramientas/grabar_sesion.py -d 30 -n prueba_artefacto_verde
python herramientas/analizar_sesion.py --ultima
```

Consulta [`herramientas/README.md`](herramientas/README.md) para ver todos los
comandos y archivos producidos.

## Seguridad y competencia

- La rutina no comienza hasta soltar el botón físico de inicio.
- Las pruebas deben realizarse con la pista despejada y posibilidad de cortar
  alimentación inmediatamente.
- La Raspberry debe usar una fuente de 5 V adecuada e independiente de la
  MegaPi.
- Wi‑Fi, Bluetooth, SSH y las páginas web son herramientas de desarrollo. La
  comunicación inalámbrica debe desactivarse durante una ronda oficial; el
  enlace interno Raspberry–MegaPi funciona por USB.
- Las reglas nacionales pueden diferir de las internacionales. Debe prevalecer
  siempre la versión entregada por el organizador nacional.

## Referencias oficiales

- [WRO 2026 — temporada Robots Meet Culture](https://wro-association.org/competition/2026-season/)
- [Reglas WRO 2026 RoboMission Junior — Heritage Heroes](https://wro-association.org/wp-content/uploads/WRO-2026-RoboMission-Junior-Game-Rules.pdf)
- [Makeblock Ultimate 2.0 — documentación oficial](https://support.makeblock.com/hc/en-us/articles/1500003399081-About-mBot-Ultimate-2-0)
- [Manual y lista de piezas Ultimate 2.0](https://download.makeblock.com/Ultimate-V2.0_EN_Original%20Forms.pdf)
