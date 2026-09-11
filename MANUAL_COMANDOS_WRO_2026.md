# Manual de Guía Rápida: Comandos de la Raspberry Pi y Operación WRO RoboMission Junior 2026

**Equipo:** Colegio Los Cedros — WRO RoboMission Junior 2026 (*Heritage Heroes*)  
**Plataforma:** Raspberry Pi 3B (Debian 13 arm64) + Makeblock MegaPi (ATmega2560)  
**Última actualización:** Temporada 2026

---

## 📑 Tabla de Contenidos

1. [Ficha Técnica y Conectividad](#1-ficha-técnica-y-conectividad)
2. [Comandos de Control desde la PC de Desarrollo](#2-comandos-de-control-desde-la-pc-de-desarrollo)
3. [Comandos en la Raspberry Pi (SSH / Consola)](#3-comandos-en-la-raspberry-pi-ssh--consola)
4. [Herramientas Web de Calibración en Vivo](#4-herramientas-web-de-calibración-en-vivo)
5. [Protocolo de Comunicación Serial (MegaPi ↔ Raspberry Pi)](#5-protocolo-de-comunicación-serial-megapi--raspberry-pi)
6. [Programación de la Ruta en la MegaPi](#6-programación-de-la-ruta-en-la-megapi)
7. [Comandos Críticos para el Día de la Competencia](#7-comandos-críticos-para-el-día-de-la-competencia)
8. [Checklist Operativo de Competencia (Flujo en Boxes y Mesa Oficial)](#8-checklist-operativo-de-competencia)

---

## 1. Ficha Técnica y Conectividad

### 🔌 Parámetros de Red y Enlace

| Parámetro | Valor |
|---|---|
| **Hostname / Alias SSH** | `robot-pi` (también responde por `clc-wro-rm.local` y `192.168.0.166`) |
| **Usuario** | `pi` |
| **Autenticación SSH** | Clave privada `~/.ssh/id_robot_ed25519` (configurada en `~/.ssh/config`, sin clave interactiva) |
| **Permisos de Administrador** | `sudo` sin contraseña (`NOPASSWD`) habilitado en la Pi |
| **Directorio raíz del proyecto** | `/home/pi/WRO-RM-2026` |
| **Directorio de visión** | `/home/pi/WRO-RM-2026/vision` |
| **Cámara USB (UVC)** | `/dev/video0` (USB Camera 320x240 @ 20 fps reales) |
| **Controlador MegaPi** | `/dev/ttyUSB0` (115200 baudios, 8N1) |

### 🌐 Puertos Web y Servicios Expuestos

| Puerto | Herramienta | URL Local / Navegador | Función |
|---|---|---|---|
| **8080** | `vision_server.py` | `http://robot-pi:8080/` | Dashboard en vivo, streaming MJPEG y telemetría JSON |
| **8081** | `calibrar_web.py` | `http://robot-pi:8081/` | Calibrador HSV de artefactos 3D y centro de garra (`cx_garra`) |
| **8082** | `calibrar_camara.py` | `http://robot-pi:8082/` | Calibrador de controles UVC (exposición manual, ganancia, balance de blancos) |
| **8083** | `calibrar_destinos.py` | `http://robot-pi:8083/` | Calibrador HSV de cuadros planos del museo y marcos blancos |
| **8084** | `calibrar_torres.py` | `http://robot-pi:8084/` | Calibrador de torres de recolección (corona abierta) y destino (tapa negra) |

---

## 2. Comandos de Control desde la PC de Desarrollo

Ejecutar en la terminal de la PC (PowerShell, Git Bash o CMD) desde la raíz del proyecto `WRO-RM-2026`:

### 🔄 Sincronización Bidireccional y Diagnóstico (`herramientas/sync_pi.py`)

```bash
# 1. Enviar cambios locales de vision/ a la Pi y reiniciar automáticamente wro-vision.service
python herramientas/sync_pi.py push

# 2. Descargar la configuración calibrada (config.json) y los logs de la Pi a la PC
python herramientas/sync_pi.py pull

# 3. Consultar el estado del servicio wro-vision, /dev/video0 y /dev/ttyUSB0
python herramientas/sync_pi.py status

# 4. Ver en vivo las últimas 50 líneas de log del servicio en la Raspberry Pi
python herramientas/sync_pi.py logs -n 50

# 5. Reiniciar el servicio de visión en la Pi remotamente
python herramientas/sync_pi.py restart

# 6. Ejecutar las pruebas unitarias de visión directamente en la Raspberry
python herramientas/sync_pi.py test
```

### 🎥 Grabación Coordinada de Pruebas en Pista (`herramientas/grabar_sesion.py`)

Graba de forma sincronizada:
1. Cámara Externa (MSI StarCam 370i u otra webcam de mesa a 640x480).
2. Cámara Onboard del robot (stream MJPEG con overlay de detección).
3. Telemetría en JSONL (eventos de visión y trazas `#` de la MegaPi).

```bash
# Grabar una corrida de 30 segundos con nombre descriptivo
python herramientas/grabar_sesion.py -d 30 -n prueba_slot1

# Grabar corrida completa en modo interactivo (presionar ENTER para finalizar la grabación al detenerse el robot)
python herramientas/grabar_sesion.py -n corrida_oficial_01

# Grabar únicamente el stream del robot (sin cámara externa conectada a la PC)
python herramientas/grabar_sesion.py --solo-robot -d 20

# Grabar únicamente la cámara externa de la mesa
python herramientas/grabar_sesion.py --solo-externa -d 20

# Grabar sin generar el video compuesto inmediatamente (útil para no demorar entre pruebas rápidas)
python herramientas/grabar_sesion.py -d 25 --no-analisis
```

### 📊 Análisis Side-by-Side y Evaluación Offline (`herramientas/analizar_sesion.py`)

```bash
# Analizar y generar video compuesto Side-by-Side de la última corrida grabada
python herramientas/analizar_sesion.py --ultima

# Analizar una carpeta de sesión específica
python herramientas/analizar_sesion.py --sesion sesiones/sesion_20260901_015047_prueba_test

# Evaluar algoritmos de visión sobre un video grabado previamente (sin encender el robot ni la Pi)
python herramientas/evaluar_vision.py sesiones/sesion_20260901_015047_prueba_test/camara_robot.mp4 --modo FILA
```

---

## 3. Comandos en la Raspberry Pi (SSH / Consola)

Para ingresar por consola segura a la Raspberry Pi:
```bash
ssh robot-pi
# O directamente por dirección IP:
ssh pi@192.168.0.166
```

### ⚙️ Control del Servicio Systemd (`wro-vision.service`)

El servicio ejecuta `vision_server.py` automáticamente al arrancar la Pi.

```bash
# Ver estado del servicio (activo, PID, consumo de memoria y últimas líneas)
sudo systemctl status wro-vision

# Reiniciar el servicio (tras modificar config.json manualmente)
sudo systemctl restart wro-vision

# Detener el servicio (requerido si vas a correr vision_server.py o pruebas a mano)
sudo systemctl stop wro-vision

# Iniciar el servicio
sudo systemctl start wro-vision

# Monitorear logs del servicio en tiempo real (seguimiento continuo con -f)
journalctl -u wro-vision -f

# Ver las últimas 100 líneas del log del servicio sin paginación
journalctl -u wro-vision -n 100 --no-pager

# Habilitar autoarranque en el inicio del sistema
sudo systemctl enable wro-vision

# Deshabilitar autoarranque
sudo systemctl disable wro-vision
```

---

### 👁️ Ejecución Manual de `vision_server.py`

Ubicarse en la carpeta: `cd /home/pi/WRO-RM-2026/vision`

```bash
# 1. Modo producción (con enlace serie a la MegaPi y dashboard web en puerto 8080)
python3 vision_server.py

# 2. Modo prueba de mesa: sin MegaPi conectada (imprime las tramas 'T ...' en la consola)
python3 vision_server.py --sin-serial --web --color AUTO

# 3. Modo verboso con seguimiento inicial de un color específico
python3 vision_server.py --verbose --color VERDE

# 4. Especificar archivo de configuración alternativo y puerto web personalizado
python3 vision_server.py --config config_alternativo.json --puerto-web 8090
```

---

### 🧪 Pruebas Unitarias y Validación en la Pi

```bash
cd /home/pi/WRO-RM-2026/vision

# 1. Probar detector y lógica de decisión sin cámara (usa imágenes sintéticas)
python3 test_detector.py

# 2. Probar protocolo serial completo usando puertos virtuales PTY
python3 test_protocolo.py

# 3. Verificar que no haya errores de sintaxis en los scripts Python
python3 -m py_compile vision_core.py vision_server.py calibrar_web.py calibrar_camara.py
```

---

### 🌐 Consultas Rápidas por Terminal a la API de Visión (CURL)

Cuando `vision_server.py` o el servicio están corriendo:

```bash
# Ver estado actual del servidor (FPS, modo, si serial está conectado)
curl -s http://localhost:8080/estado | jq .

# Ver eventos de telemetría recientes
curl -s http://localhost:8080/telemetria | tail -n 15

# Reiniciar el buffer de telemetría para una nueva corrida
curl -s http://localhost:8080/telemetria/reset
```

---

## 4. Herramientas Web de Calibración en Vivo

> **Gestor de Exclusividad de Cámara:** Todos los scripts de calibración implementan `GestorExclusividadCamara`. Al ejecutarse, **pausan temporalmente el servicio `wro-vision`** para tomar control exclusivo de `/dev/video0`, y lo reanudan automáticamente al cerrarse con `Ctrl+C`.

### 1️⃣ Calibrador de Cámara UVC (`calibrar_camara.py`)
* **Propósito:** Ajustar iluminación, exposición manual (para evitar desenfoque por movimiento), balance de blancos y ganancia antes de calibrar colores.
* **Puerto:** `8082`
* **Comando:**
  ```bash
  cd /home/pi/WRO-RM-2026/vision
  python3 calibrar_camara.py --puerto 8082
  ```
* **Uso:** Abrir `http://robot-pi:8082`. Presionar **Auto-Medir y Fijar** para que la cámara mida la luz ambiental de la pista y fije exposición y balance de blancos en manual. Luego presionar **Guardar en config.json**.

---

### 2️⃣ Calibrador HSV de Artefactos 3D (`calibrar_web.py`)
* **Propósito:** Ajustar rangos HSV de los objetos 3D (`ROJO`, `VERDE`, `AZUL`, `AMARILLO`, `NEGRO`), centro de garra (`cx_garra`) y tabla de distancias `ey -> mm`.
* **Puerto:** `8081`
* **Comando:**
  ```bash
  cd /home/pi/WRO-RM-2026/vision
  python3 calibrar_web.py --puerto 8081
  ```
* **Uso:** Abrir `http://robot-pi:8081`. Seleccionar el color deseado, ajustar los deslizadores de H, S y V. Verificar que los dedos azules de la garra queden enmascarados en las `zonas_ignoradas`.

---

### 3️⃣ Calibrador de Destinos del Museo (`calibrar_destinos.py`)
* **Propósito:** Calibrar los cuadros planos impresos en la pista (área del museo) sin alterar la calibración de los artefactos 3D.
* **Puerto:** `8083`
* **Comando:**
  ```bash
  cd /home/pi/WRO-RM-2026/vision
  python3 calibrar_destinos.py --puerto 8083
  ```
* **Uso:** Abrir `http://robot-pi:8083`. Ajusta la región de interés (ROI vertical) y la detección del marco blanco exterior que rodea al cuadro.

---

### 4️⃣ Calibrador de Torres (`calibrar_torres.py`)
* **Propósito:** Calibrar la distinción entre Torres a Recolectar (cuerpo amarillo + corona abierta blanca) y Torres de Destino (cuerpo amarillo + tapa cerrada negra).
* **Puerto:** `8084`
* **Comando:**
  ```bash
  cd /home/pi/WRO-RM-2026/vision
  python3 calibrar_torres.py --puerto 8084
  ```
* **Uso:** Abrir `http://robot-pi:8084`. Modifica exclusivamente la sección `torres` de `config.json`.

---

## 5. Protocolo de Comunicación Serial (MegaPi ↔ Raspberry Pi)

* **Parámetros de enlace:** 115200 baudios, 8 bits de datos, sin paridad, 1 bit de parada (8N1).
* **Terminación de línea:** `\n`.

### 📤 MegaPi ➔ Raspberry Pi (Comandos)

| Comando | Acción |
|---|---|
| `C AUTO` | Identifica el artefacto centrado frente a la garra y fija el seguimiento |
| `C <COLOR>` | Sigue exclusivamente ese color (`ROJO`, `VERDE`, `AZUL`, `AMARILLO`, `NEGRO`) |
| `M PAUSA` | Detiene procesamiento visual intensivo; mantiene latido serial activo |
| `M ARTEFACTO <COLOR>` | Activa el detector de artefactos 3D para el color indicado |
| `M DESTINO <COLOR>` | Activa el detector de cuadros planos del museo con marco blanco |
| `M TORRE_REC` | Modo torre a recolectar (corona blanca superior abierta) |
| `M TORRE_DEST` | Modo torre de destino (tapa negra superior cerrada) |
| `M FILA` y luego `F` | Devuelve los colores de los cuatro artefactos ordenados de izquierda a derecha |
| `X` | Escaneo diagnóstico de los 5 colores en el cuadro actual |
| `S 0` / `S 1` | Pausa (`0`) o reanuda (`1`) el streaming continuo de tramas `T` |
| `P` | Ping (la Pi responde con versión de firmware y modos disponibles) |
| `# mensaje` | Mensaje de depuración emitido por la MegaPi (la Pi lo registra en telemetría sin procesarlo) |

### 📥 Raspberry Pi ➔ MegaPi (Trama periódica por cuadro)

```text
T <found> <color> <ex> <ey> <area> <dist> <fps> <confianza>
```

* **Ejemplo:** `T 1 VERDE -14 112 1840 160 22 92`

| Campo | Significado |
|---|---|
| `found` | `1` si hay un objetivo válido detectado; `0` si no hay detección |
| `color` | Color identificado o `NINGUNO` |
| `ex` | Error horizontal en píxeles respecto al centro de garra (`ex < 0`: girar a la izquierda; `ex > 0`: girar a la derecha) |
| `ey` | Fila inferior del contorno en píxeles (aumenta al aproximarse al objeto) |
| `area` | Área en píxeles del contorno detectado |
| `dist` | Distancia estimada al objeto en milímetros (según `tabla_distancia`) |
| `fps` | Frecuencia de cuadros por segundo de la cámara |
| `confianza` | Calidad de la forma geométrica (0 a 100) |

---

## 6. Programación de la Ruta en la MegaPi

| Archivo | Para qué sirve |
|---|---|
| `robot_WRO/ruta_visitantes/prueba_wwl5_opt/prueba_wwl5_opt.ino` | **Sketch de trabajo.** La ruta se escribe dentro de `loop()`. |
| `robot_WRO/ruta_visitantes/calibrar_gyro/calibrar_gyro.ino` | Herramienta de calibración por Monitor Serie (menú de letras). |
| `DIAGNOSTICO_MEGAPI.md` | Por qué el código es así: análisis del firmware, las librerías y las mediciones. |

---

### 🥇 A. Las cuatro reglas de oro

1. **Los grados de giro son reales.** No descuentes la pasada del frenado. Si quieres 90°, escribe `90.0`. El robot mide cuánto se pasó al frenar y lo corrige solo.
2. **El último parámetro es un TIEMPO LÍMITE, no una espera.** La función vuelve en cuanto termina el movimiento. Ponerlo holgado no cuesta ni una décima; ponerlo corto corta la maniobra a la mitad.
3. **La velocidad ya no da precisión.** Sube las velocidades. Solo baja una cuando haya una razón *mecánica* (no tumbar la torre, no empujar el artefacto de más).
4. **Nunca uses `delay()` dentro de la ruta.** Usa `_delay(segundos)`. `delay()` congela el giroscopio, los encoders y el watchdog.

---

### 📏 B. Unidades

| Magnitud | Unidad | Detalle |
|---|---|---|
| **Giros** | grados reales del robot | Los mide el giroscopio. Sin recorte de ±180: `180.0` y `360.0` son válidos. |
| **Distancias** | grados del eje de salida | Una vuelta completa de rueda = `360`. |
| **Velocidad** | rpm del eje de salida | `PWM = rpm × 1,61` (con `RPM_MAX = 158`). |
| **Tiempos** | segundos | Siempre tiempo **límite**, nunca espera. |

**Convertir milímetros a grados de encoder** — mídelo una vez y anótalo aquí:

```c
// 1. Marca el suelo, ejecuta esto solo, y mide con una regla lo que avanzó:
avanzar(1000, V_RECTO, 8.0);

// 2. MM_POR_GRADO = (milímetros medidos) / 1000
// 3. A partir de ahí:  grados = milímetros / MM_POR_GRADO
```

> Valor medido en esta pista: `MM_POR_GRADO = ______`  ← **rellenar**

---

### 🚗 C. Referencia de funciones de movimiento

```c
avanzar   (long grados, float velocidad, float timeoutSeg);
retroceder(long grados, float velocidad, float timeoutSeg);

girarIzquierdaGyro(float grados, float velocidad);   // timeout por defecto 6 s
girarDerechaGyro  (float grados, float velocidad);
girarGyro         (float grados, float velocidad, int sentido, float timeoutSeg);
                                                     // sentido: +1 izquierda, -1 derecha

detener(float segundos);        // frena en seco y espera
_delay (float segundos);        // pausa manteniendo vivos giroscopio y encoders
```

**Con sensores de línea:**

```c
// Avanza recto hasta cruzar una línea negra perpendicular y se centra sobre ella
avanzarRectoGyroLineaPerpendicular(long  gradosMaximos,
                                   float velocidadBase,
                                   float timeoutSeg          = 10.0,
                                   long  rango               = 135,   // barrido de centrado
                                   long  offsetCentro        = 0,     // corrimiento fino
                                   float velocidadEscaneo    = 35.0,
                                   float Kp                  = 1.5,   // ignorado
                                   float gradosMinimosBusqueda = 0.0);

// Gira hasta encontrar la línea (o hasta gradosMaximos) y luego se centra
girarIzquierdaGyroLinea(float gradosMaximos, float velocidad, float gradosMinimosBusqueda, float timeoutSeg = 4.0);
girarDerechaGyroLinea  (float gradosMaximos, float velocidad, float gradosMinimosBusqueda, float timeoutSeg = 4.0);

centrarEnLinea           (float velocidad, float timeoutSeg);
centrarLineaPerpendicular(float velocidad, float timeoutSeg = 3.0, int ladoInicial = 0);

seguirLineaTiempo (float segundos, float velocidadBase, float ganancia = 45.0);
seguirLineaGrados (long grados,    float velocidadBase, float ganancia = 45.0, float tiempoMax = 10.0);

int  leerLineaIzq();  bool esNegroIzq();   // A4, analógico, umbral UMBRAL_NEGRO_IZQ
int  leerLineaDer();  bool esNegroDer();   // pin 2, digital
void imprimirSensoresLinea();              // para ajustar los umbrales
```

**Compatibilidad (siguen existiendo, pero no las uses en ruta nueva):**
`avanzarRectoGyro(grados, vel, Kp, tiempo)` — el `Kp` se ignora, equivale a `avanzar()`.
`girarIzquierda/girarDerecha(gradosRueda, vel, t)` — giro medido por encoder, sin giroscopio.

---

### ⚡ D. Velocidades: mínimos y máximos

Constantes ya definidas en el sketch. Úsalas en vez de números sueltos:

```c
#define V_GIRO_FINO      50.0f   // ajustes pequeños, giros con carga delicada
#define V_GIRO           90.0f   // giro normal
#define V_GIRO_RAPIDO   120.0f   // giros grandes (90°, 160°, 180°)

#define V_APROX          35.0f   // acercarse a un objeto sin tumbarlo
#define V_RECTO_MEDIO    60.0f   // tramos cortos, búsqueda de línea
#define V_RECTO          90.0f   // uso general
#define V_RECTO_RAPIDO  150.0f   // tramos largos
```

| rpm | PWM | Qué es |
|---|---|---|
| 24 / 27 | 39 / 43 | Mínimo absoluto para girar / avanzar |
| **35** | 56 | **Mínimo recomendado.** Arranca siempre, con o sin carga |
| 50 | 81 | Ajustes de pocos grados |
| 90 | 145 | Uso general |
| **120** | 194 | **Verificado**: 5 giros de 90° con ±0,16° de dispersión |
| 150 | 242 | Tramos rectos largos |
| 158 | 255 | Tope. Por encima de 158 se satura, no pasa nada más |

* Por debajo del mínimo el código sube el PWM al suelo automáticamente. **Pedir 20 rpm y pedir 24 dan exactamente lo mismo**, así que poner velocidades bajas ya no sirve para nada.
* **Techo de giros:** 120 rpm está verificado con datos. Puedes probar 150 vigilando que las ruedas no patinen — si patinan, el ángulo sale bien pero el robot además se desplaza y pierde posición.
* **Techo de rectas:** 158 rpm sin problema. El encoder mide la pasada del frenado y la corrige.
* Si al subir la velocidad se pasa de largo, **baja** `GIRO_K_DECEL` / `RECTO_K_DECEL`: un valor más bajo empieza a frenar antes.

---

### 🦾 E. Mecanismos

```c
abrirGarra();      cerrarGarra();      // micro-servos del puerto 7 (A10 / A11)
bajar_pala();      subir_pala();       // servo grande del pin 5
barrer();          posicionar();
depositar();                           // baja despacio de 63° a 70° y abre la garra
recolectar(modo);                      // ver tabla
parabrisas(long pasos, float velocidad = 100.0);   // avanza barriendo con los micro-servos
```

| Llamada | Ángulo del servo grande | Uso típico |
|---|---|---|
| `recolectar(1)` | 63° | Posición de transporte |
| `recolectar(2)` | 120° | Pala arriba / liberada |
| `recolectar(3)` | 111° | Acomodar artefacto |
| `recolectar(4)` | 95° | Sujetar torre |
| `bajar_pala()` | 105° | Bajar a nivel de suelo |
| `subir_pala()` | 105° → 0° (rampa) | Subida lenta |

| Constante | Valor | |
|---|---|---|
| `GARRA_ABIERTA_S1` / `GARRA_ABIERTA_S2` | 0 / 180 | Garra abierta |
| `GARRA_CERRADA_S1` / `GARRA_CERRADA_S2` | 108 / 58 | Garra cerrada |

---

### 🔧 F. Constantes de calibración

Están todas juntas en el bloque **1. CONFIGURACIÓN** al principio del sketch.

| Constante | Valor actual | De dónde sale | Cuándo volver a medirla |
|---|---|---|---|
| `RPM_MAX` | `158.0f` | Opción `v` del calibrador | Al cambiar motores o batería |
| `TRIM_MOTOR_IZQ` | `0.938f` | `158,3 / 168,7` de la opción `v` | Al cambiar motores |
| `GIRO_PWM_MIN` | `39` | Opción `m` | Al cambiar de superficie o ruedas |
| `RECTO_PWM_MIN` | `43` | Opción `m` | Ídem |
| `GYRO_ESCALA` | `1.000f` | Opción `a` | Solo si los giros salen sistemáticamente cortos o largos |
| `UMBRAL_NEGRO_IZQ` | `31` | `imprimirSensoresLinea()` | **Cada vez que cambie la luz de la sala** |
| `UMBRAL_NEGRO_DER` | `41` | Ídem (sensor digital, se ajusta con su potenciómetro) | Ídem |
| `DEPURAR` | `1` | — | **Ponlo a `0` en competencia** |

**Menú del calibrador** (Monitor Serie a 115200, escribir una letra):

| Tecla | Prueba |
|---|---|
| `s` | Salud del bus I2C (errores y µs por lectura) |
| `b` | Sesgo, ruido y deriva del giroscopio (robot quieto) |
| `a` | **Escala del giroscopio automática** — el robot da 3 vueltas él solo |
| `e` | Escala a mano (poco fiable, ver aviso abajo) |
| `v` | Velocidad máxima de cada motor (ruedas al aire) |
| `m` | PWM mínimo de arranque (robot en el suelo) |
| `t` | 5 giros de 90° con el lazo completo |
| `?` | Repetir el menú |

> ⚠️ **No midas la escala del giroscopio girando el robot a mano.** El sensor integra rotación sobre *su propio eje Z*, no rumbo sobre el suelo. En cuanto lo levantas o lo inclinas, las dos cosas dejan de coincidir y salen resultados que se contradicen entre sí. Usa la opción `a`.

---

### 🖥️ G. Leer la salida de depuración

Con `DEPURAR 1`, cada maniobra imprime una línea. **Todas empiezan por `# `** a propósito: ese es el prefijo de mensaje de depuración del protocolo de la sección 5, así que la Raspberry Pi las registra en telemetría sin intentar interpretarlas. **No quites ese prefijo.**

```text
# giro pedido=90.0 real=90.22 ms=812 errI2C=0
# recto pedido=742 real=744 desvio=-0.31 ms=1904
# FIN. Errores I2C: 0 | recuperaciones de bus: 0 | rumbo final: 271.44
```

| Campo | Significado |
|---|---|
| `pedido` | Lo que le pediste |
| `real` | Lo que hizo de verdad, ya con la corrección aplicada |
| `desvio` | Grados que se torció el rumbo durante una recta |
| `ms` | Milisegundos que tardó la maniobra completa |
| `errI2C` | Lecturas de giroscopio fallidas acumuladas. Debería quedarse en `0` |

| Aviso | Qué significa | Qué hacer |
|---|---|---|
| `<<< TIMEOUT` | Se agotó el tiempo límite | Sube el último parámetro de esa llamada |
| `<<< ATASCADO` | El robot estuvo 1,5 s sin avanzar | Problema **mecánico** en ese punto de la pista, no de código |
| `# AVISO: ... reinicio por WATCHDOG` | La placa se colgó y se reinició sola | Revisa el cableado del giroscopio (sección I) |
| `# ERROR: el MPU6050 no responde` | Cable RJ25 suelto o mal conectado | Reconecta el puerto del giroscopio |
| `errI2C` o `recuperaciones` > 0 | El bus sufrió con los motores en marcha | Separa el cable RJ25 de los cables de motor |

---

### 🚦 H. Plantilla de una ruta nueva

```c
void loop()
{
  Serial.println(F("# Listo. Pulsa el boton."));

  while (digitalRead(BOTON_PIN) == HIGH) tarea();   // esperar pulsación
  while (digitalRead(BOTON_PIN) == LOW)  tarea();   // esperar que se suelte
  _delay(0.5);

  gyro.calibrar(150);      // el robot lleva rato quieto: mejor momento para el sesgo
  rutinaIniciada = true;   // a partir de aquí el botón funciona como PARO DE EMERGENCIA

  // ---------------- TU RUTA ----------------
  avanzar(500, V_RECTO, 5.0);
  girarDerechaGyro(90.0, V_GIRO);
  recolectar(1);
  // ...
  // -----------------------------------------

  detener(1.0);
  while (1) tarea();       // no repetir la rutina
}
```

* El **mismo botón** arranca la rutina y, una vez arrancada, la aborta: tres lecturas seguidas en 30 ms frenan los motores y dejan el puente en H en reposo.
* `tarea()` es lo que mantiene vivos el giroscopio, los encoders y el watchdog. Cualquier bucle de espera propio tiene que llamarla.

---

### ⚠️ I. Errores frecuentes

| Error | Consecuencia |
|---|---|
| Usar `delay()` en vez de `_delay()` | El giroscopio deja de integrar, los encoders se desactualizan y el watchdog reinicia la placa al segundo |
| Poner el tiempo límite justo | La maniobra se corta a la mitad y sale `<<< TIMEOUT` |
| Bajar la velocidad "para que salga más preciso" | No mejora nada y alarga la ronda. La precisión la da la fase de corrección |
| Descontar la pasada del frenado en los grados | El robot ya la descuenta solo: se quedaría corto el doble |
| Quitar el prefijo `# ` de los mensajes | La Raspberry Pi intenta interpretarlos como protocolo |
| Dejar `DEPURAR 1` en competencia | Unos milisegundos por maniobra y telemetría innecesaria |
| Conectar un motor al **SLOT4** | Sus pines están ocupados por el servo grande y los TCRT (ver tabla) |
| Añadir más de 12 servos | La librería Servo pasaría a usar Timer1 y **rompería el PWM del motor izquierdo** |

---

### 🔌 J. Pines y timers ocupados

| Función | Pines | Nota |
|---|---|---|
| **Motor izquierdo (SLOT1)** | enc 18 / 31, PWM **12**, dir 34 / 35 | PWM por Timer1 a 7 812 Hz |
| **Motor derecho (SLOT2)** | enc 19 / 38, PWM **8**, dir 37 / 36 | PWM por Timer4 a 7 812 Hz |
| **Giroscopio MPU6050** | I2C: 20 (SDA) / 21 (SCL) | Puerto RJ25 nº 8, bus a 400 kHz |
| **Servo pala** | 5 | Coincide con el PWM del SLOT4 |
| **Micro-servos garra** | A10 / A11 | Puerto RJ25 nº 7 |
| **TCRT izquierdo** | A4 (analógico) | Coincide con DIR1 del SLOT4 |
| **TCRT derecho** | 2 (digital) | Coincide con el encoder A del SLOT4 |
| **Botón de inicio / paro** | 4 | `INPUT_PULLUP` |
| **Serial hacia la Raspberry Pi** | 0 / 1 (USB) | 115200 8N1 |

| Recurso | Estado |
|---|---|
| **SLOT3** | Libre (enc 3 / 49, PWM 9, dir 43 / 42) |
| **SLOT4** | **NO usable** — sus cinco pines están repartidos entre el servo grande y los dos TCRT |
| **Serial1** (pines 18 / 19) | **No disponible** — son las interrupciones de los encoders |
| **Timer0** | `millis()` / `micros()` — no tocar |
| **Timer1 / Timer4** | PWM de los motores de tracción |
| **Timer2** | PWM del SLOT3 |
| **Timer3** | Libre |
| **Timer5** | Librería Servo (hasta 12 servos) |

---

## 7. Comandos Críticos para el Día de la Competencia

### 🛑 A. Cumplimiento Reglamentario WRO (Desactivación de Redes Inalámbricas)

> **Regla WRO:** Durante la inspección técnica, tiempo de cuarentena y rondas oficiales está **estrictamente prohibida la comunicación inalámbrica**. La Raspberry Pi no debe emitir Wi-Fi ni Bluetooth.

```bash
# --- ANTES DE LA INSPECCIÓN Y ENTRAR A CUARENTENA ---
# 1. Bloquear Wi-Fi y Bluetooth por hardware/software:
sudo rfkill block wifi
sudo rfkill block bluetooth

# 2. Comprobar que ambas interfaces figuren como 'blocked: yes':
rfkill list

# --- AL FINALIZAR LA RONDA Y VOLVER A BOXES ---
# Reactivar interfaces para calibrar o descargar telemetría:
sudo rfkill unblock wifi
sudo rfkill unblock bluetooth
```

---

### 🔍 B. Diagnóstico Express de Hardware en Boxes (Checklist de 30 segundos)

Si el robot enciende pero no detecta objetos o la MegaPi no responde:

```bash
# 1. ¿La cámara USB está reconocida por el kernel de Linux?
v4l2-ctl --list-devices
# Debe listar: 'USB Camera ... /dev/video0'

# 2. ¿El convertidor USB-Serial de la MegaPi está conectado?
ls -la /dev/ttyUSB*
# Debe responder: /dev/ttyUSB0 con grupo dialout

# 3. ¿La cámara está bloqueada por un proceso muerto o colgado?
sudo fuser -v /dev/video0
# Si hay un PID fantasma bloqueando el dispositivo, forzar su cierre inmediato:
sudo fuser -k /dev/video0

# 4. Ver los últimos eventos de conexión/desconexión USB en el sistema:
dmesg | grep -E 'usb|tty' | tail -n 20
```

---

### ⚡ C. Salud Eléctrica y Temperatura (¡Vital en Raspberry Pi 3B!)

Una batería descargada o una mala conexión con el convertidor de 5 V / 2.5 A causa **caídas de tensión (Undervoltage)** que reducen la velocidad del procesador o reinician la Pi en mitad de la pista.

```bash
# 1. Consultar estado de subvoltaje y throttling:
vcgencmd get_throttled

# 2. Medir temperatura actual del procesador:
vcgencmd measure_temp
# Valor óptimo: < 65°C. Si supera los 80°C se producirá throttling térmico.

# 3. Medir voltaje del núcleo de la Pi:
vcgencmd measure_volts core
```

#### 📖 Decodificación de `vcgencmd get_throttled`:

| Valor Hexadecimal | Diagnóstico | Acción requerida |
|---|---|---|
| `throttled=0x0` | **Estado óptimo.** Alimentación y temperatura perfectas. | Ninguna. Listo para competir. |
| `throttled=0x50000` | Hubo subvoltaje en algún momento previo desde el encendido. | Revisar conectores de alimentación. |
| `throttled=0x50005` | **¡PELIGRO! Subvoltaje ACTIVO.** La CPU está operando frenada. | **Reemplazar o cargar la batería inmediatamente.** |
| `throttled=0x20000` | Hubo estrangulamiento térmico (sobrecalentamiento previo). | Permitir enfriamiento o mejorar disipación. |

---

### 💾 D. Respaldo y Restauración de Emergencia de la Configuración

Antes de realizar cambios de último minuto en los colores en boxes:

```bash
cd /home/pi/WRO-RM-2026/vision

# 1. Crear copia de respaldo con fecha y hora:
cp config.json config.json.bak_$(date +%H%M)

# 2. Listar respaldos existentes:
ls -lh config.json.bak_*

# 3. Restaurar respaldo previo si la nueva calibración falló:
cp config.json.bak_1030 config.json
sudo systemctl restart wro-vision
```

---

### 🛑 E. Apagado Seguro del Robot

> **ADVERTENCIA CRÍTICA:** **NUNCA** apagues el robot cortando el interruptor principal de la batería sin apagar la Raspberry Pi primero. Esto puede **corromper el sistema de archivos de la tarjeta microSD**, dejando el robot inoperativo antes de una ronda.

```bash
# Ordenar apagado seguro del sistema operativo:
sudo poweroff
# O alternativamente:
sudo shutdown -h now

# Esperar a que el LED verde (actividad microSD) deje de parpadear y se apague por completo
# (aproximadamente 5 a 10 segundos). Solo entonces apagar el interruptor físico de batería.
```

---

## 8. Checklist Operativo de Competencia

Sigue este procedimiento en cada una de las rondas oficiales:

```mermaid
flowchart TD
    Z[0. Firmware MegaPi con DEPURAR 0 y batería cargada] --> A[1. Encender Robot en Boxes]
    A --> B[2. Calibrar Cámara UVC en :8082 si cambió la luz]
    B --> C[3. Verificar HSV de Artefactos en :8081]
    C --> D[4. Comprobar Voltaje: vcgencmd get_throttled == 0x0]
    D --> E[5. Hacer respaldo: cp config.json config.json.bak_ronda]
    E --> F[6. Bloquear Redes: sudo rfkill block all]
    F --> G[7. Confirmar bloqueo con rfkill list]
    G --> H[8. Entrega a Cuarentena e Inspección Técnica]
    H --> I[9. Ejecutar Ronda Oficial en Mesa WRO]
    I --> J[10. Regreso a Boxes y Desbloqueo: sudo rfkill unblock all]
```

| Paso | Acción | Comando / Verificación |
|---|---|---|
| **0** | Firmware de la MegaPi | `prueba_wwl5_opt.ino` con **`DEPURAR 0`** cargado y batería con carga alta (el PWM mínimo de arranque depende del voltaje). |
| **1** | Encender robot en boxes | Conectar batería y esperar 25 s a que suba el sistema. |
| **2** | Calibrar cámara UVC | Entrar a `http://robot-pi:8082`, usar **Auto-Medir y Fijar** y **Guardar**. |
| **3** | Calibrar colores HSV | Entrar a `http://robot-pi:8081` y verificar las máscaras sobre los objetos en la pista. |
| **4** | Comprobar salud eléctrica | `vcgencmd get_throttled` (debe arrojar `0x0`). Batería con carga > 80%. |
| **5** | Respaldar configuración | `cp config.json config.json.bak_rondaX` |
| **6** | Bloquear señales de radio | `sudo rfkill block wifi && sudo rfkill block bluetooth` |
| **7** | Inspección de radio | `rfkill list` (ambas deben decir `Soft blocked: yes`). |
| **8** | Cuarentena | Entregar robot a la mesa de jueces con interruptor listo. |
| **9** | Ronda Oficial | Iniciar con el pulsador de la MegaPi en la zona de salida. |
| **10** | Retorno a boxes | `sudo rfkill unblock wifi && sudo rfkill unblock bluetooth` para retomar análisis. |
