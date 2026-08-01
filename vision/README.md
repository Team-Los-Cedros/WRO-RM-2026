# Visión con Raspberry Pi 3B para el robot WRO RoboMission 2026

La Raspberry Pi actúa como **sensor inteligente**: mira por la webcam USB, encuentra
el objeto del color pedido y le manda a la MegaPi el error de posición. La MegaPi
sigue siendo el cerebro que decide la rutina — así el robot nunca depende de que
la Pi arranque a tiempo, y si la Pi falla la rutina por encoders sigue funcionando.

```
 webcam USB ──► Raspberry Pi 3B ──serial 115200──► MegaPi ──► motores / garra
                (OpenCV, HSV)      "T 1 ROJO -34 ..."        (encoders + giroscopio)
```

---

## 0. Estado de esta Raspberry (verificado el 2026-08-01)

| | |
|---|---|
| Modelo | Raspberry Pi 3 Model B Rev 1.2, 905 MB RAM |
| Sistema | Debian 13 (trixie), arm64, hostname `clc-wro-rm` |
| Python | 3.13.5 · OpenCV 4.10.0 · numpy 2.2.4 · pyserial 3.5 |
| Cámara | `USB Camera` 32e6:9221 en `/dev/video0`, MJPG hasta 1920×1080 |
| Ya instalado | `python3-opencv`, `v4l-utils`, `fswebcam` (con esta tomaste la foto) |

**No hace falta instalar nada.** `install.sh` está para reinstalar en otra tarjeta.
`fswebcam` no lo usa este sistema; OpenCV habla con la cámara por V4L2 directamente.

### ⚠ Problema de alimentación pendiente

```
throttled=0x50005     under-voltage AHORA + throttling AHORA
frequency(arm)=600 MHz    (la Pi 3B debería ir a 1200 MHz)
55 eventos de undervoltage desde el arranque
```

La Pi está corriendo **a la mitad de su velocidad** por falta de corriente, y el
USB da errores (`Failed to resubmit video URB`, frames MJPEG corruptos). Necesita
una fuente de **5 V / 2.5 A real** (o una power bank que entregue 2.5 A) y un cable
micro-USB corto y grueso — los cables finos son la causa más común. Compruébalo con:

```bash
vcgencmd get_throttled
```

`0x0` es lo correcto. Cualquier otra cosa significa que el rendimiento no es real.
Aun así, el sistema **funciona a 20 fps incluso throttled**, así que no bloquea el
desarrollo; pero no lleves el robot a competir en este estado.

## 1. Instalación en otra tarjeta

```bash
bash /home/pi/vision/install.sh
```

Instala `python3-opencv`, `python3-serial` y `v4l-utils` desde los repos de Debian.
**No uses `pip install opencv-python`** en una Pi 3B: intenta compilar desde fuente
y tarda horas.

## 2. Calibrar

Todo se calibra desde el navegador de tu PC, sin monitor en la Pi.

```bash
python3 /home/pi/vision/calibrar_web.py
```

Abre `http://192.168.0.174:8081`. Verás la imagen real y la máscara del color, con
sliders HSV.

**Orden de calibración:**

1. **Color** — mueve los sliders hasta que la máscara marque *sólo* el objeto.
   Comprueba en varios puntos de la pista, no solo en uno. Pulsa *Guardar*.
2. **`cx_garra`** — pon un objeto justo delante de la garra y edita `cx_garra`
   en `config.json` hasta que la línea amarilla pase por su centro. Es el valor
   que define "centrado" para el robot.
3. **`tabla_distancia`** — coloca el objeto a distancias medidas con regla
   (60, 100, 150, 220, 300, 420 mm desde la garra) y anota el `ey` que muestra la
   página. Cada par `[ey, mm]` va a la tabla.

> **Lo más importante:** el `config.json` fija exposición y balance de blancos con
> `v4l2-ctl`. Si los dejas en automático, el HSV del objeto cambia solo con moverse
> por la pista y la calibración deja de servir. Es la causa número uno de que una
> detección por color "funcione en casa y falle en la competencia". Calibra con la
> iluminación de la sede el mismo día.

### Antes de calibrar: comprueba la zona ciega

La cámara va en lo alto del robot con una inclinación **leve** hacia abajo. Eso da
buen alcance al frente, pero crea una **zona ciega cercana**: llega un punto en que
el objeto está tan cerca que sale por debajo del encuadre.

Mide dónde empieza esa zona: acerca un objeto poco a poco mirando `calibrar_web.py`
y anota a qué distancia de la garra deja de verse. Ese número manda:

- **si es menor de ~12 cm**, perfecto: el robot llega guiado casi hasta el agarre;
- **si es mayor de ~20 cm**, el tramo final a ciegas es demasiado largo y el agarre
  fallará seguido. Inclina más la cámara hacia abajo, aunque pierdas alcance —
  para buscar objetos lejanos basta con girar el robot, pero el agarre no perdona.

Ese valor va a `VIS_GRADOS_CIEGOS` en el sketch: los grados de encoder que el robot
avanza a ciegas cuando pierde de vista el objeto por cercanía.

### La misión: 4 objetos de 5 colores

Salen 4 objetos de color y posición aleatorias, de entre rojo, verde, negro, azul y
amarillo. Hay que dejarlos en la zona de descarga en ese orden fijo de izquierda a
derecha, con el hueco del color ausente vacío.

Como cada color tiene hueco fijo, **no hay que compactar nada**: basta con saber qué
colores salieron (comando `X`) y llevar cada uno a su posición. Eso está resuelto en
el sketch con `visionEscanearColores()`, `colorAusente()` y `depositarEnHueco()`;
lo que falta calibrar en pista es `POS_DESCARGA_GRADOS[5]`.

## 3. Probar

```bash
python3 /home/pi/vision/vision_server.py --sin-serial --web
```

Imprime las tramas en pantalla y publica el vídeo con overlay en
`http://192.168.0.174:8080`. Con la MegaPi conectada:

```bash
python3 /home/pi/vision/vision_server.py --verbose
```

## 4. Autoarranque

```bash
sudo cp /home/pi/vision/wro-vision.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now wro-vision
journalctl -u wro-vision -f
```

---

## Protocolo serial

**Pi → MegaPi**, una línea por frame a 20 Hz:

```
T <found> <color> <ex> <ey> <area> <dist> <fps>
T 1 ROJO -34 187 950 145 24
T 0 ROJO 0 0 0 -1 24
```

| campo   | significado |
|---------|-------------|
| `found` | 1 si ve el objeto |
| `ex`    | error horizontal en px. **Negativo = objeto a la izquierda** |
| `ey`    | fila del borde inferior del objeto (mayor = más cerca) |
| `area`  | área del contorno en px |
| `dist`  | mm estimados por la tabla de calibración (−1 si no hay dato) |

**MegaPi → Pi:**

| comando | efecto |
|---------|--------|
| `C ROJO` | cambia el color buscado |
| `X` | escanea los 5 colores en el frame actual y responde `X ROJO 1 -35 278 VERDE 0 0 -1 ...` |
| `S 0` / `S 1` | apaga/enciende el envío continuo |
| `P` | ping → la Pi responde `K <version> <colores>` |
| `# texto` | log de la MegaPi; la Pi lo imprime y no lo interpreta |

`X` es el que resuelve tu regla de juego: salen 4 de los 5 colores y hay que saber
cuál falta para dejar su hueco vacío. Cuesta ~25 ms (5 máscaras), así que se usa
una vez al principio de la ronda, no en el lazo de control.

El prefijo `#` permite depurar el Arduino por el mismo cable sin romper el
protocolo. En el sketch usa `logPi("mensaje")` en vez de `Serial.println()`.

## Probar el protocolo sin la MegaPi

```bash
python3 /home/pi/vision/test_protocolo.py
```

Crea un puerto serie virtual, hace de MegaPi y comprueba las tramas y los cinco
comandos. Útil antes de tocar el robot y después de cambiar la configuración.

## Rendimiento medido en esta Pi 3B

Con el CPU throttled a 600 MHz por el problema de alimentación:

| | |
|---|---|
| Bucle completo (captura + detección + envío) | **20 fps** |
| Detección de un color | 4 ms |
| Escaneo de los 5 colores (comando `X`) | 25 ms |
| Tramas al Arduino | 20/s |

La cámara entrega 20 fps reales; el procesamiento no es el cuello de botella ni de
lejos. Dos cosas medidas que conviene no deshacer:

- **`CAP_PROP_BUFFERSIZE = 1` parte el framerate a la mitad** (20 → 10 fps). Con un
  solo buffer encolado, el driver descarta el frame N+1 mientras se procesa el N.
  Se dejan 3 buffers y la latencia se controla con el hilo lector de `Camara`, que
  siempre entrega el frame más reciente.
- **`hz_envio` demasiado cerca del framerate hace aliasing**: con la cámara a 20.8 fps
  (48 ms) y un periodo de envío de 50 ms exactos, un frame de cada dos llegaba
  "demasiado pronto" y se perdía, quedando en 10 tramas/s. Por eso el periodo lleva
  un margen del 10 %.

Lo que lo hace barato: sólo se genera la máscara del color activo (no de los cinco),
la conversión BGR→HSV se hace únicamente sobre la ROI inferior, y
`cv2.setNumThreads(2)` porque más hilos no ayudan en operaciones tan pequeñas y
compiten con la captura.

## El color negro es el caso difícil

El negro comparte HSV con las sombras y con las líneas negras de la pista. Por eso
`config.json` tiene una sección `reglas_color` que le exige más área, más altura de
caja y una relación de aspecto más cuadrada que a los demás. Aun así:

- calíbralo **el último**, y verifícalo con el robot mirando a una línea negra del
  suelo para confirmar que no la confunde con un objeto;
- si sigue dando falsos positivos, sube `alto_min_px` — un objeto tiene altura en la
  imagen, una línea en el suelo no.

## Archivos

| archivo | qué hace |
|---------|----------|
| `vision_core.py` | cámara (hilo lector), detección HSV, distancia, overlay |
| `vision_server.py` | bucle principal + enlace serial con la MegaPi |
| `calibrar_web.py` | calibrador HSV por navegador |
| `test_protocolo.py` | prueba del protocolo con un puerto serie virtual |
| `config.json` | toda la configuración: cámara, colores, geometría, serial |
| `install.sh` | instalación de dependencias |
| `wro-vision.service` | unidad systemd para el autoarranque |

El lado Arduino está en [`robot_WRO/prueba_vision/prueba_vision.ino`](../robot_WRO/prueba_vision/prueba_vision.ino).
