# Visión y ciclo de cuatro artefactos - WRO RoboMission Junior 2026

La Raspberry Pi reconoce y localiza artefactos; la MegaPi conserva el control de
motores, encoders, giroscopio, garra, secuencia y parada física.

```text
webcam USB -> Raspberry Pi 3B -> USB serial 115200 -> MegaPi -> robot
              OpenCV / HSV       T 1 VERDE ...        navegación y garra
```

El sketch coordinado está en
[`robot_WRO/prueba_vision/prueba_vision.ino`](../robot_WRO/prueba_vision/prueba_vision.ino).

## Qué cambió

- `C AUTO` reconoce el color que está alineado con la garra; después la MegaPi
  fija `C <COLOR>` para que el seguimiento no cambie durante el acercamiento.
- La identidad se vota durante varios frames. Área y confianza pesan en el voto
  para que piezas amarillas pequeñas del modelo verde no cambien su identidad.
- Se ignoran dos rectángulos inferiores que corresponden a los dedos azules de
  la garra. Antes eran detectados como un supuesto artefacto azul.
- Se consideran todos los contornos plausibles y se elige por centro, forma y
  área; ya no se acepta ciegamente el contorno más grande. Esto evita que una
  zona verde impresa en la pista gane al artefacto verde.
- El robot visita cuatro posiciones físicas conocidas (`slots`), guarda
  `slot -> color`, recoge, entrega y regresa por la ruta inversa.
- Los desplazamientos laterales del museo se hacen desde una línea de espera
  alejada. Solo entonces se avanza perpendicularmente hacia el exhibidor; así
  la garra no barre artefactos ya colocados.
- Los ángulos vigentes de la garra y la pala se copiaron de la rutina probada
  `prueba_centrales.ino`: cierre `108/58`, bajar `105`, transporte `120`,
  recoger `111`, posicionar `50`, depositar `40` y barrer `117` grados.
- Un límite interno de 116 s evita iniciar un ciclo que no tenga margen dentro
  de los 120 s reglamentarios.

## Algoritmo de una ronda

El robot inicia en el área blanca, centrado entre los cuatro cuadros de
artefactos y mirando hacia ellos. Los slots se numeran de izquierda a derecha
`0, 1, 2, 3` desde su perspectiva.

1. Visita los slots en orden `1, 2, 0, 3` para resolver primero los más cercanos.
2. Hace un desplazamiento en L, queda mirando perpendicularmente al slot y pide
   `C AUTO`.
3. Vota el color durante 1.8 s y guarda la posición conocida del slot.
4. Fija ese color, centra por `ex`, avanza con corrección proporcional y usa
   `ey/dist` para detenerse en la pose de precaptura.
5. Ejecuta la secuencia mecánica que ya funcionaba para sacar y reacomodar el
   artefacto en la garra.
6. Deshace la L, toma la ruta probada al museo y se detiene antes de los
   exhibidores.
7. Se desplaza lateralmente según el destino fijo
   `ROJO, VERDE, NEGRO, AZUL, AMARILLO`, avanza, deposita, retrocede y deshace
   el desplazamiento lateral.
8. Regresa exactamente por la transformación inversa y continúa con otro slot.

No se necesita conocer de antemano cuál de los cinco colores quedó fuera de la
ronda. Cada objeto revela su destino justo antes de recogerlo. El comando `X`
se mantiene como diagnóstico de lo visible en un frame, pero no se usa como
mapa global porque desde el inicio la cámara solo alcanza a ver dos objetos.

## Estado de la Raspberry verificado el 31-08-2026

| Componente | Estado |
|---|---|
| Host | `clc-wro-rm`, Raspberry Pi 3B, Debian 13 arm64 |
| Dirección de desarrollo | `192.168.0.166` |
| Cámara | USB Camera en `/dev/video0`, 20 fps reales a 320x240 |
| Servicio | El archivo existe en el proyecto, pero la unidad no estaba instalada |
| Enlace MegaPi | `/dev/ttyUSB0`; estaba desconectada durante esta verificación |

La calibración vigente extraída al repositorio fija exposición `750`,
temperatura `4500`, ganancia `25`, brillo `12`, contraste `33`, saturación `68`,
gamma `100` y nitidez `2`. Esos valores se conservaron en `config.json`; son una
base, no sustituyen la calibración con la luz del evento.

## Probar sin mover el robot

En la Raspberry, con la MegaPi desconectada:

```bash
cd /home/pi/WRO-RM-2026/vision
python3 vision_server.py --sin-serial --web --color AUTO
```

El overlay queda en `http://192.168.0.166:8080`. La consola debe emitir:

```text
T 1 AMARILLO -12 105 2632 310 20 86
T 0 NINGUNO 0 0 0 -1 20 0
```

Pruebas sin cámara ni MegaPi:

```bash
python3 test_detector.py
python3 -m py_compile vision_core.py vision_server.py test_protocolo.py
```

Prueba del protocolo usando la cámara y un puerto serie virtual:

```bash
python3 test_protocolo.py
```

## Protocolo serial v2

Pi a MegaPi, una trama por frame:

```text
T <found> <color> <ex> <ey> <area> <dist> <fps> <confianza>
```

| Campo | Significado |
|---|---|
| `found` | `1` si hay objetivo válido |
| `color` | Color reconocido o `NINGUNO` |
| `ex` | Error horizontal; negativo izquierda, positivo derecha |
| `ey` | Fila inferior del contorno; aumenta al acercarse |
| `area` | Área del contorno en píxeles |
| `dist` | Distancia estimada por la tabla, en mm |
| `fps` | Frecuencia de procesamiento |
| `confianza` | Calidad geométrica de 0 a 100 |

MegaPi a Pi:

| Comando | Efecto |
|---|---|
| `C AUTO` | Reconoce el artefacto más alineado |
| `C ROJO`, etc. | Sigue exclusivamente ese color |
| `X` | Diagnóstico de los cinco colores en el frame actual |
| `S 0` / `S 1` | Detiene o activa el stream serial |
| `P` | Responde versión y modos disponibles |
| `# texto` | Log de MegaPi que la Pi imprime sin interpretarlo |

## Calibración obligatoria en pista

No se debe probar directamente con cuatro objetos. En el sketch, cambia
temporalmente `NUM_ARTEFACTOS_OBJETIVO` de `4` a `1` y calibra en este orden.

### 1. Cámara, imagen y color

Primero fija la respuesta de la cámara con la iluminación real del recinto:

```bash
python3 calibrar_camara.py
```

Abre `http://192.168.0.166:8082`. Puedes ajustar exposición, ganancia,
balance de blancos, brillo, contraste, saturación, gamma y nitidez en tiempo
real. La opción **Auto-Medir y Fijar** mide la escena y deja exposición y
balance en manual para que el HSV no cambie mientras el robot se mueve. Guarda
el resultado en `config.json` antes de calibrar los colores.

Después ejecuta el calibrador HSV y de geometría:

Ejecuta:

```bash
python3 calibrar_web.py
```

Abre `http://192.168.0.166:8081`. Ajusta HSV con cada artefacto en los cuatro
slots y con la iluminación real. Exposición y balance de blancos deben quedar
manuales. Verifica especialmente:

- azul: los dedos de la garra no deben producir contorno;
- verde: la franja verde inferior de la pista no debe ser el objetivo;
- negro: las líneas negras laterales no deben ganar al objeto centrado.

Si cambia la cámara o el soporte, corrige `zonas_ignoradas` en `config.json`.
Sus coordenadas son fracciones `[x0, y0, x1, y1]` de la imagen.

### 2. Centro y distancia

- Ajusta `geometria.cx_garra` hasta que la línea amarilla pase por el centro
  físico entre los dedos.
- Coloca un artefacto a 60, 100, 150, 220, 300 y 420 mm; anota `ey` y reemplaza
  `tabla_distancia`.
- Ajusta `VIS_Y_PRECAPTURA` y `VIS_GRADOS_CIEGOS` solo después. La cámara debe
  perder el objeto cuando ya está centrado entre las guías, no antes.

### 3. L de los slots

Con la garra levantada y sin recoger, mide primero la separación entre centros
de dos cuadros negros. Convierte esa distancia a encoder y actualiza
`PASO_SLOT_GRADOS` (valor inicial `220`). Confirma los cuatro offsets:

```text
slot 0 = -1.5 pasos
slot 1 = -0.5 pasos
slot 2 = +0.5 pasos
slot 3 = +1.5 pasos
```

Si izquierda y derecha quedaron invertidas, invierte los signos del arreglo
`OFFSET_SLOT_GRADOS`; no intercambies nombres de motores.

### 4. Museo

La ruta verde probada se dividió en `500 + 180 = 680` grados:

- `RUTA_CRUCE_GRADOS = 128`
- `RUTA_HASTA_STAGING_GRADOS = 500`
- `APROX_MUSEO_GRADOS = 180`
- `PASO_MUSEO_GRADOS = 205` (medir)

La referencia es el exhibidor verde. Prueba primero verde; luego rojo; luego
negro, azul y amarillo. El lateral se ejecuta 180 grados de encoder antes de la
fila, por lo que ninguna rueda o garra debería tocar un objeto ya puntuado.

### 5. Ciclo y tiempo

Sube `NUM_ARTEFACTOS_OBJETIVO` a `2`, comprueba el regreso inverso y solo
después usa `4`. Mide el tiempo total. Si no caben cuatro ciclos de forma
repetible, es preferible completar tres artefactos dentro de 120 s antes que
iniciar un cuarto y alterar los ya puntuados.

## Instalación y autoarranque

En otra tarjeta:

```bash
bash install.sh
```

Para instalar el servicio:

```bash
sudo cp wro-vision.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now wro-vision
journalctl -u wro-vision -f
```

## Cumplimiento durante competencia

Las reglas generales 2026 permiten varios microcontroladores, pero prohíben la
comunicación inalámbrica entre componentes durante la ronda. El enlace interno
Raspberry-MegaPi debe ser USB/serial. SSH y la página web son solo para práctica.
Antes de inspección/ronda, apaga Wi-Fi y Bluetooth y verifica que el servidor
funcione sin red:

```bash
sudo rfkill block wifi
sudo rfkill block bluetooth
```

Para volver al modo de desarrollo:

```bash
sudo rfkill unblock wifi
sudo rfkill unblock bluetooth
```

La Raspberry requiere fuente independiente de 5 V / 2.5 A. No debe alimentarse
desde la MegaPi; ambas solo comparten el enlace USB o la masa si se usa UART.

## Archivos

| Archivo | Función |
|---|---|
| `vision_core.py` | Cámara, máscaras, contornos, AUTO, distancia y overlay |
| `vision_server.py` | Bucle principal, protocolo v2, serial y MJPEG |
| `config.json` | Cámara, HSV, zonas ignoradas, geometría y serial |
| `calibrar_camara.py` | Exposición, balance de blancos y controles UVC desde navegador |
| `calibrar_web.py` | Calibración HSV y geometría desde navegador |
| `test_detector.py` | Pruebas sintéticas del selector y las máscaras |
| `test_protocolo.py` | Prueba integral con puerto serie virtual |
| `wro-vision.service` | Unidad systemd opcional |
