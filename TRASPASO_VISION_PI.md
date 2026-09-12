# Traspaso: arranque rápido de la Raspberry y ahorro de cámara

Estado al **12-09-2026, madrugada**. Escrito para quien retome el trabajo con acceso
directo a la Raspberry y pueda verificarlo sobre el robot.

**Todo lo de aquí toca SOLO la Raspberry Pi.** No se modificó nada de la MegaPi,
ni la configuración de red, ni `vision/config.json`, ni el sketch en uso
(`robot_WRO/ruta_visitantes/prueba_wwl7_opt`).

---

## 1. Los tres problemas que se atacaron

1. **La Pi 3B tardaba ~40 s** desde subir el suiche hasta que la MegaPi recibe
   `VISION OK`. Los jueces esperan de pie.
2. **Consumo en espera**: la Pi y la cámara trabajando a tope mientras nadie las
   necesita. En PAUSA decodificaba 20 fotos/s, comprimía dos JPEG por foto
   aunque nadie mirara la web, y escribía telemetría en la SD unas 20 veces/s.
3. **Confirmado por el equipo: cuando la Pi termina de arrancar, la MegaPi se
   reinicia y la corrida queda interrumpida.** La causa no es el software de
   visión: al abrir el puerto USB, Linux activa la línea DTR del CH340, que es
   exactamente la que resetea el ATmega2560 (es como el IDE de Arduino resetea
   la placa para cargar código). `evitar_reset_dtr: true` no lo evita en Linux
   porque el kernel sube DTR dentro del `open()`, antes de que pyserial pueda
   bajarla; ese truco solo funciona en Windows.

   **Regla operativa mientras el enlace sea por USB:** encender, esperar a que
   la Pi termine de arrancar, y SOLO entonces pulsar el botón de inicio. La
   solución de fondo (no aplicada, se descartó por falta de tiempo) es pasar el
   enlace al header de Raspberry Pi de la MegaPi, que lleva TX/RX/GND con
   conversión 5V-3.3V y **no tiene línea DTR**, así que no puede resetear nada.

---

## 2. Qué se cambió

### a) `vision/wro-vision.service` — que la visión no espere al resto del arranque

Antes tenía `After=multi-user.target`, así que arrancaba **al final de todo**:
esperaba a cloud-init (unos 8 s), a NetworkManager (7,3 s) y a ssh. Ahora usa
`DefaultDependencies=no` más `After=local-fs.target`, y `RestartSec=1`.

Medido en la Pi del robot con `systemd-analyze critical-chain wro-vision.service`:

| | Arranque del servicio |
|---|---|
| Antes | `@27.196s` |
| Después | `@7.369s` |

**No se desactivó ningún servicio del sistema.** La red, ssh, avahi y cloud-init
siguen subiendo igual que antes; la visión simplemente ya no los espera. Esto era
un requisito explícito del equipo: necesitan conectarse a la Pi después de que
arranque.

### b) `vision/vision_core.py` — la cámara duerme, y el arranque ya no se cae

- `Camara.dormir()` suelta el dispositivo pero deja vivo el hilo de captura; el
  sensor USB deja de transmitir y la Pi deja de decodificar MJPG. `despertar()`
  reusa el camino de reconexión que ya existía. Despertar tarda **1-2 s** y no
  se cuenta como avería en el contador de reconexiones.
- `Camara.abrir()` **ya no lanza excepción** si la cámara todavía no está
  enumerada: reintenta hasta 15 s y, si no aparece, deja al hilo de captura
  reintentando solo. Antes, al arrancar tan temprano, podía perder la carrera al
  USB, systemd reiniciaba el servicio y había que volver a importar OpenCV
  (varios segundos en una Pi 3B).

### c) `vision/vision_server.py` — quién decide, y tres arreglos

Regla de dormir: **modo PAUSA + nadie en la web + MegaPi callada 90 s**.
Despierta con **cualquier byte que mande la MegaPi** (al pulsar el botón de
inicio manda `P`) o con **cualquier visita a la web**.

- Los **90 s de silencio de la MegaPi** son deliberados: durante una ronda pasan
  decenas de segundos entre un comando de visión y el siguiente, y la cámara no
  debe dormirse en mitad de la carrera. No bajar ese valor sin pensarlo.
- Dormida: no comprime JPEG, no escribe telemetría en la SD y manda la trama `T`
  a 2 Hz, pero el bucle sigue girando a unos 20 Hz para contestar `P` en **menos
  de 50 ms** (`visionComprobar()` en la MegaPi espera 300 ms).
- **Interruptor de emergencia:** `http://192.168.137.50:8080/ahorro?on=0` deja la
  cámara encendida para siempre, hasta reiniciar el servicio. Para que sea
  permanente, añadir a `config.json` de la Pi: `"ahorro": {"habilitado": false}`.
  Los valores por defecto están en el código, así que **no hace falta tocar
  config.json**.

Arreglos incluidos:

1. **Serial (`EnlaceSerial.abrir`)**: si no se podía abrir el puerto quedaba un
   objeto `Serial` cerrado en vez de `None`; el siguiente `enviar()` lanzaba
   "port is not open", se marcaba una desconexión y el ciclo se repetía: **una
   línea de log por segundo, indefinidamente**, escribiendo en la SD. Ahora se
   avisa una sola vez y se reintenta en silencio. El mensaje que veía el equipo
   era correcto (el cable USB estaba desconectado de verdad); lo que estaba mal
   era la repetición. La reconexión funciona igual que antes.
2. **Video congelado (`/stream`)**: la cámara se dormía en mitad del video. Con
   red lenta, `wfile.write()` se bloquea varios segundos; mientras está
   bloqueado no se refresca la marca de "hay alguien mirando", se cumplían los
   segundos de inactividad y la cámara se dormía. El navegador quedaba con la
   última imagen congelada y, al desbloquearse la escritura, despertaba: saltos
   y desfase. **Este era el síntoma que reportó el equipo.** Ahora se **cuentan
   los `/stream` abiertos** (visible en `/estado` como `streams`) y mientras haya
   uno, la cámara no se duerme. El margen por peticiones sueltas subió de 5 a
   15 s.
3. **Instrumentación del arranque**: el log ahora dice, por separado, cuánto
   tardó OpenCV en cargar y cuánto la cámara en abrir. Sirve para la tarea que
   queda abierta (ver sección 5).

### d) Pruebas nuevas, en el repo

Corren **sin Raspberry, sin cámara y sin MegaPi** (simulan `cv2` y `pyserial`):

- `vision/test_camara_dormir.py` — dormir/despertar, y que `abrir()` no tumbe el
  proceso si la cámara no está.
- `vision/test_serial_reconexion.py` — MegaPi apagada, encendida, y apagada en
  marcha; comprueba que el aviso no se repita.
- `vision/test_ahorro_camara.py` — integración: levanta `main()` de verdad y mide
  el ritmo de tramas `T`. Incluye el caso del cliente de video lento que
  reproduce el bug del punto 2 (comprobado: **falla sin el arreglo y pasa con
  él**).

Las tres pasan. Tardan unos 40 s en total.

> **Para correrlas en la Pi hay que parar el servicio primero**
> (`sudo systemctl stop wro-vision`), porque `GestorExclusividadCamara` usa
> `/tmp/wro_camara.lock` y el test saldría con error de cámara en uso. Acordarse
> de volver a arrancarlo. También corren en cualquier laptop con Python 3, que es
> lo más cómodo.

---

## 3. Qué está cargado en la Pi y qué falta

El equipo cargó a mano `vision_core.py`, `vision_server.py` y
`wro-vision.service` a las 00:52. **Los tres arreglos de `vision_server.py`
(serial, video congelado e instrumentación) son posteriores**, así que lo más
probable es que a la Pi le falte ese archivo. Verificar con:

```bash
ssh pi@192.168.137.50 "md5sum /home/pi/WRO-RM-2026/vision/vision_server.py /home/pi/WRO-RM-2026/vision/vision_core.py /etc/systemd/system/wro-vision.service"
```

Debe dar exactamente:

| Archivo | md5 |
|---|---|
| `vision/vision_server.py` | `dc12cbec7f3c327c8fa3f3516673a7e4` |
| `vision/vision_core.py` | `2c997a6f1927af9d92425b72a16ea59c` |
| `/etc/systemd/system/wro-vision.service` | `9ce6ea53847bfaf6cd2f84e4019594cd` |

Para cargar lo que falte:

```bash
scp vision/vision_server.py pi@192.168.137.50:/home/pi/WRO-RM-2026/vision/
```

```bash
ssh pi@192.168.137.50 "sudo systemctl restart wro-vision"
```

Y si el md5 del `.service` no coincide:

```bash
ssh pi@192.168.137.50 "sudo cp /home/pi/WRO-RM-2026/vision/wro-vision.service /etc/systemd/system/ && sudo systemctl daemon-reload && sudo systemctl restart wro-vision"
```

> **NO usar `python herramientas/sync_pi.py push`.** Copia **toda** la carpeta
> `vision/`, incluido `config.json`, y sobrescribiría la calibración de cámara y
> de colores que esté guardada en la Pi. Copiar archivo por archivo con `scp`.

---

## 4. Verificaciones pendientes, en orden

**1. Reinicio y desglose del arranque.**

```bash
ssh pi@192.168.137.50 "sudo reboot"
```

Un minuto después:

```bash
ssh pi@192.168.137.50 "systemd-analyze critical-chain wro-vision.service | head -4; journalctl -b -u wro-vision -o short-monotonic --no-pager | head -8"
```

Esperado: el servicio sobre el segundo 7, y dos líneas nuevas que dicen
`arranque: OpenCV cargado en X s` y `camara abierta en Y s`. La referencia
anterior: el servicio arrancó en 13,65 s y la cámara quedó abierta en 31,07 s, es
decir **17,4 s de Python**. Ahí está lo que falta por ganar.

- Si **X es mayor que 10 s**: el muro es la microSD (leer OpenCV desde la tarjeta
  mientras el resto del sistema también la usa). Ver sección 5.
- Si **Y es mayor que 3 s**: entonces el problema es la cámara, o los ~12
  procesos `v4l2-ctl` que se lanzan uno por cada control.

**2. Que el ahorro funcione.** Con la MegaPi callada más de 90 s:

```bash
ssh pi@192.168.137.50 "curl -s localhost:8080/estado"
```

Esperado: `camara.estado` en `DORMIDA` y `streams` en 0. Al abrir
`http://192.168.137.50:8080` en un navegador debe pasar a `CONECTADA` en 1-2 s y
verse la imagen para ajustar el ángulo. Al cerrar el navegador vuelve a dormirse
a los 90 s.

**3. Que el video ya no se congele.** Abrir el video y dejarlo un par de minutos
con la red mala. En el log **no debe aparecer** `camara dormida` mientras la
página esté abierta.

Si el video sigue yendo mal después de cargar el arreglo, **ya no es el ahorro**,
y hay que distinguir entre red y USB:

- `curl -s localhost:8080/estado` y mirar `fps`. Si el bucle interno sigue en
  unos 20 fps, la visión está bien y el problema es el transporte: el MJPEG a
  20 fps no cabe en un enlace malo, el buffer del socket se llena y la latencia
  crece sin recuperarse.
- `Corrupt JPEG data: premature end of data segment` en el journal es la cámara
  USB perdiendo datos (típico de la Pi 3B con el USB cargado) y es independiente
  de la red. Aparecía también antes de estos cambios.

**4. Que el serial se recupere.** Desconectar el cable USB de la MegaPi: debe
aparecer **una sola** línea `esperando a la MegaPi en /dev/serial/...`.
Reconectarlo: debe aparecer `serial abierto en ...`.

---

## 5. Lo que queda abierto

**A. Los 17,4 s de Python.** Es lo único que queda entre los ~31 s actuales y
unos ~20 s. El candidato es **cloud-init**: son unos 8 s de Python compitiendo
por la misma microSD justo cuando la visión carga OpenCV. Desactivarlo es un
archivo:

```bash
ssh pi@192.168.137.50 "sudo touch /etc/cloud/cloud-init.disabled && sudo reboot"
```

Y la vuelta atrás:

```bash
ssh pi@192.168.137.50 "sudo rm /etc/cloud/cloud-init.disabled && sudo reboot"
```

**Antes de hacerlo**, comprobar que la red no dependa de cloud-init:

```bash
ssh pi@192.168.137.50 "ls -la /boot/firmware/network-config 2>&1; ls -la /etc/NetworkManager/system-connections/; nmcli -t -f NAME,TYPE,DEVICE con show"
```

Si **no** existe `/boot/firmware/network-config` y los perfiles están en
`system-connections` como archivos normales, NetworkManager los lee solo y
desactivar cloud-init no toca la red. Riesgo estimado bajo, **pero si la red se
cayera haría falta monitor y teclado para revertir**. El equipo prefirió no
tocarlo la noche antes de competir; queda a criterio de quien retome esto, con
tiempo por delante y no a última hora.

**B. La estrategia que vuelve irrelevante el arranque.** Con la cámara durmiendo,
la Pi puede quedarse encendida desde boxes: con 5000 mAh, una hora de espera
cuesta del orden del 7-10 % del paquete. Encender el robot en boxes, verificar el
encuadre en la web, dejarlo encendido, y en la mesa solo pulsar inicio. Además el
reinicio de la MegaPi por DTR ocurre allá en boxes y no vuelve a pasar. Esto vale
más que cualquier segundo que se le saque al arranque.

**C. El enlace por UART** (ver sección 1, punto 3): elimina de raíz el reinicio de
la MegaPi, libera el USB para cargar código y permitiría subir a 250000 baudios,
que en el ATmega a 16 MHz tiene 0 % de error. Requiere soldar el header 2x10 de
la MegaPi. Notas para cuando haya tiempo:

- Solo **GND, TXD y RXD. No conectar los 5 V**: el header está pensado para
  alimentar la Pi desde la MegaPi y eso rompe la fuente independiente de 2,5 A.
- Las posiciones del header corresponden a los primeros 20 pines de la Pi:
  **6 = GND, 8 = TXD (GPIO14), 10 = RXD (GPIO15)**.
- Medir con multímetro el pad de TXD en reposo, con la MegaPi encendida y la Pi
  desconectada: debe dar unos 3,3 V. Si diera 5 V, hace falta divisor antes de
  tocar el GPIO de la Pi.
- Makeblock no publica a qué UART del ATmega va ese header. Su firmware oficial
  escucha en `Serial`, `Serial2` **y** `Serial3`, así que lo seguro es escuchar y
  responder por Serial2 y Serial3 a la vez, o identificarlo con un sketch que
  imprima una marca distinta en cada uno.
- En la Pi: `enable_uart=1`, `dtoverlay=disable-bt`, desactivar la consola de
  login por serial, y `config.json` con `"puerto": "/dev/serial0"`.

---

## 6. Cómo revertir

Kill switch inmediato, sin tocar archivos: `http://192.168.137.50:8080/ahorro?on=0`.

Volver el servidor a la versión anterior a estos cambios:

```bash
git checkout df4be5c^ -- vision/vision_server.py vision/vision_core.py vision/wro-vision.service
```

```bash
scp vision/vision_server.py vision/vision_core.py vision/wro-vision.service pi@192.168.137.50:/home/pi/WRO-RM-2026/vision/
```

```bash
ssh pi@192.168.137.50 "sudo cp /home/pi/WRO-RM-2026/vision/wro-vision.service /etc/systemd/system/ && sudo systemctl daemon-reload && sudo systemctl restart wro-vision"
```

El commit `df4be5c` es el que introdujo la gestión de energía; su padre tiene el
servicio con `After=multi-user.target` y `RestartSec=3`.

---

## 7. Datos del entorno

| Cosa | Valor |
|---|---|
| Pi | `clc-wro-rm`, Pi 3B, Debian 13 arm64, usuario `pi`, `sudo` sin contraseña |
| Red | `192.168.137.50` por Ethernet directo a la laptop; `192.168.0.166` por wifi |
| Repo en la Pi | `/home/pi/WRO-RM-2026` |
| Servicio | `wro-vision.service` (unidad en `/etc/systemd/system/`, fuente en `vision/`) |
| Web | puerto 8080: `/`, `/stream`, `/raw`, `/estado`, `/telemetria`, `/modo`, `/ahorro` |
| MegaPi | `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0` a 115200 |
| Cámara | `/dev/v4l/by-id/usb-USB_Camera_*`, 640x360 MJPG, unos 20 fps reales |
| Sketch en uso | `robot_WRO/ruta_visitantes/prueba_wwl7_opt` (**no modificado**) |

Log en vivo: `ssh pi@192.168.137.50 "journalctl -u wro-vision -f"`.
