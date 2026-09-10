# Diagnóstico MegaPi — congelamientos y precisión de giros

Análisis de `robot_WRO/ruta_visitantes/prueba_wwl5_respaldo/prueba_wwl5_respaldo.ino`
y de las librerías `makeblock` (`MeGyro`, `MeEncoderOnBoard`, `MeMegaPi`) y `Wire`
del core Arduino AVR 1.8.8.

Resultado: `robot_WRO/ruta_visitantes/prueba_wwl5_opt/` (sketch optimizado) y
`robot_WRO/ruta_visitantes/calibrar_gyro/` (herramienta de calibración).

---

## 1. Por qué se congelaba la placa

### Causa principal: el bus I2C se puede bloquear para siempre

En todo el programa original hay **un solo punto que puede bloquear indefinidamente**:
las esperas activas dentro de `Wire`. Todos los demás bucles (`_delay`, `girarGyro`,
`centrarEnLinea`, `parabrisas`…) tienen su `timeout` y salen solos.

La cadena es:

```
_loop()  ->  giroscopio.update()  ->  MeGyro::readData()  ->  Wire.requestFrom()
                                                           ->  twi_readFrom()
                                                           ->  while (TWI_MRX == twi_state) { }
```

Ese `while` en `utility/twi.c` **no tiene salida** si `twi_timeout_us == 0`, que es el
valor por defecto. Si el MPU6050 se queda sujetando SDA a nivel bajo en mitad de una
transferencia — cosa que provocan los picos de corriente de los motores acoplándose al
cable RJ25 — la placa se queda ahí. Con las interrupciones todavía activas: los motores
siguen con el último PWM y el robot "se queda paralizado", exactamente el síntoma descrito.

El `Wire.setWireTimeout(3000, true)` que ya añadiste ataca justo esto y está bien puesto:
`twi_init()` no borra el timeout, así que sobrevive al `Wire.begin()` que hace
`giroscopio.begin()` después. Dos matices:

- 3000 µs es poco margen: una lectura de 14 bytes a 100 kHz tarda **~1,55 ms**, o sea
  que estabas a menos del doble del límite. Un timeout espurio hace que `MeGyro::update()`
  devuelva sin actualizar nada y el ángulo se queda congelado sin avisar.
- Reiniciar el maestro TWI **no despega al esclavo**. Si el MPU6050 es el que sujeta SDA,
  hay que mandarle pulsos de reloj para que suelte la línea. Eso no lo hace nadie.

### Lo que se descartó

- **RAM**: el sketch original ocupa 1 594 bytes de los 8 192 (19,5 %). Quedan ~6,5 KB
  de pila. No es desbordamiento de memoria.
- **Tormenta de interrupciones de encoder**: con `setPulse(8)` y `setRatio(46)` son
  373 pulsos por vuelta del eje; a ~185 rpm son ~1 150 int/s por motor, ~2 300 en total.
  Con `digitalRead()` dentro de la ISR son unos 10 µs cada una → 2,3 % de CPU. Molesta,
  pero no cuelga.
- **Conflicto de timers Servo/motores**: en Mega la librería Servo coge Timer5 primero y
  mete 12 servos por timer. Con 3 servos no toca Timer1/2/4. No hay conflicto.

### Por qué los `delay()` "arreglaban" el cuelgue

No lo arreglaban: bajaban la probabilidad. `_loop()` sin frenos llamaba a
`giroscopio.update()` miles de veces por segundo, dejando el bus I2C al 100 % de ocupación
y multiplicando las ocasiones de que un glitch pillara una transferencia a medias.
Espaciar las llamadas a 25 ms reducía el tráfico ~50 veces.

Hay un efecto secundario que además rompía el ángulo: `MeGyro::update()` calcula
`dt = (millis() - last_time) / 1000`. Si la llamas más rápido que 1 kHz, `dt` sale **0**
muchas veces y esas muestras **no se integran**: el giroscopio pierde grados sin decírtelo.
Así que sin el `delay` el ángulo era erróneo, y con el `delay` era grueso.

---

## 2. Por qué el ángulo cambia en cada intento (y empeora al ir rápido)

Cuatro causas, todas dependientes de la velocidad. Por eso al ir rápido se descontrolaba.

### 2.1 El giroscopio se leía cada 25 ms

`MeGyro::update()` es carísima. Por llamada:

| Parte | Coste |
|---|---|
| Lectura I2C de 14 bytes a 100 kHz (espera activa) | ~1,55 ms |
| 2 × `atan2`, 2 × `sqrt`, `floor` y ~45 operaciones en coma flotante | ~0,43 ms |
| **Total** | **~2 ms** |

(Verificado desensamblando `MeGyro.o`: 2 `atan2`, 2 `sqrt`, 1 `floor`, 15 `__mulsf3`,
8 `__divsf3`, 16 `__addsf3`/`__subsf3`, 6 conversiones int→float.)

Todo ese trabajo era para calcular X e Y con el acelerómetro, que **no usas para nada**.

Con muestreo cada 25 ms, girando a 200 °/s el robot recorre **5° entre lectura y lectura**.
Ese es el suelo de precisión del método, y crece linealmente con la velocidad.

### 2.2 El frenado no se medía — y es lentísimo

Esta es la causa dominante. Al terminar el giro:

```c
Encoder_1.runSpeed(0);
Encoder_2.runSpeed(0);
_delay(0.3);
```

`runSpeed(0)` no para el motor: solo cambia una consigna. El PWM real lo decide
`MeEncoderOnBoard::speedWithoutPos()`, y en la librería:

- `encoderMove()` solo se ejecuta **si han pasado más de 40 ms** desde la última vez.
- Cada ejecución cambia el PWM como mucho **25 cuentas** (`constrain(..., -25, 25)`).

O sea que bajar de PWM 200 a 0 puede tardar **8 ciclos × 40 ms = 320 ms**, y durante todo
ese tiempo el robot sigue girando. Esa pasada no se mide nunca, porque el bucle ya salió.
Depende de la velocidad, de la batería, del rozamiento y del punto exacto del ciclo de
40 ms en el que caíste. De ahí que cada intento saliera distinto.

Por eso los números del programa eran 85 para girar 90 y 175 para girar 180: llevaban
descontada a ojo esa pasada, y el descuento correcto cambia con la velocidad (por eso
había 84, 85, 89.5, 87 y 99 para giros parecidos).

### 2.3 Los dos motores iban a frecuencias de PWM distintas

Este es un fallo del código que genera mBlock, y no es evidente:

| Slot | Pin PWM | Timer | Configurado en `setup()` | Frecuencia real |
|---|---|---|---|---|
| SLOT1 (izq) | 12 | Timer1 | sí | **7 812 Hz** |
| SLOT2 (der) | **8** | **Timer4** | **no** | **490 Hz** |
| SLOT3 | 9 | Timer2 | sí (sin usar) | 3 906 Hz |

`setup()` tocaba `TCCR1A/B` y `TCCR2A/B`, pero el motor derecho sale por el pin 8, que en
el ATmega2560 es OC4C → **Timer4**, que se quedaba con el valor por defecto del core
(preescala 64, PWM phase-correct de 8 bits = 490 Hz).

Dos motores idénticos con el mismo valor de PWM pero a 490 Hz y a 7 812 Hz **no dan el
mismo par**, sobre todo a PWM bajo. Es casi seguro el origen del `velocidad*1.02` que
tenías puesto para compensar el motor derecho.

### 2.4 `avanzar()` y `retroceder()` no esperaban a terminar

```c
void moverRobot(long gradosIzq, long gradosDer, float velocidad, float tiempoEspera) {
  Encoder_1.move(gradosIzq, abs(velocidad));
  Encoder_2.move(-gradosDer, abs(velocidad*1.02));
  _delay(tiempoEspera);            // espera fija, pase lo que pase
}
```

`move()` lanza el PID de posición y `_delay()` espera un tiempo **fijo**. Si la batería
está más baja el movimiento tarda más, el `_delay` se acaba antes y **la siguiente orden
arranca con el robot todavía en marcha**. Si está cargada, se pierden décimas esperando
de más. Es otra fuente directa de "cada intento hace algo distinto".

---

## 3. Otros fallos encontrados

| Dónde | Problema |
|---|---|
| `MeEncoderOnBoard::setRatio(int16_t)` | El parámetro es entero: `setRatio(46.67)` guarda **46**. Todas las distancias tienen un 1,4 % de error de escala. No se puede arreglar sin tocar la librería; el sketch nuevo usa 46 explícitamente para que tus números sigan valiendo lo mismo. |
| `getPulsePos()` | Devuelve un `long` de 4 bytes que modifica una ISR, **sin desactivar interrupciones**. Se puede leer un valor a medio actualizar (salto de 256 o 65 536 cuentas). Raro pero real. |
| `seguirLineaTiempo/Grados` | `error = analogRead(A4) - digitalRead(2)`: resta un valor de 0–1023 menos un 0 o un 1. No es un seguidor de línea. (Estas funciones no se llamaban en la rutina.) |
| `_delay()` | `millis() + espera` desborda a los 49,7 días. Irrelevante en competencia, pero corregido. |
| `#include <SoftwareSerial.h>` | No se usa. Solo ocupa flash y reserva los vectores PCINT. |
| `centrarEnLinea(velocidad, 40.0)` | Ya lo habías visto: 40 segundos de timeout. Corregido a 4. |
| `imprimirSensoresLinea()` | Dice "TCRT Der (A3)" pero el pin es el 2. |

---

## 4. Qué hace la versión nueva

### 4.1 Giroscopio propio, 14 veces más rápido

Se abandona `MeGyro` y se lee el MPU6050 directamente:

- Solo el registro Z (`0x47`), **2 bytes** en vez de 14.
- Bus a **400 kHz** en vez de 100 kHz.
- Integración con `micros()` en vez de `millis()` → no hay muestras con `dt = 0`.
- Integración **trapezoidal** en vez de rectangular (menos error en las aceleraciones).
- Filtro digital del MPU a 98 Hz (`DLPF = 2`) para que la vibración de los motores no
  meta ruido, y reloj del chip desde el PLL del giro X, que deriva menos.

**~0,14 ms por lectura** en vez de ~2 ms. Eso permite muestrear **cada 2 ms** en vez de
cada 25 ms: la cuantización del ángulo baja de 5° a 0,4° a 200 °/s.

### 4.2 El rumbo ya no da la vuelta en ±180

`MeGyro::getAngleZ()` recorta a ±180 y por eso hacía falta `deltaAngulo()`. El rumbo
nuevo se acumula sin recortar (`float`, sin límite práctico), así que el problema
desaparece de raíz. Un giro de 180 o de 360 es igual de simple que uno de 30.

### 4.3 Giros de lazo cerrado con frenado activo y corrección

```
Fase 1  pwm = K · √(grados que faltan)    ← desaceleración constante
Fase 2  freno activo (PWM 0 = short brake del puente en H)
Fase 3  esperar a estar QUIETO de verdad (lo comprueba con el giroscopio)
Fase 4  medir el error real, incluida la pasada, y corregirlo a PWM bajo
        (hasta 3 intentos, tolerancia 0,8°)
```

La clave es la fase 3–4: **la pasada del frenado se mide y se corrige**. El ángulo final
deja de depender de la velocidad de crucero, de la batería y del rozamiento. Ahí es donde
ganas el "ir rápido y clavar el ángulo siempre".

Todo el movimiento va en `DIRECT_MODE` con `setMotorPwm()`, saltándose el PID de velocidad
de la librería (el de los 40 ms y las 25 cuentas por ciclo). El PWM responde en el mismo
ciclo de control, que ahora es de **5 ms**.

### 4.4 Timer4 configurado

```c
TCCR4A = _BV(WGM40);  TCCR4B = _BV(WGM42) | _BV(CS41);   // 7812 Hz, igual que Timer1
```

Los dos motores de tracción quedan a la misma frecuencia. Con esto el `*1.02` del motor
derecho probablemente sobre; compruébalo con la opción `v` de `calibrar_gyro`.

### 4.5 Anti-congelamiento en tres capas

1. `Wire.setWireTimeout(4000, true)` — ninguna operación I2C bloquea más de 4 ms.
   Con lecturas de 2 bytes a 400 kHz (~0,11 ms) el margen es de 35×, no de 2×.
2. `desatascarI2C()` — si hay 5 fallos seguidos, manda 9 pulsos de reloj + STOP para que
   el MPU6050 suelte SDA, reinicia el bus y reconfigura el chip. Se cuentan las
   recuperaciones y se imprimen al final.
3. **Watchdog de 1 s**, con el apagado seguro en `.init3` para que un reinicio por
   watchdog no deje la placa en bucle de arranque. Si algo se cuelga pese a todo, la
   placa se reinicia y te avisa por Serial en el arranque siguiente.

Y además: lecturas de encoder atómicas (`ATOMIC_BLOCK`), ISR con lectura directa de puerto
(`PINC`/`PIND`, ~0,2 µs en vez de ~4 µs), y antirrebote de 3 lecturas en el botón de parada
para que un pico eléctrico no aborte la rutina.

### 4.6 Los tiempos ahora son límites, no esperas

`avanzar(grados, velocidad, tiempo)` vuelve **en cuanto termina el movimiento**. El último
parámetro pasó a ser un tiempo límite de seguridad. Puedes dejarlo holgado sin perder ni
una décima, y el robot nunca encadena una orden nueva estando aún en marcha.

### Consumo

| | Flash | RAM |
|---|---|---|
| Original | 27 592 B (11 %) | 1 594 B (19,5 %) |
| Optimizado | 28 224 B (11 %) | 1 444 B (17,6 %) |

---

## 5. Puesta a punto (una sola sesión)

### Paso 1 — `calibrar_gyro.ino`

Súbelo, abre el Monitor Serie a **115200** y escribe letras:

| Tecla | Qué mide | Qué haces con el resultado |
|---|---|---|
| `s` | Errores del bus I2C y µs por lectura | Si hay errores > 0, baja `I2C_HZ` a `200000` |
| `b` | Sesgo y deriva del giroscopio | Deriva > 3 °/min → el MPU está mal alimentado o muy caliente |
| `e` | Escala del giroscopio (giras el robot 360° a mano) | → `GYRO_ESCALA` |
| `v` | rpm de cada motor a PWM 255 | → `RPM_MAX` (y ves si siguen descompensados) |
| `m` | PWM mínimo de arranque | → `GIRO_PWM_MIN` y `RECTO_PWM_MIN` |
| `t` | 5 giros de 90° seguidos | Comprobación: deberían salir todos dentro de ±1° |

Para `e`, gira **3 vueltas completas (1080°)** en vez de una: el error de lectura del
suelo se reparte entre tres y sale mucho más fino.

### Paso 2 — pasar los valores a `prueba_wwl5_opt.ino`

Están todos juntos en el bloque `1. CONFIGURACIÓN` de arriba del sketch.

### Paso 3 — corregir los ángulos de la rutina

**Esto es obligatorio.** Los ángulos que hay en la rutina llevaban descontada la pasada del
frenado. Ahora el robot gira lo que le pides, así que hay que poner los valores reales:

| Antes | Casi seguro debería ser |
|---|---|
| `girarDerechaGyro(85.0, 20.0)` | `90.0` |
| `girarIzquierdaGyro(85.0, 20.0)` | `90.0` |
| `girarIzquierdaGyro(84.0, 35.0)` | `90.0` |
| `girarIzquierdaGyro(175.0, 30.0)` | `180.0` |
| `girarDerechaGyro(89.5, 30.0)` | `90.0` |
| `girarDerechaGyro(87.0, 30.0)` | `90.0` |
| `girarDerechaGyro(99.0, 30.0)` + `girarDerechaGyro(5.0, 15.0)` | probablemente un solo `104.0`, o `90.0` si esos 14° extra eran para compensar |

Los que no son múltiplos de 45 (30, 50, 25, 75, 35, 160) solo los sabes tú: mira en la
pista cuánto gira de más ahora y réstaselo.

Se han dejado tal cual a propósito, para no adivinar tu intención y romperte la rutina.

### Paso 4 — subir la velocidad

Ahora sí puedes. El ángulo final ya no depende de la velocidad de crucero. Sube los
valores de velocidad de los giros hasta que la mecánica empiece a patinar (si las ruedas
patinan el giroscopio sigue midiendo bien, pero el robot se desplaza además de girar).

Con `DEPURAR 1` cada giro y cada avance imprimen por Serial lo pedido, lo real y los
milisegundos que tardaron. Es la forma rápida de ver dónde se va el tiempo.

---

## 6. Hardware — vale la pena revisarlo

El congelamiento es un fallo de bus I2C, y los fallos de bus I2C casi siempre son
eléctricos:

- **Condensador de 100 nF** entre VCC y GND lo más cerca posible del módulo MPU6050.
- **Separar el cable RJ25 del giroscopio de los cables de motor.** Que no vayan
  paralelos ni atados juntos: el di/dt de los motores se acopla capacitivamente a SDA/SCL.
- **Batería**. Si está gastada, los picos de arranque de los motores hunden los 5 V y el
  MPU6050 puede resetearse a media transferencia. Si los cuelgues aparecen sobre todo al
  final de la sesión de pruebas, es esto.
- Comprueba que el conector RJ25 hace buen contacto (es la avería más común del kit).

Con la opción `s` de `calibrar_gyro` puedes medirlo objetivamente: deja el robot moviéndose
y mira si el contador de errores sube.
