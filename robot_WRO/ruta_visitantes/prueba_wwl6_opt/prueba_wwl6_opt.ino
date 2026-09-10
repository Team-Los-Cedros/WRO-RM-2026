// ============================================================================
//  WRO Robomission Junior 2026 - Torre amarilla + barrido
//  MegaPi (ATmega2560) + Makeblock Ultimate 2.0 + Raspberry Pi 3B con webcam
//
//  prueba_wwl6_opt.ino = motores y giroscopio de prueba_wwl5_opt.ino
//                      + la ruta de prueba_wwl6.ino (la del video de Prueba_3)
//                      + la camara como asistente (seccion 12)
//
//  COMO SE USA LA CAMARA
//  La rutina sigue siendo una lista de funciones, igual que siempre. Cuando
//  llamas a una funcion vision...(), la Raspberry Pi toma el control: la MegaPi
//  le dice que busque, mide con el robot QUIETO donde esta el objeto y mueve el
//  robot con los mismos giros y avances de precision de siempre. Al terminar,
//  la rutina sigue en la linea de abajo. Si la camara no contesta o no ve nada,
//  la funcion no mueve el robot (o usa la distancia por defecto que le pasaste)
//  y la rutina continua por odometria, como antes.
//
//  LO QUE MOSTRO EL VIDEO DE PRUEBA_3 (10/09)
//  - La Pi estuvo en PAUSA toda la prueba: su imagen dice "PAUSA/NINGUNO sin
//    objeto" de principio a fin. Nunca recibio (o nunca entendio) "M TORRE_REC",
//    asi que la camara no corrigio nada. Ahora el robot lo comprueba al arrancar
//    y lo avisa con lineas "# VISION ..." (se ven en la telemetria de la Pi).
//  - Con la pala en 63 la torre cargada tapa el centro de la camara. Por eso el
//    destino se mira con la pala en 95 (PALA_VER_DESTINO) y se sube despues.
//  - Al irse despues de soltar, la garra saco la torre de su cuadro. Ahora se
//    retira recto y sube la pala antes de girar.
//
//  -------------------------------------------------------------------------
//  Lo que sigue es la explicacion original de prueba_wwl5_opt.ino.
//
//  Que cambia respecto de la version anterior:
//
//  1) El giroscopio ya NO se lee con MeGyro::update(). Esa funcion lee 14 bytes
//     por I2C a 100 kHz (~1,55 ms de espera bloqueante) y ademas calcula
//     2 atan2, 2 sqrt y unas 45 operaciones en coma flotante (~0,43 ms):
//     casi 2 ms por llamada. Por eso habia que espaciarla cada 25 ms, y con el
//     robot girando rapido 25 ms son varios grados de error. Aqui se lee solo
//     el eje Z (2 bytes) a 400 kHz: ~0,14 ms, y se muestrea cada 2 ms.
//
//  2) El rumbo se guarda SIN recortar a +-180 grados, asi que el salto de
//     +180/-180 desaparece por completo (ya no hace falta deltaAngulo()).
//
//  3) Los giros y los avances son de LAZO CERRADO con rampa de frenado y una
//     fase de correccion final: el robot mide cuanto se paso mientras frenaba
//     y lo corrige. El angulo final ya no depende de la velocidad de crucero
//     ni de como este la bateria.
//
//  4) Se configura el Timer4. El motor del SLOT1 sale por el pin 12 (Timer1) y
//     el del SLOT2 por el pin 8 (Timer4). El codigo de mBlock solo configuraba
//     Timer1 y Timer2, asi que un motor iba a 7812 Hz de PWM y el otro a 490 Hz.
//
//  5) Anti-congelamiento: timeout de I2C, rutina para desatascar el bus,
//     watchdog y lecturas de encoder atomicas.
//
//  ANGULOS: en prueba_wwl6 los giros eran de 85 y 87 porque la funcion vieja
//  se pasaba al frenar. Esta version gira exactamente lo que le pides, asi que
//  la rutina de abajo ya usa los valores reales (90) y, en los giros que tienen
//  que quedar alineados con la pista, girarARumbo() (seccion 7).
// ============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <MeMegaPi.h>
#include <avr/wdt.h>
#include <util/atomic.h>

// ============================================================================
//  1. CONFIGURACION (lo unico que normalmente vas a tocar)
// ============================================================================

// ---- Giroscopio / I2C ----
#define I2C_HZ            400000UL  // 400 kHz. Bajalo a 200000 si ves errores I2C.
#define I2C_TIMEOUT_US      4000UL  // El bus nunca bloquea mas que esto. NUNCA pongas 0.
#define GYRO_PERIODO_US     2000UL  // Cada cuanto se lee el giro (2000 us = 500 Hz)
// GYRO_ESCALA: factor de correccion de la lectura del giroscopio.
// SE QUEDA EN 1.000 A PROPOSITO. Las dos medidas manuales del 09/09 se
// contradicen entre si (0,7698 girando 360 y 1,4909 girando 1080: un factor 2
// de diferencia), asi que ninguna de las dos es fiable. Girar el robot a mano
// durante mas de un minuto no es una medida valida: cualquier inclinacion o
// levantada mete rotacion en el eje Z que no es rumbo del suelo.
// Para medirlo bien usa la opcion 'a' de calibrar_gyro.ino, que hace girar al
// robot solo, apoyado en el suelo, 3 vueltas completas.
#define GYRO_ESCALA          1.000f

// ---- Lazo de control ----
#define CTRL_PERIODO_US     5000UL  // Periodo del lazo de movimiento (5 ms = 200 Hz)
#define ENC_PERIODO_US     10000UL  // Cada cuanto se llama Encoder_x.loop()

// ---- Motores ----
// Medido 09/09: izq 168,7 rpm | der 158,3 rpm a PWM 255 (ruedas al aire).
#define RPM_MAX             158.0f  // rpm del eje de salida con PWM 255
#define PWM_TOPE               255
// El motor izquierdo es un 6,2 % mas rapido. Se le baja el PWM para compensar,
// asi el robot gira sobre su centro y el lazo de rumbo trabaja menos.
// Se corrige el rapido hacia abajo (no el lento hacia arriba) para que la
// compensacion siga valiendo tambien a PWM 255.
#define TRIM_MOTOR_IZQ       0.938f // = 158.3 / 168.7

// ---- Giros ----
// Umbral medido 09/09: arranca a girar con PWM 31; el calibrador sugiere 39.
#define GIRO_PWM_MIN            39  // PWM minimo que todavia hace girar al robot
#define GIRO_K_DECEL         40.0f  // Rampa de frenado: pwm = K * raiz(grados que faltan)
#define GIRO_TOLERANCIA       0.8f  // Error aceptable, en grados
#define GIRO_CORRECCIONES        3  // Reintentos finos como maximo
#define GIRO_MS_ASENTAR        350  // Espera MAXIMA a estar quieto (sale antes si lo esta)
#define GIRO_MS_CORR           700  // Duracion maxima de cada correccion fina
#define GIRO_PWM_ANTIATASCO    120  // Tope al que puede subir el PWM si el robot no arranca
#define GIRO_MS_ATASCO        1500  // Sin avanzar nada durante esto = atascado de verdad

// ---- Avance recto ----
// Umbral medido 09/09: arranca a avanzar con PWM 37; el calibrador sugiere 43.
#define RECTO_PWM_MIN           43
#define RECTO_K_DECEL        16.0f  // Rampa de frenado: pwm = K * raiz(grados que faltan)
#define RECTO_KP              4.0f  // Rumbo: pwm de correccion por grado de desvio
#define RECTO_KD              0.35f // Amortiguacion: pwm por (grado/segundo)
#define RECTO_TOLERANCIA        12  // Error aceptable, en grados de encoder
#define RECTO_CORREGIR           1  // 1 = corrige la distancia despues de frenar
#define RECTO_PWM_ANTIATASCO   120
#define RECTO_MS_ATASCO       1500

// ---- Escaneo de linea ----
#define ESCANEO_PWM_MIN         48  // Arranca y para muchas veces: necesita mas margen

// ---- Seguridad ----
#define USAR_WATCHDOG            1  // 1 = si algo se cuelga, la placa se reinicia sola
#define QUIETO_DPS            4.0f  // |velocidad| por debajo de esto = robot detenido
#define QUIETO_MS               60  // ms seguidos quieto para darlo por bueno
#define DEPURAR                  1  // 1 = imprime por Serial lo pedido, lo real y los ms
                                    // de cada movimiento. Ponlo a 0 en competencia:
                                    // ahorra unos ms por maniobra.
                                    // Todas las lineas empiezan por "# " a proposito: es
                                    // el prefijo de mensaje de depuracion del protocolo
                                    // con la Raspberry Pi, asi que la Pi las registra en
                                    // telemetria sin intentar interpretarlas. NO quites
                                    // ese prefijo si vas a conectar la vision.

// ---- Pines ----
#define BOTON_PIN        4
#define PIN_TCRT_IZQ    A4
#define PIN_TCRT_DER     2
#define PIN_SERVO_PALA   5
#define MPU_ADDR      0x68

// ---- Camara (Raspberry Pi) ----
// La Pi manda por el cable USB una linea "T ..." por foto (20 por segundo) con
// donde ve el objeto. Las funciones de la seccion 12 solo miran con el robot
// QUIETO y lo mueven con los mismos giros y avances de precision de siempre.
#define VISION_ACTIVA            1     // 0 = ignora la camara: odometria pura, como antes
#define MODO_CALIBRAR_VISION     0     // 0 = rutina normal. 1 = calibrar la torre a recoger,
                                       // 2 = calibrar la base (ver calibrarVision())
#define AVISAR_SIN_VISION        1     // 1 = si al pulsar el boton la Pi no esta lista, la
                                       // garra se cierra y se abre una vez para avisar
#define VIS_MS_LATENCIA        250     // tras parar, las fotos anteriores a esto se descartan
#define VIS_MUESTRAS             5     // fotos por medida (se usa la mediana)
#define VIS_MS_MEDIR          1000     // tiempo maximo para juntarlas
#define VIS_PX_POR_GRADO      9.5f     // pixeles que se corre el objeto por grado de giro.
                                       // Es solo el punto de partida: el robot lo afina en
                                       // cada centrado y MODO_CALIBRAR_VISION lo mide.
#define VIS_TOL_PX              10     // centrado aceptable: |ex| <= 10 px (~1 grado)
#define VIS_GIRO_MAX         20.0f     // si pide girar mas que esto, la deteccion es dudosa
#define VIS_INTENTOS             3     // medidas como maximo (con un giro entre cada una)
#define VIS_USAR_DISTANCIA       0     // 1 = el ultimo avance lo decide la camara con la
                                       // tabla de config.json. Dejalo en 0 hasta medirla.
#define VIS_MAX_CORR_GRADOS    150     // la camara puede cambiar un avance como mucho esto

// ============================================================================
//  VELOCIDADES  (rpm del eje de salida, la misma unidad de siempre)
//
//  Con RPM_MAX = 158 la conversion es:   PWM = rpm x 1,61
//
//     rpm   PWM   notas
//     ----  ----  --------------------------------------------------------
//      24    39   minimo absoluto para GIRAR (por debajo se queda parado)
//      27    43   minimo absoluto para AVANZAR
//      35    56   minimo RECOMENDADO: arranca siempre, con o sin carga
//      50    81   maniobras finas, ajustes de pocos grados
//      90   145   uso general
//     120   194   probado el 09/09: 5 giros de 90 con +-0,16 grados
//     150   242   tramos largos
//     158   255   tope. Por encima de 158 no pasa nada: se satura en 255.
//
//  Por debajo de los minimos el codigo sube el PWM al suelo automaticamente,
//  asi que no se rompe nada, pero pedir 20 rpm o pedir 24 da lo mismo.
// ============================================================================
#define V_GIRO_FINO      50.0f   // ajustes pequenos, giros con carga delicada
#define V_GIRO           90.0f   // giro normal
#define V_GIRO_RAPIDO   120.0f   // giros grandes (90, 160, 180)

#define V_APROX          35.0f   // acercarse a un objeto sin tumbarlo
#define V_RECTO_MEDIO    60.0f   // tramos cortos, busqueda de linea
#define V_RECTO          90.0f   // uso general
#define V_RECTO_RAPIDO  150.0f   // tramos largos

// Umbrales de los TCRT5000
int UMBRAL_NEGRO_IZQ = 31;
int UMBRAL_NEGRO_DER = 41;

// ============================================================================
//  2. WATCHDOG SEGURO
//  Esto desactiva el watchdog ANTES de main(), en el arranque. Sin esto, un
//  reinicio por watchdog puede dejar la placa en bucle de arranque.
// ============================================================================
uint8_t mcusr_copia __attribute__((section(".noinit")));
void apagarWdtAlArrancar(void) __attribute__((naked, used, section(".init3")));
void apagarWdtAlArrancar(void)
{
  mcusr_copia = MCUSR;
  MCUSR = 0;
  wdt_disable();
}

// ============================================================================
//  3. OBJETOS
// ============================================================================
MePort placaExpansora(7);   // RJ25 puerto 7 -> pines A10 y A11
Servo  servoGarra1;
Servo  servoGarra2;
Servo  miServo;

MeEncoderOnBoard Encoder_1(SLOT1);  // Motor Izquierdo -> pines 18/31, PWM 12 (Timer1)
MeEncoderOnBoard Encoder_2(SLOT2);  // Motor Derecho   -> pines 19/38, PWM  8 (Timer4)

bool rutinaIniciada = false;

// ---- Interrupciones de encoder ----
// Lectura directa de puerto en vez de digitalRead(): ~0,2 us en vez de ~4 us.
// Pin 31 = PC6 (canal B del SLOT1). Pin 38 = PD7 (canal B del SLOT2).
void isr_encoder1(void)
{
  if (PINC & _BV(PC6)) Encoder_1.pulsePosPlus();
  else                 Encoder_1.pulsePosMinus();
}

void isr_encoder2(void)
{
  if (PIND & _BV(PD7)) Encoder_2.pulsePosPlus();
  else                 Encoder_2.pulsePosMinus();
}

// ============================================================================
//  4. GIROSCOPIO RAPIDO (MPU6050, solo eje Z)
// ============================================================================
class GiroZ
{
public:
  bool  begin(void);
  bool  calibrar(uint16_t muestras = 300);
  void  update(void);
  void  reiniciarRumbo(void) { _h = 0.0f; }
  float grados(void)    const { return _h; }   // rumbo acumulado, sin saltos de +-180
  float velocidad(void) const { return _w; }   // grados por segundo
  float sesgo(void)     const { return _bias; }
  uint16_t errores(void) const { return _err; }
  uint8_t  recuperaciones(void) const { return _recup; }
  bool  ok(void)        const { return _ok; }

private:
  bool escribirReg(uint8_t reg, uint8_t valor);
  bool leerZ(int16_t &destino);
  void configurarChip(void);
  void recuperarBus(void);

  float    _h = 0.0f, _w = 0.0f, _wPrev = 0.0f, _bias = 0.0f;
  uint32_t _tUlt = 0, _tMuestra = 0;
  uint16_t _err = 0;
  uint8_t  _fallosSeguidos = 0;
  uint8_t  _recup = 0;
  bool     _ok = false;
};

GiroZ gyro;

// Libera el bus si el MPU6050 se quedo sujetando SDA (pasa si la placa se
// reinicia en mitad de una lectura I2C).
static void desatascarI2C(void)
{
  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, INPUT_PULLUP);
  delayMicroseconds(20);
  if (digitalRead(SDA) != LOW) return;

  for (uint8_t i = 0; i < 9 && digitalRead(SDA) == LOW; i++) {
    digitalWrite(SCL, LOW);
    pinMode(SCL, OUTPUT);           // fuerza SCL a 0
    delayMicroseconds(5);
    pinMode(SCL, INPUT_PULLUP);     // suelta SCL a 1
    delayMicroseconds(5);
  }
  // condicion de STOP manual
  digitalWrite(SDA, LOW);
  pinMode(SDA, OUTPUT);
  delayMicroseconds(5);
  pinMode(SCL, INPUT_PULLUP);
  delayMicroseconds(5);
  pinMode(SDA, INPUT_PULLUP);
  delayMicroseconds(5);
}

bool GiroZ::escribirReg(uint8_t reg, uint8_t valor)
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(valor);
  return (Wire.endTransmission(true) == 0);
}

bool GiroZ::leerZ(int16_t &destino)
{
  bool bien = false;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write((uint8_t)0x47);                 // GYRO_ZOUT_H
  if (Wire.endTransmission(false) == 0) {
    if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)2, (uint8_t)true) == 2) {
      uint8_t alto = Wire.read();
      uint8_t bajo = Wire.read();
      destino = (int16_t)(((uint16_t)alto << 8) | bajo);
      bien = true;
    }
  }

  if (bien) {
    _fallosSeguidos = 0;
    return true;
  }

  Wire.clearWireTimeoutFlag();
  // Si falla varias veces seguidas, lo mas probable es que el MPU6050 se haya
  // quedado sujetando SDA. Reiniciar solo el maestro no lo arregla: hay que
  // mandar pulsos de reloj para que el esclavo suelte la linea.
  if (++_fallosSeguidos >= 5) recuperarBus();
  return false;
}

void GiroZ::configurarChip(void)
{
  escribirReg(0x6B, 0x01);          // despierta; reloj = PLL del giro X (mas estable)
  escribirReg(0x1A, 0x02);          // DLPF 98 Hz: filtra la vibracion de los motores
  escribirReg(0x19, 0x00);          // salida a 1 kHz
  escribirReg(0x1B, 0x08);          // fondo de escala +-500 grados/s (65.5 LSB por grado/s)
}

void GiroZ::recuperarBus(void)
{
  _fallosSeguidos = 0;
  if (_recup < 255) _recup++;

  Wire.end();
  desatascarI2C();
  Wire.begin();
  Wire.setClock(I2C_HZ);
  Wire.setWireTimeout(I2C_TIMEOUT_US, true);
  configurarChip();
}

bool GiroZ::begin(void)
{
  desatascarI2C();
  Wire.begin();
  Wire.setClock(I2C_HZ);
  Wire.setWireTimeout(I2C_TIMEOUT_US, true);  // sin esto, un fallo del bus CUELGA la placa

  escribirReg(0x6B, 0x80);          // reset del chip (puede no responder, es normal)
  delay(120);

  if (!escribirReg(0x6B, 0x01)) {
    _ok = false;
    return false;
  }
  delay(20);
  configurarChip();
  delay(20);

  int16_t crudo;
  _ok = leerZ(crudo);
  _tUlt = _tMuestra = micros();
  return _ok;
}

bool GiroZ::calibrar(uint16_t muestras)
{
  long    suma   = 0;
  int16_t minimo = 32767, maximo = -32768;
  uint16_t buenas = 0;

  for (uint16_t i = 0; i < muestras; i++) {
    int16_t v;
    if (leerZ(v)) {
      suma += v;
      if (v < minimo) minimo = v;
      if (v > maximo) maximo = v;
      buenas++;
    } else {
      _err++;
    }
    delay(2);
  }

  if (buenas < (muestras / 2)) { _ok = false; return false; }

  _bias  = (float)suma / (float)buenas;
  _h     = 0.0f;
  _w     = 0.0f;
  _wPrev = 0.0f;
  _tUlt = _tMuestra = micros();

  // Si el rango pico a pico es grande, el robot se movio durante la calibracion.
  return (((long)maximo - (long)minimo) < 250);
}

void GiroZ::update(void)
{
  uint32_t ahora = micros();
  if (ahora - _tMuestra < GYRO_PERIODO_US) return;
  _tMuestra = ahora;

  int16_t crudo;
  if (!leerZ(crudo)) {
    _err++;
    return;                      // no tocamos _tUlt: el hueco se integra en la siguiente
  }

  float dt = (float)(ahora - _tUlt) * 1.0e-6f;
  _tUlt = ahora;
  if (dt > 0.05f) dt = 0.05f;    // recorte de seguridad

  _wPrev = _w;
  _w = ((float)crudo - _bias) * (GYRO_ESCALA / 65.5f);
  _h += 0.5f * (_w + _wPrev) * dt;   // integracion trapezoidal (mejor que rectangular)
}

// ============================================================================
//  5. NUCLEO: tarea() y _delay()
// ============================================================================
void pararTodo(void);

static void revisarBotonParada(void)
{
  if (!rutinaIniciada) return;

  static uint32_t tUlt = 0;
  static uint8_t  cuenta = 0;
  uint32_t ahora = millis();
  if (ahora - tUlt < 10) return;
  tUlt = ahora;

  if (digitalRead(BOTON_PIN) == LOW) {
    if (++cuenta >= 3) pararTodo();     // 3 lecturas seguidas: antirrebote
  } else {
    cuenta = 0;
  }
}

// ---- Enlace con la Raspberry Pi (se lee dentro de tarea()) ----
//  La Pi manda:     T <visto> <modo> <ex> <ey> <area> <dist> <fps> <confianza>
//                   K ...  cuando entiende un comando,   E ...  cuando no
//  La MegaPi manda: M TORRE_REC | M TORRE_DEST | M PAUSA   y   P (ping)
//  Todo lo que la MegaPi imprime para depurar empieza por "# ".
struct DatosVision {
  bool     visto;           // la ultima foto ve el objeto
  int      ex;              // px respecto del centro de la garra: + = a la derecha
  int      ey;              // px: fila del pie del objeto (mas grande = mas cerca)
  int      dist;            // lo que diga la tabla de config.json (-1 = sin tabla)
  int      confianza;       // 0..100
  char     modo[12];        // modo con el que la Pi saco esa foto
  uint32_t fotos;           // tramas T recibidas desde el arranque
  uint32_t tFoto;           // millis() de la ultima trama T
  bool     piContesta;      // contesto algo con "K"
  bool     piConoceTorres;  // su version tiene TORRE_REC / TORRE_DEST
  bool     errorModo;       // contesto "E MODO": no conoce el modo pedido
};
DatosVision vis = {};

static char    bufVis[128];
static uint8_t nBufVis = 0;

static void visionLinea(char *s)
{
  if (s[0] == 'T' && s[1] == ' ') {
    char *t = strtok(s + 2, " ");  if (!t) return;
    bool visto = (atoi(t) != 0);
    char *m  = strtok(NULL, " ");  if (!m) return;
    char *sx = strtok(NULL, " ");  if (!sx) return;
    char *sy = strtok(NULL, " ");  if (!sy) return;
    if (!strtok(NULL, " ")) return;             // area (no se usa)
    char *sd = strtok(NULL, " ");  if (!sd) return;
    strtok(NULL, " ");                          // fps (no se usa)
    char *sc = strtok(NULL, " ");

    vis.visto     = visto;
    vis.ex        = atoi(sx);
    vis.ey        = atoi(sy);
    vis.dist      = atoi(sd);
    vis.confianza = sc ? atoi(sc) : 0;
    strncpy(vis.modo, m, sizeof(vis.modo) - 1);
    vis.modo[sizeof(vis.modo) - 1] = '\0';
    vis.tFoto = millis();
    vis.fotos++;
  } else if (s[0] == 'K') {
    vis.piContesta = true;
    if (strstr(s, "TORRE_REC")) vis.piConoceTorres = true;
  } else if (s[0] == 'E' && strstr(s, "MODO")) {
    vis.errorModo = true;
  }
}

// Vacia el buffer serie y procesa las lineas completas. Nunca bloquea.
void visionActualizar(void)
{
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (nBufVis > 0) { bufVis[nBufVis] = '\0'; visionLinea(bufVis); nBufVis = 0; }
    } else if (nBufVis < sizeof(bufVis) - 1) {
      bufVis[nBufVis++] = c;
    } else {
      nBufVis = 0;                              // linea corrupta o demasiado larga
    }
  }
}

// Sustituye a _loop(). Hay que llamarla constantemente: se autolimita sola.
void tarea(void)
{
#if USAR_WATCHDOG
  wdt_reset();
#endif

  gyro.update();                        // se limita solo a GYRO_PERIODO_US

  uint32_t ahora = micros();
  static uint32_t tEnc = 0;
  if (ahora - tEnc >= ENC_PERIODO_US) {
    tEnc = ahora;
    Encoder_1.loop();
    Encoder_2.loop();
  }

  revisarBotonParada();
  visionActualizar();                   // lee lo que mando la Pi (sin esperar)
}

void _loop(void) { tarea(); }           // alias por compatibilidad

void _delay(float segundos)
{
  if (segundos <= 0.0f) return;
  uint32_t espera = (uint32_t)(segundos * 1000.0f);
  uint32_t inicio = millis();
  while (millis() - inicio < espera) tarea();   // ya no hace falta delay(2)
}

// Mantiene el lazo de control a un periodo fijo sin usar delay() a ciegas.
static inline void esperarPeriodo(uint32_t &tRef)
{
  tRef += CTRL_PERIODO_US;
  while ((int32_t)(micros() - tRef) < 0) tarea();
  if ((int32_t)(micros() - tRef) > (int32_t)CTRL_PERIODO_US) tRef = micros();
}

// Espera a que el robot deje de girar de verdad (lo mide con el giroscopio).
static void esperarQuieto(uint16_t msMax)
{
  uint32_t inicio = millis();
  uint32_t desde  = inicio;
  while (millis() - inicio < msMax) {
    tarea();
    if (fabs(gyro.velocidad()) > QUIETO_DPS) desde = millis();
    else if (millis() - desde >= QUIETO_MS)  return;
  }
}

// ============================================================================
//  6. CAPA DE MOTORES
//
//  Todo el movimiento se hace en DIRECT_MODE con setMotorPwm(). Asi el PID
//  interno de la libreria no interfiere: ese PID solo se actualiza cada 40 ms
//  y ademas sube o baja el PWM como maximo 25 cuentas por ciclo, o sea que
//  tarda hasta 400 ms en frenar. Ese retardo era la causa principal de que el
//  angulo final cambiara en cada intento.
//
//  Convenio de signos (identico al codigo original):
//     avanzar        -> Encoder_1 positivo, Encoder_2 negativo
//     girar sentido +1 -> los dos positivos
// ============================================================================

#define ENC_PULSOS   8.0f
#define ENC_RATIO   46.0f   // OJO: setRatio() recibe int16_t, asi que 46.67 se
                            // trunca a 46 dentro de la libreria. Usamos el mismo
                            // valor para que tus distancias sigan significando
                            // lo mismo que antes.
const float GRADOS_POR_PULSO = 360.0f / (ENC_PULSOS * ENC_RATIO);

// pulsePos lo modifica una interrupcion y es de 4 bytes: hay que leerlo con las
// interrupciones apagadas o se puede leer un valor a medio actualizar.
static inline long pulsosSeguro(MeEncoderOnBoard &e)
{
  long v;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { v = e.getPulsePos(); }
  return v;
}

static inline long posGrados(MeEncoderOnBoard &e)
{
  return (long)(pulsosSeguro(e) * GRADOS_POR_PULSO);
}

static inline int16_t pwmDesdeRpm(float rpm)
{
  float p = fabs(rpm) * (255.0f / RPM_MAX);
  if (p > (float)PWM_TOPE) p = (float)PWM_TOPE;
  return (int16_t)p;
}

static inline void modoDirecto(void)
{
  Encoder_1.setMotionMode(DIRECT_MODE);
  Encoder_2.setMotionMode(DIRECT_MODE);
}

// izq/der > 0 = esa rueda hacia adelante. TRIM_MOTOR_IZQ iguala los dos motores.
static inline void pwmRuedas(int16_t izq, int16_t der)
{
  Encoder_1.setMotorPwm(constrain((int16_t)((float)izq * TRIM_MOTOR_IZQ), -PWM_TOPE, PWM_TOPE));
  Encoder_2.setMotorPwm(constrain(-der, -PWM_TOPE, PWM_TOPE));
}

// p > 0 = sentido "izquierda" del codigo original
static inline void pwmGiro(int16_t p)
{
  p = constrain(p, -PWM_TOPE, PWM_TOPE);
  Encoder_1.setMotorPwm((int16_t)((float)p * TRIM_MOTOR_IZQ));
  Encoder_2.setMotorPwm(p);
}

// PWM 0 con los pines de direccion puestos = freno corto del puente en H.
// Es un frenado activo e inmediato, no una parada por inercia.
static inline void frenar(void)
{
  modoDirecto();
  Encoder_1.setMotorPwm(0);
  Encoder_2.setMotorPwm(0);
}

void detener(float tiempoEspera)
{
  frenar();
  _delay(tiempoEspera);
}

void pararTodo(void)
{
  frenar();
  rutinaIniciada = false;

  uint32_t t = millis();
  while (millis() - t < 700) {          // deja que frene de verdad
#if USAR_WATCHDOG
    wdt_reset();
#endif
  }

  // Pines de direccion a 0 = puente en H en reposo. Sin esto los motores zumban.
  digitalWrite(34, LOW); digitalWrite(35, LOW);   // SLOT1
  digitalWrite(36, LOW); digitalWrite(37, LOW);   // SLOT2

#if USAR_WATCHDOG
  wdt_disable();
#endif
  Serial.println(F("# PARADA por boton"));
  for (;;) { delay(200); }
}

// ============================================================================
//  7. GIROS CON GIROSCOPIO
//
//  Fase 1: gira con rampa de frenado  pwm = K * raiz(grados que faltan).
//          Esa curva es la de una desaceleracion constante, asi que el robot
//          llega al objetivo ya muy lento y se pasa poquisimo.
//  Fase 2: frena, espera a estar QUIETO de verdad, mide el error real
//          (incluida la pasada del frenado) y lo corrige a PWM bajo.
//
//  Resultado: el angulo final no depende de la velocidad de crucero. Puedes
//  subir la velocidad todo lo que aguante la mecanica.
// ============================================================================

// Refuerzo anti-atasco: devuelve cuantas cuentas de PWM hay que sumar al suelo
// cuando el robot lleva msParado sin moverse. Sube +1 cada 12 ms tras una
// espera de 120 ms, hasta el tope.
static inline int16_t refuerzoAntiAtasco(uint32_t msParado, int16_t suelo, int16_t tope)
{
  if (msParado <= 120) return suelo;
  int16_t extra = (int16_t)((msParado - 120) / 12);
  int16_t v = suelo + extra;
  return (v > tope) ? tope : v;
}

void girarGyro(float gradosObjetivo, float velocidad, int sentido, float timeoutSeg = 6.0f)
{
  if (gradosObjetivo <= 0.0f) return;

  tarea();
  const float    h0         = gyro.grados();
  const int16_t  pwmCrucero = max(pwmDesdeRpm(velocidad), (int16_t)GIRO_PWM_MIN);
  const uint32_t tIni       = millis();
  const uint32_t tMax       = (uint32_t)(timeoutSeg * 1000.0f);

  modoDirecto();
  uint32_t tCtrl = micros();
  bool porTimeout = false;
  bool porAtasco  = false;

  // ---- Fase 1: crucero con rampa de frenado ----
  float    mejor     = 0.0f;      // maximo recorrido visto hasta ahora
  uint32_t tProgreso = millis();  // ultima vez que el robot avanzo de verdad

  for (;;) {
    tarea();
    float recorrido = fabs(gyro.grados() - h0);
    float falta     = gradosObjetivo - recorrido;
    if (falta <= 0.0f) break;
    if (millis() - tIni > tMax) { porTimeout = true; break; }

    // Detector de progreso. Sustituye al timeout fijo como criterio de fallo:
    // un giro lento pero que avanza NO se aborta; uno bloqueado se corta ya.
    if (recorrido > mejor + 0.5f) { mejor = recorrido; tProgreso = millis(); }
    else if (millis() - tProgreso > GIRO_MS_ATASCO) { porAtasco = true; break; }

    int16_t p = (int16_t)(GIRO_K_DECEL * sqrt(falta));
    p = constrain(p, (int16_t)GIRO_PWM_MIN, pwmCrucero);

    // Si el robot no arranca (roce, alfombra, bateria baja), empuja mas fuerte.
    // Solo lejos del objetivo: cerca hay que dejar que la rampa frene.
    if (falta > 6.0f) {
      int16_t suelo = refuerzoAntiAtasco(millis() - tProgreso,
                                         GIRO_PWM_MIN, GIRO_PWM_ANTIATASCO);
      if (p < suelo) p = suelo;
    }

    pwmGiro(sentido > 0 ? p : (int16_t)-p);
    esperarPeriodo(tCtrl);
  }

  frenar();
  esperarQuieto(GIRO_MS_ASENTAR);

  // ---- Fase 2: correcciones finas ----
  for (uint8_t n = 0; n < GIRO_CORRECCIONES && !porTimeout && !porAtasco; n++) {
    tarea();
    float err = gradosObjetivo - fabs(gyro.grados() - h0);
    if (fabs(err) <= GIRO_TOLERANCIA) break;
    if (millis() - tIni > tMax) { porTimeout = true; break; }

    int8_t   dir    = (err > 0.0f) ? (int8_t)sentido : (int8_t)-sentido;
    uint32_t tCorr  = millis();
    uint32_t tMov   = millis();
    uint32_t tCtrl2 = micros();

    while (millis() - tCorr < GIRO_MS_CORR) {
      tarea();
      float e = gradosObjetivo - fabs(gyro.grados() - h0);
      if ((err > 0.0f && e <= 0.0f) || (err < 0.0f && e >= 0.0f)) break;

      // El tope es GIRO_PWM_ANTIATASCO, NO pwmCrucero: en los giros lentos
      // pwmCrucero vale lo mismo que GIRO_PWM_MIN y la correccion no podia
      // subir de ahi, asi que si el robot se quedaba pegado no salia nunca.
      if (fabs(gyro.velocidad()) > 8.0f) tMov = millis();
      int16_t p = refuerzoAntiAtasco(millis() - tMov, GIRO_PWM_MIN, GIRO_PWM_ANTIATASCO);

      pwmGiro(dir > 0 ? p : (int16_t)-p);
      esperarPeriodo(tCtrl2);
    }
    frenar();
    esperarQuieto(GIRO_MS_ASENTAR);
  }

#if DEPURAR
  Serial.print(F("# giro pedido="));  Serial.print(gradosObjetivo, 1);
  Serial.print(F(" real="));        Serial.print(fabs(gyro.grados() - h0), 2);
  Serial.print(F(" ms="));          Serial.print(millis() - tIni);
  Serial.print(F(" errI2C="));      Serial.print(gyro.errores());
  if (porTimeout) Serial.print(F("  <<< TIMEOUT"));
  if (porAtasco)  Serial.print(F("  <<< ATASCADO"));
  Serial.println();
#endif
}

// ---- Rumbo absoluto ----
// signoGiroIzq: +1 si girar a la izquierda hace crecer gyro.grados(). Depende de
// como esta montado el MPU6050, asi que se aprende solo en cada giro.
int8_t signoGiroIzq = 0;

static void aprenderSignoGiro(float h0, int8_t sentido)
{
  float d = gyro.grados() - h0;
  if (fabs(d) > 3.0f) signoGiroIzq = ((d > 0.0f) == (sentido > 0)) ? 1 : -1;
}

void girarIzquierdaGyro(float gradosObjetivo, float velocidad)
{
  float h0 = gyro.grados();
  girarGyro(gradosObjetivo, velocidad, +1);
  aprenderSignoGiro(h0, +1);
}

void girarDerechaGyro(float gradosObjetivo, float velocidad)
{
  float h0 = gyro.grados();
  girarGyro(gradosObjetivo, velocidad, -1);
  aprenderSignoGiro(h0, -1);
}

// Rumbo del robot respecto de como estaba al pulsar el boton (gyro.calibrar()
// lo pone en 0): 0 = de frente como arranco, +90 = mirando a su izquierda,
// -90 = mirando a su derecha.
float rumboActual(void)
{
  float h = gyro.grados();
  return (signoGiroIzq < 0) ? -h : h;
}

// Gira lo justo para quedar mirando al rumbo pedido, por el lado mas corto.
// A diferencia de girarDerechaGyro(90), no arrastra el error de los giros
// anteriores ni el de una correccion de la camara: sirve para "enderezar".
void girarARumbo(float rumbo, float velocidad)
{
  if (signoGiroIzq == 0) Serial.println(F("# AVISO girarARumbo: aun no hubo ningun giro, se supone signo +1"));
  float dif = rumbo - rumboActual();
  while (dif >  180.0f) dif -= 360.0f;
  while (dif < -180.0f) dif += 360.0f;
  if      (dif >  GIRO_TOLERANCIA) girarIzquierdaGyro(dif, velocidad);
  else if (dif < -GIRO_TOLERANCIA) girarDerechaGyro(-dif, velocidad);
}

// ============================================================================
//  8. AVANCE RECTO CON GIROSCOPIO
// ============================================================================

// Aplica una velocidad base manteniendo el rumbo. base puede ser negativo.
// La correccion se limita al propio valor de base para que ninguna rueda llegue
// a invertirse: si se invirtiera, la distancia recorrida (que se mide con el
// valor absoluto de cada encoder) saldria inflada y el avance se quedaria corto.
static inline void aplicarRecto(int16_t base, float h0)
{
  float desvio = gyro.grados() - h0;
  float corr   = RECTO_KP * desvio + RECTO_KD * gyro.velocidad();

  float limite = (float)abs(base);
  if (limite > 80.0f) limite = 80.0f;
  corr = constrain(corr, -limite, limite);

  pwmRuedas((int16_t)((float)base - corr), (int16_t)((float)base + corr));
}

// sentido: +1 adelante, -1 atras. grados = grados de eje de salida (como antes).
void moverRecto(long grados, float velocidad, float timeoutSeg, int8_t sentido)
{
  if (grados <= 0) return;

  tarea();
  const float    h0         = gyro.grados();
  const long     p0i        = posGrados(Encoder_1);
  const long     p0d        = posGrados(Encoder_2);
  const int16_t  pwmCrucero = max(pwmDesdeRpm(velocidad), (int16_t)RECTO_PWM_MIN);
  const uint32_t tIni       = millis();
  const uint32_t tMax       = (uint32_t)(timeoutSeg * 1000.0f);

  modoDirecto();
  uint32_t tCtrl = micros();
  bool porTimeout = false, porAtasco = false;

  long     mejor     = 0;
  uint32_t tProgreso = millis();

  for (;;) {
    tarea();
    long avance = (labs(posGrados(Encoder_1) - p0i) + labs(posGrados(Encoder_2) - p0d)) / 2;
    long falta  = grados - avance;
    if (falta <= 0) break;
    if (millis() - tIni > tMax) { porTimeout = true; break; }

    if (avance > mejor + 3) { mejor = avance; tProgreso = millis(); }
    else if (millis() - tProgreso > RECTO_MS_ATASCO) { porAtasco = true; break; }

    int16_t base = (int16_t)(RECTO_K_DECEL * sqrt((float)falta));
    base = constrain(base, (int16_t)RECTO_PWM_MIN, pwmCrucero);

    if (falta > 25) {
      int16_t suelo = refuerzoAntiAtasco(millis() - tProgreso,
                                         RECTO_PWM_MIN, RECTO_PWM_ANTIATASCO);
      if (base < suelo) base = suelo;
    }

    aplicarRecto((int16_t)(base * sentido), h0);
    esperarPeriodo(tCtrl);
  }

  frenar();
  esperarQuieto(180);

#if RECTO_CORREGIR
  // ---- Correccion fina de distancia (mide lo que se paso al frenar) ----
  for (uint8_t n = 0; n < 2 && !porTimeout && !porAtasco; n++) {
    tarea();
    long avance = (labs(posGrados(Encoder_1) - p0i) + labs(posGrados(Encoder_2) - p0d)) / 2;
    long err    = grados - avance;
    if (labs(err) <= RECTO_TOLERANCIA) break;
    if (millis() - tIni > tMax) { porTimeout = true; break; }

    int8_t   dir    = (err > 0) ? sentido : (int8_t)-sentido;
    long     ultAv  = avance;
    uint32_t tCorr  = millis();
    uint32_t tMov   = millis();
    uint32_t tCtrl2 = micros();

    while (millis() - tCorr < 500) {
      tarea();
      long av = (labs(posGrados(Encoder_1) - p0i) + labs(posGrados(Encoder_2) - p0d)) / 2;
      long e  = grados - av;
      if ((err > 0 && e <= 0) || (err < 0 && e >= 0)) break;

      if (labs(av - ultAv) > 2) { ultAv = av; tMov = millis(); }
      int16_t p = refuerzoAntiAtasco(millis() - tMov, RECTO_PWM_MIN, RECTO_PWM_ANTIATASCO);

      aplicarRecto((int16_t)(p * dir), h0);
      esperarPeriodo(tCtrl2);
    }
    frenar();
    esperarQuieto(120);
  }
#endif

#if DEPURAR
  long avanceFinal = (labs(posGrados(Encoder_1) - p0i) + labs(posGrados(Encoder_2) - p0d)) / 2;
  Serial.print(F("# recto pedido=")); Serial.print(grados);
  Serial.print(F(" real="));        Serial.print(avanceFinal);
  Serial.print(F(" desvio="));      Serial.print(gyro.grados() - h0, 2);
  Serial.print(F(" ms="));          Serial.print(millis() - tIni);
  if (porTimeout) Serial.print(F("  <<< TIMEOUT"));
  if (porAtasco)  Serial.print(F("  <<< ATASCADO"));
  Serial.println();
#endif
}

void avanzar   (long grados, float velocidad, float timeoutSeg) { moverRecto(grados, velocidad, timeoutSeg, +1); }
void retroceder(long grados, float velocidad, float timeoutSeg) { moverRecto(grados, velocidad, timeoutSeg, -1); }

// Se mantiene la firma antigua para no tocar la rutina. El parametro Kp ya no
// se usa: el rumbo se corrige con RECTO_KP / RECTO_KD, que son globales.
void avanzarRectoGyro(long grados, float velocidadBase, float Kp, float tiempoMax)
{
  (void)Kp;
  moverRecto(grados, velocidadBase, tiempoMax, +1);
}

// ---- Giro sobre el eje medido por ENCODER (sin giroscopio) ----
// Se conserva porque el codigo viejo lo tenia. Para giros de precision usa
// siempre girarIzquierdaGyro / girarDerechaGyro.
void girarPorEncoder(long gradosRueda, float velocidad, int sentido, float timeoutSeg)
{
  if (gradosRueda <= 0) return;

  tarea();
  const long     p0i        = posGrados(Encoder_1);
  const long     p0d        = posGrados(Encoder_2);
  const int16_t  pwmCrucero = max(pwmDesdeRpm(velocidad), (int16_t)GIRO_PWM_MIN);
  const uint32_t tIni       = millis();
  const uint32_t tMax       = (uint32_t)(timeoutSeg * 1000.0f);

  modoDirecto();
  uint32_t tCtrl = micros();

  for (;;) {
    tarea();
    long avance = (labs(posGrados(Encoder_1) - p0i) + labs(posGrados(Encoder_2) - p0d)) / 2;
    long falta  = gradosRueda - avance;
    if (falta <= 0) break;
    if (millis() - tIni > tMax) break;

    int16_t p = (int16_t)(RECTO_K_DECEL * sqrt((float)falta));
    p = constrain(p, (int16_t)GIRO_PWM_MIN, pwmCrucero);
    pwmGiro(sentido > 0 ? p : (int16_t)-p);
    esperarPeriodo(tCtrl);
  }
  frenar();
  esperarQuieto(150);
}

void moverRobot(long gradosIzq, long gradosDer, float velocidad, float tiempoEspera)
{
  if ((gradosIzq >= 0) == (gradosDer >= 0)) {
    moverRecto(labs(gradosIzq), velocidad, tiempoEspera, gradosIzq >= 0 ? +1 : -1);
  } else {
    girarPorEncoder(labs(gradosIzq), velocidad, gradosIzq > 0 ? +1 : -1, tiempoEspera);
  }
}
void girarIzquierda(long grados, float velocidad, float tiempoEspera) { girarPorEncoder(grados, velocidad, +1, tiempoEspera); }
void girarDerecha  (long grados, float velocidad, float tiempoEspera) { girarPorEncoder(grados, velocidad, -1, tiempoEspera); }

// ============================================================================
//  9. SENSORES DE LINEA TCRT5000
// ============================================================================

int  leerLineaIzq(void) { return analogRead(PIN_TCRT_IZQ); }   // A4, analogico
int  leerLineaDer(void) { return digitalRead(PIN_TCRT_DER); }  // pin 2, digital

bool esNegroIzq(void) { return leerLineaIzq() >= UMBRAL_NEGRO_IZQ; }
bool esNegroDer(void) { return leerLineaDer(); }

void imprimirSensoresLinea(void)
{
  Serial.print(F("# TCRT Izq (A4): "));
  Serial.print(leerLineaIzq());
  Serial.print(F(" | TCRT Der (D2): "));
  Serial.println(leerLineaDer());
}

/**
 * Centra el robot sobre la linea negra con micro-giros.
 */
void centrarEnLinea(float velocidad, float timeoutSeg)
{
  const uint32_t tIni = millis();
  const uint32_t tMax = (uint32_t)(timeoutSeg * 1000.0f);
  const int16_t  p    = max(pwmDesdeRpm(velocidad * 0.6f), (int16_t)GIRO_PWM_MIN);

  modoDirecto();
  uint32_t tCtrl = micros();

  while (millis() - tIni < tMax) {
    tarea();
    bool izq = esNegroIzq();
    bool der = esNegroDer();

    if (izq == der) break;                 // los dos igual = centrado
    pwmGiro(izq ? p : (int16_t)-p);        // la linea esta del lado que ve negro
    esperarPeriodo(tCtrl);
  }
  frenar();
  esperarQuieto(120);
}

/**
 * Gira con giroscopio hasta encontrar la linea negra, o hasta gradosMaximos.
 */
void girarGyroLinea(float gradosMaximos, float velocidad, int sentido,
                    float gradosMinimosBusqueda, float timeoutSeg = 4.0f)
{
  tarea();
  const float    h0         = gyro.grados();
  const int16_t  pwmCrucero = max(pwmDesdeRpm(velocidad), (int16_t)GIRO_PWM_MIN);
  const uint32_t tIni       = millis();
  const uint32_t tMax       = (uint32_t)(timeoutSeg * 1000.0f);

  modoDirecto();
  uint32_t tCtrl = micros();
  bool lineaDetectada = false;

  for (;;) {
    tarea();
    float recorrido = fabs(gyro.grados() - h0);
    float falta     = gradosMaximos - recorrido;
    if (falta <= 0.0f) break;
    if (millis() - tIni > tMax) break;

    if (recorrido >= gradosMinimosBusqueda && (esNegroIzq() || esNegroDer())) {
      lineaDetectada = true;
      break;
    }

    int16_t p = (int16_t)(GIRO_K_DECEL * sqrt(falta));
    p = constrain(p, (int16_t)GIRO_PWM_MIN, pwmCrucero);
    pwmGiro(sentido > 0 ? p : (int16_t)-p);
    esperarPeriodo(tCtrl);
  }

  frenar();
  esperarQuieto(150);

  if (lineaDetectada) centrarEnLinea(velocidad, 4.0f);
}

void girarIzquierdaGyroLinea(float gradosMaximos, float velocidad, float gradosMinimosBusqueda, float timeoutSeg = 4.0f)
{ girarGyroLinea(gradosMaximos, velocidad, +1, gradosMinimosBusqueda, timeoutSeg); }

void girarDerechaGyroLinea(float gradosMaximos, float velocidad, float gradosMinimosBusqueda, float timeoutSeg = 4.0f)
{ girarGyroLinea(gradosMaximos, velocidad, -1, gradosMinimosBusqueda, timeoutSeg); }

/**
 * Sigue la linea negra durante unos segundos.
 *
 * OJO: en el codigo viejo el error se calculaba como
 *      error = analogRead(A4) - digitalRead(2)
 * o sea restando un valor de 0..1023 menos un 0 o un 1. Eso no es un
 * seguidor de linea, es practicamente "gira segun lo que vea el sensor
 * izquierdo". Aqui se compara estado contra estado, que es lo correcto
 * mientras el sensor derecho sea digital.
 */
void seguirLineaTiempo(float segundos, float velocidadBase, float ganancia = 45.0f)
{
  const uint32_t tIni = millis();
  const uint32_t tMax = (uint32_t)(segundos * 1000.0f);
  const int16_t  base = max(pwmDesdeRpm(velocidadBase), (int16_t)RECTO_PWM_MIN);

  modoDirecto();
  uint32_t tCtrl = micros();

  while (millis() - tIni < tMax) {
    tarea();
    int8_t  error = (int8_t)(esNegroIzq() ? 1 : 0) - (int8_t)(esNegroDer() ? 1 : 0);
    int16_t corr  = (int16_t)(error * ganancia);
    pwmRuedas((int16_t)(base - corr), (int16_t)(base + corr));
    esperarPeriodo(tCtrl);
  }
  frenar();
  esperarQuieto(150);
}

/**
 * Sigue la linea negra durante una cantidad de grados de encoder.
 */
void seguirLineaGrados(long grados, float velocidadBase, float ganancia = 45.0f, float tiempoMax = 10.0f)
{
  tarea();
  const long     p0i  = posGrados(Encoder_1);
  const long     p0d  = posGrados(Encoder_2);
  const uint32_t tIni = millis();
  const uint32_t tMax = (uint32_t)(tiempoMax * 1000.0f);
  const int16_t  pwmCrucero = max(pwmDesdeRpm(velocidadBase), (int16_t)RECTO_PWM_MIN);

  modoDirecto();
  uint32_t tCtrl = micros();

  while (millis() - tIni < tMax) {
    tarea();
    long avance = (labs(posGrados(Encoder_1) - p0i) + labs(posGrados(Encoder_2) - p0d)) / 2;
    long falta  = grados - avance;
    if (falta <= 0) break;

    int16_t base = (int16_t)(RECTO_K_DECEL * sqrt((float)falta));
    base = constrain(base, (int16_t)RECTO_PWM_MIN, pwmCrucero);

    int8_t  error = (int8_t)(esNegroIzq() ? 1 : 0) - (int8_t)(esNegroDer() ? 1 : 0);
    int16_t corr  = (int16_t)(error * ganancia);
    pwmRuedas((int16_t)(base - corr), (int16_t)(base + corr));
    esperarPeriodo(tCtrl);
  }
  frenar();
  esperarQuieto(150);
}

// ============================================================================
//  10. AVANCE RECTO HASTA LINEA PERPENDICULAR (con barrido de centrado)
// ============================================================================

/**
 * Centra el robot sobre una linea perpendicular con micro-giros.
 * ladoInicial: +1 linea vista a la izquierda, -1 a la derecha, 0 neutro.
 */
void centrarLineaPerpendicular(float velocidadCentrado, float timeoutSeg = 3.0f, int ladoInicial = 0)
{
  const uint32_t tIni = millis();
  const uint32_t tMax = (uint32_t)(timeoutSeg * 1000.0f);
  const int16_t  p    = max(pwmDesdeRpm(velocidadCentrado * 0.6f), (int16_t)GIRO_PWM_MIN);

  uint8_t confirmacion = 0;
  int     ultimoLado   = ladoInicial;

  modoDirecto();
  uint32_t tCtrl = micros();

  while (millis() - tIni < tMax) {
    tarea();
    bool izq = esNegroIzq();
    bool der = esNegroDer();

    if (izq && der) {
      confirmacion++; ultimoLado = 0;
      if (confirmacion >= 2) break;
    } else if (!izq && !der) {
      confirmacion++;
      if (confirmacion >= 4) break;
    } else {
      confirmacion = 0;
      ultimoLado = izq ? 1 : -1;
    }

    int16_t v = 0;
    if      (izq && !der)      v =  p;
    else if (!izq && der)      v = -p;
    else if (ultimoLado != 0)  v = (int16_t)(p * ultimoLado);

    pwmGiro(v);
    esperarPeriodo(tCtrl);
  }
  frenar();
  esperarQuieto(120);
}

// Posicion de avance relativa (mismo calculo que el codigo original).
long posAvanceRelativo(long izq0, long der0)
{
  return ((posGrados(Encoder_1) - izq0) - (posGrados(Encoder_2) - der0)) / 2;
}

/**
 * Avanza recto con giroscopio hasta detectar una linea negra perpendicular,
 * luego hace un barrido adelante/atras para medir el ancho de la linea y se
 * coloca en su centro.
 *
 * El parametro Kp se conserva por compatibilidad pero ya no se usa: el rumbo
 * lo controlan RECTO_KP y RECTO_KD.
 */
void avanzarRectoGyroLineaPerpendicular(long  gradosMaximos,
                                        float velocidadBase,
                                        float timeoutSeg = 10.0f,
                                        long  rango = 135,
                                        long  offsetCentro = 0,
                                        float velocidadEscaneo = 35.0f,
                                        float Kp = 1.5f,
                                        float gradosMinimosBusqueda = 0.0f)
{
  (void)Kp;

  // ---------- 1) Avanzar hasta detectar la linea ----------
  tarea();
  float h0  = gyro.grados();
  long  iz0 = posGrados(Encoder_1);
  long  de0 = posGrados(Encoder_2);

  const int16_t  pwmCrucero = max(pwmDesdeRpm(velocidadBase), (int16_t)RECTO_PWM_MIN);
  uint32_t tIni  = millis();
  uint32_t tMax  = (uint32_t)(timeoutSeg * 1000.0f);
  uint32_t tCtrl = micros();
  bool detectada = false;

  modoDirecto();

  while (millis() - tIni < tMax) {
    tarea();
    long av    = (labs(posGrados(Encoder_1) - iz0) + labs(posGrados(Encoder_2) - de0)) / 2;
    long falta = gradosMaximos - av;
    if (falta <= 0) break;

    if ((float)av >= gradosMinimosBusqueda && esNegroIzq() && esNegroDer()) {
      detectada = true;
      break;
    }

    int16_t base = (int16_t)(RECTO_K_DECEL * sqrt((float)falta));
    base = constrain(base, (int16_t)RECTO_PWM_MIN, pwmCrucero);
    aplicarRecto(base, h0);
    esperarPeriodo(tCtrl);
  }

  frenar();
  esperarQuieto(150);
  if (!detectada) return;

  // ---------- 2) Barrido adelante / atras para medir la banda negra ----------
  const int16_t pwmEscaneo = max(pwmDesdeRpm(velocidadEscaneo), (int16_t)ESCANEO_PWM_MIN);

  tarea();
  h0  = gyro.grados();
  iz0 = posGrados(Encoder_1);
  de0 = posGrados(Encoder_2);

  long minLinea = 1000000L, maxLinea = -1000000L;
  long minAmbos = 1000000L, maxAmbos = -1000000L;
  long mejorPos = 0, mejorAnalog = -1;
  int  mejorScore = -1;
  bool bandaLinea = false, bandaAmbos = false;

  const long destinos[2] = { rango, -rango };

  for (uint8_t i = 0; i < 2; i++) {
    uint32_t tPaso = millis();
    tCtrl = micros();

    while (millis() - tPaso < 4000UL) {
      tarea();
      long p      = posAvanceRelativo(iz0, de0);
      long err    = destinos[i] - p;
      long errAbs = labs(err);
      if (errAbs <= 4) break;

      int  li = leerLineaIzq();
      int  ld = leerLineaDer();
      bool izq = (li >= UMBRAL_NEGRO_IZQ);
      bool der = (ld != 0);

      int  sc     = (izq ? 1 : 0) + (der ? 1 : 0);
      long analog = (long)li + ld;

      if (sc > mejorScore || (sc == mejorScore && analog > mejorAnalog)) {
        mejorScore = sc; mejorAnalog = analog; mejorPos = p;
      }
      if (sc > 0) {
        if (p < minLinea) minLinea = p;
        if (p > maxLinea) maxLinea = p;
        bandaLinea = true;
      }
      if (izq && der) {
        if (p < minAmbos) minAmbos = p;
        if (p > maxAmbos) maxAmbos = p;
        bandaAmbos = true;
      }

      // rampa de frenado tambien en el barrido, para no pasarse del destino
      int16_t base = (int16_t)(RECTO_K_DECEL * sqrt((float)errAbs));
      base = constrain(base, (int16_t)ESCANEO_PWM_MIN, pwmEscaneo);
      aplicarRecto((err > 0) ? base : (int16_t)-base, h0);
      esperarPeriodo(tCtrl);
    }
    frenar();
    esperarQuieto(120);
  }

  if (!bandaLinea && mejorScore <= 0) return;

  // ---------- 3) Ir al centro de la banda ----------
  long objetivo;
  if      (bandaAmbos) objetivo = (minAmbos + maxAmbos) / 2;
  else if (bandaLinea) objetivo = (minLinea + maxLinea) / 2;
  else                 objetivo = mejorPos;
  objetivo += offsetCentro;

  tarea();
  h0    = gyro.grados();
  tIni  = millis();
  tCtrl = micros();

  while (millis() - tIni < 2500UL) {
    tarea();
    long err    = objetivo - posAvanceRelativo(iz0, de0);
    long errAbs = labs(err);
    if (errAbs <= 3) break;

    int16_t base = (int16_t)(RECTO_K_DECEL * sqrt((float)errAbs));
    base = constrain(base, (int16_t)ESCANEO_PWM_MIN, pwmEscaneo);
    aplicarRecto((err > 0) ? base : (int16_t)-base, h0);
    esperarPeriodo(tCtrl);
  }
  frenar();
  esperarQuieto(150);
}

// ============================================================================
//  11. SERVOS
// ============================================================================

const int GARRA_ABIERTA_S1 = 0;
const int GARRA_CERRADA_S1 = 108;
const int GARRA_CERRADA_S2 = 58;
const int GARRA_ABIERTA_S2 = 180;

const int anguloBajar       = 105;
const int anguloTransportar = 63;
const int anguloDepositar   = 70;

void abrirGarra(void)
{
  servoGarra1.write(GARRA_ABIERTA_S1);
  servoGarra2.write(GARRA_ABIERTA_S2);
  _delay(1.0);
}

void cerrarGarra(void)
{
  servoGarra1.write(GARRA_CERRADA_S1);
  servoGarra2.write(GARRA_CERRADA_S2);
  _delay(1.0);
}

void subir_pala(void)
{
  for (int angulo = anguloBajar; angulo >= 0; angulo--) {
    miServo.write(angulo);
    _delay(0.050);
  }
  _delay(1.0);
}

void bajar_pala(void)
{
  miServo.write(anguloBajar);
  _delay(1.0);
}

void barrer(void)
{
  abrirGarra();
  miServo.write(117);
  _delay(1.0);
}

void recolectar(int modo)
{
  if      (modo == 1) miServo.write(anguloTransportar);
  else if (modo == 2) miServo.write(120);
  else if (modo == 3) miServo.write(111);
  else if (modo == 4) miServo.write(95);
  _delay(1.0);
}

void posicionar(void)
{
  miServo.write(70);
  _delay(1.0);
  abrirGarra();
}

void depositar(void)
{
  for (int angulo = anguloTransportar; angulo <= anguloDepositar; angulo++) {
    miServo.write(angulo);
    _delay(0.50);
  }
  _delay(0.5);
  abrirGarra();
}

/**
 * Avanza mientras mueve los micro-servos de lado a lado (efecto parabrisas).
 */
void parabrisas(long pasos, float velocidad = 100.0f)
{
  const int izqS1 = 0,   izqS2 = 0;
  const int derS1 = 180, derS2 = 180;

  servoGarra1.write(izqS1);
  servoGarra2.write(izqS2);
  _delay(0.5);
  miServo.write(115);
  _delay(0.5);

  tarea();
  const float    h0         = gyro.grados();
  const long     p0i        = posGrados(Encoder_1);
  const long     p0d        = posGrados(Encoder_2);
  const int16_t  pwmCrucero = max(pwmDesdeRpm(velocidad), (int16_t)RECTO_PWM_MIN);
  const uint32_t tIni       = millis();
  const uint32_t tMax       = (uint32_t)((pasos / max(fabs(velocidad), 1.0f)) * 2000.0f) + 2000UL;

  modoDirecto();
  uint32_t tCtrl    = micros();
  uint32_t tBarrido = millis();
  bool     enIzquierda = true;

  for (;;) {
    tarea();
    long avance = (labs(posGrados(Encoder_1) - p0i) + labs(posGrados(Encoder_2) - p0d)) / 2;
    long falta  = (pasos - 15) - avance;
    if (falta <= 0) break;
    if (millis() - tIni > tMax) break;

    if (millis() - tBarrido >= 350) {       // cambia de lado cada 350 ms
      tBarrido = millis();
      enIzquierda = !enIzquierda;
      servoGarra1.write(enIzquierda ? izqS1 : derS1);
      servoGarra2.write(enIzquierda ? izqS2 : derS2);
    }

    int16_t base = (int16_t)(RECTO_K_DECEL * sqrt((float)falta));
    base = constrain(base, (int16_t)RECTO_PWM_MIN, pwmCrucero);
    aplicarRecto(base, h0);
    esperarPeriodo(tCtrl);
  }

  frenar();
  esperarQuieto(150);

  miServo.write(114);
  _delay(0.5);
}

// ============================================================================
//  12. LA CAMARA TOMA EL CONTROL
//
//  Se usan como cualquier otro movimiento, metiendolas en la rutina:
//
//    visionCentrar("TORRE_REC");   gira hasta tener la torre frente a la garra
//    visionCentrar("TORRE_DEST");  gira hasta tener la base frente a la garra
//    avanzar(visionDistancia(265), V_APROX, 5.0);
//                                  avanza lo que mide la camara (265 si no mide)
//
//  visionCentrar() no gira "a ojo" mientras mira: para, espera fotos tomadas
//  con el robot quieto, calcula los grados, gira con el giroscopio y vuelve a
//  mirar (hasta VIS_INTENTOS medidas). Asi el retardo de la camara (~0,2 s)
//  no le hace pasarse, que es lo que le pasaba a visionCentrarRapido().
// ============================================================================

const int  PALA_VER_DESTINO   = 95;   // = recolectar(4). Con la torre en la garra la
                                      // camara todavia ve por encima de ella. Si lo
                                      // cambias, mide otra vez torres.destino.zonas_ignoradas.
const int  PALA_TRAS_SOLTAR   = 40;   // pala arriba despues de soltar, antes de girar
const long RETIRO_TRAS_SOLTAR = 110;  // grados hacia atras, en recto, despues de soltar

float pxPorGrado = VIS_PX_POR_GRADO;  // se afina solo en cada centrado

// Ultima medida buena de visionCentrar(). La usa visionDistancia().
struct { bool valida; int ex, ey, dist; } visUltima = {};

// Comprueba que la Pi contesta y que su codigo conoce los modos de torre.
bool visionComprobar(uint16_t msMax)
{
#if VISION_ACTIVA
  vis.piContesta = false;
  Serial.println(F("P"));
  uint32_t t0 = millis();
  while (millis() - t0 < msMax && !vis.piContesta) tarea();

  if (!vis.piContesta) {
    Serial.println(F("# VISION: la Pi no contesta (cable USB, o servicio wro-vision parado)"));
  } else if (!vis.piConoceTorres) {
    Serial.println(F("# VISION: la Pi tiene codigo viejo sin TORRE_REC/TORRE_DEST. Ejecuta: python herramientas/sync_pi.py push"));
  } else {
    Serial.println(F("# VISION OK"));
    return true;
  }
#else
  (void)msMax;
#endif
  return false;
}

// Pide un modo a la Pi y espera a que las fotos lleguen ya en ese modo.
bool visionModo(const char *modo, uint16_t msMax = 700)
{
#if VISION_ACTIVA
  vis.errorModo = false;
  Serial.print(F("M "));
  Serial.println(modo);
  uint32_t t0 = millis();
  while (millis() - t0 < msMax && !vis.errorModo) {
    tarea();
    if (millis() - vis.tFoto < 200 && strcmp(vis.modo, modo) == 0) return true;
  }
  Serial.print(F("# VISION: la Pi no paso a "));
  Serial.print(modo);
  if      (vis.errorModo)              Serial.println(F(" (contesto E MODO: codigo viejo, haz sync_pi.py push)"));
  else if (vis.fotos == 0)             Serial.println(F(" (no llega ninguna foto)"));
  else if (millis() - vis.tFoto > 500) Serial.println(F(" (dejaron de llegar fotos)"));
  else                                 Serial.println(F(" (sigue en otro modo)"));
#else
  (void)modo; (void)msMax;
#endif
  return false;
}

void visionPausar(void)
{
#if VISION_ACTIVA
  Serial.println(F("M PAUSA"));
#endif
}

int mediana(int *v, uint8_t n)
{
  for (uint8_t i = 1; i < n; i++) {           // insercion: n es muy pequeno
    int x = v[i];
    int8_t j = (int8_t)i - 1;
    while (j >= 0 && v[j] > x) { v[j + 1] = v[j]; j--; }
    v[j + 1] = x;
  }
  return v[n / 2];
}

// Mide con el robot QUIETO: descarta las fotos tomadas mientras se movia
// (VIS_MS_LATENCIA) y devuelve la mediana de VIS_MUESTRAS fotos en las que la
// Pi, en el modo pedido, ve el objeto. Si las fotos no coinciden entre si
// (por ejemplo, salta entre dos torres) devuelve false.
bool visionMedir(const char *modo, int &ex, int &ey, int &dist)
{
#if VISION_ACTIVA
  int vx[VIS_MUESTRAS], vy[VIS_MUESTRAS], vd[VIS_MUESTRAS], copia[VIS_MUESTRAS];
  uint8_t n = 0, sinVer = 0;

  uint32_t t0 = millis();
  while (millis() - t0 < VIS_MS_LATENCIA) tarea();

  uint32_t ultima = vis.fotos;
  while (n < VIS_MUESTRAS && millis() - t0 < VIS_MS_LATENCIA + VIS_MS_MEDIR) {
    tarea();
    if (vis.fotos == ultima) continue;        // todavia no llego una foto nueva
    ultima = vis.fotos;
    if (strcmp(vis.modo, modo) != 0) continue;
    if (!vis.visto) { sinVer++; continue; }
    vx[n] = vis.ex; vy[n] = vis.ey; vd[n] = vis.dist; n++;
  }

  const uint8_t minimo = (VIS_MUESTRAS + 1) / 2;
  if (n >= minimo) {
    for (uint8_t i = 0; i < n; i++) copia[i] = vx[i];
    int mx = mediana(copia, n);
    uint8_t coinciden = 0;
    for (uint8_t i = 0; i < n; i++) if (abs(vx[i] - mx) <= 30) coinciden++;
    if (coinciden >= minimo) {
      ex = mx;
      ey = mediana(vy, n);
      dist = mediana(vd, n);
      return true;
    }
  }
  Serial.print(F("# VISION "));
  Serial.print(modo);
  Serial.print(F(": no se ve bien (")); Serial.print(n);
  Serial.print(F(" fotos con objeto, ")); Serial.print(sinVer);
  Serial.println(F(" sin objeto)"));
#else
  (void)modo; (void)ex; (void)ey; (void)dist;
#endif
  return false;
}

// Gira con el giroscopio hasta que el objeto quede frente a la garra.
// Devuelve true si quedo centrado. Si la camara no lo ve, no mueve el robot.
bool visionCentrar(const char *modo)
{
  visUltima.valida = false;
  if (!visionModo(modo)) return false;

  int   ex = 0, ey = 0, d = -1;
  int   exAntes = 0;
  float giroAntes = 0.0f;                     // ultimo giro, con signo (+ = derecha)
  float girado = 0.0f;                        // total girado por la camara

  for (uint8_t i = 0; i < VIS_INTENTOS; i++) {
    if (!visionMedir(modo, ex, ey, d)) return false;

    // Afina pxPorGrado con lo que se corrio de verdad la imagen en el ultimo giro.
    if (fabs(giroAntes) > 1.5f) {
      float medido = (float)(exAntes - ex) / giroAntes;
      if (medido > 4.0f && medido < 25.0f) pxPorGrado = 0.5f * pxPorGrado + 0.5f * medido;
    }

    visUltima.valida = true;
    visUltima.ex = ex; visUltima.ey = ey; visUltima.dist = d;

    if (abs(ex) <= VIS_TOL_PX || i == VIS_INTENTOS - 1) break;

    float ang = (float)ex / pxPorGrado;       // + = objeto a la derecha
    if (fabs(ang) > VIS_GIRO_MAX) {
      Serial.print(F("# VISION ")); Serial.print(modo);
      Serial.print(F(": pide girar ")); Serial.print(ang, 1);
      Serial.println(F(" grados, demasiado: no se corrige"));
      visUltima.valida = false;
      return false;
    }
    float g0 = gyro.grados();
    if (ang > 0.0f) girarDerechaGyro(ang, V_GIRO_FINO);
    else            girarIzquierdaGyro(-ang, V_GIRO_FINO);
    giroAntes = (ang > 0.0f ? 1.0f : -1.0f) * fabs(gyro.grados() - g0);
    exAntes   = ex;
    girado   += giroAntes;
  }

  bool ok = (abs(ex) <= VIS_TOL_PX);
  Serial.print(F("# VISION ")); Serial.print(modo);
  Serial.print(ok ? F(" centrada ex=") : F(" SIN centrar ex=")); Serial.print(ex);
  Serial.print(F(" ey=")); Serial.print(ey);
  Serial.print(F(" dist=")); Serial.print(d);
  Serial.print(F(" giro=")); Serial.print(girado, 1);
  Serial.print(F(" px/grado=")); Serial.println(pxPorGrado, 1);
  return ok;
}

// Grados que faltan hasta el punto de agarre o de colocacion, segun la ultima
// medida de visionCentrar(). Con VIS_USAR_DISTANCIA = 0, o si la camara no
// midio, devuelve gradosPorDefecto: lo mismo que avanzaria sin camara.
long visionDistancia(long gradosPorDefecto)
{
#if VISION_ACTIVA && VIS_USAR_DISTANCIA
  if (visUltima.valida && visUltima.dist > 0) {
    long g = constrain((long)visUltima.dist,
                       gradosPorDefecto - VIS_MAX_CORR_GRADOS,
                       gradosPorDefecto + VIS_MAX_CORR_GRADOS);
    Serial.print(F("# VISION distancia: ")); Serial.print(g);
    Serial.print(F(" grados (sin camara serian ")); Serial.print(gradosPorDefecto);
    Serial.println(F(")"));
    return g;
  }
#endif
  return gradosPorDefecto;
}

// Deja la parte superior encima de la base y se va sin tocarla. La base tiene
// que seguir completamente dentro de su cuadro amarillo: si queda solo en parte
// son 15 puntos en vez de 25 (reglas 2026, apartado 3.2).
void colocarSobreBase(void)
{
  depositar();                                    // baja de 63 a 70 despacio y abre la garra
  retroceder(RETIRO_TRAS_SOLTAR, V_APROX, 3.0);   // primero recto hacia atras...
  miServo.write(PALA_TRAS_SOLTAR);                // ...y la pala arriba antes de girar
  _delay(0.6);
}

// Aviso visible de que la Pi no esta lista: la garra se cierra y se abre.
void avisoSinVision(void)
{
  servoGarra1.write(GARRA_CERRADA_S1); servoGarra2.write(GARRA_CERRADA_S2); _delay(0.35);
  servoGarra1.write(GARRA_ABIERTA_S1); servoGarra2.write(GARRA_ABIERTA_S2); _delay(0.35);
}

#if MODO_CALIBRAR_VISION
static void esperarBoton(void)
{
  while (digitalRead(BOTON_PIN) == HIGH) tarea();
  while (digitalRead(BOTON_PIN) == LOW)  tarea();
  _delay(0.3);
}

// Mide la tabla [ey, grados que faltan] de config.json retrocediendo desde la
// posicion ideal, y de paso cx_garra y px/grado. El resultado sale por la
// telemetria de la Pi en lineas "# CAL ...".
//  MODO_CALIBRAR_VISION 1 = torre a recoger (TORRE_REC):
//     pulsa: baja la pala y abre la garra. Pon una torre justo donde la garra la
//     agarra y pulsa otra vez.
//  MODO_CALIBRAR_VISION 2 = base de destino (TORRE_DEST):
//     pulsa: abre la garra con la pala en 63. Mete la parte superior entre los
//     dedos y pulsa (la garra se cierra). Pon el robot con esa pieza justo encima
//     de la base, donde deberia soltarla, y pulsa otra vez.
void calibrarVision(void)
{
  const bool    destino = (MODO_CALIBRAR_VISION == 2);
  const char   *modo    = destino ? "TORRE_DEST" : "TORRE_REC";
  const long    PASO    = destino ? 80 : 40;
  const uint8_t PUNTOS  = 8;

  if (destino) { miServo.write(anguloTransportar); abrirGarra(); esperarBoton(); cerrarGarra(); }
  else         { bajar_pala(); abrirGarra(); }
  esperarBoton();

  gyro.calibrar(150);
  rutinaIniciada = true;                      // desde aqui el boton vuelve a ser parada
  if (!visionComprobar(500) || !visionModo(modo)) return;

  int     tabEy[PUNTOS];
  long    tabG[PUNTOS];
  uint8_t nTab = 0;
  long    atras = 0;
  int     ex, ey, d;

  if (destino) {                              // en 63 la pieza tapa la camara: se aleja
    retroceder(160, V_APROX, 4.0);            // un poco y baja la pala para mirar
    atras = 160;
    miServo.write(PALA_VER_DESTINO);
    _delay(0.8);
  } else if (visionMedir(modo, ex, ey, d)) {
    Serial.print(F("# CAL ex con la torre en la garra = ")); Serial.print(ex);
    Serial.println(F("  (si no es 0, suma ese valor a torres.cx_garra)"));
  }

  for (uint8_t i = 0; i < PUNTOS; i++) {
    if (visionMedir(modo, ex, ey, d)) {
      tabEy[nTab] = ey; tabG[nTab] = atras; nTab++;
      Serial.print(F("# CAL ey=")); Serial.print(ey);
      Serial.print(F(" faltan=")); Serial.println(atras);
    }
    if (i < PUNTOS - 1) { retroceder(PASO, V_APROX, 3.0); atras += PASO; }
  }

  // px/grado: gira 6 grados a la derecha mirando el objeto y vuelve
  int ex0, ex1;
  if (visionMedir(modo, ex0, ey, d)) {
    float g0 = gyro.grados();
    girarDerechaGyro(6.0, V_GIRO_FINO);
    float g = fabs(gyro.grados() - g0);
    if (g > 2.0f && visionMedir(modo, ex1, ey, d)) {
      Serial.print(F("# CAL px_por_grado=")); Serial.println((float)(ex0 - ex1) / g, 2);
    }
    girarIzquierdaGyro(6.0, V_GIRO_FINO);
  }

  Serial.print(F("# CAL tabla_distancia para torres."));
  Serial.print(destino ? F("destino") : F("recolectar"));
  Serial.print(F(": ["));
  for (uint8_t i = 0; i < nTab; i++) {
    if (i) Serial.print(F(", "));
    Serial.print('['); Serial.print(tabEy[i]); Serial.print(F(", "));
    Serial.print(tabG[i]); Serial.print(']');
  }
  Serial.println(']');
  visionPausar();
}
#endif

// ============================================================================
//  13. SETUP
// ============================================================================

void setup()
{
  Serial.begin(115200);

  pinMode(BOTON_PIN,    INPUT_PULLUP);
  pinMode(PIN_TCRT_IZQ, INPUT);
  pinMode(PIN_TCRT_DER, INPUT);

  // ------------------------------------------------------------------
  //  PWM DE LOS MOTORES
  //  SLOT1 -> pin 12 = OC1B (Timer1)
  //  SLOT2 -> pin  8 = OC4C (Timer4)   <-- mBlock NO lo configuraba
  //  SLOT3 -> pin  9 = OC2B (Timer2)
  //  Los tres quedan en Fast PWM de 8 bits con preescala 8:
  //  Timer1/Timer4 = 7812 Hz, Timer2 = 3906 Hz.
  //  Antes el motor derecho iba a 490 Hz (valor por defecto del core) y por eso
  //  no respondia igual que el izquierdo al mismo valor de PWM.
  // ------------------------------------------------------------------
  TCCR1A = _BV(WGM10);               TCCR1B = _BV(WGM12) | _BV(CS11);
  TCCR2A = _BV(WGM21) | _BV(WGM20);  TCCR2B = _BV(CS21);
  TCCR4A = _BV(WGM40);               TCCR4B = _BV(WGM42) | _BV(CS41);

  // ------------------------------------------------------------------
  //  ENCODERS
  // ------------------------------------------------------------------
  attachInterrupt(digitalPinToInterrupt(Encoder_1.getPortA()), isr_encoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(Encoder_2.getPortA()), isr_encoder2, RISING);

  Encoder_1.setPulse(8);
  Encoder_2.setPulse(8);
  Encoder_1.setRatio(46);   // setRatio() es int16_t: 46.67 se truncaba a 46 igual
  Encoder_2.setRatio(46);
  Encoder_1.setPosPid(1.8, 0, 1.2);
  Encoder_2.setPosPid(1.8, 0, 1.2);
  Encoder_1.setSpeedPid(0.18, 0, 0);
  Encoder_2.setSpeedPid(0.18, 0, 0);
  modoDirecto();
  frenar();

  // ------------------------------------------------------------------
  //  SERVOS
  // ------------------------------------------------------------------
  miServo.attach(PIN_SERVO_PALA);
  servoGarra1.attach(placaExpansora.pin1());   // A10
  servoGarra2.attach(placaExpansora.pin2());   // A11
  miServo.write(0);
  servoGarra1.write(GARRA_ABIERTA_S1);
  servoGarra2.write(GARRA_ABIERTA_S2);
  delay(700);                                   // que terminen de moverse

  // ------------------------------------------------------------------
  //  GIROSCOPIO (lo ultimo, con el robot totalmente quieto)
  // ------------------------------------------------------------------
  if (!gyro.begin()) {
    Serial.println(F("# ERROR: el MPU6050 no responde. Revisa el cable RJ25."));
  } else if (!gyro.calibrar(300)) {
    Serial.println(F("# AVISO: calibracion de giro dudosa (el robot se movio?)."));
  }

  Serial.print(F("# Sesgo giro Z = ")); Serial.println(gyro.sesgo(), 1);
  if (mcusr_copia & _BV(WDRF)) Serial.println(F("# AVISO: el arranque anterior fue un reinicio por WATCHDOG."));

  // ------------------------------------------------------------------
  //  CAMARA: comprueba que la Pi contesta. Se repite al pulsar el boton,
  //  por si la Pi termino de arrancar despues que la MegaPi.
  // ------------------------------------------------------------------
  visionComprobar(400);

#if USAR_WATCHDOG
  wdt_enable(WDTO_1S);      // si algo bloquea mas de 1 s, la placa se reinicia
#endif
}

// ============================================================================
//  14. RUTINA DE COMPETENCIA
//
//  Es la ruta de prueba_wwl6.ino (la del video de Prueba_3): recoger la parte
//  superior de la torre amarilla, ponerla sobre su base y barrer. Entre
//  parentesis, en los comentarios, van los valores que tenia prueba_wwl6.
//
//  RUMBOS: girarARumbo(r) deja al robot mirando al rumbo r respecto de como
//  arranco (0 = de frente, -90 = a su derecha). Se usa en los giros que tienen
//  que quedar alineados con la pista, para que el error de un giro o de una
//  correccion de la camara no se arrastre al tramo siguiente.
//
//  TIEMPOS: el ultimo parametro de avanzar/retroceder es un TIEMPO LIMITE de
//  seguridad, no una espera: la funcion vuelve en cuanto termina.
// ============================================================================

void loop()
{
  Serial.println(F("# Listo. Pulsa el boton."));

#if MODO_CALIBRAR_VISION
  calibrarVision();
  while (1) tarea();
#endif

  while (digitalRead(BOTON_PIN) == HIGH) tarea();   // esperar pulsacion
  while (digitalRead(BOTON_PIN) == LOW)  tarea();   // esperar que se suelte

  _delay(0.5);

  // Recalibra el sesgo del giroscopio justo antes de arrancar: el robot lleva
  // un rato quieto, es el mejor momento, y asi corrige la deriva por temperatura.
  // Ademas deja el rumbo en 0: desde aqui se miden los rumbos de girarARumbo().
  gyro.calibrar(150);

  // La camara tiene que estar lista ANTES de salir. Si no lo esta, la garra
  // avisa (se cierra y se abre) y la rutina sigue igual, pero por odometria.
  bool camaraLista = visionComprobar(300);
  if (VISION_ACTIVA && AVISAR_SIN_VISION && !camaraLista) avisoSinVision();

  rutinaIniciada = true;

  // ---- 1. Ponerse frente a la torre amarilla ----
  girarIzquierdaGyro(90.0, V_GIRO);             // (izq 85 a 95 rpm)
  avanzar(100, V_RECTO_MEDIO, 3.0);             // (100)
  girarARumbo(0.0, V_GIRO);                     // de frente otra vez, como arranco (der 85)
  avanzar(605, V_RECTO, 6.0);                   // (605)

  // ---- 2. Recoger la parte superior de la torre ----
  bajar_pala();                                 // 105: garra abierta a ras del suelo
  visionCentrar("TORRE_REC");                   // LA CAMARA gira el robot hasta tener la torre entre los dedos
  avanzar(visionDistancia(265), V_APROX, 5.0);  // (265) despacio para no tumbarla
  cerrarGarra();
  recolectar(4);                                // 95 = PALA_VER_DESTINO: la levanta un poco
  // Si queda torcida en la garra, re-agarrarla como en prueba_wwl5_opt:
  // retroceder(110, V_APROX, 4.0); abrirGarra(); avanzar(45, V_APROX, 2.5); cerrarGarra();
  retroceder(146, V_APROX, 4.0);                // (115 + 31)

  // ---- 3. Llevarla hasta la base y colocarla encima ----
  girarARumbo(-90.0, V_GIRO_FINO);              // a la derecha de como arranco (der 87 a 30 rpm)
  // Se lleva BAJA (pala en 95): asi la camara ve la base por encima de la torre.
  avanzar(1450, V_RECTO_RAPIDO, 10.0);          // (1450) hasta donde todavia se ve la base
  visionCentrar("TORRE_DEST");                  // LA CAMARA corrige el rumbo mirando la base
  long falta = visionDistancia(2087 - 1450);    // 2087 = tramo completo medido en wwl5_opt
  recolectar(1);                                // 63: la sube por encima de la base
  avanzar(falta, V_APROX, 5.0);
  colocarSobreBase();                           // baja despacio, suelta, retrocede recto y sube la pala
  visionPausar();

  // ---- 4. Barrer ----
  girarARumbo(-115.0, V_GIRO);                  // (der 25)
  avanzar(227, V_RECTO, 4.0);                   // (227 a 207 rpm: la funcion vieja se pasaba
                                                //  al frenar; en wwl5_opt este tramo es 590)
  girarARumbo(-190.0, V_GIRO);                  // (der 75)
  recolectar(2);
  parabrisas(550, V_RECTO_MEDIO);               // (550)

  detener(1.0);

#if DEPURAR
  Serial.print(F("# FIN. Errores I2C: "));       Serial.print(gyro.errores());
  Serial.print(F(" | recuperaciones de bus: ")); Serial.print(gyro.recuperaciones());
  Serial.print(F(" | rumbo final: "));           Serial.println(rumboActual(), 2);
#endif

  while (1) tarea();     // no repetir la rutina
}
