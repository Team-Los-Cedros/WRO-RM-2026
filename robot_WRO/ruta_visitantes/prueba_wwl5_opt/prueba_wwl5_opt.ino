// ============================================================================
//  WRO Robomission Junior 2026 - Ruta visitantes
//  MegaPi (ATmega2560) + Makeblock Ultimate 2.0
//
//  VERSION OPTIMIZADA (reemplaza a prueba_wwl5_respaldo.ino)
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
//  IMPORTANTE - LEE ESTO ANTES DE PROBAR:
//  Los angulos de los giros que estan mas abajo son los MISMOS numeros que
//  tenias. Antes esos numeros llevaban descontada la pasada de frenado
//  (por eso ponias 85 para girar 90). Ahora el robot gira lo que le pides,
//  asi que casi todos hay que subirlos a su valor real (85 -> 90, 175 -> 180).
//  Sube el robot, corre calibrar_gyro.ino primero, y luego ajusta.
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
#define GYRO_ESCALA          1.000f // Factor de calibracion. Lo da calibrar_gyro.ino

// ---- Lazo de control ----
#define CTRL_PERIODO_US     5000UL  // Periodo del lazo de movimiento (5 ms = 200 Hz)
#define ENC_PERIODO_US     10000UL  // Cada cuanto se llama Encoder_x.loop()

// ---- Motores ----
#define RPM_MAX             185.0f  // rpm del eje de salida con PWM 255 (calibrar_gyro.ino)
#define PWM_TOPE               255

// ---- Giros ----
#define GIRO_PWM_MIN            45  // PWM minimo que todavia hace girar al robot
#define GIRO_K_DECEL         40.0f  // Rampa de frenado: pwm = K * raiz(grados que faltan)
#define GIRO_TOLERANCIA       0.8f  // Error aceptable, en grados
#define GIRO_CORRECCIONES        3  // Reintentos finos como maximo
#define GIRO_MS_ASENTAR        220  // Espera maxima a que el robot quede quieto
#define GIRO_MS_CORR           600  // Duracion maxima de cada correccion fina

// ---- Avance recto ----
#define RECTO_PWM_MIN           35
#define RECTO_K_DECEL        16.0f  // Rampa de frenado: pwm = K * raiz(grados que faltan)
#define RECTO_KP              4.0f  // Rumbo: pwm de correccion por grado de desvio
#define RECTO_KD              0.35f // Amortiguacion: pwm por (grado/segundo)
#define RECTO_TOLERANCIA        12  // Error aceptable, en grados de encoder
#define RECTO_CORREGIR           1  // 1 = corrige la distancia despues de frenar

// ---- Escaneo de linea ----
#define ESCANEO_PWM_MIN         42  // Por debajo de esto el robot no arranca

// ---- Seguridad ----
#define USAR_WATCHDOG            1  // 1 = si algo se cuelga, la placa se reinicia sola
#define QUIETO_DPS            4.0f  // |velocidad| por debajo de esto = robot detenido
#define QUIETO_MS               60  // ms seguidos quieto para darlo por bueno
#define DEPURAR                  1  // 1 = imprime por Serial lo pedido, lo real y los ms
                                    // de cada movimiento. Ponlo a 0 en competencia:
                                    // ahorra unos ms por maniobra.

// ---- Pines ----
#define BOTON_PIN        4
#define PIN_TCRT_IZQ    A4
#define PIN_TCRT_DER     2
#define PIN_SERVO_PALA   5
#define MPU_ADDR      0x68

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

// izq/der > 0 = esa rueda hacia adelante
static inline void pwmRuedas(int16_t izq, int16_t der)
{
  Encoder_1.setMotorPwm(constrain(izq, -PWM_TOPE, PWM_TOPE));
  Encoder_2.setMotorPwm(constrain(-der, -PWM_TOPE, PWM_TOPE));
}

// p > 0 = sentido "izquierda" del codigo original
static inline void pwmGiro(int16_t p)
{
  p = constrain(p, -PWM_TOPE, PWM_TOPE);
  Encoder_1.setMotorPwm(p);
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
  Serial.println(F("PARADA por boton"));
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

void girarGyro(float gradosObjetivo, float velocidad, int sentido, float timeoutSeg = 3.0f)
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

  // ---- Fase 1: crucero con rampa de frenado ----
  for (;;) {
    tarea();
    float falta = gradosObjetivo - fabs(gyro.grados() - h0);
    if (falta <= 0.0f) break;
    if (millis() - tIni > tMax) { porTimeout = true; break; }

    int16_t p = (int16_t)(GIRO_K_DECEL * sqrt(falta));
    p = constrain(p, (int16_t)GIRO_PWM_MIN, pwmCrucero);
    pwmGiro(sentido > 0 ? p : (int16_t)-p);

    esperarPeriodo(tCtrl);
  }

  frenar();
  esperarQuieto(GIRO_MS_ASENTAR);

  // ---- Fase 2: correcciones finas ----
  for (uint8_t n = 0; n < GIRO_CORRECCIONES && !porTimeout; n++) {
    tarea();
    float err = gradosObjetivo - fabs(gyro.grados() - h0);
    if (fabs(err) <= GIRO_TOLERANCIA) break;
    if (millis() - tIni > tMax) { porTimeout = true; break; }

    int8_t   dir     = (err > 0.0f) ? (int8_t)sentido : (int8_t)-sentido;
    int16_t  p       = GIRO_PWM_MIN;
    uint32_t tCorr   = millis();
    uint32_t tSubir  = millis();
    uint32_t tCtrl2  = micros();

    while (millis() - tCorr < GIRO_MS_CORR) {
      tarea();
      float e = gradosObjetivo - fabs(gyro.grados() - h0);
      if ((err > 0.0f && e <= 0.0f) || (err < 0.0f && e >= 0.0f)) break;

      // Si el robot no arranca por el roce, sube el PWM poco a poco.
      if (fabs(gyro.velocidad()) < 8.0f && millis() - tSubir > 40) {
        tSubir = millis();
        if (p < pwmCrucero) p += 3;
      }
      pwmGiro(dir > 0 ? p : (int16_t)-p);
      esperarPeriodo(tCtrl2);
    }
    frenar();
    esperarQuieto(GIRO_MS_ASENTAR);
  }

#if DEPURAR
  Serial.print(F("giro pedido="));  Serial.print(gradosObjetivo, 1);
  Serial.print(F(" real="));        Serial.print(fabs(gyro.grados() - h0), 2);
  Serial.print(F(" ms="));          Serial.print(millis() - tIni);
  Serial.print(F(" errI2C="));      Serial.print(gyro.errores());
  if (porTimeout) Serial.print(F("  <<< TIMEOUT"));
  Serial.println();
#endif
}

void girarIzquierdaGyro(float gradosObjetivo, float velocidad) { girarGyro(gradosObjetivo, velocidad, +1); }
void girarDerechaGyro  (float gradosObjetivo, float velocidad) { girarGyro(gradosObjetivo, velocidad, -1); }

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

  for (;;) {
    tarea();
    long avance = (labs(posGrados(Encoder_1) - p0i) + labs(posGrados(Encoder_2) - p0d)) / 2;
    long falta  = grados - avance;
    if (falta <= 0) break;
    if (millis() - tIni > tMax) break;

    int16_t base = (int16_t)(RECTO_K_DECEL * sqrt((float)falta));
    base = constrain(base, (int16_t)RECTO_PWM_MIN, pwmCrucero);
    aplicarRecto((int16_t)(base * sentido), h0);

    esperarPeriodo(tCtrl);
  }

  frenar();
  esperarQuieto(180);

#if RECTO_CORREGIR
  // ---- Correccion fina de distancia (mide lo que se paso al frenar) ----
  for (uint8_t n = 0; n < 2; n++) {
    tarea();
    long avance = (labs(posGrados(Encoder_1) - p0i) + labs(posGrados(Encoder_2) - p0d)) / 2;
    long err    = grados - avance;
    if (labs(err) <= RECTO_TOLERANCIA) break;
    if (millis() - tIni > tMax) break;

    int8_t   dir    = (err > 0) ? sentido : (int8_t)-sentido;
    int16_t  p      = RECTO_PWM_MIN;
    uint32_t tCorr  = millis();
    uint32_t tSubir = millis();
    uint32_t tCtrl2 = micros();

    while (millis() - tCorr < 400) {
      tarea();
      long av = (labs(posGrados(Encoder_1) - p0i) + labs(posGrados(Encoder_2) - p0d)) / 2;
      long e  = grados - av;
      if ((err > 0 && e <= 0) || (err < 0 && e >= 0)) break;

      if (millis() - tSubir > 40) { tSubir = millis(); if (p < 90) p += 4; }
      aplicarRecto((int16_t)(p * dir), h0);
      esperarPeriodo(tCtrl2);
    }
    frenar();
    esperarQuieto(120);
  }
#endif

#if DEPURAR
  long avanceFinal = (labs(posGrados(Encoder_1) - p0i) + labs(posGrados(Encoder_2) - p0d)) / 2;
  Serial.print(F("recto pedido=")); Serial.print(grados);
  Serial.print(F(" real="));        Serial.print(avanceFinal);
  Serial.print(F(" desvio="));      Serial.print(gyro.grados() - h0, 2);
  Serial.print(F(" ms="));          Serial.println(millis() - tIni);
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
  Serial.print(F("TCRT Izq (A4): "));
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
                                        float velocidadEscaneo = 10.0f,
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
//  12. SETUP
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
    Serial.println(F("ERROR: el MPU6050 no responde. Revisa el cable RJ25."));
  } else if (!gyro.calibrar(300)) {
    Serial.println(F("AVISO: calibracion de giro dudosa (el robot se movio?)."));
  }

  Serial.print(F("Sesgo giro Z = ")); Serial.println(gyro.sesgo(), 1);
  if (mcusr_copia & _BV(WDRF)) Serial.println(F("AVISO: el arranque anterior fue un reinicio por WATCHDOG."));

#if USAR_WATCHDOG
  wdt_enable(WDTO_1S);      // si algo bloquea mas de 1 s, la placa se reinicia
#endif
}

// ============================================================================
//  13. RUTINA DE COMPETENCIA
//
//  RECORDATORIO: los angulos de abajo son los que tenias, y llevaban
//  descontada la pasada del frenado. Ahora el robot gira exactamente lo que
//  le pides, asi que la mayoria hay que subirlos a su valor real.
//  El ultimo parametro de avanzar/retroceder/avanzarRectoGyro ya NO es una
//  espera fija: ahora es un TIEMPO LIMITE de seguridad. La funcion vuelve en
//  cuanto termina el movimiento, asi que puedes dejarlo holgado sin perder
//  tiempo.
// ============================================================================

void loop()
{
  Serial.println(F("Listo. Pulsa el boton."));

  while (digitalRead(BOTON_PIN) == HIGH) tarea();   // esperar pulsacion
  while (digitalRead(BOTON_PIN) == LOW)  tarea();   // esperar que se suelte

  _delay(0.5);

  // Recalibra el sesgo del giroscopio justo antes de arrancar: el robot lleva
  // un rato quieto, es el mejor momento, y asi corrige la deriva por temperatura.
  gyro.calibrar(150);

  rutinaIniciada = true;

  /* --- colores no aleatorios --- */
  girarDerechaGyro(85.0, 20.0);
  avanzarRectoGyro(202, 50, 1.5, 3.0);
  girarIzquierdaGyro(85.0, 20.0);

  avanzarRectoGyro(742, 50, 1.5, 7.0);
  recolectar(2);
  retroceder(400, 25, 5.0);

  girarIzquierdaGyro(84.0, 35.0);
  avanzarRectoGyro(1157, 150, 6.0, 5.0);
  _delay(0.3);
  avanzarRectoGyroLineaPerpendicular(730, 45, 10.0, 90, 4);
  _delay(0.3);

  girarIzquierdaGyro(175.0, 30.0);
  avanzarRectoGyro(205, 45, 6.0, 2.0);
  recolectar(1);

  // Separar el rojo
  retroceder(200, 25, 3.0);
  girarDerechaGyro(30.0, 40.0);
  avanzar(366, 40, 4.0);
  recolectar(2);
  girarDerechaGyro(50.0, 40.0);

  // Empujar a la zona
  avanzar(810, 125, 3.5);
  recolectar(1);
  avanzar(55, 90, 2.0);
  retroceder(750, 85, 4.0);

  // Voltear hacia el verde  (este era el giro que fallaba al cruzar el +-180)
  recolectar(2);
  girarIzquierdaGyro(160.0, 32.0);
  avanzar(505, 145, 3.0);

  // Acomodar el verde si queda fuera
  recolectar(3);
  servoGarra2.write(GARRA_CERRADA_S2);
  girarIzquierdaGyro(35.0, 40.0);
  abrirGarra();
  avanzar(45, 60, 2.0);

  // ---- TORRES AMARILLAS ----
  retroceder(100, 65, 2.0);
  retroceder(870, 220, 3.0);
  girarDerechaGyro(99.0, 30.0);
  _delay(0.3);
  girarDerechaGyro(5.0, 15.0);
  recolectar(1);

  // Ir y centrar en linea
  avanzarRectoGyro(160, 195, 3.5, 3.0);
  _delay(0.3);
  avanzarRectoGyroLineaPerpendicular(390, 35, 10.0, 40, 2, 30, 2, 0);

  // Ir a la AMARILLA
  avanzar(315, 78, 3.0);
  girarDerechaGyro(89.5, 30.0);
  _delay(0.3);

  // Recolectar
  bajar_pala();
  avanzar(250, 30, 4.0);
  cerrarGarra();
  recolectar(4);
  retroceder(110, 15, 3.5);
  abrirGarra();
  avanzar(45, 65, 1.5);
  cerrarGarra();

  // Ir a llevar la torre
  retroceder(31, 35, 2.0);
  girarDerechaGyro(87.0, 30.0);
  recolectar(1);
  avanzarRectoGyro(2027, 207, 22.5, 8.0);
  depositar();
  retroceder(110, 25, 4.0);

  girarDerechaGyro(25.0, 50.0);
  avanzarRectoGyro(590, 207, 22.5, 3.0);
  recolectar(2);
  girarDerechaGyro(75.0, 50.0);
  parabrisas(550);

  detener(1.0);

#if DEPURAR
  Serial.print(F("FIN. Errores I2C: "));       Serial.print(gyro.errores());
  Serial.print(F(" | recuperaciones de bus: ")); Serial.print(gyro.recuperaciones());
  Serial.print(F(" | rumbo final: "));         Serial.println(gyro.grados(), 2);
#endif

  while (1) tarea();     // no repetir la rutina
}
