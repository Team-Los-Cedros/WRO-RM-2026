// ============================================================================
//  calibrar_gyro.ino  -  Herramienta de calibracion para prueba_wwl5_opt.ino
//
//  Sirve para sacar los cuatro numeros que hacen falta en el sketch principal:
//     GYRO_ESCALA    factor de correccion del giroscopio
//     RPM_MAX        rpm del eje de salida con PWM 255
//     GIRO_PWM_MIN   PWM minimo que hace girar al robot
//     RECTO_PWM_MIN  PWM minimo que hace avanzar al robot
//  y ademas comprueba la salud del bus I2C (errores y tiempo de lectura).
//
//  USO: sube el sketch, abre el Monitor Serie a 115200 con final de linea
//       "Nueva linea" o "Sin ajuste de linea", y escribe una letra:
//
//     s  Salud del bus I2C: errores y microsegundos por lectura
//     b  Sesgo y ruido del giroscopio (deja el robot QUIETO)
//     e  Escala del giroscopio: gira el robot 360 grados A MANO
//     v  Velocidad maxima de los motores (rpm a PWM 255)
//     m  PWM minimo de arranque (giro y recto)
//     t  Prueba de repetibilidad: 5 giros de 90 grados
//     ?  Vuelve a mostrar este menu
// ============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <MeMegaPi.h>

#define I2C_HZ          400000UL
#define I2C_TIMEOUT_US    4000UL
#define MPU_ADDR            0x68
#define PWM_TOPE             255

MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);

// ---------------------------------------------------------------- giroscopio
float    gBias = 0.0f, gH = 0.0f, gW = 0.0f, gWprev = 0.0f;
uint32_t gTUlt = 0, gTMuestra = 0;
uint16_t gErr = 0;
uint32_t gLecturas = 0, gMicrosTotal = 0;

bool gyroEscribir(uint8_t reg, uint8_t valor)
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(valor);
  return (Wire.endTransmission(true) == 0);
}

bool gyroLeerZ(int16_t &destino)
{
  uint32_t t0 = micros();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write((uint8_t)0x47);
  if (Wire.endTransmission(false) != 0) { Wire.clearWireTimeoutFlag(); gErr++; return false; }
  if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)2, (uint8_t)true) != 2) {
    Wire.clearWireTimeoutFlag(); gErr++; return false;
  }
  uint8_t alto = Wire.read();
  uint8_t bajo = Wire.read();
  destino = (int16_t)(((uint16_t)alto << 8) | bajo);
  gMicrosTotal += (micros() - t0);
  gLecturas++;
  return true;
}

bool gyroBegin(void)
{
  Wire.begin();
  Wire.setClock(I2C_HZ);
  Wire.setWireTimeout(I2C_TIMEOUT_US, true);

  gyroEscribir(0x6B, 0x80);
  delay(120);
  Wire.setClock(I2C_HZ);
  if (!gyroEscribir(0x6B, 0x01)) return false;
  delay(20);
  gyroEscribir(0x1A, 0x02);
  gyroEscribir(0x19, 0x00);
  gyroEscribir(0x1B, 0x08);
  delay(20);

  int16_t v;
  bool ok = gyroLeerZ(v);
  gTUlt = gTMuestra = micros();
  return ok;
}

void gyroCalibrar(uint16_t muestras)
{
  long suma = 0;
  uint16_t buenas = 0;
  for (uint16_t i = 0; i < muestras; i++) {
    int16_t v;
    if (gyroLeerZ(v)) { suma += v; buenas++; }
    delay(2);
  }
  if (buenas) gBias = (float)suma / (float)buenas;
  gH = gW = gWprev = 0.0f;
  gTUlt = gTMuestra = micros();
}

void gyroUpdate(void)
{
  uint32_t ahora = micros();
  if (ahora - gTMuestra < 2000UL) return;
  gTMuestra = ahora;

  int16_t crudo;
  if (!gyroLeerZ(crudo)) return;

  float dt = (float)(ahora - gTUlt) * 1.0e-6f;
  gTUlt = ahora;
  if (dt > 0.05f) dt = 0.05f;

  gWprev = gW;
  gW = ((float)crudo - gBias) / 65.5f;
  gH += 0.5f * (gW + gWprev) * dt;
}

// ------------------------------------------------------------------- motores
void isr_encoder1(void) { if (PINC & _BV(PC6)) Encoder_1.pulsePosPlus(); else Encoder_1.pulsePosMinus(); }
void isr_encoder2(void) { if (PIND & _BV(PD7)) Encoder_2.pulsePosPlus(); else Encoder_2.pulsePosMinus(); }

void modoDirecto(void)
{
  Encoder_1.setMotionMode(DIRECT_MODE);
  Encoder_2.setMotionMode(DIRECT_MODE);
}
void pwmGiro(int16_t p)
{
  p = constrain(p, -PWM_TOPE, PWM_TOPE);
  Encoder_1.setMotorPwm(p);
  Encoder_2.setMotorPwm(p);
}
void pwmRuedas(int16_t izq, int16_t der)
{
  Encoder_1.setMotorPwm(constrain(izq, -PWM_TOPE, PWM_TOPE));
  Encoder_2.setMotorPwm(constrain(-der, -PWM_TOPE, PWM_TOPE));
}
void frenar(void) { modoDirecto(); Encoder_1.setMotorPwm(0); Encoder_2.setMotorPwm(0); }

void tarea(void)
{
  gyroUpdate();
  static uint32_t tEnc = 0;
  uint32_t ahora = micros();
  if (ahora - tEnc >= 10000UL) { tEnc = ahora; Encoder_1.loop(); Encoder_2.loop(); }
}

void esperar(uint32_t ms)
{
  uint32_t t = millis();
  while (millis() - t < ms) tarea();
}

// ============================================================================
//  PRUEBAS
// ============================================================================

void pruebaSalud(void)
{
  Serial.println(F("\n--- Salud del bus I2C (5 s) ---"));
  gErr = 0; gLecturas = 0; gMicrosTotal = 0;
  uint32_t t = millis();
  while (millis() - t < 5000UL) gyroUpdate();

  Serial.print(F("Lecturas correctas : ")); Serial.println(gLecturas);
  Serial.print(F("Errores            : ")); Serial.println(gErr);
  Serial.print(F("Microsegundos/lect : "));
  Serial.println(gLecturas ? (gMicrosTotal / gLecturas) : 0);
  if (gErr == 0) Serial.println(F("OK: el bus va fino a 400 kHz."));
  else           Serial.println(F("AVISO: baja I2C_HZ a 200000 y revisa el cable RJ25."));
}

void pruebaSesgo(void)
{
  Serial.println(F("\n--- Sesgo y ruido (deja el robot QUIETO 6 s) ---"));
  esperar(500);

  long suma = 0;
  int16_t minimo = 32767, maximo = -32768;
  uint16_t n = 0;
  uint32_t t = millis();
  while (millis() - t < 3000UL) {
    int16_t v;
    if (gyroLeerZ(v)) { suma += v; if (v < minimo) minimo = v; if (v > maximo) maximo = v; n++; }
    delay(2);
  }
  if (!n) { Serial.println(F("Sin lecturas.")); return; }

  float bias = (float)suma / (float)n;
  Serial.print(F("Sesgo crudo   : ")); Serial.println(bias, 2);
  Serial.print(F("Ruido pico-pico: "));
  Serial.print(((float)maximo - (float)minimo) / 65.5f, 2);
  Serial.println(F(" grados/s"));

  // deriva con ese sesgo aplicado
  gBias = bias;
  gH = 0.0f; gW = 0.0f; gWprev = 0.0f;
  gTUlt = gTMuestra = micros();
  t = millis();
  while (millis() - t < 3000UL) gyroUpdate();
  Serial.print(F("Deriva en 3 s : ")); Serial.print(gH, 3); Serial.println(F(" grados"));
  Serial.print(F("Deriva/minuto : ")); Serial.print(gH * 20.0f, 2); Serial.println(F(" grados"));
}

void pruebaEscala(void)
{
  Serial.println(F("\n--- Escala del giroscopio ---"));
  Serial.println(F("1. Marca la posicion del robot en el suelo."));
  Serial.println(F("2. Pulsa ENTER, gira el robot A MANO 360 grados exactos"));
  Serial.println(F("   (mejor 3 vueltas = 1080 grados, sale mas preciso)."));
  Serial.println(F("3. Vuelve a pulsar ENTER."));
  Serial.print  (F("Calibrando sesgo... "));
  gyroCalibrar(300);
  Serial.println(F("listo. Pulsa ENTER para empezar."));

  while (!Serial.available()) tarea();
  while (Serial.available()) Serial.read();
  gH = 0.0f; gW = 0.0f; gWprev = 0.0f;
  gTUlt = gTMuestra = micros();
  Serial.println(F("GIRANDO... pulsa ENTER al terminar."));

  while (!Serial.available()) tarea();
  while (Serial.available()) Serial.read();

  Serial.print(F("El giroscopio midio: ")); Serial.print(gH, 2); Serial.println(F(" grados"));
  if (fabs(gH) > 30.0f) {
    Serial.print(F("Si giraste 360  -> GYRO_ESCALA = ")); Serial.println(360.0f  / fabs(gH), 4);
    Serial.print(F("Si giraste 720  -> GYRO_ESCALA = ")); Serial.println(720.0f  / fabs(gH), 4);
    Serial.print(F("Si giraste 1080 -> GYRO_ESCALA = ")); Serial.println(1080.0f / fabs(gH), 4);
  }
}

void pruebaVelocidad(void)
{
  Serial.println(F("\n--- Velocidad maxima (levanta las ruedas del suelo) ---"));
  Serial.println(F("Arranca en 3 s..."));
  esperar(3000);

  modoDirecto();
  pwmRuedas(255, 255);
  esperar(2500);                    // dejar que se estabilice

  float sumaIzq = 0, sumaDer = 0;
  uint16_t n = 0;
  uint32_t t = millis();
  while (millis() - t < 1500UL) {
    tarea();
    sumaIzq += fabs(Encoder_1.getCurrentSpeed());
    sumaDer += fabs(Encoder_2.getCurrentSpeed());
    n++;
    delay(10);
  }
  frenar();

  float vIzq = sumaIzq / n;
  float vDer = sumaDer / n;
  Serial.print(F("Motor izquierdo (SLOT1): ")); Serial.print(vIzq, 1); Serial.println(F(" rpm"));
  Serial.print(F("Motor derecho   (SLOT2): ")); Serial.print(vDer, 1); Serial.println(F(" rpm"));
  Serial.print(F("Pon RPM_MAX = ")); Serial.println(min(vIzq, vDer), 0);
  float dif = fabs(vIzq - vDer) / max(vIzq, vDer) * 100.0f;
  Serial.print(F("Diferencia entre motores: ")); Serial.print(dif, 1); Serial.println(F(" %"));
  if (dif > 8.0f) Serial.println(F("AVISO: mas de un 8 % de diferencia. Revisa mecanica o driver."));
}

void pruebaPwmMinimo(void)
{
  Serial.println(F("\n--- PWM minimo de arranque (robot EN EL SUELO) ---"));
  esperar(1500);

  // giro
  gyroCalibrar(200);
  modoDirecto();
  int16_t pGiro = 0;
  for (int16_t p = 15; p <= 120; p += 2) {
    pwmGiro(p);
    uint32_t t = millis();
    float pico = 0;
    while (millis() - t < 260UL) { tarea(); if (fabs(gW) > pico) pico = fabs(gW); }
    if (pico > 20.0f) { pGiro = p; break; }
  }
  frenar();
  esperar(600);

  // recto
  long p0i = Encoder_1.getPulsePos();
  int16_t pRecto = 0;
  for (int16_t p = 15; p <= 120; p += 2) {
    long antes = Encoder_1.getPulsePos();
    pwmRuedas(p, p);
    esperar(260);
    if (labs(Encoder_1.getPulsePos() - antes) > 25) { pRecto = p; break; }
  }
  frenar();
  (void)p0i;

  Serial.print(F("GIRO_PWM_MIN  sugerido = ")); Serial.println(pGiro  ? pGiro  + 8 : 45);
  Serial.print(F("RECTO_PWM_MIN sugerido = ")); Serial.println(pRecto ? pRecto + 6 : 35);
  Serial.println(F("(se suma un margen para que arranque siempre)"));
}

void pruebaRepetibilidad(void)
{
  Serial.println(F("\n--- 5 giros de 90 grados (robot EN EL SUELO) ---"));
  Serial.println(F("Marca la posicion inicial. Arranca en 3 s..."));
  esperar(3000);
  gyroCalibrar(250);

  for (uint8_t i = 0; i < 5; i++) {
    float h0 = gH;
    modoDirecto();
    uint32_t t = millis();
    while (millis() - t < 4000UL) {
      tarea();
      float falta = 90.0f - fabs(gH - h0);
      if (falta <= 0.0f) break;
      int16_t p = (int16_t)(40.0f * sqrt(falta));
      p = constrain(p, (int16_t)45, (int16_t)200);
      pwmGiro(p);
      delay(5);
    }
    frenar();
    esperar(700);                       // dejar que se asiente y medir la pasada
    Serial.print(F("Giro ")); Serial.print(i + 1);
    Serial.print(F(": ")); Serial.print(fabs(gH - h0), 2); Serial.println(F(" grados"));
    esperar(800);
  }
  Serial.print(F("Rumbo acumulado tras 5 giros (deberia ser ~450): "));
  Serial.println(fabs(gH), 2);
}

void menu(void)
{
  Serial.println(F("\n================ CALIBRACION MEGAPI ================"));
  Serial.println(F(" s  Salud del bus I2C"));
  Serial.println(F(" b  Sesgo y ruido del giroscopio (robot quieto)"));
  Serial.println(F(" e  Escala del giroscopio (girar 360 a mano)"));
  Serial.println(F(" v  Velocidad maxima (ruedas al aire)"));
  Serial.println(F(" m  PWM minimo de arranque (robot en el suelo)"));
  Serial.println(F(" t  5 giros de 90 grados (robot en el suelo)"));
  Serial.println(F(" ?  Repetir este menu"));
  Serial.println(F("===================================================="));
}

void setup()
{
  Serial.begin(115200);
  while (!Serial) { ; }

  pinMode(4, INPUT_PULLUP);

  TCCR1A = _BV(WGM10);               TCCR1B = _BV(WGM12) | _BV(CS11);
  TCCR2A = _BV(WGM21) | _BV(WGM20);  TCCR2B = _BV(CS21);
  TCCR4A = _BV(WGM40);               TCCR4B = _BV(WGM42) | _BV(CS41);

  attachInterrupt(digitalPinToInterrupt(Encoder_1.getPortA()), isr_encoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(Encoder_2.getPortA()), isr_encoder2, RISING);
  Encoder_1.setPulse(8); Encoder_2.setPulse(8);
  Encoder_1.setRatio(46); Encoder_2.setRatio(46);
  modoDirecto();
  frenar();

  if (!gyroBegin()) Serial.println(F("ERROR: el MPU6050 no responde."));
  gyroCalibrar(250);

  menu();
}

void loop()
{
  tarea();
  if (!Serial.available()) return;

  char c = Serial.read();
  while (Serial.available()) Serial.read();     // vaciar el resto de la linea

  switch (c) {
    case 's': pruebaSalud();          break;
    case 'b': pruebaSesgo();          break;
    case 'e': pruebaEscala();         break;
    case 'v': pruebaVelocidad();      break;
    case 'm': pruebaPwmMinimo();      break;
    case 't': pruebaRepetibilidad();  break;
    case '?': menu();                 break;
    default:  return;
  }
  Serial.println(F("\nListo. Escribe otra letra (? para el menu)."));
}
