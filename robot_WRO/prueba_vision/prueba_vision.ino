// =========================================================================
// WRO RoboMission Junior 2026 - 4 artefactos con MegaPi + Raspberry Pi 3B
//
// Estrategia:
//   1. El robot inicia centrado frente a los cuatro artefactos.
//   2. Se desplaza en L hasta cada slot, reconoce el color y afina la pose.
//   3. Ejecuta la secuencia mecanica probada en prueba_centrales.ino.
//   4. Vuelve al centro, viaja al museo y descarga en ROJO, VERDE, NEGRO,
//      AZUL o AMARILLO sin desplazarse lateralmente junto a los ya puestos.
//   5. Regresa por la misma ruta y repite hasta cuatro veces o hasta 116 s.
//
// IMPORTANTE: los parametros marcados CALIBRAR son puntos de partida. Antes
// de una ronda oficial hay que medir PASO_SLOT_GRADOS, PASO_MUSEO_GRADOS y
// las dos distancias de ruta sobre la pista concreta.
// =========================================================================

#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <MeMegaPi.h>

#define BOTON_PIN 4
#define PIN_TCRT_IZQ A4
#define PIN_TCRT_DER A3

#define SERIAL_VISION Serial
#define BAUD_VISION 115200

MeGyro giroscopio(8);
MePort placaExpansora(7);
MeEncoderOnBoard Encoder_1(SLOT1);  // izquierdo
MeEncoderOnBoard Encoder_2(SLOT2);  // derecho
Servo servoGarra1;
Servo servoGarra2;
Servo miServo;

const int PIN_SERVO_PALA = 5;

bool rutinaIniciada = false;
bool falloNavegacion = false;
unsigned long inicioRutinaMs = 0;

void _loop();
void _delay(float segundos);
void motoresParar();

void isr_process_encoder1(void) {
  if (digitalRead(Encoder_1.getPortB()) == 0) Encoder_1.pulsePosMinus();
  else Encoder_1.pulsePosPlus();
}

void isr_process_encoder2(void) {
  if (digitalRead(Encoder_2.getPortB()) == 0) Encoder_2.pulsePosMinus();
  else Encoder_2.pulsePosPlus();
}

// =========================================================================
// PROTOCOLO DE VISION
// =========================================================================
// Pi -> MegaPi:
//   T <found> <color> <ex> <ey> <area> <dist> <fps> <confianza>
// MegaPi -> Pi:
//   C AUTO   reconoce el artefacto alineado
//   C ROJO   (o cualquier color) sigue solo ese artefacto
//   S 0|1    detiene/activa el envio continuo

enum ColorObjeto {
  COLOR_NINGUNO = -1,
  COLOR_ROJO = 0,
  COLOR_VERDE = 1,
  COLOR_NEGRO = 2,
  COLOR_AZUL = 3,
  COLOR_AMARILLO = 4
};

// Evita que el preprocesador de Arduino genere estos prototipos antes del
// enum (lo que haria que ColorObjeto todavia no existiera al compilar).
ColorObjeto colorDesdeTexto(const char* texto);
void logColor(const char* prefijo, ColorObjeto color);
void visionPedirColor(ColorObjeto color);
void visionPausar();
void visionPedirDestino(ColorObjeto color);
bool visionEscanearFila(float timeoutSeg);
ColorObjeto visionIdentificarColor(float timeoutSeg);
bool prepararCapturaPorVision(ColorObjeto color);
bool irSlotAMuseoYDepositar(byte slot, ColorObjeto color);
long offsetDestino(ColorObjeto color);

const char* const NOMBRES_COLOR[5] = {
  "ROJO", "VERDE", "NEGRO", "AZUL", "AMARILLO"
};

struct VisionData {
  bool encontrado;
  ColorObjeto color;
  int ex;
  int ey;
  long area;
  int dist;
  int fps;
  int confianza;
  unsigned long tRecepcion;
  unsigned long secuencia;
};

VisionData vision = {
  false, COLOR_NINGUNO, 0, 0, 0, -1, 0, 0, 0, 0
};

char bufVision[128];
byte idxVision = 0;

ColorObjeto colorEnSlot[4] = {
  COLOR_NINGUNO, COLOR_NINGUNO, COLOR_NINGUNO, COLOR_NINGUNO
};
bool colorCompletado[5] = {false, false, false, false, false};
ColorObjeto filaRecibida[4] = {
  COLOR_NINGUNO, COLOR_NINGUNO, COLOR_NINGUNO, COLOR_NINGUNO
};
byte cantidadFilaRecibida = 0;
unsigned long secuenciaFila = 0;

ColorObjeto colorDesdeTexto(const char* texto) {
  if (!texto) return COLOR_NINGUNO;
  for (int i = 0; i < 5; i++) {
    if (strcmp(texto, NOMBRES_COLOR[i]) == 0) return (ColorObjeto)i;
  }
  return COLOR_NINGUNO;
}

void logPi(const char* msg) {
  SERIAL_VISION.print('#');
  SERIAL_VISION.println(msg);
}

void logColor(const char* prefijo, ColorObjeto color) {
  SERIAL_VISION.print('#');
  SERIAL_VISION.print(prefijo);
  if (color >= COLOR_ROJO && color <= COLOR_AMARILLO) {
    SERIAL_VISION.println(NOMBRES_COLOR[(int)color]);
  } else {
    SERIAL_VISION.println("NINGUNO");
  }
}

void visionProcesarLinea(char* linea) {
  if (linea[0] == 'F' && linea[1] == ' ') {
    char* tok = strtok(linea + 2, " ");
    if (!tok) return;
    int cantidad = atoi(tok);
    if (cantidad < 0 || cantidad > 4) return;
    for (int i = 0; i < 4; i++) filaRecibida[i] = COLOR_NINGUNO;
    for (int i = 0; i < cantidad; i++) {
      tok = strtok(NULL, " ");
      if (!tok) return;
      filaRecibida[i] = colorDesdeTexto(tok);
    }
    cantidadFilaRecibida = cantidad;
    secuenciaFila++;
    return;
  }

  if (linea[0] != 'T' || linea[1] != ' ') return;

  char* tok = strtok(linea + 2, " ");
  if (!tok) return;
  bool encontrado = atoi(tok) != 0;

  tok = strtok(NULL, " ");
  if (!tok) return;
  ColorObjeto color = colorDesdeTexto(tok);

  tok = strtok(NULL, " "); if (!tok) return; int ex = atoi(tok);
  tok = strtok(NULL, " "); if (!tok) return; int ey = atoi(tok);
  tok = strtok(NULL, " "); if (!tok) return; long area = atol(tok);
  tok = strtok(NULL, " "); if (!tok) return; int dist = atoi(tok);
  tok = strtok(NULL, " "); int fps = tok ? atoi(tok) : 0;
  tok = strtok(NULL, " "); int confianza = tok ? atoi(tok) : 0;

  vision.encontrado = encontrado;
  vision.color = encontrado ? color : COLOR_NINGUNO;
  vision.ex = ex;
  vision.ey = ey;
  vision.area = area;
  vision.dist = dist;
  vision.fps = fps;
  vision.confianza = confianza;
  vision.tRecepcion = millis();
  vision.secuencia++;
}

void visionActualizar() {
  while (SERIAL_VISION.available()) {
    char c = SERIAL_VISION.read();
    if (c == '\n' || c == '\r') {
      if (idxVision > 0) {
        bufVision[idxVision] = '\0';
        visionProcesarLinea(bufVision);
        idxVision = 0;
      }
    } else if (idxVision < sizeof(bufVision) - 1) {
      bufVision[idxVision++] = c;
    } else {
      idxVision = 0;
    }
  }
}

bool visionViva(unsigned long msMax = 350) {
  return vision.tRecepcion != 0 && millis() - vision.tRecepcion < msMax;
}

bool visionVeObjeto(unsigned long msMax = 350) {
  return visionViva(msMax) && vision.encontrado;
}

void visionPedirModo(const char* modo) {
  SERIAL_VISION.print("M ARTEFACTO ");
  SERIAL_VISION.println(modo);
  vision.encontrado = false;
  vision.color = COLOR_NINGUNO;
  vision.tRecepcion = 0;
}

void visionPausar() {
  SERIAL_VISION.println("M PAUSA");
  vision.encontrado = false;
  vision.color = COLOR_NINGUNO;
  vision.tRecepcion = 0;
}

void visionPedirDestino(ColorObjeto color) {
  if (color < COLOR_ROJO || color > COLOR_AMARILLO) return;
  SERIAL_VISION.print("M DESTINO ");
  SERIAL_VISION.println(NOMBRES_COLOR[(int)color]);
  vision.encontrado = false;
  vision.color = COLOR_NINGUNO;
  vision.tRecepcion = 0;
}

bool visionEscanearFila(float timeoutSeg) {
  ColorObjeto anterior[4] = {
    COLOR_NINGUNO, COLOR_NINGUNO, COLOR_NINGUNO, COLOR_NINGUNO
  };
  int estables = 0;
  unsigned long ultimaSecuencia = secuenciaFila;
  unsigned long inicio = millis();
  unsigned long timeoutMs = (unsigned long)(timeoutSeg * 1000.0);

  SERIAL_VISION.println("M FILA");
  while (millis() - inicio < timeoutMs) {
    SERIAL_VISION.println("F");
    unsigned long espera = millis();
    while (secuenciaFila == ultimaSecuencia && millis() - espera < 220) {
      _loop();
      delay(2);
    }
    if (secuenciaFila == ultimaSecuencia) continue;
    ultimaSecuencia = secuenciaFila;
    if (cantidadFilaRecibida != 4) {
      estables = 0;
      continue;
    }

    bool valida = true;
    bool iguales = true;
    for (int i = 0; i < 4; i++) {
      if (filaRecibida[i] < COLOR_ROJO || filaRecibida[i] > COLOR_AMARILLO) valida = false;
      if (filaRecibida[i] != anterior[i]) iguales = false;
      for (int j = 0; j < i; j++) {
        if (filaRecibida[i] == filaRecibida[j]) valida = false;
      }
    }
    if (!valida) {
      estables = 0;
      continue;
    }
    estables = iguales ? estables + 1 : 1;
    for (int i = 0; i < 4; i++) anterior[i] = filaRecibida[i];
    if (estables >= 3) {
      for (int i = 0; i < 4; i++) colorEnSlot[i] = anterior[i];
      visionPausar();
      return true;
    }
  }
  visionPausar();
  return false;
}

void visionPedirColor(ColorObjeto color) {
  if (color >= COLOR_ROJO && color <= COLOR_AMARILLO) {
    visionPedirModo(NOMBRES_COLOR[(int)color]);
  }
}

// Vota durante varios frames. El area da peso adicional para que los detalles
// amarillos del artefacto verde no ganen a la estructura verde completa.
ColorObjeto visionIdentificarColor(float timeoutSeg) {
  const int CONFIANZA_MIN = 45;
  const int EX_MAX = 110;
  int votos[5] = {0, 0, 0, 0, 0};
  int framesValidos = 0;
  unsigned long ultimaSecuencia = vision.secuencia;
  unsigned long inicio = millis();
  unsigned long timeoutMs = (unsigned long)(timeoutSeg * 1000.0);

  visionPedirModo("AUTO");

  while (millis() - inicio < timeoutMs) {
    _loop();
    if (vision.secuencia == ultimaSecuencia) {
      delay(2);
      continue;
    }
    ultimaSecuencia = vision.secuencia;

    if (!visionVeObjeto() || vision.color < COLOR_ROJO ||
        vision.color > COLOR_AMARILLO ||
        abs(vision.ex) > EX_MAX || vision.confianza < CONFIANZA_MIN) {
      continue;
    }

    int idx = (int)vision.color;
    if (colorCompletado[idx]) continue;

    int peso = 1;
    if (vision.confianza >= 70) peso++;
    if (vision.area >= 600) peso++;
    if (vision.area >= 1800) peso++;
    votos[idx] += peso;
    framesValidos++;
  }

  int mejor = -1;
  int segundo = -1;
  for (int i = 0; i < 5; i++) {
    if (mejor < 0 || votos[i] > votos[mejor]) {
      segundo = mejor;
      mejor = i;
    } else if (segundo < 0 || votos[i] > votos[segundo]) {
      segundo = i;
    }
  }

  int votosSegundo = segundo >= 0 ? votos[segundo] : 0;
  if (framesValidos >= 5 && mejor >= 0 && votos[mejor] >= 8 &&
      votos[mejor] >= votosSegundo + 3) {
    return (ColorObjeto)mejor;
  }

  logPi("color no estable");
  return COLOR_NINGUNO;
}

// =========================================================================
// BUCLE DE FONDO Y PARADA
// =========================================================================

void paradaEmergencia() {
  motoresParar();
  Encoder_1.move(0, 0);
  Encoder_2.move(0, 0);
  Encoder_1.setTarPWM(0);
  Encoder_2.setTarPWM(0);
  SERIAL_VISION.println("S 0");

  unsigned long inicio = millis();
  while (true) {
    if (millis() - inicio < 1200) {
      Encoder_1.loop();
      Encoder_2.loop();
    } else {
      delay(250);
    }
  }
}

void _loop() {
  unsigned long ahora = millis();
  static unsigned long ultimoMotores = 0;
  static unsigned long ultimoGyro = 0;

  if (ahora - ultimoMotores >= 10) {
    Encoder_1.loop();
    Encoder_2.loop();
    ultimoMotores = ahora;
  }
  if (ahora - ultimoGyro >= 20) {
    giroscopio.update();
    ultimoGyro = ahora;
  }
  visionActualizar();

  if (rutinaIniciada && digitalRead(BOTON_PIN) == LOW) {
    paradaEmergencia();
  }
}

void _delay(float segundos) {
  if (segundos < 0.0) segundos = 0.0;
  unsigned long fin = millis() + (unsigned long)(segundos * 1000.0);
  while ((long)(fin - millis()) > 0) {
    _loop();
    delay(2);
  }
}

// =========================================================================
// MOVIMIENTO
// =========================================================================

float limitar(float v, float minimo, float maximo) {
  if (v < minimo) return minimo;
  if (v > maximo) return maximo;
  return v;
}

void motoresTanque(float velIzq, float velDer) {
  Encoder_1.runSpeed(velIzq);
  Encoder_2.runSpeed(-velDer);
}

void motoresParar() {
  Encoder_1.runSpeed(0);
  Encoder_2.runSpeed(0);
}

void detener(float segundos) {
  motoresParar();
  _delay(segundos);
}

// Movimiento de posicion usado solo donde hay una secuencia mecanica ya
// probada. +/+ significa adelante para ambos lados.
void moverRobot(long gradosIzq, long gradosDer, float velocidad, float espera) {
  Encoder_1.move(gradosIzq, abs(velocidad));
  Encoder_2.move(-gradosDer, abs(velocidad * 1.02));
  _delay(espera);
}

void avanzar(long grados, float velocidad, float espera) {
  moverRobot(grados, grados, velocidad, espera);
}

void retroceder(long grados, float velocidad, float espera) {
  moverRobot(-grados, -grados, velocidad, espera);
}

// Para rutas largas usa encoder como distancia y gyro como rumbo. El signo
// de grados define el sentido: positivo adelante, negativo atras.
bool moverRectoGyro(long grados, float velocidad, float timeoutSeg = 8.0) {
  if (grados == 0) return true;

  giroscopio.update();
  float rumbo = giroscopio.getAngleZ();
  long inicioIzq = Encoder_1.getCurPos();
  long inicioDer = Encoder_2.getCurPos();
  long objetivo = abs(grados);
  float signo = grados > 0 ? 1.0 : -1.0;
  unsigned long inicio = millis();
  unsigned long timeoutMs = (unsigned long)(timeoutSeg * 1000.0);

  while (millis() - inicio < timeoutMs) {
    _loop();
    long avanceIzq = labs(Encoder_1.getCurPos() - inicioIzq);
    long avanceDer = labs(Encoder_2.getCurPos() - inicioDer);
    long avance = (avanceIzq + avanceDer) / 2;
    if (avance >= objetivo) {
      motoresParar();
      _delay(0.12);
      return true;
    }

    long restante = objetivo - avance;
    float base = abs(velocidad);
    if (restante < 80) base = limitar(base * restante / 80.0, 24.0, base);
    base *= signo;

    float error = giroscopio.getAngleZ() - rumbo;
    float correccion = limitar(error * 1.5, -18.0, 18.0);
    if (signo < 0) correccion = -correccion;
    motoresTanque(base - correccion, base + correccion);
    delay(3);
  }

  motoresParar();
  logPi("TIMEOUT moverRectoGyro");
  return false;
}

// sentido: +1 derecha, -1 izquierda.
bool girarGyro(int sentido, float gradosObjetivo, float velocidadMax) {
  giroscopio.update();
  float anguloAnterior = giroscopio.getAngleZ();
  float girado = 0.0;
  unsigned long inicio = millis();

  while (millis() - inicio < 5000) {
    _loop();
    // Integra cambios cortos para atravesar correctamente el salto
    // +180/-180 que entrega el giroscopio.
    float anguloActual = giroscopio.getAngleZ();
    float delta = anguloActual - anguloAnterior;
    if (delta > 180.0) delta -= 360.0;
    if (delta < -180.0) delta += 360.0;
    girado += abs(delta);
    anguloAnterior = anguloActual;
    float restante = gradosObjetivo - girado;
    if (restante <= 0.0) {
      motoresParar();
      _delay(0.18);
      return true;
    }

    float v = abs(velocidadMax);
    if (restante < 20.0) v = limitar(v * restante / 20.0, 13.0, v);
    if (sentido > 0) motoresTanque(-v, v);  // derecha
    else motoresTanque(v, -v);              // izquierda
    delay(3);
  }

  motoresParar();
  logPi("TIMEOUT girarGyro");
  return false;
}

bool girarDerechaGyro(float grados, float velocidad) {
  return girarGyro(+1, grados, velocidad);
}

bool girarIzquierdaGyro(float grados, float velocidad) {
  return girarGyro(-1, grados, velocidad);
}

// =========================================================================
// GARRA Y PALA - ANGULOS PROBADOS
// =========================================================================

const int GARRA_ABIERTA_S1 = 0;
const int GARRA_CERRADA_S1 = 108;
const int GARRA_CERRADA_S2 = 58;
const int GARRA_ABIERTA_S2 = 180;

const int PALA_INICIAL = 0;
const int PALA_BAJAR = 105;          // Posicion normal de vision y transporte (despega 6° del suelo)
const int PALA_MODO_1 = 63;
const int PALA_ASENTAR = 120;        // Antiguo recolectar(2): maxima inclinacion para asentar objeto al depositar
const int PALA_RECOGER = 111;        // Antiguo recolectar(3): ras de suelo para embocar pala bajo el objeto
const int PALA_POSICIONAR = 50;
const int PALA_DEPOSITAR = 40;
const int PALA_BARRER = 117;         // Modo barrido frontal para empujar al museo

void abrirGarra() {
  servoGarra1.write(GARRA_ABIERTA_S1);
  servoGarra2.write(GARRA_ABIERTA_S2);
  _delay(0.75);
}

void cerrarGarra() {
  servoGarra1.write(GARRA_CERRADA_S1);
  servoGarra2.write(GARRA_CERRADA_S2);
  _delay(0.75);
}

void bajarPala() {
  miServo.write(PALA_BAJAR);
  _delay(0.75);
}

void bajar_pala() {
  bajarPala();
}

void recolectar(int modo) {
  if (modo == 1) miServo.write(PALA_MODO_1);
  else if (modo == 2) miServo.write(PALA_ASENTAR);
  else miServo.write(PALA_RECOGER);
  _delay(0.75);
}

void posicionar() {
  miServo.write(PALA_POSICIONAR);
  _delay(0.75);
}

void barrer() {
  abrirGarra();
  miServo.write(PALA_BARRER);
  _delay(0.75);
}

// =========================================================================
// CENTRADO Y APROXIMACION POR VISION (640x360)
// =========================================================================

const int VIS_TOLERANCIA_PX = 8;
const float VIS_KP_GIRO = 0.45;
const float VIS_VEL_GIRO_MIN = 18;
const float VIS_VEL_GIRO_MAX = 45;
const float VIS_KP_AVANCE = 0.22;
const int VIS_DIST_PRECAPTURA_MM = 75; // Distancia donde las paletas MG-90 abrazan de punta el artefacto
const int VIS_Y_PRECAPTURA = 338;      // En 640x360, y=338 corresponde a ~70 mm
const long VIS_GRADOS_CIEGOS = 90;     // ~5 cm de avance si entra al punto ciego inferior

float velocidadGiroVision(int ex) {
  float v = limitar(VIS_KP_GIRO * abs(ex), VIS_VEL_GIRO_MIN, VIS_VEL_GIRO_MAX);
  return ex >= 0 ? v : -v;
}

bool visionCentrar(float timeoutSeg = 3.5) {
  unsigned long inicio = millis();
  unsigned long timeoutMs = (unsigned long)(timeoutSeg * 1000.0);
  int consecutivos = 0;

  while (millis() - inicio < timeoutMs) {
    _loop();
    if (!visionVeObjeto()) {
      motoresParar();
      delay(3);
      continue;
    }

    if (abs(vision.ex) <= VIS_TOLERANCIA_PX) {
      motoresParar();
      if (++consecutivos >= 4) {
        _delay(0.12);
        return true;
      }
    } else {
      consecutivos = 0;
      float v = velocidadGiroVision(vision.ex);
      motoresTanque(-v, v);
    }
    delay(3);
  }

  motoresParar();
  logPi("TIMEOUT visionCentrar");
  return false;
}

bool visionBuscar(float gradosBarrido = 35.0, float velocidad = 24.0) {
  for (int fase = 0; fase < 3; fase++) {
    int sentido = fase == 0 ? +1 : -1;
    float limite = fase == 1 ? gradosBarrido * 2.0 : gradosBarrido;
    if (fase == 2) sentido = +1;

    giroscopio.update();
    float inicioAngulo = giroscopio.getAngleZ();
    unsigned long inicio = millis();
    while (abs(giroscopio.getAngleZ() - inicioAngulo) < limite &&
           millis() - inicio < 3500) {
      _loop();
      if (visionVeObjeto()) {
        motoresParar();
        _delay(0.12);
        return true;
      }
      if (sentido > 0) motoresTanque(-velocidad, velocidad);
      else motoresTanque(velocidad, -velocidad);
      delay(3);
    }
    motoresParar();
    _delay(0.10);
  }
  return false;
}

bool visionAproximar(int distObjetivoMm, float velocidad, float timeoutSeg) {
  unsigned long inicio = millis();
  unsigned long ultimoVisto = millis();
  unsigned long timeoutMs = (unsigned long)(timeoutSeg * 1000.0);
  int ultimaDist = -1;
  int ultimoEy = 0;

  while (millis() - inicio < timeoutMs) {
    _loop();
    if (visionVeObjeto()) {
      ultimoVisto = millis();
      ultimaDist = vision.dist;
      ultimoEy = vision.ey;

      // Condicion de llegada: distancia objetivo alcanzada o base del objeto en pre-captura
      if ((vision.dist > 0 && vision.dist <= distObjetivoMm) ||
          vision.ey >= VIS_Y_PRECAPTURA) {
        motoresParar();
        _delay(0.12);
        return true;
      }

      float v = velocidad;
      if (vision.dist > 0 && vision.dist < 160) {
        v = limitar(velocidad * vision.dist / 160.0, 22.0, velocidad);
      }
      float corr = limitar(VIS_KP_AVANCE * vision.ex, -v * 0.65, v * 0.65);
      motoresTanque(v - corr, v + corr);
    } else {
      // Si el objeto entra al punto ciego inferior cerca del chasis
      if ((ultimaDist > 0 && ultimaDist < 130) || ultimoEy > 290) {
        motoresParar();
        avanzar(VIS_GRADOS_CIEGOS, 28, 1.0);
        return true;
      }
      if (millis() - ultimoVisto > 500) {
        motoresParar();
        return false;
      }
      motoresTanque(velocidad * 0.45, velocidad * 0.45);
    }
    delay(3);
  }

  motoresParar();
  return false;
}

bool prepararCapturaPorVision(ColorObjeto color) {
  bajarPala();   // 105°: pala en posicion normal de vision
  abrirGarra();  // Paletas alineadas a los laterales (brazos hacia los lados)
  visionPedirColor(color);
  _delay(0.25);

  if (!visionVeObjeto(600) && !visionBuscar()) {
    logPi("artefacto no encontrado");
    return false;
  }
  if (!visionCentrar()) return false;
  if (!visionAproximar(150, 40.0, 5.0)) return false;
  visionCentrar(1.8);
  return visionAproximar(VIS_DIST_PRECAPTURA_MM, 28.0, 4.0);
}

// Secuencia mecanica en 2 pasos de prueba_centrales.ino
void capturarConRutinaProbada() {
  bajarPala();                     // Asegurar 105°
  cerrarGarra();                   // 1. Paletas pivotan al frente y atrapan artefacto de punta
  retroceder(120, 25, 1.8);        // Extrae el artefacto aislándolo de la fila
  _delay(0.5);

  abrirGarra();                    // 2. Abre paletas
  recolectar(3);                   // Baja pala a 111° ras de suelo
  avanzar(90, 55, 1.0);            // Emboca completamente el artefacto
  cerrarGarra();                   // Abraza firme
  bajarPala();                     // Sube a 105° para despegar 6° del suelo y transportar
  retroceder(360, 35, 3.5);        // Retrocede hasta la mitad de la pista
  detener(0.5);
}

// =========================================================================
// GEOMETRIA DE LA PISTA Y CICLO DE ARTEFACTOS
// =========================================================================

// Separacion entre centros de slots centrales
const long PASO_SLOT_GRADOS = 220;
const long OFFSET_SLOT_GRADOS[4] = {
  -(PASO_SLOT_GRADOS * 3L) / 2L,
  -PASO_SLOT_GRADOS / 2L,
  PASO_SLOT_GRADOS / 2L,
  (PASO_SLOT_GRADOS * 3L) / 2L
};

const byte ORDEN_SLOTS[4] = {1, 2, 0, 3};

// Giros gyro para media vuelta (180°)
const float GIRO_SALIDA_1 = 89.0;
const float GIRO_SALIDA_2 = 89.0;
const long RUTA_HASTA_MUSEO_GRADOS = 620;

// Expositores en orden oficial: ROJO, VERDE, NEGRO, AZUL, AMARILLO
// El eje horizontal que une el centro de los artefactos con el museo llega
// a la casilla central de las cinco: NEGRO.
const int DESTINO_REFERENCIA = COLOR_NEGRO;
const long PASO_MUSEO_GRADOS = 195;  // Separacion entre casillas de museo

const byte NUM_ARTEFACTOS_OBJETIVO = 4;
const unsigned long LIMITE_RUTINA_MS = 116000UL;
const unsigned long RESERVA_NUEVO_CICLO_MS = 26000UL;

bool quedaTiempo(unsigned long reservaMs) {
  return millis() - inicioRutinaMs + reservaMs < LIMITE_RUTINA_MS;
}

// Desplazamiento perpendicular en L; termina con el mismo rumbo
bool desplazarLateral(long grados) {
  if (grados == 0) return true;
  if (grados < 0) {
    if (!girarIzquierdaGyro(89.0, 22.0)) return false;
    if (!moverRectoGyro(-grados, 42.0, 5.0)) return false;
    if (!girarDerechaGyro(89.0, 22.0)) return false;
  } else {
    if (!girarDerechaGyro(89.0, 22.0)) return false;
    if (!moverRectoGyro(grados, 42.0, 5.0)) return false;
    if (!girarIzquierdaGyro(89.0, 22.0)) return false;
  }
  return true;
}

bool irCentroASlot(byte slot) {
  if (slot > 3) return false;
  return desplazarLateral(OFFSET_SLOT_GRADOS[slot]);
}

bool volverSlotACentro(byte slot) {
  if (slot > 3) return false;
  return desplazarLateral(-OFFSET_SLOT_GRADOS[slot]);
}

long offsetDestino(ColorObjeto color) {
  if (color < COLOR_ROJO || color > COLOR_AMARILLO) return 0;
  return ((long)((int)color - DESTINO_REFERENCIA)) * PASO_MUSEO_GRADOS;
}

// Secuencia probada de deposito
void depositarConRutinaProbada() {
  recolectar(2);             // 120°: presiona hacia abajo para asentar
  retroceder(23, 26, 1.0);   // Despega de la cuña interna
  recolectar(3);             // 111°: sube pala para que paletas abran sin rozar suelo
  abrirGarra();              // Abre paletas
  barrer();                  // 117°: pala en modo empuje frontal
  avanzar(68, 36, 0.75);     // Empuja artefacto al expositor
  retroceder(90, 28, 1.0);   // Retrocede suave para no arrastrarlo
  recolectar(3);             // Sube pala
}

bool irSlotAMuseoYDepositar(byte slot, ColorObjeto color) {
  // 1. Deshacer el desplazamiento lateral del slot para quedar en el centro
  if (!volverSlotACentro(slot)) return false;

  // 2. Dar media vuelta (180°) hacia los expositores del museo
  if (!girarIzquierdaGyro(GIRO_SALIDA_1, 20.0)) return false;
  if (!girarIzquierdaGyro(GIRO_SALIDA_2, 20.0)) return false;

  // 3. Desplazarse lateralmente al expositor correspondiente al color
  long lateral = offsetDestino(color);
  if (!desplazarLateral(lateral)) return false;

  // 4. Avanzar recto hacia el expositor
  if (!moverRectoGyro(RUTA_HASTA_MUSEO_GRADOS, 60.0, 6.0)) return false;

  // Correccion final por un detector exclusivo de cuadros planos. Si no hay
  // una lectura estable se conserva la pose odometrica en vez de buscar a
  // ciegas entre colores de la pista.
  visionPedirDestino(color);
  _delay(0.30);
  bool destinoVisible = visionVeObjeto(600);
  if (!destinoVisible) {
    // En modo DESTINO solo se aceptan cuadros planos con borde blanco.
    // El barrido corto no puede seguir el artefacto que lleva la garra.
    destinoVisible = visionBuscar(18.0, 16.0);
  }
  if (destinoVisible) {
    if (!visionCentrar(2.0)) logPi("destino visible pero no centro estable");
  } else {
    logPi("destino no visible; usando llegada odometrica");
  }

  // 5. Depositar el artefacto con la rutina precisa
  depositarConRutinaProbada();
  visionPausar();

  // 6. Retorno seguro:
  // Retroceso corto para despejar el expositor
  retroceder(160, 45, 1.5);
  // Deshacer el desplazamiento lateral en zona despejada
  if (!desplazarLateral(-lateral)) return false;
  // Media vuelta (180°) para volver mirando hacia los slots del centro
  if (!girarIzquierdaGyro(GIRO_SALIDA_1, 22.0)) return false;
  if (!girarIzquierdaGyro(GIRO_SALIDA_2, 22.0)) return false;
  // Retornar al centro de la pista
  if (!moverRectoGyro(RUTA_HASTA_MUSEO_GRADOS - 160, 62.0, 6.0)) return false;

  return true;
}

void imprimirMapaSlots() {
  logPi("mapa incremental de slots:");
  for (int i = 0; i < 4; i++) {
    SERIAL_VISION.print("#slot ");
    SERIAL_VISION.print(i);
    SERIAL_VISION.print(" = ");
    if (colorEnSlot[i] == COLOR_NINGUNO) SERIAL_VISION.println("?");
    else SERIAL_VISION.println(NOMBRES_COLOR[(int)colorEnSlot[i]]);
  }
}

bool procesarSlot(byte slot) {
  SERIAL_VISION.print("#visitando slot ");
  SERIAL_VISION.println(slot);

  if (!irCentroASlot(slot)) {
    falloNavegacion = true;
    return false;
  }

  ColorObjeto color = colorEnSlot[slot];
  if (color == COLOR_NINGUNO) color = visionIdentificarColor(1.8);
  if (color == COLOR_NINGUNO) color = visionIdentificarColor(1.5);

  colorEnSlot[slot] = color;
  if (color == COLOR_NINGUNO) {
    logPi("slot sin identidad; se omite");
    if (!volverSlotACentro(slot)) falloNavegacion = true;
    return false;
  }
  logColor("identificado: ", color);

  if (!prepararCapturaPorVision(color)) {
    logPi("fallo en aproximacion; recuperando");
    moverRectoGyro(-180, 34.0, 3.0);
    if (!volverSlotACentro(slot)) falloNavegacion = true;
    falloNavegacion = true;
    return false;
  }

  // Captura en 2 pasos y retroceso a mitad de pista
  capturarConRutinaProbada();
  visionPausar();

  // Viaje al museo, deposito y retorno al centro para el proximo
  if (!irSlotAMuseoYDepositar(slot, color)) {
    falloNavegacion = true;
    return false;
  }

  colorCompletado[(int)color] = true;
  logColor("depositado: ", color);
  return true;
}

// =========================================================================
// SETUP Y PROGRAMA PRINCIPAL
// =========================================================================

void setup() {
  SERIAL_VISION.begin(BAUD_VISION);
  pinMode(BOTON_PIN, INPUT_PULLUP);
  pinMode(PIN_TCRT_IZQ, INPUT);
  pinMode(PIN_TCRT_DER, INPUT);

  Wire.setWireTimeout(3000, true);
  giroscopio.begin();
  _delay(2.0);

  miServo.attach(PIN_SERVO_PALA);
  servoGarra1.attach(placaExpansora.pin1());
  servoGarra2.attach(placaExpansora.pin2());
  miServo.write(PALA_INICIAL);
  abrirGarra();

  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);

  attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
  Encoder_1.setPulse(8);
  Encoder_1.setRatio(46.67);
  Encoder_1.setPosPid(1.8, 0, 1.2);
  Encoder_1.setSpeedPid(0.18, 0, 0);

  attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);
  Encoder_2.setPulse(8);
  Encoder_2.setRatio(46.67);
  Encoder_2.setPosPid(1.8, 0, 1.2);
  Encoder_2.setSpeedPid(0.18, 0, 0);

  logPi("MegaPi lista; esperando boton");
}

void loop() {
  while (digitalRead(BOTON_PIN) == HIGH) _loop();
  while (digitalRead(BOTON_PIN) == LOW) _loop();
  _delay(0.45);

  bajarPala();  // Mantener la pala abajo para no obstruir la cámara al buscar artefactos
  abrirGarra(); // Asegurar garra abierta para la visión

  rutinaIniciada = true;
  inicioRutinaMs = millis();
  SERIAL_VISION.println("S 1");

  unsigned long esperaPi = millis();
  while (!visionViva(1000) && millis() - esperaPi < 2200) _loop();
  if (!visionViva(1000)) logPi("AVISO: enlace de vision no confirmado");

  // La unica busqueda global se hace aqui: robot inmovil, centrado y mirando
  // la fila original. Desde este punto cada slot se sigue por su color fijo.
  if (visionEscanearFila(2.4)) {
    logPi("mapa inicial de cuatro slots confirmado");
    imprimirMapaSlots();
  } else {
    logPi("fila no estable; se usara AUTO solo al llegar a cada slot");
  }

  for (int intento = 0; intento < NUM_ARTEFACTOS_OBJETIVO; intento++) {
    if (!quedaTiempo(RESERVA_NUEVO_CICLO_MS)) {
      logPi("sin tiempo seguro para otro artefacto");
      break;
    }

    byte slot = ORDEN_SLOTS[intento];
    bool depositado = procesarSlot(slot);
    if (falloNavegacion) {
      logPi("fallo critico de navegacion; fin seguro");
      break;
    }
  }

  imprimirMapaSlots();
  motoresParar();
  SERIAL_VISION.println("S 0");
  logPi("rutina finalizada");
  while (true) {
    _loop();
    delay(20);
  }
}
