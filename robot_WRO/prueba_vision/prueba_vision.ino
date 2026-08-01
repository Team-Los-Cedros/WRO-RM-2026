// =========================================================================
// WRO RoboMission Junior 2026 - MegaPi + Raspberry Pi 3B (vision por color)
//
// La Raspberry hace de "sensor inteligente": manda 20 veces por segundo una
// linea con la posicion del objeto. La MegaPi sigue siendo el cerebro que
// decide la rutina.
//
// CABLEADO (opcion recomendada): cable USB de la Raspberry al puerto USB de
// programacion de la MegaPi. En ese caso SERIAL_VISION = Serial.
// Si prefieres los pines TX2/RX2 usa SERIAL_VISION = Serial2 y lee las notas
// de nivel logico al final de este archivo.
// =========================================================================

#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <MeMegaPi.h>

#define BOTON_PIN 4
#define PIN_TCRT_IZQ A4
#define PIN_TCRT_DER A3

// ---- Enlace con la Raspberry -------------------------------------------
// Serial  = cable USB (recomendado, sin conversion de niveles)
// Serial2 = pines TX2(16)/RX2(17)
#define SERIAL_VISION Serial
#define BAUD_VISION   115200

MeGyro giroscopio(8);
MePort placaExpansora(7);
Servo servoGarra1;
Servo servoGarra2;
Servo miServo;
const int pinServo = 5;

MeEncoderOnBoard Encoder_1(SLOT1); // Motor Izquierdo
MeEncoderOnBoard Encoder_2(SLOT2); // Motor Derecho

void isr_process_encoder1(void){
  if(digitalRead(Encoder_1.getPortB()) == 0){ Encoder_1.pulsePosMinus(); }
  else{ Encoder_1.pulsePosPlus(); }
}
void isr_process_encoder2(void){
  if(digitalRead(Encoder_2.getPortB()) == 0){ Encoder_2.pulsePosMinus(); }
  else{ Encoder_2.pulsePosPlus(); }
}

bool rutinaIniciada = false;

// Declaraciones adelantadas. El IDE de Arduino genera los prototipos solo,
// pero dejarlos escritos evita sorpresas al reordenar el archivo.
void _loop();
void _delay(float seconds);
void avanzar(long grados, float velocidad, float tiempoEspera);
void retroceder(long grados, float velocidad, float tiempoEspera);

// =========================================================================
// VISION: recepcion no bloqueante de las tramas de la Raspberry
// =========================================================================
//
// Trama:  T <found> <color> <ex> <ey> <area> <dist> <fps>
//   ex   : error horizontal en px. NEGATIVO = objeto a la izquierda.
//   ey   : fila del borde inferior (mayor = mas cerca)
//   dist : mm estimados (-1 si no hay dato)

struct VisionData {
  bool  encontrado;
  int   ex;
  int   ey;
  long  area;
  int   dist;
  int   fps;
  unsigned long tRecepcion;   // millis() de la ultima trama valida
};

VisionData vision = {false, 0, 0, 0, -1, 0, 0};

// 128 bytes: la respuesta al comando X lleva los cinco colores y ronda los
// 75 caracteres. Con 64 se cortaba.
char  bufVision[128];
byte  idxVision = 0;

// Orden de descarga de IZQUIERDA a DERECHA segun la pista oficial.
// Salen 4 de estos 5 colores en cada ronda; el hueco del que no aparezca se
// deja vacio, y como cada color tiene posicion fija no hay que recolocar nada.
const char* const ORDEN_COLORES[5] = {"ROJO", "VERDE", "NEGRO", "AZUL", "AMARILLO"};
bool colorPresente[5] = {false, false, false, false, false};
volatile bool respuestaEscaneo = false;

// La MegaPi no debe usar Serial.print() a secas si comparte el cable con la
// Pi: el prefijo '#' le dice a la Raspberry que es un log y no una trama.
void logPi(const char* msg) {
  SERIAL_VISION.print('#');
  SERIAL_VISION.println(msg);
}

// Respuesta al comando X:
//   X ROJO 1 -34 145 VERDE 0 0 -1 NEGRO 1 88 260 AZUL 0 0 -1 AMARILLO 1 12 190
void visionProcesarEscaneo(char* linea) {
  for (int i = 0; i < 5; i++) colorPresente[i] = false;

  char* tok = strtok(linea + 2, " ");
  while (tok != NULL) {
    int idx = -1;
    for (int i = 0; i < 5; i++) {
      if (strcmp(tok, ORDEN_COLORES[i]) == 0) { idx = i; break; }
    }
    char* tFound = strtok(NULL, " ");   // found
    strtok(NULL, " ");                  // ex  (no se usa en el escaneo)
    strtok(NULL, " ");                  // dist
    if (idx >= 0 && tFound != NULL) colorPresente[idx] = (atoi(tFound) != 0);
    tok = strtok(NULL, " ");
  }
  respuestaEscaneo = true;
}

void visionProcesarLinea(char* linea) {
  if (linea[0] == 'K' || linea[0] == 'E') return;   // acuse de recibo de la Pi
  if (linea[0] == 'X' && linea[1] == ' ') { visionProcesarEscaneo(linea); return; }
  if (linea[0] != 'T' || linea[1] != ' ') return;

  char* tok = strtok(linea + 2, " ");   // found
  if (!tok) return;
  bool found = (atoi(tok) != 0);

  tok = strtok(NULL, " ");              // color (no se usa aqui)
  if (!tok) return;

  tok = strtok(NULL, " "); if (!tok) return; int ex   = atoi(tok);
  tok = strtok(NULL, " "); if (!tok) return; int ey   = atoi(tok);
  tok = strtok(NULL, " "); if (!tok) return; long ar  = atol(tok);
  tok = strtok(NULL, " "); if (!tok) return; int dist = atoi(tok);
  tok = strtok(NULL, " ");                   int fps  = tok ? atoi(tok) : 0;

  vision.encontrado = found;
  vision.ex   = ex;
  vision.ey   = ey;
  vision.area = ar;
  vision.dist = dist;
  vision.fps  = fps;
  vision.tRecepcion = millis();
}

// Se llama desde _loop(); nunca bloquea.
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
      idxVision = 0;   // linea corrupta o demasiado larga: se descarta
    }
  }
}

// true si la ultima trama es reciente (el enlace esta vivo)
bool visionViva(unsigned long msMax = 300) {
  return (millis() - vision.tRecepcion) < msMax;
}

// true si ademas se esta viendo el objeto
bool visionVeObjeto(unsigned long msMax = 300) {
  return visionViva(msMax) && vision.encontrado;
}

void visionPedirColor(const char* color) {
  SERIAL_VISION.print("C ");
  SERIAL_VISION.println(color);
  vision.encontrado = false;
  vision.tRecepcion = 0;
}

/**
 * Pregunta a la Raspberry cuales de los cinco colores se ven desde aqui.
 * Rellena colorPresente[]. Conviene llamarlo al principio de la ronda, desde
 * un punto donde se vean los cuatro objetos.
 * @return true si la Pi contesto a tiempo.
 */
bool visionEscanearColores(float timeoutSeg = 2.0) {
  respuestaEscaneo = false;
  SERIAL_VISION.println("X");

  unsigned long t0 = millis();
  while (millis() - t0 < (unsigned long)(timeoutSeg * 1000.0)) {
    _loop();
    if (respuestaEscaneo) return true;
    delay(5);
  }
  logPi("TIMEOUT en el escaneo de colores");
  return false;
}

// Indice del color que NO salio en esta ronda (-1 si se vieron los cinco o
// si faltan datos). Su hueco en la zona de descarga se deja vacio.
int colorAusente() {
  int idx = -1, cuenta = 0;
  for (int i = 0; i < 5; i++) {
    if (!colorPresente[i]) { idx = i; cuenta++; }
  }
  return (cuenta == 1) ? idx : -1;
}

// =========================================================================
// BUCLE DE FONDO
// =========================================================================

void _loop() {
  unsigned long tiempoActual = millis();

  static unsigned long ultimoTiempoMotores = 0;
  if(tiempoActual - ultimoTiempoMotores >= 10) {
    Encoder_1.loop();
    Encoder_2.loop();
    ultimoTiempoMotores = tiempoActual;
  }

  static unsigned long ultimoGiroscopio = 0;
  if(tiempoActual - ultimoGiroscopio >= 25) {
    giroscopio.update();
    ultimoGiroscopio = tiempoActual;
  }

  visionActualizar();

  if (rutinaIniciada == true && digitalRead(BOTON_PIN) == LOW) {
    Encoder_1.move(0, 0);
    Encoder_2.move(0, 0);
    Encoder_1.setTarPWM(0);
    Encoder_2.setTarPWM(0);
    unsigned long inicioParada = millis();
    while (true) {
      tiempoActual = millis();
      if (tiempoActual - inicioParada < 1500) {
        if(tiempoActual - ultimoTiempoMotores >= 10) {
          Encoder_1.loop();
          Encoder_2.loop();
          ultimoTiempoMotores = tiempoActual;
        }
      } else {
        delay(500);
      }
    }
  }
}

void _delay(float seconds) {
  if(seconds < 0.0) seconds = 0.0;
  unsigned long endTime = millis() + (unsigned long)(seconds * 1000.0);
  while(millis() < endTime) { _loop(); delay(2); }
}

// =========================================================================
// MOVIMIENTO (misma convencion de signos que prueba_sensores2)
// =========================================================================

void moverRobot(long gradosIzq, long gradosDer, float velocidad, float tiempoEspera) {
  Encoder_1.move(gradosIzq, abs(velocidad));
  Encoder_2.move(-gradosDer, abs(velocidad*1.02));
  _delay(tiempoEspera);
}
void avanzar(long g, float v, float t)    { moverRobot(g, g, v, t); }
void retroceder(long g, float v, float t) { moverRobot(-g, -g, v, t); }
void detener(float t)                     { Encoder_1.setTarPWM(0); Encoder_2.setTarPWM(0); }

// Control directo de velocidad en "coordenadas logicas": + = adelante.
// Coincide con moverRobot(): giro a la derecha => velIzq negativa, velDer positiva.
void motoresTanque(float velIzq, float velDer) {
  Encoder_1.runSpeed(velIzq);
  Encoder_2.runSpeed(-velDer);
}
void motoresParar() {
  Encoder_1.runSpeed(0);
  Encoder_2.runSpeed(0);
}

void girarDerechaGyro(float gradosObjetivo, float velocidad) {
  giroscopio.update();
  float anguloInicial = giroscopio.getAngleZ();
  Encoder_1.runSpeed(-abs(velocidad));
  Encoder_2.runSpeed(-abs(velocidad));
  unsigned long t0 = millis();
  while (abs(giroscopio.getAngleZ() - anguloInicial) < gradosObjetivo) {
    _loop();
    if (millis() - t0 > 5000) { logPi("TIMEOUT giro derecha"); break; }
  }
  motoresParar();
  _delay(0.3);
}

void girarIzquierdaGyro(float gradosObjetivo, float velocidad) {
  giroscopio.update();
  float anguloInicial = giroscopio.getAngleZ();
  Encoder_1.runSpeed(abs(velocidad));
  Encoder_2.runSpeed(abs(velocidad));
  unsigned long t0 = millis();
  while (abs(giroscopio.getAngleZ() - anguloInicial) < gradosObjetivo) {
    _loop();
    if (millis() - t0 > 5000) { logPi("TIMEOUT giro izquierda"); break; }
  }
  motoresParar();
  _delay(0.3);
}

// =========================================================================
// GARRA Y PALA
// =========================================================================

const int GARRA_ABIERTA_S1 = 0;
const int GARRA_CERRADA_S1 = 126;
const int GARRA_CERRADA_S2 = 50;
const int GARRA_ABIERTA_S2 = 180;
const int anguloBajar = 117;

void abrirGarra()  { servoGarra1.write(GARRA_ABIERTA_S1); servoGarra2.write(GARRA_ABIERTA_S2); _delay(0.6); }
void cerrarGarra() { servoGarra1.write(GARRA_CERRADA_S1); servoGarra2.write(GARRA_CERRADA_S2); _delay(0.6); }
void bajar_pala()  { miServo.write(anguloBajar); _delay(1.0); }
void subir_pala()  { for (int a = anguloBajar; a >= 0; a--) { miServo.write(a); _delay(0.050); } _delay(1.0); }
void recolectar(int modo) { miServo.write(modo == 1 ? 97 : 90); _delay(1.0); }
void posicionar()  { miServo.write(50); _delay(1.0); }
void depositar()   { miServo.write(40); _delay(1.0); abrirGarra(); }

// =========================================================================
// MANIOBRAS GUIADAS POR VISION
// =========================================================================

// --- Parametros a ajustar en pista ---
const int   VIS_TOLERANCIA_PX   = 8;    // |ex| por debajo del cual se considera centrado
const float VIS_KP_GIRO         = 0.55; // px -> velocidad de giro
const float VIS_VEL_GIRO_MIN    = 22;   // por debajo de esto los motores no vencen la friccion
const float VIS_VEL_GIRO_MAX    = 55;
const float VIS_KP_AVANCE       = 0.30; // correccion lateral mientras avanza
const int   VIS_DIST_AGARRE_MM  = 70;   // distancia a la que el objeto entra en la garra
const long  VIS_GRADOS_CIEGOS   = 90;   // encoder a avanzar cuando el objeto sale del encuadre

float limitar(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

// Aplica la zona muerta: mantiene el signo pero fuerza un minimo util.
float velGiroDesdeError(int ex) {
  float v = VIS_KP_GIRO * abs(ex);
  v = limitar(v, VIS_VEL_GIRO_MIN, VIS_VEL_GIRO_MAX);
  return (ex >= 0) ? v : -v;
}

/**
 * Gira sobre su eje hasta que el objeto queda alineado con la garra.
 * @return true si quedo centrado; false por timeout o por perder el objeto.
 */
bool visionCentrar(float timeoutSeg = 4.0) {
  unsigned long t0 = millis();
  unsigned long timeoutMs = (unsigned long)(timeoutSeg * 1000.0);
  int centradoConsecutivos = 0;

  while (millis() - t0 < timeoutMs) {
    _loop();

    if (!visionVeObjeto()) {
      motoresParar();
      delay(5);
      continue;   // se espera a que vuelva a verlo; el timeout corta si no vuelve
    }

    if (abs(vision.ex) <= VIS_TOLERANCIA_PX) {
      // Se exigen varias lecturas seguidas para no dar por bueno un frame suelto
      if (++centradoConsecutivos >= 3) {
        motoresParar();
        _delay(0.15);
        return true;
      }
    } else {
      centradoConsecutivos = 0;
    }

    float v = velGiroDesdeError(vision.ex);
    motoresTanque(-v, v);   // ex>0 (objeto a la derecha) => giro a la derecha
    delay(5);
  }

  motoresParar();
  logPi("TIMEOUT visionCentrar");
  return false;
}

/**
 * Barre girando sobre su eje hasta encontrar el objeto del color activo.
 * Hace un barrido a la derecha y, si no lo halla, uno mas amplio a la izquierda.
 */
bool visionBuscar(float gradosBarrido = 60.0, float velocidad = 30.0) {
  for (int fase = 0; fase < 3; fase++) {
    // fase 0: derecha; fase 1: izquierda (doble recorrido); fase 2: volver
    float signo = (fase == 0) ? -1.0 : 1.0;
    float limite = (fase == 1) ? gradosBarrido * 2.0 : gradosBarrido;
    if (fase == 2) { signo = -1.0; limite = gradosBarrido; }

    giroscopio.update();
    float aFase = giroscopio.getAngleZ();
    Encoder_1.runSpeed(signo * abs(velocidad));
    Encoder_2.runSpeed(signo * abs(velocidad));

    unsigned long t0 = millis();
    while (abs(giroscopio.getAngleZ() - aFase) < limite) {
      _loop();
      if (visionVeObjeto()) {
        motoresParar();
        _delay(0.2);
        return true;
      }
      if (millis() - t0 > 6000) break;
    }
    motoresParar();
    _delay(0.2);
  }

  logPi("visionBuscar: objeto no encontrado");
  return false;
}

/**
 * Avanza hacia el objeto corrigiendo la direccion con el error de vision.
 * Se detiene al llegar a distObjetivoMm, o cuando el objeto sale del encuadre
 * (lo normal cuando ya esta muy cerca: la camara deja de verlo).
 *
 * @return true si llego a distancia de agarre.
 */
bool visionAproximar(int distObjetivoMm = VIS_DIST_AGARRE_MM,
                     float velocidad = 45.0, float timeoutSeg = 8.0) {
  unsigned long t0 = millis();
  unsigned long timeoutMs = (unsigned long)(timeoutSeg * 1000.0);
  unsigned long tUltimoVisto = millis();
  int ultimaDist = -1;

  while (millis() - t0 < timeoutMs) {
    _loop();

    if (visionVeObjeto()) {
      tUltimoVisto = millis();
      ultimaDist = vision.dist;

      if (vision.dist > 0 && vision.dist <= distObjetivoMm) {
        motoresParar();
        _delay(0.15);
        return true;
      }

      // Frena progresivamente en los ultimos 15 cm para no empujar el objeto
      float vel = velocidad;
      if (vision.dist > 0 && vision.dist < 150) {
        vel = limitar(velocidad * (vision.dist / 150.0), 25.0, velocidad);
      }

      float corr = limitar(VIS_KP_AVANCE * vision.ex, -vel * 0.7, vel * 0.7);
      motoresTanque(vel - corr, vel + corr);
    } else {
      // Si lo perdio estando ya cerca, es que entro en la zona ciega de la
      // camara: se avanza a ciegas lo justo para meterlo en la garra.
      if (ultimaDist > 0 && ultimaDist < 160) {
        motoresParar();
        _delay(0.1);
        logPi("objeto en zona ciega: avance final por encoder");
        avanzar(VIS_GRADOS_CIEGOS, 30, 1.5);
        return true;
      }
      // Si lo perdio lejos, sigue recto un momento por si fue un parpadeo
      if (millis() - tUltimoVisto > 500) {
        motoresParar();
        logPi("objeto perdido durante la aproximacion");
        return false;
      }
      motoresTanque(velocidad * 0.6, velocidad * 0.6);
    }
    delay(5);
  }

  motoresParar();
  logPi("TIMEOUT visionAproximar");
  return false;
}

/**
 * Secuencia completa: buscar -> centrar -> aproximar -> agarrar.
 */
bool recogerObjetoPorVision(const char* color) {
  visionPedirColor(color);
  _delay(0.4);   // la Pi necesita un par de frames con el color nuevo

  if (!visionVeObjeto(600)) {
    if (!visionBuscar()) return false;
  }

  abrirGarra();
  bajar_pala();

  // Dos pasadas: centrar de lejos, acercarse, y refinar el centrado de cerca,
  // donde un error de pocos pixeles ya son varios milimetros.
  if (!visionCentrar()) return false;
  if (!visionAproximar(180, 50.0)) return false;
  visionCentrar(2.0);
  if (!visionAproximar(VIS_DIST_AGARRE_MM, 32.0, 5.0)) return false;

  cerrarGarra();
  recolectar(2);
  return true;
}

// =========================================================================
// ZONA DE DESCARGA
// =========================================================================
//
// Los cinco huecos van de izquierda a derecha en el orden de ORDEN_COLORES.
// Como cada color tiene su hueco fijo, el del color ausente simplemente se
// queda vacio: no hay que compactar ni recalcular nada.
//
// POS_DESCARGA_GRADOS son los grados de encoder desde el borde IZQUIERDO de
// la zona hasta el centro de cada hueco. HAY QUE MEDIRLOS EN LA PISTA:
// coloca el robot alineado con el hueco 0, pon el contador a cero y avanza
// lateralmente hasta cada hueco anotando el valor.
const long POS_DESCARGA_GRADOS[5] = {0, 150, 300, 450, 600};

/**
 * Lleva el objeto ya sujetado al hueco que le toca a su color.
 * @param idxColor indice dentro de ORDEN_COLORES (0 = rojo ... 4 = amarillo)
 * @param gradosActuales posicion lateral actual del robot en la zona
 * @return los grados en los que queda el robot, para encadenar el siguiente
 */
long depositarEnHueco(int idxColor, long gradosActuales) {
  if (idxColor < 0 || idxColor > 4) return gradosActuales;

  long objetivo = POS_DESCARGA_GRADOS[idxColor];
  long delta = objetivo - gradosActuales;

  if (delta > 0)      avanzar(delta, 55, 2.5);
  else if (delta < 0) retroceder(-delta, 55, 2.5);

  posicionar();
  depositar();
  _delay(0.3);

  return objetivo;
}

// =========================================================================

void setup() {
  SERIAL_VISION.begin(BAUD_VISION);

  pinMode(BOTON_PIN, INPUT_PULLUP);
  pinMode(PIN_TCRT_IZQ, INPUT);
  pinMode(PIN_TCRT_DER, INPUT);

  Wire.setWireTimeout(3000, true);
  giroscopio.begin();
  _delay(2.0);

  miServo.attach(pinServo);
  servoGarra1.attach(placaExpansora.pin1());
  servoGarra2.attach(placaExpansora.pin2());
  miServo.write(0);
  abrirGarra();

  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);

  attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
  Encoder_1.setPulse(8);
  Encoder_1.setRatio(46.67);
  Encoder_1.setPosPid(1.8,0,1.2);
  Encoder_1.setSpeedPid(0.18,0,0);

  attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);
  Encoder_2.setPulse(8);
  Encoder_2.setRatio(46.67);
  Encoder_2.setPosPid(1.8,0,1.2);
  Encoder_2.setSpeedPid(0.18,0,0);

  logPi("MegaPi lista");
}

void loop() {
  // 1. Esperar el boton
  while (digitalRead(BOTON_PIN) == HIGH) { _loop(); }
  while (digitalRead(BOTON_PIN) == LOW)  { _loop(); }
  _delay(0.5);
  rutinaIniciada = true;

  // 2. Comprobar que la Raspberry esta enviando datos antes de arrancar
  if (!visionViva(1000)) {
    logPi("AVISO: sin datos de la Raspberry");
  }

  // ==========================================
  // RUTINA DE PRUEBA
  // ==========================================
  //
  // Paso 1: averiguar que cuatro colores salieron en esta ronda.
  // Hazlo desde un punto donde se vean los objetos; si desde el cuadro de
  // inicio no se ven todos, muevete primero y escanea despues.
  if (visionEscanearColores()) {
    for (int i = 0; i < 5; i++) {
      SERIAL_VISION.print('#');
      SERIAL_VISION.print(ORDEN_COLORES[i]);
      SERIAL_VISION.println(colorPresente[i] ? ": presente" : ": NO salio");
    }
    int ausente = colorAusente();
    if (ausente >= 0) {
      SERIAL_VISION.print("#hueco vacio: ");
      SERIAL_VISION.println(ORDEN_COLORES[ausente]);
    }
  }

  // Paso 2: recoger y depositar cada color presente, en el orden de la zona.
  long posLateral = 0;
  for (int i = 0; i < 5; i++) {
    if (!colorPresente[i]) continue;

    if (recogerObjetoPorVision(ORDEN_COLORES[i])) {
      SERIAL_VISION.print("#capturado: ");
      SERIAL_VISION.println(ORDEN_COLORES[i]);
      // Aqui falta el traslado desde donde este el robot hasta el borde
      // izquierdo de la zona de descarga; eso depende de tu recorrido.
      posLateral = depositarEnHueco(i, posLateral);
    } else {
      SERIAL_VISION.print("#fallo al capturar: ");
      SERIAL_VISION.println(ORDEN_COLORES[i]);
    }
  }

  detener(0.5);
  while(1) { _loop(); }
}

// =========================================================================
// NOTAS DE CABLEADO
// =========================================================================
//
// OPCION A (recomendada) - USB
//   Cable USB tipo A-B desde la Raspberry al conector de programacion de la
//   MegaPi. SERIAL_VISION = Serial. En la Pi el puerto sera /dev/ttyUSB0.
//   Ventajas: sin conversion de niveles, alimentacion y datos por un cable,
//   se puede seguir programando la placa por el mismo puerto.
//   Ojo: al abrir el puerto la Pi resetea la MegaPi (por eso el script espera
//   2.5 s). Si no quieres ese reset, pon "evitar_reset_dtr": true en config.json.
//
// OPCION B - UART por pines (TX2 = 16, RX2 = 17)
//   SERIAL_VISION = Serial2.
//     Pi GPIO14 (TX, pin 8)  --> MegaPi RX2 (17)     directo, 3.3V basta
//     Pi GPIO15 (RX, pin 10) <-- MegaPi TX2 (16)     CON DIVISOR: 5V danan la Pi
//        MegaPi TX2 --[1k]--+--[2k]-- GND
//                           +--> Pi GPIO15
//     GND de la Pi --- GND de la MegaPi   (obligatorio)
//   Y en la Pi: sudo raspi-config -> Interface -> Serial ->
//     login shell por serie NO, hardware serial SI. El puerto es /dev/serial0.
//   Verifica primero que los pines 16/17 esten libres: en este proyecto el
//   puerto RJ25 numero 7 se usa para los servos de la garra, y en la MegaPi
//   algunos puertos RJ25 comparten pines con los UART. Antes de cablear,
//   confirma que servoGarra1/2 no queden en 16/17 imprimiendo
//   placaExpansora.pin1() y pin2() en el setup.
//
// NUNCA alimentes la Raspberry Pi 3B desde la MegaPi: necesita hasta 2.5 A.
// Usa una power bank independiente y comparte solo la masa.
