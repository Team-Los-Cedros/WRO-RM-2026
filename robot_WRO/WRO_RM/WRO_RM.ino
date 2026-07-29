#include <Arduino.h>
#include <Wire.h>
#include <MeMegaPi.h>
#include "Adafruit_TCS34725.h" // Asegúrate de instalar esta librería desde el gestor

// ==========================================
// DEFINICIÓN DE MOTORES (Encoders)
// ==========================================
MeEncoderOnBoard Encoder_1(SLOT1); // Puerto 1
MeEncoderOnBoard Encoder_2(SLOT2); // Puerto 2

void isr_process_encoder1(void){
  if(digitalRead(Encoder_1.getPortB()) == 0) { Encoder_1.pulsePosMinus(); }
  else { Encoder_1.pulsePosPlus(); }
}

void isr_process_encoder2(void){
  if(digitalRead(Encoder_2.getPortB()) == 0) { Encoder_2.pulsePosMinus(); }
  else { Encoder_2.pulsePosPlus(); }
}

// ==========================================
// DEFINICIÓN DE SENSORES
// ==========================================
MeLineFollower seguidorLinea(8) ; // Puerto 8
MeGyro giroscopio(6);            // Puerto 6
Adafruit_TCS34725 sensorColor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X); // SCL / SDA

// ==========================================
// DEFINICIÓN DE SERVOMOTORES
// ==========================================
MePort placaExpansora(7); // Puerto 7 usado como adaptador RJ25
Servo servoGarra1;
Servo servoGarra2;
Servo servoElevador;      // Pin 5 directo

void setup() {
  Serial.begin(115200);
  
  // 1. Configuración de registros para motores MegaPi
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);
  
  // 2. Inicializar Interrupciones y PID de los Encoders
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

  // 3. Inicializar Servomotores
  // Usamos los pines extraídos del puerto 7 mediante placaExpansora.pin1() y pin2()
  servoGarra1.attach(placaExpansora.pin1());
  servoGarra2.attach(placaExpansora.pin2());
  servoElevador.attach(5); // Conexión directa al pin 5
  
  // 4. Inicializar Sensores
  giroscopio.begin();
  
  if (sensorColor.begin()) {
    Serial.println("Sensor TCS34725 detectado exitosamente.");
  } else {
    Serial.println("ERROR: No se encuentra el TCS34725. Verifica SCL/SDA.");
  }
  
  Serial.println("--- INICIANDO TEST DE COMPONENTES ---");
}

void _loop() {
  // Las funciones loop() de los encoders deben correr todo el tiempo para aplicar el PID
  Encoder_1.loop();
  Encoder_2.loop();
  giroscopio.update(); // Actualiza los valores de los ejes
}

void loop() {
  _loop(); // Mantenemos el cálculo constante en segundo plano

  // Movimiento base a baja velocidad (Basado en tu bloque de Makeblock)
  Encoder_1.runSpeed(30);
  Encoder_2.runSpeed(-30);

  // Monitorización y testeo de actuadores cada 500 ms para no saturar el Serial
  static unsigned long ultimoTiempo = 0;
  static bool estadoServos = false;
  
  if (millis() - ultimoTiempo > 500) {
    ultimoTiempo = millis();
    
    // --- LECTURA SEGUIDOR DE LÍNEA ---
    // Retorna 0 (ambos en negro), 1 (izquierdo negro), 2 (derecho negro), 3 (ambos blancos)
    int estadoLinea = seguidorLinea.readSensors();
    Serial.print("Linea: "); Serial.print(estadoLinea);
    
    // --- LECTURA GIROSCOPIO ---
    Serial.print(" | Angulo Z: "); 
    Serial.print(giroscopio.getAngleZ()); // El eje Z es el de rotación en el suelo
    
    // --- LECTURA SENSOR DE COLOR ---
    uint16_t r, g, b, c;
    sensorColor.getRawData(&r, &g, &b, &c);
    Serial.print(" | RGB: "); 
    Serial.print(r); Serial.print(","); 
    Serial.print(g); Serial.print(","); 
    Serial.print(b);
    Serial.println();
    
    // --- MOVIMIENTO DE PRUEBA DE SERVOMOTORES ---
    // Intercala entre 0 y 90 grados para confirmar que reciben señal y tienen fuerza
    if (estadoServos) {
      servoGarra1.write(90);
      servoGarra2.write(90);
      servoElevador.write(90);
    } else {
      servoGarra1.write(0);
      servoGarra2.write(0);
      servoElevador.write(0);
    }
    estadoServos = !estadoServos;
  }
}