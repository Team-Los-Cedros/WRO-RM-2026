#include <Arduino.h>
#include <Wire.h>
#include <MeMegaPi.h>
#include "utility/Servo.h" // Mantenemos la librería corregida para evitar errores de compilación

// ==========================================
// ⚙️ TIEMPOS PERSONALIZADOS (En milisegundos)
// ==========================================
const unsigned long TIEMPO_RECTO_1   = 35000; // 35 segundos recto
const unsigned long TIEMPO_IZQUIERDA  = 1000;  // Tiempo de giro a la izquierda (Ajústalo a tu gusto)
const unsigned long TIEMPO_RECTO_2   = 40000; // 40 segundos recto

// Configuración de velocidades (Valores entre -255 y 255)
const int VELOCIDAD_LINEAL = 80;  // Velocidad para avanzar recto
const int VELOCIDAD_GIRO   = 60;  // Velocidad para hacer el giro

// ==========================================
// DEFINICIÓN DE COMPONENTES
// ==========================================
MeEncoderOnBoard Encoder_1(SLOT1); 
MeEncoderOnBoard Encoder_2(SLOT2); 
Servo servoMG996R; 

// Interrupciones de los Encoders
void isr_process_encoder1(void){ if(digitalRead(Encoder_1.getPortB()) == 0) { Encoder_1.pulsePosMinus(); } else { Encoder_1.pulsePosPlus(); } }
void isr_process_encoder2(void){ if(digitalRead(Encoder_2.getPortB()) == 0) { Encoder_2.pulsePosMinus(); } else { Encoder_2.pulsePosPlus(); } }

void _loop() {
  Encoder_1.loop();
  Encoder_2.loop();
}

// Función especial para contar tiempo sin congelar el PID de los encoders
void esperar(unsigned long tiempoMS) {
  unsigned long tiempoInicial = millis();
  while (millis() - tiempoInicial < tiempoMS) {
    _loop(); // Mantiene vivos los motores mientras corre el tiempo
  }
}

// ==========================================
// ACCIONES DE MOVIMIENTO
// ==========================================
void detener() {
  Encoder_1.runSpeed(0);
  Encoder_2.runSpeed(0);
}

void moverAdelante() {
  Encoder_1.runSpeed(VELOCIDAD_LINEAL);
  Encoder_2.runSpeed(-VELOCIDAD_LINEAL); // Signo invertido por la posición física de los motores
}

void girarIzquierda() {
  Encoder_1.runSpeed(-VELOCIDAD_GIRO);
  Encoder_2.runSpeed(-VELOCIDAD_GIRO);
}

// ==========================================
// CONFIGURACIÓN INICIAL (SETUP)
// ==========================================
void setup() {
  Serial.begin(115200);
  
  // Configuración de los temporizadores de la MegaPi
  TCCR1A = _BV(WGM10); TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20); TCCR2B = _BV(CS21);
  
  attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
  Encoder_1.setPulse(8); Encoder_1.setRatio(46.67);
  Encoder_1.setPosPid(1.8, 0, 1.2); Encoder_1.setSpeedPid(0.18, 0, 0);
  
  attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);
  Encoder_2.setPulse(8); Encoder_2.setRatio(46.67);
  Encoder_2.setPosPid(1.8, 0, 1.2); Encoder_2.setSpeedPid(0.18, 0, 0);

  // Servo conectado al Pin 5 directo
  servoMG996R.attach(5); 
  
  Serial.println("--- ROBOT LISTO PARA INICIAR LA RUTA ---");
  esperar(2000); // 2 segundos de pausa de seguridad antes de arrancar
  
  // ========================================================
  // 🗺️ EJECUCIÓN DE TU NUEVA RUTA
  // ========================================================
  
  // 1. Va recto por 35 segundos
  Serial.println("1. Avanzando recto por 35 segundos...");
  moverAdelante();
  esperar(TIEMPO_RECTO_1);
  detener();
  esperar(600); // Pequeña pausa para absorber la inercia del chasis

  // 2. Gira a la izquierda
  Serial.println("2. Girando a la izquierda...");
  girarIzquierda();
  esperar(TIEMPO_IZQUIERDA);
  detener();
  esperar(600); // Otra pausa de estabilidad

  // 3. Va recto por 40 segundos
  Serial.println("3. Avanzando recto por 40 segundos...");
  moverAdelante();
  esperar(TIEMPO_RECTO_2);
  detener();

  Serial.println("--- RUTA COMPLETADA CON ÉXITO ---");
}

void loop() {
  // Al terminar la rutina en el setup, el loop bloquea el robot para que no repita movimientos
  detener();
  _loop(); 
}