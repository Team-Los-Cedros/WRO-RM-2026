#include <Wire.h>
#include "Adafruit_TCS34725.h"

// Inicializamos el sensor con el tiempo de integración y ganancia por defecto
// TCS34725_INTEGRATIONTIME_50MS: Tiempo que tarda en medir (a menor tiempo, más rápido pero menos preciso)
// TCS34725_GAIN_4X: Amplificación de la luz (opciones: 1X, 4X, 16X, 60X)
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

void setup() {
  Serial.begin(9600);
  Serial.println("Probando sensor de color TCS34725...");

  // Verificar si el sensor está conectado correctamente
  if (!tcs.begin()) {
    Serial.println("¡Error! No se encontró el sensor TCS34725. Revisa las conexiones.");
    while (1); // Detener el programa si no hay sensor
  }
  
  Serial.println("Sensor detectado correctamente. Empezando lecturas...");
}

void loop() {
  uint16_t r, g, b, c;
  uint16_t colorTemp, lux;

  // Leer los valores crudos del sensor
  // r = Rojo, g = Verde, b = Azul, c = Clear (Luz blanca/total)
  tcs.getRawData(&r, &g, &b, &c);

  // Calcular la temperatura de color y la iluminación en luxes
  colorTemp = tcs.calculateColorTemperature(r, g, b);
  lux = tcs.calculateLux(r, g, b);

  // Mostrar los resultados en el Monitor Serie
  Serial.print("Rojo: "); Serial.print(r); Serial.print(" | ");
  Serial.print("Verde: "); Serial.print(g); Serial.print(" | ");
  Serial.print("Azul: "); Serial.print(b); Serial.print(" | ");
  Serial.print("Clear: "); Serial.print(c); Serial.print(" || ");
  
  Serial.print("Temp Color: "); Serial.print(colorTemp); Serial.print(" K | ");
  Serial.print("Lux: "); Serial.println(lux);

  // Esperar 500 milisegundos antes de la siguiente lectura
  delay(500);
}