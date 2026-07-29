#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <math.h>

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// Estructura para almacenar nuestra base de datos de colores
struct ColorReferencia {
  const char* nombre;
  int r;
  int g;
  int b;
};

// =========================================================================
// MATRIZ DE CALIBRACIÓN (Modifica estos valores con tus propias lecturas)
// =========================================================================
// NOTA: Estos valores deben ser los RGB normalizados (0-255) que da tu sensor.
const int NUM_COLORES = 8;
ColorReferencia misColores[NUM_COLORES] = {
  // Nombre          R    G    B
  {"ROJO",          193, 43, 36},
  {"VERDE",          84, 105, 64},
  {"AZUL",           87, 89, 97},
  {"AMARILLO",      122, 91, 38},
  {"CIAN / CELESTE", 92, 90, 68},
  {"MAGENTA",       94, 84, 76},
  {"BLANCO",        100, 87, 63},
  {"NEGRO / OSCURO", 96, 96, 64}  // Valores balanceados pero con 'c' muy baja
};

void setup() {
  Serial.begin(9600);
  if (!tcs.begin()) {
    Serial.println("¡Error! Sensor TCS34725 no encontrado.");
    while (1);
  }
  Serial.println("Sensor calibrado por matriz euclidiana listo.");
}

void loop() {
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);

  // Umbral de oscuridad absoluta
  if (c < 40) {
    Serial.println("Color detectado: NEGRO / OSCURIDAD");
    delay(500);
    return;
  }

  // Normalización RGB (0 - 255) basada en la intensidad total
  uint32_t sum = c;
  int r_norm = (int)(((float)r / sum) * 255.0);
  int g_norm = (int)(((float)g / sum) * 255.0);
  int b_norm = (int)(((float)b / sum) * 255.0);

  // Algoritmo del vecino más cercano (Distancia Euclidiana)
  int indiceGanador = 0;
  float menorDistancia = 999999.0; // Un número muy alto para empezar

  for (int i = 0; i < NUM_COLORES; i++) {
    // Fórmula matemática: d = sqrt((r2-r1)^2 + (g2-g1)^2 + (b2-b1)^2)
    long difR = r_norm - misColores[i].r;
    long difG = g_norm - misColores[i].g;
    long difB = b_norm - misColores[i].b;
    
    float distancia = sqrt((difR * difR) + (difG * difG) + (difB * difB));

    // Si este color está más cerca que el anterior, lo guardamos
    if (distancia < menorDistancia) {
      menorDistancia = distancia;
      indiceGanador = i;
    }
  }

  // Imprimir el color más preciso
  Serial.print("Color detectado: ");
  Serial.print(misColores[indiceGanador].nombre);
  
  // Opcional: Descomenta la línea de abajo para ver los valores reales y calibrar mejor
  Serial.print(" -> R:"); Serial.print(r_norm); Serial.print(" G:"); Serial.print(g_norm); Serial.print(" B:"); Serial.println(b_norm);
  
  Serial.println();
  delay(2000);
}