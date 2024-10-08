#include <Arduino.h>

// Definición de pines
#define LOAD_CURRENT_PIN A2     // A2 para corriente de carga

// Parámetros de muestreo
const int numSamples = 50;   // Número de muestras para suavizar
const int sampleDelay = 5;   // Retardo entre muestras en milisegundos

// Prototipos de funciones
float smoothADCReadings(int pin);
float readVoltage(int pin);
float readCurrent(int pin);

void setup() {
  pinMode(LOAD_CURRENT_PIN, INPUT);
  Serial.begin(115200);
  Serial.println("Inicialización de medición de corriente de carga");
}

void loop() {
  float adcValue = smoothADCReadings(LOAD_CURRENT_PIN);
  float loadVoltage = readVoltage(adcValue);
  float loadCurrent = readCurrent(loadVoltage);

  Serial.print("Lectura ADC cruda: ");
  Serial.println(adcValue);

  Serial.print("Voltaje en ADC (calibrado): ");
  Serial.print(loadVoltage, 4);  // Mostrar voltaje con 4 decimales
  Serial.println(" V");

  Serial.print("Corriente de carga: ");
  Serial.print(loadCurrent, 3);  // Mostrar corriente con 3 decimales
  Serial.println(" A");

  delay(5000);  // Retardo del bucle principal
}

float smoothADCReadings(int pin) {
  long total = 0;
  for (int i = 0; i < numSamples; i++) {
    total += analogRead(pin);
    delay(sampleDelay);
  }
  return total / numSamples;
}

float readVoltage(float adcValue) {
  // Ajusta el valor de referencia aquí para el ESP32-C3 con referencia de 3.3V y ADC de 12 bits
  float referenceVoltage = 3.3;
  float voltage = adcValue * (referenceVoltage / 4095.0); // ADC de 12 bits

  // Compensación personalizada: ajustar el voltaje para que coincida con el valor sin carga
  float calibrationFactor = 1.840 / 2.0396;  // Comparar el valor observado con el valor esperado
  voltage *= calibrationFactor;

  return voltage;
}

float readCurrent(float voltage) {
  // Calcular la corriente basada en la tensión medida y la sensibilidad del sensor
  float offsetVoltage = 1.840; // Offset ajustado para 0A
  float sensitivity = 0.066;   // Sensibilidad de 66 mV/A para ACS712-30A

  float current = (voltage - offsetVoltage) / sensitivity;

  // Asegurarse de que la corriente no sea negativa
  current = max(current, 0.0f);

  return current;
}
