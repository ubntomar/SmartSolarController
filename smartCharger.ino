#include <Wire.h>
#include <Adafruit_INA219.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include "esp_task_wdt.h"
#include <Preferences.h>

Preferences preferences;
// TIEMPO EN SEGUNDOS PARA EL WATCHDOG
#define WDT_TIMEOUT 5

// DEFINICIÓN DE PINES I2C
#define SDA_PIN 8  // Pin SDA para comunicación I2C (en ESP32/ESP8266)
#define SCL_PIN 9  // Pin SCL para comunicación I2C (en ESP32/ESP8266)

// PINES DE CONTROL
#define LOAD_CONTROL_PIN 7   // Pin de control del sistema
#define LED_SOLAR   3   // Pin para el LED indicador de corriente solar

// SENSORES INA219
Adafruit_INA219 ina219_1(0x40);  // Sensor INA219 dirección 0x40 (panel->batería)
Adafruit_INA219 ina219_2(0x41);  // Sensor INA219 dirección 0x41 (batería->carga)

// CONFIGURACIÓN PWM
const int  pwmPin        = 2;       // Pin para la señal PWM
const int  pwmFrequency  = 40000;   // Frecuencia PWM de 40 kHz
const int  pwmResolution = 8;       // Resolución de 8 bits (0-255)

// CONFIGURACIÓN DE LECTURAS
const int   numSamples        = 20;       // Nº de muestras p/ promedio
const float maxAllowedCurrent = 6000.0;   // Evitar picos falsos (6 A)

// PARÁMETROS DE CARGA
const float bulkVoltage        = 14.4;    // Voltaje etapa BULK
const float absorptionVoltage  = 14.4;    // Voltaje etapa ABSORCIÓN
const float floatVoltage       = 13.6;    // Voltaje etapa FLOTACIÓN
float absorptionCurrentThreshold = 350.0;    // mA para pasar a FLOTACIÓN
const float maxChargeCurrent   = 10000.0; // Corriente máx (mA) = 10 A

// TIEMPO MÁXIMO DE ABSORCIÓN (ej. 2 horas = 7200 s)
const unsigned long absorptionTimeLimit = 7200UL;
unsigned long absorptionStartTime = 0;

// ESTADOS DE CARGA
enum ChargeState {
  BULK_CHARGE,
  ABSORPTION_CHARGE,
  FLOAT_CHARGE,
  ERROR
};

ChargeState currentState = BULK_CHARGE;

// VARIABLE GLOBAL DE PWM (0-255 antes de invertir)
int currentPWM = 0;

// ALMACENAR CORRIENTES GENERADAS
float panelToBatteryCurrent =0;
float batteryToLoadCurrent =0;

// PARÁMETROS DE CONTROL DE VOLTAJE
const float LVD = 12.0;  // Low Voltage Disconnect
const float LVR = 12.5;  // Low Voltage Reconnect

// Configuración del punto de acceso
const char *ssid = "Cargador";
const char *password = "12345678";

// Servidor web
AsyncWebServer server(80);

// Variables para almacenar los valores de entrada
float batteryCapacity = 50.0; // Capacidad de la batería en amperios hora (valor predeterminado)
float thresholdPercentage = 1.0; // Umbral de corriente en porcentaje (valor predeterminado)

// ---------------------------------------------------------------------------
// SETUP
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Iniciando sensores INA219...");

  // Pines de control
  pinMode(LOAD_CONTROL_PIN, OUTPUT);
  pinMode(LED_SOLAR,   OUTPUT);
  digitalWrite(LOAD_CONTROL_PIN, HIGH);
  digitalWrite(LED_SOLAR,   LOW);

  // Configuración del watchdog
  esp_task_wdt_config_t wdtConfig = {
      .timeout_ms = WDT_TIMEOUT * 1000,  // Tiempo en milisegundos
      .idle_core_mask = 0,               // Núcleo (para ESP32 multicore, 0 para ESP32C3)
      .trigger_panic = true              // Reiniciar el sistema en caso de timeout
  };

  // Inicialización del watchdog
  if (esp_task_wdt_init(&wdtConfig) == ESP_OK) {
    Serial.println("Watchdog iniciado correctamente.");
  } else {
    Serial.println("Error al iniciar el Watchdog.");
  }

  // Agregar la tarea principal (loop) al Watchdog
  esp_task_wdt_add(NULL);  // NULL agrega la tarea actual (loop)

  // Inicializar I2C
  Wire.begin(SDA_PIN, SCL_PIN);

  // Inicializar sensores INA219
  if (!ina219_1.begin()) {
    Serial.println("No se pudo encontrar INA219 en 0x40.");
    while (1);
  }
  if (!ina219_2.begin()) {
    Serial.println("No se pudo encontrar INA219 en 0x41.");
    while (1);
  }

  // Calibración básica (adaptar si necesitas más rango)
  ina219_1.setCalibration_32V_2A();
  ina219_2.setCalibration_32V_2A();

  Serial.println("Sensores INA219 listos.");

  // Configurar PWM
  bool success = ledcAttach(pwmPin, pwmFrequency, pwmResolution);
  if (!success) {
    Serial.println("Error al configurar el PWM");
    while (true);
  }

  // Iniciar PWM en cero
  setPWM(0);
  currentState = BULK_CHARGE;

  // Configurar el punto de acceso
  WiFi.softAP(ssid, password);
  Serial.println("Punto de acceso iniciado");
  Serial.print("IP del servidor: ");
  Serial.println(WiFi.softAPIP());

  // Configurar el servidor web
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", getHTML());
  });

   // Iniciar Preferences en modo lectura
  preferences.begin("my-app", true);
  batteryCapacity = preferences.getFloat("batteryCap");
  thresholdPercentage = preferences.getFloat("thresholdPerc");
  preferences.end();

  // Actualizar absorptionCurrentThreshold
  absorptionCurrentThreshold = (batteryCapacity * thresholdPercentage) *10;

  server.on("/data", HTTP_GET, [](AsyncWebServerRequest *request){
    String json = getData();
    request->send(200, "application/json", json);
  });

  server.on("/update", HTTP_POST, [](AsyncWebServerRequest *request){
    if (request->hasParam("batteryCapacity", true) && request->hasParam("thresholdPercentage", true)) {
      batteryCapacity = request->getParam("batteryCapacity", true)->value().toFloat();
      thresholdPercentage = request->getParam("thresholdPercentage", true)->value().toFloat();

      // Actualizar absorptionCurrentThreshold
      absorptionCurrentThreshold = (batteryCapacity * thresholdPercentage) *10;

      // >>> Guardar en NVS
      preferences.begin("my-app", false);
      preferences.putFloat("batteryCap", batteryCapacity);
      preferences.putFloat("thresholdPerc", thresholdPercentage);
      preferences.end();

      // Redirigir a la página principal
      request->redirect("/");
    } else {
      request->send(400, "text/plain", "Parámetros inválidos");
    }
  });

  server.begin();
}

// ---------------------------------------------------------------------------
// LOOP
// ---------------------------------------------------------------------------
void loop() {
  esp_task_wdt_reset();  // Reset del watchdog para evitar reinicio
  // LEER DATOS DE SENSORES
  // Promedia corriente de panel->batería
  panelToBatteryCurrent = getAverageCurrent(ina219_1);
  // Promedia corriente de batería->carga
  batteryToLoadCurrent = getAverageCurrent(ina219_2);
  // Voltaje panel
  float voltagePanel    = ina219_1.getBusVoltage_V();
  // Voltaje batería
  float voltageBateriaEnSensor2 = ina219_2.getBusVoltage_V();

  // ENCENDER LED SI HAY CORRIENTE DESDE PANEL
  if (panelToBatteryCurrent > 0) {
    digitalWrite(LED_SOLAR, HIGH);
  } else {
    digitalWrite(LED_SOLAR, LOW);
  }

  // MOSTRAR EN SERIAL
  Serial.println("--------------------------------------------");
  Serial.print("Panel->Batería: Corriente = ");
  Serial.print(panelToBatteryCurrent);
  Serial.print(" mA, VoltajePanel = ");
  Serial.print(voltagePanel);
  Serial.println(" V");

  Serial.print("Batería->Carga : Corriente = ");
  Serial.print(batteryToLoadCurrent);
  Serial.print(" mA, VoltajeBat = ");
  Serial.print(voltageBateriaEnSensor2);
  Serial.println(" V");

  // IMPRIMIR ESTADO DE CARGA
  Serial.print("Estado de carga: ");
  Serial.println(getChargeStateString(currentState));

  // IMPRIMIR HASTA QUE VOLTAGE ES LA ETAPA BULK
  Serial.print("Voltaje etapa BULK: ");
  Serial.println(bulkVoltage);

  // VERIFICAR Y ASEGURAR QUE SI NO ESTÁ INGRESANDO CORRIENTE DE LOS PANELES SOLARES ENTONCES EL PWM DEBE ESTAR EN 0 PARA NO GENERAR RAFAGAS DE CARGA ABRUPTA A LA BATERÍA
  if (panelToBatteryCurrent <=10.0 && currentPWM !=0 ) {
    // Si la corriente de páneles es muy pequeña o igual a 0mA, debemos forzar el PWM a 0 para no permitir que un valor alto pueda estropear el hardware o la batería.
    currentPWM=0;
    Serial.println("Forzando el PWM a 0 ya que NO se detecta presencia de corriente de páneles solares");
  }

  // -------------------------------------------------------------------------
  // CONTROL DE VOLTAJE (LVD y LVR)
  // -------------------------------------------------------------------------
  if (voltageBateriaEnSensor2 < LVD) {
    // Si el voltaje de la batería cae por debajo de LVD, desactivar el sistema
    digitalWrite(LOAD_CONTROL_PIN, LOW);
    Serial.println("Desactivando el sistema (voltaje < LVD)");
  } else if (voltageBateriaEnSensor2 > LVR) {
    // Si el voltaje de la batería sube por encima de LVR, reactivar el sistema
    digitalWrite(LOAD_CONTROL_PIN, HIGH);
    Serial.println("Reactivando el sistema (voltaje > LVR)");
  }

  // -------------------------------------------------------------------------
  // RE-ENTRY CHECK (NUEVA LÓGICA)
  //  Si el voltaje de la batería se mantiene por debajo de un umbral
  //  (ej. 12.6 V) por un cierto tiempo (ej. 30 s), forzamos un retorno a BULK
  // -------------------------------------------------------------------------
  const float reEnterBulkVoltage = 12.6;      // Umbral para re-entrar a Bulk
  const unsigned long reEnterTime = 30000UL;  // Tiempo en ms (30 s)
  static unsigned long lowVoltageStart = 0;   // Para contabilizar tiempo
  static bool belowThreshold = false;

  if (voltageBateriaEnSensor2 < reEnterBulkVoltage) {
    // Si la tensión está por debajo del umbral...
    if (!belowThreshold) {
      // Primera vez que cruzamos por debajo
      belowThreshold = true;
      lowVoltageStart = millis();
    } else {
      // Ya estábamos por debajo, verificar cuánto tiempo
      if (millis() - lowVoltageStart >= reEnterTime) {
        // Forzar retorno a Bulk si no estamos ya en Bulk
        if (currentState != BULK_CHARGE) {
          currentState = BULK_CHARGE;
          Serial.println("-> Forzando retorno a BULK_CHARGE (batería < 12.6 V por 30s)");
        }
      }
    }
  } else {
    // Si la tensión sube por encima del umbral, reseteamos
    belowThreshold = false;
    lowVoltageStart = 0;
  }

  // ACTUALIZAR ETAPAS DE CARGA
  updateChargeState(voltageBateriaEnSensor2, panelToBatteryCurrent);

  // Pausa
  delay(1000);
}

// ---------------------------------------------------------------------------
// FUNCIÓN GET AVERAGE CURRENT (INA219) - VERSIÓN ORIGINAL (shunt 10 mΩ)
// ---------------------------------------------------------------------------
float getAverageCurrent(Adafruit_INA219 &ina) {
  float totalCurrent = 0;
  int validSamples   = 0;

  for (int i = 0; i < numSamples; i++) {
    float current_mA = ina.getCurrent_mA() * 10; // Multiplica por 10 (shunt 10 mΩ)

    // Ignorar valores fuera del rango permitido
    if (current_mA >= 0 && current_mA <= maxAllowedCurrent) {
      totalCurrent += current_mA;
      validSamples++;
    }
    delay(5); // pequeña pausa para estabilizar lecturas
  }

  if (validSamples == 0) {
    return 0;
  }
  return totalCurrent / validSamples;
}

// ---------------------------------------------------------------------------
// MÁQUINA DE ESTADOS PARA CARGA
// ---------------------------------------------------------------------------
void updateChargeState(float batteryVoltage, float chargeCurrent) {
  switch (currentState) {
    case BULK_CHARGE:
      bulkControl(batteryVoltage, chargeCurrent);

      // Transición a Absorción si llegamos a bulkVoltage (14.4V)
      if (batteryVoltage >= bulkVoltage) {
        currentState = ABSORPTION_CHARGE;
        absorptionStartTime = millis();
        Serial.println("-> Transición a ABSORPTION_CHARGE");
      }
      break;

    case ABSORPTION_CHARGE:
      absorptionControl(batteryVoltage, chargeCurrent);

      // Pasar a Flotación si corriente < xxx mA , tener en cuenta la corriente de carga para determinar cuanto entra a la batería.
      if (chargeCurrent <= ( absorptionCurrentThreshold + batteryToLoadCurrent ) ) {
        currentState = FLOAT_CHARGE;
        Serial.println("-> Transición a FLOAT_CHARGE (corriente < threshold)");
      }
      // O si se supera el tiempo máximo de absorción
      else if ((millis() - absorptionStartTime) / 1000UL >= absorptionTimeLimit) {
        currentState = FLOAT_CHARGE;
        Serial.println("-> Transición a FLOAT_CHARGE (tiempo max Absorción)");
      }
      break;

    case FLOAT_CHARGE:
      if (chargeCurrent <= ( absorptionCurrentThreshold + batteryToLoadCurrent ) ) {
        floatControl(batteryVoltage);
      }
      else{
        Serial.println("Por alguna razón estoy en FLOAT_CHARGE pero no estoy cumpliendo chargeCurrent <= absorptionCurrentThreshold, voy a pasar a  ABSORPTION_CHARGE  para asegurar etapa correcta ");
        currentState = ABSORPTION_CHARGE;
      }

      break;

    case ERROR:
      // Apagar todo
      digitalWrite(LOAD_CONTROL_PIN, LOW);
      digitalWrite(LED_SOLAR, LOW);
      setPWM(0);
      Serial.println("Error en el sistema de carga.");
      while (true);
      break;
  }
}

// ---------------------------------------------------------------------------
// CONTROL BULK: Subir PWM hasta ~14.4 V, limitando corriente a 10 A
// ---------------------------------------------------------------------------
void bulkControl(float batteryVoltage, float chargeCurrent) {
  if (chargeCurrent > maxChargeCurrent) {
    adjustPWM(-5); // reducir PWM si supera 10A
  }
  else if (batteryVoltage < bulkVoltage) {
    adjustPWM(+1); // subir PWM para alcanzar 14.4V
  } else {
    // si estamos cerca del voltaje objetivo, reducimos un poco
    adjustPWM(-1);
  }
}

// ---------------------------------------------------------------------------
// CONTROL ABSORPTION: Mantener ~14.4 V, sin pasar 10 A
// ---------------------------------------------------------------------------
void absorptionControl(float batteryVoltage, float chargeCurrent) {
  if (batteryVoltage > absorptionVoltage) {
    adjustPWM(-1);
  }
  else if (batteryVoltage < absorptionVoltage) {
    if (chargeCurrent < maxChargeCurrent) {
      adjustPWM(+1);
    } else {
      adjustPWM(-2);
    }
  }
}

// ---------------------------------------------------------------------------
// CONTROL FLOAT: Mantener ~13.6 V
// ---------------------------------------------------------------------------
void floatControl(float batteryVoltage) {
  if (batteryVoltage > floatVoltage) {
    adjustPWM(-1);
  } else if (batteryVoltage < floatVoltage) {
    adjustPWM(+1);
  }
}

// ---------------------------------------------------------------------------
// AJUSTE INCREMENTAL DEL PWM (0 - 255), SIN INVERSIÓN AQUÍ
// ---------------------------------------------------------------------------
void adjustPWM(int step) {
  currentPWM += step;
  currentPWM = constrain(currentPWM, 0, 255);
  setPWM(currentPWM);
}

// ---------------------------------------------------------------------------
// APLICACIÓN DEL PWM "INVERTIDO" (requerimiento hardware)
// ---------------------------------------------------------------------------
void setPWM(int pwmValue) {
  // Aseguramos límites
  pwmValue = constrain(pwmValue, 0, 255);

  // Cálculo de duty cycle invertido (como en el script original)
  // Primero convertimos pwmValue (0..255) a %,
  // luego invertimos y volvemos a 0..255
  int dutyCyclePercentage = map(pwmValue, 0, 255, 0, 100);
  int invertedDutyCycle   = 255 - (dutyCyclePercentage * 255 / 100);

  ledcWrite(pwmPin, 255-pwmValue);

  // (Opcional) para depuración
  Serial.print("PWM calculado: ");
  Serial.print(pwmValue);
  Serial.print(" (");
  Serial.print(dutyCyclePercentage);
  Serial.print("%), invertido -> ");
  Serial.println(invertedDutyCycle);
}

// ---------------------------------------------------------------------------
// FUNCIÓN PARA OBTENER EL ESTADO DE CARGA COMO CADENA DE TEXTO
// ---------------------------------------------------------------------------
String getChargeStateString(ChargeState state) {
  switch (state) {
    case BULK_CHARGE:
      return "BULK_CHARGE";
    case ABSORPTION_CHARGE:
      return "ABSORPTION_CHARGE";
    case FLOAT_CHARGE:
      return "FLOAT_CHARGE";
    case ERROR:
      return "ERROR";
    default:
      return "UNKNOWN";
  }
}

// ---------------------------------------------------------------------------
// FUNCIÓN PARA OBTENER LOS DATOS EN FORMATO JSON
// ---------------------------------------------------------------------------
String getData() {
  String json = "{";
  json += "\"panelToBatteryCurrent\": " + String(panelToBatteryCurrent) + ",";
  json += "\"batteryToLoadCurrent\": " + String(batteryToLoadCurrent) + ",";
  json += "\"voltagePanel\": " + String(ina219_1.getBusVoltage_V()) + ",";
  json += "\"voltageBateriaEnSensor2\": " + String(ina219_2.getBusVoltage_V()) + ",";
  json += "\"chargeState\": \"" + getChargeStateString(currentState) + "\",";
  json += "\"bulkVoltage\": " + String(bulkVoltage) + ",";
  json += "\"currentPWM\": " + String(currentPWM) + ",";
  json += "\"LVD\": " + String(LVD) + ",";
  json += "\"LVR\": " + String(LVR) + ",";
  json += "\"absorptionCurrentThreshold\": " + String(absorptionCurrentThreshold) + ",";
  json += "\"batteryCapacity\": " + String(batteryCapacity) + ",";
  json += "\"thresholdPercentage\": " + String(thresholdPercentage);
  json += "}";
  return json;
}

// ---------------------------------------------------------------------------
// FUNCIÓN PARA OBTENER EL HTML DE LA INTERFAZ WEB
// ---------------------------------------------------------------------------
String getHTML() {
  // Encabezado básico, vista móvil y estilos
  String html = "<!DOCTYPE html><html lang='es'>";
  html += "<head>";
  html += "<meta charset='UTF-8'>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  html += "<title>Cargador</title>";

  // Estilos CSS
  html += "<style>";
  // Estructura general
  html += "body { font-family: Arial, sans-serif; margin: 0; padding: 0; background-color: #f0f0f0; }";
  html += ".container { max-width: 800px; margin: 0 auto; padding: 20px; }";
  html += "h1 { text-align: center; margin-bottom: 20px; }";
  html += "h2 { text-align: center; margin-bottom: 20px; }";

  // Tabla
  html += ".table-wrap { overflow-x: auto; margin-bottom: 20px; }";  // contenedor para scroll horizontal en móviles
  html += "table { width: 100%; border-collapse: collapse; min-width: 400px; background-color: #fff; box-shadow: 0 0 10px rgba(0, 0, 0, 0.1); }";
  html += "th, td { border: 1px solid #ddd; padding: 8px; text-align: left; }";
  html += "th { background-color: #f2f2f2; }";
  html += "tr:nth-child(even) { background-color: #fafafa; }";

  // Formulario
  html += ".form-container { background-color: #fff; padding: 20px; box-shadow: 0 0 10px rgba(0, 0, 0, 0.1); margin-bottom: 20px; }";
  html += ".form-group { margin-bottom: 15px; }";
  html += ".form-group label { display: block; margin-bottom: 5px; }";
  html += ".form-group input { width: 100%; padding: 8px; box-sizing: border-box; }";
  html += ".form-group input[type='submit'] { background-color: #4CAF50; color: white; border: none; cursor: pointer; }";
  html += ".form-group input[type='submit']:hover { background-color: #45a049; }";

  // Para ver cuando un valor cambia (opcional)
  html += ".changed { background-color: #d7ffd7; transition: background-color 1s ease; }";
  html += "</style>";
  html += "</head>";

  html += "<body>";
  html += "<div class='container'>";
  html += "<h1>Estado del Cargador</h1>";

  // Contenedor responsive para la tabla
  html += "<div class='table-wrap'>";
  html += "<table>";
  html += "<tr><th>Parámetro</th><th>Valor</th></tr>";

  // Mismos campos e IDs
  html += "<tr><td>Corriente Panel a Batería</td><td id='panelToBatteryCurrent'>-</td></tr>";
  html += "<tr><td>Corriente Batería a Carga</td><td id='batteryToLoadCurrent'>-</td></tr>";
  html += "<tr><td>Voltaje Panel</td><td id='voltagePanel'>-</td></tr>";
  html += "<tr><td>Voltaje Batería</td><td id='voltageBateriaEnSensor2'>-</td></tr>";
  html += "<tr><td>Estado de Carga</td><td id='chargeState'>-</td></tr>";
  html += "<tr><td>Voltaje Etapa BULK</td><td id='bulkVoltage'>-</td></tr>";
  html += "<tr><td>PWM Actual</td><td id='currentPWM'>-</td></tr>";
  html += "<tr><td>LVD</td><td id='LVD'>-</td></tr>";
  html += "<tr><td>LVR</td><td id='LVR'>-</td></tr>";
  html += "<tr><td>Umbral de Corriente</td><td id='absorptionCurrentThreshold'>-</td></tr>";
  html += "<tr><td>Capacidad de la Batería (Ah)</td><td id='batteryCapacity'>-</td></tr>";
  html += "<tr><td>Umbral de Corriente (%)</td><td id='thresholdPercentage'>-</td></tr>";

  html += "</table>";
  html += "</div>"; // .table-wrap

  // Formulario para actualizar valores
  html += "<h2>Configuración</h2>";
  html += "<div class='form-container'>";
  html += "<form action='/update' method='POST'>";
  html += "<div class='form-group'>";
  html += "<label for='batteryCapacity'>Capacidad de la batería (Ah):</label>";
  html += "<input type='number' id='batteryCapacity' name='batteryCapacity' step='0.1' min='0' required>";
  html += "</div>";
  html += "<div class='form-group'>";
  html += "<label for='thresholdPercentage'>Umbral de corriente (%):</label>";
  html += "<input type='number' id='thresholdPercentage' name='thresholdPercentage' step='0.1' min='0.5' max='5' required>";
  html += "</div>";
  html += "<div class='form-group'>";
  html += "<input type='submit' value='Actualizar'>";
  html += "</div>";
  html += "</form>";
  html += "</div>"; // .form-container

  html += "</div>"; // .container

  // Script para fetch de datos y actualización
  html += "<script>";
  html += "function updateData() {";
  html += "  fetch('/data').then(response => response.json()).then(data => {";
  // Actualizar cada campo por ID
  html += "    updateField('panelToBatteryCurrent', data.panelToBatteryCurrent);";
  html += "    updateField('batteryToLoadCurrent', data.batteryToLoadCurrent);";
  html += "    updateField('voltagePanel', data.voltagePanel);";
  html += "    updateField('voltageBateriaEnSensor2', data.voltageBateriaEnSensor2);";
  html += "    updateField('chargeState', data.chargeState);";
  html += "    updateField('bulkVoltage', data.bulkVoltage);";
  html += "    updateField('currentPWM', data.currentPWM);";
  html += "    updateField('LVD', data.LVD);";
  html += "    updateField('LVR', data.LVR);";
  html += "    updateField('absorptionCurrentThreshold', data.absorptionCurrentThreshold);";
  html += "    updateField('batteryCapacity', data.batteryCapacity);";
  html += "    updateField('thresholdPercentage', data.thresholdPercentage);";
  html += "  });";
  html += "}";

  // Función para sobrescribir texto + resaltar cambio
  html += "function updateField(id, newValue) {";
  html += "  let el = document.getElementById(id);";
  html += "  if (el.innerText != newValue) {";
  html += "    el.innerText = newValue;";
  // Añadir clase 'changed' para animación
  html += "    el.classList.add('changed');";
  html += "    setTimeout(() => { el.classList.remove('changed'); }, 1000);";
  html += "  }";
  html += "}";

  // Llamar updateData cada segundo
  html += "setInterval(updateData, 1000);";
  html += "</script>";

  html += "</body></html>";
  return html;
}
