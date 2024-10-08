#include <Arduino.h>

// Pin definitions
#define BATTERY_VOLTAGE_PIN 0   // GPIO0 for battery voltage
#define SOLAR_CURRENT_PIN 3     // GPIO3 for solar panel current
#define LOAD_CURRENT_PIN 2      // GPIO2 for load current
#define PWM_CONTROL_PIN 6       // GPIO6 for MOSFET control
#define ON_OFF_LOAD_CONTROL 7   // GPIO7 for load control

// Thresholds and limits
#define CURRENT_SENSOR_THRESHOLD 0.1  // Threshold below which current is considered 0
#define LOAD_CURRENT_THRESHOLD 0.1    // Threshold below which load current is considered 0
#define MAX_LOAD_CURRENT 10.0         // Maximum expected load current
#define MAX_SAFE_LOAD_CURRENT 15.0    // Maximum safe current for the load
#define LOAD_DISCONNECT_DELAY 5000    // Delay before disconnecting load (5 seconds)

// Battery parameters
#define MAX_BATTERY_VOLTAGE 14.4  // Maximum safe battery voltage
#define MIN_BATTERY_VOLTAGE 10.5  // Minimum safe battery voltage
#define FLOAT_VOLTAGE 13.6        // Float voltage for maintenance
#define MAX_CHARGING_CURRENT 5.0  // Maximum charging current
#define FLOAT_CURRENT_BATTERY 0.5 // Float current for battery maintenance
#define MIN_CHARGING_CURRENT 0.1  // Minimum current to consider charging

// Load reconnection voltage
#define LOAD_RECONNECT_VOLTAGE (MIN_BATTERY_VOLTAGE + 0.5)  // 0.5V above MIN_BATTERY_VOLTAGE
// PWM control
#define PWM_RESOLUTION 8    // 8-bit resolution (0-255)


// Sampling parameters
const int numSamples = 50;  // Number of samples for smoothing
const int sampleDelay = 5;  // Delay between samples in milliseconds

// Charging states
enum ChargingState {
  IDLE,
  BULK,
  ABSORPTION,
  FLOAT,
  ERROR
};
// Error types
enum ErrorType {
  NO_ERROR,
  OVERVOLTAGE,
  UNDERVOLTAGE,
  OVERCHARGE,
  OVERDISCHARGE,
  UNKNOWN_ERROR
};
// Global variables
ChargingState currentState = IDLE;
int pwmDutyCycle = 0;
ErrorType currentError = NO_ERROR;
unsigned long lastOvercurrentTime = 0;
bool loadEnabled = true;  // Inicializamos con la carga habilitada




// Function prototypes
float smoothADCReadings(int pin);
float readVoltage(int pin, float resistorRatio);
float readCurrent(int pin);
void updateChargingState(float voltage, float solarCurrent, float loadCurrent);
void adjustPWM(float voltage, float solarCurrent, float loadCurrent);
void safetyCheck(float voltage, float solarCurrent, float loadCurrent);
void controlLoad(float voltage, float solarCurrent, float loadCurrent);
void displayStatus(float voltage, float solarCurrent, float loadCurrent);
String getStateString(ChargingState state);

void setup() {
  pinMode(BATTERY_VOLTAGE_PIN, INPUT);
  pinMode(SOLAR_CURRENT_PIN, INPUT);
  pinMode(LOAD_CURRENT_PIN, INPUT);
  pinMode(PWM_CONTROL_PIN, OUTPUT);
  pinMode(ON_OFF_LOAD_CONTROL, OUTPUT);
  
  Serial.begin(115200);
  
  analogWrite(PWM_CONTROL_PIN, 0);  // Start with PWM off
  digitalWrite(ON_OFF_LOAD_CONTROL, HIGH);  // Start with load connected
  

  Serial.println("Improved Solar Charge Controller for 24/7 Communication Systems Initialized");
  Serial.print("Load will disconnect at: ");
  Serial.print(MIN_BATTERY_VOLTAGE);
  Serial.println(" V");
  Serial.print("Load will reconnect at: ");
  Serial.print(LOAD_RECONNECT_VOLTAGE);
  Serial.println(" V");
}

void loop() {
  float batteryVoltage = readVoltage(BATTERY_VOLTAGE_PIN, 11.0);
  float solarCurrent = readSolarCurrent(SOLAR_CURRENT_PIN);
  float loadCurrent = readLoadCurrent(LOAD_CURRENT_PIN);
  
  safetyCheck(batteryVoltage, solarCurrent, loadCurrent);
  updateChargingState(batteryVoltage, solarCurrent, loadCurrent);
  adjustPWM(batteryVoltage, solarCurrent, loadCurrent);
  controlLoad(batteryVoltage, solarCurrent, loadCurrent);
  displayStatus(batteryVoltage, solarCurrent, loadCurrent);

  delay(5000);  // Main loop delay
}

float smoothADCReadings(int pin) {
  long total = 0;
  for (int i = 0; i < numSamples; i++) {
    total += analogRead(pin);
    delay(sampleDelay);
  }
  return total / numSamples;
}

float readVoltage(int pin, float resistorRatio) {
  int sensorValue = smoothADCReadings(pin);
  float voltage = sensorValue * (3.3 / 4095.0);  // Convert ADC reading to voltage

  // Dynamic calibration based on multiple data points
  float correctionFactor;
  if (sensorValue <= 1469) {
    correctionFactor = 12.0 / (voltage * resistorRatio);
  } else if (sensorValue <= 1532) {
    correctionFactor = ((12.5 - 12.0) / (1532 - 1469)) * (sensorValue - 1469) + 12.0;
    correctionFactor /= (voltage * resistorRatio);
  } else if (sensorValue <= 1597) {
    correctionFactor = ((13.0 - 12.5) / (1597 - 1532)) * (sensorValue - 1532) + 12.5;
    correctionFactor /= (voltage * resistorRatio);
  } else if (sensorValue <= 1661) {
    correctionFactor = ((13.5 - 13.0) / (1661 - 1597)) * (sensorValue - 1597) + 13.0;
    correctionFactor /= (voltage * resistorRatio);
  } else if (sensorValue <= 1725) {
    correctionFactor = ((14.0 - 13.5) / (1725 - 1661)) * (sensorValue - 1661) + 13.5;
    correctionFactor /= (voltage * resistorRatio);
  } else if (sensorValue <= 1789) {
    correctionFactor = ((14.5 - 14.0) / (1789 - 1725)) * (sensorValue - 1725) + 14.0;
    correctionFactor /= (voltage * resistorRatio);
  } else if (sensorValue <= 1853) {
    correctionFactor = ((15.0 - 14.5) / (1853 - 1789)) * (sensorValue - 1789) + 14.5;
    correctionFactor /= (voltage * resistorRatio);
  } else {
    correctionFactor = 15.0 / (voltage * resistorRatio);
  }

  return voltage * resistorRatio * correctionFactor;
}

float readSolarCurrent(int pin) {
  int sensorValue = smoothADCReadings(pin);
  float voltage = sensorValue * (3.3 / 4095.0);
  float offsetVoltage = 2220 * (3.3 / 4095.0);
  float sensitivity = 0.0458;
  float current = (voltage - offsetVoltage) / sensitivity;
  
  return max(current, 0.0f);
}

float readLoadCurrent(int pin) {
  float adcValue = smoothADCReadings(pin);
  float referenceVoltage = 3.3;
  float voltage = adcValue * (referenceVoltage / 4095.0); // ADC de 12 bits

  // Compensación personalizada: ajustar el voltaje para que coincida con el valor sin carga
  float calibrationFactor = 1.840 / 2.0396;  // Comparar el valor observado con el valor esperado
  voltage *= calibrationFactor;

  // Calcular la corriente basada en la tensión medida y la sensibilidad del sensor
  float offsetVoltage = 1.840; // Offset ajustado para 0A
  float sensitivity = 0.066;   // Sensibilidad de 66 mV/A para ACS712-30A

  float current = (voltage - offsetVoltage) / sensitivity;

  // Asegurarse de que la corriente no sea negativa
  return max(current, 0.0f);
}


void updateChargingState(float voltage, float solarCurrent, float loadCurrent) {
  float batteryCurrent = solarCurrent - loadCurrent;
  ChargingState previousState = currentState;
  
  if (currentState == ERROR) {
    // Permanece en ERROR hasta que se resuelva manualmente
    return;
  }

  if (solarCurrent <= CURRENT_SENSOR_THRESHOLD) {
    // Si no hay corriente solar, asumimos que es de noche
    currentState = IDLE;
  } else if (voltage >= FLOAT_VOLTAGE && batteryCurrent <= FLOAT_CURRENT_BATTERY) {
    currentState = FLOAT;
  } else if (voltage >= MAX_BATTERY_VOLTAGE) {
    currentState = ABSORPTION;
  } else if (batteryCurrent > MIN_CHARGING_CURRENT) {
    currentState = BULK;
  } else {
    currentState = IDLE;
  }

  if (currentState != previousState) {
    Serial.print("Charging state changed to: ");
    Serial.println(getStateString(currentState));
  }
}

void adjustPWM(float voltage, float solarCurrent, float loadCurrent) {
  float batteryCurrent = solarCurrent - loadCurrent;
  
  switch (currentState) {
    case IDLE:
      pwmDutyCycle = 0;
      break;
    case BULK:
      if (batteryCurrent < MAX_CHARGING_CURRENT) {
        pwmDutyCycle = min(pwmDutyCycle + 1, 255);
      } else {
        pwmDutyCycle = max(pwmDutyCycle - 1, 0);
      }
      break;
    case ABSORPTION:
      if (voltage < MAX_BATTERY_VOLTAGE) {
        pwmDutyCycle = min(pwmDutyCycle + 1, 255);
      } else {
        pwmDutyCycle = max(pwmDutyCycle - 1, 0);
      }
      break;
    case FLOAT:
      // Ajustamos el PWM considerando tanto la corriente de la batería como la de la carga
      if (voltage < FLOAT_VOLTAGE || batteryCurrent + loadCurrent < FLOAT_CURRENT_BATTERY) {
        pwmDutyCycle = min(pwmDutyCycle + 1, 255);
      } else {
        pwmDutyCycle = max(pwmDutyCycle - 1, 0);
      }
      break;
    case ERROR:
      pwmDutyCycle = 0;
      break;
  }
  
  analogWrite(PWM_CONTROL_PIN, pwmDutyCycle);
}

void safetyCheck(float voltage, float solarCurrent, float loadCurrent) {
  float batteryCurrent = solarCurrent - loadCurrent;
  
  if (voltage > MAX_BATTERY_VOLTAGE + 0.5) {
    currentState = ERROR;
    currentError = OVERVOLTAGE;
  } else if (voltage < MIN_BATTERY_VOLTAGE) {
    currentState = ERROR;
    currentError = UNDERVOLTAGE;
  } else if (batteryCurrent > MAX_CHARGING_CURRENT + 0.5) {
    currentState = ERROR;
    currentError = OVERCHARGE;
  } else if (batteryCurrent < -MAX_CHARGING_CURRENT) {
    // Check if the overcurrent condition has persisted
    if (lastOvercurrentTime == 0) {
      lastOvercurrentTime = millis();
    } else if (millis() - lastOvercurrentTime > LOAD_DISCONNECT_DELAY) {
      currentState = ERROR;
      currentError = OVERDISCHARGE;
    }
  } else {
    lastOvercurrentTime = 0;
    currentError = NO_ERROR;
    return;  // Exit the function if there's no error
  }

  // If we've reached this point, there's an error
  pwmDutyCycle = 0;
  analogWrite(PWM_CONTROL_PIN, pwmDutyCycle);
  digitalWrite(ON_OFF_LOAD_CONTROL, LOW);  // Disconnect load
  
  Serial.println("SAFETY ERROR: " + getErrorString(currentError));
  Serial.println("Charging stopped and load disconnected");
}

void controlLoad(float voltage, float solarCurrent, float loadCurrent) {
  if (currentState == ERROR) {
    if (loadEnabled) {
      digitalWrite(ON_OFF_LOAD_CONTROL, LOW);     // Desconectar carga en caso de error
      loadEnabled = false;
      Serial.println("Load disconnected due to system error");
    }
  } else if (voltage <= MIN_BATTERY_VOLTAGE) {
    if (loadEnabled) {
      digitalWrite(ON_OFF_LOAD_CONTROL, LOW);     // Desconectar carga si el voltaje alcanza el mínimo
      loadEnabled = false;
      Serial.println("Load disconnected due to minimum battery voltage reached");
    }
  } else if (loadCurrent > MAX_SAFE_LOAD_CURRENT) {
    if (loadEnabled) {
      digitalWrite(ON_OFF_LOAD_CONTROL, LOW);     // Desconectar carga si la corriente es excesiva
      loadEnabled = false;
      Serial.println("Load disconnected due to excessive current draw");
    }
  } else if (!loadEnabled && voltage >= LOAD_RECONNECT_VOLTAGE) {
    digitalWrite(ON_OFF_LOAD_CONTROL, HIGH);      // Reconectar carga si las condiciones son seguras
    loadEnabled = true;
    Serial.println("Load reconnected");
  } else if (!loadEnabled && voltage > MIN_BATTERY_VOLTAGE) {
    // Asegurarse de que la carga esté conectada si el voltaje está por encima del mínimo
    digitalWrite(ON_OFF_LOAD_CONTROL, HIGH);
    loadEnabled = true;
    Serial.println("Load connected as voltage is above minimum");
  }

  // Actualizar el estado de la carga cada vez que se llama a esta función
  digitalWrite(ON_OFF_LOAD_CONTROL, loadEnabled ? HIGH : LOW);
}


void displayStatus(float voltage, float solarCurrent, float loadCurrent) {
  float batteryCurrent = solarCurrent - loadCurrent;
  
  Serial.println("----------------------------------------");
  Serial.print("Battery Voltage: ");
  Serial.print(voltage, 2);
  Serial.println(" V");
  
  Serial.print("Solar Current: ");
  Serial.print(solarCurrent, 2);
  Serial.println(" A (100%)");
  
  // Calculamos y mostramos el porcentaje para la corriente de carga
  float loadPercentage = (solarCurrent > 0) ? (loadCurrent / solarCurrent) * 100 : 0;
  Serial.print("Load Current: ");
  Serial.print(loadCurrent, 2);
  Serial.print(" A (");
  Serial.print(loadPercentage, 1);
  Serial.println("%)");
  
  // Calculamos y mostramos el porcentaje para la corriente de la batería
  float batteryPercentage = (solarCurrent > 0) ? (batteryCurrent / solarCurrent) * 100 : 0;
  Serial.print("Battery Current: ");
  Serial.print(batteryCurrent, 2);
  Serial.print(" A (");
  Serial.print(batteryPercentage, 1);
  Serial.println("%)");
  
  Serial.print("Charging State: ");
  Serial.println(getStateString(currentState));
  
  if (currentState == ERROR) {
    Serial.print("Error Type: ");
    Serial.println(getErrorString(currentError));
  }
  
  Serial.print("PWM Duty Cycle: ");
  Serial.print((pwmDutyCycle / 255.0) * 100);
  Serial.println("%");
  
  Serial.print("Load Status: ");
  Serial.println(loadEnabled ? "ON" : "OFF");
  
  Serial.println("----------------------------------------\n");
}

String getErrorString(ErrorType error) {
  switch (error) {
    case NO_ERROR: return "No Error";
    case OVERVOLTAGE: return "Overvoltage";
    case UNDERVOLTAGE: return "Undervoltage";
    case OVERCHARGE: return "Overcharge";
    case OVERDISCHARGE: return "Overdischarge";
    default: return "Unknown Error";
  }
}

String getStateString(ChargingState state) {
  switch (state) {
    case IDLE: return "IDLE";
    case BULK: return "BULK";
    case ABSORPTION: return "ABSORPTION";
    case FLOAT: return "FLOAT";
    case ERROR: return "ERROR";
    default: return "UNKNOWN";
  }
}