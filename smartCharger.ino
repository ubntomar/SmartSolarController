#include <Arduino.h>

// Pin definitions
#define BATTERY_VOLTAGE_PIN 0  // GPIO0 for battery voltage
#define CURRENT_SENSOR_PIN 3   // GPIO3 for current sensor
#define PWM_CONTROL_PIN 6      // GPIO6 for MOSFET control
#define ON_OFF_LOAD_CONTROL 7  // GPIO7 for load control

// Battery parameters
#define MAX_BATTERY_VOLTAGE 14.4  // Maximum safe battery voltage
#define MIN_BATTERY_VOLTAGE 10.5  // Minimum safe battery voltage
#define FLOAT_VOLTAGE 13.6        // Float voltage for maintenance
#define MAX_CHARGING_CURRENT 5.0  // Maximum charging current
#define FLOAT_CURRENT 0.5         // Float current for maintenance
#define MIN_CHARGING_CURRENT 0.1  // Minimum current to consider charging

// PWM control
#define PWM_RESOLUTION 8    // 8-bit resolution (0-255)

// Sampling parameters
const int numSamples = 50;  // Number of samples for smoothing
const int sampleDelay = 5;  // Delay between samples in milliseconds

// Charging states
enum ChargingState {
  BULK,
  ABSORPTION,
  FLOAT,
  IDLE,
  ERROR
};

// Global variables
ChargingState currentState = IDLE;
int pwmDutyCycle = 0;

// Function prototypes
float smoothADCReadings(int pin);
float readVoltage(int pin, float resistorRatio);
float readCurrent(int pin);
void updateChargingState(float voltage, float current);
void adjustPWM(float voltage, float current);
void safetyCheck(float voltage, float current);
void displayStatus(float voltage, float current);

void setup() {
  pinMode(BATTERY_VOLTAGE_PIN, INPUT);
  pinMode(CURRENT_SENSOR_PIN, INPUT);
  pinMode(PWM_CONTROL_PIN, OUTPUT);
  pinMode(ON_OFF_LOAD_CONTROL, OUTPUT);
  
  Serial.begin(115200);
  
  // Configure PWM
  analogWrite(PWM_CONTROL_PIN, 0);  // Start with PWM off
  
  digitalWrite(ON_OFF_LOAD_CONTROL, LOW);  // Start with load disconnected
  
  Serial.println("Battery Charge Controller Initialized");
}

void loop() {
  float batteryVoltage = readVoltage(BATTERY_VOLTAGE_PIN, 11.0);  // Resistor divider ratio: (10k + 1k) / 1k = 11
  float chargingCurrent = readCurrent(CURRENT_SENSOR_PIN);

  safetyCheck(batteryVoltage, chargingCurrent);
  updateChargingState(batteryVoltage, chargingCurrent);
  adjustPWM(batteryVoltage, chargingCurrent);
  displayStatus(batteryVoltage, chargingCurrent);

  delay(1000);  // Main loop delay
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

float readCurrent(int pin) {
  int sensorValue = smoothADCReadings(pin);
  float voltage = sensorValue * (3.3 / 4095.0);
  float offsetVoltage = 2220 * (3.3 / 4095.0);
  float sensitivity = 0.0458;
  float current = (voltage - offsetVoltage) / sensitivity;
  return (abs(current) < 0.05) ? 0 : current;
}

void updateChargingState(float voltage, float current) {
  switch (currentState) {
    case IDLE:
      if (current > MIN_CHARGING_CURRENT) {
        currentState = BULK;
        Serial.println("Solar panel connected. Starting BULK charging.");
      }
      break;
    case BULK:
      if (voltage >= MAX_BATTERY_VOLTAGE) {
        currentState = ABSORPTION;
        Serial.println("Entering ABSORPTION stage");
      } else if (current <= MIN_CHARGING_CURRENT) {
        currentState = IDLE;
        Serial.println("Insufficient charging current. Entering IDLE state.");
      }
      break;
    case ABSORPTION:
      if (current <= FLOAT_CURRENT) {
        currentState = FLOAT;
        Serial.println("Entering FLOAT stage");
      } else if (current <= MIN_CHARGING_CURRENT) {
        currentState = IDLE;
        Serial.println("Insufficient charging current. Entering IDLE state.");
      }
      break;
    case FLOAT:
      if (voltage < (FLOAT_VOLTAGE - 0.5)) {
        currentState = BULK;
        Serial.println("Re-entering BULK stage");
      } else if (current <= MIN_CHARGING_CURRENT) {
        currentState = IDLE;
        Serial.println("Insufficient charging current. Entering IDLE state.");
      }
      break;
    case ERROR:
      // Stay in ERROR state until manually reset
      break;
  }
}

void adjustPWM(float voltage, float current) {
  switch (currentState) {
    case IDLE:
      pwmDutyCycle = 0;
      break;
    case BULK:
      if (current < MAX_CHARGING_CURRENT) {
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
      if (voltage < FLOAT_VOLTAGE) {
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

void safetyCheck(float voltage, float current) {
  if (voltage > MAX_BATTERY_VOLTAGE + 0.5 || voltage < MIN_BATTERY_VOLTAGE || current > MAX_CHARGING_CURRENT + 0.5) {
    currentState = ERROR;
    pwmDutyCycle = 0;
    analogWrite(PWM_CONTROL_PIN, pwmDutyCycle);
    digitalWrite(ON_OFF_LOAD_CONTROL, LOW);  // Disconnect load
    Serial.println("SAFETY ERROR: Charging stopped");
  }
}

void displayStatus(float voltage, float current) {
  Serial.println("----------------------------------------");
  Serial.print("Battery Voltage: ");
  Serial.print(voltage, 2);
  Serial.println(" V");
  
  Serial.print("Charging Current: ");
  Serial.print(current, 2);
  Serial.println(" A");
  
  Serial.print("Charging State: ");
  switch (currentState) {
    case IDLE:
      Serial.println("IDLE");
      break;
    case BULK:
      Serial.println("BULK");
      break;
    case ABSORPTION:
      Serial.println("ABSORPTION");
      break;
    case FLOAT:
      Serial.println("FLOAT");
      break;
    case ERROR:
      Serial.println("ERROR");
      break;
  }
  
  Serial.print("PWM Duty Cycle: ");
  Serial.print((pwmDutyCycle / 255.0) * 100);
  Serial.println("%");
  Serial.println("----------------------------------------\n");
}