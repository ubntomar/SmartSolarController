# Cargador Solar con ESP32 e INA219

Este proyecto implementa un cargador solar basado en un ESP32 que:

- **Lee corriente y voltaje** de dos sensores INA219.
- **Controla** la carga de una batería en distintas etapas (Bulk, Absorción y Flotación) mediante **PWM invertido**.
- **Administra** la desconexión de la carga (Low Voltage Disconnect).
- Ofrece una **interfaz web** para monitoreo y configuración vía WiFi (AP propio).
- Incluye **Watchdog** para evitar cuelgues del sistema.

---

## Características principales

1. **Sensado de Corriente**  
   Utiliza dos sensores INA219:
   - `ina219_1 (0x40)`: mide la corriente del Panel → Batería.
   - `ina219_2 (0x41)`: mide la corriente de la Batería → Carga.

2. **Etapas de Carga**  
   - **Bulk Charge**: Carga rápida hasta ~14.4 V, con límite de corriente (10 A).
   - **Absorción**: Mantiene ~14.4 V hasta que la corriente sea inferior a cierto umbral o se cumpla un tiempo máximo.
   - **Flotación**: Mantiene ~13.6 V para evitar sobrecarga.

3. **Protección de Batería (LVD/LVR)**  
   - LVD = 12.0 V → desconecta la carga cuando la batería está demasiado baja.
   - LVR = 12.5 V → reconecta la carga cuando el voltaje de la batería se recupera.

4. **Control PWM Invertido**  
   - Frecuencia de 40 kHz.
   - Resolución de 8 bits (0–255).
   - El hardware requiere una salida invertida (se realiza en software).

5. **Interfaz Web**  
   - Crea un **Access Point** con SSID `"Cargador"` y password `"12345678"`.
   - Muestra tabla de datos (corrientes, voltajes, estado de carga, PWM, etc.).
   - Permite modificar la capacidad de la batería (Ah) y el umbral de corriente (%) para personalizar la transición a flotación.
   - Guarda valores en NVS usando `Preferences`.

6. **Watchdog**  
   - Se configura en 5 s (`WDT_TIMEOUT = 5`).
   - Reinicia el sistema si no se restablece a tiempo.

---

## Requisitos de Hardware

- **ESP32** con soporte para `ledcWrite` y WiFi.
- **2 x Sensores INA219** (direcciones 0x40 y 0x41).
- **Batería** (AGM/GEL de 12 V nominal).
- **Panel solar** acorde a la batería.
- **MOSFET o driver** para manejar la carga con PWM.
- Conexiones I2C en pines definidos (`SDA_PIN = 8`, `SCL_PIN = 9`) o adapta según tu placa.
- **Pin de control de carga**: `LOAD_CONTROL_PIN = 7`.
- **Pin LED** (opcional) para indicar presencia de corriente solar (`LED_SOLAR = 3`).

---

## Librerías necesarias

- [Adafruit INA219](https://github.com/adafruit/Adafruit_INA219)
- [ESPAsyncWebServer](https://github.com/me-no-dev/ESPAsyncWebServer)
- [AsyncTCP](https://github.com/me-no-dev/AsyncTCP) (para ESPAsyncWebServer)
- **Preferences**, **WiFi** y **esp_task_wdt** (incluidas en el core de ESP32)

---

## Configuración e instalación

1. **Clona o descarga** este repositorio en tu carpeta de proyectos (Arduino o PlatformIO).
2. **Abre** el archivo principal (por ejemplo, `main.ino` o `src/main.cpp`) en tu IDE.
3. **Instala** las librerías indicadas (si no las tienes).
4. **Selecciona** la placa adecuada (p.e. “ESP32 Dev Module”) en tu IDE.
5. **Modifica** los pines I2C y PWM si tu placa lo requiere.  
6. **Compila y sube** el firmware a tu ESP32.

---

## Uso

1. **Alimenta el sistema**: Conecta el panel solar y la batería.  
2. **Enciende el ESP32**: Creará un AP llamado `"Cargador"` (clave `"12345678"`).  
3. **Conéctate** a la red `"Cargador"` desde tu dispositivo.  
4. **Abre el navegador** en `http://192.168.4.1/`.  
5. **Monitoreo**: Verás una tabla con los valores de corriente, voltaje y estado de carga.  
6. **Configuración**:  
   - En la sección “Configuración”, puedes cambiar la capacidad de la batería y el umbral de corriente (%).  
   - Al presionar “Actualizar”, los valores se guardarán en la memoria no volátil (NVS).  
7. **Serial** (opcional): Puedes abrir el monitor serie a 115200 baudios para ver logs detallados.

---

## Estructura del código

- **Declaraciones y definiciones**: Configura pines, umbrales, frecuencias, etc.
- **`setup()`**:  
  - Inicializa Serial, I2C, INA219, servidor web, punto de acceso, Watchdog, etc.
- **`loop()`**:  
  - Reajusta el Watchdog.
  - Lee corrientes y voltajes.
  - Controla LVD/LVR.
  - Gestiona la máquina de estados de carga (Bulk, Absorción, Flotación).
  - Actualiza el PWM según condiciones.

- **Funciones principales**:
  - `getAverageCurrent()`: Lectura media de corriente en INA219.
  - `updateChargeState()`: Máquina de estados de carga.
  - `bulkControl()`, `absorptionControl()`, `floatControl()`: Lógicas de control PWM en cada etapa.
  - `setPWM()` y `adjustPWM()`: Ajustes del PWM invertido.
  - `getData()`: Devuelve datos en formato JSON para la interfaz web.
  - `getHTML()`: Devuelve la página HTML con la tabla y el formulario.

---

## Código principal

A continuación, se muestra el sketch (fragmento simplificado). El código completo está en este repositorio:

```cpp
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include "esp_task_wdt.h"
#include <Preferences.h>

// Configuraciones varias
Preferences preferences;
#define WDT_TIMEOUT 5
#define SDA_PIN 8
#define SCL_PIN 9
#define LOAD_CONTROL_PIN 7
#define LED_SOLAR 3

Adafruit_INA219 ina219_1(0x40);
Adafruit_INA219 ina219_2(0x41);

...

void setup() {
  // Inicialización de sensores, WiFi AP, Servidor, Watchdog, etc.
}

void loop() {
  // Lectura de sensores
  // Máquina de estados de carga
  // Control LVD / LVR
  // Actualización PWM
  // Servidor web asíncrono
}

...
