 * Proyecto ECO-SENSE 1.0
 * Código para la placa ESP32 (Arduino IDE)
 * Lee los sensores DHT11 (T/H) y ACS712 (Voltaje) y los envía al script de Python a través del puerto serial.

#include <Arduino.h>
#include "DHT.h" // Librería para el sensor DHT

// 1. Pines de los sensores
#define PIN_TEMP 13       // Pin G13 para el sensor DHT11
#define PIN_CORRIENTE 34  // Pin G34 para el sensor ACS712
#define TIPO_DHT DHT11

// 2. Límites de Alerta (Solo Temperatura)
const float LIMITE_TEMP_ALTA = 60.0;
const float LIMITE_TEMP_BAJA = 10.0;

// 3. Calibración (no se utiliza si V=0.00)
const int ADC_CERO = 2048;

// 4. Inicialización de Objetos
DHT sensor_dht(PIN_TEMP, TIPO_DHT); // Crea el objeto sensor
bool sensores_listos = false;

// 5. Función de Setup (se ejecuta una vez)
void setup() {
    // Fija la velocidad del puerto USB (debe coincidir con Python)
    Serial.begin(115200);
    
    // Inicia el sensor DHT
    sensor_dht.begin();
    
    Serial.println(F("Iniciando sensores T, H y V (Arduino C++)..."));
    Serial.println(F("Sensores listos. Enviando datos..."));
    Serial.printf("Límites de Alerta (solo Temp): T > %.0fC, T <= %.0fC\n", LIMITE_TEMP_ALTA, LIMITE_TEMP_BAJA);
    sensores_listos = true;
}

// 6. Función para leer Voltaje (ACS712)
float leer_voltaje_acs712() {
    long lectura_adc = 0;
    // Promedio de 20 lecturas para estabilizar
    for (int i = 0; i < 20; i++) {
        lectura_adc += analogRead(PIN_CORRIENTE);
    }
    float adc_promedio = (float)lectura_adc / 20.0;
    
    // Convertimos ADC (0-4095) a Voltaje (0-3.3V)
    float voltaje = (adc_promedio / 4095.0) * 3.3;
    return voltaje;
}

// 7. Bucle Principal (se repite)
void loop() {
    if (!sensores_listos) {
        delay(1000);
        return;
    }

    try {
        // --- LECTURA DE 3 VALORES ---
        float temp = sensor_dht.readTemperature(); // T° en Celsius
        float hum = sensor_dht.readHumidity();     // Humedad en %
        float volt = leer_voltaje_acs712();        // Voltaje en V
        
        // isnan() chequea si la lectura del DHT falló
        if (isnan(temp) || isnan(hum)) {
            Serial.println(F("ErrorT_H")); // Error de Temp o Humedad
        } else {
            // --- Lógica de Alertas (solo por Temp) ---
            if (temp >= LIMITE_TEMP_ALTA) {
                Serial.printf("ALERTA_ALTA:T=%.1f,H=%.0f,V=%.2f\n", temp, hum, volt);
            } else if (temp <= LIMITE_TEMP_BAJA) {
                Serial.printf("ALERTA_BAJA:T=%.1f,H=%.0f,V=%.2f\n", temp, hum, volt);
            } else {
                // Envía el formato estándar "OK"
                Serial.printf("OK:T=%.1f,H=%.0f,V=%.2f\n", temp, hum, volt);
            }
        }
    } catch (const std::exception& e) {
        Serial.printf("Error en bucle: %s\n", e.what());
    }
    
    // El DHT11 necesita al menos 2 seg (2000ms) entre lecturas
    // para evitar errores y sincronizar con el 'timeout=3' de Python.
    delay(2000); 
}
