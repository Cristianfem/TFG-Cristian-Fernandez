#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_SHT31.h>
#include <Adafruit_VEML7700.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "INA236.h"
#include <TinyGPS++.h>
#include "loramac.h"
#include "LoRaBoards.h"
#include "lmic.h"

// Configuración de Pines 
#define ONE_WIRE_BUS 15

#define GPS_RX_PIN 34
#define GPS_TX_PIN 12
#define GPS_BAUDRATE 9600

// Tiempos y Umbrales 
const long interval_tx_active = 5 * 60 * 1000; // 5 minutos entre envíos cuando está cargando
const int sleepMinutes = 15;                   // 15 min de sueño si está en modo ahorro (sin carga)

// Umbral de irradiancia para modo noche (W/m^2). Modificado a 1.0 según requerimiento.
const float irradiance_limit = 1.0; 
const float K_LUX_A_W_M2 = 100;

// Lógica de Sueño Nocturno Incremental
const int nightSleepIntervals[] = {5, 15, 30}; // Tiempos escalonados en minutos
RTC_DATA_ATTR int nightSleepIndex = 0;             // Índice guardado en memoria persistente

// Objetos
Adafruit_SHT31 sht31 = Adafruit_SHT31();
Adafruit_VEML7700 veml = Adafruit_VEML7700();
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
INA236 INA(0x40);
TinyGPSPlus gps;

bool sht31Connected = false;
bool vemlConnected = false;
bool ds18b20Connected = false;
bool ina236Connected = false;

RTC_DATA_ATTR bool isFirstSendOfDay = true;
int state = 1; 
RTC_DATA_ATTR bool Signal = true;

// Temporizadores para el Modo Carga
unsigned long Last_Tx = 0;
unsigned long Last_Log = 0;

// Prototipos
int CheckState();
void ErrorMode();

// Función: Conversión Lux a Irradiancia
float calculateIrradiance(float lux) {
    if (isnan(lux)) return 0.0;
    float lux_corregido = lux;
    if (lux > 1000.0) {
        lux_corregido = lux * (1.0023 + lux * (8.1488e-5 + lux * (-9.3924e-9 + lux * 6.0135e-13)));
    }
    float irradiance = lux_corregido / K_LUX_A_W_M2;
    return (irradiance < 0) ? 0.0 : irradiance; 
}

void DeepSleep(int minutesToSleep) {
    Serial.print("Entrando en sueño profundo por ");
    Serial.print(minutesToSleep);
    Serial.println(" minutos...");
    Serial.flush(); 
    uint64_t timeToSleep_uS = (uint64_t)minutesToSleep * 60 * 1000000;
    esp_sleep_enable_timer_wakeup(timeToSleep_uS); 
    esp_deep_sleep_start();
}

void waitforLora() {
    Serial.println("Esperando a que LoRa termine el envío... ");
    while (LMIC.opmode & OP_TXRXPEND) {
        loopLMIC();
        delay(10); 
    }
    Serial.println("Envio LoRa finalizado");
}

void smartDelay(unsigned long ms) {
    unsigned long start = millis();
    while (millis() - start < ms) {
        loopLMIC();
        delay(10);
    }
}

void setupSensors() {
    Serial.println(F("--- Inicializando Sensores ---"));
    if (sht31.begin(0x44)) {   
        sht31Connected = true;
        Serial.println("SHT31... OK");
    } else Serial.println("SHT31... fallo");

    if (veml.begin()) {
        vemlConnected = true;
        Serial.println("VEML7700... OK");
        veml.setGain(VEML7700_GAIN_1_8);
        veml.setIntegrationTime(VEML7700_IT_25MS);
    } else Serial.println("VEML7700... fallo");

    sensors.begin();
    if (sensors.getDeviceCount() > 0) {
        ds18b20Connected = true;
        Serial.print("DS18B20... OK ("); Serial.print(sensors.getDeviceCount()); Serial.println(" detectados)");
    } else Serial.println("DS18B20... fallo");

    if (INA.begin()) {
        INA.setMaxCurrentShunt(1, 0.001); 
        ina236Connected = true;
        Serial.println("INA236... OK");
    } else Serial.println("INA236... fallo");
    Serial.println(F("------------------------------"));
}


// Envía todos los datos en formato binario (14 bytes)

void sendAllData() {
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println("Radio ocupada, envío cancelado.");
        return;
    }

    Serial.println("\n--- Recopilando TODOS los datos (Binario) ---");

    float temp_sht = sht31Connected ? sht31.readTemperature() : 0.0;
    float hum = sht31Connected ? sht31.readHumidity() : 0.0;
    
    float temp_ds18b20 = 0.0;
    if (ds18b20Connected) {
        sensors.requestTemperatures();
        temp_ds18b20 = sensors.getTempCByIndex(0);
    }

    float lux = vemlConnected ? veml.readLux() : 0.0;
    float irradiancia = calculateIrradiance(lux);

    float bus_voltage = ina236Connected ? INA.getBusVoltage() : 0.0;
    float current = ina236Connected ? INA.getCurrent_mA() : 0.0;
    
    bool cargando = PMU->isCharging();
    float battPercent = PMU->getBatteryPercent();
    
    state = CheckState();
    if (state == 2 && Signal == true) state = 1;

    //  Construcción del payload (14 BYTES) 
    uint8_t payload[14];

    int16_t t_sht_int = round(temp_sht * 100);
    payload[0] = t_sht_int >> 8; payload[1] = t_sht_int & 0xFF;

    uint16_t hum_int = round(hum * 10);
    payload[2] = hum_int >> 8; payload[3] = hum_int & 0xFF;

    int16_t t_ds_int = round(temp_ds18b20 * 100);
    payload[4] = t_ds_int >> 8; payload[5] = t_ds_int & 0xFF;

    uint16_t irr_int = round(irradiancia * 10);
    payload[6] = irr_int >> 8; payload[7] = irr_int & 0xFF;

    uint16_t v_bus_int = round(bus_voltage * 100);
    payload[8] = v_bus_int >> 8; payload[9] = v_bus_int & 0xFF;

    int16_t current_int = round(current * 10);
    payload[10] = current_int >> 8; payload[11] = current_int & 0xFF;

    payload[12] = round(battPercent);

    uint8_t flags = 0;
    flags |= (state & 0x0F);
    if (cargando) flags |= (1 << 4);
    payload[13] = flags;

    Serial.print("Payload BIN enviado: ");
    for (int i = 0; i < 14; i++) {
        if (payload[i] < 16) Serial.print("0");
        Serial.print(payload[i], HEX); Serial.print(" ");
    }
    Serial.println();

    LMIC_setTxData2(1, payload, sizeof(payload), 0);
    waitforLora();
}

void sendGPSData(unsigned long timeout_ms) {
    Serial.println("Iniciando lectura de GPS...");
    float lat = 0.0, lng = 0.0;
    uint32_t date_val = 0; 
    uint32_t time_val = 0; 
    bool gps_valid = false;
    unsigned long startTime = millis();
    
    while (millis() - startTime < timeout_ms) {
        while (Serial1.available() > 0) { 
            if (gps.encode(Serial1.read())) {
                if (gps.location.isValid() && !gps_valid) {
                    lat = gps.location.lat();
                    lng = gps.location.lng();
                    gps_valid = true;
                }
                if (gps.date.isValid()) date_val = gps.date.value();
                if (gps.time.isValid()) time_val = gps.time.value();
            }
        }
        if (gps_valid && date_val > 0 && time_val > 0) break;
    }

    state = CheckState();

    if (state == 2 || state== 3) {
        if (gps_valid) {
            Signal = false;
            Serial.print("Ubicación GPS: "); Serial.print(lat, 6); Serial.print(", "); Serial.println(lng, 6);
        } else {
            Serial.println("GPS no válido. Usando defecto.");
            lat = 40.322; lng = -3.865;
            if(state==2) state = 1; 
            if(state==3) state = 3; 
            Signal = true;
        }
    } 

    waitforLora();
    char payload_gps_1[60];
    if (gps_valid && date_val > 0) {
        snprintf(payload_gps_1, sizeof(payload_gps_1), "GPS1_la:%.5f,lo:%.5f,D:%lu,T:%lu,st:%d", lat, lng, date_val, time_val, state);
    } else {
        snprintf(payload_gps_1, sizeof(payload_gps_1), "GPS1_la:%.5f,lo:%.5f,st:%d", lat, lng, state);
    }
    
    LMIC_setTxData2(1, (uint8_t*)payload_gps_1, strlen(payload_gps_1), 0);
    waitforLora();
}

// Función: Modo Noche (Con espera incremental)
void NightMode() {
    Serial.println("Entrando en modo Noche (Irradiancia <= límite).");
    isFirstSendOfDay = true;
    waitforLora(); 
    
    
    // Enviamos el paquete de datos antes de dormir
    sendAllData(); 
    waitforLora(); // esperar a que la radio termine antes de cortar la energía
    

    // Obtenemos los minutos del array según el índice actual
    int minsToSleep = nightSleepIntervals[nightSleepIndex];
    
    Serial.print("-> Nivel de sueño incremental: ");
    Serial.print(nightSleepIndex);
    Serial.print(" (Dormirá "); Serial.print(minsToSleep); Serial.println(" minutos)");
    
    // Incrementamos el nivel para la próxima vez (Máximo se quedará en el nivel 3 -> 60 minutos)
    if (nightSleepIndex < 2) {
        nightSleepIndex++;
    }
    
    DeepSleep(minsToSleep); 
}

// Función: Modo Día cargando 
void DayMode_Charge(float irradiancia) {
    if (CheckState() == 0) ErrorMode();
    
    unsigned long currentMillis = millis();
    
    // Si ha pasado el intervalo, enviamos todo de una sola vez
    if (Last_Tx == 0 || (currentMillis - Last_Tx >= interval_tx_active)) {
        sendAllData();
        Last_Tx = currentMillis; 
    }
    
    if (currentMillis - Last_Log >= 60000) {
        Last_Log = currentMillis;
        Serial.println("=== Modo Dia Cargando ===");
        Serial.print("Irradiancia: "); Serial.print(irradiancia, 2); Serial.println(" W/m^2");
        Serial.print("Batería: "); Serial.print(PMU->getBatteryPercent()); Serial.println("%");

        long time_to_tx = (interval_tx_active - (currentMillis - Last_Tx)) / 60000;
        Serial.print("Próximo envío en ~"); Serial.print(time_to_tx < 0 ? 0 : time_to_tx); Serial.println(" min.");
        Serial.println("=========================");
    }
    
    delay(100);
}

// Función: Modo Día - Ahorro (Envía y Duerme)
void DayMode_NoCharge() {
    Serial.println("Modo Ahorro (No carga o <50% Bat).");
    
    waitforLora();
    sendAllData(); // Mandamos todo el paquete en milisegundos
    waitforLora();

    int tiempoSuenoActual = sleepMinutes; // 15 minutos por defecto
    
    if (CheckState() == 3) { 
        tiempoSuenoActual = 15; // Si la batería es crítica
        Serial.println("-> Alerta: Batería baja (<20%). Aplicando 15 minutos de sueño.");
    }

    DeepSleep(tiempoSuenoActual);
}

int CheckState() {
    float chk_t = sht31Connected ? sht31.readTemperature() : NAN;
    float chk_h = sht31Connected ? sht31.readHumidity() : NAN;
    float chk_l = vemlConnected ? veml.readLux() : NAN;
    float bus_voltage = ina236Connected ? INA.getBusVoltage() : NAN;

    bool values_ok = true;
    if (isnan(chk_t) || chk_t < -20.0 || chk_t > 60.0) values_ok = false;
    if (isnan(chk_h) || chk_h < 0.0 || chk_h > 100.0) values_ok = false;
    if (chk_l < 0.0) values_ok = false;
    if (isnan(bus_voltage) || bus_voltage < 0.0 || bus_voltage > 32.0) values_ok = false;
    
    bool hardware_ok = sht31Connected && vemlConnected && ina236Connected;
    bool bateria_ok = (PMU->getBatteryPercent() > 20.0);
    
    if (!hardware_ok || !values_ok) return 0;
    if (!bateria_ok) return 3;
    return 2;
}

void ErrorMode() {
    Serial.println(F("!!! ERROR DETECTADO - APAGANDO !!!"));
    float v = 0.0, i = 0.0;
    if (ina236Connected) {
        v = INA.getBusVoltage();
        i = INA.getCurrent_mA();
    }
    char mensajeError[50];
    snprintf(mensajeError, sizeof(mensajeError), "ERR_V:%.2f,I:%.2f", v, i);
    waitforLora();
    LMIC_setTxData2(1, (uint8_t*)mensajeError, strlen(mensajeError), 0);
    waitforLora();
    DeepSleep(sleepMinutes);
}

void setup() {
    Serial.begin(115200);
    delay(2000); 
    Serial.println(F("========================================="));
    Serial.println(F("=== Iniciando programa ==="));
    
    setupBoards();
    delay(500);

    if (PMU) {
        Serial.println("Configurando AXP2101: Encendiendo GPS (ALDO4)...");
        PMU->setPowerChannelVoltage(XPOWERS_ALDO4, 3300);
        PMU->enablePowerOutput(XPOWERS_ALDO4);
        delay(500);
    }

    Serial1.begin(GPS_BAUDRATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

    Wire.begin();
    setupSensors();
    
    Serial.println("Inicializando LMIC...");
    setupLMIC();
    Serial.println(F("=== Inicialización completada ==="));
}

void loop() {
    loopLMIC();
    if (LMIC.opmode & OP_TXRXPEND) return;

    if (joinStatus != EV_JOINED) { 
        if (millis() - Last_Log >= 10000) { 
            Last_Log = millis();
            Serial.println("Esperando el JOIN de LoRaWAN...");
        }
        delay(100);
        return;
    }

    if (!vemlConnected || !sht31Connected) {
        Serial.println("ERROR: Sensores I2C desconectados.");
        ErrorMode();
        return;
    }

    float lux = veml.readLux();
    float irradiancia = calculateIrradiance(lux);
    
    // Comprobación de modo noche/día
    if (irradiancia <= irradiance_limit) {
        Serial.print("Irradiancia baja: "); Serial.print(irradiancia); Serial.println(" W/m^2");
        NightMode();
    } else {
        // --- ES DE DÍA: Reseteamos el contador de la noche ---
        nightSleepIndex = 0; 
        
        if (isFirstSendOfDay) {
            Serial.println("Es el primer envío del día. Actualizando GPS.");
            sendGPSData(1000); 
            isFirstSendOfDay = false;
        }

        bool cargando = PMU->isCharging();
        float battPercent = PMU->getBatteryPercent();
        
        if (cargando || battPercent >= 98.0) { 
            if (battPercent > 20) {
                DayMode_Charge(irradiancia);
            } else {
                DayMode_NoCharge();
            }
        } else {
            DayMode_NoCharge();
        }
    }
}
