# TFG-Cristian-Fernández
# Implementación de un Nodo de Telemetría Inalámbrica para Instalaciones Fotovoltaicas

Repositorio oficial del Trabajo de Fin de Grado (TFG) desarrollado para el Grado en Ingeniería de Tecnologías Industriales en la Universidad Rey Juan Carlos (URJC).

Este proyecto propone una solución de bajo coste y código abierto (Open Source) para la monitorización en tiempo real de plantas de energía solar. Utiliza tecnología de comunicación LPWAN (LoRaWAN) y una arquitectura de Edge Computing para garantizar la independencia de servicios comerciales en la nube.

## 🛠️ Arquitectura y Tecnologías

El sistema se divide en dos bloques principales:

1. **Nodo Sensor:** Basado en el microcontrolador LILYGO TTGo-TBeam (ESP32 + LoRa SX1276). Se encarga de la adquisición de datos eléctricos y ambientales mediante buses I2C y OneWire.
2. **Servidor Local:** Implementado sobre una Raspberry Pi 4 con un módulo concentrador RAK5146. Alojado en contenedores Docker, procesa y visualiza la información de forma completamente local.

### Stack de Software
* **Firmware:** C/C++ (PlatformIO / Arduino IDE)
* **Network Server:** ChirpStack
* **Data Flow & Logic:** Node-RED
* **Time Series Database (TSDB):** InfluxDB
* **Dashboard & Visualización:** Grafana

### Sensores Integrados
* **INA236:** Monitorización de tensión, corriente y potencia DC del panel solar.
* **VEML7700:** Estimación de irradiancia lumínica ($W/m^2$).
* **SHT31:** Temperatura y humedad ambiente.
* **DS18B20:** Temperatura de contacto de la célula fotovoltaica.

## 📂 Estructura del Repositorio

* `/firmware`: Código fuente en C++ para el microcontrolador ESP32, incluyendo la máquina de estados y la lógica de *Deep Sleep*.
* `/servidor`: Archivo `docker-compose.yml` para desplegar la infraestructura de red local y el archivo `.json` exportado del flujo de Node-RED.

## 👨‍💻 Autor
**Cristian Fernández Díaz** *Grado en Ingeniería de Tecnologías Industriales* *Universidad Rey Juan Carlos*
