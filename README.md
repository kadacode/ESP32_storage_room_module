# ESP32-storage-room-module
Storage room module for logging in- and outside air conditions plus VFD control.

Based on DS18B20, DHT22, BME280 and SCD30 sensors, it uploads temperature, humidity, dewpoint, air pressure and air CO2 content data to two cloud platforms, receives and locally stores control variables from mydevices by Cayenne and includes restart in case of software or task hang for long-time autonomy.

Install the following libraries: Adafruit_Unified_Sensor, CayenneMQTT, DallasTemperature, DHT_sensor_library, OneWire, scd30-master, SparkFun_BME280, SparkFun_SCD30_Arduino_Library

First-time coding so surely it is incomplete and not optimised. So far it seems to work. Feel free to improve and share.

Some pictures are included to get a sense of the build and functioning.
