# AZ-Touch_ESP32_MQTT_IO_Thermostat
AZ-Touch ESP 2.8" color touch BMP280 setup as a room thermostat

I bought a AZ-Touch wall mount housing set with a 2.8" touch screen for ESP32 (www.az-delivery.de)
Mounted a ESP-WROOM-32D processor and BMP280 temperature sensor on the pcb board.
Made a program for a room thermostat that will publish is setpoint to a MQTT broker
On the other hand the thermostat has a MQTT subscription for getting an extern setpoint
The program is based on my ESP8266-MQTT-IO project
