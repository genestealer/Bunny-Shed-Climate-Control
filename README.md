# Bunny-Shed-Climate-Control
Bunny shed heating controller using ESP8266 NodeMCU, 433Mhz transmitter, MQTT and Home Assistant

Richard Huish 2015-2017

ESP8266 based with local home-assistant.io GUI, 433Mhz transmitter for heater/fan control and DHT22 temperature-humidity sensor
    
  ----------
  Key Libraries:
  
  ESP8266WiFi.h>    https://github.com/esp8266/Arduino
  
  RCSwitch.h        https://github.com/sui77/rc-switch
  
  DHT.h             https://github.com/adafruit/DHT-sensor-library
  
  Adafruit_Sensor.g https://github.com/adafruit/Adafruit_Sensor required for DHT.h
  
  ----------
  
  GUI: Locally hosted home assistant https://home-assistant.io
  
  MQTT: Locally hosted broker https://mosquitto.org/
  
  ----------
  
  The circuit:
  
    NodeMCU Amica (ESP8266)
  
  Inputs:
  
    DHT22 temperature-humidity sensor - GPIO pin 5 (NodeMCU Pin D1)
    
  Outputs:
  
    433Mhz Transmitter - GPIO pin 2 (NodeMCU Pin D4)
    
    LED_NODEMCU - pin 16 (NodeMCU Pin D0)
    
    LED_ESP - GPIO pin 2 (NodeMCU Pin D4) (Shared with 433Mhz TX)
    
    ----------
    
  Notes:
  
    NodeMCU lED lights to show MQTT conenction.
    
    ESP lED lights to show WIFI conenction.
    
![alt text](Bunny-Shed-Heating-Controller.jpg "A photo of my setup")    
