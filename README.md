# Bunny-Shed-Climate-Control
Bunny shed heating controller using ESP8266 NodeMCU, 433Mhz transmitter, MQTT and Home Assistant

Richard Huish 2016

ESP8266 based with local home-assistant.io GUI, 433Mhz transmitter for heater/fan control and DHT22 temperature-humidity sensor
    
  ----------
  Key Libraries:
  
  ESP8266WiFi.h>    https://github.com/esp8266/Arduino
  
  RCSwitch.h        https://github.com/sui77/rc-switch
  
  DHT.h             https://github.com/adafruit/DHT-sensor-library
  
  Adafruit_Sensor.g https://github.com/adafruit/Adafruit_Sensor required for DHT.h
  
  ----------
  
  GUi: Locally hosted home assistant https://home-assistant.io
  
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
