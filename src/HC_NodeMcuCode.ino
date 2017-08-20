/***************************************************
  Bunny Shed Climate Control
  Richard Huish 2016-2017
  ESP8266 based with local home-assistant.io GUI,
    433Mhz transmitter for heater/fan control
    and DHT22 temperature-humidity sensor
    Temperature and humidity sent as JSON via MQTT
  ----------
  Github: https://github.com/Genestealer/Bunny-Shed-Climate-Control
  ----------
  Key Libraries:
  ESP8266WiFi.h     https://github.com/esp8266/Arduino
  RCSwitch.h        https://github.com/sui77/rc-switch
  DHT.h             https://github.com/adafruit/DHT-sensor-library
  Adafruit_Sensor.h https://github.com/adafruit/Adafruit_Sensor required for DHT.h
  ESP8266mDNS.h     https://github.com/esp8266/Arduino/tree/master/libraries/ESP8266mDNS
  WiFiUdp.h         https://github.com/esp8266/Arduino
  ArduinoOTA.h      https://github.com/esp8266/Arduino
  ArduinoJson.h     https://bblanchon.github.io/ArduinoJson/
  ----------
  GUI: Locally hosted home assistant
  MQTT: Locally hosted broker https://mosquitto.org/
  OTA updates
  ----------
  The circuit:
    NodeMCU Amica (ESP8266)
  Inputs:
    DHT22 temperature-humidity sensor - GPIO pin 5 (NodeMCU Pin D1)
  Outputs:
    433Mhz Transmitter - GPIO pin 2 (NODEMCU Pin D4)
    LED_NODEMCU - pin 16 (NodeMCU Pin D0)
    LED_ESP - GPIO pin 2 (NodeMCU Pin D4) (Shared with 433Mhz TX)
    ----------
  Notes:
    NodeMCU lED lights to show MQTT conenction.
    ESP lED lights to show WIFI conenction.
    433Mhz TX controls both the heater and fan
    ----------
    Edits made to the PlatformIO Project Configuration File:
      platform = espressif8266_stage = https://github.com/esp8266/Arduino/issues/2833 as the standard has an outdated Arduino Core for the ESP8266, ref http://docs.platformio.org/en/latest/platforms/espressif8266.html#over-the-air-ota-update
      build_flags = -DMQTT_MAX_PACKET_SIZE=512 = Overide max JSON size, until libary is updated to inclde this option https://github.com/knolleary/pubsubclient/issues/110#issuecomment-174953049
    ----------
    Sources:
    https://github.com/mertenats/open-home-automation/tree/master/ha_mqtt_sensor_dht22
    Create a JSON object
      Example https://github.com/mertenats/Open-Home-Automation/blob/master/ha_mqtt_sensor_dht22/ha_mqtt_sensor_dht22.ino
      Doc : https://github.com/bblanchon/ArduinoJson/wiki/API%20Reference
  ****************************************************/

// Note: Libaries are inluced in "Project Dependencies" file platformio.ini
#include <ESP8266WiFi.h>     // ESP8266 core for Arduino https://github.com/esp8266/Arduino
#include <PubSubClient.h>    // Arduino Client for MQTT https://github.com/knolleary/pubsubclient
#include <RCSwitch.h>        // RF control lib, https://github.com/sui77/rc-switch
#include <DHT.h>             // DHT Sensor lib, https://github.com/adafruit/DHT-sensor-library
#include <Adafruit_Sensor.h> // Have to add for the DHT to work https://github.com/adafruit/Adafruit_Sensor
#include <private.h>         // Passwords etc not for github
#include <ESP8266mDNS.h>     // Needed for Over-the-Air ESP8266 programming https://github.com/esp8266/Arduino
#include <WiFiUdp.h>         // Needed for Over-the-Air ESP8266 programming https://github.com/esp8266/Arduino
#include <ArduinoOTA.h>      // Needed for Over-the-Air ESP8266 programming https://github.com/esp8266/Arduino
#include <ArduinoJson.h>     // For sending MQTT JSON messages https://bblanchon.github.io/ArduinoJson/

// Define state machine states
typedef enum {
  s_idle = 0,         // state idle
  s_HeaterStart = 1,  // state start
  s_HeaterOn = 2,     // state on
  s_HeaterStop = 3,   // state stop
  s_CoolerStart = 4,  // state start
  s_CoolerOn = 5,     // state on
  s_CoolerStop = 6,   // state stop
} e_state;
int stateMachine = 0;

// DHT sensor parameters
#define DHTPIN 5 // GPIO pin 5 (NodeMCU Pin D1)
#define DHTTYPE DHT22

// DHT sensor instance
DHT dht(DHTPIN, DHTTYPE, 15);

// WiFi parameters
const char* wifi_ssid = secret_wifi_ssid; // Wifi access point SSID
const char* wifi_password = secret_wifi_password; // Wifi access point password

// 433Mhz transmitter parameters
const int tx433Mhz_pin = 2; // GPIO pin 2 (NODEMCU Pin D4)
const int setPulseLength = 305;

// 433MHZ TX instance
RCSwitch mySwitch = RCSwitch();

// MQTT Settings
const char* mqtt_server = secret_mqtt_server; // E.G. 192.168.1.xx
const char* clientName = secret_clientName; // Client to report to MQTT
const char* mqtt_username = secret_mqtt_username; // MQTT Username
const char* mqtt_password = secret_mqtt_password; // MQTT Password
boolean willRetain = true; // MQTT Last Will and Testament
const char* willMessage = "offline"; // MQTT Last Will and Testament Message
const int json_buffer_size = 256;

// Subscribe
const char* subscribeSetHeaterTemperature = secret_subscribeSetHeaterTemperature; //
const char* subscribeSetCoolerTemperature = secret_subscribeSetCoolerTemperature; //

// Publish
const char* publishHeaterOutputState = secret_publishHeaterOutputState; //
const char* publishCoolerOutputState = secret_publishCoolerOutputState; //

const char* publishSetHeaterTemperature = secret_publishSetHeaterTemperature; //
const char* publishSetCoolerTemperature = secret_publishSetCoolerTemperature; //

const char* publishTemperature = secret_publishTemperature; //
const char* publishHumidity = secret_publishHumidity; //

const char* publishLastWillTopic = secret_publishLastWillTopic; //
const char* publishClientName = secret_publishClientName; // E.G. Home/Shed/clientName"
const char* publishIpAddress = secret_publishIpAddress; // E.G. Home/Shed/IpAddress"
const char* publishSignalStrength = secret_publishSignalStrength; // E.G. Home/Shed/SignalStrength"
const char* publishHostName = secret_publishHostName; // E.G. Home/Shed/HostName"
const char* publishSSID = secret_publishSSID; // E.G. Home/Shed/SSID"

const char* publishSensorJsonTopic = secret_publishSensorJsonTopic;
const char* publishStatusJsonTopic = secret_publishStatusJsonTopic;

// MQTT instance
WiFiClient espClient;
PubSubClient mqttClient(espClient);
char message_buff[100];
long lastReconnectAttempt = 0; // Reconnecting MQTT - non-blocking https://github.com/knolleary/pubsubclient/blob/master/examples/mqtt_reconnect_nonblocking/mqtt_reconnect_nonblocking.ino

// MQTT publish frequency
unsigned long previousMillis = 0;
const long publishInterval = 60000; // Publish requency in milliseconds 60000 = 1 min

// LED output parameters
const int DIGITAL_PIN_LED_ESP = 2; // Define LED on ESP8266 sub-modual
const int DIGITAL_PIN_LED_NODEMCU = 16; // Define LED on NodeMCU board - Lights on pin LOW

// Climate parameters
// Target temperatures (Set in code, but modified by web commands, local setpoint in case of internet connection break)
float targetHeaterTemperature = 8;
float targetCoolerTemperature = 25;

// Target temperature Hysteresis
const float targetHeaterTemperatureHyst = 1; // DHT22 has 0.5 accuracy
const float targetCoolerTemperatureHyst = 1; // DHT22 has 0.5 accuracy

// Output powered status
bool outputHeaterPoweredStatus = false;
bool outputCoolerPoweredStatus = false;

// Setp the connection to WIFI and the MQTT Broker. Normally called only once from setup
void setup_wifi() {
  /* Explicitly set the ESP8266 to be a WiFi-client, otherwise, it by default,
     would try to act as both a client and an access-point and could cause
     network-issues with your other WiFi-devices on your WiFi-network. */
  WiFi.mode(WIFI_STA);

  // Connect to the WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.print(wifi_ssid);
  WiFi.begin(wifi_ssid, wifi_password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }
  Serial.print("");
  Serial.println("WiFi connected");

  Serial.printf("RSSI: %d dBm\n", WiFi.RSSI());
  Serial.print("Connected, IP address: ");
  Serial.println(WiFi.localIP());
  Serial.printf("Hostname: %s\n", WiFi.hostname().c_str());

  digitalWrite(DIGITAL_PIN_LED_NODEMCU, LOW); // Lights on LOW. Light the NodeMCU LED to show wifi connection.
}

// Setup Over-the-Air programming, called from the setup.
// https://www.penninkhof.com/2015/12/1610-over-the-air-esp8266-programming-using-platformio/
void setup_OTA() {
  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);
  // Hostname defaults to esp8266-[ChipID]
  // ArduinoOTA.setHostname("myesp8266");
  // No authentication by default
  // ArduinoOTA.setPassword("admin");
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";
    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
}

// MQTT payload
void mqttcallback(char* topic, byte* payload, unsigned int length) {
  //If you want to publish a message from within the message callback function, it is necessary to make a copy of the topic and payload values as the client uses the same internal buffer for inbound and outbound messages:
  //http://www.hivemq.com/blog/mqtt-client-library-encyclopedia-arduino-pubsubclient/
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();


  // create character buffer with ending null terminator (string)
  int i = 0;
  for (i = 0; i < length; i++) {
    message_buff[i] = payload[i];
  }
  message_buff[i] = '\0';
  // Check the value of the message
  String msgString = String(message_buff);
  Serial.println(msgString);

  // Check the message topic
  String srtTopic = topic;
  //String strTopicCompairSetpoint = subscribeSetHeaterTemperature;

  if (srtTopic.equals(subscribeSetHeaterTemperature))
  {
    if (targetHeaterTemperature != msgString.toFloat())
    {
      Serial.println("new heater setpoint");
      targetHeaterTemperature = msgString.toFloat();
    }
  }
  else if (srtTopic.equals(subscribeSetCoolerTemperature))
  {
    if (targetCoolerTemperature != msgString.toFloat())
    {
      Serial.println("new cooler setpoint");
      targetCoolerTemperature = msgString.toFloat();
    }
  }
}

void publishNodeState() {
  // Update status to online, retained = true - last will Message will drop in if we go offline
  mqttClient.publish(publishLastWillTopic, "online", true);
  // Gather data
  char bufIP[16]; // Wifi IP address
  sprintf(bufIP, "%d.%d.%d.%d", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3] );
  char bufMAC[6]; // Wifi MAC address
  sprintf(bufMAC, "%02x:%02x:%02x:%02x:%02x:%02x", WiFi.macAddress()[0], WiFi.macAddress()[1], WiFi.macAddress()[2], WiFi.macAddress()[3], WiFi.macAddress()[4], WiFi.macAddress()[5] );
  // Create and publish the JSON object.
  StaticJsonBuffer<json_buffer_size> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
  // INFO: the data must be converted into a string; a problem occurs when using floats...
  root["ClientName"] = String(clientName);
  root["IP"] = String(bufIP);
  root["MAC"] = String(bufMAC);
  root["RSSI"] = String(WiFi.RSSI());
  root["HostName"] = String(WiFi.hostname());
  root["ConnectedSSID"] = String(WiFi.SSID());
  root.prettyPrintTo(Serial);
  Serial.println(""); // Add new line as prettyPrintTo leaves the line open.
  char data[json_buffer_size];
  root.printTo(data, root.measureLength() + 1);
  if (!mqttClient.publish(publishStatusJsonTopic, data, true)) // retained = true
    Serial.print(F("Failed to publish JSON Status to [")), Serial.print(publishStatusJsonTopic), Serial.print("] ");
  else
    Serial.print(F("JSON Status Published [")), Serial.print(publishStatusJsonTopic), Serial.println("] ");
}

/*
  Non-Blocking mqtt reconnect.
  Called from checkMqttConnection.
  Based on example from 5ace47b Sep 7, 2015 https://github.com/knolleary/pubsubclient/blob/master/examples/mqtt_reconnect_nonblocking/mqtt_reconnect_nonblocking.ino
*/
boolean mqttReconnect() {
  // Call on the background functions to allow them to do their thing
  yield();
  // Attempt to connect
  if (mqttClient.connect(clientName, mqtt_username, mqtt_password, publishLastWillTopic, 0, willRetain, willMessage)) {

    Serial.print("Attempting MQTT connection...");
    // Publish node state data
    publishNodeState();
    // Resubscribe to feeds
    mqttClient.subscribe(subscribeSetHeaterTemperature);
    mqttClient.subscribe(subscribeSetCoolerTemperature);
    Serial.println("Connected to MQTT server");
  }
  else
  {
    Serial.print("Failed MQTT connection, rc=");
    Serial.print(mqttClient.state());
    Serial.println(" try again in 1.5 seconds");
  }
  return mqttClient.connected(); // Return connection state
}


/*
  Checks if connection to the MQTT server is ok. Client connected
  using a non-blocking reconnect function. If the client loses
  its connection, it attempts to reconnect every 5 seconds
  without blocking the main loop.
  Called from main loop.
*/
void checkMqttConnection() {
  if (!mqttClient.connected()) {
    // We are not connected. Turn off the wifi LED
    digitalWrite(DIGITAL_PIN_LED_ESP, HIGH); // Lights on LOW
    long now = millis();
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      // Attempt to reconnect
      if (mqttReconnect()) {
        lastReconnectAttempt = 0;
      }
    }
  } else {
    // We are connected.
    digitalWrite(DIGITAL_PIN_LED_ESP, LOW); // Lights on LOW
    //Call on the background functions to allow them to do their thing.
    yield();
    // Client connected: MQTT client loop processing
    mqttClient.loop();
  }
}

void mqttPublishData(bool publishInstant) {
  // Only run when publishInterval in milliseonds exspires or publishInstant = true
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= publishInterval || publishInstant == true) {
    previousMillis = currentMillis; // save the last time this ran
    // Check conenction to MQTT server
    if (mqttClient.connected()) {
      // Publish node state data
      publishNodeState();

      // New JSON data method
      StaticJsonBuffer<json_buffer_size> jsonBuffer;
      JsonObject& root = jsonBuffer.createObject();
      // INFO: the data must be converted into a string; a problem occurs when using floats...
      root["Temperature"] = String(dht.readTemperature());
      root["Humidity"] = String(dht.readHumidity());
      root["TargetHeaterTemperature"] = String(targetHeaterTemperature);
      root["TargetCoolerTemperature"] = String(targetCoolerTemperature);
      root["HeaterState"] = String(outputHeaterPoweredStatus);
      root["CoolerState"] = String(outputCoolerPoweredStatus);
      root.prettyPrintTo(Serial);
      Serial.println(""); // Add new line as prettyPrintTo leaves the line open.
      char data[json_buffer_size];
      root.printTo(data, root.measureLength() + 1);
      if (!mqttClient.publish(publishSensorJsonTopic, data))
        Serial.print(F("Failed to publish JSON sensor data to [")), Serial.print(publishSensorJsonTopic), Serial.print("] ");
      else
        Serial.print(F("JSON Sensor data published to [")), Serial.print(publishSensorJsonTopic), Serial.println("] ");

      Serial.println("JSON Sensor Published");

      // Old legacy method
      // Grab the current state of the sensor
      String strTemp = String(dht.readTemperature()); //Could use String(dht.readTemperature()).c_str()) to do it all in one line
      if (!mqttClient.publish(publishTemperature, String(dht.readTemperature()).c_str())) // Convert dht.readTemperature() to string object, then to char array.
        Serial.print(F("Failed to published to [")), Serial.print(publishTemperature), Serial.print("] ");
      else
        Serial.print(F("Temperature published to [")), Serial.print(publishTemperature), Serial.println("] ");

      String strHumi = String(dht.readHumidity());
      if (!mqttClient.publish(publishHumidity, strHumi.c_str()))
        Serial.print(F("Failed to humidity to [")), Serial.print(publishHumidity), Serial.print("] ");
      else
        Serial.print(F("Humidity published to [")), Serial.print(publishHumidity), Serial.println("] ");

      String strSetpoint = String(targetHeaterTemperature);
      if (!mqttClient.publish(publishSetHeaterTemperature, strSetpoint.c_str(), true)) // retained = true
        Serial.print(F("Failed to target temperature to [")), Serial.print(publishSetHeaterTemperature), Serial.print("] ");
      else
        Serial.print(F("Target temperature published to [")), Serial.print(publishSetHeaterTemperature), Serial.println("] ");

      String strHeaterOutput = String(outputHeaterPoweredStatus);
      if (!mqttClient.publish(publishHeaterOutputState, strHeaterOutput.c_str()))
        Serial.print(F("Failed to output heater state to [")), Serial.print(publishHeaterOutputState), Serial.print("] ");
      else
        Serial.print(F("Output heater state published to [")), Serial.print(publishHeaterOutputState), Serial.println("] ");

      String strCoolerOutput = String(outputCoolerPoweredStatus);
      if (!mqttClient.publish(publishCoolerOutputState, strCoolerOutput.c_str()))
        Serial.print(F("Failed to output cooler state to [")), Serial.print(publishCoolerOutputState), Serial.print("] ");
      else
        Serial.print(F("Output cooler state published to [")), Serial.print(publishCoolerOutputState), Serial.println("] ");
    }}}

// Returns true if heating is required
boolean checkHeatRequired(float roomTemperature, float targetTemperature, float targetTempHyst, bool poweredState) {
  // Check if heating is active.
  if (!poweredState) {
    // Heating is not active. Room is cooling down
    // Use Hyst to delay starting heating until we are passed the target to avoid over cycling the heater.
    if (roomTemperature < (targetTemperature - targetTempHyst))
      return true; // Heating is now needed
    else // Else room is not cold enough
      return false; // Heating not needed yet
  }
  else
  {
    // Heater is active. Room is heating up.
    // Use Hyst to delay stopping heating until we are passed the target to avoid over cycling the heater.
    if (roomTemperature > (targetTemperature + targetTempHyst))
      return false; // Heating no longer needed
    else // Else room is not hot enough
      return true; // Heating is still needed
  }
}

// Returns true if cooling is required
boolean checkCoolRequired(float roomTemperature, float targetTemperature, float targetTempHyst, bool poweredState) {
  // Check if cooling is active.
  if (!poweredState) {
    // Cooler is not active. Room is warming up
    // Use Hyst to delay starting cooling until we are passed the target to avoid over cycling the cooler.
    if (roomTemperature > (targetTemperature + targetTempHyst))
      return true; // Cooling is now needed
    else // Else room is not warm enough
      return false; // Cooling not needed yet
  }
  else
  {
    // Cooler is active. Room is cooling down.
    // Use Hyst to delay stopping cooling until we are passed the target to avoid over cycling the cooler.
    if (roomTemperature < (targetTemperature - targetTempHyst))
      return false; // Cooling no longer needed
    else // Else room is not cold enough
      return true; // Cooling is still needed
  }
}

// Control heater
void controlHeater(boolean heaterStateRequested) {
  // Acknowledgements
  // thisoldgeek/ESP8266-RCSwitch
  // https://github.com/thisoldgeek/ESP8266-RCSwitch
  // Find the codes for your RC Switch using https://github.com/ninjablocks/433Utils (RF_Sniffer.ino)
  if (heaterStateRequested == 1)
  {
    mySwitch.send(secret_HeaterOnCommand, 24);  // Replace codes as required
    Serial.println(F("433Mhz TX ON command sent!"));
    outputHeaterPoweredStatus = true;
  } else {
    mySwitch.send(secret_HeaterOffCommand, 24);  // Replace codes as required
    Serial.println(F("433Mhz TX OFF command sent!"));
    outputHeaterPoweredStatus = false;
  }
  // Publish state change, instantly without waiting for publishInterval.
  mqttPublishData(true);
}

// Control cooler
void controlCooler(boolean coolerStateRequested) {
  // Acknowledgements
  // thisoldgeek/ESP8266-RCSwitch
  // https://github.com/thisoldgeek/ESP8266-RCSwitch
  // Find the codes for your RC Switch using https://github.com/ninjablocks/433Utils (RF_Sniffer.ino)
  if (coolerStateRequested == 1)
  {
    mySwitch.send(secret_CoolerOnCommand, 24);  // Replace codes as required
    Serial.println(F("433Mhz TX ON command sent!"));
    outputCoolerPoweredStatus = true;
  } else {
    mySwitch.send(secret_CoolerOffCommand, 24);  // Replace codes as required
    Serial.println(F("433Mhz TX OFF command sent!"));
    outputCoolerPoweredStatus = false;
  }
  // Publish state change, instantly without waiting for publishInterval.
  mqttPublishData(true);
}


// State machine for controller
void checkState() {
  switch (stateMachine) {
    case s_idle:
      // State is currently: idle. Neather cooling or heating.
      if (checkHeatRequired(dht.readTemperature(), targetHeaterTemperature, targetHeaterTemperatureHyst, false))
        stateMachine = s_HeaterStart;    // Heat required, start.
      else if (checkCoolRequired(dht.readTemperature(), targetCoolerTemperature, targetCoolerTemperatureHyst, false))
        stateMachine = s_CoolerStart;    // Cooling required, start.
      break;

    case s_HeaterStart:
      // State is currently: starting
      Serial.println("State is currently: starting heating");
      // Command the heater to turn on.
      controlHeater(true);
      stateMachine = s_HeaterOn;
      break;

    case s_HeaterOn:
      // State is currently: On
      // This inhibits the cooler turning on.
      // Check if we need to stop, by checking if heat is still required.
      if (!checkHeatRequired(dht.readTemperature(), targetHeaterTemperature, targetHeaterTemperatureHyst, true))
        stateMachine = s_HeaterStop; // Heat no longer required, stop.
      break;

    case s_HeaterStop:
      // State is currently: stopping
      Serial.println("State is currently: stopping heating");
      // Command the heater to turn off.
      controlHeater(false);
      // Set state mahcine to idle on the next loop
      stateMachine = s_idle;
      break;

    case s_CoolerStart:
      // State is currently: starting
      Serial.println("State is currently: starting cooling");
      // Command the heater to turn on.
      controlCooler(true);
      stateMachine = s_CoolerOn;
      break;

    case s_CoolerOn:
      // State is currently: On
      // This inhibits the heater turning on.
      // Check if we need to stop, by checking if cooling is still required.
      if (!checkCoolRequired(dht.readTemperature(), targetCoolerTemperature, targetCoolerTemperatureHyst, true))
        stateMachine = s_CoolerStop; // Cooling no longer required, stop.
      break;

    case s_CoolerStop:
      // State is currently: stopping
      Serial.println("State is currently: stopping cooling");
      // Command the cooler to turn off.
      controlCooler(false);
      // Set state mahcine to idle on the next loop
      stateMachine = s_idle;
      break;
  }}

void setup() {
  // Initialize pins
  pinMode(DIGITAL_PIN_LED_NODEMCU, OUTPUT);
  pinMode(DIGITAL_PIN_LED_ESP, OUTPUT);
  // Initialize pin start values
  digitalWrite(DIGITAL_PIN_LED_NODEMCU, LOW); // Lights on HIGH
  digitalWrite(DIGITAL_PIN_LED_ESP, HIGH); // Lights on LOW
  // set serial speed
  Serial.begin(115200);
  Serial.println("Setup Starting");

  // Setup 433Mhz Transmitter
  pinMode(tx433Mhz_pin, OUTPUT);
  // Set 433Mhz pin to output
  mySwitch.enableTransmit(tx433Mhz_pin);
  // Set pulse length.
  mySwitch.setPulseLength(setPulseLength);
  // Optional set protocol (default is 1, will work for most outlets)
  mySwitch.setProtocol(1);

  // Init temperature and humidity sensor
  dht.begin();

  // Call on the background functions to allow them to do their thing
  yield();
  // Setup wifi
  setup_wifi();
  // Call on the background functions to allow them to do their thing
  yield();
  // Setup OTA updates.
  setup_OTA();
  // Call on the background functions to allow them to do their thing
  yield();
  // Set MQTT settings
  mqttClient.setServer(mqtt_server, 1883);
  mqttClient.setCallback(mqttcallback);
  // Call on the background functions to allow them to do their thing
  yield();
  Serial.println("Setup Complete");
}

// Main working loop
void loop() {
  // Call on the background functions to allow them to do their thing.
  yield();
  // First check if we are connected to the MQTT broker
  checkMqttConnection();
  // Call on the background functions to allow them to do their thing.
  yield();
  // Check the status and do actions
  checkState();
  // Publish MQTT
  mqttPublishData(false);
  //Call on the background functions to allow them to do their thing.
  yield();
  // Check for Over The Air updates
  ArduinoOTA.handle();

  // Deal with millis rollover, hack by resetting the esp every 48 days
  if (millis() > 4147200000)
    ESP.restart();
}
