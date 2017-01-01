/***************************************************
  Bunny Shed Climate Control
  Richard Huish 2016
  ESP8266 based with local home-assistant.io GUI,
    433Mhz transmitter for heater/fan control
    and DHT22 temperature-humidity sensor
  ----------
  Key Libraries:
  ESP8266WiFi.h>    https://github.com/esp8266/Arduino
  RCSwitch.h        https://github.com/sui77/rc-switch
  DHT.h             https://github.com/adafruit/DHT-sensor-library
  Adafruit_Sensor.g https://github.com/adafruit/Adafruit_Sensor required for DHT.h
  ----------
  GUi: Locally hosted home assistant
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
****************************************************/

// Note: Libaries are inluced in "Project Dependencies" file platformio.ini
#include <ESP8266WiFi.h>  // ESP8266 core for Arduino https://github.com/esp8266/Arduino
#include <PubSubClient.h>  // Arduino Client for MQTT https://github.com/knolleary/pubsubclient
#include <RCSwitch.h> //  https://github.com/sui77/rc-switch
#include <DHT.h>  //  https://github.com/adafruit/DHT-sensor-library
#include <Adafruit_Sensor.h> // have to add for the DHT to work https://github.com/adafruit/Adafruit_Sensor
#include <secretes.h> // Passwords etc not for github

// Define state machine states
typedef enum {
  s_idle = 0,   // state idle
  s_start = 1,  // state start
  s_on = 2,     // state on
  s_stop = 3,   // state stop
} e_state;
int stateMachine = 0;

// DHT sensor parameters
#define DHTPIN 5 // GPIO pin 5 (NodeMCU Pin D1)
#define DHTTYPE DHT22

// DHT sensor instance
DHT dht(DHTPIN, DHTTYPE, 15);

// 433Mhz transmitter parameters
const int tx433Mhz_pin = 2; // GPIO pin 2 (NODEMCU Pin D4)
const int setPulseLength = 305;

// 433MHZ TX instance
RCSwitch mySwitch = RCSwitch();

// WiFi parameters
const char* wifi_ssid = secrete_wifi_ssid; // Wifi access point SSID
const char* wifi_password = secrete_wifi_password; // Wifi access point password

// MQTT Settings
const char* mqtt_server = secrete_mqtt_server; // E.G. 192.168.1.xx
const char* clientName = secrete_clientName; // Client to report to MQTT
const char* mqtt_username = secrete_mqtt_username; // MQTT Username
const char* mqtt_password = secrete_mqtt_password; // MQTT Password
boolean willRetain = true; // MQTT Last Will and Testament
const char* willMessage = "offline"; // MQTT Last Will and Testament Message

// Publish
const char* publishOutputState = secrete_publishOutputState; //
const char* publishTemperature = secrete_publishTemperature; //
const char* publishHumidity = secrete_publishHumidity; //
const char* publishSetTemperature = secrete_publishSetTemperature; //
const char* publishLastWillTopic = secrete_publishLastWillTopic; //

// Subscribe
const char* subscribeSetTemperature = secrete_subscribeSetTemperature; //

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
// Target temperature (Set in code, but modified by web commands, local setpoint incase of internet connection break)
float targetTemperature = 8; //CHANGE TO ACCOMIDATE FAN AND HEATER
// Target temperature Hysteresis
const float targetTemperatureHyst = 1;
// Output powered status
bool outputPoweredStatus = false;
// // Output powered mode
// bool heaterOrCooler = true; //

// Setp the connection to WIFI and the MQTT Broker. Normally called only once from setup
void setup_wifi() {
  // We start by connecting to a WiFi network
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
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  digitalWrite(DIGITAL_PIN_LED_NODEMCU, LOW); // Lights on LOW. Light the NodeMCU LED to show wifi connection.
}

// MQTT payload in seconds will turn on output. A payload of 0 will turn off the output.
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
  String strTopicCompairSetpoint = subscribeSetTemperature;

if (srtTopic.equals(strTopicCompairSetpoint)) {
    if (targetTemperature != msgString.toFloat()) {
      Serial.println("new setpoint");
      targetTemperature = msgString.toFloat();
    }
  }

}

/*
  Non-Blocking mqtt reconnect.
  Called from checkMqttConnection.
*/
boolean mqttReconnect() {
  // Call on the background functions to allow them to do their thing
  yield();
  // Attempt to connect
  if (mqttClient.connect(clientName, mqtt_username, mqtt_password, publishLastWillTopic, 0, willRetain, willMessage)) {

    Serial.print("Attempting MQTT connection...");

    // Once connected, update status to online - will Message will drop in if we go offline ...
    mqttClient.publish(publishLastWillTopic,"online",true);

    // Resubscribe to feeds
    mqttClient.subscribe(subscribeSetTemperature);
    Serial.println("connected");

  } else {
    Serial.print("Failed MQTT connection, rc=");
    Serial.print(mqttClient.state());
    Serial.println(" try in 1.5 seconds");
  }
  return mqttClient.connected();
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
    //call on the background functions to allow them to do their thing.
    yield();
    // Client connected: MQTT client loop processing
    mqttClient.loop();
  }
}


// Returns true if heating is required
boolean checkHeatRequired(float roomTemperature, float targetTemperature, float targetTempHyst) {
  // Is room too cold ?
  if (roomTemperature < (targetTemperature - targetTempHyst))
  {
    // Heat needed
    return true;
  }
  // Else room is hot enough
  else
  {
    // No heat needed
    return false;
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
    mySwitch.send(2006879, 24);
    Serial.println(F("433Mhz TX ON command sent!"));
    outputPoweredStatus = true;
  }
  else
  {
    mySwitch.send(2006871, 24);
    Serial.println(F("433Mhz TX OFF command sent!"));
    outputPoweredStatus = false;

  }

  String strOutput = String(outputPoweredStatus);
  if (!mqttClient.publish(publishOutputState, strOutput.c_str()))
    Serial.print(F("Failed to output state to [")), Serial.print(publishOutputState), Serial.print("] ");
  else
    Serial.print(F("Output state published to [")), Serial.print(publishOutputState), Serial.println("] ");

}


// State machine for controller
void checkState() {
  switch (stateMachine) {
    case s_idle:
      // State is currently: idle
      // Check if we need to start, by checking if heat is still required.
      if (checkHeatRequired(dht.readTemperature(), targetTemperature, targetTemperatureHyst))
      {
        // Heat no longer required, stop.
        stateMachine = s_start;
      }
      break;
    case s_start:
      // State is currently: starting
      Serial.println("State is currently: starting");
      // Command the heater to turn on.
      controlHeater(true);
      stateMachine = s_on;
      break;

    case s_on:
      // State is currently: On
      // Check if we need to stop, by checking if heat is still required.
      if (!checkHeatRequired(dht.readTemperature(), targetTemperature, targetTemperatureHyst))
      {
        // Heat no longer required, stop.
        stateMachine = s_stop;
      }
      break;

    case s_stop:
      // State is currently: stopping
      Serial.println("State is currently: stopping");
      // Command the heater to turn off.
      controlHeater(false);
      // Set state mahcine to idle on the next loop
      stateMachine = s_idle;
      break;
  }
}

void mtqqPublish() {

  // Only run when publishInterval in milliseonds exspires
  unsigned long currentMillis = millis();
  // CODE TO MOVE TO functions
  if (currentMillis - previousMillis >= publishInterval) {
    // save the last time this ran
    previousMillis = currentMillis;
    if (mqttClient.connected()) {

      // Publish data
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

      String strSetpoint = String(targetTemperature);
      if (!mqttClient.publish(publishSetTemperature, strSetpoint.c_str(), true)) // retained = true
        Serial.print(F("Failed to target temperature to [")), Serial.print(publishSetTemperature), Serial.print("] ");
      else
        Serial.print(F("Target temperature published to [")), Serial.print(publishSetTemperature), Serial.println("] ");

      String strOutput = String(outputPoweredStatus);
      if (!mqttClient.publish(publishOutputState, strOutput.c_str()))
        Serial.print(F("Failed to output state to [")), Serial.print(publishOutputState), Serial.print("] ");
      else
        Serial.print(F("Output state published to [")), Serial.print(publishOutputState), Serial.println("] ");
    }
  }

}





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
  // Set MQTT settings
  mqttClient.setServer(mqtt_server, 1883);
  mqttClient.setCallback(mqttcallback);
  // Call on the background functions to allow them to do their thing
  yield();
  Serial.println("Setup Complete");
}



/// Main working loop
void loop() {
  yield(); //call on the background functions to allow them to do their thing.
  // First check if we are connected to the MQTT broker
  checkMqttConnection();
  //call on the background functions to allow them to do their thing.
  yield();
  // Check the status and do actions
  checkState();

  mtqqPublish();
}
