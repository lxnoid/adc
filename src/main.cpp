#include <Arduino.h>
#ifdef ESP32
  #include <WiFi.h>
#else
  #include <ESP8266Wifi.h>
#endif
#include "LittleFS.h"
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFiClient.h>
#include <movingAvg.h>

// generic setup
char wifiSsid[1024]      = "";  
char wifiPassword[1024]  = "";
char mqttServer[1024]    = ""; 
int  mqttPort            = 0; 
char mqttUser[1024]      = "";
char mqttPassword[1024]  = "";
char mqttClientId[1024]  = "";

WiFiClient espClient;
PubSubClient mqtt_client(espClient);


#define NUMBER_MUX_CHANNELS 3
#define NUMBER_ADC_CHANNELS 8 // should be 2 ^ NUMBER_MUX_CHANNELS

#define  PIN_MUX0   D6
#define  PIN_MUX1   D7
#define  PIN_MUX2   D8
#define  PIN_ANALOG A0

#define NUMBER_OF_MOVING_VALUES 3

// ==================================== Globals ==========================================
unsigned long current_millis = 0;
unsigned long prev_millis = 0;

int mqtt_status = 10 * 1000; //10s - sending every 10s
uint8_t mux_channels[NUMBER_MUX_CHANNELS] = { PIN_MUX0, PIN_MUX1, PIN_MUX2 }; 
movingAvg* adc_channel_values[NUMBER_ADC_CHANNELS];

// ==================================== CODE ==================================== 
void select_adc_channel(int channel) {
  if ((channel >= 0) && (channel < NUMBER_ADC_CHANNELS)) {
    for (int i = 0; i < NUMBER_MUX_CHANNELS; i++) {
      //if we find the bit to be set, then we active the matchting pin. Going from lowest bit to highest.
      if (channel & (1 << i)) {
        digitalWrite(mux_channels[i], HIGH);
      } else {
        digitalWrite(mux_channels[i], LOW);
      }
    }
  }
}

// ==================================== SETUP ==================================== 

void setup() {
  Serial.begin(115200);
  //read config
  if (!LittleFS.begin()) {
    Serial.println("LITTLEFS Mount Failed");
  }
  File cfile = LittleFS.open("/config.json", "r");
  if (cfile) {
    String config_data;
    while (cfile.available()) {
      config_data += char(cfile.read());
    }
    Serial.print(config_data);   
    DynamicJsonDocument cjson(config_data.length());
    deserializeJson(cjson, config_data);
    Serial.println("Deserialize done.");

    const char* c_wifiSsid     = cjson["wifiSsid"];  
    const char* c_wifiPassword = cjson["wifiPassword"];
    const char* c_mqttServer   = cjson["mqttServer"]; 
    const char* c_mqttPort     = cjson["mqttPort"]; 
    const char* c_mqttUser     = cjson["mqttUser"]; 
    const char* c_mqttPassword = cjson["mqttPassword"];
    const char* c_mqttClientId = cjson["mqttClientID"];

    Serial.println(c_wifiSsid);

    if (strlen(c_wifiSsid)) { 
      snprintf(wifiSsid, 1024, "%s", c_wifiSsid); 
    }
    if (strlen(c_wifiPassword)) { 
      snprintf(wifiPassword, 1024, "%s", c_wifiPassword); 
    }
    if (strlen(c_mqttServer)) { 
      snprintf(mqttServer, 1024, "%s", c_mqttServer); 
    }
    if (strlen(c_mqttPort)) { 
      mqttPort = atoi(c_mqttPort);
    }
    if (strlen(c_mqttUser)) { 
      snprintf(mqttUser, 1024, "%s", c_mqttUser);
    }
    if (strlen(c_mqttPassword)) { 
      snprintf(mqttPassword, 1024, "%s", c_mqttPassword);
    }
    if (strlen(c_mqttClientId)) {
       snprintf(mqttClientId, 1024, "%s", c_mqttClientId);
    }
  } else {
    Serial.println("Config File missing.");
    for (;;)
      delay(1);
  }
  delay(100);
  cfile.close();
  
  Serial.println("Configuring DIO/ADC.\n");


  // ADC/DIO -------------------------------------------------------------------------------
  // pinMode(PIN_ADC, ANALOG); - just a reminder, real code: analogRead(analogInPin);
  for (int i = 0; i < NUMBER_MUX_CHANNELS; i++) {
    pinMode(mux_channels[i], OUTPUT);
    digitalWrite(mux_channels[i], LOW);
  }

  for (int i = 0; i < NUMBER_ADC_CHANNELS; i++) {
    adc_channel_values[i] = new movingAvg(NUMBER_OF_MOVING_VALUES);
    adc_channel_values[i]->begin();
  }

  
  // Wifi ------------------------------------------------------------------------------
  Serial.begin(115200);
  delay(100);
  Serial.println("Connecting to WiFi");

  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  WiFi.setOutputPower(10.0);
  WiFi.setPhyMode(WIFI_PHY_MODE_11N);
  WiFi.persistent(false);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  WiFi.setAutoReconnect(true);
  
  WiFi.begin(wifiSsid, wifiPassword);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.println("...Connecting to WiFi");
    delay(1000);
  }  
  Serial.println("Connected to WiFi");
  
  delay(100);
  
  // MQTT ------------------------------------------------------------------------------
  mqtt_client.setServer(mqttServer, mqttPort);
  
  while (!mqtt_client.connected()) {
    Serial.println("...Connecting to MQTT");
    if (mqtt_client.connect(mqttClientId, mqttUser, mqttPassword )) {
      Serial.println("Connected to MQTT");
    } else {
      Serial.print("Failed connecting MQTT with state: ");
      Serial.print(mqtt_client.state());
      delay(2000);
    }
  }

  mqtt_client.publish("d1mini/adc_values/hello", "Hello World.");
  Serial.println("-- End of Setup --");
}

boolean mqtt_reconnect() {
  if (mqtt_client.connect(mqttClientId, mqttUser, mqttPassword )) {
    mqtt_client.publish("d1mini/adc_values/hello","Hello World, again", true);
  }
  return mqtt_client.connected();
}

// ==================================== LOOP ==================================== 
void loop() {
  static bool trigger_adc = true;

  // put your main code here, to run repeatedly:
  mqtt_client.loop();
  current_millis = millis();

  if (trigger_adc)
  {
    Serial.println("-- Sampling --");
    for (int i = 0; i < NUMBER_ADC_CHANNELS; i++) {
      int measured_value;
      //set mux
      (void)select_adc_channel(i);
      delayMicroseconds(10);
      //read adc
      measured_value = analogRead(PIN_ANALOG);
      //push into filter
      adc_channel_values[i]->reading(measured_value);
    }
    trigger_adc = false;
  }

  if ((int)(current_millis - prev_millis) >= mqtt_status) {
    Serial.println("-- 30s reached --");
    for (int i = 0; i < NUMBER_ADC_CHANNELS; i++) {
      char mqtt_message[80];
      char mqtt_topic[80];
      int value = adc_channel_values[i]->getAvg();
      snprintf(mqtt_message, 80, "{ value: %d }", value);
      snprintf(mqtt_topic, 80, "d1mini/adc_values/adc%d", i);
      Serial.println(mqtt_topic);
      Serial.println(mqtt_message);
      boolean status = mqtt_client.publish(mqtt_topic, mqtt_message, 80);
      Serial.println(status);
    }  
    trigger_adc = true;
    prev_millis = current_millis;
  }
  //check if MQTT and Wifi connected (wifi is set to reconnect on auto)
  if (!mqtt_client.connected()) {
    mqtt_reconnect();
    delay(200);
  }
}