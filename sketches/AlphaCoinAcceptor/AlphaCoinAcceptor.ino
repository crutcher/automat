#include <ArduinoOTA.h>

// Shutup FastLED pragma message:
#define FASTLED_INTERNAL
#include <FastLED.h>

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#ifndef WIFI_SSID
#define WIFI_SSID "_infinet"
#endif

#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD "spaceship"
#endif

#ifndef MQTT_BROKER
#define MQTT_BROKER "10.133.10.98"
#endif

#ifndef MQTT_PORT
#define MQTT_PORT 1883
#endif

String COIN_CONTROL_TOPIC; // "automat/coin/{ID}/control"
String COIN_ACCEPTED_TOPIC; // "automat/coin/{ID}/accepted"

String AUTOMAT_CELL_ID;

const String ENABLE_KEY = "enable";

// PINS:
// LED_BUILTIN

#define PIN_A0 17
#define PIN_D0 16
#define PIN_D5 14
#define PIN_D6 12
#define PIN_D7 13
#define PIN_D8 15

#define PIN_TX 1
#define PIN_RX 3
#define PIN_D1 5
#define PIN_D2 4
#define PIN_D3 0
#define PIN_D4 2

const int ENABLE_ACCEPTOR_POWER_PIN = PIN_D1;
const int OVERRIDE_BUTTON_SIGNAL_PIN = PIN_D3;
const int COIN_SIGNAL_PIN = PIN_D2;

/**
 * Format a binary MAC address as a hex string.
 *
 */
String macToStr(const uint8_t mac[6]) {
  String result;
  for (int i = 0; i < 6; ++i) {
    result += String(mac[i], HEX);
    if (i < 5)
      result += ':';
  }
  return result;
}

StaticJsonDocument<256> MQTT_RECEIPT_DOC;

bool enabled = false;

void onMqttMessage(char* topic, byte* payload, unsigned int payload_length) {
  Serial.print("onMqttMessage: ");
  Serial.println(topic);
  
  // handle message arrived
  if (!strcmp(topic, (char*) COIN_CONTROL_TOPIC.c_str())) {
    // Environment Update.
    Serial.println("Control Message");

    if (MQTT_RECEIPT_DOC.containsKey(ENABLE_KEY)) {
      enabled = MQTT_RECEIPT_DOC[ENABLE_KEY];
      Serial.print("Enabled: ");
      Serial.println(enabled);
    }
  }
}

WiFiClient wifiClient;
PubSubClient mqttClient(MQTT_BROKER, MQTT_PORT, onMqttMessage, wifiClient);

void setup() {
  Serial.begin(115200);
  delay(10);
  Serial.println("setup");
    
  pinMode(ENABLE_ACCEPTOR_POWER_PIN, OUTPUT);
  pinMode(OVERRIDE_BUTTON_SIGNAL_PIN, INPUT_PULLUP);
  pinMode(COIN_SIGNAL_PIN, INPUT_PULLUP);

  // Setup WiFi.
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
  }

  // WiFi Connected.
  Serial.println("");
  Serial.println("WiFi connected");  
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // Determine Cell Id.
  {
    uint8_t mac[6];
    WiFi.macAddress(mac);
    AUTOMAT_CELL_ID = "automat/coin/" + macToStr(mac);
  }
  Serial.print("AutomatCellId: ");
  Serial.println(AUTOMAT_CELL_ID);

  // Setup MQTT
  Serial.println("Connecting to MQTT Server: ");
  Serial.println(MQTT_BROKER);
  if (!mqttClient.connect((char*) AUTOMAT_CELL_ID.c_str())) {
    Serial.println("MQTT connect failed; Reseting ...");
    abort();
  }
  Serial.print("Connected as: ");
  Serial.println(AUTOMAT_CELL_ID);

  COIN_ACCEPTED_TOPIC = AUTOMAT_CELL_ID + "/accepted";

  COIN_CONTROL_TOPIC = AUTOMAT_CELL_ID + "/control";
  Serial.print("Subscribing to environment updates: ");
  Serial.println(COIN_CONTROL_TOPIC);
  mqttClient.subscribe((char*) COIN_CONTROL_TOPIC.c_str(), 1);

  // ArduinoOTA
  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  // ArduinoOTA.setHostname("myesp8266");

  // No authentication by default
  // ArduinoOTA.setPassword((const char *)"123");

  ArduinoOTA.onStart([]() {
    Serial.println("Start");
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

bool coinButtonIsPressed() {
  return ((not digitalRead(OVERRIDE_BUTTON_SIGNAL_PIN)) | (not digitalRead(COIN_SIGNAL_PIN)));
}

unsigned int coin_count = 0;
StaticJsonDocument<256> MQTT_SEND_DOC;
char MQTT_SEND_BUFFER[256];

void loop() {
  digitalWrite(ENABLE_ACCEPTOR_POWER_PIN, enabled);
  
  if (coinButtonIsPressed()) {
    Serial.println("Coin");

    enabled = false;

    MQTT_SEND_DOC["seq"] = coin_count;
    MQTT_SEND_DOC["ms"] = millis();
    coin_count++;
   
    serializeJson(MQTT_SEND_DOC, MQTT_SEND_BUFFER);
    if (!mqttClient.publish((char*) COIN_ACCEPTED_TOPIC.c_str(), MQTT_SEND_BUFFER)) {
      // Send failure, rearm.
      enabled = true;
    }
  }
}
