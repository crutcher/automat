
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#ifndef WIFI_SSID
#define WIFI_SSID "bubbles"
#endif

#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD "monkeyshine42"
#endif

#ifndef MQTT_BROKER
#define MQTT_BROKER "10.133.10.98"
#endif

#ifndef MQTT_PORT
#define MQTT_PORT 1883
#endif


String AUTOMAT_CELL_ID;
String STATE_TOPIC;


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

void callback(char* topic, byte* payload, unsigned int length) {
  // handle message arrived
}

WiFiClient wifiClient;
PubSubClient client(MQTT_BROKER, MQTT_PORT, callback, wifiClient);

StaticJsonDocument<256> STATE_DOC;
char STATE_OUTPUT_BUFFER[128];

const char* OPEN_FIELD = "open";

const long STATE_TIMEOUT = 500;
long LAST_STATE_MILLIS = 0;
void markStateDirty() {
  LAST_STATE_MILLIS = 0;
}
void maybeSendState() {
  if (millis() - LAST_STATE_MILLIS < STATE_TIMEOUT) {
    return;
  }
  serializeJson(STATE_DOC, STATE_OUTPUT_BUFFER);
  if (client.publish((char*) STATE_TOPIC.c_str(), STATE_OUTPUT_BUFFER)) {
    LAST_STATE_MILLIS = millis();
  }
}

void setup() {
  Serial.begin(115200);
  delay(10);

  STATE_DOC[OPEN_FIELD] = false;

  Serial.println("");
  Serial.println("setup");

  {
    Serial.print("Connecting to ");
    Serial.println(WIFI_SSID);
  
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
    while (WiFi.status() != WL_CONNECTED) {
      Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected");  
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    {
      uint8_t mac[6];
      WiFi.macAddress(mac);
      AUTOMAT_CELL_ID = "automatCell/" + macToStr(mac);
    }
    STATE_TOPIC = AUTOMAT_CELL_ID + "/state";
    
    String helloTopic = AUTOMAT_CELL_ID + "/hello"; 
    if (client.connect((char*) AUTOMAT_CELL_ID.c_str()) &&
        client.publish((char*) helloTopic.c_str(), "hello")) {
      Serial.println("hello");
    } else {
      Serial.println("MQTT connect failed");
      Serial.println("Will reset and try again...");
      abort();
    }
  }

}

void loop() {

  {
    bool opened = doorOpened();
    if (STATE_DOC[OPEN_FIELD] != opened) {
      STATE_DOC[OPEN_FIELD] = opened;
      markStateDirty();
    }
  }

  client.loop();
}
