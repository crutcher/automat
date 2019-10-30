#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

const char* ssid = "bubbles";
const char* password = "monkeyshine42";
const char* MQTT_BROKER = "192.168.43.131";
const int MQTT_PORT = 1883;

String AUTOMAT_CELL_ID;
String STATE_TOPIC;

// PINS:
// LED_BUILTIN

// FIXME(crutcher): We should be able to use IDE constants (D1, D2, etc).
// But it seems the board-setup-selection is wrong in our env; so
// instead we're redefining them for the Wemos D1
const int BUTTON_LED_PIN = 5; // D1
const int BUTTON_PIN = 2;     // D4
const int DOOR_PIN = 4;       // D2
const int DOOR_LATCH_PIN = 15;    // D8

String macToStr(const uint8_t* mac) {
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

  pinMode(BUTTON_LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);  
  pinMode(DOOR_PIN, INPUT_PULLUP);
  pinMode(DOOR_LATCH_PIN, OUTPUT);

  Serial.println("");
  Serial.println("setup");

  digitalWrite(BUTTON_LED_PIN, HIGH);
  {
    Serial.print("Connecting to ");
    Serial.println(ssid);
  
    WiFi.begin(ssid, password);
  
    while (WiFi.status() != WL_CONNECTED) {
      delay(100);
      digitalWrite(BUTTON_LED_PIN, LOW);
      delay(100);
      digitalWrite(BUTTON_LED_PIN, HIGH);
      
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

bool buttonPressed() {
  return not digitalRead(BUTTON_PIN);
}

bool doorOpened() {
  return digitalRead(DOOR_PIN);
}

const long buttonAnimDelay = 100;
long buttonAnimTimer = 0;
bool buttonAnimState = false;

void renderButtonLed() {
  if (!buttonPressed()) {
    digitalWrite(DOOR_LATCH_PIN, LOW);
    return;
  }

  long now = millis();
  if (now - buttonAnimTimer > buttonAnimDelay) {
    buttonAnimState = !buttonAnimState;
    buttonAnimTimer = now;
  }

  digitalWrite(BUTTON_LED_PIN, buttonAnimState);
  digitalWrite(DOOR_LATCH_PIN, buttonAnimState);
}

void loop() {
  {
    bool opened = doorOpened();
    if (STATE_DOC[OPEN_FIELD] != opened) {
      STATE_DOC[OPEN_FIELD] = opened;
      markStateDirty();
    }
  }
  
  if (!buttonPressed()) {
    digitalWrite(BUTTON_LED_PIN, doorOpened());
  } else {
    renderButtonLed();
  }

  maybeSendState();
}
