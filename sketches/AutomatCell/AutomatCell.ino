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


String AUTOMAT_CELL_ID;
String STATE_TOPIC;

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

// FIXME(crutcher): We should be able to use IDE constants (D1, D2, etc).
// But it seems the board-setup-selection is wrong in our env; so
// instead we're redefining them for the Wemos D1
const int BUTTON_LED_PIN = PIN_D1;
const int BUTTON_PIN = PIN_D2;
const int DOOR_PIN = PIN_A0;
const int DOOR_LATCH_PIN = PIN_D8;

const int LED_CLOCK_PIN = PIN_D4;
const int LED_DATA_PIN = PIN_D3;

const int VFD_DATA = PIN_D0;
const int VFD_CLOCK = PIN_D5;
const int VFD_LATCH = PIN_D6;
const int VFD_OE = PIN_D7;

#define NUM_LEDS 4

CRGB leds[NUM_LEDS];


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

  Serial.println("");
  Serial.print("A0");
  Serial.println(PIN_A0);




  STATE_DOC[OPEN_FIELD] = false;

  pinMode(BUTTON_LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);  
 // pinMode(DOOR_PIN, INPUT_PULLUP);
  pinMode(DOOR_LATCH_PIN, OUTPUT);

  pinMode(VFD_DATA, OUTPUT);
  pinMode(VFD_CLOCK, OUTPUT);
  pinMode(VFD_LATCH, OUTPUT);
  pinMode(VFD_OE, OUTPUT);

  pinMode(PIN_RX, OUTPUT);

  pinMode(LED_DATA_PIN, OUTPUT);
  pinMode(LED_CLOCK_PIN, OUTPUT);

  FastLED.addLeds<WS2801, LED_DATA_PIN, LED_CLOCK_PIN, RGB>(leds, NUM_LEDS);

  Serial.println("");
  Serial.println("setup");

  digitalWrite(BUTTON_LED_PIN, HIGH);
  {
    Serial.print("Connecting to ");
    Serial.println(WIFI_SSID);
  
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
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
  return analogRead(DOOR_PIN) > 800;
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

  digitalWrite(PIN_RX, buttonAnimState);
  
  digitalWrite(BUTTON_LED_PIN, buttonAnimState);
  digitalWrite(DOOR_LATCH_PIN, buttonAnimState);

}

int brightness = 0;
long lastDigitMillis = 0;
long DIGIT_DELAY = 800;

CRGB kolor(22, 120, 50);

void loop() {
//  digitalWrite(VFD_OE, HIGH);
  brightness = (brightness + 30) % 255;
//  analogWrite(VFD_OE, brightness);
  digitalWrite(VFD_OE, HIGH);

  long now = millis();
  if (now - lastDigitMillis > DIGIT_DELAY) {
    lastDigitMillis = now;
    digitalWrite(VFD_LATCH, LOW);
    shiftOut(VFD_DATA, VFD_CLOCK, MSBFIRST, (millis() / 100) & 0xFF);
    digitalWrite(VFD_LATCH, HIGH);
  }

  {
    bool opened = doorOpened();
    if (STATE_DOC[OPEN_FIELD] != opened) {
      STATE_DOC[OPEN_FIELD] = opened;
      markStateDirty();
    }
  }

  for (int i = 0; i < NUM_LEDS; ++i) {
    if (buttonPressed()) {
      if ((millis() / 200) % 2) {
        leds[i] = CRGB::Red;
      } else {
        leds[i] = CRGB::Blue;
      }
    } else {
      leds[i] = kolor;
    }
  }
  FastLED.show();

  renderButtonLed();
  if (!buttonPressed()) {
    digitalWrite(BUTTON_LED_PIN, doorOpened());
  }

  maybeSendState();
}
