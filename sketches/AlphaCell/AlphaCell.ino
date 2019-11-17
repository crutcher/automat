// Shutup FastLED pragma message:
#define FASTLED_INTERNAL
#include <FastLED.h>

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>


CRGB AUTOMAT_GREEN(22, 120, 50);

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


const String AUTOMAT_ENVIRONMENT_TOPIC = "automat/environment";

const String PHASE_KEY = "phase";
const String POWER_KEY = "power";
const String GLITCH_KEY = "glitch";


String AUTOMAT_CONTROL_TOPIC; // "automat/cell/{ID}/control"


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

#define LED_STRAND_SIZE 4

CRGB LED_STRAND[LED_STRAND_SIZE];

void setLedColor(CRGB color) {
  for (int i = 0; i < LED_STRAND_SIZE; ++i) {
    LED_STRAND[i] = color;
  }    
}


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

class PhaseTimer {
  public:
    PhaseTimer(
        unsigned long offset,
        unsigned int dwell_weight,
        unsigned int update_weight) {
      _offset = offset;
      _dwell_weight = dwell_weight;
      _update_weight = update_weight;
    }

    unsigned long phase_time() {
      return millis() + _offset;
    }

    void update_phase(unsigned long remote_time) {
      unsigned long local_time = phase_time();

      long delta = remote_time - local_time;

      _offset += (_update_weight * delta) / (_dwell_weight + _update_weight);
    }
       
  private:
    unsigned int _dwell_weight;
    unsigned int _update_weight;
    unsigned long _offset;
};

PhaseTimer phase_timer(0, 4, 1);


StaticJsonDocument<256> MQTT_RECEIPT_DOC;

void onMqttMessage(char* topic, byte* payload, unsigned int payload_length) {
  Serial.print("onMqttMessage: ");
  Serial.println(topic);
  
  // handle message arrived
  if (!strcmp(topic, (char*) AUTOMAT_ENVIRONMENT_TOPIC.c_str())) {
    // Environment Update.
    Serial.println("Environment Update");

    // Zero-Copy Mode (modifies 'payload'):
    if (deserializeJson(MQTT_RECEIPT_DOC, payload) == DeserializationError::Ok) {
      phase_timer.update_phase(MQTT_RECEIPT_DOC[PHASE_KEY]);
      Serial.print("Phase: ");
      Serial.println(phase_timer.phase_time());
    }
    
  } else if (!strcmp(topic, (char*) AUTOMAT_CONTROL_TOPIC.c_str())) {
    // Control Message.
    Serial.println("Control Message");
    
  }
}

WiFiClient wifiClient;
PubSubClient mqttClient(MQTT_BROKER, MQTT_PORT, onMqttMessage, wifiClient);

bool buttonIsPressed() {
  return not digitalRead(BUTTON_PIN);
}

bool doorIsOpen() {
  return analogRead(DOOR_PIN) > 800;
}


void setup() {
  pinMode(BUTTON_LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);  
 // pinMode(DOOR_PIN, INPUT_PULLUP);
  pinMode(DOOR_LATCH_PIN, OUTPUT);

  pinMode(VFD_DATA, OUTPUT);
  pinMode(VFD_CLOCK, OUTPUT);
  pinMode(VFD_LATCH, OUTPUT);
  pinMode(VFD_OE, OUTPUT);

  pinMode(LED_DATA_PIN, OUTPUT);
  pinMode(LED_CLOCK_PIN, OUTPUT);

  FastLED.addLeds<
    WS2801,
    LED_DATA_PIN,
    LED_CLOCK_PIN,
    RGB>(
      LED_STRAND,
      LED_STRAND_SIZE);

  // Setup serial debugging.
  Serial.begin(115200);
  delay(10);
  Serial.println("");
  Serial.println("setup");

  // Setup WiFi.
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    setLedColor(CRGB::Green);
    FastLED.show();
    digitalWrite(BUTTON_LED_PIN, LOW);
 
    delay(100);
    setLedColor(CRGB::Red);
    FastLED.show();
    digitalWrite(BUTTON_LED_PIN, HIGH);
      
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
    AUTOMAT_CELL_ID = "automat/cell/" + macToStr(mac);
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

  Serial.print("Subscribing to environment updates: ");
  Serial.println(AUTOMAT_ENVIRONMENT_TOPIC);
  mqttClient.subscribe((char*) AUTOMAT_ENVIRONMENT_TOPIC.c_str(), 1);

  AUTOMAT_CONTROL_TOPIC = AUTOMAT_CELL_ID + "/control";
  Serial.print("Subscribing to environment updates: ");
  Serial.println(AUTOMAT_CONTROL_TOPIC);
  mqttClient.subscribe((char*) AUTOMAT_CONTROL_TOPIC.c_str(), 1);
}
  
void loop() {
  FastLED.show();
  mqttClient.loop();
}
