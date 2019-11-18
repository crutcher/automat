#include <ArduinoOTA.h>

// Shutup FastLED pragma message:
#define FASTLED_INTERNAL
#include <FastLED.h>

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>


CRGB AUTOMAT_GREEN(22, 164, 40);


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
const String POWER_HIGH_KEY = "power_high";
const String POWER_LOW_KEY = "power_low";
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

const int CELL_RESET_PIN = PIN_RX;

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

int scale(int val, int in_low, int in_high, int out_low, int out_high) {
  return (val - in_low) * (out_high - out_low) / (in_high - in_low) + out_low;
}

/**
 * Clamp a value between [low, high].
 */
int clamp(int val, int low, int high) {
  if (val > high) {
    return high;
  } else if (val < low) {
    return low;
  }
  return val;
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
      unsigned long remote_offset = remote_time - millis();
      
      if (!_initialized) {
        _initialized = true;
        _offset = remote_offset;

      } else {
        unsigned int denom = _dwell_weight + _update_weight;

        // (a * old + b * new) / (a + b)
        // (a * old / (a + b)) + (b * new / (a + b))

        _offset = _dwell_weight * (_offset / denom) + _update_weight * (remote_offset / denom);
      }
    }
       
  private:
    unsigned int _initialized = false;
    unsigned int _dwell_weight;
    unsigned int _update_weight;
    unsigned long _offset;
};

PhaseTimer phase_timer(0, 4, 1);


class CycleTimer {
  public:
    CycleTimer(long period_millis, PhaseTimer *phase_timer) {
      _period_millis = period_millis;
      _phase_timer = phase_timer;
    }

    long value() {
      return _phase_timer->phase_time() % _period_millis;
    }

    uint8_t byteValue() {
      return (256 * value()) / _period_millis;
    }

  private:
    long _period_millis;
    PhaseTimer *_phase_timer;
};

class PulseGenerator {
  public:
    PulseGenerator(long pulse_time, long delay_ms, PhaseTimer *phase_timer) {
      _phase_timer = phase_timer;
      _pulse_time = pulse_time;
      _delay_ms = delay_ms;
      _last_pulse = 0;
    }

    bool pulse() {
      bool p = false;
      unsigned long now = _phase_timer->phase_time();

      if (_last_pulse + _pulse_time > now) {
        // The previous pulse is still active.
        p = true;
        
      } else if (_last_pulse + _delay_ms < now) {
        // Start a new pulse.
        _last_pulse = now;
        p = true;
      }

      return p;
    }

  private:
    PhaseTimer *_phase_timer;
    long _pulse_time;
    long _delay_ms;
    unsigned long _last_pulse;
};


class PurrGenerator {
  public:
    PurrGenerator(PhaseTimer *phase_timer) {
      _purr_h1 = new PulseGenerator(6, 150, phase_timer);
      _purr_h2 = new PulseGenerator(6, 75, phase_timer);
      _purr_h3 = new PulseGenerator(6, 37, phase_timer);
    }

    bool pulse() {
      return _purr_h1->pulse() | _purr_h2->pulse() | _purr_h3->pulse();
    }

  private:
   PulseGenerator *_purr_h1, *_purr_h2, *_purr_h3;
};

PurrGenerator purr(&phase_timer);



class CellPower {
  public:
    CellPower(
        PhaseTimer *phase_timer,
        uint8_t power_low,
        uint8_t power_high,
        uint8_t glitch) {
      _phase_timer = phase_timer;
      _heartbeat = new CycleTimer(6500, phase_timer);
      _power_low = power_low;
      _power_high = power_high;
      _glitch = glitch;
    }

    void loop() {
      int wave = scale(
        cubicwave8(_heartbeat->byteValue()),
        0, 255,
        _power_low, _power_high);

      wave += scale(
        inoise8(_phase_timer->phase_time()),
        0, 255,
        -_glitch / 8, _glitch / 8);

      _power = clamp(wave, 0, 255);
    }

    uint8_t power() {
      return _power;
    }

    void set_power_high(uint8_t power_high) {
      _power_high = power_high;
    }

    void set_power_low(uint8_t power_low) {
      _power_low = power_low;
    }

    uint8_t get_glitch() {
      return _glitch;
    }

    void update_glitch(uint8_t glitch) {
      if (random(10) == 0) {
        _glitch = (((int) glitch) + ((int) _glitch)) / 2;
      }
    }

    void dump() {
      Serial.print("CellPower(");
      Serial.print(_power_low);
      Serial.print(", ");
      Serial.print(_power_high);
      Serial.print(")[");
      Serial.print(_glitch);
      Serial.println("]");
    }

  private:
    PhaseTimer *_phase_timer;
    CycleTimer *_heartbeat;
    
    uint8_t _power_high;
    uint8_t _power_low;
    uint8_t _glitch;
    uint8_t _power;
};

CellPower cell_power(&phase_timer, 0, 255, 255);

/**
class VfdBank< {
  public:
    VfdBank(uint8_t num_tubes, uint8_t data_pin, uint8_t clock_pin, uint8_t latch_pin, uint8_t oe_pin) {
      _num_tubes = num_tubes;
      _data_pin = data_pin;
      _clock_pin = clock_pin;
      _latch_pin = latch_pin;
    }

  private:
    uint8_t _num_tubes;
    uint8_t _data_pin;
    uint8_t _clock_pin;
    uint8_t _latch_pin;

    byte[] bank
}
*/

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

      if (MQTT_RECEIPT_DOC.containsKey(PHASE_KEY)) {
        phase_timer.update_phase(MQTT_RECEIPT_DOC[PHASE_KEY]);
        Serial.print("Phase: ");
        Serial.println(phase_timer.phase_time());
      }

      if (MQTT_RECEIPT_DOC.containsKey(POWER_LOW_KEY)) {
        cell_power.set_power_low(MQTT_RECEIPT_DOC[POWER_LOW_KEY]);
        Serial.println("Set power_low");
      }
      
      if (MQTT_RECEIPT_DOC.containsKey(POWER_HIGH_KEY)) {
        cell_power.set_power_high(MQTT_RECEIPT_DOC[POWER_HIGH_KEY]);
        Serial.println("Set power_high");
      }
      
      if (MQTT_RECEIPT_DOC.containsKey(GLITCH_KEY)) {
        cell_power.update_glitch(MQTT_RECEIPT_DOC[GLITCH_KEY]);
        Serial.println("Set glitch");
      }

      cell_power.dump();
      
    }
    
  } else if (!strcmp(topic, (char*) AUTOMAT_CONTROL_TOPIC.c_str())) {
    // Control Message.
    Serial.println("Control Message");
    
  }
}

WiFiClient wifiClient;
PubSubClient mqttClient(MQTT_BROKER, MQTT_PORT, onMqttMessage, wifiClient);

bool buttonIsPressed() {
  return (not digitalRead(BUTTON_PIN)) || (not digitalRead(CELL_RESET_PIN));
}

bool doorIsOpen() {
  return analogRead(DOOR_PIN) > 800;
}


void setup() {
  // Setup serial debugging.
  Serial.begin(115200, SERIAL_8N1, SERIAL_TX_ONLY);
  delay(10);
  Serial.println("");
  Serial.println("setup");

  pinMode(BUTTON_LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);  
 // pinMode(DOOR_PIN, INPUT_PULLUP);
  pinMode(DOOR_LATCH_PIN, OUTPUT);
  pinMode(CELL_RESET_PIN, INPUT);

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


// Door Latch Notes:
// https://www.vin.com/apputil/content/defaultadv1.aspx?id=5328490&pid=11349&print=1
// 'purr' is roughly:
//  if (millis() % 30 > 20) {
//    digitalWrite(DOOR_LATCH_PIN, HIGH);
//  } else {
//    digitalWrite(DOOR_LATCH_PIN, LOW);
//  }

void loop() {
  ArduinoOTA.handle();
  mqttClient.loop();
  
  // FIXME: debugging.
  if (buttonIsPressed()) {
      digitalWrite(DOOR_LATCH_PIN, purr.pulse());
  } else {
    digitalWrite(DOOR_LATCH_PIN, LOW);
  }

  // POWER!
  cell_power.loop();
  FastLED.setBrightness(cell_power.power());
  // wemosd1's analogWrite is 0-1024 (not 0-255).
  // VFD_OE is active-low.
  analogWrite(VFD_OE, 1024 - (cell_power.power() * 4));

  setLedColor(AUTOMAT_GREEN);
    
  FastLED.show();
}
