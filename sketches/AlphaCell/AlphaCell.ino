// Style:
// ClassNames.methodNames()
// ClassNames.member_names_
// globalFunctions()
// local_variables
// g_global_variables
// CONSTANT_VALUES

#include "automat.h"


#include <ArduinoOTA.h>

// Shutup FastLED pragma message:
#define FASTLED_INTERNAL
#include <FastLED.h>

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>


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
const int FACE_BUTTON_LED_PIN = PIN_D1;
const int FACE_BUTTON_PIN = PIN_D2;
const int DOOR_SENSOR_PIN = PIN_A0;
const int LATCH_SOLENOID_PIN = PIN_D8;

const int REAR_BUTTON_PIN = PIN_RX;

const int CELL_LIGHTS_CLOCK_PIN = PIN_D4;
const int CELL_LIGHTS_DATA_PIN = PIN_D3;

const int VFD_DATA_PIN = PIN_D0;
const int VFD_CLOCK_PIN = PIN_D5;
const int VFD_LATCH_PIN = PIN_D6;
const int VFD_OE_PIN = PIN_D7;


CRGB AUTOMAT_GREEN(22, 164, 40);


#ifndef WIFI_SSID
#define WIFI_SSID "1337"
#endif

#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD "password"
#endif

#ifndef MQTT_BROKER
#define MQTT_BROKER "10.1.1.198"
#endif

#ifndef MQTT_PORT
#define MQTT_PORT 1883
#endif


const String AUTOMAT_ENVIRONMENT_TOPIC = "automat/environment";

const String PHASE_KEY = "phase";
const String POWER_HIGH_KEY = "power_high";
const String POWER_LOW_KEY = "power_low";
const String GLITCH_KEY = "glitch";

const String ASCII_KEY = "ascii";
const String GLYPHS_KEY = "glyphs";
const String OVERLAY_KEY = "overlay";

const String MESSAGE_KEY = "message";
const String OPEN_KEY = "open";


String g_cell_control_topic; // "automat/cell/{ID}/control"
String g_automat_cell_id;


#define g_fastled_values_size 4

CRGB g_fastled_values[g_fastled_values_size];

void setLedColor(CRGB color) {
  for (int i = 0; i < g_fastled_values_size; ++i) {
    g_fastled_values[i] = color;
  }    
}

int scaleRange(int val, int in_low, int in_high, int out_low, int out_high) {
  return (val - in_low) * (out_high - out_low) / (in_high - in_low) + out_low;
}

/**
 * Clamp a value between [low, high].
 *
 * @param val: the value to clamp.
 * @param low: the lowest output, inclusive.
 * @param high: the highest output, inclusive.
 * @returns: the clamped value
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
 * @param mac: the mac value to format.
 * @returns: a string "XX:XX:XX:XX:XX:XX"
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

/**
 * A network syncable millis() timer.
 */
class PhaseTimer {
  public:
    PhaseTimer(
        unsigned long offset,
        unsigned int dwell_weight,
        unsigned int update_weight) {
      offset_ = offset;
      dwell_weight_ = dwell_weight;
      update_weight_ = update_weight;
    }

    unsigned long phase_time() {
      return millis() + offset_;
    }

    /**
     * Update the local idea of time with a remote message.
     *
     * We do not model roundtrip timing here; just average
     * phase time as it reaches us. The assumption is that
     * all nodes are roughly the same 'distance' from the
     * master.
     */
    void update_phase(unsigned long remote_time) {
      // This is what the offset of the remote time is wrt to our clock.
      // This ignores transmission delays.
      unsigned long remote_offset = remote_time - millis();
      
      if (!initialized_) {
        // The very first message is authoritative.
        initialized_ = true;
        offset_ = remote_offset;

      } else {
        // Subsequent messages are averaged.

        unsigned int denom = dwell_weight_ + update_weight_;

        // (a * old + b * new) / (a + b)
        // (a * old / (a + b)) + (b * new / (a + b))

        offset_ = dwell_weight_ * (offset_ / denom) + update_weight_ * (remote_offset / denom);
      }
    }
       
  private:
    unsigned int initialized_ = false;
    unsigned int dwell_weight_;
    unsigned int update_weight_;
    unsigned long offset_;
};

PhaseTimer g_phase_timer(0, 4, 1);


class CycleTimer {
  public:
    CycleTimer(long period_millis, PhaseTimer *phase_timer) {
      period_millis_ = period_millis;
      phase_timer_ = phase_timer;
    }

    long value() {
      return phase_timer_->phase_time() % period_millis_;
    }

    uint8_t byteValue() {
      return (256 * value()) / period_millis_;
    }

  private:
    long period_millis_;
    PhaseTimer *phase_timer_;
};

class PulseGenerator {
  public:
    PulseGenerator(long pulse_time, long delay_ms, PhaseTimer *phase_timer) {
      phase_timer_ = phase_timer;
      pulse_time_ = pulse_time;
      delay_ms_ = delay_ms;
      last_pulse_ = 0;
    }

    bool pulse() {
      bool p = false;
      unsigned long now = phase_timer_->phase_time();

      if (last_pulse_ + pulse_time_ > now) {
        // The previous pulse is still active.
        p = true;
        
      } else if (last_pulse_ + delay_ms_ < now) {
        // Start a new pulse.
        last_pulse_ = now;
        p = true;
      }

      return p;
    }

  private:
    PhaseTimer *phase_timer_;
    long pulse_time_;
    long delay_ms_;
    unsigned long last_pulse_;
};


class PurrGenerator {
  public:
    PurrGenerator(PhaseTimer *phase_timer) {
      purr_h1_ = new PulseGenerator(6, 150, phase_timer);
      purr_h2_ = new PulseGenerator(6, 75, phase_timer);
      purr_h3_ = new PulseGenerator(6, 37, phase_timer);
    }

    bool pulse() {
      return purr_h1_->pulse() | purr_h2_->pulse() | purr_h3_->pulse();
    }

  private:
   PulseGenerator *purr_h1_, *purr_h2_, *purr_h3_;
};

PurrGenerator purr(&g_phase_timer);



class CellPower {
  public:
    CellPower(
        PhaseTimer *phase_timer,
        uint8_t power_low,
        uint8_t power_high,
        uint8_t glitch) {
      phase_timer_ = phase_timer;
      heartbeat_ = new CycleTimer(6500, phase_timer);
      power_low_ = power_low;
      power_high_ = power_high;
      glitch_ = glitch;
    }

    void loop() {
      int wave = scaleRange(
        cubicwave8(heartbeat_->byteValue()),
        0, 255,
        power_low_, power_high_);

      wave += scaleRange(
        inoise8(phase_timer_->phase_time()),
        0, 255,
        -glitch_ / 8, glitch_ / 8);

      power_ = clamp(wave, 0, 255);
    }

    uint8_t power() {
      return power_;
    }

    void set_power_high(uint8_t power_high) {
      power_high_ = power_high;
    }

    void set_power_low(uint8_t power_low) {
      power_low_ = power_low;
    }

    uint8_t get_glitch() {
      return glitch_;
    }

    void update_glitch(uint8_t glitch) {
      if (random(10) == 0) {
        glitch_ = (((int) glitch) + ((int) glitch_)) / 2;
      }
    }

    void dump() {
      Serial.print("CellPower(");
      Serial.print(power_low_);
      Serial.print(", ");
      Serial.print(power_high_);
      Serial.print(")[");
      Serial.print(glitch_);
      Serial.println("]");
    }

  private:
    PhaseTimer *phase_timer_;
    CycleTimer *heartbeat_;
    
    uint8_t power_high_;
    uint8_t power_low_;
    uint8_t glitch_;
    uint8_t power_;
};

CellPower g_cell_power(&g_phase_timer, 0, 255, 60);


class VfdBank {
  public:
    uint8_t num_tubes_;
    byte *ghost_bank_;
    byte *solid_bank_;
    byte *flicker_bank_;

    long ghost_counter_ = 0;

    const long GHOST_RATIO = 35;

    VfdBank(
        uint8_t num_tubes,
        uint8_t data_pin,
        uint8_t clock_pin,
        uint8_t latch_pin,
        uint8_t oe_pin) {
      num_tubes_ = num_tubes;
      data_pin_ = data_pin;
      clock_pin_ = clock_pin;
      latch_pin_ = latch_pin;

      solid_bank_ = new byte[num_tubes];
      ghost_bank_ = new byte[num_tubes];
      flicker_bank_ = new byte[num_tubes];
    }

    void show() {
      ghost_counter_ = (ghost_counter_ + 1) % GHOST_RATIO;
      bool ghost_active = ghost_counter_ == 0;

      bool flicker_active = random(180, 256) < cubicwave8(inoise8(millis() / 2));

      digitalWrite(latch_pin_, LOW);
      for (int i = num_tubes_; i > 0; --i) {
        int idx = i - 1;

        byte value = solid_bank_[idx];

        if (ghost_active) {
          value |= ghost_bank_[idx];
        }

        if (flicker_active) {
          value &= ~flicker_bank_[idx];
        }

        shiftOut(data_pin_, clock_pin_, MSBFIRST, value);
      }
      digitalWrite(latch_pin_, HIGH);
    }

    void clear_solid() {
      for (int i = 0; i < num_tubes_; ++i) {
        solid_bank_[i] = 0;
      }
    }

    void clear_ghost() {
      for (int i = 0; i < num_tubes_; ++i) {
        ghost_bank_[i] = 0;
      }
    }

    void clear_flicker() {
      for (int i = 0; i < num_tubes_; ++i) {
        flicker_bank_[i] = 0;
      }
    }

    void random_flicker(int seed, int count) {
      clear_flicker();

      const unsigned int p1 = 16807;
      const unsigned int p2 = 0;
      const unsigned int N = (1 << 31) - 1;
      unsigned int z = (p1 * seed + p2) % N;

      for (int i = 0; i < count; ++i) {
        off_t bank = z % num_tubes_;
        z = (p1 * z + p2) % N;

        off_t bit = z % 8;
        z = (p1 * z + p2) % N;

        flicker_bank_[bank] |= 1 << bit;
      }
    }


    void clear() {
      clear_solid();
      clear_ghost();
      clear_flicker();
    }

  private:
    uint8_t data_pin_;
    uint8_t clock_pin_;
    uint8_t latch_pin_;

};

VfdBank vfd_bank(5, VFD_DATA_PIN, VFD_CLOCK_PIN, VFD_LATCH_PIN, VFD_OE_PIN);


StaticJsonDocument<256> g_onMqttMessage_parse_doc;

void onMqttMessage(char* topic, byte* payload, unsigned int payload_length) {
  Serial.print("onMqttMessage: ");
  Serial.println(topic);

  String topicStr(topic);
  
  // handle message arrived
  if (topicStr.equals(AUTOMAT_ENVIRONMENT_TOPIC)) {
    // Environment Update.
    Serial.println("MQTT Environment Update");

    // Zero-Copy Mode (modifies 'payload'):
    if (deserializeJson(g_onMqttMessage_parse_doc, payload) == DeserializationError::Ok) {

      if (g_onMqttMessage_parse_doc.containsKey(PHASE_KEY)) {
        g_phase_timer.update_phase(g_onMqttMessage_parse_doc[PHASE_KEY]);
        Serial.print("Phase: ");
        Serial.println(g_phase_timer.phase_time());
      }

      if (g_onMqttMessage_parse_doc.containsKey(POWER_LOW_KEY)) {
        g_cell_power.set_power_low(g_onMqttMessage_parse_doc[POWER_LOW_KEY]);
        Serial.println("Set power_low");
      }
      
      if (g_onMqttMessage_parse_doc.containsKey(POWER_HIGH_KEY)) {
        g_cell_power.set_power_high(g_onMqttMessage_parse_doc[POWER_HIGH_KEY]);
        Serial.println("Set power_high");
      }
      
      if (g_onMqttMessage_parse_doc.containsKey(GLITCH_KEY)) {
        g_cell_power.update_glitch(g_onMqttMessage_parse_doc[GLITCH_KEY]);
        Serial.println("Set glitch");
      }

      g_cell_power.dump();
      
    }

  } else if (topicStr.endsWith("/message")) {
    Serial.println("MQTT Message Update");
    // Zero-Copy Mode (modifies 'payload'):
    if (deserializeJson(g_onMqttMessage_parse_doc, payload) == DeserializationError::Ok) {
      vfd_bank.clear_solid();
      
      if (g_onMqttMessage_parse_doc.containsKey(ASCII_KEY)) {
        String msg = g_onMqttMessage_parse_doc[ASCII_KEY];
        Serial.print("Ascii: \"");
        Serial.print(msg);
        Serial.println("\"");

        int k = min((unsigned int) vfd_bank.num_tubes_, msg.length());
        for (int i = 0; i < k; ++i) {
          vfd_bank.solid_bank_[i] |= vfd_render_ascii_glyph(msg[i]);
        }
      }

      if (g_onMqttMessage_parse_doc.containsKey(OVERLAY_KEY)) {
        String msg = g_onMqttMessage_parse_doc[OVERLAY_KEY];
        Serial.print("Ascii Overlay: \"");
        Serial.print(msg);
        Serial.println("\"");

        int k = min((unsigned int) vfd_bank.num_tubes_, msg.length());
        for (int i = 0; i < k; ++i) {
          vfd_bank.solid_bank_[i] |= vfd_render_ascii_glyph(msg[i]);
        }
      }

      if (g_onMqttMessage_parse_doc.containsKey(GLYPHS_KEY)) {
        Serial.println("Glyphs");
        String msg = g_onMqttMessage_parse_doc[GLYPHS_KEY];

        int k = min((unsigned int) vfd_bank.num_tubes_, msg.length());
        for (int i = 0; i < k; ++i) {
          vfd_bank.solid_bank_[i] |= msg[i];
        }
      }
    }
  }
}

WiFiClient g_wificlient;
PubSubClient g_pubsubclient(MQTT_BROKER, MQTT_PORT, onMqttMessage, g_wificlient);

bool buttonIsPressed() {
  return (not digitalRead(FACE_BUTTON_PIN)) || (not digitalRead(REAR_BUTTON_PIN));
}

bool doorIsOpen() {
  return analogRead(DOOR_SENSOR_PIN) > 800;
}

void subscribe(String topic) {
  Serial.print("Subscribing to MQTT Topic: ");
  Serial.println(topic);
  g_pubsubclient.subscribe((char*) topic.c_str(), 1);
}

void mqttLoop() {
  static const int RECONNECT_DELAY_MS = 500;
  static long last_reconnect_attempt = 0;

  if (!g_pubsubclient.connected()) {
    long now = millis();
    if (last_reconnect_attempt + RECONNECT_DELAY_MS > now) {
      return;
    }
    last_reconnect_attempt = now;
  
    // Setup MQTT
    Serial.println("Connecting to MQTT Server: ");
    Serial.println(MQTT_BROKER);
    if (!g_pubsubclient.connect((char*) g_automat_cell_id.c_str())) {
      Serial.println("MQTT connect failed; will retry ...");
      return;
    }

    Serial.print("MQTT Connected as: ");
    Serial.println(g_automat_cell_id);

    subscribe(AUTOMAT_ENVIRONMENT_TOPIC);

    subscribe("automat/cell/all/#");
    subscribe(g_automat_cell_id + "/#");
  }

  if (g_pubsubclient.connected()) {
    g_pubsubclient.loop();
  }
}

void initializeOtaSupport() {
  // ArduinoOTA
  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  // ArduinoOTA.setHostname("myesp8266");

  // No authentication by default
  // ArduinoOTA.setPassword((const char *)"123");
  ArduinoOTA.setPassword((const char *) "automat/cell");

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

void setup() {
  // Setup serial debugging.
  Serial.begin(115200, SERIAL_8N1, SERIAL_TX_ONLY);
  delay(10);
  Serial.println("");
  Serial.println("setup");

  pinMode(FACE_BUTTON_LED_PIN, OUTPUT);
  pinMode(FACE_BUTTON_PIN, INPUT_PULLUP);  
 // pinMode(DOOR_SENSOR_PIN, INPUT_PULLUP);
  pinMode(LATCH_SOLENOID_PIN, OUTPUT);
  pinMode(REAR_BUTTON_PIN, INPUT);

  pinMode(VFD_DATA_PIN, OUTPUT);
  pinMode(VFD_CLOCK_PIN, OUTPUT);
  pinMode(VFD_LATCH_PIN, OUTPUT);
  pinMode(VFD_OE_PIN, OUTPUT);

  pinMode(CELL_LIGHTS_DATA_PIN, OUTPUT);
  pinMode(CELL_LIGHTS_CLOCK_PIN, OUTPUT);

  vfd_bank.clear();
  vfd_bank.show();

  FastLED.addLeds<
    WS2801,
    CELL_LIGHTS_DATA_PIN,
    CELL_LIGHTS_CLOCK_PIN,
    RGB>(
      g_fastled_values,
      g_fastled_values_size);

  // Setup WiFi.
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);

  analogWrite(VFD_OE_PIN, 900);
    
  setLedColor(AUTOMAT_GREEN);
  FastLED.setBrightness(20);
  FastLED.show();

  vfd_bank.clear();
  vfd_render_ascii_string(vfd_bank.solid_bank_, "    ?");
  vfd_bank.show();

  /* TODO(crutcher): Verify?
   * Explicitly set the ESP8266 to be a WiFi-client.
   * Otherwise, it by default, would try to act as both a client
   * and an access-point and could cause network-issues with your
    other WiFi-devices on your WiFi-network. */
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    analogWrite(VFD_OE_PIN, 950);
    FastLED.setBrightness(5);
    FastLED.show();
    
    digitalWrite(FACE_BUTTON_LED_PIN, LOW);
 
    delay(1000);
    analogWrite(VFD_OE_PIN, 900);

    FastLED.setBrightness(20);
    FastLED.show();
 
    digitalWrite(FACE_BUTTON_LED_PIN, HIGH);
      
    Serial.print(".");
  }

  // WiFi Connected.
  Serial.println("");
  Serial.println("WiFi connected");  
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // Determine Cell Id.
  uint8_t mac[6];
  {
    WiFi.macAddress(mac);
    g_automat_cell_id = "automat/cell/" + macToStr(mac);

    uint8_t seed = 42;
    for (int i = 0; i < sizeof(mac); ++i) {
      seed = seed ^ mac[i];
    }
    vfd_bank.random_flicker(seed, 7);
  }
  Serial.print("AutomatCellId: ");
  Serial.println(g_automat_cell_id);

  vfd_render_ascii_string(vfd_bank.solid_bank_, "  ...");
  vfd_bank.show();

  initializeOtaSupport();
}


// Door Latch Notes:
// https://www.vin.com/apputil/content/defaultadv1.aspx?id=5328490&pid=11349&print=1
// 'purr' is roughly:
//  if (millis() % 30 > 20) {
//    digitalWrite(LATCH_SOLENOID_PIN, HIGH);
//  } else {
//    digitalWrite(LATCH_SOLENOID_PIN, LOW);
//  }

unsigned long lastDigitMillis = 0;
const long DIGIT_DELAY = 1750;

unsigned long lastFlickerDigitMillis = 0;
const long FLICKER_DIGIT_DELAY = 600;


void loop() {
  ArduinoOTA.handle();

  mqttLoop();
  
  // Simulate Power Supply!
  g_cell_power.loop();

  // FIXME: debugging.
  if (buttonIsPressed()) {
    Serial.println("Button");
    digitalWrite(LATCH_SOLENOID_PIN, HIGH);;
  } else {
    digitalWrite(LATCH_SOLENOID_PIN, LOW);
  }

  setLedColor(AUTOMAT_GREEN);
  FastLED.setBrightness(g_cell_power.power());
  FastLED.show();

  // Generate ghost bleed garbage.
  long now = millis();
  if (now - lastFlickerDigitMillis > FLICKER_DIGIT_DELAY) {
    lastFlickerDigitMillis = now;

    // Right shift the existing content.
    for (int i = vfd_bank.num_tubes_; i > 1; --i) {
      vfd_bank.ghost_bank_[i-1] = vfd_bank.ghost_bank_[i-2];
    }
    // Add a new random value.
    byte garbage = random(32, 128);
    vfd_bank.ghost_bank_[0] = VfdAsciiGlyphMap[garbage];
  }

  // wemosd1's analogWrite is 0-1024 (not 0-255).
  // VFD_OE_PIN is active-low.
  analogWrite(VFD_OE_PIN, 1024 - (g_cell_power.power() * 4));
  vfd_bank.show();
}
