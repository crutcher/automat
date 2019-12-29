#include "automat.h"

// Shutup FastLED pragma message:
#define FASTLED_INTERNAL
#include <FastLED.h>

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

#define VFD_DOT 64

// FIXME(crutcher): We should be able to use IDE constants (D1, D2, etc).
// But it seems the board-setup-selection is wrong in our env; so
// instead we're redefining them for the Wemos D1
const int BUTTON_LED_PIN = PIN_D1;
const int BUTTON_PIN = PIN_D2;
const int DOOR_PIN = PIN_A0;
const int DOOR_LATCH_PIN = PIN_D8;
const int CELL_RESET = PIN_RX;


const int LED_CLOCK_PIN = PIN_D4;
const int LED_DATA_PIN = PIN_D3;

const int VFD_DATA = PIN_D0;
const int VFD_CLOCK = PIN_D5;
const int VFD_LATCH = PIN_D6;
const int VFD_OE = PIN_D7;

class OffsetTimer {
  public:
    OffsetTimer() {
      set_timestamp(0);
    }

    void set_timestamp(long now) {
      _offset = now - millis();
    }

    long now() {
      return _offset + millis();
    }

  private:
    long _offset;
};

class CycleTimer {
  public:
    CycleTimer(long period_millis, OffsetTimer *offset) {
      _period_millis = period_millis;
      _offset = offset;
    }

    long value() {
      return _offset->now() % _period_millis;
    }

    uint8_t byteValue() {
      return (256 * value()) / _period_millis;
    }

  private:
    long _period_millis;
    OffsetTimer *_offset;
};

OffsetTimer OFFSET_TIMER;
CycleTimer HEARTBEAT(6500, &OFFSET_TIMER);
CycleTimer EYE(43000, &OFFSET_TIMER);
CycleTimer FLICKER(350, &OFFSET_TIMER);


#define NUM_LEDS 4

CRGB leds[NUM_LEDS];

const int NUM_VFD_TUBES = 5;
byte VFD_TUBES[NUM_VFD_TUBES];
byte VFD_XOR[NUM_VFD_TUBES];


// TODO(crutcher): set SEED from mac.
int SEED = 47;

void setup() {
  Serial.begin(115200);
  delay(10);

  Serial.println("");
  Serial.print("A0");
  Serial.println(PIN_A0);

  pinMode(BUTTON_LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);  
  pinMode(DOOR_PIN, INPUT);
  pinMode(DOOR_LATCH_PIN, OUTPUT);

  pinMode(CELL_RESET, FUNCTION_3); 
  pinMode(CELL_RESET, INPUT_PULLUP);

  pinMode(VFD_DATA, OUTPUT);
  pinMode(VFD_CLOCK, OUTPUT);
  pinMode(VFD_LATCH, OUTPUT);
  pinMode(VFD_OE, OUTPUT);

  pinMode(LED_DATA_PIN, OUTPUT);
  pinMode(LED_CLOCK_PIN, OUTPUT);

  FastLED.addLeds<WS2801, LED_DATA_PIN, LED_CLOCK_PIN, RGB>(leds, NUM_LEDS);
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
 
  digitalWrite(BUTTON_LED_PIN, buttonAnimState);
  digitalWrite(DOOR_LATCH_PIN, buttonAnimState);

}

int brightness = 0;
long lastDigitMillis = 0;
long DIGIT_DELAY = 800;

CRGB kolor(22, 120, 50);



int count = 0;

void showVFDTubes() {
  digitalWrite(VFD_LATCH, LOW);
  for (int i = NUM_VFD_TUBES - 1; i >= 0; --i) {
    byte val = VFD_TUBES[i] ^ VFD_XOR[i];
    if (i == 2) {
      if (count++ % 20) {
        val = 0; 
      }
    }
    shiftOut(VFD_DATA, VFD_CLOCK, MSBFIRST, val);
  }
  digitalWrite(VFD_LATCH, HIGH);
}

int clamp(int val, int low, int high) {
  if (val > high) {
    return high;
  } else if (val < low) {
    return low;
  }
  return val;
}

long lastFlicker = 0;
bool flickerOn = false;


void loop() {
  long now = millis();

  brightness = clamp(
    cubicwave8(HEARTBEAT.byteValue()) + inoise8(OFFSET_TIMER.now()) / 4 - 32,
    0, 255);
  
  // wemosd1's analogWrite is 0-1024 (not 0-255).
  // VFD_OE is active-low.
  analogWrite(VFD_OE, 1024 - (brightness * 4));

  String content = "foxxy";

  vfd_render_ascii_string(VFD_TUBES, content);

  showVFDTubes();

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
  FastLED.setBrightness(brightness);
  FastLED.show();

  renderButtonLed();
  if (!buttonPressed()) {
    digitalWrite(BUTTON_LED_PIN, doorOpened());
  }
}
