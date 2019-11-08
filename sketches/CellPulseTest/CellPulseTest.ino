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

void showVFDTubes() {
  digitalWrite(VFD_LATCH, LOW);
  for (int i = NUM_VFD_TUBES - 1; i >= 0; --i) {
    shiftOut(VFD_DATA, VFD_CLOCK, MSBFIRST, VFD_TUBES[i] ^ VFD_XOR[i]);
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

  if (now - lastFlicker > 2 * inoise8(millis())) {
    lastFlicker = now;
    if (flickerOn) {
      VFD_XOR[1] = 0;
      flickerOn = false;
    } else {
      VFD_XOR[1] = 4;
      flickerOn = true;
    }
  }

  analogWrite(BUTTON_LED_PIN, brightness * 4);

  if (now - lastDigitMillis > DIGIT_DELAY) {
    Serial.print("brightness");
    Serial.println(brightness);
    
    lastDigitMillis = now;
    // Shift Right.
    for (int i = NUM_VFD_TUBES; i > 0; --i) {
      VFD_TUBES[i] = VFD_TUBES[i-1];
    }
    VFD_TUBES[0] = (millis() / 100) & 0xFF;
  }

  int eyePos = clamp(5 * cubicwave8(EYE.byteValue()) / 255, 0, 4);
  for (int i = 0; i < NUM_VFD_TUBES; ++i) {
    VFD_TUBES[i] &= 0xbf;
  }
  VFD_TUBES[eyePos] |= 0x40;
  
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
