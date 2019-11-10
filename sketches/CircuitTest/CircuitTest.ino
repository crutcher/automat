// Shutup FastLED pragma message:
#define FASTLED_INTERNAL
#include <FastLED.h>


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


void setup() {
  Serial.begin(115200);
  delay(10);

  Serial.println("");
  Serial.println("setup()");

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
  digitalWrite(VFD_OE, LOW);

  long now = millis();
  if (now - lastDigitMillis > DIGIT_DELAY) {
    lastDigitMillis = now;
    digitalWrite(VFD_LATCH, LOW);
    shiftOut(VFD_DATA, VFD_CLOCK, MSBFIRST, (millis() / 100) & 0xFF);
    digitalWrite(VFD_LATCH, HIGH);
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
}
