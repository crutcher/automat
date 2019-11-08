

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



const int dataPin = PIN_D0;
const int clockPin = PIN_D5;
const int latchPin = PIN_D6;
const int oePin = PIN_D7;

void setup() {
  pinMode(latchPin, OUTPUT);
  pinMode(dataPin, OUTPUT);  
  pinMode(clockPin, OUTPUT);
  pinMode(oePin, OUTPUT);

  pinMode(LED_BUILTIN, OUTPUT);
}

int clamp(int val, int lower, int upper) {
  if (val > upper) {
    return upper;
  } else if (val < lower) {
    return lower;
  }
  return val;
}

int brightness = 0;
long lastDigitMillis = 0;
long DIGIT_DELAY = 800;

void loop() {
  brightness = (brightness + 30) % 255;
//  analogWrite(oePin, brightness);
  digitalWrite(oePin, HIGH);

  long now = millis();
  if (now - lastDigitMillis > DIGIT_DELAY) {
    lastDigitMillis = now;
    digitalWrite(latchPin, LOW);
    shiftOut(dataPin, clockPin, MSBFIRST, (millis() / 100) & 0xFF);
    digitalWrite(latchPin, HIGH);

    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
  }
  delay(100);
}
