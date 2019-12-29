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


// ABCDEFGH
//
// 7-Segment Map:
//
//   DDD
//  F   C
//  F   C
//   EEE
//  G   A
//  G   A
//   HHH  BB

const uint8_t vfd_character_map[] PROGMEM = {
  0b10110111, //   0  "0"
  0b10100000, //   1  "1"
  0b00111011, //   2  "2"
  0b10111001, //   3  "3"
  0b10101100, //   4  "4"
  0b10011101, //   5  "5"
  0b10011111, //   6  "6"
  0b10110000, //   7  "7"
  0b10111111, //   8  "8"
  0b10111101, //   9  "9"
  0b10111110, //  10  "A"
  0b10001111, //  11  "b"
  0b00010111, //  12  "C"
  0b10101011, //  13  "d"
  0b00011111, //  14  "E"
  0b00011110, //  15  "F"
  0b00000000, //  16  NO DISPLAY
  0b00000000, //  17  NO DISPLAY
  0b00000000, //  18  NO DISPLAY
  0b00000000, //  19  NO DISPLAY
  0b00000000, //  20  NO DISPLAY
  0b00000000, //  21  NO DISPLAY
  0b00000000, //  22  NO DISPLAY
  0b00000000, //  23  NO DISPLAY
  0b00000000, //  24  NO DISPLAY
  0b00000000, //  25  NO DISPLAY
  0b00000000, //  26  NO DISPLAY
  0b00000000, //  27  NO DISPLAY
  0b00000000, //  28  NO DISPLAY
  0b00000000, //  29  NO DISPLAY
  0b00000000, //  30  NO DISPLAY
  0b00000000, //  31  NO DISPLAY
  0b00000000, //  32  ' '
  0b01100000, //  33  '!'
  0b00100100, //  34  '"'
  0b00111100, //  35  '#'
  0b00000000, //  36  '$'  NO DISPLAY
  0b10000100, //  37  '%'
  0b00011001, //  38  '&'
  0b00100000, //  39  '''
  0b00010111, //  40  '('
  0b10110001, //  41  ')'
  0b01001000, //  42  '*'
  0b00001110, //  43  '+'
  0b10000000, //  44  ','
  0b00001000, //  45  '-'
  0b01000000, //  46  '.'
  0b00101010, //  47  '/'
  0b10110111, //  48  "0"
  0b10100000, //  49  "1"
  0b00111011, //  50  "2"
  0b10111001, //  51  "3"
  0b10101100, //  52  "4"
  0b10011101, //  53  "5"
  0b10011111, //  54  "6"
  0b10110000, //  55  "7"
  0b10111111, //  56  "8"
  0b10111101, //  57  "9"

  0b00010001, //  58  ':'
  0b10010000, //  59  ';'
  0b00001100, //  60  '<'
  0b00001001, //  61  '='
  0b00101000, //  62  '>'
  0b01110000, //  63  '?'
  0b10011011, //  64  '@'

  0b10111110, //  65  "A"
  0b10001111, //  66  "b"
  0b00010111, //  67  "C"
  0b10101011, //  68  "d"
  0b00011111, //  69  "E"
  0b00011110, //  70  "F"

  0b10010111, //  71  'G'
  0b10101110, //  72  'H'
  
  0b10100000, //  73  'I'
  0b10100011, //  74  'J'

  0b01101110, //  75  'k'
  0b00000111, //  76  'L'
  0b00100100, //  77  'm'

  0b10001010, //  78  'n'
  0b10110111, //  79  'O'

  0b00111110, //  80  'P'
  0b10111100, //  81  'q'
  0b00001010, //  82  'r'
  0b10011101, //  83  's'
  0b00001111, //  84  't'
  0b10100111, //  85  'U'
  0b10100011, //  86  'V'

  0b00100101, //  87  'w'
  0b00100010, //  88  'X'
  0b10101101, //  89  'Y'
  0b00110011, //  90  'Z'

  0b00010111, //  91  '['
  0b10001100, //  92  '\'
  0b10110001, //  93  ']'
  0b00110100, //  94  '^'

  0b00000001, //  95  '_'
  0b00000100, //  96  '`'
  
  0b10111110, //  97  "A"
  0b10001111, //  98  "b"
  0b00001011, //  99  "c"
  0b10101011, // 100  "d"
  0b00111111, // 101  "e"
  0b00011110, // 102  "F"

  0b10010111, // 103  'G'
  0b10001110, // 104  'h'
  
  0b10000000, // 105  'i'
  0b10100011, // 106  'j'

  0b01101110, // 107  'k'
  0b10100000, // 108  'l'
  0b00100100, // 109  'm'

  0b10001010, // 110  'n'
  0b10001011, // 111  'o'

  0b00111110, // 112  'P'
  0b10111100, // 113  'q'
  0b00001010, // 114  'r'
  0b10011101, // 115  's'
  0b00001111, // 116  't'
  0b10000011, // 117  'u'
  0b10000001, // 118  'v'

  0b00100101, // 119  'w'
  0b00100010, // 120  'X'
  0b10101101, // 121  'Y'
  0b00110011, // 122  'Z'

  0b00000000, // 123  '0b' NO DISPLAY
  0b00000110, // 124  '|'
  0b10000000, // 125  ','
  0b00001000, // 126  '~'
  0b00000000, // 127  'DEL' NO DISPLAY
  
// ABCDEFGH
//
// 7-Segment Map:
//
//   DDD
//  F   C
//  F   C
//   EEE
//  G   A
//  G   A
//   HHH  BB
};

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

  VFD_TUBES[0] = vfd_character_map['a'];
  VFD_TUBES[1] = vfd_character_map['<'] | vfd_character_map['='];
  VFD_TUBES[2] = vfd_character_map['b'];
  VFD_TUBES[3] = vfd_character_map['!'];
  VFD_TUBES[4] = vfd_character_map['!'];

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
