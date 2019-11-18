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

const int POWER_ACCEPTOR = PIN_D1;
const int COIN_BUTTON = PIN_D3;
const int COIN_ACCEPTED = PIN_D2;

void setup() {
  Serial.begin(115200);
  delay(10);
  Serial.println("setup");
    
  pinMode(POWER_ACCEPTOR, OUTPUT);
  pinMode(COIN_BUTTON, INPUT_PULLUP);
  pinMode(COIN_ACCEPTED, INPUT_PULLUP);

}

bool coinButtonIsPressed() {
  return ((not digitalRead(COIN_BUTTON)) | (not digitalRead(COIN_ACCEPTED)));
}

void loop() {
  digitalWrite(POWER_ACCEPTOR, HIGH);
  
  if (coinButtonIsPressed()) {
    Serial.println("Coin");
    digitalWrite(POWER_ACCEPTOR, LOW);
    delay(5000);
  }
}
