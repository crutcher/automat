

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

void setup() {
  // Setup serial debugging.
  Serial.begin(115200, SERIAL_8N1, SERIAL_TX_ONLY);
  delay(10);
  Serial.println("");
  Serial.println("setup");

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_RX, INPUT);
}


void loop() {
  // builtin LED is active-low.
  //  if (millis() % 1000 > 900) {
  //    digitalWrite(LED_BUILTIN, LOW);
  //  } else {
  //    digitalWrite(LED_BUILTIN, HIGH);
  //  }

  delay(1000);

  Serial.print("RX:");
  Serial.println(digitalRead(PIN_RX));

}
