
const int COIN_PULSE_PIN = 2;
const unsigned long COIN_PULSE_DONE_DELAY_MS = 100;

volatile int lastCoin = 0;
volatile int coinCount = 0;
volatile int coinPulseCounter = 0;
volatile unsigned long lastPulseMillis = 0;

void setup() {
  Serial.begin(9600);

  pinMode(COIN_PULSE_PIN, INPUT_PULLUP);
  attachInterrupt(
    digitalPinToInterrupt(COIN_PULSE_PIN),
    coinPulseInterrupt,
    FALLING);
}

void checkCoin() {
  if (coinPulseCounter > 0) {
    if ((lastPulseMillis + COIN_PULSE_DONE_DELAY_MS) < millis()) {
      lastCoin = coinPulseCounter;
      coinPulseCounter = 0;
      
      coinCount++;
    }
  }
}

void coinPulseInterrupt() {
  checkCoin();
  
  coinPulseCounter++;
  lastPulseMillis = millis();
}


// the loop function runs over and over again forever
void loop() {
  noInterrupts();
  checkCoin();
  interrupts();
  
  delay(1000);
  Serial.print("lastCoin:");
  Serial.print(lastCoin);
  Serial.print(" coins:");
  Serial.println(coinCount);
}
