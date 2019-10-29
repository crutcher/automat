
// PINS:
// LED_BUILTIN

// FIXME(crutcher): We should be able to use IDE constants (D1, D2, etc).
// But it seems the board-setup-selection is wrong in our env; so
// instead we're redefining them for the Wemos D1
const int BUTTON_LED_PIN = 5; // D1
const int BUTTON_PIN = 2;     // D4
const int DOOR_PIN = 4;       // D2
const int DOOR_LATCH_PIN = 15;    // D8

void setup() {
  Serial.begin(115200);
  delay(10);
  
  Serial.println("");
  Serial.println("setup");

  pinMode(BUTTON_LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  pinMode(DOOR_PIN, INPUT_PULLUP);
  pinMode(DOOR_LATCH_PIN, OUTPUT);
}

bool buttonPressed() {
  return not digitalRead(BUTTON_PIN);
}

bool doorOpened() {
  return digitalRead(DOOR_PIN);
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

void loop() {
  if (!buttonPressed()) {
    digitalWrite(BUTTON_LED_PIN, doorOpened());
  } else {
    renderButtonLed();
  }
}
