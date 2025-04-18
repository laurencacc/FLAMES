#define STEP_PIN 18
#define DIR_PIN 19
#define EN_PIN 5         // Connect EN pin of DRV8825 to this GPIO
#define BUTTON_PIN 15     // Button connected between GPIO15 and GND

bool motorRunning = false;
bool lastButtonState = HIGH;

void setup() {
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);        // Enable pin control
  pinMode(BUTTON_PIN, INPUT_PULLUP);  // Button to GND

  digitalWrite(DIR_PIN, HIGH);    // Set initial direction
  digitalWrite(EN_PIN, HIGH);     // Start with motor disabled (no power)

  Serial.begin(115200);
  Serial.println("Stepper toggle setup complete.");
}

void loop() {
  bool buttonState = digitalRead(BUTTON_PIN);

  handleButton();

  if (!systemStarted) {
    taskYIELD(); // Add this line
    return;
  }

  // Detect button press (falling edge)
  if (lastButtonState == HIGH && buttonState == LOW) {
    motorRunning = !motorRunning;  // Toggle motor state

    if (motorRunning) {
      digitalWrite(EN_PIN, LOW);   // Enable driver (motor coils energized)
      Serial.println("Motor ON");
    } else {
      digitalWrite(EN_PIN, HIGH);  // Disable driver (motor coils de-energized)
      Serial.println("Motor OFF");
    }

    delay(200);  // Debounce
  }

  lastButtonState = buttonState;

  if (motorRunning) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(800);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(800);
  } else {
    delay(10); // Light CPU usage when idle
  }

   taskYIELD();
}
