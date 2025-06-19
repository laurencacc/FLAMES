  #include <Adafruit_Sensor.h>
  #include <Adafruit_TSL2591.h>

  // --- Pins ---
  #define STEP_PIN     18
  #define DIR_PIN      19
  // #define EN_PIN       5
  #define BUTTON_PIN   26
  #define BUZZER_PIN   33
  #define MOTOR_LED    4

  bool fireFlag = false;


  // --- Stepper Motor Control ---
  const int stepDelayMicros = 600;
  volatile bool motorRunning = false;
  volatile bool motorPermanentlyStopped = false;
  bool systemStarted = false;
  bool buttonPreviouslyPressed = false;

  bool ledBlinking = false;
  unsigned long lastBlinkTime = 0;
  const unsigned long blinkInterval = 250;

  long stepCount = 0;
  const int stepsPer90 = 1600;

  // --- Sensor ---
  Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);
  // unsigned long lastSensorCheck = 0;
  const unsigned long sensorInterval = 500;

  // --- Fire Detection (EMA) ---
  uint16_t ema = 0;
  bool ema_initialized = false;
  const uint8_t EMA_ALPHA = 20;  // EMA weight out of 64
  const uint16_t THRESHOLD_OFFSET = 200;

  // --- Tasks ---
  TaskHandle_t stepperTaskHandle;
  TaskHandle_t sensorTaskHandle;


  void setup() {
    Serial.begin(9600);

    pinMode(MOTOR_LED, OUTPUT);
    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    // pinMode(EN_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    digitalWrite(MOTOR_LED, LOW);
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(DIR_PIN, HIGH);
    // digitalWrite(EN_PIN, HIGH);  // Motor off at start

    tsl.setGain(TSL2591_GAIN_MED);
    tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);

    Serial.println("🔴 System idle. Press button to start.");

    xTaskCreatePinnedToCore(stepperTask, "Stepper Task", 10000, NULL, 1, &stepperTaskHandle, 0);
    xTaskCreatePinnedToCore(sensorTask, "Sensor Task", 10000, NULL, 1, &sensorTaskHandle, 1);
  }

  void loop() {
    handleButton();

    if (!systemStarted) {
      taskYIELD();
      return;
    }

    // Blink status LED
    if (ledBlinking && millis() - lastBlinkTime >= blinkInterval) {
      lastBlinkTime = millis();
      digitalWrite(MOTOR_LED, !digitalRead(MOTOR_LED));
    }

    taskYIELD();
  }

  void handleButton() {
    static unsigned long lastDebounceTime = 0;
    const unsigned long debounceDelay = 200;
    bool pressed = digitalRead(BUTTON_PIN) == LOW;

    if (pressed && !buttonPreviouslyPressed &&
        millis() - lastDebounceTime > debounceDelay) {
      lastDebounceTime = millis();
      systemStarted = !systemStarted;

      if (systemStarted) {
        Serial.println("🟢 System Started.");
        fireFlag = false;              // ⬅️ Reset fire flag here
        ema_initialized = false;       // ⬅️ Optional: reset EMA init
        motorRunning = true;
        Serial.println("↩️ Returning to reset position...");
        resetToStartPosition();
        motorPermanentlyStopped = false;
        ledBlinking = false;
        noTone(BUZZER_PIN);
        digitalWrite(MOTOR_LED, LOW);
      } else {
        Serial.println("🔁 System Reset.");
        motorRunning = false;
        motorPermanentlyStopped = false;
        ledBlinking = false;
        digitalWrite(MOTOR_LED, LOW);
        noTone(BUZZER_PIN);
        // digitalWrite(EN_PIN, HIGH);
      }
    }

    buttonPreviouslyPressed = pressed;
  }

  void stepperTask(void* parameter) {
    bool moveRightNext = true;
    bool currentlySweeping = false;

    while (true) {
      if (motorRunning && !motorPermanentlyStopped) {
        if (!currentlySweeping && stepCount == 0) {
          digitalWrite(DIR_PIN, moveRightNext ? HIGH : LOW);
          Serial.printf("➡️ Sweeping %s from center...\n", moveRightNext ? "right" : "left");

          for (int i = 0; i < stepsPer90; i++) {
            if (!motorRunning || motorPermanentlyStopped) break;
            digitalWrite(STEP_PIN, HIGH);
            delayMicroseconds(stepDelayMicros);
            digitalWrite(STEP_PIN, LOW);
            delayMicroseconds(stepDelayMicros);
            //delayMicroseconds(100);

            stepCount += (moveRightNext ? 1 : -1);
          }

          currentlySweeping = true;
        }

        else if (currentlySweeping && stepCount != 0) {
          digitalWrite(DIR_PIN, (stepCount > 0) ? LOW : HIGH);
          Serial.printf("🔙 Returning to center from %ld steps...\n", stepCount);

          long stepsToCenter = abs(stepCount);
          for (long i = 0; i < stepsToCenter; i++) {
            if (!motorRunning || motorPermanentlyStopped) break;
            digitalWrite(STEP_PIN, HIGH);
            delayMicroseconds(stepDelayMicros);
            digitalWrite(STEP_PIN, LOW);
            delayMicroseconds(stepDelayMicros);
            //delayMicroseconds(100);

            stepCount += (stepCount > 0) ? -1 : 1;
          }

          Serial.println("✅ Back at center.");
          moveRightNext = !moveRightNext;
          currentlySweeping = false;
        }

        vTaskDelay(pdMS_TO_TICKS(50)); // small pause between sweep cycles

      } else {
        vTaskDelay(10);
      }
    }
  }


  void sensorTask(void* parameter) {
    while (true) {
      if (systemStarted && !fireFlag) {  // ⬅️ Only read if fire hasn't been flagged
        float lux = readLux();
        uint16_t lux_fixed = (uint16_t)(lux * 10.0);

        if (!ema_initialized) {
          ema = lux_fixed;
          ema_initialized = true;
        } else {
          ema = ((EMA_ALPHA * lux_fixed) + (64 - EMA_ALPHA) * ema) >> 6;
        }

        uint16_t threshold = ema + THRESHOLD_OFFSET;
        bool fire_detected = lux_fixed > threshold;

        if (fire_detected) {
          fireFlag = true;  // ⬅️ Set the flag
          Serial.printf("🔥 FIRE DETECTED! Lux: %.1f (Fixed: %u, Threshold: %u, EMA: %u)\n",
                        lux, lux_fixed, threshold, ema);
          digitalWrite(MOTOR_LED, HIGH);
          // digitalWrite(EN_PIN, HIGH);
          tone(BUZZER_PIN, 3000);
          motorRunning = false;
          motorPermanentlyStopped = true;
        } else {
          Serial.printf("Lux: %.1f (Fixed: %u, Threshold: %u, EMA: %u)\n",
                        lux, lux_fixed, threshold, ema);
        }
      }

      vTaskDelay(pdMS_TO_TICKS(sensorInterval));
    }
  }


  float readLux() {
    // delay(50);  // Let sensor settle
    uint32_t lum = tsl.getFullLuminosity();
    uint16_t ir = lum >> 16;
    uint16_t full = lum & 0xFFFF;
    return (float)(full - ir);
  }

  void resetToStartPosition() {
    if (stepCount == 0) {
      Serial.println("✅ Already at center.");
      return;
    }

    Serial.printf("↩️ Resetting from %ld steps...\n", stepCount);
    digitalWrite(DIR_PIN, stepCount > 0 ? LOW : HIGH);
    long totalSteps = abs(stepCount);

    for (long i = 0; i < totalSteps; i++) {
      digitalWrite(STEP_PIN, HIGH);
      delayMicroseconds(stepDelayMicros);
      digitalWrite(STEP_PIN, LOW);
      delayMicroseconds(stepDelayMicros);
      //delayMicroseconds(100);

      stepCount += (stepCount > 0) ? -1 : 1;
    }

    Serial.println("✅ Reset complete. At center.");
  } 