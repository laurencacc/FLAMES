#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2591.h>
#include <SPI.h>

// --- Pins ---
#define STEP_PIN     18
#define DIR_PIN      19
#define EN_PIN       5
#define BUTTON_PIN   15   // Changed to a reliable GPIO with pull-up
#define BUZZER_PIN   33
#define MOTOR_LED    4

#define SPI_SCK      14
#define SPI_MOSI     13
#define SPI_SS       12
#define DETECT_PIN   25  // FPGA detect flag output

// --- Stepper Motor Control ---
const int stepDelayMicros = 900;  // Consistent step delay
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
unsigned long lastSensorCheck = 0;
const unsigned long sensorInterval = 500;

TaskHandle_t stepperTaskHandle;

void setup() {
  Serial.begin(9600);

  // SPI setup
  SPI.begin(SPI_SCK, -1, SPI_MOSI, SPI_SS);
  pinMode(SPI_SS, OUTPUT);
  digitalWrite(SPI_SS, HIGH);
  pinMode(DETECT_PIN, INPUT);

  // Output setup
  pinMode(MOTOR_LED, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  digitalWrite(MOTOR_LED, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(DIR_PIN, HIGH);
  digitalWrite(EN_PIN, HIGH);  // Motor off at start

  // Sensor setup
  tsl.setGain(TSL2591_GAIN_MED);
  tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);

  Serial.println("🔴 System idle. Press button to start.");

  // Start stepper motor task
  xTaskCreatePinnedToCore(stepperTask, "Stepper Task", 10000, NULL, 1, &stepperTaskHandle, 0);
}

void loop() {
  handleButton();

  if (!systemStarted) {
    taskYIELD();
    return;
  }

  // Blink status LED if flagged
  if (ledBlinking && millis() - lastBlinkTime >= blinkInterval) {
    lastBlinkTime = millis();
    digitalWrite(MOTOR_LED, !digitalRead(MOTOR_LED));
  }

  // Sample every 500ms
  if (millis() - lastSensorCheck >= sensorInterval) {
    lastSensorCheck = millis();

    float lux = readLux();
    uint16_t lux_fixed = (uint16_t)(lux * 10.0);  // Send as fixed-point

    // SPI send
    digitalWrite(SPI_SS, LOW);
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    SPI.transfer16(lux_fixed);
    SPI.endTransaction();
    digitalWrite(SPI_SS, HIGH);

    // Read fire detect flag from FPGA
    if (digitalRead(DETECT_PIN) == HIGH) {
      Serial.printf("🔥 FIRE DETECTED BY FPGA! Lux: %.1f\n", lux);
      digitalWrite(MOTOR_LED, HIGH);
      digitalWrite(EN_PIN, HIGH);  // Disable motor
      tone(BUZZER_PIN, 3000);
      motorRunning = false;
      motorPermanentlyStopped = true;
    } else {
      Serial.printf("Lux: %.1f (sent %u)\n", lux, lux_fixed);
    }
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
      motorRunning = true;
      Serial.println("↩️ Returning to reset position...");
      resetToStartPosition();  // Add this function below
      motorPermanentlyStopped = false;
      ledBlinking = false;
      noTone(BUZZER_PIN);
      digitalWrite(EN_PIN, LOW);
      digitalWrite(MOTOR_LED, LOW);
    } else {
      Serial.println("🔁 System Reset.");
      motorRunning = false;
      motorPermanentlyStopped = false;
      ledBlinking = false;
      digitalWrite(MOTOR_LED, LOW);
      noTone(BUZZER_PIN);
      digitalWrite(EN_PIN, HIGH);
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
        // Start a sweep
        digitalWrite(DIR_PIN, moveRightNext ? HIGH : LOW);
        Serial.printf("➡️ Sweeping %s from center...\n", moveRightNext ? "right" : "left");

        for (int i = 0; i < stepsPer90; i++) {
          if (!motorRunning || motorPermanentlyStopped) break;
          digitalWrite(STEP_PIN, HIGH);
          delayMicroseconds(stepDelayMicros);
          digitalWrite(STEP_PIN, LOW);
          delayMicroseconds(stepDelayMicros);
          vTaskDelay(1);

          stepCount += (moveRightNext ? 1 : -1);
        }

        currentlySweeping = true;  // ✅ Set flag so we don’t sweep again until we return
      }

      else if (currentlySweeping && stepCount != 0) {
        // Return to center
        digitalWrite(DIR_PIN, (stepCount > 0) ? LOW : HIGH);
        Serial.printf("🔙 Returning to center from %ld steps...\n", stepCount);

        long stepsToCenter = abs(stepCount);
        for (long i = 0; i < stepsToCenter; i++) {
          if (!motorRunning || motorPermanentlyStopped) break;
          digitalWrite(STEP_PIN, HIGH);
          delayMicroseconds(stepDelayMicros);
          digitalWrite(STEP_PIN, LOW);
          delayMicroseconds(stepDelayMicros);
          vTaskDelay(1);

          stepCount += (stepCount > 0) ? -1 : 1;
        }

        Serial.println("✅ Back at center.");
        moveRightNext = !moveRightNext;     // Flip for next time
        currentlySweeping = false;          // ✅ Allow another sweep
      }

      delay(300);
    } else {
      vTaskDelay(10);
    }
  }
}


float readLux() {
  delay(50);  // Let the sensor settle
  uint32_t lum = tsl.getFullLuminosity();
  uint16_t ir = lum >> 16;
  uint16_t full = lum & 0xFFFF;
  return (float)(full - ir);  // Visible light
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
    vTaskDelay(1);

    stepCount += (stepCount > 0) ? -1 : 1;
  }

  Serial.println("✅ Reset complete. At center.");
}
