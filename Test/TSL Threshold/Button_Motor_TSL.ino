#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_TSL2591.h"
#include <math.h>

// --- Pins ---
#define SLED 4
#define MOTOR_LED 2
#define IN1 32
#define IN2 33
#define IN3 25
#define IN4 26
#define BUTTON_PIN 15    

bool systemStarted = false;
bool buttonPreviouslyPressed = false;

void stopStepper(); // function prototype


// --- Stepper Motor ---
const int steps[8][4] = {
  {1, 0, 0, 0},
  {1, 1, 0, 0},
  {0, 1, 0, 0},
  {0, 1, 1, 0},
  {0, 0, 1, 0},
  {0, 0, 1, 1},
  {0, 0, 0, 1},
  {1, 0, 0, 1}
};
int stepIndex = 0;
const int stepDelayMicros = 900;
volatile bool motorRunning = false;
volatile bool motorPermanentlyStopped = false;

// --- Sensor ---
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);
const float EMA_ALPHA = 0.6;
float emaLux = 0;
float adaptiveThreshold = 0;
const float THRESHOLD_OFFSET = 30;

const int windowSize = 10;
float luxWindow[windowSize];
int luxIndex = 0;
int samplesCollected = 0;
float zThreshold = 2.0;

bool baselineReady = false;
unsigned long baselineStartTime;
const unsigned long warmupDuration = 5000;
unsigned long lastSensorCheck = 0;
const unsigned long sensorInterval = 500;

// --- Task Handle ---
TaskHandle_t stepperTaskHandle;

void setup() {
  Serial.begin(9600);
  
  pinMode(BUTTON_PIN, INPUT_PULLUP); // HIGH by default, LOW when pressed

  pinMode(SLED, OUTPUT);
  pinMode(MOTOR_LED, OUTPUT);
  digitalWrite(MOTOR_LED, LOW);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  tsl.setGain(TSL2591_GAIN_LOW);
  tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);

  baselineStartTime = millis();
  Serial.println("🚀 System Starting");

  // Launch stepper motor task on Core 0
  xTaskCreatePinnedToCore(
    stepperTask,       // Function to run
    "Stepper Task",    // Name
    10000,             // Stack size
    NULL,              // Parameters
    1,                 // Priority
    &stepperTaskHandle,// Task handle
    0                  // Core 0
  );
}

void handleButton() {
  static unsigned long lastDebounceTime = 0;
  const unsigned long debounceDelay = 200;

  bool buttonState = digitalRead(BUTTON_PIN) == LOW;

  if (buttonState && !buttonPreviouslyPressed && (millis() - lastDebounceTime > debounceDelay)) {
    lastDebounceTime = millis();

    // TOGGLE systemStarted state
    systemStarted = !systemStarted;

    if (systemStarted) {
      Serial.println("🟢 System Started.");
      motorRunning = true;
      motorPermanentlyStopped = false;
      baselineReady = false;
      samplesCollected = 0;
      luxIndex = 0;
      baselineStartTime = millis();
      digitalWrite(MOTOR_LED, LOW);
    } else {
      Serial.println("🔁 System Reset.");
      motorRunning = false;
      motorPermanentlyStopped = false;
      baselineReady = false;
      samplesCollected = 0;
      luxIndex = 0;
      digitalWrite(MOTOR_LED, LOW);
      stopStepper();
    }
  }

  // Update previous state
  buttonPreviouslyPressed = buttonState;
}



void loop() {

  handleButton();

  if (!systemStarted) {
    return; // Wait until button is pressed
  }

  unsigned long nowMillis = millis();

  // Wait for warm-up
  if (!baselineReady) {
    if (nowMillis - baselineStartTime >= warmupDuration) {
      baselineReady = true;
      Serial.println("✅ Baseline ready.");
    }
    return;
  }

  // Read sensor every 500ms
  if (nowMillis - lastSensorCheck >= sensorInterval) {
    lastSensorCheck = nowMillis;

    float currentLux = readLux();
    if (isnan(currentLux) || isinf(currentLux)) {
      currentLux = 0;
    }

    emaLux = EMA_ALPHA * currentLux + (1 - EMA_ALPHA) * emaLux;
    adaptiveThreshold = emaLux + THRESHOLD_OFFSET;

    luxWindow[luxIndex] = currentLux;
    luxIndex = (luxIndex + 1) % windowSize;

    if (samplesCollected < windowSize) {
      samplesCollected++;
      Serial.println("📊 Collecting lux window...");
      return;
    }

    float mean = calculateMean(luxWindow);
    float stdDev = calculateStdDev(luxWindow, mean);
    float zScore = 0;
    if (stdDev > 0 && !isnan(stdDev) && !isinf(stdDev)) {
      zScore = (currentLux - mean) / stdDev;
    } else {
      zScore = 0;
    }
    

    Serial.print("Lux: "); Serial.print(currentLux);
    Serial.print(" | EMA: "); Serial.print(emaLux);
    Serial.print(" | Threshold: "); Serial.print(adaptiveThreshold);
    Serial.print(" | Z-Score: "); Serial.println(zScore);

    if (!motorPermanentlyStopped && currentLux > adaptiveThreshold && abs(zScore) > zThreshold) {
      Serial.println("🔥 Fire detected! Motor permanently stopped.");
      digitalWrite(MOTOR_LED, HIGH);
      motorRunning = false;
      motorPermanentlyStopped = true;
    }
  }
}

// 🔁 Motor task running on Core 0
void stepperTask(void *parameter) {
  while (true) {
    if (motorRunning && !motorPermanentlyStopped) {
      digitalWrite(IN1, steps[stepIndex][0]);
      digitalWrite(IN2, steps[stepIndex][1]);
      digitalWrite(IN3, steps[stepIndex][2]);
      digitalWrite(IN4, steps[stepIndex][3]);
      stepIndex = (stepIndex + 1) % 8;
      delayMicroseconds(stepDelayMicros);  // actual step timing
    } else {
      // Stop motor
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);

      // Give some delay to prevent watchdog timeout
      vTaskDelay(10 / portTICK_PERIOD_MS);  // 10ms delay
    }

    // 👇 Prevent watchdog timeout even during running
    vTaskDelay(1 / portTICK_PERIOD_MS);  // Yield to watchdog (1ms)
  }
}

void stopStepper() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}


// 🔦 Lux sensor
float readLux() {
  digitalWrite(SLED, HIGH);
  delay(10);
  uint32_t lum = tsl.getFullLuminosity();
  uint16_t ir = lum >> 16;
  uint16_t full = lum & 0xFFFF;
  digitalWrite(SLED, LOW);
  return tsl.calculateLux(full, ir);
}

// 📊 Stats
float calculateMean(float data[]) {
  float sum = 0;
  for (int i = 0; i < windowSize; i++) sum += data[i];
  return sum / windowSize;
}

float calculateStdDev(float data[], float mean) {
  float sumSq = 0;
  for (int i = 0; i < windowSize; i++) {
    sumSq += pow(data[i] - mean, 2);
  }
  return sqrt(sumSq / windowSize);
}
