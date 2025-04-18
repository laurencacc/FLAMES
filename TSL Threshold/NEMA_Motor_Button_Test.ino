#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_TSL2591.h"
#include <math.h>

// --- Pins ---
#define STEP_PIN 18
#define DIR_PIN 19
#define BUTTON_PIN 15
#define MOTOR_LED 2
#define SLED 4
#define EN 5

// --- Stepper Motor ---
const int stepDelayMicros = 800;  // Adjust for speed
volatile bool motorRunning = false;
volatile bool motorPermanentlyStopped = false;

// --- Sensor ---
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);
const float EMA_ALPHA = 0.6;
float emaLux = 0;
float adaptiveThreshold = 0;
const float THRESHOLD_OFFSET = 3000;

const int windowSize = 10;
float luxWindow[windowSize];
int luxIndex = 0;
int samplesCollected = 0;
float zThreshold = 1.5;

bool systemStarted = false;
bool buttonPreviouslyPressed = false;
bool baselineReady = false;

unsigned long baselineStartTime;
const unsigned long warmupDuration = 5000;
unsigned long lastSensorCheck = 0;
const unsigned long sensorInterval = 500;

// --- Task Handle ---
TaskHandle_t stepperTaskHandle;

void setup() {
  Serial.begin(9600);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(MOTOR_LED, OUTPUT);
  pinMode(SLED, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);  // Initial direction

  tsl.setGain(TSL2591_GAIN_LOW);
  tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);

  baselineStartTime = millis();
  Serial.println("🚀 System Starting");

  xTaskCreatePinnedToCore(
    stepperTask,
    "Stepper Task",
    10000,
    NULL,
    1,
    &stepperTaskHandle,
    0
  );
}

void loop() {
  handleButton();

  if (!systemStarted) return;

  unsigned long nowMillis = millis();

  if (!baselineReady) {
    if (nowMillis - baselineStartTime >= warmupDuration) {
      baselineReady = true;
      Serial.println("✅ Baseline ready.");
    }
    return;
  }

  if (nowMillis - lastSensorCheck >= sensorInterval) {
    lastSensorCheck = nowMillis;

    float currentLux = readLux();
    if (isnan(currentLux) || isinf(currentLux)) currentLux = 0;

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

void handleButton() {
  static unsigned long lastDebounceTime = 0;
  const unsigned long debounceDelay = 200;

  bool buttonState = digitalRead(BUTTON_PIN) == LOW;

  if (buttonState && !buttonPreviouslyPressed && (millis() - lastDebounceTime > debounceDelay)) {
    lastDebounceTime = millis();

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

  buttonPreviouslyPressed = buttonState;
}

// 🔁 Motor task running on Core 0
void stepperTask(void *parameter) {
  while (true) {
    if (motorRunning && !motorPermanentlyStopped) {
      digitalWrite(EN, LOW);
      digitalWrite(DIR_PIN, HIGH); // Set direction; LOW for reverse
      digitalWrite(STEP_PIN, HIGH);
      delayMicroseconds(stepDelayMicros);
      digitalWrite(STEP_PIN, LOW);
      delayMicroseconds(stepDelayMicros);
    } else {
      digitalWrite(EN, HIGH);
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

void stopStepper() {
  digitalWrite(STEP_PIN, LOW);
}

// 🔦 Lux sensor
float readLux() {
  digitalWrite(SLED, HIGH);
  delay(10);
  uint32_t lum = tsl.getFullLuminosity();
  uint16_t ir = lum >> 16;
  uint16_t full = lum & 0xFFFF;
  digitalWrite(SLED, LOW);
  return (float)visible; 
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
