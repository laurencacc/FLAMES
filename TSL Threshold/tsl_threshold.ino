#include <Wire.h>

#include <Adafruit_Sensor.h>

#include "Adafruit_TSL2591.h"

#include <math.h>

// Pin definitions
#define SLED 4 // Sensor LED
#define MOTOR_LED 2 // ESP32 built-in LED (GPIO 2)

// Sensor setup
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);

// EMA constants
const float EMA_ALPHA = 0.6;
float emaLux = 0;

// Adaptive thresholding
float adaptiveThreshold = 0;
const float THRESHOLD_OFFSET = 3000;

// Z-score detection
const int windowSize = 10;
float luxWindow[windowSize];
int luxIndex = 0;
int samplesCollected = 0; // Count of valid samples in the window
float zThreshold = 1.5;

// Warm-up timer
bool baselineReady = false;
unsigned long baselineStartTime;
const unsigned long warmupDuration = 5000; // 5 seconds

void setup() {
  Serial.begin(9600);
  pinMode(SLED, OUTPUT);
  pinMode(MOTOR_LED, OUTPUT);
  digitalWrite(MOTOR_LED, LOW);

  tsl.setGain(TSL2591_GAIN_MED);
  tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);

  baselineStartTime = millis();
  Serial.println("TSL threshold detection initializing...");
}

void loop() {
  // Wait for EMA and first luxWindow buffer to fill
  if (!baselineReady) {
    if (millis() - baselineStartTime >= warmupDuration) {
      baselineReady = true;
      Serial.println("✅ Baseline initialized, collecting initial lux data...");
    } else {
      Serial.println("⌛ Warming up...");
      delay(500);
      return;
    }
  }

  float currentLux = readLux();
  if (isnan(currentLux) || isinf(currentLux)) {
    currentLux = 0;
  }

  // Apply EMA
  emaLux = EMA_ALPHA * currentLux + (1 - EMA_ALPHA) * emaLux;

  // Adaptive thresholding
  adaptiveThreshold = emaLux + THRESHOLD_OFFSET;

  // Update sliding window for Z-score
  luxWindow[luxIndex] = currentLux;
  luxIndex = (luxIndex + 1) % windowSize;

  // Fill the window before using z-score detection
  if (samplesCollected < windowSize) {
    samplesCollected++;
    Serial.println("📊 Filling lux window...");
    delay(500);
    return;
  }

  float mean = calculateMean(luxWindow);
  float stdDev = calculateStdDev(luxWindow, mean);

  // Handle bad stdDev or no variance case
  float zScore = 0;
  if (stdDev > 0 && !isnan(stdDev) && !isinf(stdDev)) {
    zScore = (currentLux - mean) / stdDev;
  } else {
    zScore = 0;
  }

  Serial.print("Lux:");
  Serial.print(currentLux);
  Serial.print(",EMA:");
  Serial.print(emaLux);
  Serial.print(",Threshold:");
  Serial.print(adaptiveThreshold);
  Serial.print(",Z:");
  Serial.println(zScore);

  // Detection logic
  if (currentLux > adaptiveThreshold && abs(zScore) > zThreshold) {
    Serial.println("⚠️ Motor Stop");
    digitalWrite(MOTOR_LED, HIGH);
  } else {
    digitalWrite(MOTOR_LED, LOW);
  }

  delay(500);
}

float readLux() {
  digitalWrite(SLED, HIGH);
  delay(50);
  uint32_t lum = tsl.getFullLuminosity();
  uint16_t ir = lum >> 16;
  uint16_t full = lum & 0xFFFF;
  digitalWrite(SLED, LOW);
  float visible = full - ir;
  return (float) visible;
}

float calculateMean(float data[]) {
  float sum = 0;
  for (int i = 0; i < windowSize; i++) {
    sum += data[i];
  }
  return sum / windowSize;
}

float calculateStdDev(float data[], float mean) {
  float sumSq = 0;
  for (int i = 0; i < windowSize; i++) {
    sumSq += pow(data[i] - mean, 2);
  }
  return sqrt(sumSq / windowSize);
}