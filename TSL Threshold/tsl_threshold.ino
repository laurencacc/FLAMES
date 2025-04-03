#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_TSL2591.h"
#include <math.h>

// Pin definitions
#define SLED 4         // Your sensor LED
#define MOTOR_LED 2    // ESP32 built-in LED (GPIO 2)

// Sensor setup
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);

// EMA constants
const float EMA_ALPHA = 0.6;
float emaLux = 0;

// Adaptive thresholding
float adaptiveThreshold = 0;
const float THRESHOLD_OFFSET = 30;

// Z-score detection
const int windowSize = 10;
float luxWindow[windowSize];
int luxIndex = 0;
float zThreshold = 2.0;

void setup() {
  Serial.begin(9600);
  pinMode(SLED, OUTPUT);
  pinMode(MOTOR_LED, OUTPUT);
  digitalWrite(MOTOR_LED, LOW); // ensure LED is off initially

  tsl.setGain(TSL2591_GAIN_LOW);
  tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);

  Serial.println("TSL threshold detection initializing...");
}

void loop() {
  float currentLux = readLux();

  // Apply EMA
  emaLux = EMA_ALPHA * currentLux + (1 - EMA_ALPHA) * emaLux;

  // Adaptive thresholding
  adaptiveThreshold = emaLux + THRESHOLD_OFFSET;

  // Update sliding window for Z-score
  luxWindow[luxIndex] = currentLux;
  luxIndex = (luxIndex + 1) % windowSize;

  float mean = calculateMean(luxWindow);
  float stdDev = calculateStdDev(luxWindow, mean);
  float zScore = (currentLux - mean) / stdDev;

  Serial.print("Current Lux: ");
  Serial.print(currentLux);
  Serial.print(" | EMA: ");
  Serial.print(emaLux);
  Serial.print(" | Threshold: ");
  Serial.print(adaptiveThreshold);
  Serial.print(" | Z-Score: ");
  Serial.println(zScore);

  // Detection
  if (currentLux > adaptiveThreshold && abs(zScore) > zThreshold) {
    Serial.println("⚠️ Motor Stop");
    digitalWrite(MOTOR_LED, HIGH); // turn on ESP32 LED
  } else {
    digitalWrite(MOTOR_LED, LOW);  // turn off ESP32 LED
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
  return tsl.calculateLux(full, ir);
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
