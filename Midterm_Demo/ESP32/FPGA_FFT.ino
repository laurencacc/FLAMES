#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>
#include "Adafruit_TSL2591.h"
#include "arduinoFFT.h"

// ----- Pins -----
#define STEP_PIN 18
#define DIR_PIN 19
#define EN_PIN 5
#define BUTTON_PIN 15
#define BUZZER_PIN 4
#define MOTOR_LED 2
#define GPIO_TRIGGER_PI 12           // output to Pi
#define GPIO_FLAG_TO_FPGA 13         // output to FPGA
#define FPGA_FINAL_ALERT_PIN 14      // input from FPGA

// ----- FFT Config -----
const uint16_t samples = 64;
const double samplingFrequency = 20.0;
const int adcPin = 34;
double vReal[samples];
double vImag[samples];
int flameConfirmCount = 0;
bool runFFT = false;

ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, samples, samplingFrequency);

// ----- TSL Config -----
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);
const float EMA_ALPHA = 0.8;
float emaLux = 0;
float adaptiveThreshold = 0;
const float THRESHOLD_OFFSET = 500;
const int windowSize = 10;
float luxWindow[windowSize];
int luxIndex = 0;
int samplesCollected = 0;
float zThreshold = 1.2;
bool baselineReady = false;
unsigned long baselineStartTime;
const unsigned long warmupDuration = 5000;
unsigned long lastSensorCheck = 0;
const unsigned long sensorInterval = 500;

// ----- Stepper -----
volatile bool motorRunning = false;
volatile bool motorPermanentlyStopped = false;
bool systemStarted = false;
bool buttonPreviouslyPressed = false;
bool ledBlinking = false;
unsigned long lastBlinkTime = 0;
const unsigned long blinkInterval = 250;

TaskHandle_t stepperTaskHandle;

void setup() {
  Serial.begin(9600);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(MOTOR_LED, OUTPUT);
  pinMode(GPIO_TRIGGER_PI, OUTPUT);
  pinMode(GPIO_FLAG_TO_FPGA, OUTPUT);
  pinMode(FPGA_FINAL_ALERT_PIN, INPUT);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(MOTOR_LED, LOW);
  digitalWrite(EN_PIN, LOW);
  digitalWrite(DIR_PIN, HIGH);
  digitalWrite(GPIO_TRIGGER_PI, LOW);
  digitalWrite(GPIO_FLAG_TO_FPGA, LOW);

  tsl.setGain(TSL2591_GAIN_MED);
  tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);

  Serial.println("🔴 System idle. Press button to start.");
  xTaskCreatePinnedToCore(stepperTask, "Stepper Task", 10000, NULL, 1, &stepperTaskHandle, 0);
}

void loop() {
  handleButton();

  if (!systemStarted) {
    taskYIELD();
    return;
  }

  if (ledBlinking && millis() - lastBlinkTime >= blinkInterval) {
    lastBlinkTime = millis();
    digitalWrite(MOTOR_LED, !digitalRead(MOTOR_LED));
  }

  if (!baselineReady) {
    if (millis() - baselineStartTime >= warmupDuration) {
      baselineReady = true;
      Serial.println("✅ Baseline ready.");
    }
    taskYIELD();
    return;
  }

  if (millis() - lastSensorCheck >= sensorInterval) {
    lastSensorCheck = millis();
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
    float zScore = (stdDev > 0) ? (currentLux - mean) / stdDev : 0;

    Serial.printf("Lux:%.1f EMA:%.1f Thr:%.1f Z:%.2f\n", currentLux, emaLux, adaptiveThreshold, zScore);

    if (currentLux > adaptiveThreshold && fabs(zScore) > zThreshold) {
      Serial.println("🔥 TSL Spike Detected. Starting FFT + Triggering Pi.");
      runFFT = true;
      digitalWrite(GPIO_TRIGGER_PI, HIGH);  // trigger Pi
    }
  }

  if (runFFT) {
    runPhotodiodeFFT();
    runFFT = false;
  }

  // Final confirmation from FPGA
  if (digitalRead(FPGA_FINAL_ALERT_PIN) == HIGH) {
    Serial.println("🚨 FPGA Final Alert Confirmed!");
    tone(BUZZER_PIN, 3000);
    ledBlinking = true;
    motorRunning = false;
    digitalWrite(EN_PIN, HIGH);
  }

  taskYIELD();
}

void handleButton() {
  static unsigned long lastDebounceTime = 0;
  const unsigned long debounceDelay = 200;
  bool pressed = digitalRead(BUTTON_PIN) == LOW;

  if (pressed && !buttonPreviouslyPressed && millis() - lastDebounceTime > debounceDelay) {
    lastDebounceTime = millis();
    systemStarted = !systemStarted;

    if (systemStarted) {
      Serial.println("🟢 System Started.");
      motorRunning = true;
      motorPermanentlyStopped = false;
      ledBlinking = false;
      noTone(BUZZER_PIN);
      baselineReady = false;
      samplesCollected = 0;
      luxIndex = 0;
      emaLux = 0;
      baselineStartTime = millis();
      digitalWrite(EN_PIN, LOW);
      digitalWrite(MOTOR_LED, LOW);
      digitalWrite(GPIO_TRIGGER_PI, LOW);
      digitalWrite(GPIO_FLAG_TO_FPGA, LOW);
    } else {
      Serial.println("🔁 System Reset.");
      motorRunning = false;
      motorPermanentlyStopped = false;
      ledBlinking = false;
      digitalWrite(MOTOR_LED, LOW);
      noTone(BUZZER_PIN);
      digitalWrite(EN_PIN, HIGH);
      digitalWrite(GPIO_TRIGGER_PI, LOW);
      digitalWrite(GPIO_FLAG_TO_FPGA, LOW);
    }
  }

  buttonPreviouslyPressed = pressed;
}

void runPhotodiodeFFT() {
  for (uint16_t i = 0; i < samples; i++) {
    vReal[i] = analogRead(adcPin);
    vImag[i] = 0.0;
    delayMicroseconds(1000000 / samplingFrequency);
  }

  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();

  double freqResolution = samplingFrequency / samples;
  int startBin = ceil(5.0 / freqResolution);
  int endBin = floor(10.0 / freqResolution);

  const double threshold = 80.0;
  int activeBins = 0;
  double energySum = 0.0;

  for (int i = startBin; i <= endBin; i++) {
    double magnitude = vReal[i];
    if (magnitude > threshold) {
      activeBins++;
      energySum += magnitude;
    }
  }

  bool flameDetectedNow = (activeBins >= 5) && (energySum > 1000.0);
  if (flameDetectedNow) {
    flameConfirmCount++;
  } else {
    flameConfirmCount = 0;
  }

  if (flameConfirmCount >= 2) {
    Serial.println("🔥 FFT confirms fire.");
    digitalWrite(GPIO_FLAG_TO_FPGA, HIGH);  // notify FPGA
  }
}

void stepperTask(void* parameter) {
  while (true) {
    if (motorRunning && !motorPermanentlyStopped) {
      digitalWrite(STEP_PIN, HIGH);
      delayMicroseconds(900);
      digitalWrite(STEP_PIN, LOW);
      delayMicroseconds(900);
    } else {
      delay(10);
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

float readLux() {
  delay(50);
  uint32_t lum = tsl.getFullLuminosity();
  uint16_t ir = lum >> 16;
  uint16_t full = lum & 0xFFFF;
  float visible = full - ir;
  return (float)visible;
}

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
