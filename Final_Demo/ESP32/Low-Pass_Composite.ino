#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2591.h>
#include <SPI.h>
#include "arduinoFFT.h"
#define USE_INTERNAL_ADC true  // Set to false to use ADS8319
volatile bool resetting = false;


#define ADC_pin 27  // Only used if USE_INTERNAL_ADC = true
const double samplingFrequency = 100.0;

// --- Pins ---
#define STEP_PIN     18
#define DIR_PIN      19
#define BUTTON_PIN   26
#define BUZZER_PIN   33
#define MOTOR_LED    4

#define FFT_FLAG_PIN     17    // ESP32 ➜ FPGA // Titled CS on MCU Board for FPGA
#define FINAL_ALERT_PIN  25   // FPGA ➜ ESP32 // Titled Alert Pin On MCU board for FPGA
#define PI_TRIGGER_PIN 32 // Titled CS for Pi on MCU Board (second from top pin)
#define FPGA_RESET_PIN 13  // Choose any available GPIO // Titled MOSI
// Sleep pin ?

// --- Filter Coefficients ---
const double w0 = 2 * M_PI * 10.0;  // 10 Hz cutoff frequency
const double t = 1.0 / samplingFrequency;
const double a = (w0 * t) / (2.0 + w0 * t);
const double b = (2.0 - w0 * t) / (2.0 + w0 * t);

// --- Persistent Filter State ---
double x_prev = 0.0;
double y_prev = 0.0;

// --- Constants ---
const unsigned long sensorInterval = 500;
const int stepDelayMicros = 600;
const int stepsPer90 = 400;
const uint16_t samples = 1024;
const float VREF = 3.3;
const int ADC_BITS = 12;
const double alpha = 0.2;
const double thresholdMargin = 60.0;

// --- State ---
bool fireFlag = false;
bool tslTriggered = false;
bool systemStarted = false;
bool buttonPreviouslyPressed = false;
volatile bool motorRunning = false;
volatile bool motorPermanentlyStopped = false;
bool ledBlinking = false;
unsigned long lastBlinkTime = 0;
long stepCount = 0;
uint16_t ema = 0;
bool ema_initialized = false;
double emaMagnitude = 0.0;
bool emaFFTInitialized = false;

// --- Stepper State ---
struct StepperState {
  bool moveRightNext;
  bool currentlySweeping;
};
StepperState stepperState = { true, false };


// --- TSL2591 ---
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);
const uint8_t EMA_ALPHA = 20;
const uint16_t THRESHOLD_OFFSET = 50;

// --- FFT ---
double vReal[samples];
double vImag[samples];
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, samples, samplingFrequency);
double freqResolution;
int startBin, endBin;

// --- Tasks ---
TaskHandle_t stepperTaskHandle;
TaskHandle_t sensorTaskHandle;

void setup() {
  esp_log_level_set("*", ESP_LOG_VERBOSE);
  Serial.begin(115200);
  // Temporary with button gone
  // systemStarted = true;
  pinMode(MOTOR_LED, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(PI_TRIGGER_PIN, OUTPUT);
  pinMode(FPGA_RESET_PIN, OUTPUT);
  digitalWrite(FPGA_RESET_PIN, HIGH);  // Default state = not reset
 

  digitalWrite(PI_TRIGGER_PIN, LOW);

  // === FPGA Communication Pins ===
  pinMode(FFT_FLAG_PIN, OUTPUT);
  pinMode(FINAL_ALERT_PIN, INPUT);

  digitalWrite(FFT_FLAG_PIN, LOW);

  tsl.setGain(TSL2591_GAIN_MED);
  tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);

  freqResolution = samplingFrequency / samples;
  startBin = ceil(5.0 / freqResolution);
  endBin = floor(10.0 / freqResolution);

  Serial.println("🔴 System idle. Press button to start.");
  xTaskCreatePinnedToCore(stepperTask, "Stepper Task", 20000, NULL, 1, &stepperTaskHandle, 0);
  xTaskCreatePinnedToCore(sensorTask, "Sensor Task", 20000, NULL, 1, &sensorTaskHandle, 1);
}

void loop() {
  handleButton();
  if (!systemStarted) {
    taskYIELD();
    return;
  }
  if (ledBlinking && millis() - lastBlinkTime >= 250) {
    lastBlinkTime = millis();
    digitalWrite(MOTOR_LED, !digitalRead(MOTOR_LED));
  }
  taskYIELD();
}

void handleButton() {
  static unsigned long lastDebounceTime = 0;
  const unsigned long debounceDelay = 500;
  bool pressed = digitalRead(BUTTON_PIN) == LOW;
  if (pressed && !buttonPreviouslyPressed && millis() - lastDebounceTime > debounceDelay) {
    lastDebounceTime = millis();
    systemStarted = !systemStarted;
    digitalWrite(FFT_FLAG_PIN, LOW);
    if (systemStarted) {  // Only start if it was stopped and NOT in fire-locked mode
      Serial.println("🟢 System Started.");
      resetting = true;  // 🚨 PAUSE motor during full reset
      tslTriggered = false;
      ema_initialized = false;
      emaFFTInitialized = false;
      motorRunning = false;  // make sure stepper is paused
      motorPermanentlyStopped = false;
      ledBlinking = false;
      fireFlag = false;
      stepperState.moveRightNext = true;
      stepperState.currentlySweeping = false;
      digitalWrite(MOTOR_LED, LOW);
      noTone(BUZZER_PIN);
      resetToStartPosition();  // 🌀 Center the motor
      x_prev = y_prev = 0.0;
      stepCount = 0;

      // Reset FPGA
      digitalWrite(FPGA_RESET_PIN, LOW);
      delay(50);
      digitalWrite(FPGA_RESET_PIN, HIGH);
      Serial.println("🔄 FPGA Reset");

      resetting = false;  // ✅ Allow stepper to resume
      motorRunning = true;
    } else {
      Serial.println("🔁 System Reset.");
      motorRunning = false;
      motorPermanentlyStopped = false;
      fireFlag = false;
      stepperState.moveRightNext = true;
      stepperState.currentlySweeping = false;
      ledBlinking = false;
      digitalWrite(MOTOR_LED, LOW);
      noTone(BUZZER_PIN);
    }
  }
  buttonPreviouslyPressed = pressed;
}

void stepperTask(void* parameter) {
  while (true) {
    if (resetting) {
      vTaskDelay(10);
      continue;
    }
    if (motorRunning && !motorPermanentlyStopped) {
      if (!stepperState.currentlySweeping && stepCount == 0) {
        digitalWrite(DIR_PIN, stepperState.moveRightNext ? HIGH : LOW);
        for (int i = 0; i < stepsPer90 && motorRunning && !motorPermanentlyStopped && !resetting; i++) {
          digitalWrite(STEP_PIN, HIGH); delayMicroseconds(1000);
          digitalWrite(STEP_PIN, LOW);  delayMicroseconds(7333);
          stepCount += (stepperState.moveRightNext ? 1 : -1);
          if (i % 100 == 0) vTaskDelay(1);
        }
        stepperState.currentlySweeping = true;
      } else if (stepperState.currentlySweeping && stepCount != 0) {
        digitalWrite(DIR_PIN, (stepCount > 0) ? LOW : HIGH);
        long stepsToCenter = abs(stepCount);
        for (long i = 0; i < stepsToCenter && motorRunning && !motorPermanentlyStopped && !resetting; i++) {
          digitalWrite(STEP_PIN, HIGH); delayMicroseconds(1000);
          digitalWrite(STEP_PIN, LOW);  delayMicroseconds(7333);
          stepCount += (stepCount > 0) ? -1 : 1;
          if (i % 100 == 0) vTaskDelay(1);
        }
        stepperState.moveRightNext = !stepperState.moveRightNext;
        stepperState.currentlySweeping = false;
      }
      vTaskDelay(pdMS_TO_TICKS(50));
    } else {
      vTaskDelay(10);
    }
  }
}


void sensorTask(void* parameter) {
  while (true) {
    if (!systemStarted || fireFlag) {
      vTaskDelay(pdMS_TO_TICKS(sensorInterval));
      digitalWrite(FFT_FLAG_PIN, LOW);
      continue;
    }

    float lux = readLux();
    uint16_t lux_fixed = (uint16_t)(lux * 10.0);
    if (!ema_initialized) {
      ema = lux_fixed;
      ema_initialized = true;
    } else {
      ema = ((EMA_ALPHA * lux_fixed) + (64 - EMA_ALPHA) * ema) >> 6;
    }

    uint16_t threshold = ema + THRESHOLD_OFFSET;

    if (!tslTriggered && lux_fixed > threshold) {
      Serial.println("⚠️  TSL Triggered. Starting FFT + Pi check...");
      tslTriggered = true;

      // Pause motor while confirming
      motorRunning = false;

      // Tell Pi to start camera detection
      digitalWrite(PI_TRIGGER_PIN, HIGH);
      delay(100);  // Allow Pi to detect the rising edge

      bool flameConfirmed = runFftDetection();

      digitalWrite(PI_TRIGGER_PIN, LOW);
      if (flameConfirmed) {
        fireFlag = true;
        Serial.println("✅ FIRE CONFIRMED by full pipeline (TSL ➜ Pi ➜ FFT ➜ FPGA).");
        ledBlinking = true;
        tone(BUZZER_PIN, 3000);
        motorPermanentlyStopped = true;
      } else {
        Serial.println("❌ Detection complete. No fire confirmed after full pipeline (TSL ➜ Pi ➜ FFT ➜ FPGA).");
        
        // Reset motor and detection state
        vTaskDelay(100);
        resumeAfterFalseDetection();

      }

      // ✅ Reset for next detection attempt
      tslTriggered = false;
      ema_initialized = false;
      emaFFTInitialized = false;
    }

    vTaskDelay(pdMS_TO_TICKS(sensorInterval));  // ✅ Keep sensor reads paced
  }
}

float readLux() {
  uint32_t lum = tsl.getFullLuminosity();
  uint16_t ir = lum >> 16;
  uint16_t full = lum & 0xFFFF;
  return (float)(full - ir);
}

uint16_t readADC() {
  if (USE_INTERNAL_ADC) {
    return analogRead(ADC_pin);  // 12-bit value (0–4095)
  } 
}

bool runFftDetection() {
  digitalWrite(FFT_FLAG_PIN, LOW);
  const double ADC_OFFSET = 2048.0;
  double currentBandSum = 0.0;
  int activeBins = 0;
  double energySum = 0.0;

  for (uint16_t i = 0; i < samples; i++) {
    uint16_t adcValue = readADC();
    double x_curr = (adcValue - ADC_OFFSET) / 16.0;  // optional normalization
    double y_curr = a * (x_curr + x_prev) + b * y_prev;

    x_prev = x_curr;
    y_prev = y_curr;

    vReal[i] = y_curr;
    vImag[i] = 0.0;
    delayMicroseconds(1000000 / samplingFrequency);
  }

  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();

  double bandAvg = 0.0;
  for (int i = startBin; i <= endBin; i++) {
    bandAvg += vReal[i];
  }
  bandAvg /= (endBin - startBin + 1);

  if (!emaFFTInitialized) {
    emaMagnitude = bandAvg;
    emaFFTInitialized = true;
  } else {
    emaMagnitude = alpha * bandAvg + (1 - alpha) * emaMagnitude;
  }

  emaMagnitude = min(emaMagnitude, 50000.0);
  double adaptiveThreshold = emaMagnitude + thresholdMargin;

  for (int i = startBin; i <= endBin; i++) {
    if (vReal[i] > adaptiveThreshold) {
      activeBins++;
      energySum += vReal[i];
    }
  }

  bool flameDetectedNow = (activeBins >= 12) && (energySum > (3.5 * adaptiveThreshold));

  Serial.println("📊 --- FFT Detection Debug ---");
  Serial.printf("📉 Band Avg: %.1f\n", bandAvg);
  Serial.printf("📈 EMA Magnitude: %.1f\n", emaMagnitude);
  Serial.printf("🔺 Adaptive Threshold: %.1f\n", adaptiveThreshold);
  Serial.printf("📦 Active Bins: %d\n", activeBins);
  Serial.printf("⚡ Energy Sum: %.1f\n", energySum);
  Serial.printf("🔥 FFT Detected Fire: %s\n", flameDetectedNow ? "YES" : "NO");
  Serial.println("-------------------------------");

  if (flameDetectedNow) {
    digitalWrite(FFT_FLAG_PIN, HIGH);  // ✅ HOLD HIGH
    Serial.println("🚩 FFT_FLAG_PIN set HIGH");

    unsigned long startTime = millis();
    bool finalFire = false;

    while (millis() - startTime < 2000) {
      if (digitalRead(FINAL_ALERT_PIN)) {
        finalFire = true;
        break;
      }
      delay(10);
    }

    Serial.printf("🔥 FPGA Response: %s\n", finalFire ? "FIRE" : "NO FIRE");
    digitalWrite(FFT_FLAG_PIN, LOW);  // ✅ Drop only after response
    return finalFire;
  }

  return false;
}


void resetToStartPosition() {
  if (stepCount == 0) return;
  digitalWrite(DIR_PIN, stepCount > 0 ? LOW : HIGH);
  long totalSteps = abs(stepCount);
  for (long i = 0; i < totalSteps; i++) {
    digitalWrite(STEP_PIN, HIGH); delayMicroseconds(1000);
    digitalWrite(STEP_PIN, LOW);  delayMicroseconds(7333);
    stepCount += (stepCount > 0) ? -1 : 1;
    if (i % 100 == 0) vTaskDelay(1);
  }
} 

void resumeAfterFalseDetection() {
  tslTriggered = false;
  ema_initialized = false;
  emaFFTInitialized = false;
  x_prev = y_prev = 0.0;
  resetToStartPosition();
  stepCount = 0;
  motorRunning = true;
  stepperState.moveRightNext = true;
  stepperState.currentlySweeping = false;
  Serial.println("🔄 Resuming sweep after false detection.");
}
