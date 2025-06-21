#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2591.h>
#include <SPI.h>
#include "arduinoFFT.h"

// --- Pins ---
#define STEP_PIN     18
#define DIR_PIN      19
#define BUTTON_PIN   26
#define BUZZER_PIN   33
#define MOTOR_LED    4

#define PIN_SCLK     14
#define PIN_CS       27  // Changed to match FFT code
#define PIN_MISO     16
#define PIN_MOSI     13  // Not used for ADS8319
#define FFT_FLAG_PIN     5    // ESP32 ➜ FPGA
#define FINAL_ALERT_PIN  25   // FPGA ➜ ESP32
#define PI_TRIGGER_PIN 32

// --- Constants ---
const unsigned long sensorInterval = 500;
const int stepDelayMicros = 600;
const int stepsPer90 = 1600;
const uint16_t samples = 1024;
const double samplingFrequency = 100.0;
const float VREF = 3.3;
const int ADC_BITS = 16;
const double alpha = 0.2;
const double thresholdMargin = 1000.0;

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

// --- TSL2591 ---
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);
const uint8_t EMA_ALPHA = 20;
const uint16_t THRESHOLD_OFFSET = 200;

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
  Serial.begin(115200);
  // Temporary with button gone
  // systemStarted = true;
  pinMode(MOTOR_LED, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(PIN_CS, OUTPUT);
  pinMode(PIN_SCLK, OUTPUT);
  pinMode(PIN_MISO, INPUT);
  pinMode(PI_TRIGGER_PIN, OUTPUT); 

  digitalWrite(PI_TRIGGER_PIN, LOW);
  digitalWrite(PIN_CS, HIGH);
  digitalWrite(PIN_SCLK, LOW);

  // === FPGA Communication Pins ===
  pinMode(FFT_FLAG_PIN, OUTPUT);
  pinMode(FINAL_ALERT_PIN, INPUT);

  digitalWrite(FFT_FLAG_PIN, LOW);

  SPI.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, PIN_CS);

  tsl.setGain(TSL2591_GAIN_MED);
  tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);

  freqResolution = samplingFrequency / samples;
  startBin = ceil(5.0 / freqResolution);
  endBin = floor(10.0 / freqResolution);

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
  if (ledBlinking && millis() - lastBlinkTime >= 250) {
    lastBlinkTime = millis();
    digitalWrite(MOTOR_LED, !digitalRead(MOTOR_LED));
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
      fireFlag = false;
      tslTriggered = false;
      ema_initialized = false;
      emaFFTInitialized = false;
      motorRunning = true;
      motorPermanentlyStopped = false;
      ledBlinking = false;
      digitalWrite(MOTOR_LED, LOW);
      noTone(BUZZER_PIN);
      resetToStartPosition();
    } else {
      Serial.println("🔁 System Reset.");
      motorRunning = false;
      motorPermanentlyStopped = false;
      ledBlinking = false;
      digitalWrite(MOTOR_LED, LOW);
      noTone(BUZZER_PIN);
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
        for (int i = 0; i < stepsPer90 && motorRunning && !motorPermanentlyStopped; i++) {
          digitalWrite(STEP_PIN, HIGH); delayMicroseconds(stepDelayMicros);
          digitalWrite(STEP_PIN, LOW);  delayMicroseconds(stepDelayMicros);
          stepCount += (moveRightNext ? 1 : -1);
        }
        currentlySweeping = true;
      } else if (currentlySweeping && stepCount != 0) {
        digitalWrite(DIR_PIN, (stepCount > 0) ? LOW : HIGH);
        long stepsToCenter = abs(stepCount);
        for (long i = 0; i < stepsToCenter && motorRunning && !motorPermanentlyStopped; i++) {
          digitalWrite(STEP_PIN, HIGH); delayMicroseconds(stepDelayMicros);
          digitalWrite(STEP_PIN, LOW);  delayMicroseconds(stepDelayMicros);
          stepCount += (stepCount > 0) ? -1 : 1;
        }
        moveRightNext = !moveRightNext;
        currentlySweeping = false;
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

      // Tell Pi to start camera detection
      digitalWrite(PI_TRIGGER_PIN, HIGH);
      delay(100);  // Allow Pi to detect the rising edge

      bool flameConfirmed = runFftDetection();

      digitalWrite(PI_TRIGGER_PIN, LOW);
hostname 
      if (flameConfirmed) {
        fireFlag = true;
        Serial.println("✅ FIRE CONFIRMED by full pipeline (TSL ➜ Pi ➜ FFT ➜ FPGA).");
        digitalWrite(MOTOR_LED, HIGH);
        tone(BUZZER_PIN, 3000);
        motorRunning = false;
        motorPermanentlyStopped = true;
      } else {
        Serial.println("❌ Detection complete. No fire confirmed after full pipeline (TSL ➜ Pi ➜ FFT ➜ FPGA).");
      }
    } else {
      Serial.printf("Lux: %.1f (Fixed: %u, Threshold: %u, EMA: %u)\n", lux, lux_fixed, threshold, ema);
    }

    vTaskDelay(pdMS_TO_TICKS(sensorInterval));
  }
}

float readLux() {
  uint32_t lum = tsl.getFullLuminosity();
  uint16_t ir = lum >> 16;
  uint16_t full = lum & 0xFFFF;
  return (float)(full - ir);
}

uint16_t readADS8319() {
  uint16_t result = 0;
  digitalWrite(PIN_CS, LOW);
  delayMicroseconds(1);
  for (int i = 0; i < 16; i++) {
    digitalWrite(PIN_SCLK, HIGH);
    delayMicroseconds(1);
    result <<= 1;
    if (digitalRead(PIN_MISO)) result |= 0x01;
    digitalWrite(PIN_SCLK, LOW);
    delayMicroseconds(1);
  }
  digitalWrite(PIN_CS, HIGH);
  return result;
}

bool runFftDetection() {
  const double ADC_OFFSET = 32768.0;
  double currentBandSum = 0.0;
  int activeBins = 0;
  double energySum = 0.0;

  for (uint16_t i = 0; i < samples; i++) {
    uint16_t adcValue = readADS8319();
    vReal[i] = adcValue - ADC_OFFSET;
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

  bool flameDetectedNow = (activeBins >= 5) && (energySum > (2 * adaptiveThreshold));
  if (flameDetectedNow) {
    digitalWrite(FFT_FLAG_PIN, HIGH);  // Tell FPGA
    delay(100);                         // Allow FPGA to latch
    bool finalFire = digitalRead(FINAL_ALERT_PIN);  // Get decision
    digitalWrite(FFT_FLAG_PIN, LOW);   // Clear after
    return finalFire;
  }
  return false;
}

void resetToStartPosition() {
  if (stepCount == 0) return;
  digitalWrite(DIR_PIN, stepCount > 0 ? LOW : HIGH);
  long totalSteps = abs(stepCount);
  for (long i = 0; i < totalSteps; i++) {
    digitalWrite(STEP_PIN, HIGH); delayMicroseconds(stepDelayMicros);
    digitalWrite(STEP_PIN, LOW);  delayMicroseconds(stepDelayMicros);
    stepCount += (stepCount > 0) ? -1 : 1;
  }
}
