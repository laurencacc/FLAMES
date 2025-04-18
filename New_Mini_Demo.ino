#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_TSL2591.h"
#include <math.h>

// --- Pins ---
#define STEP_PIN 18
#define DIR_PIN 19
#define EN_PIN 5
#define BUTTON_PIN 15
#define BUZZER_PIN 4
#define MOTOR_LED 2

// --- Stepper Motor ---
const int stepDelayMicros = 900;
volatile bool motorRunning = false;
volatile bool motorPermanentlyStopped = false;
bool systemStarted = false;
bool buttonPreviouslyPressed = false;

bool ledBlinking = false;
unsigned long lastBlinkTime = 0;
const unsigned long blinkInterval = 250;  // 250ms = fast blink


// --- Sensor ---
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

// --- Task Handle ---
TaskHandle_t stepperTaskHandle;

void setup() {
  Serial.begin(9600);

  // pinMode(SLED, OUTPUT); 
  pinMode(MOTOR_LED, OUTPUT);
  digitalWrite(MOTOR_LED, LOW);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW); // start silent
  noTone(BUZZER_PIN);


  digitalWrite(DIR_PIN, HIGH); // Initial motor direction
  digitalWrite(EN_PIN, LOW);  

  tsl.setGain(TSL2591_GAIN_HIGH);
  tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);

  Serial.println("🔴 System idle. Press button to start.");

  // Start stepper motor task on Core 0
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

void handleButton() {
  static unsigned long lastDebounceTime = 0;
  const unsigned long debounceDelay = 200;
  bool buttonState = digitalRead(BUTTON_PIN) == LOW;

  if (buttonState && !buttonPreviouslyPressed && (millis() - lastDebounceTime > debounceDelay)) {
    lastDebounceTime = millis();
    systemStarted = !systemStarted;

    if (systemStarted) {
      //Serial.print("System Started: "); Serial.println(systemStarted);
      //Serial.print("Motor Running: "); Serial.println(motorRunning);
      //Serial.print("Permanently Stopped: "); Serial.println(motorPermanentlyStopped);
      Serial.println("🟢 System Started.");
      motorPermanentlyStopped = false;
      baselineReady = false;
      samplesCollected = 0;
      luxIndex = 0;
      emaLux = 0;  // Reset EMA to start fresh
      baselineStartTime = millis();
      motorRunning = true;   // ✅ Start motor immediately    
      //digitalWrite(EN_PIN, LOW);        // ✅ Enable driver
      digitalWrite(MOTOR_LED, LOW);
      digitalWrite(BUZZER_PIN, LOW);
      noTone(BUZZER_PIN);  
    } else {
      //Serial.print("System Started: "); Serial.println(systemStarted);
      //Serial.print("Motor Running: "); Serial.println(motorRunning);
      //Serial.print("Permanently Stopped: "); Serial.println(motorPermanentlyStopped);
      Serial.println("🔁 System Reset.");
      motorPermanentlyStopped = false;
      baselineReady = false;
      samplesCollected = 0;
      luxIndex = 0;
      //digitalWrite(EN_PIN, HIGH); // Disable motor
      ledBlinking = false;
      digitalWrite(MOTOR_LED, LOW);
      digitalWrite(BUZZER_PIN, LOW);
    }
  }

  buttonPreviouslyPressed = buttonState;
}

void loop() {
  handleButton();

  if (ledBlinking) {
  if (millis() - lastBlinkTime >= blinkInterval) {
    lastBlinkTime = millis();
    digitalWrite(MOTOR_LED, !digitalRead(MOTOR_LED)); // Toggle LED
  }
}


  if (!systemStarted) {
    taskYIELD();
    return;
  }

  unsigned long nowMillis = millis();

  // Warm-up wait
if (!baselineReady) {
  if (nowMillis - baselineStartTime >= warmupDuration) {
    baselineReady = true;
    // motorRunning = true;  // ✅ Start the motor after warmup
    motorPermanentlyStopped = false; // ✅ Redundant safety reset
    Serial.println("✅ Baseline ready. Motor running.");
  }
  return;
}



  // Lux read every 500ms
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
    float zScore = (stdDev > 0) ? (currentLux - mean) / stdDev : 0;

    Serial.print("Lux: "); Serial.print(currentLux);
    Serial.print(" | EMA: "); Serial.print(emaLux);
    Serial.print(" | Threshold: "); Serial.print(adaptiveThreshold);
    Serial.print(" | Z-Score: "); Serial.println(zScore);

    if (!motorPermanentlyStopped && currentLux > adaptiveThreshold && abs(zScore) > zThreshold) {
      Serial.println("🔥 Fire detected! Motor stopped.");
      motorRunning = false;
      //motorPermanentlyStopped = true;
      ledBlinking = true;
      digitalWrite(BUZZER_PIN, HIGH);
      tone(BUZZER_PIN, 3000);
      //digitalWrite(EN_PIN, HIGH); // Disable motor
    }
  }

  taskYIELD(); // Yield to Core 0
}

void stepperTask(void *parameter) {
  while (true) {
    if (motorRunning && !motorPermanentlyStopped) {
      digitalWrite(STEP_PIN, HIGH);
      delayMicroseconds(stepDelayMicros);
      digitalWrite(STEP_PIN, LOW);
      delayMicroseconds(stepDelayMicros);
    } else {
      delay(10); // Lower CPU use
    }

    vTaskDelay(1 / portTICK_PERIOD_MS); // Feed watchdog
  }
}

float readLux() {
  // digitalWrite(SLED, HIGH);
  delay(50);
  uint32_t lum = tsl.getFullLuminosity();
  uint16_t ir = lum >> 16;
  uint16_t full = lum & 0xFFFF;
  // digitalWrite(SLED, LOW);
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
