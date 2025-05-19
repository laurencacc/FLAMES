#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2591.h>
#include <SPI.h>

// --- TSL2591 ---
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);

// --- SPI Pins ---
#define SPI_SCK    14
#define SPI_MOSI   13
#define SPI_SS     12
#define DETECT_PIN 25
#define MCU_LED    2

// --- Stepper Motor Pins ---
#define STEP_PIN   18
#define DIR_PIN    19
#define EN_PIN     5
#define BUTTON_PIN 15
#define BUZZER_PIN 4

// --- Motor Control ---
const int stepDelayMicros = 900;
volatile bool motorRunning = false;
volatile bool motorPermanentlyStopped = false;
bool systemStarted = false;
bool buttonPreviouslyPressed = false;
bool ledBlinking = false;
unsigned long lastBlinkTime = 0;
const unsigned long blinkInterval = 250;

// --- Task Handle ---
TaskHandle_t stepperTaskHandle;

void setup() {
  Serial.begin(9600);

  // --- SPI Setup ---
  SPI.begin(SPI_SCK, -1, SPI_MOSI, SPI_SS);
  pinMode(SPI_SS, OUTPUT);
  digitalWrite(SPI_SS, HIGH);
  pinMode(DETECT_PIN, INPUT);

  // --- IO Setup ---
  pinMode(MCU_LED, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);

  digitalWrite(MCU_LED, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(DIR_PIN, HIGH);
  digitalWrite(EN_PIN, LOW);
  noTone(BUZZER_PIN);

  // --- Sensor Setup ---
  tsl.setGain(TSL2591_GAIN_MED);
  tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);
  delay(500);

  Serial.println("🔴 System idle. Press button to start.");

  // Start Motor Task
  xTaskCreatePinnedToCore(stepperTask, "Stepper Task", 10000, NULL, 1, &stepperTaskHandle, 0);
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
      digitalWrite(EN_PIN, LOW);
      digitalWrite(MCU_LED, LOW);
    } else {
      Serial.println("🔁 System Reset.");
      motorRunning = false;
      motorPermanentlyStopped = false;
      ledBlinking = false;
      digitalWrite(MCU_LED, LOW);
      noTone(BUZZER_PIN);
      digitalWrite(EN_PIN, HIGH);
    }
  }

  buttonPreviouslyPressed = pressed;
}

void loop() {
  handleButton();

  if (!systemStarted) {
    taskYIELD();
    return;
  }

  if (ledBlinking && millis() - lastBlinkTime >= blinkInterval) {
    lastBlinkTime = millis();
    digitalWrite(MCU_LED, !digitalRead(MCU_LED));
  }

  float lux = readLux();
  uint16_t lux_fixed = (uint16_t)(lux * 10);

  digitalWrite(SPI_SS, LOW);
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  SPI.transfer16(lux_fixed);
  SPI.endTransaction();
  digitalWrite(SPI_SS, HIGH);

  if (digitalRead(DETECT_PIN) == HIGH) {
    Serial.println("🔥 FIRE DETECTED BY FPGA!");
    Serial.printf("Lux: %.1f (sent %u)\n", lux, lux_fixed);
    digitalWrite(MCU_LED, HIGH);
    motorRunning = false;
    ledBlinking = true;
    tone(BUZZER_PIN, 3000);
    digitalWrite(EN_PIN, HIGH);
  } else {
    Serial.printf("Lux: %.1f (sent %u)\n", lux, lux_fixed);
  }

  delay(500);
}

void stepperTask(void* parameter) {
  while (true) {
    if (motorRunning && !motorPermanentlyStopped) {
      digitalWrite(STEP_PIN, HIGH);
      delayMicroseconds(stepDelayMicros);
      digitalWrite(STEP_PIN, LOW);
      delayMicroseconds(stepDelayMicros);
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
  return (float)(full - ir);
}
