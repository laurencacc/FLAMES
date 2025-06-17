#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2591.h>
#include <SPI.h>

// TSL2591
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);

// SPI Pins
#define SPI_SCK    14
#define SPI_MOSI   13
#define SPI_SS     12
#define DETECT_PIN 25  // Connected to Basys3 Pmod (e.g., JA7)
#define MCU_LED 2  // Built-in LED on most ESP32 boards


void setup() {
  Serial.begin(9600);

  // SPI Setup
  SPI.begin(SPI_SCK, -1, SPI_MOSI, SPI_SS);
  pinMode(SPI_SS, OUTPUT);
  digitalWrite(SPI_SS, HIGH);
  pinMode(DETECT_PIN, INPUT);

  // Sensor setup
  tsl.setGain(TSL2591_GAIN_MED);
  tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);
  pinMode(MCU_LED, OUTPUT);
  digitalWrite(MCU_LED, LOW);  // Start with LED off
  delay(500);
  Serial.println("ESP32 initialized.");
}

void loop() {
  float lux = readLux();
  uint16_t lux_fixed = (uint16_t)(lux * 10); // fixed-point: 1 decimal precision

  // SPI send to FPGA
  digitalWrite(SPI_SS, LOW);
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  SPI.transfer16(lux_fixed);
  SPI.endTransaction();
  digitalWrite(SPI_SS, HIGH);

  // Check detect flag from FPGA
  if (digitalRead(DETECT_PIN) == HIGH) {
    Serial.println("🔥 FIRE DETECTED BY FPGA!");
    Serial.printf("Lux: %.1f (sent %u)\n", lux, lux_fixed);
    digitalWrite(MCU_LED, HIGH);  // turn LED on
  } else {
    Serial.printf("Lux: %.1f (sent %u)\n", lux, lux_fixed);
    digitalWrite(MCU_LED, LOW);   // turn LED off
  }

  delay(500);  // Sample every 0.5 sec
}

float readLux() {
  uint32_t lum = tsl.getFullLuminosity();
  uint16_t ir = lum >> 16;
  uint16_t full = lum & 0xFFFF;
  return (float)(full - ir);  // visible light
}