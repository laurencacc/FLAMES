#include <Wire.h>
#include "Adafruit_TSL2591.h"
#include <SPI.h>

#define CS_PIN 5  // Chip select for FPGA

Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);

void setup() {
  Wire.begin();
  SPI.begin();
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  tsl.begin();
  tsl.setGain(TSL2591_GAIN_MED);
  tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);
}

void loop() {
  float lux = readLux();
  uint16_t fixedLux = (uint16_t)(lux * 256.0); // Convert to Q8.8

  digitalWrite(CS_PIN, LOW);
  SPI.transfer16(fixedLux);
  digitalWrite(CS_PIN, HIGH);

  delay(50);
}

float readLux() {
  uint32_t lum = tsl.getFullLuminosity();
  uint16_t ir = lum >> 16;
  uint16_t full = lum & 0xFFFF;
  return (float)(full - ir);
}
