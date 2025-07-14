#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2591.h>

// --- Fire Detection (EMA + IR Ratio) ---
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);
uint16_t ema = 0;
bool ema_initialized = false;
const uint8_t EMA_ALPHA = 20;
const uint16_t THRESHOLD_OFFSET = 200;
const float IR_RATIO_THRESHOLD = 0.35;
const unsigned long sensorInterval = 500;

// --- Flags ---
bool motorStopped = false;
bool fireConfirmed = false;

void setup() {
  Serial.begin(9600);
  tsl.setGain(TSL2591_GAIN_MED);
  tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);
  Serial.println("🔎 Fire detection initialized...");
}

void loop() {
  float lux_visible;
  float ir_ratio;
  getLightData(lux_visible, ir_ratio);
  uint16_t lux_fixed = (uint16_t)(lux_visible * 10.0);

  // --- Update EMA
  if (!ema_initialized) {
    ema = lux_fixed;
    ema_initialized = true;
  } else {
    ema = ((EMA_ALPHA * lux_fixed) + (64 - EMA_ALPHA) * ema) >> 6;
  }

  uint16_t threshold = ema + THRESHOLD_OFFSET;
  bool lux_trigger = lux_fixed > threshold;

  // --- Always log current sensor values
  Serial.printf("Lux: %.1f (Fixed: %u), EMA: %u, Threshold: %u, IR Ratio: %.2f\n",
                lux_visible, lux_fixed, ema, threshold, ir_ratio);

  // --- Detection Logic
  if (lux_trigger && !motorStopped) {
    motorStopped = true;
    Serial.println("⚠️ Motor stopped due to light spike!");
  }

  if (motorStopped && !fireConfirmed) {
    bool ir_trigger = ir_ratio > IR_RATIO_THRESHOLD;
    if (ir_trigger) {
      fireConfirmed = true;
      Serial.println("🔥 FIRE CONFIRMED via IR ratio!");
    } else {
      Serial.println("🔍 Verifying... IR spike not high enough yet");
    }
  }

  delay(sensorInterval);
}

void getLightData(float &lux_visible, float &ir_ratio) {
  uint32_t lum = tsl.getFullLuminosity();
  uint16_t ir = lum >> 16;
  uint16_t full = lum & 0xFFFF;
  lux_visible = (float)(full - ir);

  uint32_t real_full = (uint32_t)ir + (uint32_t)full;
  ir_ratio = (real_full == 0) ? 0.0 : (float)ir / (float)real_full;
}
