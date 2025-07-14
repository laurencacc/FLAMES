#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2591.h>

// --- Fire Detection (EMA + Shape Analysis) ---
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);
uint16_t ema = 0;
bool ema_initialized = false;
const uint8_t EMA_ALPHA = 20;
const uint16_t THRESHOLD_OFFSET = 200;
const unsigned long sensorInterval = 200;

// --- Flags ---
bool motorStopped = false;
bool fireConfirmed = false;

// --- Buffer for shape analysis ---
const int N = 20;  // Number of samples for flicker shape
float luxBuffer[N];
int bufferIndex = 0;
bool bufferFull = false;

// --- Thresholds for shape-based fire detection ---
const float MIN_AVG_SLOPE = 10.0;
const float MAX_AVG_SLOPE = 120.0;

const float MIN_VARIANCE = 1000.0;
const float MAX_VARIANCE = 9000.0;


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

  // --- Log sensor values
  Serial.printf("Lux: %.1f (Fixed: %u), EMA: %u, Threshold: %u, IR Ratio: %.2f\n",
                lux_visible, lux_fixed, ema, threshold, ir_ratio);

  // --- Store into lux buffer
  updateLuxBuffer(lux_visible);

  // --- Detection Logic
  if (lux_trigger && !motorStopped) {
    motorStopped = true;
    Serial.println("⚠️ Motor stopped due to light spike!");
  }

  if (motorStopped && !fireConfirmed && bufferFull) {
    if (isFireLike(ir_ratio)) {
      fireConfirmed = true;
      Serial.println("🔥 FIRE CONFIRMED via shape analysis!");
    } else {
      Serial.println("❌ Not fire-like. Possibly a sparkler or false positive.");
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

void updateLuxBuffer(float newLux) {
  luxBuffer[bufferIndex++] = newLux;
  if (bufferIndex >= N) {
    bufferIndex = 0;
    bufferFull = true;
  }
}

float computeAvgSlope() {
  float totalSlope = 0;
  for (int i = 1; i < N; i++) {
    totalSlope += abs(luxBuffer[i] - luxBuffer[i - 1]);
  }
  return totalSlope / (N - 1);
}

float computeVariance() {
  float mean = 0;
  for (int i = 0; i < N; i++) {
    mean += luxBuffer[i];
  }
  mean /= N;

  float variance = 0;
  for (int i = 0; i < N; i++) {
    float diff = luxBuffer[i] - mean;
    variance += diff * diff;
  }
  return variance / N;
}

bool isFireLike(float ir_ratio) {
  float avgSlope = computeAvgSlope();
  float variance = computeVariance();

  Serial.printf("📉 AvgSlope: %.2f, 📊 Variance: %.2f\n", avgSlope, variance);
  
  if (ir_ratio >= 0.37) return false;


  return (avgSlope > MIN_AVG_SLOPE && avgSlope < MAX_AVG_SLOPE) &&
         (variance > MIN_VARIANCE && variance < MAX_VARIANCE);
}

