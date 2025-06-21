#include "arduinoFFT.h"

const uint16_t samples = 1024;
const double samplingFrequency = 100.0;
int flameConfirmCount = 0;

double vReal[samples];
double vImag[samples];

ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, samples, samplingFrequency);

// SPI ADC (ADS8319) Pin Setup
const int PIN_CS   = 27;
const int PIN_MISO = 16;
const int PIN_MOSI = 13;  // Not used for ADS8319
const int PIN_SCLK = 14;

const float VREF = 3.3;
const int ADC_BITS = 16;

// EMA Variables
const double alpha = 0.2;
double emaMagnitude = 0.0;
bool emaInitialized = false;
const double thresholdMargin = 1000.0;  // reduced to improve detection

// Detection Range
double freqResolution;
int startBin, endBin;

void setup() {
  Serial.begin(115200);

  pinMode(PIN_CS, OUTPUT);
  pinMode(PIN_SCLK, OUTPUT);
  pinMode(PIN_MISO, INPUT);
  digitalWrite(PIN_CS, HIGH);
  digitalWrite(PIN_SCLK, LOW);

  Serial.println("Starting FFT for Flame Detection with EMA Thresholding (5–10 Hz)");

  freqResolution = samplingFrequency / samples;
  startBin = ceil(5.0 / freqResolution);
  endBin = floor(10.0 / freqResolution);
}

uint16_t readADS8319() {
  uint16_t result = 0;
  digitalWrite(PIN_CS, LOW);
  delayMicroseconds(1);

  for (int i = 0; i < 16; i++) {
    digitalWrite(PIN_SCLK, HIGH);
    delayMicroseconds(1);
    result <<= 1;
    if (digitalRead(PIN_MISO)) {
      result |= 0x01;
    }
    digitalWrite(PIN_SCLK, LOW);
    delayMicroseconds(1);
  }

  digitalWrite(PIN_CS, HIGH);
  return result;
}

void loop() {
  Serial.println("[CYCLE_START]");
  Serial.print("[CYCLE_TIMESTAMP],");
  Serial.println(millis() / 1000.0, 3);

  const double ADC_OFFSET = 32768.0;
  Serial.println("Time_s,ADC_Value");
  for (uint16_t i = 0; i < samples; i++) {
    uint16_t adcValue = readADS8319();
    vReal[i] = adcValue - ADC_OFFSET;
    vImag[i] = 0.0;

    double time_s = i / samplingFrequency;
    Serial.print(time_s, 3);
    Serial.print(",");
    Serial.println(adcValue);

    delayMicroseconds(1000000 / samplingFrequency);
  }


  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();

  int activeBins = 0;
  double energySum = 0.0;
  double peakMagnitude = 0.0;
  double peakFrequency = 0.0;
  double currentBandSum = 0.0;

  for (int i = 1; i < samples / 2; i++) {
    double freq = i * freqResolution;
    double magnitude = vReal[i];

    if (i >= startBin && i <= endBin) {
      currentBandSum += magnitude;
    }

    if (magnitude > peakMagnitude) {
      peakMagnitude = magnitude;
      peakFrequency = freq;
    }

    Serial.print(freq, 3);
    Serial.print(",");
    Serial.println(magnitude, 2);
  }

  // Serial.println(">> 5–10 Hz bin magnitudes:");
  // for (int i = startBin; i <= endBin; i++) {
  //   Serial.print("Bin ");
  //   Serial.print(i);
  //   Serial.print(" (");
  //   Serial.print(i * freqResolution, 2);
  //   Serial.print(" Hz): ");
  //   Serial.println(vReal[i], 2);
  // }

  // Use average magnitude for EMA
  int binCount = endBin - startBin + 1;
  double bandAvg = currentBandSum / binCount;

  if (!emaInitialized) {
    emaMagnitude = bandAvg;
    emaInitialized = true;
  } else {
    emaMagnitude = alpha * bandAvg + (1 - alpha) * emaMagnitude;
  }

  // Cap EMA to avoid runaway baseline
  emaMagnitude = min(emaMagnitude, 50000.0);
  double adaptiveThreshold = emaMagnitude + thresholdMargin;

  // Re-check active bins using threshold
  activeBins = 0;
  energySum = 0.0;
  for (int i = startBin; i <= endBin; i++) {
    if (vReal[i] > adaptiveThreshold) {
      activeBins++;
      energySum += vReal[i];
    }
  }

  Serial.println("====== FFT Flame Detection Summary ======");
  Serial.print("Cycle Time (s): ");
  Serial.println(millis() / 1000.0, 3);
  Serial.print("Peak Frequency (Hz): ");
  Serial.println(peakFrequency, 3);
  Serial.print("EMA Baseline: ");
  Serial.println(emaMagnitude, 2);
  Serial.print("Adaptive Threshold: ");
  Serial.println(adaptiveThreshold, 2);
  Serial.print("Active Bins (5–10 Hz): ");
  Serial.println(activeBins);
  Serial.print("Energy Sum (5–10 Hz): ");
  Serial.println(energySum, 2);
  bool flameDetectedNow = (activeBins >= 5) && (energySum > (2 * adaptiveThreshold));
  Serial.print("🔥 Flame Detected? ");
  Serial.println(flameDetectedNow ? "YES" : "no");
  Serial.println("=========================================\n");


  Serial.println("[CYCLE_END]\n");

  delay(500);
}
