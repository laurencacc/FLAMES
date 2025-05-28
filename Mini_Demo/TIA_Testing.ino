#include "arduinoFFT.h"

const uint16_t samples = 64;
const double samplingFrequency = 20.0;
const int adcPin = 34;
int flameConfirmCount = 0;

double vReal[samples];
double vImag[samples];

ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, samples, samplingFrequency);

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  while (!Serial);
  Serial.println("Starting FFT for Flame Detection (5–10 Hz)");
}

void loop() {
  // 1. Sample ADC
  for (uint16_t i = 0; i < samples; i++) {
    vReal[i] = analogRead(adcPin);
    vImag[i] = 0.0;
    delayMicroseconds(1000000 / samplingFrequency);
  }

  // 2. Run FFT
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();

  // 3. Detection over 5–10 Hz
  double freqResolution = samplingFrequency / samples;
  int startBin = ceil(5.0 / freqResolution);
  int endBin = floor(10.0 / freqResolution);

  const double threshold = 80.0;
  int activeBins = 0;
  double energySum = 0.0;

  for (int i = startBin; i <= endBin; i++) {
    double freq = i * freqResolution;
    double magnitude = vReal[i];

    Serial.print(freq, 3);
    Serial.print("\t");
    Serial.println(magnitude, 2);

    if (magnitude > threshold) {
      activeBins++;
      energySum += magnitude;
    }
  }

  Serial.println();

  // 4. Determine flame detection for this round
  bool flameDetectedNow = (activeBins >= 5) && (energySum > 1000.0);

  if (flameDetectedNow) {
    flameConfirmCount++;
  } else {
    flameConfirmCount = 0;
  }

  if (flameConfirmCount >= 2) {
    Serial.println("🔥 Flame Detected!");
  }

  Serial.println();
  delay(500);
} 