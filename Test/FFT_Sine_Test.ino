#include "arduinoFFT.h"
#include <math.h>  // Needed for sin()

#define PI 3.14159265358979323846

const uint16_t samples = 1024;
const double samplingFrequency = 100.0;
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
  const double signalFreq = 7.0;
  for (uint16_t i = 0; i < samples; i++) {
    vReal[i] = 100.0 * sin(2 * PI * signalFreq * i / samplingFrequency);
    vImag[i] = 0.0;
  }

  // === Time Domain Output ===
  Serial.println("--- Time Domain ---");
  Serial.println("Time(ms)\tSignal");
  for (uint16_t i = 0; i < samples; i++) {
    double time_ms = (i * 1000.0) / samplingFrequency;
    Serial.print(time_ms, 2);
    Serial.print("\t");
    Serial.println(vReal[i], 2);
  }

  // === Run FFT ===
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();

  // === Frequency Domain Output ===
  Serial.println("--- Frequency Domain ---");
  Serial.println("Freq(Hz)\tMagnitude");
  double freqResolution = samplingFrequency / samples;
  for (uint16_t i = 0; i < samples / 2; i++) {  // Only half has unique data
    double freq = i * freqResolution;
    Serial.print(freq, 2);
    Serial.print("\t");
    Serial.println(vReal[i], 2);
  }

  // === Flame Detection ===
  int startBin = ceil(5.0 / freqResolution);
  int endBin = floor(10.0 / freqResolution);
  const double threshold = 80.0;
  int activeBins = 0;
  double energySum = 0.0;

  for (int i = startBin; i <= endBin; i++) {
    if (vReal[i] > threshold) {
      activeBins++;
      energySum += vReal[i];
    }
  }

  bool flameDetectedNow = (activeBins >= 5) && (energySum > 1000.0);
  if (flameDetectedNow) flameConfirmCount++;
  else flameConfirmCount = 0;

  if (flameConfirmCount >= 2) {
    Serial.println("🔥 Flame Detected!");
  }

  Serial.println();
  delay(2000);  // Give time for plotting before next run
}

