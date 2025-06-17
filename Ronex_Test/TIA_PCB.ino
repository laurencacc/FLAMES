const int PIN_CS   = 27;
const int PIN_MISO = 16;
const int PIN_MOSI = 13;  // Not used by ADS8319 (it's input only)
const int PIN_SCLK = 14;

const float VREF = 3.3;  // Reference voltage of the ADS8319
const int ADC_BITS = 16; // 16-bit ADC

void setup() {
  Serial.begin(115200);

  pinMode(PIN_CS, OUTPUT);
  pinMode(PIN_SCLK, OUTPUT);
  pinMode(PIN_MISO, INPUT);

  digitalWrite(PIN_CS, HIGH);
  digitalWrite(PIN_SCLK, LOW);
}

// Function to perform a single ADC read (16 bits)
uint16_t readADS8319() {
  uint16_t result = 0;

  // Start communication
  digitalWrite(PIN_CS, LOW);
  delayMicroseconds(1);

  // Wait for 16 clock cycles to receive data
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
  uint16_t adcValue = readADS8319();

  // Convert ADC value to voltage
  float voltage = (adcValue / 65535.0) * VREF;

  Serial.println(voltage);
  delay(10);  // Smooth the plot
}
