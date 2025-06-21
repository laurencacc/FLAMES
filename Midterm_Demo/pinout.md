
# 🔌 Pinout Documentation

## 🧠 Overview
This system integrates the ESP32, Basys 3 FPGA, and Raspberry Pi to form a fire detection pipeline:
- **TSL2591** triggers FFT detection.
- **ESP32** sends detection flag to **FPGA**.
- **ESP32** also triggers the **Raspberry Pi**, which runs camera-based fire detection.
- Final decision is handled by the **FPGA**, which sends the confirmed alert back to the **ESP32**.

---

## 📟 ESP32 Pinout

| Function                    | ESP32 Pin | Direction     | Connected To         |
|-----------------------------|-----------|---------------|----------------------|
| Stepper Motor - STEP        | GPIO 18   | Output        | Stepper Driver       |
| Stepper Motor - DIR         | GPIO 19   | Output        | Stepper Driver       |
| Motor LED                   | GPIO 4    | Output        | Onboard LED          |
| Buzzer                      | GPIO 33   | Output        | Buzzer               |
| Button                      | GPIO 26   | Input         | Push Button (GND)    |
| TSL2591 I2C - SCL           | GPIO 22   | Output        | TSL2591 SCL          |
| TSL2591 I2C - SDA           | GPIO 21   | I/O           | TSL2591 SDA          |
| ADS8319 - CS                | GPIO 27   | Output        | ADC Chip Select      |
| ADS8319 - SCLK              | GPIO 14   | Output        | ADC Clock            |
| ADS8319 - MISO              | GPIO 16   | Input         | ADC MISO             |
| Pi Trigger (TSL detected)   | GPIO 32   | Output        | Pi GPIO 18 (input)   |
| FFT Fire Flag (to FPGA)     | GPIO 5    | Output        | Basys 3 JA1          |
| Final Fire Alert (from FPGA)| GPIO 25   | Input         | Basys 3 JA3          |

---

## 🧠 Basys 3 FPGA Pinout

| Signal                | JA Port Pin | Direction | Connected To       |
|----------------------|-------------|-----------|--------------------|
| FFT Fire Flag In     | JA1         | Input     | ESP32 GPIO 5       |
| Pi Fire Flag In      | JA2         | Input     | Pi GPIO 19         |
| Final Alert Out      | JA3         | Output    | ESP32 GPIO 25      |
| GND                  | JA5         | GND       | Shared Ground      |
| VCC (3.3V)           | JA6 (optional) | Output | 3.3V to other logic if needed |

> ⚠️ **NOTE:** All Basys 3 inputs/outputs must be 3.3V logic. Double-check level compatibility!

---

## 🍓 Raspberry Pi Pinout

| Function                    | Pi GPIO | Direction | Connected To         |
|-----------------------------|---------|-----------|----------------------|
| Trigger From ESP32          | GPIO 18 | Input     | ESP32 GPIO 32        |
| Fire Flag to FPGA           | GPIO 19 | Output    | Basys 3 JA2          |
| Ground                      | GND     | GND       | Shared Ground        |

---

## 🔁 Shared Connections

- **All devices must share a common ground** to ensure signal integrity.
- Use short wires for GPIO-level signals to avoid noise.
