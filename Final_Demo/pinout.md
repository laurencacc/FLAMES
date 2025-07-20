
# Pinout Documentation

## Overview
This system integrates the ESP32, Basys 3 FPGA, and Raspberry Pi to form a fire detection pipeline:
- **TSL2591**, **FDS1010**, & **Raspberry Pi Camera Module** triggers detection.
- **ESP32** sends detection flag to **FPGA**.
- **ESP32** also triggers the **Raspberry Pi**, which runs camera-based fire detection.
- Final decision is handled by the **FPGA**, which sends the confirmed alert back to the **ESP32**.

---

## ESP32 Pinout

| Function                    | ESP32 Pin               | Direction     | Connected To         |
|-----------------------------|-------------------------|---------------|----------------------|
| Stepper Motor - STEP        | GPIO 18                 | Output        | Stepper Driver       |
| Stepper Motor - DIR         | GPIO 19                 | Output        | Stepper Driver       |
| Motor LED                   | GPIO 4                  | Output        | Onboard LED          |
| Buzzer                      | GPIO 33                 | Output        | Buzzer               |
| Button                      | GPIO 26                 | Input         | Push Button (GND)    |
| TSL2591 I2C - SCL           | GPIO 22                 | Output        | TSL2591 SCL          |
| TSL2591 I2C - SDA           | GPIO 21                 | I/O           | TSL2591 SDA          |
| Internal ADC                | GPIO 27                 | Input         | 12-bit ADC           |
| Pi Trigger (TSL detected)   | GPIO 32 - CS Pi         | Output        | Pi GPIO 18 (input)   |
| FFT Fire Flag (to FPGA)     | GPIO 17 - CS FPGA       | Output        | Basys 3 JA1          |
| Final Fire Alert (from FPGA)| GPIO 25 - Trigger Pin   | Input         | Basys 3 JA3          |
| FPGA Reset Pin              | GPIO 13 - MOSI          | Output        | Basys 3 JA4          |

---

## Basys 3 FPGA Pinout

| Signal               | JA Port Pin | Direction | Connected To       |
|----------------------|-------------|-----------|--------------------|
| FFT Fire Flag In     | JA1         | Input     | ESP32 GPIO 17      |
| Pi Fire Flag In      | JA2         | Input     | Pi GPIO 19         |
| Final Alert Out      | JA3         | Output    | ESP32 GPIO 25      |
| Reset Pin            | JA4         | Input     | ESP32 GPIO 13      |
| GND                  | JA5         | GND       | Shared Ground      |

> **NOTE:** All Basys 3 inputs/outputs must be 3.3V logic. Double-check level compatibility!

---

## Raspberry Pi Pinout

| Function                    | Pi GPIO | Direction | Connected To         |
|-----------------------------|---------|-----------|----------------------|
| Trigger From ESP32          | GPIO 18 | Input     | ESP32 GPIO 32        |
| Fire Flag to FPGA           | GPIO 19 | Output    | Basys 3 JA2          |
| Ground                      | GND     | GND       | Shared Ground        |
| Camera Connection           | CSI     | Input     | CMOS Sensor          |

---

## Shared Connections

- **All devices must share a common ground** to ensure signal integrity.
- Use short wires for GPIO-level signals to avoid noise.
