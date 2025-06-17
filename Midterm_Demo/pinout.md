ESP32-Wroom-32D
| **Signal**      | **ESP32 GPIO**    | **Direction** | **Connected To**         | **Notes**                   |
| --------------- | ----------------- | ------------- | ------------------------ | --------------------------- |
| TSL2591 Sensor  | I²C (e.g., 21/22) | Input         | TSL2591                  | Standard I²C connection     |
| FFT Flag Output | 13                | Output        | FPGA (fft\_flag)         | High when FFT detects fire  |
| Final Alert In  | 14                | Input         | FPGA (final\_alert\_out) | Triggers buzzer/motor stop  |
| Pi Trigger      | 23                | Output        | Pi GPIO 18               | Tells Pi to start detection |
| GND             | —                 | GND           | Shared with all          | Required for logic levels   |

BASYS 3 Board
| **Signal**      | **FPGA Pin**       | **Direction** | **Connected To**     | **Notes**           |
| --------------- | ------------------ | ------------- | -------------------- | ------------------- |
| FFT Flag In     | (your\_pin1)       | Input         | ESP32 GPIO 13        | Comes from ESP32    |
| Camera Flag In  | (your\_pin2)       | Input         | Pi GPIO 19           | Comes from Pi       |
| Final Alert Out | (your\_pin3)       | Output        | ESP32 GPIO 14        | Combines both flags |
| clk             | (your\_clk\_pin)   | Input         | Onboard oscillator   | Typically 50 MHz    |
| reset\_n        | (your\_reset\_pin) | Input         | Button or logic high | Active-low reset    |
| GND             | —                  | GND           | Shared with all      |                     |

Raspberry Pi 5
| **Signal**            | **Pi GPIO (BCM)** | **Physical Pin** | **Direction** | **Connected To** | **Notes**                  |
| --------------------- | ----------------- | ---------------- | ------------- | ---------------- | -------------------------- |
| Trigger In from ESP32 | 18                | Pin 12           | Input         | ESP32 GPIO 23    | Starts OpenCV detection    |
| Camera Flag Out       | 19                | Pin 35           | Output        | FPGA (cam\_flag) | High when fire is detected |
| GND                   | —                 | Pin 6 or 14      | GND           | Shared           | Required                   |
