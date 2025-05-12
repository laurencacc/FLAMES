# 🔥 FLAMES

**SD Group 6 | FLAMES Project**  
University of Central Florida  
Spring 2025  

## Overview

The FLAMES (Fire Detection and Monitoring for Emergency Systems) project is a real-time fire detection system developed by undergraduate students in Electrical, Computer, Optics Engineering. Unlike traditional smoke detectors, FLAMES uses optical sensors and signal processing to detect fire based solely on light characteristics.

## Project Structure

- **FPGA/**  
  Contains Verilog/HDL code used for real-time signal processing and output control.

- **Mini_Demo/**  
  Compact demonstration setup to test core functionality in a controlled environment.

- **Ronex_Test/**  
  Code and tests involving Ronex's board and motor interface trials.

- **TSL Threshold/**  
  Light intensity thresholding algorithms using the TSL2591 sensor, including EMA and Z-score logic.

- **Test/**  
  General testing and prototyping files.

 - **Raspberry_Pi/**  
  Raspberry Pi code using OpenCV on Python

## Technologies Used

- ESP32 Microcontroller  
- Adafruit TSL2591 Light Sensor  
- DRV8825 Stepper Motor Driver  
- Basys3 FPGA Board
- NEMA23 Motor  
- MATLAB HDL Coder  
- C++, Verilog, and Python  
- OpenCV (for image-based analysis)

## Objective

To detect fire signals quickly and accurately using light intensity, frequency characteristics, and real-time control systems—without relying on traditional smoke detection methods.

## Contributors

- Lauren Caccamise  
- Ronex Faustin
- Neil Singh
