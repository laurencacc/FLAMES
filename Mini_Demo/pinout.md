# 🔌 Lauren's ESP32 ↔ Basys 3 Wire Mapping Summary

| **ESP32 GPIO** | **Function**        | **Connects To**   | **Basys 3 JA Pin** | **FPGA Pin** | **Constraint Name** |
|----------------|---------------------|-------------------|--------------------|--------------|----------------------|
| `GPIO13`       | SPI MOSI            | → `spi_mosi`      | JA1 (top right)    | `J1`         | `JA1`               |
| `GPIO14`       | SPI CLK             | → `spi_clk`       | JA3                | `J2`         | `JA3`               |
| `GPIO12`       | SPI SS (CS)         | → `spi_cs`        | JA4                | `G2`         | `JA4`               |
| `GPIO25`       | FIRE DETECT READ    | ← `detect_flag`   | JA7 (bottom right) | `H1`         | `JA7`               |
| GND            | Ground              | ↔ JA GND          | JA5 or JA11        | —            | —                   |
| (optional)     | 3.3V if needed      | ↔ JA VCC          | JA6 or JA12        | —            | —                   |
