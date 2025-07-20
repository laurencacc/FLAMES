## Clock input (using 100 MHz system clock on W5)
set_property -dict { PACKAGE_PIN W5 IOSTANDARD LVCMOS33 } [get_ports clk]
create_clock -add -name sys_clk_pin -period 10.00 -waveform {0 5} [get_ports clk]

## Reset (now driven by ESP32 via JA4, pin G2)
set_property -dict { PACKAGE_PIN G2 IOSTANDARD LVCMOS33 } [get_ports reset_n]

## FFT flag input from ESP32 (JA1)
set_property -dict { PACKAGE_PIN J1 IOSTANDARD LVCMOS33 } [get_ports fft_flag_in]

## Camera flag input from Pi (JA2)
set_property -dict { PACKAGE_PIN L2 IOSTANDARD LVCMOS33 } [get_ports cam_flag_in]

## Final alert output to ESP32 (JA3)
set_property -dict { PACKAGE_PIN J2 IOSTANDARD LVCMOS33 } [get_ports final_alert_out]

## LEDs for debug
set_property -dict { PACKAGE_PIN U16 IOSTANDARD LVCMOS33 } [get_ports led[0]]
set_property -dict { PACKAGE_PIN E19 IOSTANDARD LVCMOS33 } [get_ports led[1]]
set_property -dict { PACKAGE_PIN U19 IOSTANDARD LVCMOS33 } [get_ports led[2]]

## Optional: SPI bus config
set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4 [current_design]
