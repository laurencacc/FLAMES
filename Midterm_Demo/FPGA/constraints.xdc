## Clock input (using 100 MHz system clock on W5)
set_property -dict { PACKAGE_PIN W5   IOSTANDARD LVCMOS33 } [get_ports clk]
create_clock -add -name sys_clk_pin -period 10.00 -waveform {0 5} [get_ports clk]

## Reset button (active low) - using center button
set_property -dict { PACKAGE_PIN U18   IOSTANDARD LVCMOS33 } [get_ports reset_n]

## FPGA input: fft_flag_in (from ESP32)
set_property -dict { PACKAGE_PIN J1   IOSTANDARD LVCMOS33 } [get_ports fft_flag_in]

## FPGA input: cam_flag_in (from Pi or ESP32)
set_property -dict { PACKAGE_PIN L2   IOSTANDARD LVCMOS33 } [get_ports cam_flag_in]

## FPGA output: final_alert_out (to ESP32)
set_property -dict { PACKAGE_PIN J2   IOSTANDARD LVCMOS33 } [get_ports final_alert_out]

set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4 [current_design]