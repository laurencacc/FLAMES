## SPI from ESP32 to FPGA (Corrected)

set_property PACKAGE_PIN J1 [get_ports JA1]   ; JA1 = spi_mosi
set_property PACKAGE_PIN J2 [get_ports JA3]   ; JA3 = spi_clk
set_property PACKAGE_PIN G2 [get_ports JA4]   ; JA4 = spi_ss
set_property PACKAGE_PIN H1 [get_ports JA7]   ; JA7 = detect flag

set_property IOSTANDARD LVCMOS33 [get_ports JA1]
set_property IOSTANDARD LVCMOS33 [get_ports JA3]
set_property IOSTANDARD LVCMOS33 [get_ports JA4]
set_property IOSTANDARD LVCMOS33 [get_ports JA7]


set_property PACKAGE_PIN W5 [get_ports CLK100MHZ]
set_property IOSTANDARD LVCMOS33 [get_ports CLK100MHZ]

set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4 [current_design]

set_property PACKAGE_PIN U16 [get_ports {LED0}]
set_property IOSTANDARD LVCMOS33 [get_ports {LED0}]
