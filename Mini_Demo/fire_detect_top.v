`timescale 1ns / 1ps

module fire_detect_top(
  input CLK100MHZ,            // Basys3 clock
  input JA1,                  // SPI_MOSI
  input JA3,                  // SPI_SCK
  input JA4,                  // SPI_SS
  output JA7,                  // detect flag to ESP32
  output LED0                 // debug LED
);
  wire [15:0] lux_data;
  wire valid;
  wire detect;
  wire debug_led;

  spi_slave spi_inst (
    .clk(CLK100MHZ),
    .spi_clk(JA3),
    .spi_mosi(JA1),
    .spi_cs(JA4),
    .data_out(lux_data),
    .data_valid(valid),
    .debug_led(debug_led)
  );

  fire_detect fire_inst (
    .clk(CLK100MHZ),
    .lux_in(lux_data),
    .data_valid(valid),
    .fire_detected(detect)
  );

  assign JA7 = detect;
  assign LED0 = debug_led;
endmodule

