module spi_slave (
  input wire clk,              // Basys3 100 MHz clock
  input wire spi_clk,          // SPI clock from ESP32
  input wire spi_mosi,         // SPI data from ESP32
  input wire spi_cs,           // SPI SS (active-low)
  output reg [15:0] data_out,  // Received lux data
  output reg data_valid,       // Goes high for one clock when data received
  output reg debug_led         // Toggles on each full word
);

  reg [15:0] shift_reg = 0;
  reg [4:0] bit_cnt = 0;

  // Synchronizers
  reg [1:0] spi_clk_sync = 0;
  reg [1:0] spi_mosi_sync = 0;

  reg spi_clk_prev = 0;
  wire spi_clk_rising = (spi_clk_sync[1] == 1) && (spi_clk_prev == 0);

  always @(posedge clk) begin
    // Synchronize external signals to internal clock domain
    spi_clk_sync  <= {spi_clk_sync[0], spi_clk};
    spi_mosi_sync <= {spi_mosi_sync[0], spi_mosi};

    spi_clk_prev <= spi_clk_sync[1];
    data_valid <= 0;

    if (!spi_cs) begin
      if (spi_clk_rising) begin
        shift_reg <= {shift_reg[14:0], spi_mosi_sync[1]};
        bit_cnt <= bit_cnt + 1;
        if (bit_cnt == 15) begin
          data_out <= {shift_reg[14:0], spi_mosi_sync[1]};
          data_valid <= 1;
          bit_cnt <= 0;
          debug_led <= ~debug_led;  // Blink to confirm transfer
        end
      end
    end else begin
      bit_cnt <= 0;
    end
  end
endmodule
