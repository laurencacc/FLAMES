module fire_detect(
  input clk,
  input [15:0] lux_in,
  input data_valid,
  output reg fire_detected
);

  reg [15:0] ema = 0;
  reg ema_initialized = 0;

  parameter EMA_ALPHA = 8'd20;
  parameter THRESHOLD_OFFSET = 16'd200;

  wire [15:0] ema_new;
  wire [16:0] threshold;

  assign ema_new = ((EMA_ALPHA * lux_in) + ((64 - EMA_ALPHA) * ema)) >> 6;
  assign threshold = {1'b0, ema} + {1'b0, THRESHOLD_OFFSET};

  always @(posedge clk) begin
    if (data_valid) begin
      if (!ema_initialized) begin
        ema <= lux_in;
        ema_initialized <= 1;
        fire_detected <= 0;  // Don't trigger detection on first sample
      end else begin
        ema <= ema_new;
        fire_detected <= ({1'b0, lux_in} > threshold) ? 1'b1 : 1'b0;
      end
    end
  end
endmodule
