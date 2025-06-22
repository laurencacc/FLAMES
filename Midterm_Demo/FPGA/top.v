module top (
    input wire clk,               // 50 MHz or 100 MHz FPGA clock
    input wire reset_n,           // Active-low reset
    input wire fft_flag_in,       // From ESP32
    input wire cam_flag_in,       // From Pi
    output wire final_alert_out,  // To ESP32
    output wire [2:0] led         // Debug LEDs
);

    wire fft_debug;
    wire cam_debug;

    fire_alert_logic fire_logic_inst (
        .clk(clk),
        .reset_n(reset_n),
        .fft_flag_in(fft_flag_in),
        .cam_flag_in(cam_flag_in),
        .final_alert_out(final_alert_out),
        .fft_debug(fft_debug),
        .cam_debug(cam_debug)
    );

    assign led[0] = fft_debug;
    assign led[1] = cam_debug;
    assign led[2] = final_alert_out;

endmodule
