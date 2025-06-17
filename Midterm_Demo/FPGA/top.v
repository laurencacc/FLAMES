module top (
    input wire clk,             // 50 MHz or 100 MHz FPGA clock
    input wire reset_n,         // Active-low reset
    input wire fft_flag_in,     // From ESP32
    input wire cam_flag_in,     // From Pi
    output wire final_alert_out // To ESP32
);

    // Instantiate fire_alert_logic
    fire_alert_logic fire_logic_inst (
        .clk(clk),
        .reset_n(reset_n),
        .fft_flag(fft_flag_in),
        .cam_flag(cam_flag_in),
        .final_alert(final_alert_out)
    );

endmodule
