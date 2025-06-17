module fire_alert_logic (
    input wire clk,              // System clock (can be slow, e.g. 1 Hz-50 MHz)
    input wire reset_n,          // Active-low reset
    input wire fft_flag,         // From ESP32: FFT fire detected
    input wire cam_flag,         // From Pi: Camera fire detected
    output reg final_alert       // To ESP32: Final confirmed fire alert
);

// Optional: Synchronize inputs if coming from async sources
reg fft_sync_1, fft_sync_2;
reg cam_sync_1, cam_sync_2;

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        {fft_sync_1, fft_sync_2} <= 2'b00;
        {cam_sync_1, cam_sync_2} <= 2'b00;
    end else begin
        fft_sync_1 <= fft_flag;
        fft_sync_2 <= fft_sync_1;

        cam_sync_1 <= cam_flag;
        cam_sync_2 <= cam_sync_1;
    end
end

// Combine synced flags
always @(posedge clk or negedge reset_n) begin
    if (!reset_n)
        final_alert <= 1'b0;
    else
        final_alert <= fft_sync_2 & cam_sync_2;
end

endmodule
