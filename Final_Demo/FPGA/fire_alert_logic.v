module fire_alert_logic #(
    parameter DELAY_CYCLES = 10
)(
    input  wire clk,
    input  wire reset_n,
    input  wire fft_flag_in,
    input  wire cam_flag_in,
    output reg  final_alert_out,
    output wire fft_debug,
    output wire cam_debug
);

    // === 1. Synchronize inputs
    reg fft_sync0, fft_sync1;
    reg cam_sync0, cam_sync1;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            fft_sync0 <= 0; fft_sync1 <= 0;
            cam_sync0 <= 0; cam_sync1 <= 0;
        end else begin
            fft_sync0 <= fft_flag_in;
            fft_sync1 <= fft_sync0;
            cam_sync0 <= cam_flag_in;
            cam_sync1 <= cam_sync0;
        end
    end

    // === 2. Latch
    reg fft_latched, cam_latched;
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            fft_latched <= 0;
            cam_latched <= 0;
        end else begin
            if (fft_sync1) fft_latched <= 1;
            if (cam_sync1) cam_latched <= 1;
        end
    end

    assign fft_debug = fft_latched;
    assign cam_debug = cam_latched;

    // === 3. Final alert delay logic
    localparam DELAY_WIDTH = 4;
    reg [DELAY_WIDTH-1:0] delay_cnt;
    reg delay_active;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            delay_cnt       <= 0;
            delay_active    <= 0;
            final_alert_out <= 0;
        end else if (!final_alert_out) begin
            if (fft_latched && cam_latched && !delay_active) begin
                delay_active <= 1;
                delay_cnt    <= 0;
            end

            if (delay_active) begin
                if (delay_cnt < DELAY_CYCLES - 1) begin
                    delay_cnt <= delay_cnt + 1;
                end else begin
                    final_alert_out <= 1;
                end
            end
        end
    end

endmodule
