`timescale 1ns/1ps
`include "project_global_param.vh"

module tb_mic_serial_rx;

    reg clk_50m = 0;
    reg rst_n   = 0;

    wire bclk, lrck, sample_pulse, tick_1ms;
    reg  [MAX_CHAN_NUM-1:0] mic_sdin;
    wire [DATA_WIDTH-1:0]   ch_data [0:MAX_CHAN_NUM-1];
    wire [MAX_CHAN_NUM-1:0] ch_valid;

    clk_gen u_clk_gen (
        .clk_50m      (clk_50m),
        .rst_n        (rst_n),
        .bclk         (bclk),
        .lrck         (lrck),
        .sample_pulse (sample_pulse),
        .tick_1ms     (tick_1ms)
    );

    mic_serial_rx u_mic_serial_rx (
        .clk_50m  (clk_50m),
        .rst_n    (rst_n),
        .bclk     (bclk),
        .lrck     (lrck),
        .mic_sdin (mic_sdin),
        .ch_data  (ch_data),
        .ch_valid (ch_valid)
    );

    always #10 clk_50m = ~clk_50m;

    reg bclk_d, lrck_d;
    always @(posedge clk_50m) begin
        bclk_d <= bclk;
        lrck_d <= lrck;
    end
    wire bclk_fall = !bclk && bclk_d;
    wire lrck_fall_tb = !lrck && lrck_d;

    reg [23:0] tx_shift [0:3];
    reg [5:0]  tx_bit_cnt;
    reg        tx_active;

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            tx_bit_cnt <= 6'd0;
            tx_active  <= 1'b0;
            mic_sdin   <= {MAX_CHAN_NUM{1'b0}};
            for (int i = 0; i < 4; i++) tx_shift[i] <= 24'd0;
        end else begin
            if (lrck_fall_tb) begin
                tx_shift[0] <= 24'h123456;
                tx_shift[1] <= 24'hABCDEF;
                tx_shift[2] <= 24'h0F0F0F;
                tx_shift[3] <= 24'hF0F0F0;
                tx_bit_cnt  <= 6'd0;
                tx_active   <= 1'b1;
            end else if (bclk_fall && tx_active) begin
                // 标准 I2S：LRCLK 边沿后的第一个 BCLK 下降沿即送 MSB，
                // 之前多出的"填充位"会使接收端整体右移 1 位（读回值 = 期望>>1）
                mic_sdin[0] <= tx_shift[0][23];
                mic_sdin[1] <= tx_shift[1][23];
                mic_sdin[2] <= tx_shift[2][23];
                mic_sdin[3] <= tx_shift[3][23];
                for (int i = 0; i < 4; i++)
                    tx_shift[i] <= {tx_shift[i][22:0], 1'b0};
                if (tx_bit_cnt == 6'd23)
                    tx_active <= 1'b0;
                else
                    tx_bit_cnt <= tx_bit_cnt + 1'b1;
            end
        end
    end

    integer valid_cnt = 0;
    integer err_cnt   = 0;

    always @(posedge clk_50m) begin
        if (ch_valid[0]) begin
            valid_cnt = valid_cnt + 1;
            $display("Time %0t: ch0=%h ch1=%h ch2=%h ch3=%h",
                     $time, ch_data[0], ch_data[1], ch_data[2], ch_data[3]);
            if (ch_data[0] !== 24'h123456 ||
                ch_data[1] !== 24'hABCDEF ||
                ch_data[2] !== 24'h0F0F0F ||
                ch_data[3] !== 24'hF0F0F0) begin
                err_cnt = err_cnt + 1;
                $display("ERROR: data mismatch at time %0t", $time);
            end
        end
    end

    initial begin
        rst_n = 0;
        #200;
        rst_n = 1;
        #5_000_000;
        $display("========================================");
        $display("Protocol: Standard I2S (1 BCLK delay)");
        $display("Total valid pulses: %0d", valid_cnt);
        $display("Total errors      : %0d", err_cnt);
        $display("========================================");
        if (err_cnt == 0 && valid_cnt > 0)
            $display("PASS");
        else
            $display("FAIL");
        $finish;
    end

endmodule