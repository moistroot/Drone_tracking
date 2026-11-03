`timescale 1ns/1ps
`include "project_global_param.vh"

module tb_clk_gen;

    reg clk_50m = 0;
    reg rst_n   = 0;

    wire bclk;
    wire lrck;
    wire sample_pulse;
    wire tick_1ms;

    clk_gen u_clk_gen (
        .clk_50m      (clk_50m),
        .rst_n        (rst_n),
        .bclk         (bclk),
        .lrck         (lrck),
        .sample_pulse (sample_pulse),
        .tick_1ms     (tick_1ms)
    );

    always #10 clk_50m = ~clk_50m;

    integer sample_cnt    = 0;
    integer bclk_rise_cnt = 0;
    integer tick_cnt      = 0;
    reg     bclk_d        = 0;

    always @(posedge clk_50m) begin
        bclk_d <= bclk;
        if (sample_pulse)    sample_cnt    = sample_cnt + 1;
        if (bclk && !bclk_d) bclk_rise_cnt = bclk_rise_cnt + 1;
        if (tick_1ms)        tick_cnt      = tick_cnt + 1;
    end

    initial begin
        rst_n = 0;
        #200;
        rst_n = 1;

        repeat (50_000_000) @(posedge clk_50m);

        $display("========================================");
        $display("clk_gen 1s simulation result");
        $display("CUR_SAMPLE_FREQ = %0d Hz", CUR_SAMPLE_FREQ);
        $display("sample_pulse count = %0d (expected 10000 +/-1)", sample_cnt);
        $display("bclk rise count    = %0d (expected 640000)", bclk_rise_cnt);
        $display("tick_1ms count     = %0d (expected 1000)", tick_cnt);
        $display("BCLK avg period    = %f ns (expected 1562.5 ns)",
                 1_000_000_000.0 / bclk_rise_cnt);
        $display("========================================");

        $finish;
    end

endmodule