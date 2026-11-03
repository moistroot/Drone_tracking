// clk_gen.sv - 50MHz 主时钟 -> I2S BCLK / LRCLK / 采样脉冲 / 1ms tick
`timescale 1ns/1ps
`include "project_global_param.vh"
module clk_gen (
    input  wire clk_50m,       
    input  wire rst_n,          
    output reg  bclk,           // 位时钟，= 64 × CUR_SAMPLE_FREQ
    output reg  lrck,           
    output reg  sample_pulse,   // 采样同步脉冲，单周期，频率 = CUR_SAMPLE_FREQ
    output reg  tick_1ms        
);
    localparam integer SYS_CLK          = SYS_CLK_FREQ;
    localparam integer BCLK_FREQ        = 64 * CUR_SAMPLE_FREQ;
    localparam integer BCLK_TOGGLE_INC  = 2 * BCLK_FREQ;
    localparam integer BCLK_ACC_WIDTH   = $clog2(SYS_CLK + BCLK_TOGGLE_INC + 1);
    localparam integer TICK_DIV         = SYS_CLK_FREQ / 1000;
    localparam integer TICK_CNT_WIDTH   = $clog2(TICK_DIV);
    reg [1:0] rst_sync;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)
            rst_sync <= 2'b00;
        else
            rst_sync <= {rst_sync[0], 1'b1};
    end
    wire rst_n_sync = rst_sync[1];
    reg  [BCLK_ACC_WIDTH-1:0] bclk_acc;
    wire [BCLK_ACC_WIDTH-1:0] bclk_acc_next = bclk_acc + BCLK_TOGGLE_INC;
    wire                      bclk_toggle   = (bclk_acc_next >= SYS_CLK);
    wire [BCLK_ACC_WIDTH-1:0] bclk_acc_load = bclk_toggle ? (bclk_acc_next - SYS_CLK) : bclk_acc_next;
    wire                      bclk_next     = bclk_toggle ? ~bclk : bclk;
    wire                      bclk_fall     = bclk_toggle && (bclk == 1'b1);
    reg [5:0] bclk_cycle_cnt;
    reg [4:0] lrck_half_cnt;
    reg [TICK_CNT_WIDTH-1:0] tick_cnt;
    always @(posedge clk_50m or negedge rst_n_sync) begin
        if (!rst_n_sync) begin
            bclk_acc        <= 0;
            bclk            <= 1'b0;
            bclk_cycle_cnt  <= 6'd0;
            lrck_half_cnt   <= 5'd0;
            lrck            <= 1'b0;
            sample_pulse    <= 1'b0;
            tick_cnt        <= 0;
            tick_1ms        <= 1'b0;
        end else begin
            sample_pulse <= 1'b0;
            tick_1ms     <= 1'b0;
            bclk_acc <= bclk_acc_load;
            bclk     <= bclk_next;
            if (bclk_fall) begin
                if (lrck_half_cnt == 5'd31) begin
                    lrck_half_cnt <= 5'd0;
                    lrck          <= ~lrck;
                end else begin
                    lrck_half_cnt <= lrck_half_cnt + 1'b1;
                end
                if (bclk_cycle_cnt == 6'd63) begin
                    bclk_cycle_cnt <= 6'd0;
                    sample_pulse   <= 1'b1;
                end else begin
                    bclk_cycle_cnt <= bclk_cycle_cnt + 1'b1;
                end
            end
            if (tick_cnt == TICK_DIV - 1) begin
                tick_cnt <= 0;
                tick_1ms <= 1'b1;
            end else begin
                tick_cnt <= tick_cnt + 1'b1;
            end
        end
    end
endmodule
