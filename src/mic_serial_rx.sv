// 4 路 I2S 串行接收，24位移位，并行锁存
`timescale 1ns/1ps
`include "project_global_param.vh"
module mic_serial_rx (
    input  wire                    clk_50m,
    input  wire                    rst_n,
    input  wire                    bclk,
    input  wire                    lrck,
    input  wire [MAX_CHAN_NUM-1:0] mic_sdin,
    output reg  [DATA_WIDTH-1:0]   ch_data [0:MAX_CHAN_NUM-1],
    output reg  [MAX_CHAN_NUM-1:0] ch_valid
);
    localparam integer I2S_DELAY_BITS = 1;
    reg bclk_d;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) bclk_d <= 1'b0;
        else        bclk_d <= bclk;
    end
    wire bclk_rise = bclk && !bclk_d;
    reg lrck_d;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) lrck_d <= 1'b0;
        else        lrck_d <= lrck;
    end
    wire lrck_fall = !lrck && lrck_d;
    localparam integer BIT_CNT_LAST = I2S_DELAY_BITS + DATA_WIDTH - 1;
    reg [5:0] bit_cnt;
    reg       receiving;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            bit_cnt   <= 6'd0;
            receiving <= 1'b0;
        end else begin
            if (lrck_fall) begin
                bit_cnt   <= 6'd0;
                receiving <= 1'b1;
            end else if (receiving && bclk_rise) begin
                if (bit_cnt == BIT_CNT_LAST) begin
                    bit_cnt   <= 6'd0;
                    receiving <= 1'b0;
                end else begin
                    bit_cnt <= bit_cnt + 1'b1;
                end
            end
        end
    end
    wire bit_sample = receiving && bclk_rise &&
                      (bit_cnt >= I2S_DELAY_BITS) &&
                      (bit_cnt <  I2S_DELAY_BITS + DATA_WIDTH);
    wire bit_last   = receiving && bclk_rise && (bit_cnt == BIT_CNT_LAST);
    generate
        for (genvar ch = 0; ch < MAX_CHAN_NUM; ch = ch + 1) begin: mic_rx_gen
            reg [DATA_WIDTH-1:0] shift_reg;
            always @(posedge clk_50m or negedge rst_n) begin
                if (!rst_n) begin
                    shift_reg    <= {DATA_WIDTH{1'b0}};
                    ch_data[ch]  <= {DATA_WIDTH{1'b0}};
                    ch_valid[ch] <= 1'b0;
                end else begin
                    ch_valid[ch] <= 1'b0;
                    if (lrck_fall) begin
                        shift_reg <= {DATA_WIDTH{1'b0}};
                    end else if (bit_sample) begin
                        shift_reg <= {shift_reg[DATA_WIDTH-2:0], mic_sdin[ch]};
                    end
                    if (bit_last) begin
                        ch_data[ch]  <= {shift_reg[DATA_WIDTH-2:0], mic_sdin[ch]};
                        ch_valid[ch] <= 1'b1;
                    end
                end
            end
        end
    endgenerate
endmodule
