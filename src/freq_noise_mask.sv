//分包2子模块2.5：自适应频点掩码
`timescale 1ns/1ps
`include "project_global_param.vh"
module freq_noise_mask (
    input  wire         clk_50m,
    input  wire         rst_n,
    input  wire         valid,        
    input  wire [FFT_IDX_WIDTH-1:0] idx,
    input  wire [DATA_WIDTH-1:0] re,    
    input  wire [DATA_WIDTH-1:0] im,   
    input  wire         frame_end,      
    input  wire         restart,        // AGC 换挡：重新学习噪声基底
    output reg  [FFT_BINS-1:0] freq_valid_mask,
    output reg          learn_done      
);
    localparam integer LEARN_FRAMES = 3;
    localparam [FFT_IDX_WIDTH-1:0] POINTS_HALF = CUR_FFT_POINT/2;
    reg [25:0] accum [0:FFT_BINS-1];            // 26 位：3 帧 × 24 位能量
    
    wire signed [47:0] re_sq = $signed(re) * $signed(re);
    wire signed [47:0] im_sq = $signed(im) * $signed(im);
    wire [23:0] energy = re_sq[46:23] + im_sq[46:23];
   
    wire [26:0] energy_x8 = {energy, 3'b000};                   
    wire pass = (energy_x8 > {1'b0, accum[idx]});             
   
    reg [1:0]  learn_cnt;                      
    reg        learn_first;                    
    integer k;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            learn_cnt        <= 0;
            learn_first      <= 1'b1;
            learn_done       <= 1'b0;
            freq_valid_mask  <= {FFT_BINS{1'b0}};
            for (k = 0; k < FFT_BINS; k = k + 1)
                accum[k] <= 26'b0;
        end else begin
            if (restart) begin
                learn_cnt       <= 2'd0;
                learn_first     <= 1'b1;
                learn_done      <= 1'b0;
                freq_valid_mask <= {FFT_BINS{1'b0}};
            end
            if (valid && (idx <= POINTS_HALF)) begin
                if (learn_cnt < LEARN_FRAMES) begin
                   
                    accum[idx] <= learn_first ? {{2{1'b0}}, energy}
                                              : accum[idx] + {{2{1'b0}}, energy};
                end else begin
                    freq_valid_mask[idx] <= pass && (idx >= MASK_BIN_LO) && (idx <= MASK_BIN_HI);
                end
            end
            if (frame_end) begin
                if (learn_cnt < LEARN_FRAMES) begin
                    learn_cnt <= learn_cnt + 1'b1;
                    if (learn_cnt == 2'd0)
                        learn_first <= 1'b0;
                    if (learn_cnt == LEARN_FRAMES - 1)
                        learn_done <= 1'b1;
                end else begin
                    learn_cnt <= LEARN_FRAMES;   // 饱和
                end
            end
        end
    end
endmodule
