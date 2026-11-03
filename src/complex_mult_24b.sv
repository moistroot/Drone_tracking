// 分包3子模块3.1：24位复数共轭乘法（4个DSP48E1）
`timescale 1ns/1ps
`include "project_global_param.vh"
module complex_mult_24b (
    input  wire         clk_50m,
    input  wire         rst_n,
    input  wire [DATA_WIDTH-1:0] a_re, a_im,
    input  wire [DATA_WIDTH-1:0] b_re, b_im,
    output reg  [DATA_WIDTH-1:0] out_re, out_im
);
    // S1: 乘法
    reg signed [47:0] m1, m2, m3, m4;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            m1 <= 0; m2 <= 0; m3 <= 0; m4 <= 0;
        end else begin
            m1 <= $signed(a_re) * $signed(b_re);
            m2 <= $signed(a_im) * $signed(b_im);
            m3 <= $signed(a_im) * $signed(b_re);
            m4 <= $signed(a_re) * $signed(b_im);
        end
    end
    // S2: 加减
    reg signed [47:0] re_full, im_full;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            re_full <= 0; im_full <= 0;
        end else begin
            re_full <= m1 + m2;   // a_re*b_re + a_im*b_im
            im_full <= m3 - m4;   // a_im*b_re - a_re*b_im
        end
    end
    // S3: 截位 + 对称饱和
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            out_re <= {DATA_WIDTH{1'b0}};
            out_im <= {DATA_WIDTH{1'b0}};
        end else begin
            out_re <= sat(re_full);
            out_im <= sat(im_full);
        end
    end
    function automatic [DATA_WIDTH-1:0] sat(input signed [47:0] v);
        if (v > 48'sd8388607 <<< 24)
            sat = 24'h7FFFFF;
        else if (v < -(48'sd8388608 <<< 24))
            sat = 24'h800000;
        else
            sat = v >>> 24;
    endfunction
endmodule
