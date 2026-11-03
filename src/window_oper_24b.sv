// window_oper_24b.sv - 分包2子模块2.2：24位加窗乘法（1个DSP48E1）
`timescale 1ns/1ps
`include "project_global_param.vh"
module window_oper_24b (
    input  wire                  clk_50m,
    input  wire                  rst_n,
    input  wire [DATA_WIDTH-1:0] data_in,     // 有符号采样数据
    input  wire [DATA_WIDTH-1:0] win_coef,    // 无符号窗系数
    output reg  [DATA_WIDTH-1:0] data_out
);
    wire signed [48:0] prod = $signed({1'b0, win_coef}) * $signed(data_in);
    wire signed [25:0] prod_hi = $signed(prod[46:23]);
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)
            data_out <= {DATA_WIDTH{1'b0}};
        else if (prod_hi > 25'sd8388607)
            data_out <= 24'h7FFFFF;
        else if (prod_hi < -25'sd8388608)
            data_out <= 24'h800000;
        else
            data_out <= prod_hi[23:0];
    end
endmodule
