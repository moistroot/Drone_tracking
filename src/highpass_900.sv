// 单通道 4 阶 Butterworth 高通（fc = 900 Hz @ fs = 10 kHz）+ 固定增益
`timescale 1ns/1ps
`include "project_global_param.vh"
module highpass_900 #(
    parameter integer GAIN_SHIFT = 4,   // 高通后固定增益 2^GAIN_SHIFT
    parameter integer ENABLE     = 1    // 若为0 ，则直通（FAST_SIM 用）
)(
    input  wire                          clk_50m,
    input  wire                          rst_n,
    input  wire signed [DATA_WIDTH-1:0]  din,
    input  wire                          din_valid,
    output wire signed [DATA_WIDTH-1:0]  dout,
    output wire                          dout_valid
);
    // 定点格式
    localparam integer COEF_FRAC = 17;             
    localparam integer COEF_W    = 20;             
    localparam integer MID_W     = 26;             
    localparam integer ACC_W     = 48;             
    localparam integer PROD_W    = COEF_W + MID_W;  
    localparam signed [COEF_W-1:0] S1_B0 =  20'sd80847;
    localparam signed [COEF_W-1:0] S1_B1 = -20'sd161695;
    localparam signed [COEF_W-1:0] S1_B2 =  20'sd80847;
    localparam signed [COEF_W-1:0] S1_A1 = -20'sd148047;
    localparam signed [COEF_W-1:0] S1_A2 =  20'sd44271;
    localparam signed [COEF_W-1:0] S2_B0 =  20'sd100303;
    localparam signed [COEF_W-1:0] S2_B1 = -20'sd200605;
    localparam signed [COEF_W-1:0] S2_B2 =  20'sd100303;
    localparam signed [COEF_W-1:0] S2_A1 = -20'sd183673;
    localparam signed [COEF_W-1:0] S2_A2 =  20'sd86465;
    // 舍入与饱和
    localparam signed [ACC_W-1:0] RND_HALF = 48'sd65536;      // 2^(COEF_FRAC-1)
    function automatic signed [ACC_W-1:0] rnd_q17(input signed [ACC_W-1:0] acc);
        if (acc < 0)
            rnd_q17 = -((-acc + RND_HALF) >>> COEF_FRAC);
        else
            rnd_q17 =  ( (acc  + RND_HALF) >>> COEF_FRAC);
    endfunction
    // 26 位饱和（-2^25 ~ 2^25-1）
    function automatic signed [MID_W-1:0] sat_mid(input signed [ACC_W-1:0] v);
        if      (v > 26'sd33554431) sat_mid = 26'sh1FFFFFF;
        else if (v < 26'sh2000000)  sat_mid = 26'sh2000000;   // -2^25
        else                        sat_mid = v[MID_W-1:0];
    endfunction
    // 24 位饱和（-2^23 ~ 2^23-1）
    function automatic signed [DATA_WIDTH-1:0] sat_out(input signed [ACC_W-1:0] v);
        if      (v > 24'sd8388607)  sat_out = 24'sh7FFFFF;
        else if (v < 24'sh800000)   sat_out = 24'sh800000;    // -2^23
        else                        sat_out = v[DATA_WIDTH-1:0];
    endfunction
    // 乘积符号扩展到累加器宽度
    function automatic signed [ACC_W-1:0] sext_prod(input signed [PROD_W-1:0] p);
        sext_prod = {{(ACC_W-PROD_W){p[PROD_W-1]}}, p};
    endfunction
    // 状态寄存器
    reg signed [DATA_WIDTH-1:0] s1_x1, s1_x2;   // 第1级输入历史
    reg signed [MID_W-1:0]      s1_y1, s1_y2;   // 第1级输出历史
    reg signed [MID_W-1:0]      s2_x;           // 第2级当前输入
    reg signed [MID_W-1:0]      s2_x1, s2_x2;   // 第2级输入历史
    reg signed [MID_W-1:0]      s2_y1, s2_y2;   // 第2级输出历史
    reg signed [ACC_W-1:0] acc1_r, acc2_r;
    reg signed [DATA_WIDTH-1:0] dout_r;
    reg                         dout_valid_r;
    reg                         v1, v2, v3;
    // 第 1 级累加
    wire signed [PROD_W-1:0] p1_b0 = S1_B0 * din;
    wire signed [PROD_W-1:0] p1_b1 = S1_B1 * s1_x1;
    wire signed [PROD_W-1:0] p1_b2 = S1_B2 * s1_x2;
    wire signed [PROD_W-1:0] p1_a1 = S1_A1 * s1_y1;
    wire signed [PROD_W-1:0] p1_a2 = S1_A2 * s1_y2;
    wire signed [ACC_W-1:0] acc1 = sext_prod(p1_b0) + sext_prod(p1_b1) + sext_prod(p1_b2)
                                 - sext_prod(p1_a1) - sext_prod(p1_a2);
    wire signed [ACC_W-1:0] y1_full = rnd_q17(acc1_r);
    wire signed [MID_W-1:0] y1_new  = sat_mid(y1_full);
    // 第 2 级累加
    wire signed [PROD_W-1:0] p2_b0 = S2_B0 * s2_x;
    wire signed [PROD_W-1:0] p2_b1 = S2_B1 * s2_x1;
    wire signed [PROD_W-1:0] p2_b2 = S2_B2 * s2_x2;
    wire signed [PROD_W-1:0] p2_a1 = S2_A1 * s2_y1;
    wire signed [PROD_W-1:0] p2_a2 = S2_A2 * s2_y2;
    wire signed [ACC_W-1:0] acc2 = sext_prod(p2_b0) + sext_prod(p2_b1) + sext_prod(p2_b2)
                                 - sext_prod(p2_a1) - sext_prod(p2_a2);
    wire signed [ACC_W-1:0] y2_full = rnd_q17(acc2_r);
    wire signed [MID_W-1:0] y2_new  = sat_mid(y2_full);
    // 末级增益 + 饱和
    wire signed [ACC_W-1:0] y2_ext   = {{(ACC_W-MID_W){y2_new[MID_W-1]}}, y2_new};
    wire signed [ACC_W-1:0] y2_scale = y2_ext <<< GAIN_SHIFT;
    wire signed [DATA_WIDTH-1:0] dout_new = sat_out(y2_scale);
    // 流水线
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            s1_x1 <= {DATA_WIDTH{1'b0}};
            s1_x2 <= {DATA_WIDTH{1'b0}};
            s1_y1 <= {MID_W{1'b0}};
            s1_y2 <= {MID_W{1'b0}};
            s2_x  <= {MID_W{1'b0}};
            s2_x1 <= {MID_W{1'b0}};
            s2_x2 <= {MID_W{1'b0}};
            s2_y1 <= {MID_W{1'b0}};
            s2_y2 <= {MID_W{1'b0}};
            acc1_r <= {ACC_W{1'b0}};
            acc2_r <= {ACC_W{1'b0}};
            dout_r <= {DATA_WIDTH{1'b0}};
            dout_valid_r <= 1'b0;
            v1 <= 1'b0;
            v2 <= 1'b0;
            v3 <= 1'b0;
        end else begin
            v1 <= din_valid;
            v2 <= v1;
            v3 <= v2;
            dout_valid_r <= v3;
            // 拍0：第一级累加 + 输入历史移位
            if (din_valid) begin
                acc1_r <= acc1;
                s1_x2  <= s1_x1;
                s1_x1  <= din;
            end
            // 拍1：第一级输出，同时把结果送进第二级的输入
            if (v1) begin
                s1_y2 <= s1_y1;
                s1_y1 <= y1_new;
                s2_x  <= y1_new;
            end
            // 拍2：第二级累加 + 输入历史移位
            if (v2) begin
                acc2_r <= acc2;
                s2_x2  <= s2_x1;
                s2_x1  <= s2_x;
            end
            // 拍3：第二级输出 + 增益饱和
            if (v3) begin
                s2_y2  <= s2_y1;
                s2_y1  <= y2_new;
                dout_r <= dout_new;
            end
        end
    end
    // 旁路
    assign dout       = ENABLE ? dout_r       : din;
    assign dout_valid = ENABLE ? dout_valid_r : din_valid;
endmodule
