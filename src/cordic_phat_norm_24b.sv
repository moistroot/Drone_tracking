// 分包3子模块3.2：CORDIC PHAT 归一化
`timescale 1ns/1ps
`include "project_global_param.vh"
module cordic_phat_norm_24b (
    input  wire         clk_50m,
    input  wire         rst_n,
    input  wire [DATA_WIDTH-1:0] re_in,      
    input  wire [DATA_WIDTH-1:0] im_in,     
    output reg  [DATA_WIDTH-1:0] re_out,    
    output reg  [DATA_WIDTH-1:0] im_out,   
    // 角度输出（测向用）
    output reg signed [25:0] angle_out,
    output reg               angle_valid     // 若为0 =，输入模长过小
);
    localparam integer N_STAGE = 20;
    localparam integer W = DATA_WIDTH + 4;        // 28 位
    localparam integer ZW = 26;                   // z 位宽
    // atan(2^-i) / 2π × 2^26
    localparam [ZW-1:0] ATAN_TAB [0:N_STAGE-1] = '{
        26'd8388608, 26'd4952084, 26'd2616545, 26'd1328199, 26'd666677,
        26'd333664,  26'd166872,  26'd83441,   26'd41721,   26'd20861,
        26'd10430,   26'd5215,    26'd2608,    26'd1304,    26'd652,
        26'd326,     26'd163,     26'd81,      26'd41,      26'd20
    };
    // 象限预处理
    wire signed [W-1:0] x_in_ext = $signed({{4{re_in[23]}}, re_in});
    wire signed [W-1:0] y_in_ext = $signed({{4{im_in[23]}}, im_in});
    reg  [41:0] flip_pipe;          // 与数据通路对齐：xr[20] 在第 T+42 拍有效
                                    // （xv[0] +1、段1 +20、zr[0] +1、段2 +20）
    // 段1：Vectoring（求 K·R 与 θ）
    reg signed [W-1:0]  xv [0:N_STAGE];
    reg signed [W-1:0]  yv [0:N_STAGE];
    reg signed [ZW-1:0] zv [0:N_STAGE];
    integer i;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 0; i <= N_STAGE; i = i + 1) begin
                xv[i] <= 0; yv[i] <= 0; zv[i] <= 0;
            end
        end else begin
            xv[0] <= re_in[23] ? -x_in_ext : x_in_ext;
            yv[0] <= re_in[23] ? -y_in_ext : y_in_ext;
            zv[0] <= 0;
            for (i = 0; i < N_STAGE; i = i + 1) begin
                if (yv[i] < 0) begin
                    xv[i+1] <= xv[i] - (yv[i] >>> i);
                    yv[i+1] <= yv[i] + (xv[i] >>> i);
                    zv[i+1] <= zv[i] - $signed({{(ZW-26){ATAN_TAB[i][25]}}, ATAN_TAB[i]});
                end else begin
                    xv[i+1] <= xv[i] + (yv[i] >>> i);
                    yv[i+1] <= yv[i] - (xv[i] >>> i);
                    zv[i+1] <= zv[i] + $signed({{(ZW-26){ATAN_TAB[i][25]}}, ATAN_TAB[i]});
                end
            end
        end
    end
    localparam signed [W-1:0] SMALL_LIM = 28'sd165;   // = R_MIN(100) × K
    wire vec_small = (xv[N_STAGE] < SMALL_LIM) && (xv[N_STAGE] > -SMALL_LIM);
    reg [20:0] vs_pipe;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) vs_pipe <= 21'b0;
        else        vs_pipe <= {vs_pipe[19:0], vec_small};
    end
    wire vec_small_al = vs_pipe[20];      // 第 T+42 拍有效，与 xr[20]/re_fin 对齐
    // 段2：Rotation（单位向量）
    localparam signed [W-1:0] X0 = 28'sd2547003;   // 2^22/K
    reg signed [W-1:0]  xr [0:N_STAGE];
    reg signed [W-1:0]  yr [0:N_STAGE];
    reg signed [ZW-1:0] zr [0:N_STAGE];
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 0; i <= N_STAGE; i = i + 1) begin
                xr[i] <= 0; yr[i] <= 0; zr[i] <= 0;
            end
        end else begin
            xr[0] <= X0;
            yr[0] <= 0;
            zr[0] <= zv[N_STAGE];
            for (i = 0; i < N_STAGE; i = i + 1) begin
                if (zr[i] >= 0) begin
                    xr[i+1] <= xr[i] - (yr[i] >>> i);
                    yr[i+1] <= yr[i] + (xr[i] >>> i);
                    zr[i+1] <= zr[i] - $signed({{(ZW-26){ATAN_TAB[i][25]}}, ATAN_TAB[i]});
                end else begin
                    xr[i+1] <= xr[i] + (yr[i] >>> i);
                    yr[i+1] <= yr[i] - (xr[i] >>> i);
                    zr[i+1] <= zr[i] + $signed({{(ZW-26){ATAN_TAB[i][25]}}, ATAN_TAB[i]});
                end
            end
        end
    end
    // 输出：截位 + 小值保护
    wire signed [W-1:0] re_fin = flip_pipe[41] ? -xr[N_STAGE] : xr[N_STAGE];
    wire signed [W-1:0] im_fin = flip_pipe[41] ? -yr[N_STAGE] : yr[N_STAGE];
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            re_out <= {DATA_WIDTH{1'b0}};
            im_out <= {DATA_WIDTH{1'b0}};
        end else if (vec_small_al) begin
            re_out <= {DATA_WIDTH{1'b0}};
            im_out <= {DATA_WIDTH{1'b0}};
        end else begin
            re_out <= re_fin[DATA_WIDTH-1:0];
            im_out <= im_fin[DATA_WIDTH-1:0];
        end
    end
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)
            flip_pipe <= 42'b0;
        else
            flip_pipe <= {flip_pipe[40:0], re_in[23]};
    end
    // 角度输出（测向使用）
    wire signed [ZW-1:0] angle_corr = flip_pipe[20] ?
                                      (zv[N_STAGE] + 26'sd33554432) : zv[N_STAGE];
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            angle_out   <= 26'sd0;
            angle_valid <= 1'b0;
        end else begin
            angle_out   <= angle_corr;
            angle_valid <= ~vec_small;      // 与上面同拍寄存，保持对齐
        end
    end
endmodule
