// 分包2子模块2.1：汉宁窗系数ROM（BRAM）
`timescale 1ns/1ps
`include "project_global_param.vh"
module hanning_rom_24b (
    input  wire                  clk_50m,
    input  wire [FFT_IDX_WIDTH-1:0] addr,   // 0 ~ CUR_FFT_POINT-1
    output reg  [DATA_WIDTH-1:0] dout
);
    (* rom_style = "block" *) reg [DATA_WIDTH-1:0] rom [0:CUR_FFT_POINT-1];
    initial $readmemh("hanning_256.mem", rom);
    always @(posedge clk_50m)
        dout <= rom[addr];
endmodule
