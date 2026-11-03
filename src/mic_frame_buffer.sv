// 4 路独立 Ping-Pong 帧缓存 + 4 路并行读接口
`timescale 1ns/1ps
`include "project_global_param.vh"
module mic_frame_buffer (
    input  wire                       clk_50m,
    input  wire                       rst_n,
    input  wire [DATA_WIDTH-1:0]      ch_data [0:MAX_CHAN_NUM-1],
    input  wire [MAX_CHAN_NUM-1:0]    ch_valid,
    input  wire                       rd_en,
    input  wire [FFT_IDX_WIDTH-1:0]   rd_addr,
    output reg  [DATA_WIDTH-1:0]      rd_data [0:MAX_CHAN_NUM-1],
    output reg                        frame_valid,
    output reg  [FRAME_CNT_WIDTH-1:0] frame_cnt,
    output reg                        overflow_error
);
    localparam integer PT_W = $clog2(CUR_FFT_POINT);   // 8
    reg [PT_W-1:0]            wr_addr;
    reg                       buf_sel;
    reg [FRAME_CNT_WIDTH-1:0] frame_cnt_reg;
    wire write_en   = ch_valid[0];
    wire frame_full = (wr_addr == CUR_FFT_POINT - 1);
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            wr_addr        <= {PT_W{1'b0}};
            buf_sel        <= 1'b0;
            frame_cnt_reg  <= {FRAME_CNT_WIDTH{1'b0}};
            frame_valid    <= 1'b0;
            overflow_error <= 1'b0;
        end else begin
            frame_valid <= 1'b0;
            if (write_en) begin
                if (frame_full) begin
                    wr_addr       <= {PT_W{1'b0}};
                    buf_sel       <= ~buf_sel;
                    frame_cnt_reg <= frame_cnt_reg + 1'b1;
                    frame_valid   <= 1'b1;
                end else begin
                    wr_addr <= wr_addr + 1'b1;
                end
            end
        end
    end
    assign frame_cnt = frame_cnt_reg;
    generate
        for (genvar ch = 0; ch < MAX_CHAN_NUM; ch = ch + 1) begin: buf_gen
            (* ram_style = "block" *)
            reg [DATA_WIDTH-1:0] frame_buffer_0 [0:CUR_FFT_POINT-1];
            (* ram_style = "block" *)
            reg [DATA_WIDTH-1:0] frame_buffer_1 [0:CUR_FFT_POINT-1];
            always @(posedge clk_50m) begin
                if (write_en) begin
                    if (buf_sel)
                        frame_buffer_1[wr_addr] <= ch_data[ch];
                    else
                        frame_buffer_0[wr_addr] <= ch_data[ch];
                end
            end
            always @(posedge clk_50m) begin
                if (rd_en) begin
                    if (buf_sel)
                        rd_data[ch] <= frame_buffer_0[rd_addr];
                    else
                        rd_data[ch] <= frame_buffer_1[rd_addr];
                end
            end
        end
    endgenerate
endmodule
