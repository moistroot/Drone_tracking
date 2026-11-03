//四通道共享幂次增益 AGC（帧缓存读侧）
`timescale 1ns/1ps
`include "project_global_param.vh"
module agc_shared_24b #(
    parameter integer ENABLE     = 1,    //弱为0 ，则直通（单元测试旁路）
    parameter integer TARGET_BIT = 22,   // 输出峰值目标最高位
    parameter integer GMAX       = 8     // 最大增益指数（≤15）
)(
    input  wire                          clk_50m,
    input  wire                          rst_n,
    // 写侧统计（HPF 输出，未加 AGC 增益）
    input  wire                          sample_valid, 
    input  wire [DATA_WIDTH-1:0]         samp [0:MAX_CHAN_NUM-1],
    input  wire                          frame_done,   
    // 读侧数据（帧缓存 4 路并行读输出）
    input  wire [DATA_WIDTH-1:0]         din [0:MAX_CHAN_NUM-1],
    output wire [DATA_WIDTH-1:0]         dout [0:MAX_CHAN_NUM-1],
    // 状态
    output reg  [3:0]                    gain,
    output reg                           gain_changed
);
    // 帧峰值统计
    wire [DATA_WIDTH-1:0] mag [0:MAX_CHAN_NUM-1];
    generate
        for (genvar ch = 0; ch < MAX_CHAN_NUM; ch = ch + 1) begin: mag_gen
            assign mag[ch] = samp[ch][DATA_WIDTH-1]
                           ? (~samp[ch] + {{(DATA_WIDTH-1){1'b0}}, 1'b1})
                           : samp[ch];
        end
    endgenerate
    // 通道间最大值（链式比较，4 路）
    wire [DATA_WIDTH-1:0] mag_max [0:MAX_CHAN_NUM-1];
    assign mag_max[0] = mag[0];
    generate
        for (genvar gi = 1; gi < MAX_CHAN_NUM; gi = gi + 1) begin: max_gen
            assign mag_max[gi] = (mag[gi] > mag_max[gi-1]) ? mag[gi] : mag_max[gi-1];
        end
    endgenerate
    wire [DATA_WIDTH-1:0] samp_max = mag_max[MAX_CHAN_NUM-1];
    function automatic [4:0] lead_bit(input [DATA_WIDTH-1:0] v);
        integer i;
        begin
            lead_bit = 5'd0;
            for (i = 0; i < DATA_WIDTH; i = i + 1)
                if (v[i]) lead_bit = i[4:0];
        end
    endfunction
    reg [DATA_WIDTH-1:0] frame_peak;
    // 目标挡位与迟滞判决（组合，帧结束时用当前 frame_peak）
    wire [4:0] lead_peak = lead_bit(frame_peak);
    wire [5:0] want_gain = (TARGET_BIT > lead_peak)
                         ? (TARGET_BIT - lead_peak) : 6'd0;
    wire [3:0] gain_des  = (want_gain > GMAX) ? GMAX[3:0] : want_gain[3:0];
    wire [5:0] out_lead  = {1'b0, lead_peak} + {2'b0, gain};
    wire       can_raise = (out_lead < (TARGET_BIT - 1));
    wire [3:0] gain_next = (gain_des < gain) ? gain_des :
                           ((gain_des > gain) && can_raise) ? gain_des : gain;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            frame_peak   <= {DATA_WIDTH{1'b0}};
            gain         <= 4'd0;
            gain_changed <= 1'b0;
        end else begin
            gain_changed <= 1'b0;
            if (sample_valid)
                frame_peak <= (samp_max > frame_peak) ? samp_max : frame_peak;
            if (ENABLE != 0 && frame_done) begin
                frame_peak <= {DATA_WIDTH{1'b0}};
                if (gain_next != gain) begin
                    gain         <= gain_next;
                    gain_changed <= 1'b1;
                end
            end
        end
    end
    // 读侧增益（4 路组合，保持 1 拍读延迟）
    function automatic [DATA_WIDTH-1:0] sat24(input signed [32:0] v);
        begin
            if (v > 33'sd8388607)       sat24 = 24'sh7FFFFF;
            else if (v < -33'sd8388608) sat24 = 24'sh800000;
            else                        sat24 = v[DATA_WIDTH-1:0];
        end
    endfunction
    generate
        for (genvar gc = 0; gc < MAX_CHAN_NUM; gc = gc + 1) begin: gain_gen
            wire signed [32:0] din_ext  = {{(33-DATA_WIDTH){din[gc][DATA_WIDTH-1]}},
                                            din[gc]};
            wire signed [32:0] din_gain = din_ext <<< gain;
            assign dout[gc] = (ENABLE != 0) ? sat24(din_gain) : din[gc];
        end
    endgenerate
endmodule
