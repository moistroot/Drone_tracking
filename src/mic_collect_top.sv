// 分包1顶层：4路I2S麦克风同步采集 + 帧缓存 + 读接口
`timescale 1ns/1ps
`include "project_global_param.vh"
module mic_collect_top #(
    parameter integer HPF_BYPASS = 0
)(
    input  wire         clk_50m,
    input  wire         rst_n,
    input  wire [MAX_CHAN_NUM-1:0] mic_pdm_sdin,
    output wire         mic_bclk,
    output wire         mic_lrck,
    output wire [3:0]   board_led,
    output reg  [DATA_WIDTH-1:0] mic_ch_data [0:MAX_CHAN_NUM-1],
    output reg  [FRAME_CNT_WIDTH-1:0]  mic_frame_cnt,
    output reg          mic_frame_valid,
    input  wire                        mic_rd_en,
    input  wire [FFT_IDX_WIDTH-1:0]    mic_rd_addr,
    output wire [DATA_WIDTH-1:0]       mic_rd_data [0:MAX_CHAN_NUM-1],
    // （1 拍） 频域噪声掩码重新学习
    output wire                        agc_gain_changed
);
    wire         bclk_int;
    wire         lrck_int;
    wire         sample_pulse;
    wire         tick_1ms;
    wire [DATA_WIDTH-1:0] ch_data [0:MAX_CHAN_NUM-1];
    wire [MAX_CHAN_NUM-1:0] ch_valid;
    // 高通之后的样本：写帧缓存与 mic_ch_data 
    wire [DATA_WIDTH-1:0] ch_data_hp [0:MAX_CHAN_NUM-1];
    wire [MAX_CHAN_NUM-1:0] ch_valid_hp;
    wire         frame_valid_int;
    wire [FRAME_CNT_WIDTH-1:0] frame_cnt_int;
    wire         overflow_error;
    clk_gen u_clk_gen (
        .clk_50m      (clk_50m),
        .rst_n        (rst_n),
        .bclk         (bclk_int),
        .lrck         (lrck_int),
        .sample_pulse (sample_pulse),
        .tick_1ms     (tick_1ms)
    );
    assign mic_bclk = bclk_int;
    assign mic_lrck = lrck_int;
    mic_serial_rx u_mic_serial_rx (
        .clk_50m  (clk_50m),
        .rst_n    (rst_n),
        .bclk     (bclk_int),
        .lrck     (lrck_int),
        .mic_sdin (mic_pdm_sdin),
        .ch_data  (ch_data),
        .ch_valid (ch_valid)
    );
    localparam integer HPF_EN  = HPF_BYPASS ? 0 : HPF_ENABLE;
   
    localparam integer AGC_EN  = HPF_BYPASS ? 0 : AGC_ENABLE;
    localparam integer HPF_GAIN_USE = (AGC_EN != 0) ? 0 : HPF_GAIN_SHIFT;
    generate
        for (genvar ch = 0; ch < MAX_CHAN_NUM; ch = ch + 1) begin: hpf_gen
            highpass_900 #(
                .GAIN_SHIFT (HPF_GAIN_USE),
                .ENABLE     (HPF_EN)
            ) u_hpf (
                .clk_50m    (clk_50m),
                .rst_n      (rst_n),
                .din        (ch_data[ch]),
                .din_valid  (ch_valid[ch]),
                .dout       (ch_data_hp[ch]),
                .dout_valid (ch_valid_hp[ch])
            );
        end
    endgenerate
    wire [DATA_WIDTH-1:0] mic_rd_data_raw [0:MAX_CHAN_NUM-1];
    mic_frame_buffer u_mic_frame_buffer (
        .clk_50m        (clk_50m),
        .rst_n          (rst_n),
        .ch_data        (ch_data_hp),
        .ch_valid       (ch_valid_hp),
        .rd_en          (mic_rd_en),
        .rd_addr        (mic_rd_addr),
        .rd_data        (mic_rd_data_raw),
        .frame_valid    (frame_valid_int),
        .frame_cnt      (frame_cnt_int),
        .overflow_error (overflow_error)
    );
    wire [3:0] agc_gain_dbg;
    agc_shared_24b #(
        .ENABLE     (AGC_EN),
        .TARGET_BIT (AGC_TARGET_BIT),
        .GMAX       (AGC_GMAX)
    ) u_agc (
        .clk_50m     (clk_50m),
        .rst_n       (rst_n),
        .sample_valid(ch_valid_hp[0]),
        .samp        (ch_data_hp),
        .frame_done  (frame_valid_int),
        .din         (mic_rd_data_raw),
        .dout        (mic_rd_data),
        .gain        (agc_gain_dbg),
        .gain_changed(agc_gain_changed)
    );
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            mic_ch_data     <= '{default: {DATA_WIDTH{1'b0}}};
            mic_frame_cnt   <= {FRAME_CNT_WIDTH{1'b0}};
            mic_frame_valid <= 1'b0;
        end else begin
            // 调试总线保持 AGC 之前的 HPF 输出（固定电平，便于逐点比对）
            mic_ch_data     <= ch_data_hp;
            mic_frame_cnt   <= frame_cnt_int;
            mic_frame_valid <= frame_valid_int;
        end
    end
    localparam integer LED_HALF_PERIOD = SYS_CLK_FREQ / 2;
    reg [25:0] led_cnt;
    reg        led0_reg;
    reg        led1_reg;
    reg        led2_reg;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            led_cnt  <= 26'd0;
            led0_reg <= 1'b0;
            led1_reg <= 1'b0;
            led2_reg <= 1'b0;
        end else begin
            if (led_cnt == LED_HALF_PERIOD[25:0]) begin
                led0_reg <= ~led0_reg;
                led_cnt  <= 26'd0;
            end else begin
                led_cnt <= led_cnt + 1'b1;
            end
            if (frame_valid_int)
                led1_reg <= ~led1_reg;
            if (frame_valid_int && (frame_cnt_int == {FRAME_CNT_WIDTH{1'b0}}))
                led2_reg <= ~led2_reg;
        end
    end
    assign board_led[0] = led0_reg;
    assign board_led[1] = led1_reg;
    assign board_led[2] = led2_reg;
    assign board_led[3] = 1'b1;
endmodule
