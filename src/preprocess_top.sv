// 分包2子模块2.6：4 路并行加窗 FFT 与频域预处理顶层
`timescale 1ns/1ps
`include "project_global_param.vh"
module preprocess_top (
    input  wire         clk_50m,
    input  wire         rst_n,
   
    input  wire [DATA_WIDTH-1:0] mic_ch_data [0:MAX_CHAN_NUM-1],
    input  wire [FRAME_CNT_WIDTH-1:0] mic_frame_cnt,
    input  wire         mic_frame_valid,
    // 帧缓存 4 路并行读接口（对接 mic_collect_top）
    output wire         rd_en,
    output wire [FFT_IDX_WIDTH-1:0] rd_addr,
    input  wire [DATA_WIDTH-1:0] rd_data [0:MAX_CHAN_NUM-1],
    // 前端 AGC 换挡（重新学习噪声基底）
    input  wire         agc_gain_changed,
    // 频域帧输出（对接分包3，任务书接口）
    output reg  [FFT_IDX_WIDTH-1:0] fft_idx,
    output reg  [DATA_WIDTH-1:0] fft_re [0:MAX_CHAN_NUM-1],
    output reg  [DATA_WIDTH-1:0] fft_im [0:MAX_CHAN_NUM-1],
    output reg  [FFT_BINS-1:0] freq_valid_mask,
    output reg  [FRAME_CNT_WIDTH-1:0] pre_frame_cnt,
    output reg          pre_frame_valid,
    output wire         mask_learn_done,   // 掩码学习完成
   
    output wire [9:0]   fft_ip_latency
);
    assign fft_ip_latency = FFT_IP_LATENCY[9:0];
    // 1. 汉宁窗 ROM（4 路共用，地址相同）
    wire [DATA_WIDTH-1:0] win_coef;
    hanning_rom_24b u_hanning (
        .clk_50m (clk_50m),
        .addr    (rd_addr),
        .dout    (win_coef)
    );
    // 2. 4 路并行 FFT 调度（内含 4×window_oper/4×FFT IP）
    wire         mask_valid;
    wire [FFT_IDX_WIDTH-1:0] mask_idx;
    wire [DATA_WIDTH-1:0]    mask_re, mask_im;
    wire         mask_frame_end;
    wire         frame_sync_err;
    fft_parallel_scheduler u_scheduler (
        .clk_50m        (clk_50m),
        .rst_n          (rst_n),
        .mic_frame_cnt  (mic_frame_cnt),
        .mic_frame_valid(mic_frame_valid),
        .rd_en          (rd_en),
        .rd_addr        (rd_addr),
        .rd_data        (rd_data),
        .win_coef       (win_coef),
        .fft_idx        (fft_idx),
        .fft_re         (fft_re),
        .fft_im         (fft_im),
        .pre_frame_cnt  (pre_frame_cnt),
        .pre_frame_valid(pre_frame_valid),
        .mask_valid     (mask_valid),
        .mask_idx       (mask_idx),
        .mask_re        (mask_re),
        .mask_im        (mask_im),
        .mask_frame_end (mask_frame_end),
        .frame_sync_err (frame_sync_err)
    );
    // 3. 噪声掩码
    freq_noise_mask u_mask (
        .clk_50m         (clk_50m),
        .rst_n           (rst_n),
        .valid           (mask_valid),
        .idx             (mask_idx),
        .re              (mask_re),
        .im              (mask_im),
        .frame_end       (mask_frame_end),
        .restart         (agc_gain_changed),
        .freq_valid_mask (freq_valid_mask),
        .learn_done      (mask_learn_done)
    );
endmodule
