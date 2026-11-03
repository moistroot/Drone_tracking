// phat_position_top.sv - 分包3子模块3.7：PHAT互相关与时延定位顶层
`timescale 1ns/1ps
`include "project_global_param.vh"
module phat_position_top (
    input  wire         clk_50m,
    input  wire         rst_n,
    // 频域数据输入（对接分包2，任务书接口）
    input  wire [FFT_IDX_WIDTH-1:0] fft_idx,
    input  wire [DATA_WIDTH-1:0] fft_re [0:MAX_CHAN_NUM-1],
    input  wire [DATA_WIDTH-1:0] fft_im [0:MAX_CHAN_NUM-1],
    input  wire [FFT_BINS-1:0] freq_valid_mask,
    input  wire [FRAME_CNT_WIDTH-1:0] pre_frame_cnt,
    input  wire         pre_frame_valid,
    input  wire         mask_learn_done,  
    input  wire [9:0]   fft_ip_latency,
    // 麦克风阵列配置
    input  wire [COORD_WIDTH-1:0] mic_coord_x [0:MAX_CHAN_NUM-1],
    input  wire [COORD_WIDTH-1:0] mic_coord_y [0:MAX_CHAN_NUM-1],
   
    output reg signed [15:0] target_azimuth_01deg,    // -1800..+1800
    output reg signed [15:0] target_elevation_01deg,  // 0..+900
    output reg  [TDOA_WIDTH-1:0] tdoa_result [0:MAX_PAIR_NUM-1],
    output reg          direction_valid,
    // 串口输出
    output wire         uart_tx
);
    // 内部连线
    wire [DATA_WIDTH-1:0] ifft_din_re, ifft_din_im;
    wire         ifft_din_valid, ifft_din_last;
    wire         ifft_din_ready;
    wire         ifft_cfg_valid;
    wire [TDOA_WIDTH-1:0] tdoa_peak;
    wire         tdoa_peak_valid;
    wire         tdoa_peak_used;
    wire [MAX_PAIR_NUM-1:0] sched_pair_used;
    reg  [TDOA_WIDTH-1:0] sched_tdoa [0:MAX_PAIR_NUM-1];
    wire         all_done;
    wire [FRAME_CNT_WIDTH-1:0] frame_cnt_out;
    wire         busy;
    wire signed [15:0] az01, el01;
    wire         dir_valid_raw;
    wire dir_valid_gated = dir_valid_raw && mask_learn_done;
    // 1. 配对调度
    pair_scheduler u_scheduler (
        .clk_50m         (clk_50m),
        .rst_n           (rst_n),
        .pre_frame_valid (pre_frame_valid),
        .fft_idx         (fft_idx),
        .fft_re          (fft_re),
        .fft_im          (fft_im),
        .freq_valid_mask (freq_valid_mask),
        .pre_frame_cnt   (pre_frame_cnt),
        .fft_ip_latency  (fft_ip_latency),
        .ifft_din_re     (ifft_din_re),
        .ifft_din_im     (ifft_din_im),
        .ifft_din_valid  (ifft_din_valid),
        .ifft_din_last   (ifft_din_last),
        .ifft_din_ready  (ifft_din_ready),
        .ifft_cfg_valid  (ifft_cfg_valid),
        .tdoa_in         (tdoa_peak),
        .tdoa_in_valid   (tdoa_peak_valid),
        .tdoa_in_used    (tdoa_peak_used),
        .tdoa_result     (sched_tdoa),
        .pair_used       (sched_pair_used),
        .all_done        (all_done),
        .frame_cnt_out   (frame_cnt_out),
        .busy            (busy)
    );
    // 2. IFFT + 峰值检测
    ifft_peak_detect u_peak (
        .clk_50m    (clk_50m),
        .rst_n      (rst_n),
        .din_re     (ifft_din_re),
        .din_im     (ifft_din_im),
        .din_valid  (ifft_din_valid),
        .din_last   (ifft_din_last),
        .din_ready  (ifft_din_ready),
        .din_cfg_valid (ifft_cfg_valid),
        .tdoa_out   (tdoa_peak),
        .tdoa_valid (tdoa_peak_valid),
        .tdoa_used  (tdoa_peak_used)
    );
    // 3. TDOA 测向解算
    tdoa_solver u_solver (
        .clk_50m                (clk_50m),
        .rst_n                  (rst_n),
        .tdoa_result            (sched_tdoa),
        .pair_used              (sched_pair_used),
        .all_done               (all_done),
        .mic_coord_x            (mic_coord_x),
        .mic_coord_y            (mic_coord_y),
        .pre_frame_cnt          (pre_frame_cnt),
        .mask_learn_done        (mask_learn_done),
        .target_azimuth_01deg   (az01),
        .target_elevation_01deg (el01),
        .direction_valid        (dir_valid_raw)
    );
    // 4. UART 输出
    localparam [6:0] UART_LAT = 7'd100;   // > 解算 81 拍
    reg  [6:0]        uart_dly;
    reg               uart_req;      // 单周期发帧脉冲
    reg               tx_valid;      // 锁存
    reg  signed [15:0] tx_az, tx_el; // 本帧角度（无效时0）
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            uart_dly <= 7'd0;
            uart_req <= 1'b0;
            tx_valid <= 1'b0;
            tx_az    <= 16'sd0;
            tx_el    <= 16'sd0;
        end else begin
            uart_req <= 1'b0;
            if (all_done) begin              // 新帧
                uart_dly <= UART_LAT;
                tx_valid <= 1'b0;
                tx_az    <= 16'sd0;
                tx_el    <= 16'sd0;
            end else if (uart_dly != 7'd0) begin
                uart_dly <= uart_dly - 7'd1;
                if (uart_dly == 7'd1)
                    uart_req <= 1'b1;        // 发帧
            end
            if (dir_valid_gated) begin       // 锁存
                tx_valid <= 1'b1;
                tx_az    <= az01;
                tx_el    <= el01;
            end
        end
    end
    uart_tx_module u_uart (
        .clk_50m              (clk_50m),
        .rst_n                (rst_n),
        .frame_valid          (uart_req),
        .frame_cnt            (frame_cnt_out),
        .tdoa_vals            (sched_tdoa),
        .pos_azimuth_01deg    (tx_az),
        .pos_elevation_01deg  (tx_el),
        .pos_valid            (tx_valid),
        .uart_tx              (uart_tx)
    );
    // 结果输出寄存器
    integer p;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            target_azimuth_01deg   <= 16'sd0;
            target_elevation_01deg <= 16'sd0;
            direction_valid        <= 1'b0;
            for (p = 0; p < MAX_PAIR_NUM; p = p + 1)
                tdoa_result[p] <= {TDOA_WIDTH{1'b0}};
        end else begin
            if (dir_valid_gated) begin
                target_azimuth_01deg   <= az01;
                target_elevation_01deg <= el01;
            end
            direction_valid <= dir_valid_gated;
            for (p = 0; p < MAX_PAIR_NUM; p = p + 1)
                tdoa_result[p] <= sched_tdoa[p];
        end
    end
endmodule
