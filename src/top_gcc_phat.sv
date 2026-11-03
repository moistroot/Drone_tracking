// 系统顶层
`timescale 1ns/1ps
`include "project_global_param.vh"
module top_gcc_phat (
    input  wire         clk_50m,        
    input  wire         rst_n,         
    input  wire [3:0]   mic_pdm_sdin,  
    output wire         mic_bclk,      
    output wire         mic_lrck,       
    output wire [3:0]   board_led,     
    output wire         uart_tx,       
    output wire         seg_sh_cp,     
    output wire         seg_st_cp,      
    output wire         seg_ds,      
    output wire         servo_pan_pwm,  
    output wire         servo_tilt_pwm 
);
   
    wire [DATA_WIDTH-1:0] mic_ch_data [0:MAX_CHAN_NUM-1];
    wire [FRAME_CNT_WIDTH-1:0] mic_frame_cnt;
    wire mic_frame_valid;
    wire mic_rd_en;
    wire [FFT_IDX_WIDTH-1:0]   mic_rd_addr;
    wire [DATA_WIDTH-1:0]      mic_rd_data [0:MAX_CHAN_NUM-1]; 
    wire mic_agc_gain_changed;                  
    
    wire [FFT_IDX_WIDTH-1:0]  fft_idx;
    wire [DATA_WIDTH-1:0]     fft_re [0:MAX_CHAN_NUM-1];
    wire [DATA_WIDTH-1:0]     fft_im [0:MAX_CHAN_NUM-1];
    wire [FFT_BINS-1:0]  freq_valid_mask;  
    wire [FRAME_CNT_WIDTH-1:0] pre_frame_cnt;
    wire pre_frame_valid;
    wire mask_learn_done;                        
    wire [9:0] fft_ip_latency;                  
    // 分包3 输出
    wire signed [15:0] target_azimuth_01deg;    
    wire signed [15:0] target_elevation_01deg;   
    wire [TDOA_WIDTH-1:0]  tdoa_vals [0:MAX_PAIR_NUM-1];
    wire direction_valid;
    // 阵列几何配置（4 路十字阵列，单位 cm）
    wire [COORD_WIDTH-1:0] mic_coord_x [0:MAX_CHAN_NUM-1];
    wire [COORD_WIDTH-1:0] mic_coord_y [0:MAX_CHAN_NUM-1];
    // 1. 分包1：采集
    mic_collect_top u_collect (
        .clk_50m         (clk_50m),
        .rst_n           (rst_n),
        .mic_pdm_sdin    (mic_pdm_sdin),
        .mic_bclk        (mic_bclk),
        .mic_lrck        (mic_lrck),
        .board_led       (board_led),
        .mic_ch_data     (mic_ch_data),
        .mic_frame_cnt   (mic_frame_cnt),
        .mic_frame_valid (mic_frame_valid),
        .mic_rd_en       (mic_rd_en),
        .mic_rd_addr     (mic_rd_addr),
        .mic_rd_data     (mic_rd_data),
        .agc_gain_changed(mic_agc_gain_changed)
    );
    // 2. 分包2：预处理
    preprocess_top u_preprocess (
        .clk_50m         (clk_50m),
        .rst_n           (rst_n),
        .mic_ch_data     (mic_ch_data),
        .mic_frame_cnt   (mic_frame_cnt),
        .mic_frame_valid (mic_frame_valid),
        .rd_en           (mic_rd_en),
        .rd_addr         (mic_rd_addr),
        .rd_data         (mic_rd_data),
        .agc_gain_changed(mic_agc_gain_changed),
        .fft_idx         (fft_idx),
        .fft_re          (fft_re),
        .fft_im          (fft_im),
        .freq_valid_mask (freq_valid_mask),
        .pre_frame_cnt   (pre_frame_cnt),
        .pre_frame_valid (pre_frame_valid),
        .mask_learn_done (mask_learn_done),
        .fft_ip_latency  (fft_ip_latency)
    );
    // 3. 分包3：测向
    phat_position_top u_position (
        .clk_50m                (clk_50m),
        .rst_n                  (rst_n),
        .fft_idx                (fft_idx),
        .fft_re                 (fft_re),
        .fft_im                 (fft_im),
        .freq_valid_mask        (freq_valid_mask),
        .pre_frame_cnt          (pre_frame_cnt),
        .pre_frame_valid        (pre_frame_valid),
        .mask_learn_done        (mask_learn_done),
        .fft_ip_latency         (fft_ip_latency),
        .mic_coord_x            (mic_coord_x),
        .mic_coord_y            (mic_coord_y),
        .target_azimuth_01deg   (target_azimuth_01deg),
        .target_elevation_01deg (target_elevation_01deg),
        .tdoa_result            (tdoa_vals),
        .direction_valid        (direction_valid),
        .uart_tx                (uart_tx)
    );
    assign mic_coord_x[0] = -16'sd19;   // 左（-19）
    assign mic_coord_y[0] = 16'sd0;
    assign mic_coord_x[1] = 16'sd19;    // 右（+19）
    assign mic_coord_y[1] = 16'sd0;
    assign mic_coord_x[2] = 16'sd0;
    assign mic_coord_y[2] = -16'sd19;   // 下（-19）
    assign mic_coord_x[3] = 16'sd0;
    assign mic_coord_y[3] = 16'sd19;    // 上（+19）
    seg8_hc595 u_seg8 (
        .clk_50m      (clk_50m),
        .rst_n        (rst_n),
        .az_01deg     (target_azimuth_01deg),
        .el_01deg     (target_elevation_01deg),
        .sample_valid (direction_valid),
        .sh_cp        (seg_sh_cp),
        .st_cp        (seg_st_cp),
        .ds           (seg_ds)
    );
    servo_pan_tilt u_servo_pan_tilt (
        .clk_50m  (clk_50m),
        .rst_n    (rst_n),
        .az_01deg (target_azimuth_01deg),
        .el_01deg (target_elevation_01deg),
        .update   (direction_valid),
        .pan_pwm  (servo_pan_pwm),
        .tilt_pwm (servo_tilt_pwm)
    );
endmodule
