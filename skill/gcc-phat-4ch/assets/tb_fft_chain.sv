// ============================================================
// tb_fft_chain.sv - 分包2 加窗FFT链路验证（4 路并行版）
//
// 场景：4 路 256 点 1kHz 正弦（10kHz 采样，24 位，幅度 8,000,000）
//   连续 3 帧 → 检查频域突发输出在索引 25/26 处出现峰值
//   （1kHz×256/10000 = 25.6 个周期 → 频点 25.6）
//
// 2026-10-27：适配 4 路并行 256 点 FFT。
//   - 读接口无 rd_ch，rd_data 为 4 通道并行数组
//   - 掩码端口宽度改为 FFT_BINS（129）
//   - 帧缓存读模型：同一地址同时给 4 路
// ============================================================
`timescale 1ns/1ps
`include "project_global_param.vh"

module tb_fft_chain;

    reg         clk_50m;
    reg         rst_n;

    // ---- DUT ----
    reg  [DATA_WIDTH-1:0] mic_ch_data [0:MAX_CHAN_NUM-1];
    reg  [FRAME_CNT_WIDTH-1:0] mic_frame_cnt;
    reg         mic_frame_valid;
    wire        rd_en;
    wire [FFT_IDX_WIDTH-1:0] rd_addr;
    reg  [DATA_WIDTH-1:0] rd_data [0:MAX_CHAN_NUM-1];
    wire [FFT_IDX_WIDTH-1:0] fft_idx;
    wire [DATA_WIDTH-1:0] fft_re [0:MAX_CHAN_NUM-1];
    wire [DATA_WIDTH-1:0] fft_im [0:MAX_CHAN_NUM-1];
    wire [FFT_BINS-1:0] freq_valid_mask;
    wire [FRAME_CNT_WIDTH-1:0] pre_frame_cnt;
    wire        pre_frame_valid;
    wire [9:0]  fft_ip_latency;

    preprocess_top uut (
        .clk_50m        (clk_50m),
        .rst_n          (rst_n),
        .mic_ch_data    (mic_ch_data),
        .mic_frame_cnt  (mic_frame_cnt),
        .mic_frame_valid(mic_frame_valid),
        .rd_en          (rd_en),
        .rd_addr        (rd_addr),
        .rd_data        (rd_data),
        .agc_gain_changed(1'b0),      // 本 TB 不涉及 AGC 换挡
        .fft_idx        (fft_idx),
        .fft_re         (fft_re),
        .fft_im         (fft_im),
        .freq_valid_mask(freq_valid_mask),
        .pre_frame_cnt  (pre_frame_cnt),
        .pre_frame_valid(pre_frame_valid),
        .mask_learn_done(),
        .fft_ip_latency (fft_ip_latency)
    );

    // ---- 通道相位一致性检验的变量与工具函数 ----
    // （SystemVerilog 要求变量先声明后使用；帧 FSM 的收尾打印会引用 ph_checks，
    //   故声明放在所有 initial 之前。）
    function automatic integer iabs(input integer v);
        iabs = (v < 0) ? -v : v;
    endfunction

    integer ph_checks, ph_errs, ph_tol, ph_mag;
    initial begin
        ph_checks = 0;
        ph_errs   = 0;
    end

    initial begin
        for (int i = 0; i < MAX_CHAN_NUM; i++) begin
            mic_ch_data[i] = 0;
            rd_data[i]     = 0;
        end
        mic_frame_cnt   = 0;
        mic_frame_valid = 0;
        clk_50m = 0;
        rst_n   = 0;
        #200;
        rst_n = 1;
    end

    always #10 clk_50m = ~clk_50m;      // 50MHz

    // ---- 正弦激励（1kHz @ 10kHz 采样 → 相位步进 2π/10） ----
    // 2026-10-27：4 路分别加 0°/90°/180°/270° 初相（sin, cos, −sin, −cos），
    // 用于验证 4 颗并行 FFT 的通道映射、频点索引与相位一致性：
    // 输入乘 e^{jφ} 会把整条频谱旋转 φ，故 bin 26 应满足
    //   X1 ≈ jX0、X2 ≈ −X0、X3 ≈ −jX0（逐通道独立检验，见下方自动断言）。
    integer sin_vals [0:255];
    integer cos_vals [0:255];
    integer si;
    real    pi;
    initial begin
        pi = 3.141592653589793;
        for (si = 0; si < 256; si = si + 1) begin
            sin_vals[si] = $rtoi(8000000.0 * $sin(2.0 * pi * si / 10.0));
            cos_vals[si] = $rtoi(8000000.0 * $cos(2.0 * pi * si / 10.0));
        end
    end

    // ---- 帧缓存读模型（BRAM 同步读，1 拍延迟；4 路同址并行） ----
    always @(posedge clk_50m) begin
        if (rd_en) begin
            rd_data[0] <= sin_vals[rd_addr];
            rd_data[1] <= cos_vals[rd_addr];
            rd_data[2] <= -sin_vals[rd_addr];
            rd_data[3] <= -cos_vals[rd_addr];
        end
    end

    // ---- 帧触发：复位后 300 拍发帧 0，此后每次突发结束后 500 拍发下一帧 ----
    integer frame_cnt;
    initial begin
        frame_cnt = 0;
        @(posedge rst_n);
        repeat (300) @(posedge clk_50m);
        mic_frame_cnt   <= frame_cnt;
        mic_frame_valid <= 1'b1;
        @(posedge clk_50m);
        mic_frame_valid <= 1'b0;
        frame_cnt = frame_cnt + 1;
        // 等突发完成
        wait (pre_frame_valid === 1'b1);
        wait (pre_frame_valid === 1'b0);
        repeat (2) begin
            repeat (500) @(posedge clk_50m);
            mic_frame_cnt   <= frame_cnt;
            mic_frame_valid <= 1'b1;
            @(posedge clk_50m);
            mic_frame_valid <= 1'b0;
            frame_cnt = frame_cnt + 1;
            wait (pre_frame_valid === 1'b1);
            wait (pre_frame_valid === 1'b0);
        end
        repeat (200) @(posedge clk_50m);
        $display("=== 3帧完成：通道相位检验 %0d 次，失败 %0d 次 ===", ph_checks, ph_errs);
        if (ph_checks >= 2 && ph_errs == 0) $display("PASS");
        else                                $display("FAIL");
        $finish;
    end

    // ---- 峰值观察：突发期间打印峰值附近 4 路实/虚部 ----
    integer obs_k;
    always @(posedge clk_50m) begin
        if (pre_frame_valid && ((fft_idx >= 25 && fft_idx <= 26) ||
                                (fft_idx >= 96 && fft_idx <= 110))) begin
            obs_k = fft_idx;
            $display("t=%0t 帧%0d idx=%0d | ch0 re=%0d im=%0d | ch1 re=%0d im=%0d | ch2 re=%0d im=%0d | ch3 re=%0d im=%0d",
                     $time, pre_frame_cnt, obs_k,
                     $signed(fft_re[0]), $signed(fft_im[0]),
                     $signed(fft_re[1]), $signed(fft_im[1]),
                     $signed(fft_re[2]), $signed(fft_im[2]),
                     $signed(fft_re[3]), $signed(fft_im[3]));
        end
    end

    // ---- 通道相位一致性自动检验（bin 26，帧 1/2 共 2 次） ----
    // 输入 ch1..3 相对 ch0 依次旋转 +90°/180°/−90°，整条频谱同样旋转：
    //   X1 = jX0  → re1 = −im0, im1 = re0
    //   X2 = −X0  → re2 = −re0, im2 = −im0
    //   X3 = −jX0 → re3 = im0,  im3 = −re0
    // 容差取 |X0| 的 2%（FFT 各级截断 + 非整频点泄漏）。
    always @(posedge clk_50m) begin
        if (pre_frame_valid && pre_frame_cnt >= 1 && fft_idx == 26) begin
            ph_mag = iabs($signed(fft_re[0])) + iabs($signed(fft_im[0]));
            ph_tol = ph_mag / 50 + 2000;          // 2% + 2000 LSB 余量
            ph_checks = ph_checks + 1;
            if (iabs($signed(fft_re[1]) + $signed(fft_im[0])) > ph_tol ||
                iabs($signed(fft_im[1]) - $signed(fft_re[0])) > ph_tol)
                ph_errs = ph_errs + 1;
            if (iabs($signed(fft_re[2]) + $signed(fft_re[0])) > ph_tol ||
                iabs($signed(fft_im[2]) + $signed(fft_im[0])) > ph_tol)
                ph_errs = ph_errs + 1;
            if (iabs($signed(fft_re[3]) - $signed(fft_im[0])) > ph_tol ||
                iabs($signed(fft_im[3]) + $signed(fft_re[0])) > ph_tol)
                ph_errs = ph_errs + 1;
        end
    end

    // ---- 超时保护 ----
    initial begin
        #3000000;
        $display("TIMEOUT");
        $finish;
    end

endmodule
