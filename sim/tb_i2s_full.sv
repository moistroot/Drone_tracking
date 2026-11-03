// ============================================================
// tb_i2s_full.sv - I2S 全流程验证（真实录音时域激励）
//
// 验证对象：top_gcc_phat 整条链路
//   mic_pdm_sdin[3:0] (I2S) → mic_collect_top → preprocess_top
//   → phat_position_top → uart_tx
//
// 激励：may7_i2s_samples.mem（make_i2s_stim.py 生成）
//   默认 3 帧学习 + 12 帧测量（FAST_SIM 下 3 + 6），
//   录音 5月7日.mp3，注入 φ=45°/θ=30°；
//   每行 4 个 24 位十六进制：ch0 ch1 ch2 ch3，连续时域 PCM。
//
// TB 侧：
//   * 按标准 I2S 左声道格式在 mic_pdm_sdin 上串行发 24 位；
//   * 用 115200 8N1 接收器还原 91 字节 UART 帧；
//   * 解析帧号 / A / E / V，校验第 1..85 字节 XOR；
//   * 统计窗口取帧号 3..NFRAMES-1：V=1、A≈+45.0°、E≈+30.0°，容差 ±2.0°。
//     帧号 1~3 是掩码学习帧，fn=3 是最后一个学习帧，其 V=0 属正常；
//     实际测量帧为 fn=4..NFRAMES-1（末帧作为结束标志，不再等待）。
//
// 断言：有效测量帧数 ≥ NFRAMES-4，且所有有效测量帧角度误差 ≤ 2.0°。
// ============================================================
`timescale 1ns/1ps
`include "project_global_param.vh"

module tb_i2s_full;

    localparam integer FRAME_SAMPLES = 256;
`ifdef FAST_SIM
    // 时间压缩版：采样率 100 kHz（包参数覆盖），同样的 256 点样本序列，
    // 每帧周期从 25.6 ms 压到 2.56 ms，仿真时间约 1/10。
    localparam integer NFRAMES       = 9;           // 3 学习 + 6 测量
`else
    localparam integer NFRAMES       = 15;          // 3 学习 + 12 测量
`endif
    localparam integer NSAMP         = FRAME_SAMPLES * NFRAMES;
    localparam integer FRM_LEN       = 91;
    localparam integer BAUD_CYC      = UART_BAUD_DIV;
`ifdef FAST_SIM
    localparam integer TIMEOUT_CYC   = NFRAMES * 250_000;   // 100 kHz：帧周期 128k 拍
`else
    localparam integer TIMEOUT_CYC   = NFRAMES * 1_500_000; // 10 kHz：帧周期 1.28M 拍
`endif

    // 期望方向（0.1°）
    localparam integer EXP_A01 = 450;
    localparam integer EXP_E01 = 300;
    localparam integer TOL_01  = 20;                // ±2.0°

    reg  clk_50m = 0;
    reg  rst_n   = 0;

    wire [7:0] mic_pdm_sdin;
    wire       mic_bclk;
    wire       mic_lrck;
    wire [3:0] board_led;
    wire       uart_tx;

    // ---------------- 激励存储 ----------------
    reg [23:0] audio [0:4*NSAMP-1];
    initial $readmemh("may7_i2s_samples.mem", audio);

    // ---------------- DUT ----------------
    top_gcc_phat uut (
        .clk_50m       (clk_50m),
        .rst_n         (rst_n),
        .mic_pdm_sdin  (mic_pdm_sdin),
        .mic_bclk      (mic_bclk),
        .mic_lrck      (mic_lrck),
        .board_led     (board_led),
        .uart_tx       (uart_tx)
    );

    always #10 clk_50m = ~clk_50m;

    // ============================================================
    // 1. I2S 发送器（与 tb_mic_collect_top 相同的时序）
    //    LRCLK 下降沿装载采样；之后每个 BCLK 下降沿移出 1 位，
    //    先发 MSB，共 24 位；右声道不发（接收端只取左声道）。
    // ============================================================
    reg        bclk_d, lrck_d;
    always @(posedge clk_50m) begin
        bclk_d <= mic_bclk;
        lrck_d <= mic_lrck;
    end
    wire bclk_fall = bclk_d && !mic_bclk;
    wire lrck_fall = lrck_d && !mic_lrck;

    reg [23:0] tx_shift [0:3];
    reg [5:0]  tx_bit_cnt;
    reg        tx_active;
    integer    sample_idx;
    reg [7:0]  sdin_r;
    assign mic_pdm_sdin = sdin_r;

    integer c;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            tx_bit_cnt <= 6'd0;
            tx_active  <= 1'b0;
            sample_idx <= 0;
            sdin_r     <= 8'h00;
            for (c = 0; c < 4; c = c + 1)
                tx_shift[c] <= 24'h000000;
        end else begin
            if (lrck_fall) begin
                for (c = 0; c < 4; c = c + 1)
                    tx_shift[c] <= (sample_idx < NSAMP)
                                   ? audio[4*sample_idx + c] : 24'h000000;
                tx_bit_cnt <= 6'd0;
                tx_active  <= 1'b1;
                sample_idx <= sample_idx + 1;
            end else if (bclk_fall && tx_active) begin
                sdin_r[0]   <= tx_shift[0][23];
                sdin_r[1]   <= tx_shift[1][23];
                sdin_r[2]   <= tx_shift[2][23];
                sdin_r[3]   <= tx_shift[3][23];
                sdin_r[7:4] <= 4'h0;
                for (c = 0; c < 4; c = c + 1)
                    tx_shift[c] <= {tx_shift[c][22:0], 1'b0};
                if (tx_bit_cnt == 6'd23)
                    tx_active <= 1'b0;
                else
                    tx_bit_cnt <= tx_bit_cnt + 1'b1;
            end
        end
    end

    // ============================================================
    // 2. UART 接收器（115200 8N1，每位中点采样）
    // ============================================================
    reg [7:0]  rx_buf [0:127];
    integer    rx_n;
    reg [15:0] rx_tmr;
    reg [3:0]  rx_bit;
    reg        rx_act;
    reg        tx_d;
    reg        frame_done;

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            rx_n       <= 0;
            rx_act     <= 1'b0;
            rx_tmr     <= 16'd0;
            rx_bit     <= 4'd0;
            tx_d       <= 1'b1;
            frame_done <= 1'b0;
        end else begin
            tx_d       <= uart_tx;
            frame_done <= 1'b0;

            if (!rx_act) begin
                if (tx_d && !uart_tx) begin
                    rx_act      <= 1'b1;
                    rx_bit      <= 4'd0;
                    rx_tmr      <= BAUD_CYC + BAUD_CYC/2 + 1;
                    rx_buf[rx_n] <= 8'h00;
                end
            end else if (rx_tmr == 0) begin
                if (rx_bit < 4'd8) begin
                    if (uart_tx)
                        rx_buf[rx_n] <= rx_buf[rx_n] | (8'd1 << rx_bit);
                    rx_bit <= rx_bit + 1'b1;
                    rx_tmr <= BAUD_CYC;
                end else begin
                    rx_act <= 1'b0;
                    if (rx_n == FRM_LEN-1) begin
                        rx_n       <= 0;
                        frame_done <= 1'b1;
                    end else begin
                        rx_n <= rx_n + 1;
                    end
                end
            end else begin
                rx_tmr <= rx_tmr - 1'b1;
            end
        end
    end

    // ============================================================
    // 3. 帧解析与检查
    // ============================================================
    integer fn, a01, e01, vbit, sg, ip;
    integer n_frames, n_meas, n_valid_meas, n_bad_angle, n_bad_cks, n_bad_hdr;
    integer max_da, max_de;
    reg     done;
    integer k, cks, cks_rx;

    function automatic integer dec4(input integer base);
        dec4 = (rx_buf[base]-8'h30)*1000 + (rx_buf[base+1]-8'h30)*100
             + (rx_buf[base+2]-8'h30)*10 + (rx_buf[base+3]-8'h30);
    endfunction

    function automatic integer dec3(input integer base);
        dec3 = (rx_buf[base]-8'h30)*100 + (rx_buf[base+1]-8'h30)*10
             + (rx_buf[base+2]-8'h30);
    endfunction

    integer da, de;

    always @(posedge clk_50m) begin
        if (frame_done) begin
            n_frames = n_frames + 1;

            if (rx_buf[0] != "$" || rx_buf[6] < "0" || rx_buf[9] > "9") begin
                n_bad_hdr = n_bad_hdr + 1;
                $display("[UART] 帧头异常：%c%c%c%c%c%c ...",
                         rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3], rx_buf[4], rx_buf[5]);
            end

            fn   = dec4(6);
            sg   = (rx_buf[67] == "-") ? -1 : 1;
            ip   = dec3(68);
            a01  = sg * (ip*10 + (rx_buf[72]-8'h30));
            sg   = (rx_buf[76] == "-") ? -1 : 1;
            ip   = dec3(77);
            e01  = sg * (ip*10 + (rx_buf[81]-8'h30));
            vbit = rx_buf[85] - 8'h30;

            // 校验和：第 1..85 字节 XOR
            cks = 0;
            for (k = 1; k <= 85; k = k + 1) cks = cks ^ rx_buf[k];
            cks_rx = ((rx_buf[87] <= "9") ? (rx_buf[87]-8'h30) : (rx_buf[87]-8'h41+10))*16
                   + ((rx_buf[88] <= "9") ? (rx_buf[88]-8'h30) : (rx_buf[88]-8'h41+10));

            $display("[UART] fn=%0d A=%0d.%0d E=%0d.%0d V=%0d  cks=%02X%s",
                     fn, (a01<0?-a01:a01)/10, (a01<0?-a01:a01)%10,
                     (e01<0?-e01:e01)/10, (e01<0?-e01:e01)%10, vbit,
                     cks_rx, (cks_rx == cks) ? "" : "  <-- 校验错");
            if (cks_rx != cks) begin
                n_bad_cks = n_bad_cks + 1;
                $display("        期望校验和 %02X，实收 %02X", cks, cks_rx);
            end

            // 统计窗口：帧号 3..NFRAMES-1（fn=3 是最后 1 个学习帧，V=0 正常）
            if (fn >= 3 && fn <= NFRAMES-1) begin
                n_meas = n_meas + 1;
                if (vbit == 1) begin
                    n_valid_meas = n_valid_meas + 1;
                    da = a01 - EXP_A01; if (da < 0) da = -da;
                    de = e01 - EXP_E01; if (de < 0) de = -de;
                    if (da > max_da) max_da = da;
                    if (de > max_de) max_de = de;
                    if (da > TOL_01 || de > TOL_01) begin
                        n_bad_angle = n_bad_angle + 1;
                        $display("        !! 角度超差：ΔA=%0d ΔE=%0d（容差 %0d）",
                                 da, de, TOL_01);
                    end
                end else begin
                    $display("        !! 测量帧 V=0（未解出有效方向）");
                end
            end

            if (fn >= NFRAMES-1) done = 1'b1;
        end
    end

    // ============================================================
    // 4. 主流程
    // ============================================================
    integer sim_cycles;
    always @(posedge clk_50m) sim_cycles <= sim_cycles + 1;

    initial begin
        n_frames = 0; n_meas = 0; n_valid_meas = 0;
        n_bad_angle = 0; n_bad_cks = 0; n_bad_hdr = 0;
        max_da = 0; max_de = 0;
        done = 1'b0; sim_cycles = 0;

        #500;
        rst_n = 1'b1;

        while (!done && sim_cycles < TIMEOUT_CYC)
            @(posedge clk_50m);

        repeat (10) @(posedge clk_50m);

        $display("========================================");
        $display("I2S 全流程：共收到 UART 帧 %0d 个（NFRAMES=%0d）", n_frames, NFRAMES);
        $display("测量帧(3..%0d)：见到 %0d 个，V=1 %0d 个", NFRAMES-1, n_meas, n_valid_meas);
        $display("角度最大误差：ΔA=%0d、ΔE=%0d（0.1°）", max_da, max_de);
        $display("角度超差 %0d 个，校验错 %0d 个，帧头异常 %0d 个",
                 n_bad_angle, n_bad_cks, n_bad_hdr);
        if (!done)
            $display("超时：未收到帧号 >= 14 的 UART 帧");
        $display("========================================");
        if (done && n_valid_meas >= NFRAMES-4 && n_bad_angle == 0 &&
            n_bad_cks == 0 && n_bad_hdr == 0)
            $display("PASS");
        else
            $display("FAIL");
        $finish;
    end

endmodule
