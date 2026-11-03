// ============================================================
// tb_i2s_moving.sv - 10 m 定距移动目标的 I2S 全流程验证
//
// 激励：moving_10m.mem（make_moving_10m.py 生成，复用
//       make_real_multipos.py 的真实录音声源/残余模型）
//       生成时必须带 --highpass 900，与已验证的六距离激励一致；
//       不加滤波时约 89% 能量在 1 kHz 以下、掩码带内仅占 1.3%，
//       定点 PHAT 会解出随机峰值。
//   3 帧学习（2.483~2.560 s，最安静段）
//   + 12 帧测量（4.685~4.710 s 的相干瞬态），R=10 m 恒定：
//       fn=4 .. 15：φ = -55° + 10°×(fn-4)，θ = 20° + 2°×(fn-4)
//   每帧的四路时延按精确球面波重新施加（含近场曲率），
//   距离增益 g=300/10=30，测量帧各自定标到 0.95×2^23。
//
// 期望角度（0.1°）：A = -550 + 100×(fn-4)，E = 200 + 20×(fn-4)
//   容差 ±4.0°（与六距离测试相同）。
//
// 统计窗口 fn=4..15；末帧 fn=15 也等待，done 条件 fn >= 15。
// ============================================================
`timescale 1ns/1ps
`include "project_global_param.vh"

module tb_i2s_moving;

    localparam integer FRAME_SAMPLES = 256;
    localparam integer NFRAMES       = 15;          // 3 学习 + 12 测量
    localparam integer NSAMP         = FRAME_SAMPLES * NFRAMES;
    localparam integer FRM_LEN       = 91;
    localparam integer BAUD_CYC      = UART_BAUD_DIV;
`ifdef FAST_SIM
    localparam integer TIMEOUT_CYC   = NFRAMES * 250_000;   // 100 kHz：128k 拍/帧
`else
    localparam integer TIMEOUT_CYC   = NFRAMES * 1_500_000; // 10 kHz：1.28M 拍/帧
`endif

    localparam integer TOL_01  = 40;                // ±4.0°

    // 期望角度表（0.1°）：10 m 定距，φ/θ 线性变化
    function automatic integer exp_a(input integer f);
        case (f)
            4:  exp_a = -550;  5:  exp_a = -450;  6:  exp_a = -350;
            7:  exp_a = -250;  8:  exp_a = -150;  9:  exp_a = -50;
            10: exp_a =   50;  11: exp_a =  150;  12: exp_a =  250;
            13: exp_a =  350;  14: exp_a =  450;  15: exp_a =  550;
            default: exp_a = 0;
        endcase
    endfunction

    function automatic integer exp_e(input integer f);
        case (f)
            4:  exp_e = 200;  5:  exp_e = 220;  6:  exp_e = 240;
            7:  exp_e = 260;  8:  exp_e = 280;  9:  exp_e = 300;
            10: exp_e = 320;  11: exp_e = 340;  12: exp_e = 360;
            13: exp_e = 380;  14: exp_e = 400;  15: exp_e = 420;
            default: exp_e = 0;
        endcase
    endfunction

    reg  clk_50m = 0;
    reg  rst_n   = 0;

    wire [7:0] mic_pdm_sdin;
    wire       mic_bclk;
    wire       mic_lrck;
    wire [3:0] board_led;
    wire       uart_tx;

    // ---------------- 激励存储 ----------------
    reg [23:0] audio [0:4*NSAMP-1];
    initial $readmemh("moving_10m.mem", audio);

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
    // 1. I2S 发送器
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
    integer da, de, ea, ee;

    // 逐帧调试：把 UART 帧里的 6 组 τ（|τ|>>>10，整数采样点）写文件
    integer dbg_fd;
    integer tp, tbase, tval;
    initial dbg_fd = $fopen("moving_dbg.txt", "w");

    function automatic integer dec3i(input integer base);
        dec3i = (rx_buf[base]-8'h30)*100 + (rx_buf[base+1]-8'h30)*10
              + (rx_buf[base+2]-8'h30);
    endfunction

    function automatic integer dec4(input integer base);
        dec4 = (rx_buf[base]-8'h30)*1000 + (rx_buf[base+1]-8'h30)*100
             + (rx_buf[base+2]-8'h30)*10 + (rx_buf[base+3]-8'h30);
    endfunction

    function automatic integer dec3(input integer base);
        dec3 = (rx_buf[base]-8'h30)*100 + (rx_buf[base+1]-8'h30)*10
             + (rx_buf[base+2]-8'h30);
    endfunction

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

            // 调试：6 组 τ（|τ|>>>10，整数采样点，含符号）+ A/E/V
            $fwrite(dbg_fd, "fn=%0d V=%0d A=%0d E=%0d T=", fn, vbit, a01, e01);
            for (tp = 0; tp < 6; tp = tp + 1) begin
                tbase = 11 + 9*tp;
                tval  = dec3i(tbase + 5);
                if (rx_buf[tbase+4] == "-") tval = -tval;
                if (tp == 5) $fwrite(dbg_fd, "%0d\n", tval);
                else         $fwrite(dbg_fd, "%0d,", tval);
            end
            $fflush(dbg_fd);

            // 校验和：第 1..85 字节 XOR
            cks = 0;
            for (k = 1; k <= 85; k = k + 1) cks = cks ^ rx_buf[k];
            cks_rx = ((rx_buf[87] <= "9") ? (rx_buf[87]-8'h30) : (rx_buf[87]-8'h41+10))*16
                   + ((rx_buf[88] <= "9") ? (rx_buf[88]-8'h30) : (rx_buf[88]-8'h41+10));

            if (fn >= 4 && fn <= NFRAMES) begin
                ea = exp_a(fn);
                ee = exp_e(fn);
                $display("[UART] fn=%0d A=%0d (A=%0d.%0d°) E=%0d (E=%0d.%0d°) V=%0d  exp A=%0d (A=%0d.%0d°) E=%0d (E=%0d.%0d°)  cks=%02X%s",
                         fn, a01,
                         a01/10, (a01%10<0?-(a01%10):(a01%10)),
                         e01,
                         e01/10, (e01%10<0?-(e01%10):(e01%10)), vbit,
                         ea, ea/10, (ea%10<0?-(ea%10):(ea%10)),
                         ee, ee/10, (ee%10<0?-(ee%10):(ee%10)),
                         cks_rx, (cks_rx == cks) ? "" : "  <-- 校验错");
            end else begin
                $display("[UART] fn=%0d A=%0d (A=%0d.%0d°) E=%0d (E=%0d.%0d°) V=%0d  cks=%02X%s",
                         fn, a01,
                         a01/10, (a01%10<0?-(a01%10):(a01%10)),
                         e01,
                         e01/10, (e01%10<0?-(e01%10):(e01%10)), vbit,
                         cks_rx, (cks_rx == cks) ? "" : "  <-- 校验错");
            end
            if (cks_rx != cks) begin
                n_bad_cks = n_bad_cks + 1;
                $display("        期望校验和 %02X，实收 %02X", cks, cks_rx);
            end

            // 统计窗口：fn=4..15（fn=1..3 是学习帧，V=0 正常）
            if (fn >= 4 && fn <= NFRAMES) begin
                n_meas = n_meas + 1;
                if (vbit == 1) begin
                    n_valid_meas = n_valid_meas + 1;
                    da = a01 - exp_a(fn); if (da < 0) da = -da;
                    de = e01 - exp_e(fn); if (de < 0) de = -de;
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

            if (fn >= NFRAMES) done = 1'b1;
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
        $display("I2S 10 m 移动目标：共收到 UART 帧 %0d 个（NFRAMES=%0d）",
                 n_frames, NFRAMES);
        $display("测量帧(4..%0d)：见到 %0d 个，V=1 %0d 个", NFRAMES, n_meas, n_valid_meas);
        $display("角度最大误差：ΔA=%0d、ΔE=%0d（0.1°）", max_da, max_de);
        $display("角度超差 %0d 个，校验错 %0d 个，帧头异常 %0d 个",
                 n_bad_angle, n_bad_cks, n_bad_hdr);
        if (!done)
            $display("超时：未收到帧号 >= %0d 的 UART 帧", NFRAMES);
        $display("========================================");
        if (done && n_valid_meas == NFRAMES-3 && n_bad_angle == 0 &&
            n_bad_cks == 0 && n_bad_hdr == 0)
            $display("PASS");
        else
            $display("FAIL");
        $finish;
    end

endmodule
