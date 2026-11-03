// ============================================================
// tb_phat_position_top.sv - 分包3 PHAT 测向全链路验证（3D 声源）
//
// 阵列：十字阵，对臂间距 2d = 40 cm（每只麦离中心 20 cm）—— 2026-10-22 由 15 cm 恢复
//
// 场景 1（远场）：声源 (100,100,200)cm —— 距阵心约 245cm，约 6 倍基线 40cm，近似成立
//                 真值 τ01=τ23=4862(Q10) → φ=+45.0°、θ≈+54.8°
// 场景 2（近场）：声源 (10,10,0)cm   —— 声源在阵列平面内、距阵心仅 14cm，
//                 平面波假设不成立（近场效应，见文末说明）
//
// 激励：ch_i 频域相位 = 2πk/256 − 2πk·Δτ_i/256，
//   Δτ_i = (r_i − r_0)/3.43 采样点，r_i = √((sx−mx_i)²+(sy−my_i)²+sz²)
//   前 3 帧掩码全 0（学习期）→ 第 4 帧起给全 1 掩码，输出测向结果
//
// 断言（两个场景各一次）：
//   ① 实测 TDOA 与几何真值之差 ≤ 16（Q10，即 0.016 采样点）；
//   ② 以实测 TDOA 代入远场测向公式现算参考，
//      要求 |方位角−参考| ≤ 1 LSB 且 |俯仰角−参考| ≤ 1 LSB（0.1°/LSB）。
// ============================================================
`timescale 1ns/1ps
`include "project_global_param.vh"

module tb_phat_position_top;

    reg         clk_50m;
    reg         rst_n;

    reg         pre_frame_valid;
    reg  [9:0]  fft_idx;
    reg  [23:0] fft_re [0:MAX_CHAN_NUM-1];
    reg  [23:0] fft_im [0:MAX_CHAN_NUM-1];
    reg  [FFT_BINS-1:0] freq_valid_mask;
    reg         mask_learn_done;
    reg  [15:0] pre_frame_cnt;
    reg  [9:0]  fft_ip_latency;

    reg  signed [15:0] mic_coord_x [0:MAX_CHAN_NUM-1];
    reg  signed [15:0] mic_coord_y [0:MAX_CHAN_NUM-1];

    wire signed [15:0] target_azimuth_01deg;
    wire signed [15:0] target_elevation_01deg;
    wire [15:0] tdoa_result [0:MAX_PAIR_NUM-1];
    wire        direction_valid;
    wire        uart_tx;

    phat_position_top uut (
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
        .tdoa_result            (tdoa_result),
        .direction_valid        (direction_valid),
        .uart_tx                (uart_tx)
    );

    initial begin
        clk_50m = 0; rst_n = 0;
        pre_frame_valid = 0; fft_idx = 0;
        for (int i = 0; i < MAX_CHAN_NUM; i++) begin
            fft_re[i] = 0; fft_im[i] = 0;
            mic_coord_x[i] = 0; mic_coord_y[i] = 0;
        end
        freq_valid_mask = 0; mask_learn_done = 0; pre_frame_cnt = 0; fft_ip_latency = 310;
        // 十字阵列坐标（cm），与 top_gcc_phat 一致（半径 20 cm，整数可精确表示）。
        // **它只喂给 DUT 的死端口**（tdoa_solver 不使用），
        // 真正的激励几何由下面的 ARM_CM（real，精确 20.0）给出。
        mic_coord_x[0] = -20; mic_coord_y[0] =  0;
        mic_coord_x[1] =  20; mic_coord_y[1] =  0;
        mic_coord_x[2] =   0; mic_coord_y[2] = -20;
        mic_coord_x[3] =   0; mic_coord_y[3] =  20;
        #500; rst_n = 1;
    end

    always #10 clk_50m = ~clk_50m;

    // ================= 几何 → 期望 TDOA =================
    real    mx_r [0:3], my_r [0:3], rr [0:3], dt_r [0:3];
    integer et01, et23, cc, kk, bi, nval;
    real    pi2;
    integer exp_t01, exp_t23;

    // 每只麦离阵列中心的距离（cm）—— 激励几何的唯一来源，精确 20.0。
    localparam real ARM_CM = 20.0;
    initial begin
        mx_r[0] = -ARM_CM; my_r[0] =  0.0;
        mx_r[1] =  ARM_CM; my_r[1] =  0.0;
        mx_r[2] =  0.0;    my_r[2] = -ARM_CM;
        mx_r[3] =  0.0;    my_r[3] =  ARM_CM;
    end

    task automatic geom(input real sx, input real sy, input real sz);
        begin
            for (cc = 0; cc < 4; cc = cc + 1) begin
                rr[cc]   = $sqrt((sx-mx_r[cc])*(sx-mx_r[cc]) +
                                 (sy-my_r[cc])*(sy-my_r[cc]) + sz*sz);
            end
            // 到达时延（采样点，正 = 比 ch0 晚到）
            dt_r[0] = 0.0;
            dt_r[1] = (rr[1]-rr[0])/3.43;
            dt_r[2] = (rr[2]-rr[0])/3.43;
            dt_r[3] = (rr[3]-rr[0])/3.43;
            // 期望 TDOA（Q10）：tdoa_result[0] = τ0−τ1、tdoa_result[5] = τ2−τ3
            exp_t01 = (rr[0]-rr[1])/3.43*1024.0;
            exp_t23 = (rr[2]-rr[3])/3.43*1024.0;
            et01 = $rtoi(exp_t01 + (exp_t01 >= 0 ? 0.5 : -0.5));
            et23 = $rtoi(exp_t23 + (exp_t23 >= 0 ? 0.5 : -0.5));
            $display("[几何] 声源(%0.1f,%0.1f,%0.1f)cm: r=%0.2f/%0.2f/%0.2f/%0.2f cm, 期望τ01=%0d τ23=%0d (Q10)",
                     sx, sy, sz, rr[0], rr[1], rr[2], rr[3], et01, et23);
        end
    endtask

    // ---- 频域激励生成（Q22 单位向量谱，带时延相位） ----
    integer sig_re [0:MAX_CHAN_NUM-1][0:128];
    integer sig_im [0:MAX_CHAN_NUM-1][0:128];

    task automatic gen_stim;
        begin
            for (kk = 0; kk <= 128; kk = kk + 1) begin
                for (cc = 0; cc < MAX_CHAN_NUM; cc = cc + 1) begin
                    sig_re[cc][kk] = $rtoi(4194304.0 *
                        $cos(pi2*kk/256.0 - pi2*kk*dt_r[cc]/256.0));
                    sig_im[cc][kk] = $rtoi(4194304.0 *
                        $sin(pi2*kk/256.0 - pi2*kk*dt_r[cc]/256.0));
                end
            end
        end
    endtask

    // ---- 帧突发驱动：每帧 129 拍突发 + 6 万拍间隔 ----
    // 学习期由掩码全 0 表达（pair_scheduler 在掩码全 0 时把 phat_buf 写零，
    // 解算侧被 mask_learn_done 门控），因此两个场景之间无需复位 DUT——
    // 中途复位会让 FFT IP 需要重新走配置握手，反而引入无关风险。
    integer fn;
    task automatic run_frames(input integer nfr, input integer fn0);
        begin
            for (fn = 0; fn < nfr; fn = fn + 1) begin
                for (bi = 0; bi <= 128; bi = bi + 1) begin
                    @(posedge clk_50m); #2;
                    pre_frame_valid <= 1;
                    fft_idx <= bi;
                    for (int ci = 0; ci < MAX_CHAN_NUM; ci = ci + 1) begin
                        fft_re[ci] <= sig_re[ci][bi];
                        fft_im[ci] <= sig_im[ci][bi];
                    end
                end
                @(posedge clk_50m); #2;
                pre_frame_valid <= 0;
                // 前 3 帧掩码全 0 模拟学习期，第 4 帧起全 1；
                // 学习完成信号同步给出（与 freq_noise_mask 的 learn_done 一致）
                freq_valid_mask <= (fn >= 3) ? {FFT_BINS{1'b1}} : {FFT_BINS{1'b0}};
                mask_learn_done <= (fn >= 3);
                pre_frame_cnt   <= fn0 + fn;
                repeat (60000) @(posedge clk_50m);
                $display("[进度] 帧号 %0d 结束, t=%0t, 累计有效输出 %0d", fn0+fn, $time, nval);
            end
        end
    endtask

    // ================= 结果捕获与断言 =================
    integer n_err, cap_az, cap_el, cap_t01, cap_t23;
    integer ref_az, ref_el;

    // 参考模型：远场方向余弦 → 方位角/俯仰角（real 现算）
    real ca_r, cb_r, cg_r, rho_r, phi_r, th_r;
    // 2d = 40 cm（对臂间距）：K = 34300/(40×10000)/1024×2^23 = 702.464（2026-10-22 由 1873.237 恢复）
    localparam real COS_PER_TAU = 702.464 / 8388608.0;   // cos = τ(Q10) × 8.3740e-5
    localparam real R2D        = 57.29577951308232;

    function integer dabs(input integer x);
        dabs = (x < 0) ? -x : x;
    endfunction

    task automatic round01(input real d, output integer r);
        begin
            if (d >= 0.0) r = $rtoi(d*10.0 + 0.5);
            else          r = -$rtoi(-d*10.0 + 0.5);
        end
    endtask

    task automatic ref_dir(input integer t01, input integer t23);
        begin
            ca_r  = t01 * COS_PER_TAU;
            cb_r  = t23 * COS_PER_TAU;
            rho_r = $sqrt(ca_r*ca_r + cb_r*cb_r);
            cg_r  = $sqrt(1.0 - ca_r*ca_r - cb_r*cb_r);
            phi_r = $atan2(cb_r, ca_r) * R2D;
            th_r  = $atan2(cg_r, rho_r) * R2D;
            round01(phi_r, ref_az);
            round01(th_r,  ref_el);
        end
    endtask

    always @(posedge clk_50m) begin
        if (direction_valid) begin
            cap_az  <= $signed(target_azimuth_01deg);
            cap_el  <= $signed(target_elevation_01deg);
            cap_t01 <= $signed(tdoa_result[0]);
            cap_t23 <= $signed(tdoa_result[5]);
            nval    <= nval + 1;
        end
    end

    task automatic report(input [255:0] nm);
        begin
            $display("---- %0s ----", nm);
            if (nval == 0) begin
                $error("%0s: 全程无 direction_valid（测向链路未出结果）", nm);
                n_err = n_err + 1;
            end else begin
                ref_dir(cap_t01, cap_t23);
                $display("  有效输出 %0d 次；实测 τ01=%0d τ23=%0d（几何真值 %0d / %0d）",
                         nval, cap_t01, cap_t23, et01, et23);
                $display("  测向输出 φ=%0d θ=%0d | 由实测τ算参考 φ=%0d θ=%0d",
                         cap_az, cap_el, ref_az, ref_el);
                if (dabs(cap_t01-et01) > 16 || dabs(cap_t23-et23) > 16) begin
                    $error("%0s: TDOA 与几何真值偏差过大（τ01 %0d vs %0d，τ23 %0d vs %0d）",
                           nm, cap_t01, et01, cap_t23, et23);
                    n_err = n_err + 1;
                end
                if (dabs(cap_az-ref_az) > 1 || dabs(cap_el-ref_el) > 1) begin
                    $error("%0s: 测向误差超 1 LSB（φ %0d vs %0d，θ %0d vs %0d）",
                           nm, cap_az, ref_az, cap_el, ref_el);
                    n_err = n_err + 1;
                end
            end
        end
    endtask

    // ================= 主流程 =================
    initial begin
        pi2   = 6.283185307179586;
        n_err = 0;
        @(posedge rst_n);
        repeat (500) @(posedge clk_50m);

        // ---- 场景 1：远场 (100,100,200)cm，阵心视线 (100,100,200)/244.9
        //      远场真值 φ=+45.0°、θ=54.7356°；几何真值 τ01=τ23=4862(Q10)
        //      → 由实测τ算参考 θ≈54.8°（548）----
        geom(100.0, 100.0, 200.0);
        gen_stim;
        nval = 0; cap_az = 0; cap_el = 0; cap_t01 = 0; cap_t23 = 0;
        run_frames(6, 0);
        report("场景1-远场(100,100,200)");

        // ---- 场景 2：近场 (10,10,0)cm，声源在阵列平面内、距阵心 14.1cm
        //      远场真值 φ=+45.0°、θ=0°；TDOA 受近场弯曲影响，几何真值 τ01=τ23=5219
        //      → cosα=cosβ=0.4370（而非远场的 0.7071）→ 远场公式给 θ≈51.8°（518）。
        //      注：40cm 基线相对声源较大，近场偏差比 15cm 时更明显（51.8° vs 21.8°）。
        //      本用例验证的是"链路+解算"自洽，非几何真值（近场效应见文档 01） ----
        geom(10.0, 10.0, 0.0);
        gen_stim;
        nval = 0; cap_az = 0; cap_el = 0; cap_t01 = 0; cap_t23 = 0;
        run_frames(6, 6);      // 帧号接着排；前 3 帧掩码全 0 即新一轮学习期
        report("场景2-近场(10,10,0)");

        $display("========================================");
        $display("测向全链路错误数 : %0d", n_err);
        $display("========================================");
        if (n_err == 0) $display("PASS");
        else            $display("FAIL");
        $finish;
    end

endmodule
