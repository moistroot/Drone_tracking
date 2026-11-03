// ============================================================
// tb_tdoa_solver.sv - 测向解算（方位角 φ + 俯仰角 θ）专项验证
//
// 被验模块：tdoa_solver（由两条正交基线 TDOA 求远场方向余弦 → 角度）
//   cosα = τ01·c/(2d·fs)，cosβ = τ23·c/(2d·fs)（τ 为采样点数）
//   cosγ = √(1−cos²α−cos²β)（平面阵取 z>0 支）
//   φ = atan2(cosβ, cosα) ∈ (−180°,+180°]，θ = atan2(cosγ, √(cos²α+cos²β))
//
// 参考模型用 real 按上式现算，与 RTL 输出比对，断言 |误差| ≤ 1 LSB（0.1°）。
// 覆盖：学习期、退化（正上方）、对称点、非对称点、负方位角、超界无效。
//
// ⚠ 本 TB 是**投票版之前**的单帧语义用例（每次 all_done 后 500 拍内应出
//   direction_valid）。现 RTL 为 128 帧六对联合直方图投票，首个有效输出在
//   第 129 帧，单帧不会再出结果 —— 本 TB 的期望已过时，仅保留作编译回归；
//   投票语义的验证见 tb_tdoa_vote.sv。
// ============================================================
`timescale 1ns/1ps
`include "project_global_param.vh"

module tb_tdoa_solver;

    reg clk_50m = 0;
    reg rst_n   = 0;
    always #10 clk_50m = ~clk_50m;

    reg  [TDOA_WIDTH-1:0] tdoa_in [0:MAX_PAIR_NUM-1];
    reg  [MAX_PAIR_NUM-1:0] pair_used;
    reg                   all_done;
    reg  [COORD_WIDTH-1:0] mcx [0:MAX_CHAN_NUM-1];
    reg  [COORD_WIDTH-1:0] mcy [0:MAX_CHAN_NUM-1];
    reg  [FRAME_CNT_WIDTH-1:0] frm_cnt;
    reg                   mask_learn;

    wire signed [15:0] az01, el01;
    wire               dir_valid;

    tdoa_solver uut (
        .clk_50m                (clk_50m),
        .rst_n                  (rst_n),
        .tdoa_result            (tdoa_in),
        .pair_used              (pair_used),
        .all_done               (all_done),
        .mic_coord_x            (mcx),
        .mic_coord_y            (mcy),
        .pre_frame_cnt          (frm_cnt),
        .mask_learn_done        (mask_learn),
        .target_azimuth_01deg   (az01),
        .target_elevation_01deg (el01),
        .direction_valid        (dir_valid)
    );

    integer i, n_err, w;
    reg     got;

    // ---- 参考模型 ----
    real ca_r, cb_r, cg_r, rho_r, phi_r, th_r;
    // 2d = 40 cm（对臂间距，每只麦离中心 20 cm）：
    //   K = c/(2d·fs) = 34300/(40×10000) = 0.08575/点
    //     → /1024(Q10) ×2^23 = 702.464
    // 2026-10-22 由 2d=15cm 恢复；用例 τ 按 40/15 放大回原值（等角变换，期望角度不变）。
    localparam real COS_PER_TAU = 702.464 / 8388608.0;   // 8.3740e-5（精确常数）
    localparam real R2D        = 57.29577951308232;

    task automatic round01(input real d, output integer r);
        begin
            if (d >= 0.0) r = $rtoi(d * 10.0 + 0.5);
            else          r = -$rtoi(-d * 10.0 + 0.5);
        end
    endtask

    task automatic ref_dir(input integer t01, input integer t23,
                           output integer ra, output integer re);
        begin
            ca_r = t01 * COS_PER_TAU;
            cb_r = t23 * COS_PER_TAU;
            rho_r = $sqrt(ca_r*ca_r + cb_r*cb_r);
            cg_r  = $sqrt(1.0 - ca_r*ca_r - cb_r*cb_r);
            phi_r = $atan2(cb_r, ca_r) * R2D;
            th_r  = $atan2(cg_r, rho_r) * R2D;
            round01(phi_r, ra);
            round01(th_r,  re);
        end
    endtask

    // ---- 单用例驱动：返回值写入 got/az/el，invalid 用例等价于 500 拍内无 valid ----
    integer cap_az, cap_el;
    integer lat, lat_max;                  // all_done → direction_valid 的拍数（供 UART_LAT 校核）
    task automatic run_case(input integer t01, input integer t23, input integer learn_en);
        begin
            for (i = 0; i < MAX_PAIR_NUM; i = i + 1) tdoa_in[i] = 0;
            tdoa_in[0] = t01;              // 对 (0,1) → cosα
            tdoa_in[5] = t23;              // 对 (2,3) → cosβ
            pair_used  = {MAX_PAIR_NUM{1'b1}};  // 6 对全部计票（等效旧单帧语义）
            mask_learn = learn_en;
            @(posedge clk_50m);
            all_done = 1'b1;
            @(posedge clk_50m);
            all_done = 1'b0;

            got = 1'b0; cap_az = 0; cap_el = 0; lat = 0;
            for (w = 0; w < 500; w = w + 1) begin
                @(posedge clk_50m);
                if (dir_valid && !got) begin
                    got    = 1'b1;
                    cap_az = az01;
                    cap_el = el01;
                    lat    = w + 1;
                    if (lat > lat_max) lat_max = lat;
                end
            end
        end
    endtask

    integer exp_az, exp_el, d_az, d_el;

    task automatic check_case(input integer t01, input integer t23,
                              input [255:0] name);
        begin
            run_case(t01, t23, 1);
            ref_dir(t01, t23, exp_az, exp_el);
            if (!got) begin
                $error("用例 %s: 期望 valid 但 500 拍内无 direction_valid", name);
                n_err = n_err + 1;
            end else begin
                d_az = cap_az - exp_az;
                d_el = cap_el - exp_el;
                if (d_az < 0) d_az = -d_az;
                if (d_el < 0) d_el = -d_el;
                $display("用例 %-6s τ=(%0d,%0d): φ=%0d θ=%0d | 参考 φ=%0d θ=%0d | 误差 %0d/%0d",
                         name, t01, t23, cap_az, cap_el, exp_az, exp_el, d_az, d_el);
                if (d_az > 1 || d_el > 1) begin
                    $error("用例 %s: 角度误差超限（%0d/%0d LSB）", name, d_az, d_el);
                    n_err = n_err + 1;
                end
            end
        end
    endtask

    task automatic check_invalid(input integer t01, input integer t23,
                                 input [255:0] name);
        begin
            run_case(t01, t23, 1);
            if (got) begin
                $error("用例 %s: 期望无效但输出了 φ=%0d θ=%0d", name, cap_az, cap_el);
                n_err = n_err + 1;
            end else begin
                $display("用例 %-6s τ=(%0d,%0d): 正确判为无效（无 direction_valid）", name, t01, t23);
            end
        end
    endtask

    initial begin
        n_err = 0; lat_max = 0;
        all_done = 0; mask_learn = 0; frm_cnt = 0;
        pair_used = {MAX_PAIR_NUM{1'b1}};
        for (i = 0; i < MAX_PAIR_NUM; i = i + 1) tdoa_in[i] = 0;
        // 十字阵列坐标（与顶层一致，半径 20 cm）
        mcx[0] = -16'sd20;  mcy[0] =  16'sd0;
        mcx[1] =  16'sd20;  mcy[1] =  16'sd0;
        mcx[2] =  16'sd0;   mcy[2] = -16'sd20;
        mcx[3] =  16'sd0;   mcy[3] =  16'sd20;
        for (i = 4; i < MAX_CHAN_NUM; i = i + 1) begin mcx[i] = 0; mcy[i] = 0; end

        rst_n = 0; #200; rst_n = 1;
        repeat (5) @(posedge clk_50m);

        // ---- ① 学习期：掩码未学完 → 必须无效 ----
        run_case(5219, 5219, 0);
        if (got) begin
            $error("用例 1(学习期): 期望无效但输出了 φ=%0d θ=%0d", cap_az, cap_el);
            n_err = n_err + 1;
        end else $display("用例 1(学习期) tau=(5219,5219): 正确判为无效");

        // ---- ② 退化：声源正上方（τ01 = τ23 = 0）----
        run_case(0, 0, 1);
        if (!got) begin
            $error("用例 2(退化): 正上方应判有效（φ=0°、θ=+90°）");
            n_err = n_err + 1;
        end else begin
            $display("用例 2(退化) τ=(0,0): φ=%0d θ=%0d（期望 0/900）", cap_az, cap_el);
            if (cap_az != 0 || cap_el != 900) begin
                $error("用例 2(退化): 期望 φ=0 θ=900，实得 %0d/%0d", cap_az, cap_el);
                n_err = n_err + 1;
            end
        end

        // ---- ③~⑥ 数值用例（参考模型现算）----
        check_case( 5219,  5219, "3-对称");    // 2d = 40 cm 原用例值
        check_case(-3000,  2000, "4-非对称");
        check_case( 4000, -1500, "5-负方位");
        check_case(10000,     0, "6-沿x轴");

        // ---- ⑦⑧ 超界无效 ----
        check_invalid(13000,     0, "7-cos>1");
        check_invalid( 9000,  9000, "8-Scos2>1");

        // UART 发帧节拍校核：phat_position_top 在 all_done 后固定 100 拍锁存结果，
        // 解算延迟必须小于该值（含 1 拍锁存余量）。
        $display("解算延迟 all_done→direction_valid : 最大 %0d 拍（UART_LAT=100）", lat_max);
        if (lat_max >= 99) begin
            $error("解算延迟 %0d 拍 >= UART_LAT(100)-1，UART 会锁存到上一帧结果", lat_max);
            n_err = n_err + 1;
        end

        $display("========================================");
        $display("测向解算错误数 : %0d", n_err);
        $display("========================================");
        if (n_err == 0) $display("PASS");
        else            $display("FAIL");
        $finish;
    end

endmodule
