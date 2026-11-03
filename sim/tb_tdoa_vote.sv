// ============================================================
// tb_tdoa_vote.sv - 128 帧六对联合直方图投票 + 逐对有效性 专项验证
//
// 被验模块：tdoa_solver 的投票/滑窗/置信度路径（2026-10-24 远距改版）
//   注入：直接给 tdoa_result（Q10）与 pair_used，绕过前端。
//
// 场景 A（计票/跳帧语义）：
//   64 帧 6'b110001（01/13/23 有效）+ 10 帧全无效 + 64 帧 6'b100001
//   → 全无效帧不占槽（中期 filled 仍为 64）；
//     填满后 h01[10]=128、h23[2]=128、h13[7]=64；
//     未使用对（02/03/12）一票不投（h03[6]=0）；
//     best=(10,2)、score=640>filled、conf=1。
//
// 场景 B（干净填窗 + 角度 + 滑窗换向）：
//   128 帧 (10,2) 六对全有效 → 六张直方图各有一峰；
//   随后 128 帧 (12,6)，旧票全部滑出（h01[10]→0、h13[7]→0）；
//   两段各检查一次 direction_valid 与角度（参考按档中心现算，±1 LSB）。
//
// 场景 C（纯风声）：复位后 200 帧六对全无效
//   → filled=0、vote_ready=0、全程无 direction_valid。
//
// 角度参考：cos = τ × 727.9375 / 2^23（2d=38.6cm，与 RTL COS_K 一致）。
// 注：投票峰值附近有 ≤0.5 采样点的抛物线插值偏移，折算到角度 <0.03 LSB，
//     故角度断言用档中心作参考、容差 1 LSB 即可。
// ============================================================
`timescale 1ns/1ps
`include "project_global_param.vh"

module tb_tdoa_vote;

    reg clk_50m = 0;
    reg rst_n   = 0;
    always #10 clk_50m = ~clk_50m;

    reg  [TDOA_WIDTH-1:0]   tdoa_in [0:MAX_PAIR_NUM-1];
    reg  [MAX_PAIR_NUM-1:0] pair_used;
    reg                     all_done;
    reg                     mask_learn;
    reg  [COORD_WIDTH-1:0]  mcx [0:MAX_CHAN_NUM-1];
    reg  [COORD_WIDTH-1:0]  mcy [0:MAX_CHAN_NUM-1];
    reg  [FRAME_CNT_WIDTH-1:0] frm_cnt;

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

    integer n_err;
    integer k, t;
    integer cap_az, cap_el;
    integer exp_az, exp_el, d_az, d_el;
    reg    got_valid;

    // ---------------- 结果捕获 ----------------
    always @(posedge clk_50m) begin
        if (dir_valid) begin
            cap_az    <= $signed(az01);
            cap_el    <= $signed(el01);
            got_valid <= 1'b1;
        end
    end

    // ---------------- 参考模型 ----------------
    real ca_r, cb_r, cg_r, rho_r, phi_r, th_r;
    localparam real COS_PER_TAU = 727.9375 / 8388608.0;
    localparam real R2D        = 57.29577951308232;

    task automatic round01(input real d, output integer r);
        begin
            if (d >= 0.0) r = $rtoi(d*10.0 + 0.5);
            else          r = -$rtoi(-d*10.0 + 0.5);
        end
    endtask

    task automatic ref_dir(input integer t01, input integer t23,
                           output integer ra, output integer re);
        begin
            ca_r  = t01 * COS_PER_TAU;
            cb_r  = t23 * COS_PER_TAU;
            rho_r = $sqrt(ca_r*ca_r + cb_r*cb_r);
            cg_r  = $sqrt(1.0 - ca_r*ca_r - cb_r*cb_r);
            phi_r = $atan2(cb_r, ca_r) * R2D;
            th_r  = $atan2(cg_r, rho_r) * R2D;
            round01(phi_r, ra);
            round01(th_r,  re);
        end
    endtask

    // ---------------- 激励 ----------------
    function integer tau_of_bin(input integer b);
        tau_of_bin = (b - 11) * 1024;
    endfunction

    task automatic wait_vote_idle;
        integer g;
        begin
            g = 0;
            while ((uut.vstate != 4'd0) && (g < 20000)) begin
                @(posedge clk_50m); g = g + 1;
            end
            if (g >= 20000) begin
                $error("投票状态机超时，vstate=%0d", uut.vstate);
                n_err = n_err + 1;
            end
        end
    endtask

    // 发一帧。b01/b23 为档号（档 b 中心 = (b−11) 采样点），
    // used 为本帧有效对位图；其余四对按远场恒等式给出档号。
    // 主解算状态机回 S_IDLE 之前发的 all_done 会被漏掉（单周期脉冲），
    // 发帧前等主状态机空闲。
    task automatic wait_solver_idle;
        integer g;
        begin
            g = 0;
            while ((uut.state != 4'd0) && (g < 20000)) begin
                @(posedge clk_50m); g = g + 1;
            end
            if (g >= 20000) begin
                $error("解算状态机超时，state=%0d", uut.state);
                n_err = n_err + 1;
            end
        end
    endtask

    task automatic send_frame(input integer b01, input integer b23,
                              input [5:0] used);
        integer p;                         // 独立循环变量（不能复用调用者的 k）
        begin
            wait_vote_idle;
            wait_solver_idle;
            tdoa_in[0] = tau_of_bin(b01);                                  // 01
            tdoa_in[1] = tau_of_bin(11 + ((b01 - b23) >> 1));              // 02
            tdoa_in[2] = tau_of_bin((b01 + b23) >> 1);                     // 03
            tdoa_in[3] = tau_of_bin(22 - ((b01 + b23) >> 1));              // 12
            tdoa_in[4] = tau_of_bin(22 - (11 + ((b01 - b23) >> 1)));       // 13
            tdoa_in[5] = tau_of_bin(b23);                                  // 23
            if (used == 6'd0)
                for (p = 0; p < MAX_PAIR_NUM; p = p + 1) tdoa_in[p] = 0;
            pair_used = used;
            @(posedge clk_50m); #1;
            all_done = 1'b1;
            @(posedge clk_50m); #1;
            all_done = 1'b0;
        end
    endtask

    task automatic send_n(input integer n, input integer b01, input integer b23,
                          input [5:0] used);
        begin
            for (k = 0; k < n; k = k + 1) send_frame(b01, b23, used);
        end
    endtask

    // 最后一次 send_frame 返回时解算器可能刚进 V_DEC，还没有开始本次搜索；
    // 连续两次等到 V_IDLE，保证搜索/插值/锁存全部走完。
    task automatic settle_search;
        begin
            wait_vote_idle;
            wait_vote_idle;
            repeat (50) @(posedge clk_50m);
        end
    endtask

    task automatic check_int(input integer got, input integer exp,
                             input [255:0] name);
        begin
            if (got !== exp) begin
                $error("%0s: 期望 %0d，实得 %0d", name, exp, got);
                n_err = n_err + 1;
            end else begin
                $display("  [OK] %0s = %0d", name, got);
            end
        end
    endtask

    task automatic send_until_valid(input integer b01, input integer b23,
                                    input [5:0] used);
        begin
            got_valid = 1'b0;
            t = 0;
            while (!got_valid && (t < 40)) begin
                send_frame(b01, b23, used);
                repeat (200) @(posedge clk_50m);
                t = t + 1;
            end
            if (!got_valid) begin
                $error("投票就绪后 40 帧内仍无 direction_valid");
                n_err = n_err + 1;
            end
        end
    endtask

    task automatic check_angle(input integer t01_bin, input integer t23_bin,
                               input [255:0] name);
        begin
            ref_dir((t01_bin-11)*1024, (t23_bin-11)*1024, exp_az, exp_el);
            $display("  %0s 角度输出 φ=%0d θ=%0d | 参考 φ=%0d θ=%0d",
                     name, cap_az, cap_el, exp_az, exp_el);
            d_az = cap_az - exp_az; if (d_az < 0) d_az = -d_az;
            d_el = cap_el - exp_el; if (d_el < 0) d_el = -d_el;
            if (d_az > 1 || d_el > 1) begin
                $error("%0s 角度误差超限（φ %0d / θ %0d LSB）", name, d_az, d_el);
                n_err = n_err + 1;
            end
        end
    endtask

    initial begin
        n_err = 0; all_done = 0; mask_learn = 0; frm_cnt = 0;
        got_valid = 1'b0; cap_az = 0; cap_el = 0;
        exp_az = 0; exp_el = 0; d_az = 0; d_el = 0;
        for (k = 0; k < MAX_PAIR_NUM; k = k + 1) tdoa_in[k] = 0;
        pair_used = 0;
        for (k = 0; k < MAX_CHAN_NUM; k = k + 1) begin mcx[k] = 0; mcy[k] = 0; end

        rst_n = 0; #200; rst_n = 1;
        repeat (5) @(posedge clk_50m);
        mask_learn = 1'b1;

        // ================= 场景 A：计票与跳帧语义 =================
        $display("---- 场景 A：128 帧填窗 + 部分对无效 + 全无效跳帧 ----");
        send_n(64, 10, 2, 6'b110001);          // 01/13/23 三对有效
        settle_search;                         // 第 64 帧会触发一次搜索，等它走完
        check_int(uut.filled, 64, "A1 filled(64 帧后)");
        check_int(uut.h13[7], 64, "A2 h13[7](64 帧后)");

        got_valid = 1'b0;
        send_n(10, 10, 2, 6'b000000);          // 全无效帧：整帧跳过
        check_int(uut.filled, 64, "A3 filled(全无效 10 帧后)");

        send_n(64, 10, 2, 6'b100001);          // 后 64 帧：13 对失效
        settle_search;

        check_int(uut.vote_ready, 1, "A4 vote_ready");
        check_int(uut.filled, 128, "A5 filled");
        check_int(uut.h01[10], 128, "A6 h01[10]");
        check_int(uut.h23[2],  128, "A7 h23[2]");
        check_int(uut.h13[7],   64, "A8 h13[7](仅前 64 帧)");
        check_int(uut.h03[6],    0, "A9 h03[6](未使用对不计票)");
        check_int(uut.best_a,   10, "A10 best_a");
        check_int(uut.best_b,    2, "A11 best_b");
        check_int(uut.best_score, 640, "A12 best_score");
        check_int(uut.conf_ok,    1, "A13 conf_ok");

        // ================= 场景 B：干净填窗、角度、滑窗换向 =================
        $display("---- 场景 B：复位后六对全有效，先 (10,2) 再 (12,6) ----");
        rst_n = 0; #200; rst_n = 1;
        repeat (20) @(posedge clk_50m);
        mask_learn = 1'b1;

        send_n(128, 10, 2, 6'b111111);
        settle_search;
        wait_solver_idle;

        check_int(uut.vote_ready, 1, "B1 vote_ready");
        check_int(uut.h01[10], 128, "B2 h01[10]");
        check_int(uut.h23[2],  128, "B3 h23[2]");
        check_int(uut.h02[15], 128, "B4 h02[15]");
        check_int(uut.h13[7],  128, "B5 h13[7]");
        check_int(uut.h03[6],  128, "B6 h03[6]");
        check_int(uut.h12[16], 128, "B7 h12[16]");
        check_int(uut.best_a,   10, "B8 best_a");
        check_int(uut.best_b,    2, "B9 best_b");
        check_int(uut.conf_ok,   1, "B10 conf_ok");

        send_until_valid(10, 2, 6'b111111);
        check_angle(10, 2, "B11");

        // 换向后：旧方向的票必须全部随槽位滑出
        send_n(128, 12, 6, 6'b111111);
        settle_search;

        check_int(uut.h01[12], 128, "B12 h01[12]");
        check_int(uut.h13[8],  128, "B13 h13[8]");
        check_int(uut.h23[6],  128, "B14 h23[6]");
        check_int(uut.h01[10],   0, "B15 h01[10](旧方向已滑出)");
        check_int(uut.h13[7],    0, "B16 h13[7](旧方向已滑出)");
        check_int(uut.h23[2],    0, "B17 h23[2](旧方向已滑出)");
        check_int(uut.best_a,   12, "B18 best_a");
        check_int(uut.best_b,    6, "B19 best_b");
        check_int(uut.conf_ok,   1, "B20 conf_ok");

        got_valid = 1'b0;
        send_until_valid(12, 6, 6'b111111);
        check_angle(12, 6, "B21");

        // ================= 场景 C：纯风声 =================
        $display("---- 场景 C：复位后 200 帧六对全无效 ----");
        rst_n = 0; #200; rst_n = 1;
        repeat (20) @(posedge clk_50m);
        mask_learn = 1'b1;
        got_valid = 1'b0;

        for (k = 0; k < 200; k = k + 1) begin
            send_frame(11, 11, 6'b000000);
            repeat (20) @(posedge clk_50m);
        end
        check_int(uut.filled, 0, "C1 filled");
        check_int(uut.vote_ready, 0, "C2 vote_ready");
        if (got_valid) begin
            $error("C 纯风声场景不应输出 direction_valid");
            n_err = n_err + 1;
        end else $display("  [OK] C3 全程无 direction_valid");

        $display("========================================");
        $display("投票专项错误数 : %0d", n_err);
        $display("========================================");
        if (n_err == 0) $display("PASS");
        else            $display("FAIL");
        $finish;
    end

endmodule
