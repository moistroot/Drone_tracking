// ============================================================
// tb_audio_drone.sv - 用「真实无人机录音」驱动分包3 测向全链路
//
// 激励来源：sim_audio/audio_to_mem.py 生成的 drone_frames.mem
//   —— 录音按 256 点/帧做汉宁窗 + FFT，按目标方向乘时延相位
//      e^{-j2πkΔτ/256}，再叠每路独立带内噪声与一路带外低频干扰，
//      全局定标到 2^22 量化成 24 位。每帧 129 频点 × 4 通道 × (re,im)。
//   第 0~2 帧为「安静段」（学习期噪声基底），第 3 帧起为无人机段。
//
// 本 TB 的目的：在真实素材上比较 4 种掩码配置，验证 doc/07 的两处改动
//   A: 全带 + 现行门限（等价 E > 平均×0.375）
//   B: 全带 + doc/07 重写门限（E > 平均×2）
//   C: 带通 k=26..127 + 现行门限   ← **2026-10-05 起 RTL 已采用此配置**
//   D: 带通 k=26..127 + 重写门限（doc/07 原推荐组合；实测会让弱帧崩盘，见 doc/08 §6.6）
//
// 掩码由 TB 内的「副本」产生（与 freq_noise_mask.sv 逐行等价，
// 仅把阈值/带通做成可配），同时并联一个**真实 freq_noise_mask 实例**，
// 配置 C 下两者逐位比对，用于证明副本忠实（RTL 已含带通门，故绑 C 而非 A）。
//
// 时序约定（与 doc/02 一致）：掩码「帧内恒定」——第 N 帧突发期间用第 N-1
//   帧算出的掩码，帧末锁存更新。故第 4 帧起才有有效输出。
//
// 运行：xsim 时可用 -testplusarg MEM=<路径> 覆盖激励文件位置。
// ============================================================
`timescale 1ns/1ps
`include "project_global_param.vh"

module tb_audio_drone;

    localparam integer NFRAME   = 27;              // 3 学习 + 24 测量（NFRM 覆盖测量帧数）
    localparam integer NBIN     = 129;
    localparam integer MEMSIZE  = NFRAME*NBIN*4*2;
    localparam integer LEARN_N  = 3;

    // 目标方向（与 audio_to_mem.py 的 --phi/--theta 一致）
    localparam real PHI_DEG = 45.0;
    localparam real TH_DEG  = 30.0;
    // 带通门（doc/07 §3）：k=26 → 1016 Hz；高端取到 Nyquist
    localparam integer BIN_LO = 26;
    localparam integer BIN_HI = 127;
    // 每帧等待解算结果的超时（50 MHz 周期）
    localparam integer WAIT_MAX = 50000;

    // ---------------- 激励 ----------------
    reg [23:0] stim [0:MEMSIZE-1];
    reg [8*260-1:0] mem_path;

    reg         clk_50m;
    reg         rst_n;
    reg         pre_frame_valid;
    reg  [9:0]  fft_idx;
    reg  [23:0] fft_re [0:MAX_CHAN_NUM-1];
    reg  [23:0] fft_im [0:MAX_CHAN_NUM-1];
    reg  [FFT_BINS-1:0] mask_applied; // TB 掩码副本输出（帧内恒定，馈给 DUT）
    wire        mask_learn_done;     // TB 侧学习完成（与副本学习计数同步）
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
        .freq_valid_mask        (mask_applied),
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
        mask_applied = 0; pre_frame_cnt = 0; fft_ip_latency = 310;
        mic_coord_x[0] = -20; mic_coord_y[0] =   0;
        mic_coord_x[1] =  20; mic_coord_y[1] =   0;
        mic_coord_x[2] =   0; mic_coord_y[2] = -20;
        mic_coord_x[3] =   0; mic_coord_y[3] =  20;
    end

    always #10 clk_50m = ~clk_50m;

    // ================= 掩码副本（与 freq_noise_mask.sv 行行对应）=================
    integer cfg;                                   // 0=A 1=B 2=C 3=D
    reg [25:0] accum_tb [0:FFT_BINS-1];
    reg [1:0]  learn_cnt_tb;
    assign mask_learn_done = (learn_cnt_tb >= LEARN_N);
    reg [FFT_BINS-1:0] mask_tb, mask_rtl_lat;
    reg        relearn_pulse;
    reg        end_pulse;
    wire [FFT_BINS-1:0] rtl_mask;                  // 真实 freq_noise_mask 实例输出

    wire signed [23:0] w_re = fft_re[0];
    wire signed [23:0] w_im = fft_im[0];
    wire        burst_valid = pre_frame_valid && (fft_idx <= 128);

    wire signed [47:0] t_re_sq = w_re * w_re;
    wire signed [47:0] t_im_sq = w_im * w_im;
    wire [23:0] energy_tb = t_re_sq[46:23] + t_im_sq[46:23];

    // A：现状写法（2026-10-23 起 RTL 为 8E > accum ⇒ E > 平均×0.375；
    //    此前是 5E ⇒ 0.6，TB 副本未同步导致配置 C 交叉验证失配，2026-10-24 修正）
    wire [26:0] e8_tb  = {energy_tb, 3'b000};
    wire [26:0] ax2_tb = {accum_tb[fft_idx], 1'b0};
    wire        pass_A = (e8_tb > {1'b0, accum_tb[fft_idx]});
    // B/D：doc/07 重写 3E > 2×accum ⇒ E > 平均×2
    wire [26:0] e3_tb  = {2'b00, energy_tb, 1'b0} + {3'b000, energy_tb};
    wire        pass_B = (e3_tb > ax2_tb);
    wire        inband = (fft_idx >= BIN_LO) && (fft_idx <= BIN_HI);

    wire pass_sel   = (cfg == 0 || cfg == 2) ? pass_A : pass_B;
    wire inband_sel = (cfg >= 2) ? inband : 1'b1;

    integer kk2;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            learn_cnt_tb <= 0; mask_tb <= 0; mask_applied <= 0; mask_rtl_lat <= 0;
            for (kk2 = 0; kk2 < FFT_BINS; kk2 = kk2 + 1) accum_tb[kk2] <= 26'b0;
        end else if (relearn_pulse) begin
            // mask_rtl_lat 也要清零：否则重新学习的 2000 拍空闲里，它会一直
            // 停在上一配置的掩码快照，交叉验证会把它当成"当前掩码"而误报。
            learn_cnt_tb <= 0; mask_tb <= 0; mask_applied <= 0; mask_rtl_lat <= 0;
            for (kk2 = 0; kk2 < FFT_BINS; kk2 = kk2 + 1) accum_tb[kk2] <= 26'b0;
        end else begin
            if (burst_valid) begin
                if (learn_cnt_tb < LEARN_N)
                    accum_tb[fft_idx] <= accum_tb[fft_idx] + {{2{1'b0}}, energy_tb};
                else
                    mask_tb[fft_idx] <= pass_sel && inband_sel;
            end
            if (end_pulse) begin
                if (learn_cnt_tb < LEARN_N) learn_cnt_tb <= learn_cnt_tb + 1'b1;
                else                        learn_cnt_tb <= LEARN_N;
                mask_applied <= mask_tb;                  // 帧末锁存 → 下一帧恒定
                mask_rtl_lat <= rtl_mask;                 // 同步锁存真实模块输出
            end
        end
    end

    // ---- 并联的真实掩码模块（仅用于逐位交叉验证）----
    // 副本在 relearn_pulse 时清零并重新学习，真实模块也必须同步复位，
    // 否则一次跑 A/B/C/D 四个配置时，配置 C 比的是“重新学习的副本”与
    // “还停留在配置 A 噪声基底的真实模块”，必然失配。
    // 单独跑 +CFGSEL=2 时两者都从复位开始，故以前只在单配置模式下通过。
    wire mask_rst_n = rst_n & ~relearn_pulse;
    freq_noise_mask u_mask_rtl (
        .clk_50m        (clk_50m),
        .rst_n          (mask_rst_n),
        .valid          (burst_valid),
        .idx            (fft_idx),
        .re             (fft_re[0]),
        .im             (fft_im[0]),
        .frame_end      (end_pulse),
        .restart        (1'b0),
        .freq_valid_mask(rtl_mask),
        .learn_done     ()
    );

    // 当前生效掩码的位数（0..128）
    integer popc_now;
    integer q;
    always @* begin
        popc_now = 0;
        for (q = 0; q <= 128; q = q + 1) if (mask_applied[q]) popc_now = popc_now + 1;
    end

    // 配置 C 下副本 vs 真实模块的逐位比对。
    // 注：2026-10-05 起 RTL 已含带通门（doc/07 改动①），RTL 行为 == 配置 C，
    //     故交叉验证绑 cfg==2；绑 cfg==0 会必然失配。
    // 只在每个帧末（end_pulse 后一拍，两侧锁存值都已更新）比一次：
    // 按静止电平逐拍比较会把重新学习/切换配置时 1~3 拍的过渡态计入，
    // 而真正的掩码位差异会持续整帧，逐帧比对同样能抓到。
    reg end_pulse_d;
    always @(posedge clk_50m) end_pulse_d <= end_pulse;

    integer cmp_err;
    always @(posedge clk_50m) begin
        if (rst_n && (cfg == 2) && end_pulse_d) begin
            if (mask_applied !== mask_rtl_lat) begin
                cmp_err <= cmp_err + 1;
                $display("  [交叉验证] 帧末掩码不一致：副本 %0d 位 vs 真实 %0d 位",
                         $countones(mask_applied), $countones(mask_rtl_lat));
            end
        end
    end

    // ---------------- 结果捕获 ----------------
    integer nval, n0, w;
    reg signed [15:0] last_az, last_el, last_t01, last_t23;
    always @(posedge clk_50m) begin
        if (direction_valid) begin
            last_az  <= $signed(target_azimuth_01deg);
            last_el  <= $signed(target_elevation_01deg);
            last_t01 <= $signed(tdoa_result[0]);
            last_t23 <= $signed(tdoa_result[5]);
            nval     <= nval + 1;
        end
    end

    // ---------------- DUT 内部观测（诊断用，+DIAG=1 打开）----------------
    integer diag, hb;
    initial begin diag = 0; hb = 0; end
    always @(posedge clk_50m) begin
        if (diag) begin
            hb = hb + 1;
            if (uut.all_done)
                $display("    [DUT] t=%0t all_done τ01=%0d τ23=%0d learn_done=%0d",
                         $time, $signed(uut.sched_tdoa[0]), $signed(uut.sched_tdoa[5]), mask_learn_done);
            if (uut.dir_valid_raw)
                $display("    [DUT] t=%0t dir_valid_raw φ=%0d θ=%0d",
                         $time, $signed(uut.az01), $signed(uut.el01));
            if (uut.u_scheduler.busy && (hb % 4000 == 0))
                $display("    [DUT] t=%0t sched_state=%0d pair=%0d freq=%0d peak_dstate=%0d",
                         $time, uut.u_scheduler.state, uut.u_scheduler.pair_cnt,
                         uut.u_scheduler.freq_cnt, uut.u_peak.dstate);
        end
    end

    // ---------------- PHAT 缓存探针（+PBUF=1，只在配置A、测量帧上打印）----------------
    // all_done 时 phat_buf 里是最后一个配对 (2,3) 的 PHAT 单位矢量；
    // 打印出来可与 Python 模型逐频点比相位，判断偏差出在 PHAT 之前还是之后。
    integer pbuf_on, pk;
    initial pbuf_on = 0;
    always @(posedge clk_50m) begin
        if (uut.all_done && pbuf_on && (cfg == 0) &&
            (pre_frame_cnt >= 3) && (pre_frame_cnt <= 4)) begin
            $write("    [PBUF] 帧%0d TB掩码 ", pre_frame_cnt);
            for (pk = 0; pk <= 128; pk = pk + 1)
                $write("%0d", mask_applied[pk]);
            $write("\n    [PBUF] 帧%0d DUT掩码 ", pre_frame_cnt);
            for (pk = 0; pk <= 128; pk = pk + 1)
                $write("%0d", uut.u_scheduler.freq_valid_mask[pk]);
            $write("\n    [PBUF] 帧%0d 非零位 ", pre_frame_cnt);
            for (pk = 0; pk <= 128; pk = pk + 1)
                $write("%0d", (uut.u_scheduler.phat_buf[pk] != 48'b0));
            $write("\n");
        end
    end

    // ---------------- 几何真值（平面波 + 远场公式）----------------
    integer exp_t01, exp_t23;
    real ux, uy, uz, pr, tr;
    initial begin
        pr = PHI_DEG*3.14159265358979323846/180.0;
        tr = TH_DEG *3.14159265358979323846/180.0;
        ux = $cos(tr)*$cos(pr);
        uy = $cos(tr)*$sin(pr);
        uz = $sin(tr);
        // τ01 = ((m1−m0)·u)/3.43 ×1024，τ23 = ((m3−m2)·u)/3.43 ×1024
        exp_t01 = (40.0*ux/3.43)*1024.0;
        exp_t23 = (40.0*uy/3.43)*1024.0;
    end

    function integer dabs(input integer x);
        dabs = (x < 0) ? -x : x;
    endfunction

    // ---------------- 统计数组 ----------------
    integer e01 [0:63], e23 [0:63], pc_arr [0:63];
    integer nmeas, sum01, sum23, max01, max23, nvalid_cfg, nzero_out;
    string cname;

    task automatic do_frame(input integer fn, input integer is_meas, input integer midx);
        begin
            for (int bi = 0; bi <= 128; bi = bi + 1) begin
                @(posedge clk_50m); #2;
                pre_frame_valid <= 1;
                fft_idx <= bi[9:0];
                for (int ci = 0; ci < MAX_CHAN_NUM; ci = ci + 1) begin
                    fft_re[ci] <= stim[((fn*129 + bi)*4 + ci)*2 + 0];
                    fft_im[ci] <= stim[((fn*129 + bi)*4 + ci)*2 + 1];
                end
            end
            @(posedge clk_50m); #2;
            n0 = nval;
            pc_arr[midx] = (is_meas) ? popc_now : 0;   // 本帧突发期间生效的掩码位数
            pre_frame_valid <= 0;
            end_pulse <= 1;
            pre_frame_cnt <= fn;
            @(posedge clk_50m); #2;
            end_pulse <= 0;

            // 等本帧 6 配对处理完（all_done 脉冲，实测约 2 万拍），再留解算 81 拍余量
            w = 0;
            while (!uut.all_done && (w < WAIT_MAX)) begin
                @(posedge clk_50m); w = w + 1;
            end
            repeat (300) @(posedge clk_50m);
        end
    endtask

    task automatic report_cfg(input integer c);
        begin
            sum01 = 0; sum23 = 0; max01 = 0; max23 = 0; nvalid_cfg = 0; nzero_out = 0;
            for (int i = 0; (i < nmeas) && (i < 64); i = i + 1) begin
                if (pc_arr[i] == 0) nzero_out = nzero_out + 1;
                if (e01[i] != -99999) begin
                    nvalid_cfg = nvalid_cfg + 1;
                    sum01 = sum01 + dabs(e01[i]);
                    sum23 = sum23 + dabs(e23[i]);
                    if (dabs(e01[i]) > max01) max01 = dabs(e01[i]);
                    if (dabs(e23[i]) > max23) max23 = dabs(e23[i]);
                end
            end
            $display("==== 配置 %s ====", cname);
            $display("  有效输出 %0d/%0d 帧；掩码为 0（全灭）的帧数 %0d",
                     nvalid_cfg, nmeas, nzero_out);
            if (nvalid_cfg > 0)
                $display("  |τ01误差| 均值 %0d、最大 %0d ；|τ23误差| 均值 %0d、最大 %0d （Q10，1/1024 采样点）",
                         sum01/nvalid_cfg, max01, sum23/nvalid_cfg, max23);
            $display("  结果 φ=%0d θ=%0d（%.1f°/%.1f°），真值 %.1f°/%.1f°",
                     last_az, last_el, $itor(last_az)/10.0, $itor(last_el)/10.0, PHI_DEG, TH_DEG);
        end
    endtask

    // ================= 主流程 =================
    integer fn2, midx, c2, ncfg, nfrm, cfgsel, c0, c1;
    initial begin
        cfg = 0; cmp_err = 0; nval = 0; n0 = 0; nmeas = 0;
        for (int i = 0; i < 64; i = i + 1) begin
            e01[i] = -99999; e23[i] = -99999; pc_arr[i] = 0;
        end
        last_az = 0; last_el = 0; last_t01 = 0; last_t23 = 0;
        relearn_pulse = 0; end_pulse = 0;
        if (!$value$plusargs("MEM=%s", mem_path))
            mem_path = "D:/vivado_project/FPGA/GCC-PHAT/GCC-PHAT/sim_audio/drone_frames.mem";
        if (!$value$plusargs("NCFG=%d", ncfg)) ncfg = 4;
        if (!$value$plusargs("CFGSEL=%d", cfgsel)) cfgsel = -1;
        if (!$value$plusargs("NFRM=%d", nfrm)) nfrm = NFRAME;
        if (!$value$plusargs("DIAG=%d", diag)) diag = 0;
        if (!$value$plusargs("PBUF=%d", pbuf_on)) pbuf_on = 0;
        $readmemh(mem_path, stim);

        $display("================================================================");
        $display(" 真实无人机录音全链路仿真（%0d 帧 / 帧长 256 点 / 10 kHz）", NFRAME);
        $display(" 注入方向 φ=%.1f° θ=%.1f° → 几何真值 τ01=%0d τ23=%0d (Q10)",
                 PHI_DEG, TH_DEG, exp_t01, exp_t23);
        $display(" 带通门 k=%0d..%0d（%.0f~%.0f Hz）",
                 BIN_LO, BIN_HI, BIN_LO*10000.0/256.0, BIN_HI*10000.0/256.0);
        $display(" 激励 %0s", mem_path);
        $display("================================================================");

        #500; rst_n = 1;
        repeat (500) @(posedge clk_50m);

        // CFGSEL≥0 时只跑单个配置（便于 4 个配置并行开 4 个仿真）
        if (cfgsel >= 0) begin c0 = cfgsel; c1 = cfgsel + 1; end
        else             begin c0 = 0;      c1 = ncfg;      end
        for (c2 = c0; c2 < c1; c2 = c2 + 1) begin
            cfg = c2;
            case (c2)
                0: cname = "A 全带 + 现状门限(x0.375)";
                1: cname = "B 全带 + 重写门限(x2)";
                2: cname = "C 带通26~127 + 现状门限(x0.375)";
                default: cname = "D 带通26~127 + 重写门限(x2)";
            endcase
            // 重新学习（副本清零；DUT 见掩码全 0 即重入学习期，无需复位）
            @(posedge clk_50m); #2; relearn_pulse <= 1;
            @(posedge clk_50m); #2; relearn_pulse <= 0;
            repeat (2000) @(posedge clk_50m);

            nmeas = 0;
            for (fn2 = 0; fn2 < nfrm; fn2 = fn2 + 1) begin
                if (fn2 < LEARN_N) begin
                    do_frame(fn2, 0, 0);
                    if (cfg == 2)
                        $display("  [学习] 帧%0d 学习计数=%0d 掩码输出恒 0", fn2, learn_cnt_tb);
                end else begin
                    int ok;
                    do_frame(fn2, 1, fn2 - LEARN_N);
                    ok = (nval > n0);
                    e01[fn2-LEARN_N] = ok ? (last_t01 - exp_t01) : -99999;
                    e23[fn2-LEARN_N] = ok ? (last_t23 - exp_t23) : -99999;
                    $display("  [测量] 帧%0d 掩码位数=%0d/129 有效=%0d τ01=%0d τ23=%0d φ=%0d θ=%0d",
                             fn2, pc_arr[fn2-LEARN_N], ok, last_t01, last_t23, last_az, last_el);
                    nmeas = nmeas + 1;
                end
            end
            report_cfg(c2);
        end

        $display("================================================================");
        if (cmp_err == 0)
            $display("[交叉验证] 配置C：TB 掩码副本与 freq_noise_mask 实例逐帧逐位一致 (OK)");
        else
            $display("[交叉验证] 配置C：副本与真实模块有 %0d 次不一致（需检查副本）", cmp_err);
        $display(" 说明：有效输出从第 4 帧起（掩码帧内恒定，前 3 帧为学习期）。");
        $display("================================================================");
        if (cmp_err == 0) $display("PASS");
        else              $display("FAIL");
        $finish;
    end

endmodule
