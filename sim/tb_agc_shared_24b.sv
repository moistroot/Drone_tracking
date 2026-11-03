// ============================================================
// tb_agc_shared_24b.sv - 共享帧级 AGC 单元验证
//
// 覆盖：
//   1. 初始 0 挡，首帧峰值 lead=9 → 一帧升到 8 挡（+48 dB）；
//   2. 同幅度第二帧不再换挡（gain_changed 不重复）；
//   3. 峰值 lead=20 → 立即降到 2 挡（降挡无迟滞）；
//   4. 峰值 lead=21 → 降到 1 挡；
//   5. 迟滞死区：峰值 lead=20（目标 2 挡），当前 1 挡时
//      输出 lead=21 ≥ TARGET−1 → **保持 1 挡不升**；
//   6. 峰值 lead=19 → 满足 6 dB 死区，升到 3 挡；
//   7. 读侧缩放与 24 位饱和（正/负满幅）；
//   8. ENABLE=0 直通。
// ============================================================
`timescale 1ns/1ps
`include "project_global_param.vh"

module tb_agc_shared_24b;

    reg clk_50m = 0;
    reg rst_n   = 0;
    always #10 clk_50m = ~clk_50m;

    reg                     sample_valid;
    reg  [DATA_WIDTH-1:0]   samp [0:MAX_CHAN_NUM-1];
    reg                     frame_done;
    reg  [DATA_WIDTH-1:0]   din  [0:MAX_CHAN_NUM-1];

    wire [DATA_WIDTH-1:0] dout [0:MAX_CHAN_NUM-1];
    wire [3:0]            gain;
    wire                  gain_changed;

    wire [DATA_WIDTH-1:0] dout_passthru [0:MAX_CHAN_NUM-1];

    task automatic set_din(input integer v);
        for (int c = 0; c < MAX_CHAN_NUM; c = c + 1) din[c] = v;
    endtask

    integer n_err;
    integer n_pulse;

    agc_shared_24b #(.ENABLE(1), .TARGET_BIT(22), .GMAX(8)) u_dut (
        .clk_50m     (clk_50m),
        .rst_n       (rst_n),
        .sample_valid(sample_valid),
        .samp        (samp),
        .frame_done  (frame_done),
        .din         (din),
        .dout        (dout),
        .gain        (gain),
        .gain_changed(gain_changed)
    );

    agc_shared_24b #(.ENABLE(0), .TARGET_BIT(22), .GMAX(8)) u_bypass (
        .clk_50m     (clk_50m),
        .rst_n       (rst_n),
        .sample_valid(1'b0),
        .samp        (samp),
        .frame_done  (1'b0),
        .din         (din),
        .dout        (dout_passthru),
        .gain        (),
        .gain_changed()
    );

    always @(posedge clk_50m) if (gain_changed) n_pulse <= n_pulse + 1;

    task automatic send_sample(input integer v);
        integer c;
        begin
            @(posedge clk_50m); #1;
            sample_valid = 1'b1;
            for (c = 0; c < MAX_CHAN_NUM; c = c + 1) samp[c] = v;
            @(posedge clk_50m); #1;
            sample_valid = 1'b0;
        end
    endtask

    // 4 个样点（峰值 = amp）后给 frame_done（比最后一个 valid 晚一拍）
    task automatic run_frame(input integer amp);
        begin
            send_sample(amp);
            send_sample(-(amp/2));
            send_sample(0);
            send_sample(amp/4);
            @(posedge clk_50m); #1;
            frame_done = 1'b1;
            @(posedge clk_50m); #1;
            frame_done = 1'b0;
            repeat (3) @(posedge clk_50m);
        end
    endtask

    task automatic check_int(input integer got, input integer exp,
                             input [255:0] name);
        begin
            if (got !== exp) begin
                $error("%0s: 期望 %0d，实得 %0d", name, exp, got);
                n_err = n_err + 1;
            end else $display("  [OK] %0s = %0d", name, got);
        end
    endtask

    initial begin
        n_err = 0; n_pulse = 0;
        sample_valid = 0; frame_done = 0;
        for (int c = 0; c < MAX_CHAN_NUM; c = c + 1) begin
            samp[c] = 0;
            din[c]  = 0;
        end

        rst_n = 0; #200; rst_n = 1;
        repeat (5) @(posedge clk_50m);
        check_int(gain, 0, "1 复位增益");

        // ---- 1. 首帧峰 lead=9 → 8 挡 ----
        run_frame(1000);
        check_int(gain, 8, "2 首帧(lead=9)增益");
        check_int(n_pulse, 1, "3 换挡脉冲数");

        // ---- 2. 同幅度第二帧：不换挡 ----
        run_frame(1000);
        check_int(gain, 8, "4 同幅度保持");
        check_int(n_pulse, 1, "5 无重复脉冲");

        // ---- 3. 峰 lead=20 → 降到 2 挡 ----
        run_frame(1 << 20);
        check_int(gain, 2, "6 降挡(lead=20)");
        check_int(n_pulse, 2, "7 降挡脉冲");

        // ---- 4. 峰 lead=21 → 降到 1 挡 ----
        run_frame(1 << 21);
        check_int(gain, 1, "8 降挡(lead=21)");

        // ---- 5. 迟滞：目标 2 挡但输出 lead=21 → 保持 1 ----
        run_frame(1 << 20);
        check_int(gain, 1, "9 死区不升挡");

        // ---- 6. 峰 lead=19 → 升到 3 挡 ----
        run_frame(1 << 19);
        check_int(gain, 3, "10 死区外升挡");

        // ---- 7. 读侧缩放与饱和（4 路并行，逐路检查） ----
        set_din(24'sd1000);
        #1;
        for (int c = 0; c < MAX_CHAN_NUM; c = c + 1)
            check_int($signed(dout[c]), 8000, "11 dout=din<<3");

        set_din(-24'sd1000);
        #1;
        for (int c = 0; c < MAX_CHAN_NUM; c = c + 1)
            check_int($signed(dout[c]), -8000, "12 负输入缩放");

        set_din(24'sh7FFFFF);       // 正满幅：<<3 饱和到 0x7FFFFF
        #1;
        for (int c = 0; c < MAX_CHAN_NUM; c = c + 1)
            check_int($unsigned(dout[c]), 24'h7FFFFF, "13 正饱和");

        set_din(24'h800000);        // 负满幅（-2^23）：<<3 饱和到 0x800000
        #1;
        for (int c = 0; c < MAX_CHAN_NUM; c = c + 1)
            check_int($unsigned(dout[c]), 24'h800000, "14 负饱和");

        // ---- 8. ENABLE=0 直通 ----
        set_din(24'sd12345);
        #1;
        for (int c = 0; c < MAX_CHAN_NUM; c = c + 1)
            check_int($signed(dout_passthru[c]), 12345, "15 旁路直通");

        $display("========================================");
        $display("AGC 单元错误数 : %0d", n_err);
        $display("========================================");
        if (n_err == 0) $display("PASS");
        else            $display("FAIL");
        $finish;
    end

endmodule
