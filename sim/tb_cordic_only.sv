// ============================================================
// tb_cordic_only.sv - CORDIC PHAT 归一化：角度量程 + 新增角度输出专项验证
//
// 该模块无握手信号、流水线自由运转：把输入向量保持足够多拍，输出即稳定。
// 角度定义统一为 atan2(im, re)。
//   ① 用输出单位向量 (re_out, im_out) 反推角度（原验证方式）；
//   ② 用新增的 angle_out 端口（Q26，2^26 = 2π）直接读角度并断言误差；
//   ③ 断言 angle_valid：正常向量应为 1，零向量应为 0。
// ============================================================
`timescale 1ns/1ps

module tb_cordic_only;

    reg         clk_50m;
    reg         rst_n;
    reg  [23:0] re_in, im_in;
    wire [23:0] re_out, im_out;
    wire signed [25:0] angle_out;
    wire               angle_valid;

    cordic_phat_norm_24b uut (
        .clk_50m     (clk_50m),
        .rst_n       (rst_n),
        .re_in       (re_in),
        .im_in       (im_in),
        .re_out      (re_out),
        .im_out      (im_out),
        .angle_out   (angle_out),
        .angle_valid (angle_valid)
    );

    initial begin clk_50m = 0; rst_n = 0; re_in = 0; im_in = 0; #200; rst_n = 1; end
    always #10 clk_50m = ~clk_50m;

    real pi2, meas, err, meas_a, err_a;
    integer deg, sdeg, n_err;

    // Q26 域：2^26 = 360°
    localparam real Q26_PER_DEG = 67108864.0 / 360.0;   // 186413.51
    localparam real TOL_DEG    = 0.05;                  // 允许角度误差

    task automatic check_angle(input integer d);
        begin
            re_in = $rtoi(4194304.0 * $cos(pi2*d/360.0));
            im_in = $rtoi(4194304.0 * $sin(pi2*d/360.0));
            repeat (60) @(posedge clk_50m);

            // ① 由单位向量反推
            meas = $atan2($signed(im_out), $signed(re_out)) * 57.2957795;
            err  = meas - d;
            if (err > 180.0) err = err - 360.0;
            if (err < -180.0) err = err + 360.0;

            // ② 直接读 angle_out（Q26 → 度）
            meas_a = angle_out / Q26_PER_DEG;
            err_a  = meas_a - d;
            if (err_a > 180.0) err_a = err_a - 360.0;
            if (err_a < -180.0) err_a = err_a + 360.0;

            $display("CORDIC: 输入 %0d° → 向量法 %0.2f° (误差 %0.3f°) | angle_out %0.2f° (误差 %0.3f°) valid=%b",
                     d, meas, err, meas_a, err_a, angle_valid);

            if (err > 0.5 || err < -0.5) begin
                $error("向量法角度误差超限: 输入 %0d° 误差 %f°", d, err);
                n_err = n_err + 1;
            end
            if (err_a > TOL_DEG || err_a < -TOL_DEG) begin
                $error("angle_out 误差超限: 输入 %0d° 测得 %f° 误差 %f°", d, meas_a, err_a);
                n_err = n_err + 1;
            end
            if (angle_valid !== 1'b1) begin
                $error("angle_valid 应为 1: 输入 %0d°", d);
                n_err = n_err + 1;
            end
        end
    endtask

    // ---- 小值门控对齐（回归用例）----
    // 门控判据是 |v| < R_MIN = 100（SMALL_LIM = R_MIN·K，见 cordic_phat_norm_24b.sv），
    // 幅值 60 的小向量应被清零。
    // 若门控与数据错拍（例如误取相邻拍的模长），小向量会被放过、而别处的大向量被清零。
    // 本用例把一个小向量 @60° 夹在大向量 @30° 之间：
    //   正确 → 输出流里恰好一处全零，其余都是 30°（|re| > |im|）
    //   错拍 → 零出现在别的拍上，且漏出一个 60° 的单位向量（|re| < |im|）
    task automatic check_gate_align;
        integer k, n_zero, zidx, n_leak, ar, ai;
        reg [23:0] cap_re [0:139], cap_im [0:139];
        begin
            n_zero = 0; zidx = -1; n_leak = 0;
            for (k = 0; k < 140; k = k + 1) begin
                @(posedge clk_50m); #2;
                cap_re[k] = re_out;                 // 本拍采样到的是 43 拍前送入的向量
                cap_im[k] = im_out;
                if (k == 40) begin                  // 第 40 次激励：小向量 @60°，幅值 60
                    re_in = 24'sd30; im_in = 24'sd52;
                end else begin                      // 其余：大向量 @30°，幅值 2^22
                    re_in = 24'sd3632961; im_in = 24'sd2097152;
                end
            end
            // cap[0..43] 是激励前残留在流水线里的零，不计；
            // 第 j 次激励的响应落在 cap[j+43]（本 TB 采样约定，实测标定：第 40 次 → cap[83]）
            for (k = 44; k <= 124; k = k + 1) begin
                if ((cap_re[k] == 24'sd0) && (cap_im[k] == 24'sd0)) begin
                    n_zero = n_zero + 1; zidx = k;
                end else begin
                    ar = $signed(cap_re[k]);  if (ar < 0) ar = -ar;
                    ai = $signed(cap_im[k]);  if (ai < 0) ai = -ai;
                    if (ar <= ai) n_leak = n_leak + 1;   // 出现 60° 分量 = 小向量漏出
                end
            end
            $display("门控对齐: 窗内零输出 %0d 处（索引 %0d，应为 83 即第 40 次激励）；漏出的小向量 %0d 个",
                     n_zero, zidx, n_leak);
            if (n_leak != 0) begin
                $error("小值门控与数据错拍：%0d 处漏出了本应清零的小向量", n_leak);
                n_err = n_err + 1;
            end
            if ((n_zero != 1) || (zidx != 83)) begin
                $error("小值门控清零位置不对：期望 1 处且索引 83，实为 %0d 处、索引 %0d",
                       n_zero, zidx);
                n_err = n_err + 1;
            end
        end
    endtask

    initial begin
        pi2   = 6.283185307179586;
        n_err = 0;
        @(posedge rst_n);
        repeat (5) @(posedge clk_50m);

        // ---- 粗扫：0..350 步进 10（含 ±90°、±180° 边界）----
        for (deg = 0; deg < 360; deg = deg + 10) check_angle(deg);

        // ---- 精扫：±85..±100（象限预处理切换区）----
        for (deg = 85; deg <= 100; deg = deg + 5) begin
            check_angle(deg);
            check_angle(-deg);
        end

        // ---- 大角：±170（重点验证 flip 预旋转 180° 的补偿）----
        check_angle(170);
        check_angle(-170);
        check_angle(179);
        check_angle(-179);

        // ---- 零向量：小值保护应给出 angle_valid = 0 ----
        re_in = 0; im_in = 0;
        repeat (60) @(posedge clk_50m);
        $display("零向量: angle_valid = %b（应为 0）", angle_valid);
        if (angle_valid !== 1'b0) begin
            $error("零向量 angle_valid 应为 0，实为 %b", angle_valid);
            n_err = n_err + 1;
        end

        // ---- 小向量夹在大向量之间：门控必须与数据同拍（错拍即漏出）----
        check_gate_align;

        $display("========================================");
        $display("CORDIC 角度校验错误数 : %0d", n_err);
        $display("========================================");
        if (n_err == 0) $display("PASS");
        else            $display("FAIL");
        $finish;
    end

endmodule
