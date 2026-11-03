// ============================================================
// tb_servo_pan_tilt.sv - SG90 云台舵机驱动专项验证
//
// 验证点：
//   ① PWM 时基：周期 20 ms（仿真中按参数缩比为 2 ms，比例一致）
//   ② 脉宽映射：0~180° 对应 PULSE_MIN~PULSE_MAX，钳位正确
//   ③ 翻转映射：方位角 -180°~0° 时 pan=az+180°、tilt=180°-el
//      用户示例 (-150°, +30°) → 云台 (30°, 150°)
//   ④ 机械限位：tilt 不低于 10°
//   ⑤ 保持：update 不来时，输入变化不影响输出
//
// 仿真加速：PWM_PERIOD_CYC 取 100_000（2 ms），脉宽同比缩为
//   5_000~25_000 拍（0.1~0.5 ms）。换算比例与真实参数完全一致，
//   默认参数 1_000_000 / 25_000 / 125_000 只在实现时生效。
// ============================================================
`timescale 1ns/1ps

module tb_servo_pan_tilt;

    localparam integer PWM_PERIOD_CYC = 100_000;   // 仿真用 2 ms
    localparam integer PULSE_MIN_CYC  =   5_000;   // 对应 0°
    localparam integer PULSE_MAX_CYC  =  25_000;   // 对应 180°
    localparam integer TILT_MIN_01DEG =     100;   // 俯仰机械下限 10.0°

    reg                clk_50m;
    reg                rst_n;
    reg  signed [15:0] az_01deg;
    reg  signed [15:0] el_01deg;
    reg                update;
    wire               pan_pwm;
    wire               tilt_pwm;

    integer n_pass, n_fail;
    integer exp_pan, exp_tilt;      // 期望云台角度（0.1°）
    integer wp, wt;                 // 实测高电平拍数

    servo_pan_tilt #(
        .PWM_PERIOD_CYC (PWM_PERIOD_CYC),
        .PULSE_MIN_CYC  (PULSE_MIN_CYC),
        .PULSE_MAX_CYC  (PULSE_MAX_CYC),
        .TILT_MIN_01DEG (TILT_MIN_01DEG)
    ) dut (
        .clk_50m  (clk_50m),
        .rst_n    (rst_n),
        .az_01deg (az_01deg),
        .el_01deg (el_01deg),
        .update   (update),
        .pan_pwm  (pan_pwm),
        .tilt_pwm (tilt_pwm)
    );

    initial begin clk_50m = 1'b0; end
    always #10 clk_50m = ~clk_50m;

    function automatic integer exp_cyc(input integer ang_01deg);
        exp_cyc = PULSE_MIN_CYC + (ang_01deg * (PULSE_MAX_CYC - PULSE_MIN_CYC)) / 1800;
    endfunction

    // 等待下一组 PWM 上升沿，量出两路高电平拍数
    task automatic measure_next(output integer wp_out, output integer wt_out);
        integer np, nt;
        begin
            np = 0;
            nt = 0;
            fork
                begin
                    @(posedge pan_pwm);
                    while (pan_pwm) begin
                        @(posedge clk_50m);
                        np = np + 1;
                    end
                end
                begin
                    @(posedge tilt_pwm);
                    while (tilt_pwm) begin
                        @(posedge clk_50m);
                        nt = nt + 1;
                    end
                end
            join
            wp_out = np;
            wt_out = nt;
        end
    endtask

    task automatic check_widths(input [255:0] name);
        begin
            if ((wp >= exp_cyc(exp_pan) - 2) && (wp <= exp_cyc(exp_pan) + 2) &&
                (wt >= exp_cyc(exp_tilt) - 2) && (wt <= exp_cyc(exp_tilt) + 2)) begin
                n_pass = n_pass + 1;
                $display("PASS %0s: pan=%0d.%0d deg (w=%0d)  tilt=%0d.%0d deg (w=%0d)",
                         name, exp_pan/10, exp_pan%10, wp, exp_tilt/10, exp_tilt%10, wt);
            end else begin
                n_fail = n_fail + 1;
                $display("FAIL %0s: pan w=%0d (期望 %0d), tilt w=%0d (期望 %0d)",
                         name, wp, exp_cyc(exp_pan), wt, exp_cyc(exp_tilt));
            end
        end
    endtask

    // 打一拍 update，再等 2 个周期边界（确保新脉宽已装入活动寄存器）
    task automatic apply_angles(input signed [15:0] az_v, input signed [15:0] el_v);
        begin
            @(negedge clk_50m);
            az_01deg = az_v;
            el_01deg = el_v;
            update   = 1'b1;
            @(negedge clk_50m);
            update   = 1'b0;
            repeat (2) @(posedge pan_pwm);
        end
    endtask

    task automatic run_case(input signed [15:0] az_v, input signed [15:0] el_v,
                            input integer pan_exp, input integer tilt_exp,
                            input [255:0] name);
        begin
            apply_angles(az_v, el_v);
            measure_next(wp, wt);
            exp_pan  = pan_exp;
            exp_tilt = tilt_exp;
            check_widths(name);
        end
    endtask

    integer t0, t1;

    initial begin
        rst_n    = 1'b0;
        az_01deg = 16'sd0;
        el_01deg = 16'sd0;
        update   = 1'b0;
        n_pass   = 0;
        n_fail   = 0;

        #500;
        rst_n = 1'b1;
        repeat (5) @(posedge clk_50m);

        // ---------------- ① PWM 周期 ----------------
        @(posedge pan_pwm);
        t0 = $time;
        @(posedge pan_pwm);
        t1 = $time;
        if ((t1 - t0) == PWM_PERIOD_CYC * 20) begin
            n_pass = n_pass + 1;
            $display("PASS PWM 周期 = %0d ns（%0d 拍 @50MHz）", t1 - t0, PWM_PERIOD_CYC);
        end else begin
            n_fail = n_fail + 1;
            $display("FAIL PWM 周期 = %0d ns，期望 %0d ns", t1 - t0, PWM_PERIOD_CYC * 20);
        end

        // ---------------- ② 上电初始位：pan 90°, tilt 90° ----------------
        measure_next(wp, wt);
        exp_pan  = 900;
        exp_tilt = 900;
        check_widths("上电初始位 pan=90.0 tilt=90.0");

        // ---------------- ③ 非翻转支路 ----------------
        run_case( 16'sd900, 16'sd300,   900,  300, "az=+90.0 el=+30.0");
        run_case( 16'sd0,   16'sd50,      0,  100, "低俯仰钳位 el=5.0->10.0");

        // ---------------- ④ 翻转支路（用户示例） ----------------
        run_case(-16'sd1500, 16'sd300,  300, 1500, "翻转示例 az=-150.0 el=+30.0");
        run_case(-16'sd900,  16'sd900,  900,  900, "az=-90.0 el=+90.0");
        run_case(-16'sd1800, 16'sd900,    0,  900, "az=-180.0 边界");

        // ---------------- ⑤ 正侧边界与越界钳位 ----------------
        run_case( 16'sd1800, 16'sd900, 1800,  900, "az=+180.0 边界");
        run_case( 16'sd2000, 16'sd300, 1800,  300, "az 越界 -> pan 钳位 180.0");
        run_case(-16'sd2000, 16'sd300,    0, 1500, "az 越界 -> pan 钳位 0.0");
        run_case( 16'sd0,   16'sd2000,    0, 1800, "el 超舵机范围 -> tilt 钳位 180.0");

        // ---------------- ⑥ 无 update 时保持 ----------------
        run_case(16'sd0, 16'sd300, 0, 300, "保持测试（更新后）");
        az_01deg = 16'sd1200;          // 故意改输入但不打 update
        el_01deg = 16'sd600;
        repeat (3) @(posedge pan_pwm);
        measure_next(wp, wt);
        exp_pan  = 0;
        exp_tilt = 300;
        check_widths("保持测试（无 update，输入已变）");

        $display("============================================");
        // 注意：run_sim_all.sh 按关键字统计，通过时不要打印字面 "FAIL"，
        // 否则 "0 FAIL" 会被误计为一条失败。
        if (n_fail == 0) begin
            $display("舵机云台测试结果：%0d PASS, 全部通过", n_pass);
            $display("ALL PASS");
        end else begin
            $display("舵机云台测试结果：%0d PASS, %0d FAIL", n_pass, n_fail);
            $display("SERVO TEST FAILED");
        end
        $finish;
    end

endmodule
