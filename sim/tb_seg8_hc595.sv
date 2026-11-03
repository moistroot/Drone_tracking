// ============================================================
// tb_seg8_hc595.sv - 8 位数码管驱动（74HC595）专项验证
//
// 显示格式（整度，省略小数）：
//    d7   d6 d5 d4   d3    d2 d1 d0
//   符号   方位角    空白    俯仰角
//   例：方位 -45°、俯仰 52° -> "-045 052"
//
// 验证点：
//   ① 串行协议：每个 STCP 周期内恰好 16 个 SHCP 上升沿，帧最高位 DP=0
//   ② 位选独热：sel 每帧只有一个 1，8 个位选都能出现
//   ③ 段码正确：按 16bit 帧 {DP,seg,sel} 还原每一位的段码，与期望值比对
//   ④ 用例：上电无数据全 "-"；正/负方位角；无新数据维持上一刻；
//            -0.x° 负号抑制；越界钳位；边界 ±180°/90°
//
// 仿真用 SCAN_DIV=200（每位 4us）加速；真实参数 50000（每位 1ms）。
// 注意 SCAN_DIV 必须 > 66（一帧 16bit 移位需要 33×2 拍），否则会跳位。
// ============================================================
`timescale 1ns/1ps

module tb_seg8_hc595;

    localparam integer SCAN_DIV = 200;

    reg               clk_50m;
    reg               rst_n;
    reg  signed [15:0] az_01deg;
    reg        [15:0] el_01deg;
    reg               sample_valid;
    wire              sh_cp;
    wire              st_cp;
    wire              ds;

    seg8_hc595 #(
        .SCAN_DIV (SCAN_DIV),
        .DP_BIT   (1'b0)
    ) dut (
        .clk_50m      (clk_50m),
        .rst_n        (rst_n),
        .az_01deg     (az_01deg),
        .el_01deg     (el_01deg),
        .sample_valid (sample_valid),
        .sh_cp        (sh_cp),
        .st_cp        (st_cp),
        .ds           (ds)
    );

    initial begin clk_50m = 0; end
    always #10 clk_50m = ~clk_50m;

    // ---------------------------------------------------------
    // TB 侧独立模型：只在 sample_valid 时锁存（与 DUT 一致）
    // ---------------------------------------------------------
    reg signed [15:0] az_lat;
    reg        [15:0] el_lat;
    reg               have_data;

    always @(posedge clk_50m) begin
        if (sample_valid) begin
            az_lat    <= az_01deg;
            el_lat    <= el_01deg;
            have_data <= 1'b1;
        end
    end

    // 期望角度（整度，四舍五入，与 DUT 同算法）
    function automatic integer az_deg_of(input integer v_raw);
        integer v;
        begin
            v = v_raw;
            if (v >  1800) v =  1800;
            if (v < -1800) v = -1800;
            if (v < 0)     v = -v;
            az_deg_of = (v + 5) / 10;
        end
    endfunction

    function automatic integer el_deg_of(input integer v_raw);
        integer v;
        begin
            v = v_raw;
            if (v < 0)   v = 0;
            if (v > 900) v = 900;
            el_deg_of = (v + 5) / 10;
        end
    endfunction

    // 第 d 位（0 = 最右位）的期望数字，4'hE = "-"，4'hF = 熄灭
    function automatic [3:0] nib_of(input integer d);
        integer a_d, e_d;
        begin
            if (!have_data) begin
                nib_of = 4'hE;
            end else begin
                a_d = az_deg_of(az_lat);
                e_d = el_deg_of(el_lat);
                case (d)
                    0: nib_of = e_d % 10;
                    1: nib_of = (e_d / 10) % 10;
                    2: nib_of = 4'h0;
                    3: nib_of = 4'hF;
                    4: nib_of = a_d % 10;
                    5: nib_of = (a_d / 10) % 10;
                    6: nib_of = (a_d / 100) % 10;
                    7: nib_of = (az_lat < 0 && a_d != 0) ? 4'hE : 4'hF;
                    default: nib_of = 4'hF;
                endcase
            end
        end
    endfunction

    // 期望的 8 字符显示（仅用于日志，从左到右）
    function automatic [63:0] exp_str();
        integer   k, v;
        reg [7:0] ch;
        begin
            exp_str = 64'd0;
            for (k = 0; k < 8; k = k + 1) begin
                v = nib_of(7 - k);
                case (v)
                    4'hF:    ch = " ";
                    4'hE:    ch = "-";
                    default: ch = 8'h30 + v[7:0];
                endcase
                exp_str[(7-k)*8 +: 8] = ch;
            end
        end
    endfunction

    // ---------------------------------------------------------
    // 监视器：SHCP 上升沿移入 1 bit；STCP 上升沿锁存并校验
    // ---------------------------------------------------------
    reg [15:0] shreg;
    reg        sh_d, st_d;
    integer    n_err, n_frame, n_shcp_in_frame;
    integer    shcp_total;
    integer    seen_sel;                 // 已出现的位选集合
    reg        check_en;                 // 0 = 输入刚变化，等待一帧稳定后再校验

    always @(posedge clk_50m) begin
        sh_d <= sh_cp;
        st_d <= st_cp;
    end

    // 期望段码（低有效，{g,f,e,d,c,b,a}）
    function automatic [6:0] seg_of(input [3:0] nib);
        begin
            case (nib)
                4'h0: seg_of = 7'b1000000;
                4'h1: seg_of = 7'b1111001;
                4'h2: seg_of = 7'b0100100;
                4'h3: seg_of = 7'b0110000;
                4'h4: seg_of = 7'b0011001;
                4'h5: seg_of = 7'b0010010;
                4'h6: seg_of = 7'b0000010;
                4'h7: seg_of = 7'b1111000;
                4'h8: seg_of = 7'b0000000;
                4'h9: seg_of = 7'b0010000;
                4'hE: seg_of = 7'b0111111;   // "-"
                default: seg_of = 7'b1111111; // 熄灭
            endcase
        end
    endfunction

    task automatic check_frame(input [15:0] f);
        integer   d, k, hit;
        reg [7:0] s;
        reg [6:0] g;
        reg [6:0] exp;
        begin
            // 输入变化后，移位链里还可能残留上一轮的一帧，等稳定后再校验
            if (!check_en) disable check_frame;

            s = f[7:0];
            g = f[14:8];
            n_frame = n_frame + 1;

            if (n_shcp_in_frame != 16) begin
                n_err = n_err + 1;
                $display("  [ERR] 本帧 SHCP 数 = %0d（应为 16）", n_shcp_in_frame);
            end
            if (f[15] !== 1'b0) begin
                n_err = n_err + 1;
                $display("  [ERR] DP 位 = %b（应为 0）", f[15]);
            end

            // 独热码解码
            k = 0; hit = -1;
            for (d = 0; d < 8; d = d + 1) begin
                if (s[d] === 1'b1) begin
                    k = k + 1;
                    hit = d;
                end
            end
            if (k != 1) begin
                n_err = n_err + 1;
                $display("  [ERR] sel=%b 不是独热码", s);
                disable check_frame;
            end

            seen_sel = seen_sel | (1 << hit);
            exp      = seg_of(nib_of(hit));
            if (g !== exp) begin
                n_err = n_err + 1;
                $display("  [ERR] 位%d (sel=%b) 段码=%b 期望=%b（az=%0d el=%0d 期望屏=\"%s\"）",
                         hit, s, g, exp, az_lat, el_lat, exp_str());
            end
        end
    endtask

    always @(posedge clk_50m) begin
        if (sh_cp && !sh_d) begin
            shreg         <= {shreg[14:0], ds};
            shcp_total     = shcp_total + 1;
            n_shcp_in_frame = n_shcp_in_frame + 1;
        end
        if (st_cp && !st_d) begin
            check_frame(shreg);
            n_shcp_in_frame = 0;
        end
    end

    // ---------------------------------------------------------
    // 主流程
    // ---------------------------------------------------------
    integer base_err;

    // 输入变化后等一整轮扫描（8 位），让移位链里残留的旧帧走完再开始校验
    task automatic settle;
        begin
            check_en = 1'b0;
            repeat (SCAN_DIV * 8 + 200) @(posedge clk_50m);
            check_en = 1'b1;
        end
    endtask

    // 推一帧新角度：先摆好输入，再打一个单周期 sample_valid 脉冲
    task automatic push(input integer a, input integer e);
        begin
            @(negedge clk_50m);
            az_01deg = a[15:0];
            el_01deg = e[15:0];
            sample_valid = 1'b1;
            @(negedge clk_50m);
            sample_valid = 1'b0;
            settle;
        end
    endtask

    initial begin
        rst_n = 0; az_01deg = 16'sd0; el_01deg = 16'd0; sample_valid = 1'b0;
        az_lat = 16'sd0; el_lat = 16'd0; have_data = 1'b0;
        shreg = 16'd0; sh_d = 1'b0; st_d = 1'b0;
        n_err = 0; n_frame = 0; n_shcp_in_frame = 0; shcp_total = 0;
        seen_sel = 0; check_en = 0;
        #200;
        @(negedge clk_50m);
        rst_n = 1;

        // ---- 用例①：上电后还没有有效数据 -> 整屏 "-" ----
        $display("---- 用例1: 上电无数据 -> 整屏 '-'（期望屏 \"%s\"）----", exp_str());
        settle;
        seen_sel = 0;
        repeat (SCAN_DIV * 8 * 2) @(posedge clk_50m);
        if (seen_sel != 8'hFF) begin
            $display("  [ERR] 并非 8 个位选都出现过: %b", seen_sel);
            n_err = n_err + 1;
        end
        $display("  已检查帧数 = %0d，累计错误 = %0d", n_frame, n_err);

        // ---- 用例②：方位 +45.0°、俯仰 51.8° -> " 045 052" ----
        $display("---- 用例2: az=+45.0, el=+51.8 -> \" 045 052\" ----");
        base_err = n_err;
        push(450, 518);
        repeat (SCAN_DIV * 8 * 2) @(posedge clk_50m);
        $display("  期望屏 = \"%s\"，本用例错误 = %0d", exp_str(), n_err - base_err);

        // ---- 用例③：方位 -120.4°、俯仰 54.9° -> "-120 055" ----
        $display("---- 用例3: az=-120.4, el=+54.9 -> \"-120 055\" ----");
        base_err = n_err;
        push(-1204, 549);
        repeat (SCAN_DIV * 8 * 2) @(posedge clk_50m);
        $display("  期望屏 = \"%s\"，本用例错误 = %0d", exp_str(), n_err - base_err);

        // ---- 用例④：只改输入、不发 sample_valid -> 维持上一刻 ----
        $display("---- 用例4: 没有新数据 -> 维持 \"-120 055\"（若误锁存会变成 \" 070 010\"）----");
        base_err = n_err;
        az_01deg = 16'sd700; el_01deg = 16'd100;
        settle;
        repeat (SCAN_DIV * 8 * 2) @(posedge clk_50m);
        $display("  期望屏 = \"%s\"，本用例错误 = %0d", exp_str(), n_err - base_err);

        // ---- 用例⑤：方位 -0.4°、俯仰 90.0° -> " 000 090"（负号抑制）----
        $display("---- 用例5: az=-0.4, el=+90.0 -> \" 000 090\" ----");
        base_err = n_err;
        push(-4, 900);
        repeat (SCAN_DIV * 8 * 2) @(posedge clk_50m);
        $display("  期望屏 = \"%s\"，本用例错误 = %0d", exp_str(), n_err - base_err);

        // ---- 用例⑥：越界钳位 az=+250.0°、el=+150.0° -> " 180 090" ----
        $display("---- 用例6: az=+250.0, el=+150.0 -> \" 180 090\"（钳位）----");
        base_err = n_err;
        push(2500, 1500);
        repeat (SCAN_DIV * 8 * 2) @(posedge clk_50m);
        $display("  期望屏 = \"%s\"，本用例错误 = %0d", exp_str(), n_err - base_err);

        // ---- 用例⑦：边界 az=-180.0°、el=0.0° -> "-180 000" ----
        $display("---- 用例7: az=-180.0, el=0.0 -> \"-180 000\" ----");
        base_err = n_err;
        push(-1800, 0);
        repeat (SCAN_DIV * 8 * 2) @(posedge clk_50m);
        $display("  期望屏 = \"%s\"，本用例错误 = %0d", exp_str(), n_err - base_err);

        $display("========================================");
        $display("总帧数 = %0d，SHCP 总数 = %0d，错误数 = %0d",
                 n_frame, shcp_total, n_err);
        $display("========================================");
        if (n_err == 0) $display("PASS");
        else            $display("FAIL");
        $finish;
    end

endmodule
