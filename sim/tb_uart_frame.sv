// ============================================================
// tb_uart_frame.sv - 分包3子模块3.6 UART 帧格式验证
//
// 验证对象：uart_tx_module（115200 8N1，91 字节 ASCII 帧）
// 方法：TB 内实现一个"上位机"接收器（起始位下降沿同步、每位中点采样、
//       波特周期 434 拍），把 uart_tx 上的位流还原成 91 字节数组，
//       与按《02_接口时序说明.md》规格逐字节现算的期望数组比对，
//       并独立复算校验和（$ 之后到 * 之前的第 1..85 字节 XOR）。
//
// 用例（3 帧，覆盖符号/零值/边界角/大值）：
//   ① 常规帧：τ=(+5120,0,-2048,+1024,-1024,+3072)，A=+45.0°、E=+51.8°
//   ② 负极值：τ 全 -1023，A=-180.0°、E=+0.0°
//   ③ 满量程：τ 全 +1023，A=+180.0°、E=+90.0°
//
// 断言：① 字节数 = 91；② 91 字节与规格现算值全部相同；
//       ③ 校验和字节 = 规格 XOR 结果。
// ============================================================
`timescale 1ns/1ps
`include "project_global_param.vh"

module tb_uart_frame;

    localparam integer BAUD_CYC  = 434;     // 50 MHz / 115200（与 DUT 一致）
    localparam integer UART_PAIR = 6;       // 帧内打印的配对数（前 4 路麦克风的 6 组）
    localparam integer FRM_LEN   = 91;      // 帧长（字节）

    reg  clk_50m, rst_n;
    reg  frame_valid;
    reg  [FRAME_CNT_WIDTH-1:0] frame_cnt;
    reg  [TDOA_WIDTH-1:0] tdoa_vals [0:MAX_PAIR_NUM-1];
    reg  signed [15:0] az01, el01;
    reg  pos_valid;
    wire uart_tx;

    uart_tx_module uut (
        .clk_50m        (clk_50m),
        .rst_n          (rst_n),
        .frame_valid    (frame_valid),
        .frame_cnt      (frame_cnt),
        .tdoa_vals      (tdoa_vals),
        .pos_azimuth_01deg   (az01),
        .pos_elevation_01deg (el01),
        .pos_valid      (pos_valid),
        .uart_tx        (uart_tx)
    );

    always #10 clk_50m = ~clk_50m;

    // ================= 工具函数（先声明后使用） =================
    function automatic integer pow10i(input integer n);
        integer r, i;
        r = 1;
        for (i = 0; i < n; i = i + 1) r = r * 10;
        pow10i = r;
    endfunction

    // 取 v 的十进制第 pos 位（pos=0 为个位）
    function automatic [7:0] dec_digit(input integer v, input integer pos);
        integer d;
        d = (v / pow10i(pos)) % 10;
        dec_digit = 8'h30 + d[7:0];
    endfunction

    function automatic integer hexval(input [7:0] ch);
        if (ch >= "0" && ch <= "9")      hexval = ch - "0";
        else if (ch >= "A" && ch <= "F") hexval = ch - "A" + 10;
        else                             hexval = -1;
    endfunction

    // ================= 上位机接收器 =================
    reg  [7:0]  rx_buf [0:255];
    integer     rx_n;
    reg  [15:0] rx_tmr;
    reg  [3:0]  rx_bit;
    reg         rx_act;
    reg         tx_d;

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            rx_n <= 0; rx_act <= 1'b0; rx_tmr <= 0; rx_bit <= 0; tx_d <= 1'b1;
        end else begin
            tx_d <= uart_tx;
            if (!rx_act) begin
                // 起始位下降沿 → 启动接收：
                // 先跨过起始位（434 拍）再多半个位周期，落在 bit0 中点
                if (tx_d && !uart_tx) begin
                    rx_act <= 1'b1;
                    rx_bit <= 4'd0;
                    rx_tmr <= BAUD_CYC + BAUD_CYC/2 + 1;
                    rx_buf[rx_n] <= 8'h00;
                end
            end else if (rx_tmr == 0) begin
                if (rx_bit < 4'd8) begin
                    if (uart_tx) rx_buf[rx_n] <= rx_buf[rx_n] | (8'd1 << rx_bit);
                    rx_bit <= rx_bit + 1'b1;
                    rx_tmr <= BAUD_CYC;
                end else begin      // 8 位数据收完
                    rx_n   <= rx_n + 1;
                    rx_act <= 1'b0;
                end
            end else
                rx_tmr <= rx_tmr - 1'b1;
        end
    end

    // ================= 期望帧（按规格逐字节现算） =================
    reg [7:0] exp_buf [0:FRM_LEN-1];

    task automatic build_exp(input integer fn, input integer a01, input integer e01);
        integer p, b, v, a;
        begin
            exp_buf[0] = "$"; exp_buf[1] = "G"; exp_buf[2] = "C";
            exp_buf[3] = "C"; exp_buf[4] = "P"; exp_buf[5] = ",";
            // 帧号 4 位十进制（取低 4 位）
            a = fn % 10000;
            exp_buf[6] = dec_digit(a, 3);
            exp_buf[7] = dec_digit(a, 2);
            exp_buf[8] = dec_digit(a, 1);
            exp_buf[9] = dec_digit(a, 0);
            exp_buf[10] = ",";
            // 6 组 TDOA：'T' 号号 ':' 符 ddd ','
            for (p = 0; p < UART_PAIR; p = p + 1) begin
                b = 11 + 9*p;
                exp_buf[b+0] = "T";
                exp_buf[b+1] = (p < 3) ? "0" : ((p < 5) ? "1" : "2");
                exp_buf[b+2] = (p == 0) ? "1" : (p == 1) ? "2" : (p == 2) ? "3" :
                               (p == 3) ? "2" : "3";
                exp_buf[b+3] = ":";
                v = $signed(tdoa_vals[p]);
                exp_buf[b+4] = (v < 0) ? "-" : "+";
                a = (v < 0) ? -v : v;
                a = a >>> 10;                       // Q10 → 整数采样点
                exp_buf[b+5] = dec_digit(a, 2);
                exp_buf[b+6] = dec_digit(a, 1);
                exp_buf[b+7] = dec_digit(a, 0);
                exp_buf[b+8] = ",";
            end
            // A 字段（65..73）：'A' ':' 符 ddd '.' d ','
            exp_buf[65] = "A"; exp_buf[66] = ":";
            exp_buf[67] = (a01 < 0) ? "-" : "+";
            a = (a01 < 0) ? -a01 : a01;             // 0.1° 绝对值
            exp_buf[68] = dec_digit(a/10, 2);
            exp_buf[69] = dec_digit(a/10, 1);
            exp_buf[70] = dec_digit(a/10, 0);
            exp_buf[71] = ".";
            exp_buf[72] = dec_digit(a, 0);
            exp_buf[73] = ",";
            // E 字段（74..82）
            exp_buf[74] = "E"; exp_buf[75] = ":";
            exp_buf[76] = (e01 < 0) ? "-" : "+";
            a = (e01 < 0) ? -e01 : e01;
            exp_buf[77] = dec_digit(a/10, 2);
            exp_buf[78] = dec_digit(a/10, 1);
            exp_buf[79] = dec_digit(a/10, 0);
            exp_buf[80] = ".";
            exp_buf[81] = dec_digit(a, 0);
            exp_buf[82] = ",";
            // V 字段（83..86）
            exp_buf[83] = "V"; exp_buf[84] = ":";
            exp_buf[85] = pos_valid ? "1" : "0";
            exp_buf[86] = "*";
            // 校验和（87,88）：第 1..85 字节 XOR，大写十六进制
            a = 0;
            for (p = 1; p <= 85; p = p + 1) a = a ^ exp_buf[p];
            exp_buf[87] = (a[7:4] < 10) ? (8'h30 + a[7:4]) : (8'h41 + a[7:4] - 10);
            exp_buf[88] = (a[3:0] < 10) ? (8'h30 + a[3:0]) : (8'h41 + a[3:0] - 10);
            exp_buf[89] = 8'h0D;
            exp_buf[90] = 8'h0A;
        end
    endtask

    // ================= 驱动一帧 =================
    integer n_err, k, c, rx_to, len;
    integer a_exp, e_exp;                  // 钳位后的期望角度（0.1°）
    time    t0, t1;

    task automatic run_frame(input integer fn, input integer a01, input integer e01,
                             input integer t01, input integer t02, input integer t03,
                             input integer t12, input integer t13, input integer t23,
                             input integer pv, input integer inject);
        begin
            frame_cnt    = fn[FRAME_CNT_WIDTH-1:0];
            tdoa_vals[0] = t01[TDOA_WIDTH-1:0];
            tdoa_vals[1] = t02[TDOA_WIDTH-1:0];
            tdoa_vals[2] = t03[TDOA_WIDTH-1:0];
            tdoa_vals[3] = t12[TDOA_WIDTH-1:0];
            tdoa_vals[4] = t13[TDOA_WIDTH-1:0];
            tdoa_vals[5] = t23[TDOA_WIDTH-1:0];
            az01 = a01[15:0]; el01 = e01[15:0]; pos_valid = pv[0];

            // 等上一帧完全发完（含停止位）再发下一帧
            while (rx_act) @(posedge clk_50m);
            repeat (BAUD_CYC + 20) @(posedge clk_50m);

            rx_n = 0;
            t0 = $time;
            frame_valid = 1'b1;
            @(negedge clk_50m);
            frame_valid = 1'b0;

            // ---- 忙时重触发注入：帧发到约第 10 字节时，用**不同**数据再发一次 frame_valid ----
            // 期望：该请求被整帧丢弃，正在发送的帧不得出现"前旧后新"的拼接（校验和自洽的坏帧）
            if (inject) begin
                repeat (46200) @(posedge clk_50m);
                frame_cnt    = (fn + 7);
                tdoa_vals[0] = 16'sd1234;
                tdoa_vals[1] = -16'sd999;
                az01 = 16'sd1799; el01 = 16'sd899; pos_valid = 1'b0;
                frame_valid = 1'b1;
                @(negedge clk_50m);
                frame_valid = 1'b0;
                $display("  [注入] 发送中重触发 frame_valid（fn=%0d A=1799 E=899 V=0），应被丢弃",
                         fn + 7);
                // 恢复发送侧数据（DUT 已锁存本帧数据，恢复仅为后续用例/期望值计算一致）
                frame_cnt    = fn[FRAME_CNT_WIDTH-1:0];
                tdoa_vals[0] = t01[TDOA_WIDTH-1:0];
                tdoa_vals[1] = t02[TDOA_WIDTH-1:0];
                az01 = a01[15:0]; el01 = e01[15:0]; pos_valid = pv[0];
            end

            // 收满 91 字节（超时保护：正常帧时长 91×10×434 ≈ 395k 拍）
            rx_to = 0;
            while (rx_n < FRM_LEN && rx_to < 900000) begin
                @(posedge clk_50m);
                rx_to = rx_to + 1;
            end
            t1 = $time;
            @(posedge clk_50m);

            // 角度按契约钳位后计算期望值（与 DUT 的 az_cl/el_cl 一致）
            a_exp = (a01 > 1800) ? 1800 : ((a01 < -1800) ? -1800 : a01);
            e_exp = (e01 > 900)  ? 900  : ((e01 < 0)     ? 0     : e01);

            $display("---- 帧号 %0d：A=%0d.%0d E=%0d.%0d ----",
                     fn, a_exp/10, a_exp%10, e_exp/10, e_exp%10);
            $display("  实收 %0d 字节（耗时 %0d 拍）", rx_n, (t1-t0)/20);
            $write("  实收内容: ");
            for (k = 0; k < rx_n && k < 255; k = k + 1) $write("%c", rx_buf[k]);
            $write("\n");

            // ---- 逐字节比对 ----
            build_exp(fn, a_exp, e_exp);
            $write("  期望内容: ");
            for (k = 0; k < FRM_LEN; k = k + 1) $write("%c", exp_buf[k]);
            $write("\n");

            if (rx_n != FRM_LEN) begin
                $error("帧字节数 %0d != %0d", rx_n, FRM_LEN);
                n_err = n_err + 1;
            end
            len = (rx_n < FRM_LEN) ? rx_n : FRM_LEN;
            for (k = 0; k < len; k = k + 1) begin
                if (rx_buf[k] !== exp_buf[k]) begin
                    if (n_err < 6)
                        $error("字节 %0d 不符：实收 0x%02X 期望 0x%02X",
                               k, rx_buf[k], exp_buf[k]);
                    n_err = n_err + 1;
                end
            end

            // ---- 独立复算校验和（规格：第 1..85 字节 XOR） ----
            if (rx_n == FRM_LEN) begin
                c = 0;
                for (k = 1; k <= 85; k = k + 1) c = c ^ rx_buf[k];
                if ((hexval(rx_buf[87])*16 + hexval(rx_buf[88])) !== c) begin
                    $error("校验和 %02X%02X 与复算值 %02X 不符",
                           rx_buf[87], rx_buf[88], c);
                    n_err = n_err + 1;
                end else
                    $display("  校验和 %02X%02X 复算一致", rx_buf[87], rx_buf[88]);
            end
        end
    endtask

    // ================= 主流程 =================
    initial begin
        clk_50m = 0; rst_n = 0; frame_valid = 0; pos_valid = 0;
        frame_cnt = 0; az01 = 0; el01 = 0;
        for (k = 0; k < MAX_PAIR_NUM; k = k + 1) tdoa_vals[k] = 0;
        n_err = 0;
        #500; rst_n = 1;
        repeat (20) @(posedge clk_50m);

        run_frame(42,   450, 518,  5120,     0, -2048,  1024, -1024,  3072, 1, 0);
        run_frame(7,  -1800,   0, -1023, -1023, -1023, -1023, -1023, -1023, 1, 0);
        run_frame(9999, 1800, 900,  1023,  1023,  1023,  1023,  1023,  1023, 1, 0);
        // ④ 文档《04_板级调试指南》第 3 节的示例帧（应输出 *41 之外的 0x46 校验和）
        run_frame(42,   450, 518,  5120,     0,  5120, -5120,  5120,  5120, 1, 0);
        // ⑤ 上电学习期/无效解：V:0、角度与 TDOA 全 0（与 phat_position_top 的门控输出一致）
        run_frame(1,      0,   0,     0,     0,     0,     0,     0,     0, 0, 0);
        // ⑥ 忙时重触发：发送途中再发 frame_valid（不同数据）→ 必须整帧丢弃，不得拼接
        run_frame(12,   321,  87,   123,  -456,   789,  -321,   654,  -987, 1, 1);
        // ⑦ 越界钳位：A=+2000 → 1800、E=−50 → 0（契约保护，帧格式仍合法）
        run_frame(5,   2000, -50,     7,     7,     7,     7,     7,     7, 1, 0);

        $display("========================================");
        $display("UART 帧格式错误数 : %0d", n_err);
        $display("========================================");
        if (n_err == 0) $display("PASS");
        else            $display("FAIL");
        $finish;
    end

endmodule
