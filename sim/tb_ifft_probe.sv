// ============================================================
// tb_ifft_probe.sv - IFFT IP 行为标定（解析可判定的输入帧）
//
// 帧 A：纯 DC（X[0]=2^22）→ 输出应恒为 2^22·scale（实数常数，判缩放）
// 帧 B：单复点对 X[1]=+j·2^22, X[1023]=−j·2^22（共轭对称）→
//        IFFT 约定：h[n] = −(2·2^22·scale)·sin(2πn/1024)
//        FFT  约定：h[n] = +(2·2^22·scale)·sin(2πn/1024)   ← 由符号判定方向
// 帧 C：PHAT 时延帧（低带 0..128 相位 2πk·10/256 + 共轭镜像）
//        正确 IFFT → 峰在 984；若为 FFT → 峰在 40
// ============================================================
`timescale 1ns/1ps
`include "project_global_param.vh"

module tb_ifft_probe;

    reg         clk_50m;
    reg         rst_n;
    reg  [23:0] din_re, din_im;
    reg         din_valid, din_last, din_cfg_valid;
    wire        din_ready;
    wire [TDOA_WIDTH-1:0] tdoa_out;
    wire        tdoa_valid;

    ifft_peak_detect uut (
        .clk_50m (clk_50m), .rst_n (rst_n),
        .din_re (din_re), .din_im (din_im),
        .din_valid (din_valid), .din_last (din_last),
        .din_cfg_valid (din_cfg_valid), .din_ready (din_ready),
        .tdoa_out (tdoa_out), .tdoa_valid (tdoa_valid)
    );

    initial begin clk_50m = 0; rst_n = 0; end
    always #10 clk_50m = ~clk_50m;

    real    pi2;
    integer n, k, fno;
    integer ire, iim, TAUI;
    real    ph;

    // 发送一帧（1024 点）
    task send_frame(input integer mode);
        integer nn, jj;
        real    p;
        begin
            for (nn = 0; nn < 1024; nn = nn + 1) begin
                case (mode)
                    0: begin                       // 帧 A：纯 DC
                        ire = (nn == 0) ? 4194304 : 0;
                        iim = 0;
                    end
                    1: begin                       // 帧 B：单复点对 +j / −j
                        if      (nn == 1)    begin ire = 0;        iim =  4194304; end
                        else if (nn == 1023) begin ire = 0;        iim = -4194304; end
                        else                 begin ire = 0;        iim = 0;        end
                    end
                    default: begin                 // 帧 C：PHAT 时延帧
                        if (nn <= 128)       jj = nn;
                        else if (nn >= 896)  jj = 1024 - nn;
                        else                 jj = -1;
                        if (jj >= 0) begin
                            p = pi2 * jj * 10.0 / 256.0;
                            if (nn > 896)  p = -p;
                            if (nn == 896) p = 0.0;
                            ire = $rtoi(4194304.0 * $cos(p));
                            iim = $rtoi(4194304.0 * $sin(p));
                        end else begin
                            ire = 0; iim = 0;
                        end
                    end
                endcase
                @(negedge clk_50m);
                din_re = ire; din_im = iim;
                din_valid = 1; din_last = (nn == 1023);
                @(posedge clk_50m);
                while (!din_ready) @(posedge clk_50m);
            end
            @(negedge clk_50m); din_valid = 0; din_last = 0;
        end
    endtask

    initial begin
        pi2 = 6.283185307179586;
        din_re = 0; din_im = 0; din_valid = 0; din_last = 0; din_cfg_valid = 0;
        repeat (10) @(posedge clk_50m);
        rst_n = 1;
        repeat (20) @(posedge clk_50m);

        for (fno = 0; fno < 3; fno = fno + 1) begin
            @(negedge clk_50m); din_cfg_valid = 1;
            @(posedge clk_50m);
            @(negedge clk_50m); din_cfg_valid = 0;
            repeat (20) @(posedge clk_50m);
            $display("---- 帧 %0d 开始 ----", fno);
            send_frame(fno);
            repeat (2500) @(posedge clk_50m);
        end
        $display("=== IFFT TB 完成 ===");
        $finish;
    end

    // 输出采样观察
    always @(posedge clk_50m) begin
        if (uut.m_valid) begin
            if (fno == 0) begin
                if (uut.m_idx == 0 || uut.m_idx == 1 || uut.m_idx == 512)
                    $display("A: idx=%0d re=%0d im=%0d", uut.m_idx, $signed(uut.m_re), $signed(uut.m_im));
            end else if (fno == 1) begin
                if (uut.m_idx == 1 || uut.m_idx == 2 || uut.m_idx == 256 ||
                    uut.m_idx == 512 || uut.m_idx == 768 || uut.m_idx == 1023)
                    $display("B: idx=%0d re=%0d im=%0d", uut.m_idx, $signed(uut.m_re), $signed(uut.m_im));
            end else begin
                if (uut.m_idx <= 4 || (uut.m_idx >= 36 && uut.m_idx <= 44) ||
                    (uut.m_idx >= 980 && uut.m_idx <= 988) || uut.m_idx == 1023)
                    $display("C: idx=%0d re=%0d im=%0d", uut.m_idx, $signed(uut.m_re), $signed(uut.m_im));
            end
        end
    end

    always @(posedge clk_50m) begin
        if (fno == 2 && uut.m_valid && uut.m_last)
            $display("PEAK: idx=%0d val=%0d L=%0d R=%0d",
                     uut.peak_idx, uut.peak_val, uut.peak_L, uut.peak_R);
        if (tdoa_valid)
            $display("TDOA: %0d Q10", $signed(tdoa_out));
    end

endmodule
