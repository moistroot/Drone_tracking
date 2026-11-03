// tb_wrap_test.sv - FFT wrapper 配置生效条件最小实验
// 变体 A：配置后等 200 拍再发 256 点
// 变体 B：配置后等 5 拍再发 256 点（宏切换）
`define VARIANT_A

module tb_wrap_test;
    reg clk, rst_n;
    reg  [9:0] pt;
    reg  fwd, cfg_v;
    reg  [23:0] re, im;
    reg  v, last;
    wire [23:0] mre, mim;
    wire [9:0] midx;
    wire mv, ml;
    wire cfg_rdy;

    fft_24b_wrapper u (
        .clk(clk), .rst_n(rst_n),
        .fft_point(256), .fwd_inv(1'b1), .cfg_valid(cfg_v),
        .s_data_re(re), .s_data_im(0), .s_valid(v), .s_ready(), .s_last(last),
        .m_data_re(mre), .m_data_im(mim), .m_idx(midx), .m_valid(mv), .m_last(ml),
        .event_frame_started(), .event_tlast_missing(), .cfg_ready(cfg_rdy)
    );

    integer mv_cnt, ml_cnt, i;
    integer midx_max;
    reg ml_prev;
    integer ml_rise;
    always @(negedge clk) begin          // 负沿采样，避免与 IP 输出的 posedge 竞态
        if (mv) mv_cnt = mv_cnt + 1;
        if (ml) ml_cnt = ml_cnt + 1;
        if (mv && midx > midx_max) midx_max = midx;
        if (ml && !ml_prev) ml_rise = ml_rise + 1;
        ml_prev <= ml;
    end

    initial begin
        clk = 0; rst_n = 0; pt = 0; fwd = 0; cfg_v = 0;
        re = 0; im = 0; v = 0; last = 0;
        mv_cnt = 0; ml_cnt = 0; midx_max = -1; ml_prev = 0; ml_rise = 0;
        #200; rst_n = 1;
        repeat (2000) @(posedge clk);   // IP 复位释放后需数百拍就绪
        cfg_v = 1; @(posedge clk); #5; cfg_v = 0;
        repeat (200) @(posedge clk);    // 等 200 拍
        for (i = 0; i < 1024; i = i + 1) begin   // 4 帧 × 256 点连续
            @(posedge clk); #2;
            re = i[7:0]; v = 1; last = ((i % 256) == 255);
        end
        @(posedge clk); #2; v = 0; last = 0;
        repeat (6000) @(posedge clk);
        $display("RESULT mv_cnt=%0d ml_cnt=%0d ml_rise=%0d cfg_rdy=%0d",
                 mv_cnt, ml_cnt, ml_rise, cfg_rdy);
        $finish;
    end

    always #10 clk = ~clk;
endmodule
