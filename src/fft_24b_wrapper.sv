//  -分包2子模块2.3：Xilinx FFT IP v9.1 封装
`timescale 1ns/1ps
`include "project_global_param.vh"
module fft_24b_wrapper (
    input  wire         clk,
    input  wire         rst_n,          // 低有效复位（接 IP aresetn）
    // 运行时配置
    input  wire [FFT_IDX_WIDTH-1:0] fft_point,   // 256 / 1024（$clog2 得 NFFT）
    input  wire         fwd_inv,       // 1=FFT 0=IFFT
    input  wire         cfg_valid,     // 配置脉冲（与数据帧开始同步）
    // AXI-S 数据输入
    input  wire [DATA_WIDTH-1:0] s_data_re,
    input  wire [DATA_WIDTH-1:0] s_data_im,      // 时域输入恒接 0
    input  wire         s_valid,
    output wire         s_ready,
    input  wire         s_last,
    // AXI-S 数据输出
    output wire [DATA_WIDTH-1:0] m_data_re,
    output wire [DATA_WIDTH-1:0] m_data_im,
    output wire [FFT_IDX_WIDTH-1:0] m_idx,       // XK_INDEX（0 ~ 点数-1）
    output wire         m_valid,
    output wire         m_last,
    // 状态
    output wire         event_frame_started,
    output wire         event_tlast_missing,
    output wire         cfg_ready       // 配置通道就绪（tready=1，配置已生效）
);
    // 配置字生成
    localparam [3:0] NFFT_BITS = 4;
    function automatic [3:0] get_nfft(input [FFT_IDX_WIDTH-1:0] point);
        case (point)                          // 组合映射（$clog2 不可综合于运行时值）
            256:   get_nfft = 4'd8;
            512:   get_nfft = 4'd9;
            1024:  get_nfft = 4'd10;
            default: get_nfft = 4'd10;
        endcase
    endfunction
    function automatic [9:0] get_scale_sch(input [3:0] nfft);
        reg [3:0] n;
        integer s;
        begin
            n = (nfft > 4'd10) ? 4'd10 : nfft;          // 钳位
            get_scale_sch = 10'b0;
            get_scale_sch[1:0] = 2'b11;                 // 第一级对：>>3
            // 固定展开（循环边界必须静态），级对 s 存在条件 n >= 2s
            for (s = 2; s <= 5; s = s + 1)
                if (n >= s*2)
                    get_scale_sch[s*2-1 -: 2] = 2'b10;  // 后续级对：>>2
            // 奇数点最后一级：>>1（静态索引，避免变址 part-select 越界）
            if      (n == 4'd1) get_scale_sch[1:0] = 2'b01;
            else if (n == 4'd3) get_scale_sch[3:2] = 2'b01;
            else if (n == 4'd5) get_scale_sch[5:4] = 2'b01;
            else if (n == 4'd7) get_scale_sch[7:6] = 2'b01;
            else if (n == 4'd9) get_scale_sch[9:8] = 2'b01;
        end
    endfunction
    wire [3:0]  nfft_now  = get_nfft(fft_point);
    wire [9:0]  scale_sch = get_scale_sch(nfft_now);
    wire [23:0] cfg_word = {5'b0,
                            scale_sch,                            // [18:9]
                            fwd_inv,                              // [8]
                            4'b0,                                 // [7:4]
                            nfft_now};                            // [3:0]
    // 配置握手（脉冲 → 保持 tvalid 直到被接收）
    wire        s_axis_config_tready;
    reg cfg_pending;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            cfg_pending <= 1'b0;
        else if (cfg_valid)
            cfg_pending <= 1'b1;
        else if (cfg_pending && s_axis_config_tready)
            cfg_pending <= 1'b0;
    end
    wire        s_axis_config_tvalid = cfg_pending;
    wire [23:0] s_axis_config_tdata  = cfg_word;
    assign cfg_ready = s_axis_config_tready && !cfg_pending;
    // 数据通道
    wire [47:0] s_axis_data_tdata = {s_data_im, s_data_re};
    wire [47:0] m_axis_data_tdata;
    wire s_axis_data_tready;
    wire s_axis_data_tvalid = s_valid;
    wire s_axis_data_tlast  = s_last;
    wire m_axis_data_tvalid;
    wire m_axis_data_tlast;
    wire [15:0] m_axis_data_tuser;
    assign m_data_re = m_axis_data_tdata[23:0];
    assign m_data_im = m_axis_data_tdata[47:24];
    assign m_idx     = m_axis_data_tuser[FFT_IDX_WIDTH-1:0];
    assign m_valid   = m_axis_data_tvalid;
    assign m_last    = m_axis_data_tlast;
    assign s_ready   = s_axis_data_tready;
    // IP 例化
   xfft_0 u_xfft (
        .aclk                      (clk),
        .aresetn                   (rst_n),
        .s_axis_config_tdata       (s_axis_config_tdata),
        .s_axis_config_tvalid      (s_axis_config_tvalid),
        .s_axis_config_tready      (s_axis_config_tready),
        .s_axis_data_tdata         (s_axis_data_tdata),
        .s_axis_data_tvalid        (s_axis_data_tvalid),
        .s_axis_data_tready        (s_axis_data_tready),
        .s_axis_data_tlast         (s_axis_data_tlast),
        .m_axis_data_tdata         (m_axis_data_tdata),
        .m_axis_data_tvalid        (m_axis_data_tvalid),
        .m_axis_data_tlast         (m_axis_data_tlast),
        .m_axis_data_tuser         (m_axis_data_tuser),
        .event_frame_started        (event_frame_started),
        .event_tlast_missing        (event_tlast_missing),
        .event_tlast_unexpected     (),
        .event_data_in_channel_halt ()
    );
endmodule
