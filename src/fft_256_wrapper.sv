// fft_256_wrapper.sv - 4 路并行前向 FFT 的单通道封装
`timescale 1ns/1ps
`include "project_global_param.vh"
module fft_256_wrapper (
    input  wire         clk,
    input  wire         rst_n,          // 低有效复位（接 IP aresetn）
    // 运行时配置（每帧一次，点数固定 256）
    input  wire         cfg_valid,      // 单拍配置脉冲
    // AXI-S 数据输入
    input  wire [DATA_WIDTH-1:0] s_data_re,
    input  wire         s_valid,
    output wire         s_ready,
    input  wire         s_last,
    // AXI-S 数据输出
    output wire [DATA_WIDTH-1:0] m_data_re,
    output wire [DATA_WIDTH-1:0] m_data_im,
    output wire [7:0]   m_idx,          // XK_INDEX 0..255
    output wire         m_valid,
    output wire         m_last,
    // 状态
    output wire         event_frame_started,
    output wire         event_tlast_missing,
    output wire         cfg_ready
);
    // 固定配置字
    localparam [7:0]  SCALE_SCH_256 = 8'b1010_1011;  // [1:0]=>>3，其余 >>2
    localparam [23:0] CFG_WORD      = {7'b0, SCALE_SCH_256, 1'b1, 4'b0, 4'd8};
    // 配置握手（脉冲 → 保持 tvalid 直到被接收）
    wire        s_axis_config_tready;
    reg         cfg_pending;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            cfg_pending <= 1'b0;
        else if (cfg_valid)
            cfg_pending <= 1'b1;
        else if (cfg_pending && s_axis_config_tready)
            cfg_pending <= 1'b0;
    end
    wire [23:0] s_axis_config_tdata  = CFG_WORD;
    wire        s_axis_config_tvalid = cfg_pending;
    assign cfg_ready = s_axis_config_tready && !cfg_pending;
    // 数据通道
    wire [47:0] s_axis_data_tdata = {{DATA_WIDTH{1'b0}}, s_data_re};
    wire [47:0] m_axis_data_tdata;
    wire        s_axis_data_tready;
    wire        m_axis_data_tvalid;
    wire        m_axis_data_tlast;
    wire [7:0]  m_axis_data_tuser;
    assign m_data_re = m_axis_data_tdata[23:0];
    assign m_data_im = m_axis_data_tdata[47:24];
    assign m_idx     = m_axis_data_tuser[7:0];
    assign m_valid   = m_axis_data_tvalid;
    assign m_last    = m_axis_data_tlast;
    assign s_ready   = s_axis_data_tready;
    // IP 例化
    xfft_256 u_xfft (
        .aclk                      (clk),
        .aresetn                   (rst_n),
        .s_axis_config_tdata       (s_axis_config_tdata),
        .s_axis_config_tvalid      (s_axis_config_tvalid),
        .s_axis_config_tready      (s_axis_config_tready),
        .s_axis_data_tdata         (s_axis_data_tdata),
        .s_axis_data_tvalid        (s_valid),
        .s_axis_data_tready        (s_axis_data_tready),
        .s_axis_data_tlast         (s_last),
        .m_axis_data_tdata         (m_axis_data_tdata),
        .m_axis_data_tvalid        (m_axis_data_tvalid),
        .m_axis_data_tlast         (m_axis_data_tlast),
        .m_axis_data_tuser         (m_axis_data_tuser),
        .event_frame_started       (event_frame_started),
        .event_tlast_missing       (event_tlast_missing),
        .event_tlast_unexpected    (),
        .event_data_in_channel_halt()
    );
endmodule
