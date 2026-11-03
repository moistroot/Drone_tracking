// project_global_pkg.sv - 全局统一参数包（唯一参数源）
package project_global_pkg;
    localparam SYS_CLK_FREQ      = 50_000_000;   // 系统主频50MHz（硬件固定）
    localparam DATA_WIDTH        = 24;           // 全局采样/FFT位宽24位
    localparam COORD_WIDTH       = 16;           // 空间坐标位宽（单位：cm）
    localparam FRAME_CNT_WIDTH   = 16;           // 帧号计数器位宽
    localparam FFT_IDX_WIDTH     = 10;           // 频点/发送计数位宽（IFFT 1024点）
    localparam CHAN_IDX_WIDTH    = 2;            // 通道索引位宽（4路）
    localparam TDOA_WIDTH        = 16;           // TDOA时延值位宽
    // 当前落地版本参数（4路 10 kHz）
    localparam CUR_SAMPLE_FREQ   = 10_000;       // 当前采样率10kHz
    localparam CUR_FFT_POINT     = 256;          // 前向 FFT 点数（每通道并行）
    localparam IFFT_POINT        = 1024;         // 逆变换点数（4 倍时域内插）
    localparam FFT_BINS          = CUR_FFT_POINT/2 + 1;  // 单边频点 0..128（129 个）
    localparam UART_BAUD_DIV     = 434;
    // 通道与配对（固定 4 路）
    localparam MAX_CHAN_NUM      = 4;   // 4 路麦克风
    localparam MAX_PAIR_NUM      = 6;   // 6 组配对
    // Vivado 2018.3 FFT IP配置参数
    localparam FFT_IP_VERSION    = "9.1";        // Vivado 2018.3内置FFT IP版本
    localparam FFT_IP_ARCH       = 3;            // Pipelined Streaming I/O架构
    localparam FFT_INPUT_WIDTH   = DATA_WIDTH;   // 输入位宽
    localparam FFT_OUTPUT_WIDTH  = DATA_WIDTH;   // 输出位宽
    localparam FFT_PHASE_FACTOR  = 24;           // 相位因子位宽
    // FFT IP 实测延迟（Pipelined Streaming，256点，供分包3调度）
    localparam FFT_IP_LATENCY    = 310;          // 输入首点→输出首点周期数，TB实测后更新
    // 目标频段带通门（doc/07 改动①，2026-10-05 实测定标）
    localparam [FFT_IDX_WIDTH-1:0] MASK_BIN_LO = 26;    // 通带下边沿频点（1016 Hz）
    localparam [FFT_IDX_WIDTH-1:0] MASK_BIN_HI = 127;   // 通带上边沿频点（4961 Hz）
    // 前端高通 + 固定增益（2026-10-23 实测标定）
    localparam HPF_ENABLE     = 1;   // 1=启用板上高通
    localparam HPF_GAIN_SHIFT = 4;   // 固定增益 2^4（仅在 AGC_ENABLE=0 时使用）
    // 共享 AGC（2026-10-24，远距离 15~50 m）
    localparam AGC_ENABLE     = 1;   // 1=启用共享 AGC（HPF 固定增益置 0）
    localparam AGC_TARGET_BIT = 22;  // 输出峰值目标最高位
    localparam AGC_GMAX       = 8;   // 最大增益 2^8 = ×256
endpackage
