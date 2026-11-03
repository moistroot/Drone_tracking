// fft_parallel_scheduler.sv - 分包2子模块2.4：4 路并行 FFT 调度与缓存
`timescale 1ns/1ps
`include "project_global_param.vh"
module fft_parallel_scheduler (
    input  wire         clk_50m,
    input  wire         rst_n,
    // 时域帧输入（对接分包1 mic_frame_buffer 的并行读口）
    input  wire [FRAME_CNT_WIDTH-1:0] mic_frame_cnt,
    input  wire         mic_frame_valid,
    output wire         rd_en,
    output wire [FFT_IDX_WIDTH-1:0] rd_addr,
    input  wire [DATA_WIDTH-1:0] rd_data [0:MAX_CHAN_NUM-1],
   
    input  wire [DATA_WIDTH-1:0] win_coef,
    // 频域帧输出（对接分包3）
    output reg  [FFT_IDX_WIDTH-1:0] fft_idx,
    output reg  [DATA_WIDTH-1:0] fft_re [0:MAX_CHAN_NUM-1],
    output reg  [DATA_WIDTH-1:0] fft_im [0:MAX_CHAN_NUM-1],
    output reg  [FRAME_CNT_WIDTH-1:0] pre_frame_cnt,
    output reg          pre_frame_valid,
    // 掩码模块接口
    output wire         mask_valid,
    output wire [FFT_IDX_WIDTH-1:0] mask_idx,
    output wire [DATA_WIDTH-1:0] mask_re,
    output wire [DATA_WIDTH-1:0] mask_im,
    output reg          mask_frame_end,
    // 帧号同步检查
    output reg          frame_sync_err
);
    // 常量
    localparam [FFT_IDX_WIDTH-1:0] POINTS      = CUR_FFT_POINT;      // 256
    localparam [FFT_IDX_WIDTH-1:0] POINTS_HALF = CUR_FFT_POINT/2;    // 128
    localparam integer PT_W = $clog2(CUR_FFT_POINT);                 // 8
    // 4 颗 FFT 的 AXI-S 互连（模块内部）
    wire [MAX_CHAN_NUM-1:0] fft_cfg_valid;
    wire [DATA_WIDTH-1:0]   fft_s_re [0:MAX_CHAN_NUM-1];
    wire [MAX_CHAN_NUM-1:0] fft_s_valid;
    wire [MAX_CHAN_NUM-1:0] fft_s_last;
    wire [MAX_CHAN_NUM-1:0] fft_s_ready;
    wire [DATA_WIDTH-1:0]   fft_m_re [0:MAX_CHAN_NUM-1];
    wire [DATA_WIDTH-1:0]   fft_m_im [0:MAX_CHAN_NUM-1];
    wire [7:0]              fft_m_idx [0:MAX_CHAN_NUM-1];
    wire [MAX_CHAN_NUM-1:0] fft_m_valid;
    wire [MAX_CHAN_NUM-1:0] fft_m_last;
    // 状态机
    localparam [2:0] S_IDLE     = 3'd0;
    localparam [2:0] S_CFG_WAIT = 3'd1;   // 等配置被 4 颗 IP 接收
    localparam [2:0] S_CFG_GAP  = 3'd2;   // 配置与首点之间的间隔
    localparam [2:0] S_DATA     = 3'd3;   // 并行送 256 点 × 4 通道
    localparam [2:0] S_OUT_WAIT = 3'd4;   // 等 4 通道 FFT 输出全部收完
    localparam [2:0] S_BURST    = 3'd5;   // 129 拍突发输出
    reg  [2:0]   state;
    reg  [PT_W-1:0] in_cnt;              
    reg  [3:0]   cfg_wait_cnt;
    reg  [1:0]   gap_cnt;
    reg  [FFT_IDX_WIDTH-1:0] out_cnt;    
    reg  [MAX_CHAN_NUM-1:0] ch_done;      // m_last 到
    reg  [FRAME_CNT_WIDTH-1:0] locked_frame_cnt;   // 帧号锁存
    reg  [FRAME_CNT_WIDTH-1:0] last_frame_cnt;
    wire rd_advance = (state == S_DATA) && (&fft_s_ready);
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            state        <= S_IDLE;
            in_cnt       <= {PT_W{1'b0}};
            cfg_wait_cnt <= 4'd0;
            gap_cnt      <= 2'd0;
            out_cnt      <= {FFT_IDX_WIDTH{1'b0}};
        end else begin
            case (state)
                S_IDLE: begin
                    if (mic_frame_valid) begin
                        state        <= S_CFG_WAIT;
                        in_cnt       <= {PT_W{1'b0}};
                        cfg_wait_cnt <= 4'd0;
                    end
                end
                S_CFG_WAIT: begin
                    // 固定等 8 拍：配置通道握手 + IP 在帧边界应用配置的裕量
                    if (cfg_wait_cnt >= 4'd7) begin
                        state   <= S_CFG_GAP;
                        gap_cnt <= 2'd0;
                    end else
                        cfg_wait_cnt <= cfg_wait_cnt + 1'b1;
                end
                S_CFG_GAP: begin
                    if (gap_cnt == 2'd3)
                        state <= S_DATA;
                    else
                        gap_cnt <= gap_cnt + 1'b1;
                end
                S_DATA: begin
                    if (rd_advance) begin
                        if (in_cnt == POINTS-1) begin
                            in_cnt <= {PT_W{1'b0}};
                            state  <= S_OUT_WAIT;
                        end else
                            in_cnt <= in_cnt + 1'b1;
                    end
                end
                S_OUT_WAIT: begin
                    if (&ch_done) begin
                        out_cnt <= {FFT_IDX_WIDTH{1'b0}};
                        state   <= S_BURST;
                    end
                end
                S_BURST: begin
                    if (out_cnt == POINTS_HALF + 1)
                        state <= S_IDLE;
                    else
                        out_cnt <= out_cnt + 1'b1;
                end
                default: state <= S_IDLE;
            endcase
        end
    end
    // 配置脉冲（进入 CFG_WAIT 时发 1 拍）
    reg cfg_valid_reg;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)
            cfg_valid_reg <= 1'b0;
        else
            cfg_valid_reg <= (state == S_IDLE && mic_frame_valid);
    end
    assign fft_cfg_valid = {MAX_CHAN_NUM{cfg_valid_reg}};
    // 并行读帧缓存 + 窗 ROM
    assign rd_en   = rd_advance;
    assign rd_addr = {{(FFT_IDX_WIDTH-PT_W){1'b0}}, in_cnt};
    reg s_valid_d1, s_valid_d2, s_last_d1, s_last_d2;
    wire s_valid_src = rd_advance;
    wire s_last_src  = rd_advance && (in_cnt == POINTS-1);
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            s_valid_d1 <= 1'b0; s_valid_d2 <= 1'b0;
            s_last_d1  <= 1'b0; s_last_d2  <= 1'b0;
        end else begin
            s_valid_d1 <= s_valid_src;
            s_valid_d2 <= s_valid_d1;
            s_last_d1  <= s_last_src;
            s_last_d2  <= s_last_d1;
        end
    end
    assign fft_s_valid = {MAX_CHAN_NUM{s_valid_d2}};
    assign fft_s_last  = {MAX_CHAN_NUM{s_last_d2}};
    // 4 路并行的汉宁窗 + FFT
    generate
        for (genvar ch = 0; ch < MAX_CHAN_NUM; ch = ch + 1) begin: fft_gen
            wire [DATA_WIDTH-1:0] win_out;
            window_oper_24b u_window (
                .clk_50m  (clk_50m),
                .rst_n    (rst_n),
                .data_in  (rd_data[ch]),   // BRAM 输出，T+1 有效
                .win_coef (win_coef),      // ROM 输出，T+1 有效
                .data_out (win_out)        // T+2 有效
            );
            assign fft_s_re[ch] = win_out;
            fft_256_wrapper u_fft (
                .clk                 (clk_50m),
                .rst_n               (rst_n),
                .cfg_valid           (cfg_valid_reg),
                .s_data_re           (win_out),
                .s_valid             (s_valid_d2),
                .s_ready             (fft_s_ready[ch]),
                .s_last              (s_last_d2),
                .m_data_re           (fft_m_re[ch]),
                .m_data_im           (fft_m_im[ch]),
                .m_idx               (fft_m_idx[ch]),
                .m_valid             (fft_m_valid[ch]),
                .m_last              (fft_m_last[ch]),
                .event_frame_started (),
                .event_tlast_missing (),
                .cfg_ready           ()
            );
        end
    endgenerate
    // 频域 BRAM（每通道 129 × 48bit）
    reg [DATA_WIDTH-1:0] bram_re [0:MAX_CHAN_NUM-1];
    reg [DATA_WIDTH-1:0] bram_im [0:MAX_CHAN_NUM-1];
    generate
        for (genvar ch = 0; ch < MAX_CHAN_NUM; ch = ch + 1) begin: fb_gen
            (* ram_style = "block" *) reg [47:0] freq_bram [0:POINTS_HALF];
            always @(posedge clk_50m) begin
                if (fft_m_valid[ch] && (fft_m_idx[ch] <= POINTS_HALF))
                    freq_bram[fft_m_idx[ch]] <= {fft_m_im[ch], fft_m_re[ch]};
            end
            always @(posedge clk_50m) begin
                if (state == S_BURST && out_cnt <= POINTS_HALF) begin
                    bram_re[ch] <= freq_bram[out_cnt][23:0];
                    bram_im[ch] <= freq_bram[out_cnt][47:24];
                end
            end
        end
    endgenerate
    // 输出完成跟踪
    integer cd;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            ch_done <= {MAX_CHAN_NUM{1'b0}};
        end else begin
            if (state == S_IDLE && mic_frame_valid)
                ch_done <= {MAX_CHAN_NUM{1'b0}};
            else begin
                for (cd = 0; cd < MAX_CHAN_NUM; cd = cd + 1)
                    if (fft_m_valid[cd] && fft_m_last[cd])
                        ch_done[cd] <= 1'b1;
            end
        end
    end
    // 突发输出（129 周期，BRAM 读延迟 1 拍对齐）
    reg [FFT_IDX_WIDTH-1:0] out_cnt_d;
    integer ch;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)
            out_cnt_d <= POINTS_HALF + 1'b1;
        else
            out_cnt_d <= out_cnt;
    end
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            pre_frame_valid <= 1'b0;
            pre_frame_cnt   <= {FRAME_CNT_WIDTH{1'b0}};
            fft_idx         <= {FFT_IDX_WIDTH{1'b0}};
            for (ch = 0; ch < MAX_CHAN_NUM; ch = ch + 1) begin
                fft_re[ch] <= {DATA_WIDTH{1'b0}};
                fft_im[ch] <= {DATA_WIDTH{1'b0}};
            end
        end else begin
            pre_frame_valid <= (state == S_BURST) && (out_cnt_d <= POINTS_HALF);
            if (state == S_BURST && out_cnt_d == 0)
                pre_frame_cnt <= locked_frame_cnt;
            if (state == S_BURST) begin
                fft_idx <= out_cnt_d;
                for (ch = 0; ch < MAX_CHAN_NUM; ch = ch + 1) begin
                    fft_re[ch] <= bram_re[ch];
                    fft_im[ch] <= bram_im[ch];
                end
            end
        end
    end
    // 帧号锁存与连续性检查
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            locked_frame_cnt <= {FRAME_CNT_WIDTH{1'b0}};
            last_frame_cnt   <= {FRAME_CNT_WIDTH{1'b0}};
            frame_sync_err   <= 1'b0;
        end else begin
            if (mic_frame_valid) begin
                locked_frame_cnt <= mic_frame_cnt;
                if (mic_frame_cnt != last_frame_cnt + 1'b1 &&
                    !(last_frame_cnt == {FRAME_CNT_WIDTH{1'b1}} && mic_frame_cnt == 0))
                    frame_sync_err <= 1'b1;
                else
                    frame_sync_err <= 1'b0;
                last_frame_cnt <= mic_frame_cnt;
            end
        end
    end
    // 掩码接口
    assign mask_valid = pre_frame_valid;
    assign mask_idx   = fft_idx;
    assign mask_re    = fft_re[0];
    assign mask_im    = fft_im[0];
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)
            mask_frame_end <= 1'b0;
        else
            mask_frame_end <= (state == S_BURST) && (out_cnt == POINTS_HALF + 1);
    end
endmodule
