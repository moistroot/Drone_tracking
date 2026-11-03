// ============================================================
// tb_mic_collect_top.sv
// 端到端集成测试：I2S 输入 → 帧缓存 → 读接口 全链路验证
//
// 【验收标准覆盖说明】
// 任务书验收标准第1条要求"仿真连续1000帧，零丢点零错位"。
// 本 Testbench 跑端到端集成（I2S 输入→帧缓存→读接口），
// 按真实 10kHz 节拍，一帧 25.6ms，仿真窗口 130ms 内完成 5 帧；
// 1000 帧连续性验证由 tb_mic_frame_buffer.sv 完成（加速时钟，
// 每点 20 拍，一帧 5120 拍，跑满 1000 帧）。二者合起来覆盖：
//   - tb_mic_frame_buffer.sv  → 帧号连续、无丢帧、读写一致（1000帧）
//   - tb_mic_collect_top.sv   → I2S 输入→帧输出→读接口 全链路正确（5帧，真实节拍）
// ============================================================
`timescale 1ns/1ps
`include "project_global_param.vh"

module tb_mic_collect_top;

    reg clk_50m = 0;
    reg rst_n   = 0;

    reg  [MAX_CHAN_NUM-1:0] mic_pdm_sdin;
    wire                    mic_bclk;
    wire                    mic_lrck;
    wire [3:0]              board_led;

    wire [DATA_WIDTH-1:0]   mic_ch_data [0:MAX_CHAN_NUM-1];
    wire [FRAME_CNT_WIDTH-1:0] mic_frame_cnt;
    wire                    mic_frame_valid;

    reg                     mic_rd_en;
    reg  [FFT_IDX_WIDTH-1:0]  mic_rd_addr;
    wire [DATA_WIDTH-1:0]     mic_rd_data [0:MAX_CHAN_NUM-1];
    wire                      agc_gain_changed;   // HPF_BYPASS=1 时 AGC 关闭，恒 0

    // HPF_BYPASS=1：本 TB 用合成数据（frame_id*1000+addr）逐点比对读口，
    // 过完 900 Hz 高通就失去比对基准；高通本身由 tb_highpass_900 单独验证。
    mic_collect_top #(.HPF_BYPASS(1)) u_dut (
        .clk_50m         (clk_50m),
        .rst_n           (rst_n),
        .mic_pdm_sdin    (mic_pdm_sdin),
        .mic_bclk        (mic_bclk),
        .mic_lrck        (mic_lrck),
        .board_led       (board_led),
        .mic_ch_data     (mic_ch_data),
        .mic_frame_cnt   (mic_frame_cnt),
        .mic_frame_valid (mic_frame_valid),
        .mic_rd_en       (mic_rd_en),
        .mic_rd_addr     (mic_rd_addr),
        .mic_rd_data     (mic_rd_data),
        .agc_gain_changed(agc_gain_changed)
    );

    always #10 clk_50m = ~clk_50m;

    reg bclk_d, lrck_d;
    always @(posedge clk_50m) begin
        bclk_d <= mic_bclk;
        lrck_d <= mic_lrck;
    end
    wire bclk_fall = !mic_bclk && bclk_d;
    wire lrck_fall = !mic_lrck && lrck_d;

    reg [23:0] tx_shift [0:3];
    reg [5:0]  tx_bit_cnt;
    reg        tx_active;
    reg [15:0] tx_frame_id;
    reg [15:0] sample_idx;

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            tx_bit_cnt   <= 6'd0;
            tx_active    <= 1'b0;
            mic_pdm_sdin <= {MAX_CHAN_NUM{1'b0}};
            tx_frame_id  <= 16'd0;
            sample_idx   <= 16'd0;
        end else begin
            if (lrck_fall) begin
                tx_shift[0] <= tx_frame_id * 1000 + sample_idx;
                tx_shift[1] <= tx_frame_id * 1000 + sample_idx;
                tx_shift[2] <= tx_frame_id * 1000 + sample_idx;
                tx_shift[3] <= tx_frame_id * 1000 + sample_idx;
                tx_bit_cnt  <= 6'd0;
                tx_active   <= 1'b1;
                sample_idx  <= sample_idx + 1;
                if (sample_idx == CUR_FFT_POINT - 1) begin
                    sample_idx  <= 0;
                    tx_frame_id <= tx_frame_id + 1;
                end
            end else if (bclk_fall && tx_active) begin
                // 标准 I2S：LRCLK 边沿后的第一个 BCLK 下降沿即送 MSB
                mic_pdm_sdin[0] <= tx_shift[0][23];
                mic_pdm_sdin[1] <= tx_shift[1][23];
                mic_pdm_sdin[2] <= tx_shift[2][23];
                mic_pdm_sdin[3] <= tx_shift[3][23];
                for (int i = 0; i < 4; i++)
                    tx_shift[i] <= {tx_shift[i][22:0], 1'b0};
                if (tx_bit_cnt == 6'd23)
                    tx_active <= 1'b0;
                else
                    tx_bit_cnt <= tx_bit_cnt + 1'b1;
            end
        end
    end

    integer frame_cnt_check = 0;
    reg     frame_valid_d;

    always @(posedge clk_50m) begin
        frame_valid_d <= mic_frame_valid;
        if (mic_frame_valid) begin
            frame_cnt_check = frame_cnt_check + 1;
            $display("Time %0t: Frame %0d valid, frame_cnt=%0d",
                     $time, frame_cnt_check, mic_frame_cnt);
        end
        if (mic_frame_valid && frame_valid_d) begin
            $display("ERROR: mic_frame_valid not single-cycle at time %0t", $time);
        end
    end

    integer read_err = 0;
    reg [1:0] rd_state;
    reg [15:0] last_frame_id;
    reg [FFT_IDX_WIDTH-1:0]  cur_rd_addr;
    integer frames_read = 0;

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            mic_rd_en   <= 1'b0;
            mic_rd_addr <= 0;
            rd_state    <= 2'd0;
            last_frame_id <= 0;
            frames_read   <= 0;
        end else begin
            mic_rd_en <= 1'b0;
            case (rd_state)
                2'd0: begin
                    if (mic_frame_valid) begin
                        rd_state      <= 2'd1;
                        last_frame_id <= mic_frame_cnt - 1;
                        cur_rd_addr   <= 0;
                    end
                end
                2'd1: begin
                    mic_rd_en   <= 1'b1;
                    mic_rd_addr <= cur_rd_addr;
                    rd_state    <= 2'd2;
                end
                2'd2: begin
                    // 读口为同步读：数据比地址晚一拍 → 等一拍再校验
                    rd_state <= 2'd3;
                end
                2'd3: begin
                    // 4 通道并行读：同一地址同时返回 4 路
                    for (int c = 0; c < CUR_CHAN_NUM; c = c + 1) begin
                        if (mic_rd_data[c] !== (last_frame_id * 1000 + cur_rd_addr)) begin
                            read_err = read_err + 1;
                            $display("READ ERROR: frame=%0d ch=%0d addr=%0d got=%0d exp=%0d",
                                     last_frame_id, c, cur_rd_addr,
                                     mic_rd_data[c], last_frame_id * 1000 + cur_rd_addr);
                        end
                    end
                    if (cur_rd_addr == CUR_FFT_POINT - 1) begin
                        rd_state    <= 2'd0;
                        frames_read <= frames_read + 1;
                    end else begin
                        cur_rd_addr <= cur_rd_addr + 1;
                        rd_state    <= 2'd1;
                    end
                end
                default: rd_state <= 2'd0;
            endcase
        end
    end

    initial begin
        rst_n = 0;
        #200;
        rst_n = 1;
        #130_000_000;

        $display("========================================");
        $display("Frame count check  : %0d", frame_cnt_check);
        $display("Frames read        : %0d", frames_read);
        $display("Read errors        : %0d", read_err);
        $display("LED status         : %b", board_led);
        $display("========================================");
        if (read_err == 0 && frame_cnt_check > 0)
            $display("PASS");
        else
            $display("FAIL");
        $finish;
    end

endmodule