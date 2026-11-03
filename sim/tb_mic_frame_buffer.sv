`timescale 1ns/1ps
`include "project_global_param.vh"

module tb_mic_frame_buffer;

    reg clk_50m = 0;
    reg rst_n   = 0;

    reg  [DATA_WIDTH-1:0]   ch_data [0:MAX_CHAN_NUM-1];
    reg  [MAX_CHAN_NUM-1:0] ch_valid;

    reg                       rd_en;
    reg  [FFT_IDX_WIDTH-1:0]  rd_addr;
    wire [DATA_WIDTH-1:0]     rd_data [0:MAX_CHAN_NUM-1];

    wire                      frame_valid;
    wire [FRAME_CNT_WIDTH-1:0] frame_cnt;
    wire                      overflow_error;

    mic_frame_buffer u_mic_frame_buffer (
        .clk_50m        (clk_50m),
        .rst_n          (rst_n),
        .ch_data        (ch_data),
        .ch_valid       (ch_valid),
        .rd_en          (rd_en),
        .rd_addr        (rd_addr),
        .rd_data        (rd_data),
        .frame_valid    (frame_valid),
        .frame_cnt      (frame_cnt),
        .overflow_error (overflow_error)
    );

    always #10 clk_50m = ~clk_50m;

    integer data_cnt    = 0;
    integer frame_count = 0;
    integer sample_in_frame = 0;
    reg [15:0] write_frame_id;

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            ch_valid <= 0;
            for (int i = 0; i < MAX_CHAN_NUM; i++) ch_data[i] <= 0;
            data_cnt <= 0;
            sample_in_frame <= 0;
            write_frame_id <= 0;
        end else begin
            ch_valid <= 0;
            // 采样间隔 20 拍：一帧 256×20 = 5120 拍 > 读取侧一帧所需
            // 256 地址 × 3 拍 = 768 拍（4 通道并行），余量 > 6 倍
            if (data_cnt % 20 == 0) begin
                ch_valid <= {MAX_CHAN_NUM{1'b1}};
                for (int i = 0; i < CUR_CHAN_NUM; i++)
                    ch_data[i] <= write_frame_id * 1000 + sample_in_frame;
                sample_in_frame <= sample_in_frame + 1;
                if (sample_in_frame == CUR_FFT_POINT - 1) begin
                    sample_in_frame <= 0;
                    write_frame_id  <= write_frame_id + 1;
                end
            end
            data_cnt <= data_cnt + 1;
        end
    end

    always @(posedge clk_50m) begin
        if (frame_valid) begin
            frame_count = frame_count + 1;
            $display("Frame %0d valid, frame_cnt=%0d", frame_count, frame_cnt);
            if (frame_cnt !== frame_count[FRAME_CNT_WIDTH-1:0]) begin
                $display("ERROR: frame_cnt mismatch, expected %0d, got %0d",
                         frame_count, frame_cnt);
            end
        end
    end

    integer read_err = 0;

    reg [1:0]  rd_state;
    integer    frame_done_cnt;
    reg [15:0] last_written_frame;

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            rd_en    <= 1'b0;
            rd_addr  <= 0;
            rd_state <= 0;
            frame_done_cnt <= 0;
            last_written_frame <= 0;
        end else begin
            rd_en <= 1'b0;
            case (rd_state)
                2'd0: begin
                    if (frame_valid) begin
                        rd_state <= 2'd1;
                        rd_addr  <= 0;
                        last_written_frame <= frame_cnt - 1;
                    end
                end
                2'd1: begin
                    rd_en    <= 1'b1;
                    rd_addr  <= rd_addr;
                    rd_state <= 2'd2;
                end
                2'd2: begin
                    // 读口为同步读：rd_en 在本拍有效，数据下一拍才到 → 等一拍
                    rd_state <= 2'd3;
                end
                2'd3: begin
                    // 4 通道并行读：同一地址同时返回 4 路，逐一比对
                    for (int c = 0; c < CUR_CHAN_NUM; c = c + 1) begin
                        if (rd_data[c] !== (last_written_frame * 1000 + rd_addr)) begin
                            read_err = read_err + 1;
                            $display("READ ERROR: ch=%0d addr=%0d got=%0d exp=%0d",
                                     c, rd_addr, rd_data[c],
                                     last_written_frame * 1000 + rd_addr);
                        end
                    end
                    if (rd_addr == CUR_FFT_POINT - 1) begin
                        rd_state       <= 2'd0;
                        frame_done_cnt <= frame_done_cnt + 1;
                    end else begin
                        rd_addr  <= rd_addr + 1;
                        rd_state <= 2'd1;
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
        // 1000 帧 × 256 点 × 20 拍 × 20ns ≈ 102 ms
        #110_000_000;
        $display("========================================");
        $display("Total frames written : %0d", frame_count);
        $display("Total frames read    : %0d", frame_done_cnt);
        $display("Read errors          : %0d", read_err);
        $display("========================================");
        if (read_err == 0 && frame_done_cnt > 0)
            $display("PASS");
        else
            $display("FAIL");
        $finish;
    end

endmodule