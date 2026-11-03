`timescale 1ns/1ps
`include "project_global_param.vh"
module pair_scheduler (
    input  wire         clk_50m,
    input  wire         rst_n,
   
    input  wire         pre_frame_valid,       
    input  wire [FFT_IDX_WIDTH-1:0] fft_idx,  
    input  wire [DATA_WIDTH-1:0] fft_re [0:MAX_CHAN_NUM-1],
    input  wire [DATA_WIDTH-1:0] fft_im [0:MAX_CHAN_NUM-1],
    input  wire [FFT_BINS-1:0] freq_valid_mask,
    input  wire [FRAME_CNT_WIDTH-1:0] pre_frame_cnt,
    input  wire [9:0]   fft_ip_latency,        // 联调参数
    // IFFT 数据流（对接 ifft_peak_detect）
    output wire [DATA_WIDTH-1:0] ifft_din_re,
    output wire [DATA_WIDTH-1:0] ifft_din_im,
    output wire         ifft_din_valid,
    output wire         ifft_din_last,
    input  wire         ifft_din_ready,  
    output reg          ifft_cfg_valid,  
    // 峰值检测结果
    input  wire [TDOA_WIDTH-1:0] tdoa_in,     
    input  wire         tdoa_in_valid,
    input  wire         tdoa_in_used,          
    // TDOA 结果与状态
    output reg  [TDOA_WIDTH-1:0] tdoa_result [0:MAX_PAIR_NUM-1],
    output reg  [MAX_PAIR_NUM-1:0] pair_used,  
    output reg          all_done,              
    output reg  [FRAME_CNT_WIDTH-1:0] frame_cnt_out,
    output reg          busy                  
);
    // 常数
    localparam [FFT_IDX_WIDTH-1:0] POINTS_HALF = CUR_FFT_POINT/2;   // 128
    localparam integer PIPE_DLY = 48;            
    localparam [2:0] S_IDLE       = 3'd0; 
    localparam [2:0] S_FREQ_LOOP  = 3'd1;  
    localparam [2:0] S_FREQ_TAIL  = 3'd2;  
    localparam [2:0] S_IFFT_SEND  = 3'd3;  
    localparam [2:0] S_PEAK_WAIT  = 3'd4;  
    localparam [2:0] S_DONE       = 3'd5;  
    reg [2:0]  state;
    reg [5:0]  pair_cnt;
    reg [FFT_IDX_WIDTH-1:0] freq_cnt;
    reg [FFT_IDX_WIDTH-1:0] send_cnt;
    reg [CHAN_IDX_WIDTH-1:0] ch_a, ch_b;
    reg        burst_seen;
    wire load_en = (state == S_FREQ_LOOP) && (freq_cnt <= POINTS_HALF);
  
    reg  send_vld_d1, send_vld_d2, send_last_d1, send_last_d2;
    wire send_stall = send_vld_d2 && !ifft_din_ready;
  
    function automatic [CHAN_IDX_WIDTH-1:0] get_ch_a(input [5:0] idx);
        reg [5:0] cnt;
        integer i, j;
        begin
            cnt = 0;
            get_ch_a = 0;
            for (i = 0; i < MAX_CHAN_NUM; i = i + 1)
                for (j = i + 1; j < MAX_CHAN_NUM; j = j + 1) begin
                    if (cnt == idx) get_ch_a = i[CHAN_IDX_WIDTH-1:0];
                    cnt = cnt + 1;
                end
        end
    endfunction
    function automatic [CHAN_IDX_WIDTH-1:0] get_ch_b(input [5:0] idx);
        reg [5:0] cnt;
        integer i, j;
        begin
            cnt = 0;
            get_ch_b = 0;
            for (i = 0; i < MAX_CHAN_NUM; i = i + 1)
                for (j = i + 1; j < MAX_CHAN_NUM; j = j + 1) begin
                    if (cnt == idx) get_ch_b = j[CHAN_IDX_WIDTH-1:0];
                    cnt = cnt + 1;
                end
        end
    endfunction
  
    reg [47:0] in_rd [0:MAX_CHAN_NUM-1];    
    reg [47:0] in_sel_a, in_sel_b;
    integer ch;
    generate
        for (genvar gc = 0; gc < MAX_CHAN_NUM; gc = gc + 1) begin: inbuf_gen
            (* ram_style = "block" *) reg [47:0] in_buf [0:FFT_BINS-1];
            always @(posedge clk_50m) begin
                if (pre_frame_valid && (fft_idx < FFT_BINS))
                    in_buf[fft_idx] <= {fft_im[gc], fft_re[gc]};
            end
            always @(posedge clk_50m) begin
                if (load_en)
                    in_rd[gc] <= in_buf[freq_cnt];
            end
        end
    endgenerate
    always @(posedge clk_50m) begin
        in_sel_a <= in_rd[ch_a];
        in_sel_b <= in_rd[ch_b];
    end
    
    (* ram_style = "block" *) reg [47:0] phat_buf [0:FFT_BINS-1];
   
    wire learn_mask_zero = (freq_valid_mask == {FFT_BINS{1'b0}});
    // 主状态机
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            state    <= S_IDLE;
            pair_cnt <= 0;
            freq_cnt <= 0;
            send_cnt <= 0;
            all_done <= 1'b0;
            pair_used <= {MAX_PAIR_NUM{1'b0}};
            busy     <= 1'b0;
            ifft_cfg_valid <= 1'b0;
        end else begin
            all_done <= 1'b0;
            case (state)
                S_IDLE: begin
                  
                    if (!pre_frame_valid && burst_seen) begin
                        state    <= S_FREQ_LOOP;
                        pair_cnt <= 0;
                        freq_cnt <= 0;
                        ch_a     <= get_ch_a(0);
                        ch_b     <= get_ch_b(0);
                        busy     <= 1'b1;
                    end
                end
                S_FREQ_LOOP: begin
                    if (freq_cnt == POINTS_HALF) begin   // 0..128 共 129 拍
                        state    <= S_FREQ_TAIL;
                        freq_cnt <= 0;
                    end else begin
                        freq_cnt <= freq_cnt + 1'b1;
                    end
                end
                S_FREQ_TAIL: begin
                    if (freq_cnt == 6'd4)
                        ifft_cfg_valid <= 1'b1;
                    else if (freq_cnt == 6'd5)
                        ifft_cfg_valid <= 1'b0;
                    if (freq_cnt == PIPE_DLY) begin
                        state    <= S_IFFT_SEND;
                        send_cnt <= 0;
                        freq_cnt <= 0;
                    end else
                        freq_cnt <= freq_cnt + 1'b1;
                end
                S_IFFT_SEND: begin
                    if (send_cnt == IFFT_POINT-1) begin   
                        if (!send_stall) begin
                            send_cnt <= 0;
                            state    <= S_PEAK_WAIT;
                        end
                    end else if (!send_stall)
                        send_cnt <= send_cnt + 1'b1;
                end
                S_PEAK_WAIT: begin
                    if (tdoa_in_valid) begin
                        tdoa_result[pair_cnt] <= tdoa_in;
                        pair_used[pair_cnt]   <= tdoa_in_used;
                        if (pair_cnt == MAX_PAIR_NUM-1) begin
                            state    <= S_DONE;
                            all_done <= 1'b1;
                        end else begin
                            pair_cnt <= pair_cnt + 1'b1;
                            ch_a     <= get_ch_a(pair_cnt + 1'b1);
                            ch_b     <= get_ch_b(pair_cnt + 1'b1);
                            freq_cnt <= 0;
                            state    <= S_FREQ_LOOP;
                        end
                    end
                end
                S_DONE: begin
                    busy  <= 1'b0;
                    state <= S_IDLE;
                end
                default: state <= S_IDLE;
            endcase
        end
    end
   
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)
            burst_seen <= 1'b0;
        else if (pre_frame_valid)
            burst_seen <= 1'b1;
        else if (state != S_IDLE)
            burst_seen <= 1'b0;
    end
 
    reg [DATA_WIDTH-1:0] m_re_w, m_im_w;
    wire [DATA_WIDTH-1:0] c_re, c_im;
    complex_mult_24b u_mult (
        .clk_50m (clk_50m),
        .rst_n   (rst_n),
        .a_re    (in_sel_a[23:0]),
        .a_im    (in_sel_a[47:24]),
        .b_re    (in_sel_b[23:0]),
        .b_im    (in_sel_b[47:24]),
        .out_re  (m_re_w),
        .out_im  (m_im_w)
    );
    cordic_phat_norm_24b u_cordic (
        .clk_50m (clk_50m),
        .rst_n   (rst_n),
        .re_in   (m_re_w),
        .im_in   (m_im_w),
        .re_out  (c_re),
        .im_out  (c_im)
    );
    reg [PIPE_DLY-1:0] wr_vld;
    reg [PIPE_DLY-1:0] wr_msk;
    reg [FFT_IDX_WIDTH-1:0] addr_pipe [0:PIPE_DLY-1];
    integer d;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            wr_vld <= 0;
            wr_msk <= 0;
            for (d = 0; d < PIPE_DLY; d = d + 1)
                addr_pipe[d] <= 0;
        end else begin
            wr_vld <= {wr_vld[PIPE_DLY-2:0], load_en};
            wr_msk <= {wr_msk[PIPE_DLY-2:0], freq_valid_mask[freq_cnt]};
            addr_pipe[0] <= freq_cnt;
            for (d = 1; d < PIPE_DLY; d = d + 1)
                addr_pipe[d] <= addr_pipe[d-1];
        end
    end
    always @(posedge clk_50m) begin
        if (wr_vld[PIPE_DLY-1])
            phat_buf[addr_pipe[PIPE_DLY-1]] <= (wr_msk[PIPE_DLY-1] && !learn_mask_zero) ?
                                               {c_im, c_re} : 48'b0;
    end
  
    wire inv_flag = (send_cnt >= (IFFT_POINT - CUR_FFT_POINT/2));   // k≥896
    wire zero_flag = (send_cnt > POINTS_HALF) && (send_cnt < (IFFT_POINT - CUR_FFT_POINT/2));
    wire [FFT_IDX_WIDTH-1:0] phat_rd_addr = inv_flag ?
                                             (IFFT_POINT - send_cnt) :
                                             (zero_flag ? {FFT_IDX_WIDTH{1'b0}} : send_cnt);
    reg [DATA_WIDTH-1:0] phat_re_d1, phat_im_d1;
    reg [DATA_WIDTH-1:0] phat_re_d2, phat_im_d2;
    reg inv_d1, inv_d2, zero_d1, zero_d2;
    wire send_en  = (state == S_IFFT_SEND);
    wire send_last = (send_cnt == IFFT_POINT-1);
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            phat_re_d1 <= 0; phat_im_d1 <= 0;
            phat_re_d2 <= 0; phat_im_d2 <= 0;
            inv_d1 <= 0; inv_d2 <= 0;
            zero_d1 <= 0; zero_d2 <= 0;
            send_vld_d1 <= 0; send_vld_d2 <= 0;
            send_last_d1 <= 0; send_last_d2 <= 0;
        end else if (!send_stall) begin
            phat_re_d1 <= phat_buf[phat_rd_addr][23:0];
            phat_im_d1 <= phat_buf[phat_rd_addr][47:24];
            inv_d1     <= inv_flag;
            zero_d1    <= zero_flag;
            send_vld_d1<= send_en;
            send_last_d1 <= send_last;
            phat_re_d2 <= zero_d1 ? 24'b0 : phat_re_d1;
            phat_im_d2 <= zero_d1 ? 24'b0 : (inv_d1 ? -phat_im_d1 : phat_im_d1);
            zero_d2    <= zero_d1;
            send_vld_d2<= send_vld_d1;
            send_last_d2 <= send_last_d1;
        end
    end
    assign ifft_din_re    = phat_re_d2;
    assign ifft_din_im    = phat_im_d2;
    assign ifft_din_valid = send_vld_d2;
    assign ifft_din_last  = send_last_d2;
    // 帧号输出
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)
            frame_cnt_out <= {FRAME_CNT_WIDTH{1'b0}};
        else if (state == S_DONE)
            frame_cnt_out <= pre_frame_cnt;
    end
endmodule
