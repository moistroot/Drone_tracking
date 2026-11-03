//ƒÊFFT∏¥”√”Î∑Â÷µºÏ≤‚
`timescale 1ns/1ps
`include "project_global_param.vh"

module ifft_peak_detect (
    input  wire         clk_50m,
    input  wire         rst_n,


    input  wire [DATA_WIDTH-1:0] din_re,
    input  wire [DATA_WIDTH-1:0] din_im,
    input  wire         din_valid,
    input  wire         din_last,
    input  wire         din_cfg_valid,      


    output reg  [TDOA_WIDTH-1:0] tdoa_out,    
    output reg          tdoa_valid,         
  
    output reg          tdoa_used,

   
    output wire         din_ready            
);

   
    wire [DATA_WIDTH-1:0] m_re, m_im;
    wire [FFT_IDX_WIDTH-1:0] m_idx;
    wire         m_valid, m_last;
    wire         s_ready;

   
    assign din_ready = s_ready;

  
    wire cfg_pulse = din_cfg_valid;

    fft_24b_wrapper u_ifft (
        .clk           (clk_50m),
        .rst_n         (rst_n),
        .fft_point     (IFFT_POINT[FFT_IDX_WIDTH-1:0]),  
        .fwd_inv       (1'b0),          
        .cfg_valid     (cfg_pulse),
        .s_data_re     (din_re),
        .s_data_im     (din_im),
        .s_valid       (din_valid),
        .s_ready       (s_ready),
        .s_last        (din_last),
        .m_data_re     (m_re),
        .m_data_im     (m_im),
        .m_idx         (m_idx),
        .m_valid       (m_valid),
        .m_last        (m_last),
        .event_frame_started (),
        .event_tlast_missing ()
    );

    wire [DATA_WIDTH-1:0] abs_re = m_re[DATA_WIDTH-1] ? -m_re : m_re;

   
    localparam integer PEAK_WIN = 45;
    localparam signed [15:0] TAU_MAX_Q10 = 16'sd11523;  
    wire in_win = (m_idx <= PEAK_WIN) ||
                  (m_idx >= (IFFT_POINT - PEAK_WIN));

    reg [DATA_WIDTH-1:0] peak_val;             
    reg [DATA_WIDTH-1:0] peak_L, peak_R;       
    reg [FFT_IDX_WIDTH-1:0] peak_idx;
    reg [DATA_WIDTH-1:0] prev_abs;
    reg tracking;                            
   
    reg [DATA_WIDTH-1:0] frame_last, frame_first;

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            peak_val <= 0; peak_L <= 0; peak_R <= 0;
            peak_idx <= 0; prev_abs <= 0; tracking <= 0;
            frame_last <= 0; frame_first <= 0;
        end else begin
            if (m_valid && m_idx == 0)             frame_first <= abs_re;
            if (m_valid && m_idx == IFFT_POINT-1) frame_last  <= abs_re;
            if (m_valid) begin
                prev_abs <= abs_re;
                if (in_win && (!tracking || (abs_re > peak_val))) begin
                  
                    peak_val <= abs_re;
                    peak_idx <= m_idx;
                    peak_L   <= prev_abs;
                    tracking <= 1'b1;
                    peak_R   <= 0;
                end else if (m_idx == peak_idx + 1'b1) begin
                  
                    peak_R <= abs_re;
                end
            end
            if (m_valid && m_last) begin
                tracking <= 1'b0;
             
            end
        end
    end

  
    wire [DATA_WIDTH-1:0] interp_L = (peak_idx == 0)
                                     ? frame_last : peak_L;
    wire [DATA_WIDTH-1:0] interp_R = (peak_idx == IFFT_POINT-1)
                                     ? frame_first : peak_R;

  
    wire signed [24:0] num_raw = $signed({1'b0, interp_L}) - $signed({1'b0, interp_R});
    wire signed [26:0] den     = $signed({2'b0, interp_L}) + $signed({2'b0, interp_R})
                                 - $signed({{1'b0, peak_val}, 1'b0});  
    wire [24:0] num_mag = num_raw[24] ? -num_raw : num_raw;   
    wire [26:0] den_mag = (den < 0) ? -den : den;           
    wire        delta_neg = num_raw[24] ^ den[26];         

    localparam [2:0] D_IDLE = 3'd0, D_RUN = 3'd1, D_DONE = 3'd2;
    reg [2:0]  dstate;
    reg [5:0]  d_cnt;
    reg [33:0] rem_acc;           
    reg [15:0] quotient;
    reg signed [15:0] delta_q10;

    wire [33:0] den_ext  = {7'b0, den_mag};
    wire [33:0] rem_shl  = {rem_acc[32:0], 1'b0};   
    wire [33:0] rem_sub  = rem_shl - den_ext;     
    wire        rem_ok   = !rem_sub[33];           
    wire        den_zero = (den == 27'sd0);         
    wire        den_sat  = (num_mag >= den_mag);   
    wire [15:0] q10      = {7'b0, quotient[15:7]}; 

   
    reg div_start_d;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n)
            div_start_d <= 1'b0;
        else
            div_start_d <= m_valid && m_last;
    end
    wire div_start = div_start_d;

    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            dstate <= D_IDLE; d_cnt <= 0; rem_acc <= 0;
            quotient <= 0; delta_q10 <= 0;
        end else begin
            case (dstate)
                D_IDLE: begin
                    if (div_start) begin
                        if (den_zero) begin
                            dstate    <= D_DONE;
                            delta_q10 <= 16'sd0;
                        end else if (den_sat) begin
                            dstate    <= D_DONE;
                            delta_q10 <= delta_neg ? -16'sd512 : 16'sd512;
                        end else begin
                            dstate   <= D_RUN;
                            d_cnt    <= 0;
                            rem_acc  <= {9'b0, num_mag};   
                            quotient <= 0;
                        end
                    end
                end
                D_RUN: begin
                    if (d_cnt < 6'd16) begin
                     
                        rem_acc  <= rem_ok ? rem_sub : rem_shl;
                        quotient <= {quotient[14:0], rem_ok};
                        d_cnt    <= d_cnt + 1'b1;
                    end else begin
                        dstate <= D_DONE;
                    
                        delta_q10 <= delta_neg ? -$signed(q10) : $signed(q10);
                    end
                end
                D_DONE: dstate <= D_IDLE;
                default: dstate <= D_IDLE;
            endcase
        end
    end

   
    wire signed [17:0] tdoa_raw = (peak_idx < 512) ?
                                  $signed({8'b0, peak_idx}) * 18'sd256 :
                                  ($signed({8'b0, peak_idx}) - 18'sd1024) * 18'sd256;
    wire signed [15:0] tdoa_c = (tdoa_raw > 18'sd32767) ? 16'sd32767 :
                                (tdoa_raw < -18'sd32768) ? -16'sd32768 :
                                tdoa_raw[15:0];
  
    wire signed [16:0] tdoa_sum = $signed(tdoa_c) +
                                  $signed({delta_q10[15], delta_q10[15],
                                            delta_q10[15], delta_q10[15:2]});
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            tdoa_out   <= 0;
            tdoa_valid <= 1'b0;
            tdoa_used  <= 1'b0;
        end else begin
            tdoa_valid <= 1'b0;
            if (dstate == D_DONE) begin
            
                tdoa_out   <= (tdoa_sum >  TAU_MAX_Q10) ?  TAU_MAX_Q10 :
                              (tdoa_sum < -TAU_MAX_Q10) ? -TAU_MAX_Q10 :
                              tdoa_sum[15:0];
                tdoa_used  <= (peak_val != {DATA_WIDTH{1'b0}});
                tdoa_valid <= 1'b1;
            end
        end
    end

endmodule
