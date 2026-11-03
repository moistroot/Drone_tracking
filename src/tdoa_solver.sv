// 分包3子模块3.5：阵列 TDOA 测向解算（方位角+俯仰角）
`timescale 1ns/1ps
`include "project_global_param.vh"
module tdoa_solver (
    input  wire         clk_50m,
    input  wire         rst_n,
    input  wire [TDOA_WIDTH-1:0] tdoa_result [0:MAX_PAIR_NUM-1],  // Q10 采样点
    input  wire [MAX_PAIR_NUM-1:0] pair_used,  
    input  wire         all_done,       
    input  wire [COORD_WIDTH-1:0] mic_coord_x [0:MAX_CHAN_NUM-1], // 阵列坐标
    input  wire [COORD_WIDTH-1:0] mic_coord_y [0:MAX_CHAN_NUM-1],
    input  wire [FRAME_CNT_WIDTH-1:0] pre_frame_cnt,
    input  wire         mask_learn_done,   // 掩码学习完成
    output reg signed [15:0] target_azimuth_01deg,    // 方位角，0.1°（-1800..+1800）
    output reg signed [15:0] target_elevation_01deg,  // 俯仰角，0.1°（0..+900）
    output reg          direction_valid
);
    // 常量
    localparam signed [31:0] COS_K  = 32'sd46588;
    localparam signed [15:0] TAU_MAX = 16'sd11523;
  
    localparam [47:0] ONE_Q46 = 48'h400000000000;
    localparam [47:0] SUM_LIMIT_Q46 = 48'h500000000000;
    // 状态机
    localparam [3:0] S_IDLE = 0, S_COS = 1, S_CHK = 2, S_SUM = 3,
                     S_SQRT1 = 4, S_SQRT2 = 5, S_CORD = 6,
                     S_DEG = 7, S_OUT = 8;
    reg [3:0] state;
    reg [5:0] cnt;
    // 输入锁存
    reg signed [15:0] tau01, tau23;      
    reg cos_bad;                         
    reg deg_case;                        
    // 方向余弦（Q23 有符号，cos = 1 ? 2^23，需 24 位）
    reg signed [23:0] ca, cb;
    reg [47:0] ca2, cb2;                  // Q46（24×24 → 48 位，恒非负）
    reg [47:0] sum48;                     // Σcos?（最大 2×2^46 = 2^47，48 位够）
    wire [47:0] sum_comb = ca2 + cb2;
    wire sum_ovf = sum_comb[47] | (sum_comb[46] & (|sum_comb[45:0]));
   
    wire sum_far = (sum_comb > SUM_LIMIT_Q46);
  
    wire [47:0] w_next = ONE_Q46 - sum_comb;
    // 开方结果（Q23 无符号：√(Q46) → Q23）
    reg [23:0] cosg, rho;                 
    
    reg  [DATA_WIDTH-1:0] cord_re, cord_im;
    wire signed [25:0] cord_angle;
    wire               cord_ang_valid;
    wire [DATA_WIDTH-1:0] cord_re_out, cord_im_out;
    reg signed [25:0] phi_ang, th_ang;
    // 0.1° 换算
    function automatic signed [15:0] deg01_of(input signed [25:0] ang);
        reg [33:0] mag;
        begin
            mag = ang[25] ? ({1'b0, ~ang[25:0]} + 34'd1) : {8'b0, ang[25:0]};
            mag = (mag * 34'd225 + 34'd2097152) >>> 22;   
            deg01_of = ang[25] ? -$signed({18'b0, mag[13:0]})
                               :  $signed({18'b0, mag[13:0]});
        end
    endfunction
    // τ 5 帧中值
    function automatic signed [15:0] med5(
        input signed [15:0] a, b, c, d, e
    );
        reg signed [15:0] v0, v1, v2, v3, v4, t;
        begin
            v0 = a; v1 = b; v2 = c; v3 = d; v4 = e;
            if (v0 > v1) begin t = v0; v0 = v1; v1 = t; end
            if (v3 > v4) begin t = v3; v3 = v4; v4 = t; end
            if (v2 > v4) begin t = v2; v2 = v4; v4 = t; end
            if (v2 > v3) begin t = v2; v2 = v3; v3 = t; end
            if (v1 > v4) begin t = v1; v1 = v4; v4 = t; end
            if (v0 > v3) begin t = v0; v0 = v3; v3 = t; end
            if (v0 > v2) begin t = v0; v0 = v2; v2 = t; end
            if (v1 > v3) begin t = v1; v1 = v3; v3 = t; end
            if (v1 > v2) begin t = v1; v1 = v2; v2 = t; end
            med5 = v2;
        end
    endfunction
    // 128 帧六对联合直方图投票
    localparam integer VOTE_BINS  = 23;     // 档数（-11..+11 采样点）
    localparam integer RING_LEN   = 128;    // 滑窗帧数
    localparam integer SEARCH_PER = 4;      // 每 4 帧搜索一次
    localparam signed [16:0] BIN_CENTER_OFF = 17'sd11776;
    reg [7:0] h01 [0:VOTE_BINS-1];
    reg [7:0] h02 [0:VOTE_BINS-1];
    reg [7:0] h03 [0:VOTE_BINS-1];
    reg [7:0] h12 [0:VOTE_BINS-1];
    reg [7:0] h13 [0:VOTE_BINS-1];
    reg [7:0] h23 [0:VOTE_BINS-1];
    // 环形缓冲
    (* ram_style = "distributed" *) reg [4:0] r01 [0:RING_LEN-1];
    (* ram_style = "distributed" *) reg [4:0] r02 [0:RING_LEN-1];
    (* ram_style = "distributed" *) reg [4:0] r03 [0:RING_LEN-1];
    (* ram_style = "distributed" *) reg [4:0] r12 [0:RING_LEN-1];
    (* ram_style = "distributed" *) reg [4:0] r13 [0:RING_LEN-1];
    (* ram_style = "distributed" *) reg [4:0] r23 [0:RING_LEN-1];
    (* ram_style = "distributed" *) reg [5:0] slot_used [0:RING_LEN-1];
    function automatic [4:0] bin_of(input signed [15:0] tau);
        reg signed [16:0] t;
        reg [4:0] b;
        begin
            t = tau + BIN_CENTER_OFF;
            if (t < 0) begin
                bin_of = 5'd0;
            end else begin
                b = t[14:10];
                bin_of = (b > 5'd22) ? 5'd22 : b;
            end
        end
    endfunction
    // 3 点中值（输出级去孤峰）
    function automatic signed [15:0] med3(
        input signed [15:0] a, input signed [15:0] b, input signed [15:0] c
    );
        reg signed [15:0] lo, hi;
        begin
            lo = (a < b) ? a : b;
            hi = (a < b) ? b : a;
            if (c < lo)      med3 = lo;
            else if (c > hi) med3 = hi;
            else             med3 = c;
        end
    endfunction
    function automatic [11:0] joint_score2(input [4:0] a, input [4:0] b);
        reg signed [5:0] d, s;
        reg [4:0] i2, i3;
        reg [11:0] sc;
        begin
            d  = $signed({1'b0, a}) - $signed({1'b0, b});
            s  = $signed({1'b0, a}) + $signed({1'b0, b});
            i2 = 5'd11 + d[5:1];
            i3 = s[5:1];
            sc = {3'b0, h01[a]} + {3'b0, h01[a]}
               + {3'b0, h23[b]} + {3'b0, h23[b]};
            if (d[0] == 1'b0) begin
                sc = sc + {3'b0, h02[i2]} + {3'b0, h02[i2]}
                        + {3'b0, h13[5'd22 - i2]} + {3'b0, h13[5'd22 - i2]};
            end else begin
                sc = sc + {4'b0, h02[i2]} + {4'b0, h02[i2 + 5'd1]}
                        + {4'b0, h13[5'd22 - i2]} + {4'b0, h13[5'd21 - i2]};
            end
            if (s[0] == 1'b0) begin
                sc = sc + {3'b0, h03[i3]} + {3'b0, h03[i3]}
                        + {3'b0, h12[5'd22 - i3]} + {3'b0, h12[5'd22 - i3]};
            end else begin
                sc = sc + {4'b0, h03[i3]} + {4'b0, h03[i3 + 5'd1]}
                        + {4'b0, h12[5'd22 - i3]} + {4'b0, h12[5'd21 - i3]};
            end
            joint_score2 = sc;
        end
    endfunction
    localparam [3:0] V_IDLE = 0, V_DEC = 1, V_INC = 2, V_POST = 3,
                     V_SEARCH = 4, V_FIN = 5, V_NEIGH = 6, V_DIV = 7,
                     V_LATCH = 8;
    reg [3:0]  vstate;
    reg [4:0]  new_bin [0:5];
    reg [4:0]  old_bin [0:5];
    reg [5:0]  new_used;                   // 本帧有效对位图
    reg [5:0]  old_used;                   // 被移出槽的有效对位图
    reg [6:0]  wr_ptr;
    reg [7:0]  filled;
    reg [3:0]  search_cnt;
    reg [4:0]  sa, sb;                     // 搜索游标
    reg [4:0]  best_a, best_b;
    reg [11:0] best_score;
    reg [11:0] s_a0, s_am, s_ap, s_bm, s_bp;
    reg [4:0]  a_m, a_p, b_m, b_p;
    reg [1:0]  neigh_ph;
    reg [4:0]  na, nb;                     // 邻档求值坐标
    reg        div_started;
    reg [4:0]  div_wait;
    reg [8:0]  q_a_abs, q_b_abs;
    reg        q_a_pos, q_b_pos;
    reg        vote_ready;
    reg        conf_ok;
    reg signed [15:0] tau01_vote, tau23_vote;
    reg signed [15:0] med01_h1, med01_h2, med23_h1, med23_h2;
    reg        vote_ok_lat;
    wire [4:0] nb01 = bin_of($signed(tdoa_result[0]));
    wire [4:0] nb02 = bin_of($signed(tdoa_result[1]));
    wire [4:0] nb03 = bin_of($signed(tdoa_result[2]));
    wire [4:0] nb12 = bin_of($signed(tdoa_result[3]));
    wire [4:0] nb13 = bin_of($signed(tdoa_result[4]));
    wire [4:0] nb23 = bin_of($signed(tdoa_result[5]));
    wire [11:0] sc_comb = joint_score2(sa, sb);
    wire [11:0] sc_n    = joint_score2(na, nb);
    always @(*) begin
        case (neigh_ph)
            2'd0:    begin na = a_m;    nb = best_b; end
            2'd1:    begin na = a_p;    nb = best_b; end
            2'd2:    begin na = best_a; nb = b_m;    end
            default: begin na = best_a; nb = b_p;    end
        endcase
    end
   
    wire [12:0] sum_am_ap = {1'b0, s_am} + {1'b0, s_ap};
    wire [12:0] sum_bm_bp = {1'b0, s_bm} + {1'b0, s_bp};
    wire [12:0] two_s0     = {1'b0, s_a0} << 1;
    wire        den_a_ok   = (two_s0 > sum_am_ap);
    wire        den_b_ok   = (two_s0 > sum_bm_bp);
    wire [12:0] den_a      = den_a_ok ? (two_s0 - sum_am_ap) : 13'd1;
    wire [12:0] den_b      = den_b_ok ? (two_s0 - sum_bm_bp) : 13'd1;
    wire [11:0] diff_a     = (s_ap >= s_am) ? (s_ap - s_am) : (s_am - s_ap);
    wire [11:0] diff_b     = (s_bp >= s_bm) ? (s_bp - s_bm) : (s_bm - s_bp);
   
    wire [19:0] num_a      = den_a_ok ? ({8'b0, diff_a} << 7) : 20'd0;
    wire [19:0] num_b      = den_b_ok ? ({8'b0, diff_b} << 7) : 20'd0;
    wire        vote_div_start = (vstate == V_DIV) && !div_started;
    wire [8:0]  div_q_a, div_q_b;
    wire        div_done_a, div_done_b;
    div_serial #(.DIV_CYC(9), .NUM_W(20), .DEN_W(13)) u_div_a (
        .clk (clk_50m), .rst_n (rst_n), .start (vote_div_start),
        .num (num_a), .den (den_a), .quot (div_q_a), .done (div_done_a)
    );
    div_serial #(.DIV_CYC(9), .NUM_W(20), .DEN_W(13)) u_div_b (
        .clk (clk_50m), .rst_n (rst_n), .start (vote_div_start),
        .num (num_b), .den (den_b), .quot (div_q_b), .done (div_done_b)
    );
    // 候选档 + 1/8 采样点偏移 Q10
    wire signed [8:0] qa_s = q_a_pos ? $signed(q_a_abs) : -$signed(q_a_abs);
    wire signed [8:0] qb_s = q_b_pos ? $signed(q_b_abs) : -$signed(q_b_abs);
    wire signed [16:0] t01_full = (($signed({1'b0, best_a}) - 17'sd11) <<< 10)
                                + ($signed({{2{qa_s[8]}}, qa_s}) <<< 2);
    wire signed [16:0] t23_full = (($signed({1'b0, best_b}) - 17'sd11) <<< 10)
                                + ($signed({{2{qb_s[8]}}, qb_s}) <<< 2);
    wire signed [15:0] tau01_new = (t01_full >  17'sd11523) ?  16'sd11523 :
                                   (t01_full < -17'sd11523) ? -16'sd11523 :
                                   t01_full[15:0];
    wire signed [15:0] tau23_new = (t23_full >  17'sd11523) ?  16'sd11523 :
                                   (t23_full < -17'sd11523) ? -16'sd11523 :
                                   t23_full[15:0];
    integer vi;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            vstate <= V_IDLE;
            wr_ptr <= 7'd0; filled <= 8'd0; search_cnt <= 4'd0;
            sa <= 5'd0; sb <= 5'd0;
            best_score <= 12'd0; best_a <= 5'd11; best_b <= 5'd11;
            s_a0 <= 0; s_am <= 0; s_ap <= 0; s_bm <= 0; s_bp <= 0;
            a_m <= 0; a_p <= 0; b_m <= 0; b_p <= 0;
            neigh_ph <= 2'd0;             
            div_started <= 1'b0; div_wait <= 5'd0;
            q_a_abs <= 9'd0; q_b_abs <= 9'd0;
            q_a_pos <= 1'b0; q_b_pos <= 1'b0;
            vote_ready <= 1'b0; conf_ok <= 1'b0;
            new_used <= 6'd0; old_used <= 6'd0;
            tau01_vote <= 0; tau23_vote <= 0;
            med01_h1 <= 0; med01_h2 <= 0; med23_h1 <= 0; med23_h2 <= 0;
            for (vi = 0; vi < VOTE_BINS; vi = vi + 1) begin
                h01[vi] <= 0; h02[vi] <= 0; h03[vi] <= 0;
                h12[vi] <= 0; h13[vi] <= 0; h23[vi] <= 0;
            end
        end else begin
            case (vstate)
               
                V_IDLE: begin
                    if (all_done && mask_learn_done) begin
                        if (|pair_used[5:0]) begin
                            new_bin[0] <= nb01; new_bin[1] <= nb02;
                            new_bin[2] <= nb03; new_bin[3] <= nb12;
                            new_bin[4] <= nb13; new_bin[5] <= nb23;
                            old_bin[0] <= r01[wr_ptr]; old_bin[1] <= r02[wr_ptr];
                            old_bin[2] <= r03[wr_ptr]; old_bin[3] <= r12[wr_ptr];
                            old_bin[4] <= r13[wr_ptr]; old_bin[5] <= r23[wr_ptr];
                            new_used <= pair_used[5:0];
                            old_used <= slot_used[wr_ptr];
                            vstate <= V_DEC;
                        end
                    end
                end
                // 窗已满
                V_DEC: begin
                    if (filled == 8'd128) begin
                        if (old_used[0] && (!new_used[0] || old_bin[0] != new_bin[0]) &&
                            h01[old_bin[0]] != 0)
                            h01[old_bin[0]] <= h01[old_bin[0]] - 8'd1;
                        if (old_used[1] && (!new_used[1] || old_bin[1] != new_bin[1]) &&
                            h02[old_bin[1]] != 0)
                            h02[old_bin[1]] <= h02[old_bin[1]] - 8'd1;
                        if (old_used[2] && (!new_used[2] || old_bin[2] != new_bin[2]) &&
                            h03[old_bin[2]] != 0)
                            h03[old_bin[2]] <= h03[old_bin[2]] - 8'd1;
                        if (old_used[3] && (!new_used[3] || old_bin[3] != new_bin[3]) &&
                            h12[old_bin[3]] != 0)
                            h12[old_bin[3]] <= h12[old_bin[3]] - 8'd1;
                        if (old_used[4] && (!new_used[4] || old_bin[4] != new_bin[4]) &&
                            h13[old_bin[4]] != 0)
                            h13[old_bin[4]] <= h13[old_bin[4]] - 8'd1;
                        if (old_used[5] && (!new_used[5] || old_bin[5] != new_bin[5]) &&
                            h23[old_bin[5]] != 0)
                            h23[old_bin[5]] <= h23[old_bin[5]] - 8'd1;
                    end
                    vstate <= V_INC;
                end
                // 加入新帧
                V_INC: begin
                    if (new_used[0]) begin
                        if ((filled < 8'd128 || !old_used[0] || old_bin[0] != new_bin[0]) &&
                            h01[new_bin[0]] != 8'hFF)
                            h01[new_bin[0]] <= h01[new_bin[0]] + 8'd1;
                        r01[wr_ptr] <= new_bin[0];
                    end
                    if (new_used[1]) begin
                        if ((filled < 8'd128 || !old_used[1] || old_bin[1] != new_bin[1]) &&
                            h02[new_bin[1]] != 8'hFF)
                            h02[new_bin[1]] <= h02[new_bin[1]] + 8'd1;
                        r02[wr_ptr] <= new_bin[1];
                    end
                    if (new_used[2]) begin
                        if ((filled < 8'd128 || !old_used[2] || old_bin[2] != new_bin[2]) &&
                            h03[new_bin[2]] != 8'hFF)
                            h03[new_bin[2]] <= h03[new_bin[2]] + 8'd1;
                        r03[wr_ptr] <= new_bin[2];
                    end
                    if (new_used[3]) begin
                        if ((filled < 8'd128 || !old_used[3] || old_bin[3] != new_bin[3]) &&
                            h12[new_bin[3]] != 8'hFF)
                            h12[new_bin[3]] <= h12[new_bin[3]] + 8'd1;
                        r12[wr_ptr] <= new_bin[3];
                    end
                    if (new_used[4]) begin
                        if ((filled < 8'd128 || !old_used[4] || old_bin[4] != new_bin[4]) &&
                            h13[new_bin[4]] != 8'hFF)
                            h13[new_bin[4]] <= h13[new_bin[4]] + 8'd1;
                        r13[wr_ptr] <= new_bin[4];
                    end
                    if (new_used[5]) begin
                        if ((filled < 8'd128 || !old_used[5] || old_bin[5] != new_bin[5]) &&
                            h23[new_bin[5]] != 8'hFF)
                            h23[new_bin[5]] <= h23[new_bin[5]] + 8'd1;
                        r23[wr_ptr] <= new_bin[5];
                    end
                    slot_used[wr_ptr] <= new_used;
                    wr_ptr <= (wr_ptr == 7'd127) ? 7'd0 : wr_ptr + 7'd1;
                    if (filled < 8'd128) filled <= filled + 8'd1;
                    vstate <= V_POST;
                end
                // 每 SEARCH_PER 帧启动一次联合搜索
                V_POST: begin
                    if (search_cnt == SEARCH_PER - 1) begin
                        search_cnt <= 4'd0;
                        sa <= 5'd0; sb <= 5'd0;
                        best_score <= 12'd0;
                        best_a <= 5'd11; best_b <= 5'd11;
                        vstate <= V_SEARCH;
                    end else begin
                        search_cnt <= search_cnt + 4'd1;
                        vstate <= V_IDLE;
                    end
                end
                // 全枚举
                V_SEARCH: begin
                    if (sc_comb > best_score) begin
                        best_score <= sc_comb;
                        best_a <= sa; best_b <= sb;
                    end
                    if (sb == 5'd22) begin
                        sb <= 5'd0;
                        if (sa == 5'd22) vstate <= V_FIN;
                        else             sa <= sa + 5'd1;
                    end else begin
                        sb <= sb + 5'd1;
                    end
                end
                // 取峰值左右邻档
                V_FIN: begin
                    s_a0 <= best_score;
                    a_m <= (best_a == 5'd0)  ? 5'd0  : best_a - 5'd1;
                    a_p <= (best_a == 5'd22) ? 5'd22 : best_a + 5'd1;
                    b_m <= (best_b == 5'd0)  ? 5'd0  : best_b - 5'd1;
                    b_p <= (best_b == 5'd22) ? 5'd22 : best_b + 5'd1;
                    neigh_ph <= 2'd0;
                    vstate <= V_NEIGH;
                end
                // 4 拍求 4 个邻档的联合分
                V_NEIGH: begin
                    case (neigh_ph)
                        2'd0: s_am <= sc_n;
                        2'd1: s_ap <= sc_n;
                        2'd2: s_bm <= sc_n;
                        default: s_bp <= sc_n;
                    endcase
                    if (neigh_ph == 2'd3) begin
                        neigh_ph <= 2'd0;
                        div_started <= 1'b0;
                        div_wait <= 5'd0;
                        vstate <= V_DIV;
                    end else begin
                        neigh_ph <= neigh_ph + 2'd1;
                    end
                end
                // 两路串行除法（各 20 拍，超时 23 拍兜底）
                V_DIV: begin
                    if (!div_started) begin
                        div_started <= 1'b1;
                    end else if (div_done_a && div_done_b) begin
                        q_a_abs <= (div_q_a > 9'd128) ? 9'd128 : div_q_a;
                        q_b_abs <= (div_q_b > 9'd128) ? 9'd128 : div_q_b;
                        q_a_pos <= (s_ap >= s_am);
                        q_b_pos <= (s_bp >= s_bm);
                        vstate  <= V_LATCH;
                    end else if (div_wait == 5'd23) begin
                        q_a_abs <= 9'd0;
                        q_b_abs <= 9'd0;
                        vstate  <= V_LATCH;
                    end else begin
                        div_wait <= div_wait + 5'd1;
                    end
                end
                // 输出：首窗填满后置 ready；此后 3 点中值；置信度
                V_LATCH: begin
                    if (filled == 8'd128 && !vote_ready) begin
                        vote_ready <= 1'b1;
                        tau01_vote <= tau01_new;
                        tau23_vote <= tau23_new;
                        med01_h1 <= tau01_new; med01_h2 <= tau01_new;
                        med23_h1 <= tau23_new; med23_h2 <= tau23_new;
                    end else if (vote_ready) begin
                        tau01_vote <= med3(tau01_new, med01_h1, med01_h2);
                        tau23_vote <= med3(tau23_new, med23_h1, med23_h2);
                        med01_h2 <= med01_h1; med01_h1 <= tau01_new;
                        med23_h2 <= med23_h1; med23_h1 <= tau23_new;
                    end else begin
                        med01_h1 <= tau01_new; med01_h2 <= tau01_new;
                        med23_h1 <= tau23_new; med23_h2 <= tau23_new;
                    end
                    conf_ok <= (best_score > {4'b0, filled});
                    vstate  <= V_IDLE;
                end
                default: vstate <= V_IDLE;
            endcase
        end
    end
    reg signed [15:0] t01_h0, t01_h1, t01_h2, t01_h3;
    reg signed [15:0] t23_h0, t23_h1, t23_h2, t23_h3;
    reg [2:0] filt_cnt;
    wire signed [15:0] raw01 = $signed(tdoa_result[0]);
    wire signed [15:0] raw23 = $signed(tdoa_result[5]);
    wire signed [15:0] med01 = med5(raw01, t01_h0, t01_h1, t01_h2, t01_h3);
    wire signed [15:0] med23 = med5(raw23, t23_h0, t23_h1, t23_h2, t23_h3);
    wire signed [15:0] fb01 = (filt_cnt < 3'd4) ? raw01 : med01;
    wire signed [15:0] fb23 = (filt_cnt < 3'd4) ? raw23 : med23;
    wire signed [15:0] t01_use = vote_ready ? tau01_vote : fb01;
    wire signed [15:0] t23_use = vote_ready ? tau23_vote : fb23;
    // 开方器（复用 2 次：cosγ 与 √(cos?α+cos?β)）
    reg  sqrt_start;
    wire [23:0] sqrt_root;
    wire        sqrt_done;
    reg  [47:0] sqrt_rad;
    sqrt_serial #(.IN_W(48), .OUT_W(24), .SQRT_CYC(24)) u_sqrt (
        .clk   (clk_50m),
        .rst_n (rst_n),
        .start (sqrt_start),
        .rad   (sqrt_rad),
        .root  (sqrt_root),
        .done  (sqrt_done)
    );
    // CORDIC（复用：先 φ 后 θ，间隔 1 拍连发）
    cordic_phat_norm_24b u_cord (
        .clk_50m     (clk_50m),
        .rst_n       (rst_n),
        .re_in       (cord_re),
        .im_in       (cord_im),
        .re_out      (cord_re_out),      // 测向不用单位向量，悬空
        .im_out      (cord_im_out),
        .angle_out   (cord_angle),
        .angle_valid (cord_ang_valid)
    );
    // 主状态机
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE; cnt <= 0;
            tau01 <= 0; tau23 <= 0; cos_bad <= 0; deg_case <= 0;
            t01_h0 <= 0; t01_h1 <= 0; t01_h2 <= 0; t01_h3 <= 0;
            t23_h0 <= 0; t23_h1 <= 0; t23_h2 <= 0; t23_h3 <= 0;
            filt_cnt <= 3'd0;
            ca <= 0; cb <= 0; ca2 <= 0; cb2 <= 0; sum48 <= 0;
            cosg <= 0; rho <= 0;
            cord_re <= 0; cord_im <= 0;
            phi_ang <= 0; th_ang <= 0;
            sqrt_start <= 1'b0; sqrt_rad <= 0;
            target_azimuth_01deg   <= 0;
            target_elevation_01deg <= 0;
            direction_valid        <= 1'b0;
            vote_ok_lat            <= 1'b0;
        end else begin
            sqrt_start <= 1'b0;
            case (state)
                S_IDLE: begin
                    direction_valid <= 1'b0;
                    if (all_done && mask_learn_done) begin
                        // 投票输出（窗未满时回退 5 帧中值）；UART 的 T 字段仍是原始值
                        tau01    <= t01_use;   
                        tau23    <= t23_use;  
                       
                        vote_ok_lat <= vote_ready && conf_ok;
                        cos_bad  <= (t01_use >  TAU_MAX) ||
                                    (t01_use < -TAU_MAX) ||
                                    (t23_use >  TAU_MAX) ||
                                    (t23_use < -TAU_MAX);
                        deg_case <= (t01_use == 0) && (t23_use == 0);
                        // 中值历史移位
                        t01_h3 <= t01_h2; t01_h2 <= t01_h1;
                        t01_h1 <= t01_h0; t01_h0 <= raw01;
                        t23_h3 <= t23_h2; t23_h2 <= t23_h1;
                        t23_h1 <= t23_h0; t23_h0 <= raw23;
                        if (filt_cnt < 3'd4)
                            filt_cnt <= filt_cnt + 3'd1;
                        state    <= S_COS;
                    end
                end
               
                S_COS: begin
                    ca <= (tau01 * COS_K) >>> 6;   // 除以 64
                    cb <= (tau23 * COS_K) >>> 6;
                    state <= S_CHK;
                end
                // 平方（Q46）
                S_CHK: begin
                    ca2 <= ca * ca;
                    cb2 <= cb * cb;
                    state <= S_SUM;
                end
              
                S_SUM: begin
                    sum48 <= sum_comb;
                    if (cos_bad || sum_far) begin
                      
                        state <= S_IDLE;
                    end else if (sum_ovf) begin
                        cosg  <= 24'd0;
                        rho   <= 24'h800000;            // 1.0（Q23）
                        state <= S_CORD;
                        cnt   <= 6'd0;
                    end else if (deg_case) begin
                        state <= S_DEG;                  // 声源正上方
                    end else begin
                        sqrt_rad   <= w_next;            
                        sqrt_start <= 1'b1;
                        state      <= S_SQRT1;
                    end
                end
                // 第 1 次开方：cosγ
                S_SQRT1: begin
                    if (sqrt_done) begin
                        cosg     <= sqrt_root;
                        sqrt_rad <= sum48;              
                        sqrt_start <= 1'b1;
                        state    <= S_SQRT2;
                    end
                end
                // 第 2 次开方
                S_SQRT2: begin
                    if (sqrt_done) begin
                        rho <= sqrt_root;
                        state <= S_CORD;
                        cnt   <= 6'd0;
                    end
                end
                // CORDIC 连发两次：cnt=0 送 φ 输入、cnt=1 送 θ 输入
                S_CORD: begin
                    case (cnt)
                        6'd0: begin
                            cord_re <= {ca[23], ca[23:1]};   
                            cord_im <= {cb[23], cb[23:1]};
                        end
                        6'd1: begin
                            cord_re <= {1'b0, rho[23:1]};    
                            cord_im <= {1'b0, cosg[23:1]};   
                        end
                        6'd23: phi_ang <= cord_angle;
                        6'd24: th_ang  <= cord_angle;
                        6'd25: state   <= S_OUT;
                        default: ;
                    endcase
                    cnt <= cnt + 6'd1;
                end
                // 退化：声源正上方
                S_DEG: begin
                    target_azimuth_01deg   <= 16'sd0;
                    target_elevation_01deg <= 16'sd900;    // +90.0°
                    direction_valid        <= vote_ok_lat;
                    state                  <= S_IDLE;
                end
                // 输出（-180.0° 按 (-180°,+180°] 约定折到 +180.0°）
                S_OUT: begin
                    target_azimuth_01deg <=
                        (phi_ang == 26'h2000000) ? 16'sd1800    
                                                 : deg01_of(phi_ang);
                    target_elevation_01deg <= deg01_of(th_ang);
                    direction_valid        <= vote_ok_lat;
                    state                  <= S_IDLE;
                end
                default: state <= S_IDLE;
            endcase
        end
    end
endmodule
