// 分包3子模块3.6：UART 结果输出
`timescale 1ns/1ps
`include "project_global_param.vh"
module uart_tx_module (
    input  wire         clk_50m,
    input  wire         rst_n,
    input  wire         frame_valid,           // 每帧结果就绪脉冲
    input  wire [FRAME_CNT_WIDTH-1:0] frame_cnt,
    input  wire [TDOA_WIDTH-1:0] tdoa_vals [0:MAX_PAIR_NUM-1],  // Q10
    input  wire signed [15:0] pos_azimuth_01deg,    // 方位角 0.1°（-1800..+1800）
    input  wire signed [15:0] pos_elevation_01deg,  // 俯仰角 0.1°（0..+900）
    input  wire         pos_valid,
    output wire         uart_tx
);
    localparam integer BAUD_DIV = UART_BAUD_DIV; 
    localparam integer UART_PAIR = 6;          
    localparam integer FRM_LEN   = 91;        
  
    localparam [2:0] S_IDLE = 3'd0, S_RUN = 3'd1, S_DIG = 3'd2,
                     S_PUSH = 3'd3, S_TAIL = 3'd4;
    reg [2:0] gstate;
    reg [7:0] out_char;
    reg       char_ready;
    reg [7:0] cks;
    // 输入锁存
    reg [FRAME_CNT_WIDTH-1:0] frm;
    reg signed [TDOA_WIDTH-1:0] tdoa_l [0:UART_PAIR-1];
    reg signed [15:0] pa, pe;
    reg        pv;
    integer t;
    // 角度契约钳位：A ∈ ±1800、E ∈ 0..+900
    wire signed [15:0] az_cl = (pos_azimuth_01deg >  16'sd1800) ?  16'sd1800 :
                               (pos_azimuth_01deg < -16'sd1800) ? -16'sd1800 :
                                                                  pos_azimuth_01deg;
    wire signed [15:0] el_cl = (pos_elevation_01deg > 16'sd900)  ?  16'sd900  :
                               (pos_elevation_01deg < 16'sd0)    ?  16'sd0    :
                                                                   pos_elevation_01deg;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            frm <= 0; pa <= 0; pe <= 0; pv <= 0;
            for (t = 0; t < UART_PAIR; t = t + 1) tdoa_l[t] <= 0;
        end else if (frame_valid && gstate == S_IDLE) begin
            frm <= frame_cnt;
            pa  <= az_cl;
            pe  <= el_cl;
            pv  <= pos_valid;
            for (t = 0; t < UART_PAIR; t = t + 1)
                tdoa_l[t] <= $signed(tdoa_vals[t]);
        end
    end
    // 字节表与工具函数
    function automatic [7:0] hex_char(input [3:0] h);
        hex_char = (h < 4'd10) ? (8'h30 + {4'b0, h}) : (8'h41 + {4'b0, h} - 8'd10);
    endfunction
    function automatic [13:0] p10_next(input [13:0] p);
        case (p)
            14'd10000: p10_next = 14'd1000;
            14'd1000:  p10_next = 14'd100;
            14'd100:   p10_next = 14'd10;
            default:   p10_next = 14'd1;
        endcase
    endfunction
    function automatic [7:0] static_byte(input [6:0] i);
        case (i)
            7'd0:  static_byte = "$";  7'd1:  static_byte = "G";
            7'd2:  static_byte = "C";  7'd3:  static_byte = "C";
            7'd4:  static_byte = "P";  7'd5:  static_byte = ",";
            7'd10: static_byte = ",";
            7'd11, 7'd20, 7'd29, 7'd38, 7'd47, 7'd56: static_byte = "T";
            7'd12: static_byte = "0";  7'd13: static_byte = "1";
            7'd21: static_byte = "0";  7'd22: static_byte = "2";
            7'd30: static_byte = "0";  7'd31: static_byte = "3";
            7'd39: static_byte = "1";  7'd40: static_byte = "2";
            7'd48: static_byte = "1";  7'd49: static_byte = "3";
            7'd57: static_byte = "2";  7'd58: static_byte = "3";
            7'd14, 7'd23, 7'd32, 7'd41, 7'd50, 7'd59: static_byte = ":";
            7'd19, 7'd28, 7'd37, 7'd46, 7'd55, 7'd64: static_byte = ",";
            7'd65: static_byte = "A";  7'd66: static_byte = ":";
            7'd71: static_byte = ".";  7'd73: static_byte = ",";
            7'd74: static_byte = "E";  7'd75: static_byte = ":";
            7'd80: static_byte = ".";  7'd82: static_byte = ",";
            7'd83: static_byte = "V";  7'd84: static_byte = ":";
            7'd86: static_byte = "*";
            7'd89: static_byte = 8'h0D; 7'd90: static_byte = 8'h0A;
            default: static_byte = "0";
        endcase
    endfunction
    // 字节指针与数值
    reg [6:0]  gen_idx;                        
    reg [15:0] dv;                            
    reg [13:0] p10;                           
    reg [3:0]  dgt;                          
    reg [3:0]  dg_left;                       
    reg        dg_skip;                        
    reg        dg_ret_run;                    
 
    reg [2:0] tsel;
    always @(*) begin
        case (gen_idx)
            7'd15, 7'd16: tsel = 3'd0;
            7'd24, 7'd25: tsel = 3'd1;
            7'd33, 7'd34: tsel = 3'd2;
            7'd42, 7'd43: tsel = 3'd3;
            7'd51, 7'd52: tsel = 3'd4;
            default:      tsel = 3'd5;
        endcase
    end
    wire [16:0] tdoa_abs = tdoa_l[tsel][TDOA_WIDTH-1] ? ({1'b0, ~tdoa_l[tsel]} + 17'd1)
                                                     :  {1'b0,  tdoa_l[tsel]};
    wire [15:0] pa_abs   = pa[15] ? (~pa + 16'd1) : pa;
    wire [15:0] pe_abs   = pe[15] ? (~pe + 16'd1) : pe;
    // 字节类型判定
    localparam [3:0] K_STATIC = 4'd0,  K_FRM  = 4'd1,  K_TSIGN = 4'd2,
                     K_TDIG   = 4'd3,  K_ASIGN= 4'd4,  K_ADIG  = 4'd5,
                     K_AFRAC  = 4'd6,  K_ESIGN= 4'd7,  K_EDIG  = 4'd8,
                     K_EFRAC  = 4'd9,  K_VAL  = 4'd10, K_CKS   = 4'd11;
    reg [3:0] knd;
    always @(*) begin
        case (gen_idx)
            7'd6:  knd = K_FRM;
            7'd15, 7'd24, 7'd33, 7'd42, 7'd51, 7'd60: knd = K_TSIGN;
            7'd16, 7'd25, 7'd34, 7'd43, 7'd52, 7'd61: knd = K_TDIG;
            7'd67: knd = K_ASIGN;
            7'd68: knd = K_ADIG;
            7'd72: knd = K_AFRAC;
            7'd76: knd = K_ESIGN;
            7'd77: knd = K_EDIG;
            7'd81: knd = K_EFRAC;
            7'd85: knd = K_VAL;
            7'd87, 7'd88: knd = K_CKS;
            default: knd = K_STATIC;
        endcase
    end
    // 串行器寄存器
    reg [9:0] baud_cnt;
    reg [9:0] shift_reg;          // 空闲全 1
    reg [3:0] bit_cnt;
    reg       tx_busy;
    reg [7:0] imm_byte;
    always @(*) begin
        case (knd)
            K_TSIGN: imm_byte = tdoa_l[tsel][TDOA_WIDTH-1] ? "-" : "+";
            K_ASIGN: imm_byte = pa[15] ? "-" : "+";
            K_ESIGN: imm_byte = pe[15] ? "-" : "+";
            K_VAL:   imm_byte = pv ? "1" : "0";
            K_CKS:   imm_byte = (gen_idx == 7'd87) ? hex_char(cks[7:4])
                                                   : hex_char(cks[3:0]);
            default: imm_byte = static_byte(gen_idx);
        endcase
    end
    // 字符发生器 FSM
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            gstate <= S_IDLE; gen_idx <= 7'd0; cks <= 8'h00;
            dv <= 16'd0; p10 <= 14'd1; dgt <= 4'd0; dg_left <= 4'd0;
            dg_skip <= 1'b0; dg_ret_run <= 1'b1;
            out_char <= 8'h00; char_ready <= 1'b0;
        end else begin
            char_ready <= 1'b0;              
            case (gstate)
                S_IDLE: begin
                    gen_idx <= 7'd0;
                    cks     <= 8'h00;
                    if (frame_valid) gstate <= S_RUN;
                end
                S_RUN: begin
                    case (knd)
                        K_FRM: begin               // 帧号：拆 5 位、丢弃万位
                            dv      <= frm;
                            p10     <= 14'd10000;
                            dg_left <= 4'd5;
                            dg_skip <= 1'b1;
                            dgt     <= 4'd0;
                            gstate  <= S_DIG;
                        end
                        K_TDIG: begin              // τ：|τ|>>>10 拆 3 位
                            dv      <= tdoa_abs >> 10;
                            p10     <= 14'd100;
                            dg_left <= 4'd3;
                            dg_skip <= 1'b0;
                            dgt     <= 4'd0;
                            gstate  <= S_DIG;
                        end
                        K_ADIG: begin              // 方位角
                            dv      <= pa_abs;
                            p10     <= 14'd1000;
                            dg_left <= 4'd3;
                            dg_skip <= 1'b0;
                            dgt     <= 4'd0;
                            gstate  <= S_DIG;
                        end
                        K_EDIG: begin              // 俯仰角
                            dv      <= pe_abs;
                            p10     <= 14'd1000;
                            dg_left <= 4'd3;
                            dg_skip <= 1'b0;
                            dgt     <= 4'd0;
                            gstate  <= S_DIG;
                        end
                        K_AFRAC, K_EFRAC: begin    // 小数位 = 上一步拆位后的余数 dv
                            p10     <= 14'd1;
                            dg_left <= 4'd1;
                            dg_skip <= 1'b0;
                            dgt     <= 4'd0;
                            gstate  <= S_DIG;
                        end
                        default: begin             // 静态 / 符号 / V / 校验和
                            out_char   <= imm_byte;
                            dg_ret_run <= 1'b1;
                            gstate     <= S_PUSH;
                        end
                    endcase
                end
                S_DIG: begin
                    if (dv >= {2'b0, p10}) begin
                        dv  <= dv - {2'b0, p10};   // 逐位减法提取十进制位
                        dgt <= dgt + 1'b1;
                    end else if (dg_skip) begin    // 丢弃该位
                        dg_skip <= 1'b0;
                        dgt     <= 4'd0;
                        p10     <= p10_next(p10);
                        dg_left <= dg_left - 1'b1;
                    end else begin                 // 本位就绪 
                        out_char   <= 8'h30 + {4'b0, dgt};
                        p10        <= p10_next(p10);
                        dgt        <= 4'd0;
                        dg_left    <= dg_left - 1'b1;
                        dg_ret_run <= (dg_left == 4'd1);
                        gstate     <= S_PUSH;
                    end
                end
                S_PUSH: begin                      // 等串行器空闲后交付 1 字节
                    if (!tx_busy) begin
                        char_ready <= 1'b1;
                        if (gen_idx >= 7'd1 && gen_idx <= 7'd85)
                            cks <= cks ^ out_char; // 校验和 = 第 1..85 字节的 XOR
                        gen_idx <= gen_idx + 1'b1;
                        if (gen_idx == FRM_LEN-1)  gstate <= S_TAIL;
                        else if (dg_ret_run)       gstate <= S_RUN;
                        else                       gstate <= S_DIG;
                    end
                end
                S_TAIL: begin                    
                    if (!tx_busy) gstate <= S_IDLE;
                end
                default: gstate <= S_IDLE;
            endcase
        end
    end
    // 8N1 串行器
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            baud_cnt <= 0; shift_reg <= 10'h3FF;
            bit_cnt <= 0; tx_busy <= 1'b0;
        end else begin
            if (char_ready && !tx_busy) begin
                // 装载新字节
                shift_reg <= {1'b1, out_char, 1'b0};
                tx_busy   <= 1'b1;
                baud_cnt  <= 0;
                bit_cnt   <= 0;
            end else if (tx_busy) begin
                if (baud_cnt == BAUD_DIV-1) begin
                    baud_cnt  <= 0;
                    shift_reg <= {1'b1, shift_reg[9:1]};   
                    if (bit_cnt == 4'd9) begin
                        tx_busy <= 1'b0;                  
                    end else
                        bit_cnt <= bit_cnt + 1'b1;
                end else
                    baud_cnt <= baud_cnt + 1'b1;
            end
        end
    end
    assign uart_tx = shift_reg[0];
endmodule
