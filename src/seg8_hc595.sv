// 8 位七段数码管显示驱动
`timescale 1ns/1ps
`include "project_global_param.vh"
module seg8_hc595 #(
    parameter integer SCAN_DIV = 50_000,   
    parameter [0:0]   DP_BIT   = 1'b0      
                                          
)(
    input  wire               clk_50m,
    input  wire               rst_n,
    input  wire signed [15:0] az_01deg,    
    input  wire        [15:0] el_01deg,    
    input  wire               sample_valid,  
    output reg                sh_cp,      
    output reg                st_cp,     
    output reg                ds           
);
    reg [15:0] scan_cnt;
    reg [2:0]  dig_idx;                
    wire       scan_tick = (scan_cnt == SCAN_DIV - 1);
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            scan_cnt <= 16'd0;
            dig_idx  <= 3'd0;
        end else if (scan_tick) begin
            scan_cnt <= 16'd0;
            dig_idx  <= dig_idx + 1'b1;
        end else begin
            scan_cnt <= scan_cnt + 1'b1;
        end
    end
    wire signed [15:0] az_lim = (az_01deg >  16'sd1800) ?  16'sd1800 :
                                (az_01deg < -16'sd1800) ? -16'sd1800 : az_01deg;
    wire [11:0] az_lim12 = az_lim[11:0];                       // |az_lim| ≤ 1800 < 2^11
    wire [11:0] az_mag   = az_lim[15] ? (12'd0 - az_lim12) : az_lim12;
    wire [7:0]  az_deg   = (az_mag + 12'd5) / 12'd10;          // 0..180
    wire [15:0] el_lim = el_01deg[15] ? 16'd0 :
                         (el_01deg > 16'd900) ? 16'd900 : el_01deg;
    wire [11:0] el_mag = el_lim[11:0];
    wire [7:0]  el_deg = (el_mag + 12'd5) / 12'd10;            // 0..90
    reg [7:0] az_deg_r;                 // 方位角
    reg [7:0] el_deg_r;                 // 俯仰角
    reg       az_neg_r;                 // 方位角为负
    reg       have_data;                // 有数据
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            az_deg_r  <= 8'd0;
            el_deg_r  <= 8'd0;
            az_neg_r  <= 1'b0;
            have_data <= 1'b0;
        end else if (sample_valid) begin
            az_deg_r  <= az_deg;
            el_deg_r  <= el_deg;
          
            az_neg_r  <= az_lim[15] && (az_deg != 8'd0);
            have_data <= 1'b1;
        end
    end
    wire [3:0] az_hun = az_deg_r / 8'd100;                    
    wire [7:0] az_rem = az_deg_r - az_hun * 8'd100;          
    wire [3:0] az_ten = az_rem / 4'd10;
    wire [3:0] az_one = az_rem - az_ten * 4'd10;
    wire [3:0] el_ten = el_deg_r / 4'd10;                     
    wire [3:0] el_one = el_deg_r - el_ten * 4'd10;
    localparam [3:0] NIB_BLANK = 4'hF;  
    localparam [3:0] NIB_DASH  = 4'hE;  
    reg [3:0] nib;
    always @(*) begin
        if (!have_data) begin
            nib = NIB_DASH;                 // 没有效数据
        end else begin
            case (dig_idx)
                3'd0:    nib = el_one;                         
                3'd1:    nib = el_ten;                         
                3'd2:    nib = 4'h0;                          
                3'd3:    nib = NIB_BLANK;                      
                3'd4:    nib = az_one;                       
                3'd5:    nib = az_ten;                         
                3'd6:    nib = az_hun;                          
                3'd7:    nib = az_neg_r ? NIB_DASH : NIB_BLANK; 
            endcase
        end
    end
    reg [6:0] seg;
    always @(*) begin
        case (nib)
            4'h0:    seg = 7'b1000000;
            4'h1:    seg = 7'b1111001;
            4'h2:    seg = 7'b0100100;
            4'h3:    seg = 7'b0110000;
            4'h4:    seg = 7'b0011001;
            4'h5:    seg = 7'b0010010;
            4'h6:    seg = 7'b0000010;
            4'h7:    seg = 7'b1111000;
            4'h8:    seg = 7'b0000000;
            4'h9:    seg = 7'b0010000;
            NIB_DASH: seg = 7'b0111111;    
            default:  seg = 7'b1111111;    
        endcase
    end
   
    reg [7:0] sel;
    always @(*) begin
        sel = 8'b0000_0001 << dig_idx;
    end
    reg       div_cnt;
    wire      sck_plus = (div_cnt == 1'b1);
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) div_cnt <= 1'b0;
        else        div_cnt <= div_cnt + 1'b1;
    end
    reg [5:0] edge_cnt;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) edge_cnt <= 6'd0;
        else if (sck_plus)
            edge_cnt <= (edge_cnt == 6'd32) ? 6'd0 : edge_cnt + 1'b1;
    end
   
    reg [15:0] frame;
    always @(posedge clk_50m or negedge rst_n) begin
      
        if (!rst_n) frame <= {DP_BIT, 7'b1111111, 8'b0000_0000};
        else if (sck_plus && (edge_cnt == 6'd32))
            frame <= {DP_BIT, seg, sel};
    end
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            sh_cp <= 1'b0;
            st_cp <= 1'b0;
            ds    <= 1'b0;
        end else if (sck_plus) begin
            case (edge_cnt)
                6'd0:  begin sh_cp <= 1'b0; st_cp <= 1'b0; ds <= frame[15]; end
                6'd1:  begin sh_cp <= 1'b1; st_cp <= 1'b0; end
                6'd2:  begin sh_cp <= 1'b0; ds <= frame[14]; end
                6'd3:  begin sh_cp <= 1'b1; end
                6'd4:  begin sh_cp <= 1'b0; ds <= frame[13]; end
                6'd5:  begin sh_cp <= 1'b1; end
                6'd6:  begin sh_cp <= 1'b0; ds <= frame[12]; end
                6'd7:  begin sh_cp <= 1'b1; end
                6'd8:  begin sh_cp <= 1'b0; ds <= frame[11]; end
                6'd9:  begin sh_cp <= 1'b1; end
                6'd10: begin sh_cp <= 1'b0; ds <= frame[10]; end
                6'd11: begin sh_cp <= 1'b1; end
                6'd12: begin sh_cp <= 1'b0; ds <= frame[9];  end
                6'd13: begin sh_cp <= 1'b1; end
                6'd14: begin sh_cp <= 1'b0; ds <= frame[8];  end
                6'd15: begin sh_cp <= 1'b1; end
                6'd16: begin sh_cp <= 1'b0; ds <= frame[7];  end
                6'd17: begin sh_cp <= 1'b1; end
                6'd18: begin sh_cp <= 1'b0; ds <= frame[6];  end
                6'd19: begin sh_cp <= 1'b1; end
                6'd20: begin sh_cp <= 1'b0; ds <= frame[5];  end
                6'd21: begin sh_cp <= 1'b1; end
                6'd22: begin sh_cp <= 1'b0; ds <= frame[4];  end
                6'd23: begin sh_cp <= 1'b1; end
                6'd24: begin sh_cp <= 1'b0; ds <= frame[3];  end
                6'd25: begin sh_cp <= 1'b1; end
                6'd26: begin sh_cp <= 1'b0; ds <= frame[2];  end
                6'd27: begin sh_cp <= 1'b1; end
                6'd28: begin sh_cp <= 1'b0; ds <= frame[1];  end
                6'd29: begin sh_cp <= 1'b1; end
                6'd30: begin sh_cp <= 1'b0; ds <= frame[0];  end
                6'd31: begin sh_cp <= 1'b1; end
                6'd32: begin st_cp <= 1'b1; end       
                default: begin sh_cp <= 1'b0; st_cp <= 1'b0; ds <= 1'b0; end
            endcase
        end
    end
endmodule
