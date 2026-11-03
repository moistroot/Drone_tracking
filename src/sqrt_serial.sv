// sqrt_serial.sv - 串行开方器（tdoa_solver 辅助模块）
`timescale 1ns/1ps
module sqrt_serial #(
    parameter IN_W = 24,
    parameter OUT_W = 12,
    parameter SQRT_CYC = 12     
) (
    input  wire         clk,
    input  wire         rst_n,
    input  wire         start,
    input  wire [IN_W-1:0] rad,
    output reg  [OUT_W-1:0] root,
    output reg          done
);
    reg [IN_W-1:0]  rem_val;        
    reg [OUT_W-1:0] root_acc;     
    reg [4:0]       cnt;
    reg             running;
   
    wire [1:0] rad_hi2 = rad[(IN_W-1) - cnt*2 -: 2];
    wire [IN_W+1:0] rem_shift = {rem_val, rad_hi2};
    wire [IN_W+1:0] sub_val  = rem_shift - {{(IN_W+2-OUT_W-2){1'b0}}, root_acc, 2'b01};
    wire [OUT_W-1:0] root_acc_next = {root_acc[OUT_W-2:0], ~sub_val[IN_W+1]};
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rem_val <= 0; root_acc <= 0; cnt <= 0;
            running <= 1'b0; root <= 0; done <= 1'b0;
        end else begin
            done <= 1'b0;
            if (start) begin
                rem_val  <= 0;
                root_acc <= 0;
                cnt      <= 0;
                running  <= 1'b1;
                root     <= 0;
            end else if (running) begin
                if (~sub_val[IN_W+1]) begin          
                    rem_val  <= sub_val[IN_W-1:0];
                    root_acc <= {root_acc[OUT_W-2:0], 1'b1};
                end else begin
                    rem_val  <= rem_shift[IN_W-1:0];
                    root_acc <= {root_acc[OUT_W-2:0], 1'b0};
                end
                if (cnt == SQRT_CYC-1) begin
                    running <= 1'b0;
                    done    <= 1'b1;
                    root    <= root_acc_next;
                end else
                    cnt <= cnt + 1'b1;
            end
        end
    end
endmodule
