// ´®ÐÐ»Ö¸´³ý·¨Æ÷£¨tdoa_solver ¸¨ÖúÄ£¿é£©
`timescale 1ns/1ps
module div_serial #(
    parameter DIV_CYC = 24,         
    parameter NUM_W   = 32,         
    parameter DEN_W   = 23         
) (
    input  wire         clk,
    input  wire         rst_n,
    input  wire         start,
    input  wire [NUM_W-1:0] num,   
    input  wire [DEN_W-1:0] den,   
    output reg  [DIV_CYC-1:0] quot,
    output reg          done
);
    localparam integer RW = DEN_W + 1;     
    reg [RW-1:0]      rem;                
    reg [NUM_W-1:0]   nsh;                  
    reg [DIV_CYC-1:0] q;                   
    reg [5:0]         cnt;
    reg               running;
    reg               ovf;                   
    // ÊÔ¼õ
    wire [RW:0] rsh = {rem, 1'b0} | {{RW{1'b0}}, nsh[NUM_W-1]};
    wire [RW:0] dif = rsh - {1'b0, den};
    wire        ge  = ~dif[RW];            
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rem <= 0; nsh <= 0; q <= 0; cnt <= 0;
            running <= 1'b0; ovf <= 1'b0; quot <= 0; done <= 1'b0;
        end else begin
            done <= 1'b0;
            if (start) begin
                rem     <= 0;
                nsh     <= num;
                q       <= 0;
                ovf     <= 1'b0;
                cnt     <= 0;
                running <= 1'b1;
            end else if (running) begin
                rem <= ge ? dif[RW-1:0] : rsh[RW-1:0];
                nsh <= {nsh[NUM_W-2:0], 1'b0};
                ovf <= ovf | q[DIV_CYC-1];
                if (cnt == NUM_W-1) begin
                    running <= 1'b0;
                    // Òç³ö
                    quot <= (ovf | q[DIV_CYC-1]) ? {DIV_CYC{1'b1}}
                                                 : {q[DIV_CYC-2:0], ge};
                    done <= 1'b1;
                end else begin
                    q   <= {q[DIV_CYC-2:0], ge};
                    cnt <= cnt + 1'b1;
                end
            end
        end
    end
endmodule
