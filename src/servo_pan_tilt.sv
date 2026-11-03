// 云台舵机驱动
`timescale 1ns/1ps
`include "project_global_param.vh"
module servo_pan_tilt #(
    parameter integer PWM_PERIOD_CYC  = 1_000_000,  
    parameter integer PULSE_MIN_CYC   =    25_000, 
    parameter integer PULSE_MAX_CYC   =   125_000, 
    parameter integer TILT_MIN_01DEG  =       100,  
    parameter integer PAN_TRIM_01DEG  =         0,  
    parameter integer TILT_TRIM_01DEG =         0  
)(
    input  wire               clk_50m,
    input  wire               rst_n,
    input  wire signed [15:0] az_01deg,    
    input  wire signed [15:0] el_01deg,    
    input  wire               update,      
    output reg                pan_pwm,     
    output reg                tilt_pwm     
);
    localparam integer SPAN_CYC = PULSE_MAX_CYC - PULSE_MIN_CYC;
    
    localparam integer PAN_INIT_01DEG  = 900;
    localparam integer TILT_INIT_01DEG = 900;
    localparam integer PAN_INIT_CYC  = PULSE_MIN_CYC + (PAN_INIT_01DEG  * SPAN_CYC) / 1800;
    localparam integer TILT_INIT_CYC = PULSE_MIN_CYC + (TILT_INIT_01DEG * SPAN_CYC) / 1800;
    integer pan_raw, tilt_raw;
    integer pan_clamp, tilt_clamp;
    always @(*) begin
        if (az_01deg >= 16'sd0) begin
            pan_raw  = az_01deg;
            tilt_raw = el_01deg;
        end else begin
            pan_raw  = az_01deg + 32'sd1800;
            tilt_raw = 32'sd1800 - el_01deg;
        end
        // 机械安装微调
        pan_raw  = pan_raw  + PAN_TRIM_01DEG;
        tilt_raw = tilt_raw + TILT_TRIM_01DEG;
        
        if (pan_raw < 0)
            pan_clamp = 0;
        else if (pan_raw > 1800)
            pan_clamp = 1800;
        else
            pan_clamp = pan_raw;
        // tilt TILT_MIN..1800（10°~180°）
        if (tilt_raw < TILT_MIN_01DEG)
            tilt_clamp = TILT_MIN_01DEG;
        else if (tilt_raw > 1800)
            tilt_clamp = 1800;
        else
            tilt_clamp = tilt_raw;
    end
    reg [11:0] pan_ang_r;      
    reg [11:0] tilt_ang_r;     
    reg        calc_d;        
    reg [19:0] pan_hi_tgt;   
    reg [19:0] tilt_hi_tgt;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            pan_ang_r   <= PAN_INIT_01DEG;
            tilt_ang_r  <= TILT_INIT_01DEG;
            calc_d      <= 1'b0;
            pan_hi_tgt  <= PAN_INIT_CYC;
            tilt_hi_tgt <= TILT_INIT_CYC;
        end else begin
            calc_d <= update;
            if (update) begin
                pan_ang_r  <= pan_clamp;
                tilt_ang_r <= tilt_clamp;
            end
            if (calc_d) begin
                pan_hi_tgt  <= PULSE_MIN_CYC + (pan_ang_r  * SPAN_CYC) / 1800;
                tilt_hi_tgt <= PULSE_MIN_CYC + (tilt_ang_r * SPAN_CYC) / 1800;
            end
        end
    end
    reg [19:0] period_cnt;
    reg [19:0] pan_hi_act;
    reg [19:0] tilt_hi_act;
    always @(posedge clk_50m or negedge rst_n) begin
        if (!rst_n) begin
            period_cnt  <= 20'd0;
            pan_hi_act  <= PAN_INIT_CYC;
            tilt_hi_act <= TILT_INIT_CYC;
            pan_pwm     <= 1'b0;
            tilt_pwm    <= 1'b0;
        end else begin
            if (period_cnt == PWM_PERIOD_CYC - 1) begin
                period_cnt  <= 20'd0;
                pan_hi_act  <= pan_hi_tgt;
                tilt_hi_act <= tilt_hi_tgt;
            end else begin
                period_cnt  <= period_cnt + 20'd1;
            end
            pan_pwm  <= (period_cnt < pan_hi_act);
            tilt_pwm <= (period_cnt < tilt_hi_act);
        end
    end
endmodule
