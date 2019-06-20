`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Portland State University
// Engineer: Andrew Capatina / Ryan Bornhorst
// 
// Create Date: 05/13/2019 06:05:17 PM
// Design Name: 
// Module Name: rpm_detection
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
//          Purpose of this file is to monitor the encoder feedback from 
//      the DC motor. This module will determine the rpm and return it 
//      to the microblaze application.  
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module rpm_detection(
    input               clk, reset, // 100 MHz clock and reset inputs
    input               sensA,      // pmod HB3 encoder feedback
    output reg [31:0]   rpm_out     // Output rpm.
    );
    parameter   sample_time    = 32'h5F5E100;    // Total sample time for collecting rpm. (1 second) (100,000,000).
    reg[31:0]   time_reference; // Variable used as time reference for period. 
    reg[31:0]   enc_ticks;      // Variable that holds number of encoder ticks per revolution. 
    reg         sensA_d;        // Clocked input of encoder.
    reg         interlock;
    
       
    initial 
    begin
        time_reference  = 0;    // Initializing variables.
        enc_ticks       = 0;
        rpm_out         = 0;
        interlock       = 1;
    end
       
    always@(posedge clk)   // Invoking sequential logic. Sensitivity list: clock and reset.
    begin    
        if(~reset)  // Is there a system reset 
            begin
                time_reference  <= 0;   // Resetting all variables.     
                enc_ticks       <= 0;
                rpm_out         <= 0;
                interlock       <= 1;
            end
        else        // if there's no system reset. 
            begin
            sensA_d = sensA;
            if(time_reference >= sample_time)         // Wait until one second has passed using the time reference counter. 
                begin
                    rpm_out <= enc_ticks[31:0];    // Set RPM output and multiply by 60 for RPM.
                    time_reference <= 0;    // Reset time reference variable.      
                    enc_ticks <= 0;            
                end
            else
                begin
                    time_reference <= time_reference + 1;   // Add to time reference counter.
                    if(sensA_d == 1 && interlock == 0)    // Waiting for positive edge from encoder.
                        begin
                            enc_ticks <= enc_ticks + 1; // Increase encoder tick count. 
                            interlock <= 1;
                        end
                    else if(sensA_d == 0 && interlock == 1)    // Waiting for negative edge from encoder.
                        begin
                            interlock <= 0;
                        
                        end
                end
            end
    end
endmodule
