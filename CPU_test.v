`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/03/2020 12:43:52 AM
// Design Name: 
// Module Name: CPU_test
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module CPU_test();
    reg clock; reg reset;
    wire done; wire [9:0] result;
    
    CPU_10bits_pipelined CPUtest(.clock(clock), .reset(reset), .done(done), .result(result));
    
    initial 
    begin
        clock = 1;
        reset = 1;
        
        #100
        reset = 0;
    end
    
    always begin
        #50 clock = ~clock;
    end
       
endmodule
