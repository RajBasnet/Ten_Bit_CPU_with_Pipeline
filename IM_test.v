`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/04/2020 09:45:34 PM
// Design Name: 
// Module Name: IM_test
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


module IM_test();
    reg clock; 
    reg [9:0] address;
    wire [9:0] instruction;
    
    InstructionMemory IM_test(.clock(clock), .address(address), .instruction(instruction));
    
    initial begin
        #100
        clock = 1;
        address = 10'b0000000000;
    end
    
    always begin
        #50 clock = ~clock;
    end
    
    always begin
        #100 address <= address + 10'b0000000001;
    end
    
endmodule
