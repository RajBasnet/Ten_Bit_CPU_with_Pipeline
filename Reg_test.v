`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/05/2020 08:32:04 AM
// Design Name: 
// Module Name: Reg_test
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


module Reg_test();
    reg clk; reg reset; reg en;
    reg [9:0] data_in;
    wire [9:0] data_out;
    
    reg10Bit Reg_test(.clk(clk), .reset(reset), .en(en), .data_in(data_in), .data_out(data_out));
    
    initial begin
        clk = 1;
        reset = 1;
        en = 0;
        
        #100
        reset = 0;
        en = 0;
        data_in = 10'b0001011111; 
        
        #100
        en = 1;
        data_in = 10'b0001011111; 
        
        #100
        data_in = 10'b1111010000;
       
        #100
        data_in = 10'b1100000011; 
    end
    
    always begin
        #50 clk = ~clk;
    end
endmodule
