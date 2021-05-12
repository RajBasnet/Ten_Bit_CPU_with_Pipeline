`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/04/2020 10:44:24 PM
// Design Name: 
// Module Name: DM_test
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


module DM_test();
    reg clk, write_en, mem_read;
    reg [3:0] address; reg [9:0] write_data;
    wire [9:0] read_data;
    
    DataMemory DM_test(.clk(clk), .write_en(write_en), .mem_read(mem_read), .address(address),
                        .write_data(write_data), .read_data(read_data));
    initial begin
        clk = 1;
        write_en = 0;
        mem_read = 0;
        write_data = 10'bxxxxxxxxxx;
        address = 4'b0000;
        
        #100
        write_en = 1;
        mem_read = 1;
        write_data = 10'b0010100101;
        
        #100
        write_en = 1;
        mem_read = 1;
        write_data = 10'b0011100011;
    end
    
    always begin
        #50 clk = ~clk;
    end
    
    always begin
        #100 address = address + 4'b0001;
    end   
endmodule
