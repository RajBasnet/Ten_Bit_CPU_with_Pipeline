`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/04/2020 07:00:07 PM
// Design Name: 
// Module Name: RegFile_test
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


module RegFile_test();
    reg clk, reset, write_en;
    reg [2:0] reg_write_dest, read_addr_1, read_addr_2;
    reg [9:0] write_data;
    wire [9:0] read_data_1, read_data_2;
    
    RegFile RegFile_test(.clk(clk), .reset(reset), .write_en(write_en), .reg_write_dest(reg_write_dest), .write_data(write_data),
                            .read_addr_1(read_addr_1), .read_addr_2(read_addr_2), .read_data_1(read_data_1), .read_data_2(read_data_2));
    
    initial begin
        clk = 1;
        reset = 1;
        write_en = 0;
        
        #100
        reset = 0;
        write_en = 1;
        reg_write_dest = 3'b000;
        write_data = 10'b0010100101;
        read_addr_1 = 3'b000;
        read_addr_2 = 3'b000;
        
        #100
        write_en = 1;
        reg_write_dest = 3'b001;
        write_data = 10'b0011000011;
        read_addr_1 = 3'b111;
        read_addr_2 = 3'b001;
        
        #100
        write_en = 1;
        reg_write_dest = 3'b010;
        write_data = 10'b1111000011;
        read_addr_1 = 3'b110;
        read_addr_2 = 3'b010;
        
        #100
        write_en = 1;
        reg_write_dest = 3'b011;
        write_data = 10'b0000000011;
        read_addr_1 = 3'b011;
        read_addr_2 = 3'b100;
        
        #100
        write_en = 1;
        reg_write_dest = 3'b100;
        write_data = 10'b0011000011;
        read_addr_1 = 3'b100;
        read_addr_2 = 3'b011;
        
        #100
        write_en = 1;
        reg_write_dest = 3'b101;
        write_data = 10'b0011000011;
        read_addr_1 = 3'b011;
        read_addr_2 = 3'b101;
        
        #100
        write_en = 1;
        reg_write_dest = 3'b110;
        write_data = 10'b0011000011;
        read_addr_1 = 3'b110;
        read_addr_2 = 3'b110;
        
        #100
        write_en = 1;
        reg_write_dest = 3'b111;
        write_data = 10'b0011000011;
        read_addr_1 = 3'b110;
        read_addr_2 = 3'b111;
                                
        #100
        write_en = 0;
        reset = 0;  
    end
    
    always begin
        #50 clk =~clk;
    end
endmodule
