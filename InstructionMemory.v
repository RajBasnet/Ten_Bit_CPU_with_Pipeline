`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/04/2020 09:43:47 PM
// Design Name: 
// Module Name: InstructionMemory
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


module InstructionMemory(input clock, input [9:0] address, output reg [9:0] instruction);
    reg [9:0] memory[32:0];
    reg [1:0] program;
    integer i;
    integer mem0 = 0, mem8 = 8, distance = 30;
    
    initial begin
        program = 2'b10;
        case(program)
        2'b11:
            begin
            memory[0] <= 10'b0000100000;
            memory[1] <= 10'b0000110001;
            memory[2] <= 10'b0101010000;
            memory[3] <= 10'b0111000010;
            memory[4] <= 10'b0100111111;
            memory[5] <= 10'b1000110101;
            memory[6] <= 10'b1100001100;
            memory[7] <= 10'b0101001100;
            memory[8] <= 10'b0011000010;
            memory[9] <= 10'b111xxxxxxx;
            end
        2'b10:
            begin
            i = 4; 
            memory[0] <= 10'b0000110000;
            memory[1] <= 10'b1000110000;
            memory[2] <= 10'b1010011101; //beq
            memory[3] <= 10'b0010111000;
            
            while(i < 32)
                begin
                mem0 = mem0 + 1;
                mem8 = mem8 + 1;
                distance = distance - 4;
                memory[i] = {3'b000, 3'b011, mem0[3:0]};
                    i = i + 1;
                memory[i] = 10'b1000110000;
                    i = i + 1;
                memory[i] = {3'b101, distance[7:0]};
                    i = i + 1; 
                memory[i] = {3'b001, 3'b011, mem8[3:0]};
                    i = i + 1;
                end
            
            memory[32] <= 10'b111xxxxxxx;
            end
        endcase
    end
    always @(address)
    begin
        instruction <= memory[address];
    end
endmodule
